#!/usr/bin/env python3
"""daq_term: terminal monitor for daq-detector.

  USB  (native USB-Serial-JTAG): binary stream of DAQ1 packets -> parsed, stats, levels, optional WAV.
  UART (CP2102N console):        log lines + status line, keys forwarded to the ESP console.

Usage:
  pip install pyserial numpy rich
  python tools/daq_term.py --uart COM3 --usb COM4

Keys:
  s  start/stop USB stream (host command S / X)
  w  start/stop WAV recording on the PC (6 ch, 16 bit, 48 kHz; lost blocks filled with zeros)
  r  SD recording on the ESP     i  identify (LED)     h  ESP help (output in the log)
  c  clear counters              q  quit

Packet (little-endian): magic u32 0x31514144 "DAQ1", seq u32, first_frame u64,
sample_rate u32, channels u16, frames u16, then i16[frames][channels].
"""

import argparse
import collections
import datetime
import os
import struct
import sys
import threading
import time
import wave

import numpy as np
import serial

MAGIC = b"DAQ1"                      # 0x31514144 little-endian
HDR = struct.Struct("<IIQIHH")       # 24 bytes
MAX_FRAMES = 4096
MAX_CHANNELS = 16


def open_port(name, baud):
    """Open without toggling DTR/RTS: on the DevKitC both ports would reset the ESP32-S3."""
    port = serial.serial_for_url(name, do_not_open=True, baudrate=baud, timeout=0.05)
    try:
        port.dtr = False
        port.rts = False
    except (AttributeError, ValueError, serial.SerialException):
        pass
    port.open()
    return port


# --------------------------------------------------------------------------- USB stream

class UsbStream:
    def __init__(self, port):
        self.port = port
        self.lock = threading.Lock()
        self.running = True
        self.streaming = False
        self.wav = None
        self.wav_name = None
        self.wav_frames = 0
        self.error = None
        self.reset_counters()

    def reset_counters(self):
        with getattr(self, "lock", threading.Lock()):
            self.t0 = time.monotonic()
            self.bytes_total = 0
            self.packets = 0
            self.lost = 0            # missing seq numbers
            self.resyncs = 0         # garbage skipped before a magic word
            self.bad_headers = 0
            self.frame_jumps = 0     # first_frame not continuous although seq is
            self.last_seq = None
            self.last_first_frame = None
            self.last_frames = 0
            self.fmt = None          # (sample_rate, channels, frames)
            self.sumsq = None
            self.peak = None
            self.nsamp = 0
            self.rate_hist = collections.deque(maxlen=20)   # (t, bytes_total)

    # host commands
    def set_streaming(self, on):
        self.port.write(b"S" if on else b"X")
        self.streaming = on

    # PC-side WAV
    def toggle_wav(self):
        with self.lock:
            if self.wav:
                self.wav.close()
                self.wav = None
                return f"WAV saved: {self.wav_name} ({self.wav_frames / 48000:.1f} s)"
            if not self.fmt:
                return "WAV: no packets yet, start the stream first (s)"
            rate, ch, _ = self.fmt
            os.makedirs("recordings", exist_ok=True)
            self.wav_name = os.path.join(
                "recordings", datetime.datetime.now().strftime("usb_%Y%m%d_%H%M%S.wav"))
            w = wave.open(self.wav_name, "wb")
            w.setnchannels(ch)
            w.setsampwidth(2)
            w.setframerate(rate)
            self.wav = w
            self.wav_frames = 0
            return f"WAV recording: {self.wav_name}"

    def close(self):
        self.running = False
        with self.lock:
            if self.wav:
                self.wav.close()
                self.wav = None

    # reader thread
    def run(self):
        buf = bytearray()
        try:
            while self.running:
                data = self.port.read(max(1, min(self.port.in_waiting, 65536)))
                if not data:
                    continue
                buf += data
                with self.lock:
                    self.bytes_total += len(data)
                    self._parse(buf)
        except (serial.SerialException, OSError) as e:
            self.error = f"USB: {e}"
        except Exception as e:          # keep the UI alive, show the error
            self.error = f"USB parser: {e!r}"

    def _parse(self, buf):
        pos = 0
        n = len(buf)
        while True:
            idx = buf.find(MAGIC, pos)
            if idx < 0:
                # keep the last 3 bytes, they may be the start of a magic word
                keep = max(pos, n - 3)
                if keep > pos:
                    self.resyncs += 1 if n - pos > 3 else 0
                del buf[:keep]
                return
            if idx != pos:
                self.resyncs += 1
            if n - idx < HDR.size:
                del buf[:idx]
                return
            _, seq, first_frame, rate, ch, frames = HDR.unpack_from(buf, idx)
            if not (8000 <= rate <= 192000 and 1 <= ch <= MAX_CHANNELS and 1 <= frames <= MAX_FRAMES):
                self.bad_headers += 1
                pos = idx + 1
                continue
            plen = frames * ch * 2
            end = idx + HDR.size + plen
            if n < end + 4:                  # wait for the next magic as well
                del buf[:idx]
                return
            if buf[end:end + 4] != MAGIC:
                # short write on the ESP side: the packet was cut and the next one starts inside it
                j = buf.find(MAGIC, idx + 4, end)
                if j >= 0:
                    self.bad_headers += 1
                    pos = j
                    continue
            self._packet(seq, first_frame, rate, ch, frames, bytes(buf[idx + HDR.size:end]))
            pos = end

    def _packet(self, seq, first_frame, rate, ch, frames, payload):
        if self.fmt != (rate, ch, frames) or self.sumsq is None:
            self.fmt = (rate, ch, frames)
            self.sumsq = np.zeros(ch)
            self.peak = np.zeros(ch)
            self.nsamp = 0
        gap = 0
        if self.last_seq is not None:
            gap = (seq - self.last_seq - 1) & 0xFFFFFFFF
            if gap > 100000:           # stream restarted / ESP reset
                gap = 0
            self.lost += gap
            exp = self.last_first_frame + self.last_frames * (gap + 1)
            if first_frame != exp:
                self.frame_jumps += 1
        self.last_seq = seq
        self.last_first_frame = first_frame
        self.last_frames = frames
        self.packets += 1

        x = np.frombuffer(payload, dtype="<i2").reshape(frames, ch).astype(np.float32)
        self.sumsq += np.einsum("ij,ij->j", x, x)
        self.peak = np.maximum(self.peak, np.abs(x).max(axis=0))
        self.nsamp += frames

        if self.wav:
            if gap:
                self.wav.writeframes(bytes(gap * frames * ch * 2))
                self.wav_frames += gap * frames
            self.wav.writeframes(payload)
            self.wav_frames += frames

    def snapshot(self):
        """Stats + levels since the previous snapshot."""
        with self.lock:
            now = time.monotonic()
            self.rate_hist.append((now, self.bytes_total))
            t_a, b_a = self.rate_hist[0]
            rate = (self.bytes_total - b_a) / (now - t_a) if now > t_a else 0.0
            if self.sumsq is not None and self.nsamp:
                rms = np.sqrt(self.sumsq / self.nsamp)
                dbfs = 20 * np.log10(np.maximum(rms, 1e-3) / 32768.0)
                pk = 20 * np.log10(np.maximum(self.peak, 1e-3) / 32768.0)
                clip = self.peak >= 32767
                self.sumsq[:] = 0
                self.peak[:] = 0
                self.nsamp = 0
            else:
                dbfs = pk = clip = None
            return dict(
                rate=rate, bytes=self.bytes_total, packets=self.packets, lost=self.lost,
                resyncs=self.resyncs, bad=self.bad_headers, jumps=self.frame_jumps,
                seq=self.last_seq, first_frame=self.last_first_frame, fmt=self.fmt,
                dbfs=dbfs, peak=pk, clip=clip, uptime=now - self.t0,
                wav=self.wav_name if self.wav else None,
                wav_s=self.wav_frames / (self.fmt[0] if self.fmt else 48000),
                streaming=self.streaming, error=self.error)


# --------------------------------------------------------------------------- UART console

class UartConsole:
    def __init__(self, port, max_lines=200):
        self.port = port
        self.lines = collections.deque(maxlen=max_lines)
        self.status = ""
        self.running = True
        self.error = None
        self.lock = threading.Lock()

    def send(self, ch):
        self.port.write(ch.encode())

    def run(self):
        partial = b""
        try:
            while self.running:
                data = self.port.read(max(1, min(self.port.in_waiting, 4096)))
                if not data:
                    continue
                partial += data
                *done, partial = partial.replace(b"\r", b"\n").split(b"\n")
                with self.lock:
                    for raw in done:
                        line = strip_ansi(raw.decode("utf-8", "replace")).rstrip()
                        if not line:
                            continue
                        if line.startswith("t=") and "dBFS" in line:
                            self.status = line
                        else:
                            self.lines.append(line)
        except (serial.SerialException, OSError) as e:
            self.error = f"UART: {e}"

    def snapshot(self, n):
        with self.lock:
            return self.status, list(self.lines)[-n:]


def strip_ansi(s):
    out, i = [], 0
    while i < len(s):
        if s[i] == "\x1b":
            j = i + 1
            if j < len(s) and s[j] == "[":
                j += 1
                while j < len(s) and not ("@" <= s[j] <= "~"):
                    j += 1
            i = j + 1
            continue
        out.append(s[i])
        i += 1
    return "".join(out)


# --------------------------------------------------------------------------- keyboard

class Keys:
    def __init__(self):
        self.win = os.name == "nt"
        if not self.win and sys.stdin.isatty():
            import termios
            import tty
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        else:
            self.old = None

    def get(self):
        if self.win:
            import msvcrt
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                if ch in ("\x00", "\xe0"):   # arrows / function keys
                    msvcrt.getwch()
                    return None
                return ch
            return None
        import select
        if self.old is not None and select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def restore(self):
        if self.old is not None:
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


# --------------------------------------------------------------------------- UI

def fmt_bytes(n):
    for unit in ("B", "kB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return f"{n:.1f} {unit}" if unit != "B" else f"{n} B"
        n /= 1024


def level_bar(db, width=30, floor=-90.0):
    if db is None:
        return " " * width
    f = min(1.0, max(0.0, (db - floor) / -floor))
    n = int(round(f * width))
    return "█" * n + "·" * (width - n)


def render(usb, uart, msg, args):
    from rich.console import Group
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text

    s = usb.snapshot() if usb else None
    parts = []

    if s:
        expected = 0
        if s["fmt"]:
            rate, ch, frames = s["fmt"]
            expected = rate * ch * 2 * (1 + 24 / (frames * ch * 2))
        t = Table.grid(padding=(0, 2))
        t.add_column(style="bold")
        t.add_column()
        t.add_column(style="bold")
        t.add_column()
        stream_txt = Text("ON", style="bold green") if s["streaming"] else Text("OFF", style="bold red")
        ok = expected and s["rate"] > 0.97 * expected
        rate_style = "green" if ok else ("yellow" if s["rate"] > 0 else "red")
        t.add_row("stream", stream_txt, "rate",
                  Text(f"{s['rate'] / 1000:7.1f} kB/s" + (f"  (exp {expected / 1000:.0f})" if expected else ""),
                       style=rate_style))
        t.add_row("packets", f"{s['packets']}", "total", fmt_bytes(s["bytes"]))
        t.add_row("lost", Text(str(s["lost"]), style="red bold" if s["lost"] else "green"),
                  "resync", Text(str(s["resyncs"]), style="yellow" if s["resyncs"] else "green"))
        t.add_row("bad hdr", Text(str(s["bad"]), style="yellow" if s["bad"] else "green"),
                  "frame jumps", Text(str(s["jumps"]), style="yellow" if s["jumps"] else "green"))
        t.add_row("seq", f"{s['seq']}", "first_frame",
                  f"{s['first_frame']}" + (f"  ({s['first_frame'] / s['fmt'][0]:.1f} s)" if s["fmt"] else ""))
        if s["fmt"]:
            t.add_row("format", f"{s['fmt'][0]} Hz × {s['fmt'][1]} ch × {s['fmt'][2]} fr", "", "")
        if s["wav"]:
            t.add_row("WAV", Text(f"● {s['wav']}  {s['wav_s']:.1f} s", style="bold red"), "", "")

        lv = Table(box=None, padding=(0, 1), show_header=True, header_style="dim")
        lv.add_column("mic")
        lv.add_column("RMS dBFS", justify="right")
        lv.add_column("")
        lv.add_column("peak", justify="right")
        if s["dbfs"] is not None:
            for i, (d, p, c) in enumerate(zip(s["dbfs"], s["peak"], s["clip"])):
                style = "red" if c else ("green" if d > -70 else "dim")
                lv.add_row(f"M{i + 1}", f"{d:6.1f}", Text(level_bar(d), style=style),
                           Text(f"{p:6.1f}" + (" CLIP" if c else ""), style="red bold" if c else ""))
        else:
            lv.add_row("—", "", Text("no data (press s)", style="dim"), "")
        title = f"USB {args.usb}"
        if s["error"]:
            title += f"  [red]{s['error']}[/red]"
        parts.append(Panel(Group(t, Text(""), lv), title=title, border_style="cyan"))

    if uart:
        status, lines = uart.snapshot(args.log_lines)
        st = Text(status or "(waiting for the status line…)", style="bold" if status else "dim")
        log = Text("\n".join(lines[-args.log_lines:]) or "", style="")
        title = f"UART {args.uart}"
        if uart.error:
            title += f"  [red]{uart.error}[/red]"
        parts.append(Panel(st, title=title + " · status", border_style="magenta"))
        parts.append(Panel(log, title="log", border_style="dim", height=args.log_lines + 2))

    keys = "[b]s[/b] stream  [b]w[/b] WAV  [b]r[/b] SD rec  [b]i[/b] identify  [b]h[/b] help  [b]c[/b] clear  [b]q[/b] quit"
    parts.append(Text.from_markup(keys + (f"   │ {msg}" if msg else "")))
    return Group(*parts)


def run_plain(usb, uart, args):
    """No UI, one line per second (logging / testing)."""
    t_end = time.monotonic() + args.duration if args.duration else None
    while t_end is None or time.monotonic() < t_end:
        time.sleep(1)
        if usb:
            s = usb.snapshot()
            lv = " ".join(f"{d:6.1f}" for d in s["dbfs"]) if s["dbfs"] is not None else "-"
            print(f"usb {s['rate'] / 1000:7.1f} kB/s pk={s['packets']} lost={s['lost']} "
                  f"resync={s['resyncs']} bad={s['bad']} jumps={s['jumps']} seq={s['seq']} dBFS {lv}",
                  flush=True)
        if uart and uart.status:
            print("uart", uart.status, flush=True)


def main():
    ap = argparse.ArgumentParser(description="daq-detector terminal monitor (USB stream + UART console)")
    ap.add_argument("--usb", default="COM4", help="native USB port (binary stream), '' = off")
    ap.add_argument("--uart", default="COM3", help="UART console port (CP2102N), '' = off")
    ap.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    ap.add_argument("--no-autostart", action="store_true", help="do not send S on start")
    ap.add_argument("--log-lines", type=int, default=10)
    ap.add_argument("--plain", action="store_true", help="no UI, print stats once per second")
    ap.add_argument("--duration", type=float, default=0, help="with --plain: stop after N s")
    args = ap.parse_args()

    usb = uart = None
    threads = []
    try:
        if args.usb:
            usb = UsbStream(open_port(args.usb, 115200))
            threads.append(threading.Thread(target=usb.run, daemon=True))
        if args.uart:
            uart = UartConsole(open_port(args.uart, args.baud))
            threads.append(threading.Thread(target=uart.run, daemon=True))
    except serial.SerialException as e:
        sys.exit(f"Cannot open port: {e}")
    for th in threads:
        th.start()
    if usb and not args.no_autostart:
        usb.set_streaming(True)

    if args.plain:
        try:
            run_plain(usb, uart, args)
        except KeyboardInterrupt:
            pass
    else:
        from rich.live import Live
        keys = Keys()
        msg = ""
        try:
            with Live(render(usb, uart, msg, args), refresh_per_second=8, screen=True) as live:
                while True:
                    k = keys.get()
                    if k:
                        k = k.lower()
                        if k == "q":
                            break
                        elif k == "s" and usb:
                            usb.set_streaming(not usb.streaming)
                            msg = f"stream {'ON' if usb.streaming else 'OFF'}"
                        elif k == "w" and usb:
                            msg = usb.toggle_wav()
                        elif k == "c" and usb:
                            usb.reset_counters()
                            msg = "counters cleared"
                        elif k in "rih" and uart:
                            uart.send(k)
                            msg = {"r": "SD recording toggled", "i": "identify", "h": "help → log"}[k]
                    live.update(render(usb, uart, msg, args))
                    time.sleep(0.12)
        except KeyboardInterrupt:
            pass
        finally:
            keys.restore()

    if usb:
        try:
            usb.set_streaming(False)
        except serial.SerialException:
            pass
        usb.close()
        if usb.wav_name:
            print(f"WAV: {usb.wav_name}")
    if uart:
        uart.running = False


if __name__ == "__main__":
    main()
