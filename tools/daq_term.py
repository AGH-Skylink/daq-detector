#!/usr/bin/env python3
"""daq_term: terminal monitor for daq-detector.

  USB  (native USB-Serial-JTAG): binary stream of DAQ1 packets -> parsed, stats, levels, optional WAV;
       DAQT text packets = detection log (one CSV line per DSP frame) -> last result, optional CSV file.
  UART (CP2102N console):        log lines + status line, keys forwarded to the ESP console.
  PC localization (tools/pc_localize.py): algorithms A, B and C at the same time on the streamed audio,
       with the Python reference of the firmware (daq-sim), shown as three radars.

Usage:
  pip install pyserial numpy rich
  python tools/daq_term.py --uart COM3 --usb COM4
  python tools/daq_term.py --uart COM3 --usb COM4 --sim ../daq-simulator --track AZEL

Keys:
  s  start/stop USB stream (host command S / X)
  w  start/stop WAV recording on the PC: one mono file per mic, recordings/usb_<time>/M1.wav .. M6.wav
     (16 bit, 48 kHz; lost blocks filled with zeros)
  l  start/stop the detection CSV on the PC (columns of the SD card CSV + pc_time_s)
  p  PC localization on/off
  g  session log on/off: recordings/pc_<time>/ with the results of all methods, TDOA, groups and M1..M6.wav
  r  SD recording on the ESP     i  identify (LED)     h  ESP help (output in the log)
  c  clear counters              q  quit

Packet (little-endian): magic u32 0x31514144 "DAQ1", seq u32, first_frame u64,
sample_rate u32, channels u16, frames u16, then i16[frames][channels].
Text packet: magic u32 0x54514144 "DAQT", length u16, reserved u16, then `length` bytes of ASCII: the CSV
header block ("# daq-detector ..." + column line) once per stream start, then one CSV line per frame.
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
TMAGIC = b"DAQT"                     # 0x54514144: text packet (detection CSV)
HDR = struct.Struct("<IIQIHH")       # 24 bytes
THDR = struct.Struct("<IHH")         # 8 bytes
MAX_FRAMES = 4096
MAX_CHANNELS = 16


class MicCheck:
    """Live per-mic health from the audio stream, judged over ~1 s windows.

    NOISE  white noise over the whole band (spectral flatness 300 Hz-20 kHz > FLAT_NOISE) and loud
           (> NOISE_MIN_DBFS): what an empty or floating PDM slot sounds like (the ESP decodes random bits:
           flat spectrum up to 24 kHz, usually with a large DC offset). Real mics, also in a quiet room, have
           a far from flat spectrum (0.0-0.1 on the recordings of this array, 0.98 for such noise).
    DEAD   no signal (all samples ~constant)       CLIP  samples at full scale
    DC     DC offset above DC_MAX (counts)          OK    none of these
    """
    NFFT = 2048
    EVERY = 6                 # one spectrum every 6 packets (40 ms)
    WINDOW = 25               # spectra per verdict (~1 s)
    FLAT_NOISE = 0.5
    NOISE_MIN_DBFS = -45.0
    DC_MAX = 4000.0
    DEAD_DBFS = -100.0

    def __init__(self, ch, rate):
        self.ch, self.rate = ch, rate
        self.buf = np.zeros((self.NFFT, ch))
        self.win = np.hanning(self.NFFT)[:, None]
        f = np.fft.rfftfreq(self.NFFT, 1 / rate)
        self.band = (f >= 300) & (f <= min(20000, rate / 2 - 1000))
        self.reset()
        self.status = ["…"] * ch
        self.flatness = [None] * ch
        self.dc = [None] * ch

    def reset(self):
        self.spec = np.zeros((self.NFFT // 2 + 1, self.ch))
        self.nspec = 0
        self.packets = 0
        self.s1 = np.zeros(self.ch)
        self.s2 = np.zeros(self.ch)
        self.n = 0
        self.peak = np.zeros(self.ch)

    def add(self, x):
        """x: (frames, ch) float32 samples of one packet."""
        k = x.shape[0]
        self.buf = np.concatenate([self.buf[k:], x]) if k < self.NFFT else x[-self.NFFT:]
        self.s1 += x.sum(axis=0)
        self.s2 += np.einsum("ij,ij->j", x, x)
        self.n += k
        self.peak = np.maximum(self.peak, np.abs(x).max(axis=0))
        self.packets += 1
        if self.packets % self.EVERY == 0:
            seg = self.buf - self.buf.mean(axis=0)
            self.spec += np.abs(np.fft.rfft(seg * self.win, axis=0)) ** 2
            self.nspec += 1
            if self.nspec >= self.WINDOW:
                self._verdict()
                self.reset()

    def _verdict(self):
        p = self.spec[self.band] + 1e-12
        flat = np.exp(np.mean(np.log(p), axis=0)) / np.mean(p, axis=0)
        mean = self.s1 / self.n
        ac = np.sqrt(np.maximum(self.s2 / self.n - mean ** 2, 1e-6))
        ac_db = 20 * np.log10(ac / 32768.0)
        st = []
        for c in range(self.ch):
            if ac_db[c] < self.DEAD_DBFS:
                st.append("DEAD")
            elif flat[c] > self.FLAT_NOISE and ac_db[c] > self.NOISE_MIN_DBFS:
                st.append("NOISE")
            elif self.peak[c] >= 32767:
                st.append("CLIP")
            elif abs(mean[c]) > self.DC_MAX:
                st.append("DC")
            else:
                st.append("OK")
        self.status, self.flatness, self.dc = st, [float(v) for v in flat], [float(v) for v in mean]


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
        self.wav = None              # list of mono wave writers, one per mic (M1..M6), or None
        self.wav_name = None         # folder of the current recording
        self.wav_frames = 0
        self.csv = None              # open detection CSV file
        self.csv_name = None
        self.csv_lines = 0
        self.csv_pending = False     # --csv: open the file as soon as the header arrives
        self.det_columns = None      # column names from the header block
        self.det_header = None       # header block as received
        self.error = None
        self.loc = None              # PcLocalizer (tools/pc_localize.py) or None
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
            self.sum = None
            self.peak = None
            self.nsamp = 0
            self.check = None
            self.rate_hist = collections.deque(maxlen=20)   # (t, bytes_total)
            self.det_lines = 0           # CSV lines received
            self.det_voice = 0
            self.det_gaps = 0            # missing frames between consecutive lines
            self.det_last_frame = None
            self.det_last = None         # dict of the last line
            self.det_last_src = None     # dict of the last line with a source

    # host commands
    def set_streaming(self, on):
        self.port.write(b"S" if on else b"X")
        self.streaming = on

    # PC-side WAV: one mono file per mic
    def _close_wav(self):
        for w in self.wav:
            w.close()
        self.wav = None

    def toggle_wav(self, folder=None):
        with self.lock:
            if self.wav:
                self._close_wav()
                return f"WAV saved: {self.wav_name}/M1..M{self.fmt[1]}.wav ({self.wav_frames / self.fmt[0]:.1f} s)"
            if not self.fmt:
                return "WAV: no packets yet, start the stream first (s)"
            rate, ch, _ = self.fmt
            self.wav_name = folder or os.path.join(
                "recordings", datetime.datetime.now().strftime("usb_%Y%m%d_%H%M%S"))
            os.makedirs(self.wav_name, exist_ok=True)
            writers = []
            for c in range(ch):
                w = wave.open(os.path.join(self.wav_name, f"M{c + 1}.wav"), "wb")
                w.setnchannels(1)
                w.setsampwidth(2)
                w.setframerate(rate)
                writers.append(w)
            self.wav = writers
            self.wav_frames = 0
            return f"WAV recording: {self.wav_name}/M1..M{ch}.wav"

    # PC localization session: logs of all methods + the audio, one folder
    def toggle_session(self):
        if not self.loc:
            return "session log needs the PC localization (daq-sim)"
        if self.loc.log:
            folder = str(self.loc.log.folder)
            rows = self.loc.stop_log()
            if self.wav:
                self.toggle_wav()
            return f"session saved: {folder} ({rows} log rows + M1..M6.wav)"
        if not self.fmt:
            return "session: no packets yet, start the stream first (s)"
        folder = os.path.join("recordings", datetime.datetime.now().strftime("pc_%Y%m%d_%H%M%S"))
        if self.wav:
            self.toggle_wav()
        self.toggle_wav(folder)
        self.loc.start_log(folder)
        return f"session log: {folder} (results of all methods, TDOA, groups, M1..M6.wav) - g = stop"

    # PC-side detection CSV
    def toggle_csv(self):
        with self.lock:
            if self.csv:
                self.csv.close()
                self.csv = None
                return f"CSV saved: {self.csv_name} ({self.csv_lines} lines)"
            if not self.det_header:
                self.csv_pending = True
                return "CSV: waiting for the header (it comes when the stream starts: s, s)"
            return self._open_csv()

    def _open_csv(self):
        os.makedirs("recordings", exist_ok=True)
        self.csv_name = os.path.join(
            "recordings", datetime.datetime.now().strftime("det_%Y%m%d_%H%M%S.csv"))
        self.csv = open(self.csv_name, "w", encoding="utf-8", newline="")
        lines = self.det_header.rstrip("\n").split("\n")
        lines[-1] += ",pc_time_s"          # column line: + PC receive time (to align with other logs)
        self.csv.write("\n".join(lines) + "\n")
        self.csv_lines = 0
        self.csv_pending = False
        return f"CSV recording: {self.csv_name}"

    def close(self):
        self.running = False
        with self.lock:
            if self.wav:
                self._close_wav()
            if self.csv:
                self.csv.close()
                self.csv = None

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
            idx = find_magic(buf, pos)
            if idx < 0:
                # keep the last 3 bytes, they may be the start of a magic word
                keep = max(pos, n - 3)
                if keep > pos:
                    self.resyncs += 1 if n - pos > 3 else 0
                del buf[:keep]
                return
            if idx != pos:
                self.resyncs += 1
            if buf[idx:idx + 4] == TMAGIC:
                if n - idx < THDR.size:
                    del buf[:idx]
                    return
                _, length, _ = THDR.unpack_from(buf, idx)
                end = idx + THDR.size + length
                if length == 0 or length > 4096:
                    self.bad_headers += 1
                    pos = idx + 1
                    continue
                if n < end:
                    del buf[:idx]
                    return
                self._text(bytes(buf[idx + THDR.size:end]).decode("ascii", "replace"))
                pos = end
                continue
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
            if buf[end:end + 4] not in (MAGIC, TMAGIC):
                # short write on the ESP side: the packet was cut and the next one starts inside it
                j = find_magic(buf, idx + 4, end)
                if j >= 0:
                    self.bad_headers += 1
                    pos = j
                    continue
            self._packet(seq, first_frame, rate, ch, frames, bytes(buf[idx + HDR.size:end]))
            pos = end

    def _text(self, text):
        if text.startswith("#"):
            self.det_header = text
            self.det_columns = text.rstrip("\n").split("\n")[-1].split(",")
            self.det_last_frame = None
            if self.loc:
                self.loc.set_geometry_from_header(text)
            if self.csv_pending and not self.csv:
                self._open_csv()
            return
        if self.det_columns is None:
            return
        line = text.rstrip("\n")
        row = dict(zip(self.det_columns, line.split(",")))
        self.det_lines += 1
        try:
            frame = int(row.get("frame", ""))
            if self.det_last_frame is not None and frame - self.det_last_frame > 1536:
                self.det_gaps += (frame - self.det_last_frame) // 1536 - 1
            self.det_last_frame = frame
        except ValueError:
            pass
        self.det_voice += row.get("vad") == "1"
        self.det_last = row
        if row.get("az_deg"):
            self.det_last_src = row
        if self.csv:
            self.csv.write(f"{line},{time.time():.3f}\n")
            self.csv_lines += 1

    def _packet(self, seq, first_frame, rate, ch, frames, payload):
        if self.fmt != (rate, ch, frames) or self.sumsq is None:
            self.fmt = (rate, ch, frames)
            self.sumsq = np.zeros(ch)
            self.sum = np.zeros(ch)
            self.peak = np.zeros(ch)
            self.nsamp = 0
            self.check = MicCheck(ch, rate)
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

        pcm = np.frombuffer(payload, dtype="<i2").reshape(frames, ch)
        if self.loc:
            self.loc.feed(first_frame, rate, ch, pcm)
        x = pcm.astype(np.float32)
        self.sumsq += np.einsum("ij,ij->j", x, x)
        self.sum += x.sum(axis=0)
        self.check.add(x)
        self.peak = np.maximum(self.peak, np.abs(x).max(axis=0))
        self.nsamp += frames

        if self.wav:
            for c, w in enumerate(self.wav):
                if gap:
                    w.writeframes(bytes(gap * frames * 2))
                w.writeframes(np.ascontiguousarray(pcm[:, c]).astype("<i2").tobytes())
            self.wav_frames += (gap + 1) * frames

    def snapshot(self):
        """Stats + levels since the previous snapshot."""
        with self.lock:
            now = time.monotonic()
            self.rate_hist.append((now, self.bytes_total))
            t_a, b_a = self.rate_hist[0]
            rate = (self.bytes_total - b_a) / (now - t_a) if now > t_a else 0.0
            if self.sumsq is not None and self.nsamp:
                mean = self.sum / self.nsamp
                rms = np.sqrt(np.maximum(self.sumsq / self.nsamp - mean ** 2, 0))   # AC level (DC removed)
                dbfs = 20 * np.log10(np.maximum(rms, 1e-3) / 32768.0)
                pk = 20 * np.log10(np.maximum(self.peak, 1e-3) / 32768.0)
                clip = self.peak >= 32767
                self.sumsq[:] = 0
                self.sum[:] = 0
                self.peak[:] = 0
                self.nsamp = 0
            else:
                dbfs = pk = clip = None
            return dict(
                rate=rate, bytes=self.bytes_total, packets=self.packets, lost=self.lost,
                resyncs=self.resyncs, bad=self.bad_headers, jumps=self.frame_jumps,
                seq=self.last_seq, first_frame=self.last_first_frame, fmt=self.fmt,
                dbfs=dbfs, peak=pk, clip=clip, uptime=now - self.t0,
                check=(list(self.check.status), list(self.check.flatness), list(self.check.dc)) if self.check else None,
                wav=self.wav_name if self.wav else None,
                det_lines=self.det_lines, det_voice=self.det_voice, det_gaps=self.det_gaps,
                det_last=self.det_last, det_src=self.det_last_src,
                csv=self.csv_name if self.csv else None, csv_lines=self.csv_lines,
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


def find_magic(buf, start, end=None):
    """Earliest DAQ1 or DAQT magic in buf[start:end], -1 if none."""
    end = len(buf) if end is None else end
    hits = [i for i in (buf.find(MAGIC, start, end), buf.find(TMAGIC, start, end)) if i >= 0]
    return min(hits) if hits else -1


def det_summary(s):
    """One line: last localization result and tracks from the detection log."""
    r, src = s["det_last"], s["det_src"]
    if not r:
        return "no detection lines (firmware 0.8.0+ sends them while the stream is on)"
    out = f"lines {s['det_lines']} voice {s['det_voice']} gaps {s['det_gaps']} | vad {r.get('vad', '?')}"
    if src:
        out += (f" | {src.get('alg', '?')}/{src.get('kind', '?')} az {src.get('az_deg')} el {src.get('el_deg')}"
                f" d {src.get('dist_m')} conf {src.get('conf')}")
        if src.get("az2_deg"):
            out += f" + az {src.get('az2_deg')}"
    tr = r.get("tracks", "")
    if tr:
        out += f" | tracks {tr}" + (f" t1 az {r.get('t1_az_deg')}" if r.get("t1_id") else "")
    return out


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
            t.add_row("WAV", Text(f"● {s['wav']}/M1..M6.wav  {s['wav_s']:.1f} s", style="bold red"), "", "")
        if s["csv"]:
            t.add_row("CSV", Text(f"● {s['csv']}  {s['csv_lines']} lines", style="bold red"), "", "")
        t.add_row("detect", Text(det_summary(s), style="bold"), "", "")

        lv = Table(box=None, padding=(0, 1), show_header=True, header_style="dim")
        lv.add_column("mic")
        lv.add_column("RMS dBFS", justify="right")
        lv.add_column("")
        lv.add_column("peak", justify="right")
        lv.add_column("check")
        lv.add_column("flat", justify="right")
        lv.add_column("DC", justify="right")
        if s["dbfs"] is not None:
            chk = s["check"] or (["…"] * len(s["dbfs"]), [None] * len(s["dbfs"]), [None] * len(s["dbfs"]))
            for i, (d, p, c) in enumerate(zip(s["dbfs"], s["peak"], s["clip"])):
                st, fl, dc = chk[0][i], chk[1][i], chk[2][i]
                style = "red" if c or st in ("NOISE", "DEAD") else ("green" if d > -70 else "dim")
                st_style = {"OK": "bold green", "NOISE": "bold white on red", "DEAD": "bold white on red",
                            "CLIP": "bold red", "DC": "bold yellow"}.get(st, "dim")
                lv.add_row(f"M{i + 1}", f"{d:6.1f}", Text(level_bar(d), style=style),
                           Text(f"{p:6.1f}" + (" CLIP" if c else ""), style="red bold" if c else ""),
                           Text(f" {st} ", style=st_style), "" if fl is None else f"{fl:.2f}",
                           "" if dc is None else f"{dc:+.0f}")
        else:
            lv.add_row("—", "", Text("no data (press s)", style="dim"), "", "", "", "")
        title = f"USB {args.usb}"
        if s["error"]:
            title += f"  [red]{s['error']}[/red]"
        parts.append(Panel(Group(t, Text(""), lv), title=title, border_style="cyan"))

    if usb and usb.loc:
        parts.append(render_loc(usb.loc.snapshot(), args.show))

    if uart:
        status, lines = uart.snapshot(args.log_lines)
        st = Text(status or "(waiting for the status line…)", style="bold" if status else "dim")
        log = Text("\n".join(lines[-args.log_lines:]) or "", style="")
        title = f"UART {args.uart}"
        if uart.error:
            title += f"  [red]{uart.error}[/red]"
        parts.append(Panel(st, title=title + " · status", border_style="magenta"))
        parts.append(Panel(log, title="log", border_style="dim", height=args.log_lines + 2))

    keys = "[b]s[/b] stream  [b]w[/b] WAV  [b]l[/b] CSV  [b]p[/b] PC loc  [b]g[/b] session log  [b]r[/b] SD rec  [b]i[/b] identify  [b]h[/b] help  [b]c[/b] clear  [b]q[/b] quit"
    parts.append(Text.from_markup(keys + (f"   │ {msg}" if msg else "")))
    return Group(*parts)


def render_loc(snap, show=None):
    """Panel with the PC localization: one radar + numbers per algorithm."""
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
    from pc_localize import fmt_source, radar, track_mark

    grid = Table.grid(padding=(0, 3))
    cols = []
    for a, s in snap["algs"].items():
        if show and a not in show.upper():
            continue
        grid.add_column()
        t = Text()
        t.append(f"{a}  {s['name']}\n", style="bold")
        styles = {"ring": "dim", "label": "dim", "mic": "bold cyan", "centre": "cyan", "history": "yellow",
                  "source": "bold yellow", "track": "bold green", "held": "white", "": ""}
        for row in radar(s["sources"], s["tracks"], s["history"], s["mics"], held=s.get("held")):
            for ch, kind in row:
                t.append(ch, style=styles[kind])
            t.append("\n")
        kind = {"P": "position", "D": "direction only", "-": "no result"}[s["kind"]]
        if s.get("held"):
            t.append(f"held: az {s['held'][0]:5.1f}  (no sound for {s['held'][2]:.1f} s)\n", style="white")
        else:
            t.append(f"frame: {kind}\n", style="bold" if s["sources"] else "dim")
        for src in s["sources"][:2]:
            t.append("  " + fmt_source(*src) + "\n", style="yellow")
        if s["residual_us"] is not None:
            label = "closure" if a in "GR" else "fit residual"
            t.append(f"  {label} {s['residual_us']:.1f} us\n", style="dim")
        t.append(f"speakers: {len(s['tracks'])}\n", style="bold green" if s["tracks"] else "dim")
        for tid, az, el, dist, conf in s["tracks"][:3]:
            t.append(f"  {track_mark(tid)} #{tid} " + fmt_source(az, el, dist, conf) + "\n", style="green")
        pct = 100 * s["computed"] / s["voice"] if s["voice"] else 100
        t.append(f"{s['avg_ms']:.1f} ms/frame  voice {s['voice']}  done {pct:.0f}%  results {s['with_result']}",
                 style="dim" if pct >= 99 else "yellow")
        cols.append(t)
    if cols:
        grid.add_row(*cols)
    m = snap.get("match")
    match = ""
    if m:
        match = (f"match: M4-M6 rotated {m[0]:+.0f} deg vs M1-M3" +
                 (f" ({m[1]} frames, spread {m[2]:.0f} deg)" if m[1] >= 0 else " (fixed)") + "  ")
    elif any(a in snap["algs"] for a in "GRH"):
        match = "match: waiting for frames with both groups valid  "
    info = (match + f"geometry: {snap['geometry']}  c={snap['c']:.1f} m/s  tracker {snap['track_mode']}  "
            f"queue {snap['queue']}  dropped {snap['dropped']}  restarts {snap['resets']}")
    if not snap["enabled"]:
        info += "  [OFF, p = on]"
    title = ("PC localization  ·  1-6 mics (M1-M3 enlarged)  * last frame  . last 3 s  o held (silence)  a b = speakers"
             + ("" if snap.get("vad") != "steady" else "  ·  VAD steady"))
    if snap["error"]:
        title += f"  [red]{snap['error']}[/red]"
    from rich.console import Group
    return Panel(Group(grid, Text(info, style="red" if snap["dropped"] else "dim")), title=title,
                 border_style="green")


def run_plain(usb, uart, args):
    """No UI, one line per second (logging / testing)."""
    t_end = time.monotonic() + args.duration if args.duration else None
    while t_end is None or time.monotonic() < t_end:
        time.sleep(1)
        if args.pc_log and usb and usb.loc and usb.fmt and not usb.loc.log:
            print(usb.toggle_session(), flush=True)
            args.pc_log = False
        if usb:
            s = usb.snapshot()
            lv = " ".join(f"{d:6.1f}" for d in s["dbfs"]) if s["dbfs"] is not None else "-"
            print(f"usb {s['rate'] / 1000:7.1f} kB/s pk={s['packets']} lost={s['lost']} "
                  f"resync={s['resyncs']} bad={s['bad']} jumps={s['jumps']} seq={s['seq']} dBFS {lv}",
                  flush=True)
            if s["check"]:
                print("mic " + " ".join(f"M{i + 1}:{st}" for i, st in enumerate(s["check"][0])), flush=True)
            print("det", det_summary(s), flush=True)
            if usb.loc:
                from pc_localize import plain_line
                print(plain_line(usb.loc.snapshot()), flush=True)
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
    ap.add_argument("--csv", action="store_true", help="write the detection CSV from the start (recordings/det_*.csv)")
    ap.add_argument("--no-pc-loc", action="store_true", help="no localization on the PC")
    ap.add_argument("--sim", help="daq-simulator folder (contains daq_sim/); default: next to daq-detector or $DAQ_SIM")
    ap.add_argument("--track", default="AZ", choices=["AZ", "AZEL", "ELLIPSE"], help="tracker mode on the PC")
    ap.add_argument("--pc-alg", default="BGRK",
                    help="algorithms on the PC: A, B, C (whole array, need the full geometry), G (group M1-M3), "
                         "R (group M4-M6), H (hybrid of G and R), K (Kalman of B+R+G, the smoothed result); "
                         "default BGRK")
    ap.add_argument("--show", default="BRK", help="radars to draw (default BRK); everything in --pc-alg is computed "
                                                   "and logged, only these are drawn")
    ap.add_argument("--pc-log", action="store_true",
                    help="start the session log (key g) as soon as the stream runs: recordings/pc_<time>/")
    ap.add_argument("--vad", default="steady", choices=["steady", "device"],
                    help="PC voice detection: steady = floor is the quietest frame of the last 30 s (a steady source "
                         "stays detected), device = the firmware rule (floor rises 0.05 dB/frame, a steady sound fades "
                         "out after ~6 s)")
    ap.add_argument("--group-offset", type=float, help="rotation of the M4-M6 group relative to M1-M3 in degrees "
                                                       "(default: matched from the data)")
    ap.add_argument("--config", help="config.h with the default geometry (default: main/config.h); "
                                     "the device geometry from the stream header wins")
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
    if usb and args.csv:
        usb.csv_pending = True
    loc_msg = ""
    if usb and not args.no_pc_loc:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        try:
            from pc_localize import PcLocalizer
            usb.loc = PcLocalizer(args.sim, args.config, args.track, args.pc_alg, args.group_offset, args.vad)
            loc_msg = f"PC localization: daq-sim from {usb.loc.sim_path}"
        except ImportError as e:
            loc_msg = f"PC localization off: {e}"
            print(loc_msg, file=sys.stderr)
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
        msg = loc_msg
        try:
            with Live(render(usb, uart, msg, args), refresh_per_second=8, screen=True) as live:
                while True:
                    if args.pc_log and usb and usb.loc and usb.fmt and not usb.loc.log:
                        msg = usb.toggle_session()
                        args.pc_log = False
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
                        elif k == "l" and usb:
                            msg = usb.toggle_csv()
                        elif k == "g" and usb:
                            msg = usb.toggle_session()
                        elif k == "p" and usb and usb.loc:
                            usb.loc.enabled = not usb.loc.enabled
                            msg = f"PC localization {'ON' if usb.loc.enabled else 'OFF'}"
                        elif k == "c" and usb:
                            usb.reset_counters()
                            if usb.loc:
                                usb.loc.reset()
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
        if usb.loc and usb.loc.log:
            print(usb.toggle_session())
        usb.close()
        if usb.loc:
            usb.loc.close()
        if usb.wav_name:
            print(f"WAV: {usb.wav_name}")
        if usb.csv_name:
            print(f"CSV: {usb.csv_name} ({usb.csv_lines} lines)")
    if uart:
        uart.running = False


if __name__ == "__main__":
    main()
