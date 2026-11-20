"""PC-side localization for daq_term: algorithms A, B and C at the same time on the audio of the USB stream.

Uses the Python reference of the firmware (`daq-sim`, repository daq-simulator): the same front end
(daq_sim.tdoa, 32 ms frames, VAD, GCC-PHAT), algorithm A (daq_sim.ls, GCC-PHAT + least squares),
B (daq_sim.srp, SRP-PHAT) and C (daq_sim.music, MUSIC on M1-M3), each followed by its own tracker
(daq_sim.track), so the results can be compared with each other and with the ESP.

Threads and processes:
  feed()      called by the USB reader for every audio packet, only queues the block,
  main        framing + TDOA + A + C + trackers A, C for every frame (~8 ms per 32 ms frame),
  B process   SRP-PHAT (~40-50 ms per frame in Python, a separate process so it does not slow A and C):
              at most B_IN_FLIGHT frames at a time; voice frames that find it busy are skipped,
  B order     feeds tracker B with every frame in order (skipped / non-voice frames with no source, as on
              the device without a result), waiting for the B process where needed.

A lost USB block restarts the frame window (like dsp.c); the VAD floor and the trackers keep running.
"""
import collections
import multiprocessing as mp
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ALGS = ("A", "B", "C")
ALG_NAMES = {"A": "GCC-PHAT + LS", "B": "SRP-PHAT", "C": "MUSIC M1-M3"}
SHOW_S = 1.0                 # a frame result is shown for this long after the frame
B_IN_FLIGHT = 2              # SRP-PHAT frames queued / running in the B process


def _b_worker(sim_path, jobs, results):
    """B process: SRP-PHAT on (gen, job_id, corr, cross) jobs; ('geom', gen, mics, c, fs) rebuilds the grid."""
    sys.path.insert(0, sim_path)
    from daq_sim.geometry import Geometry
    from daq_sim.srp import SrpPhat
    from daq_sim.tdoa import TdoaEstimator
    srp = None
    while True:
        msg = jobs.get()
        if msg is None:
            return
        try:
            if msg[0] == "geom":
                _, gen, mics, c, fs = msg
                g = Geometry(np.asarray(mics), c, fs)
                srp = SrpPhat(g, TdoaEstimator(g).max_lag)
                continue
            _, gen, job_id, corr, cross = msg
            t0 = time.perf_counter()
            out = srp.process(corr, cross)
            ms = (time.perf_counter() - t0) * 1000
            results.put((gen, job_id, ms, out.sources[0].kind if out.sources else "-",
                         [(s.az_deg, s.el_deg, s.dist_m, s.confidence) for s in out.sources], None))
        except Exception as e:          # report and keep going
            results.put((msg[1], msg[2] if len(msg) > 2 else -1, 0.0, "-", [], repr(e)))


def find_daq_sim(path=None):
    """Put daq-sim on sys.path and import it. Returns the folder used, raises ImportError with a hint."""
    here = Path(__file__).resolve().parents[1]
    candidates = [path] if path else [os.environ.get("DAQ_SIM"), here.parent / "daq-simulator",
                                      here.parent / "daq-sim", here / "daq-simulator", Path.cwd()]
    for c in candidates:
        if c and (Path(c) / "daq_sim" / "tdoa.py").is_file():
            sys.path.insert(0, str(Path(c).resolve()))
            import daq_sim.tdoa  # noqa: F401
            return str(Path(c).resolve())
    raise ImportError("daq-sim not found: clone https://github.com/AGH-Skylink/daq-simulator next to daq-detector "
                      "or pass --sim PATH (folder that contains daq_sim/)")


@dataclass
class AlgState:
    name: str
    frames: int = 0              # frames seen (B: offered)
    voice: int = 0               # voice frames
    computed: int = 0            # voice frames actually localized (B may skip)
    with_result: int = 0         # localized frames that gave at least one source
    busy_s: float = 0.0          # total compute time
    last_ms: float = 0.0
    last_frame: int = -1         # frame of the last result
    last_t: float = 0.0          # monotonic time of the last result
    kind: str = "-"
    sources: list = field(default_factory=list)   # (az, el, dist, conf)
    tracks: list = field(default_factory=list)    # (id, az, el, dist, conf)
    residual_us: float = None


class PcLocalizer:
    def __init__(self, sim_path=None, config_h=None, track_mode="AZ", algs="ABC"):
        self.sim_path = find_daq_sim(sim_path)
        from daq_sim.geometry import load_geometry
        from daq_sim.track import MODES
        if track_mode not in MODES:
            raise ValueError(f"tracker mode {track_mode}, expected one of {', '.join(MODES)}")
        self.track_mode = track_mode
        self.algs = [a for a in ALGS if a in algs.upper()]
        if config_h is None:
            cand = Path(__file__).resolve().parents[1] / "main" / "config.h"
            config_h = cand if cand.is_file() else None
        self.geometry = load_geometry(config_h)
        self.geometry_src = "config.h" if config_h else "daq-sim defaults"
        self.lock = threading.Lock()
        self.q = queue.Queue(maxsize=400)          # ~2.7 s of 320-frame blocks
        self.dropped_blocks = 0                    # blocks lost because the PC was too slow
        self.resets = 0                            # frame window restarts (lost USB blocks)
        self.error = None
        self.running = True
        self.enabled = True
        self.b_gen = 0                             # geometry generation (stale B results are ignored)
        self.b_next_id = 0
        self.b_inflight = 0
        self.b_order = collections.deque()         # (gen, frame, voice, job_id or None), frame order
        self.b_results = {}                        # job_id -> (ms, kind, sources)
        self.b_cond = threading.Condition()
        self.b_proc = None
        if "B" in self.algs:
            ctx = mp.get_context("spawn")
            self.b_jobs, self.b_out = ctx.Queue(), ctx.Queue()
            self.b_proc = ctx.Process(target=_b_worker, args=(self.sim_path, self.b_jobs, self.b_out), daemon=True)
            self.b_proc.start()
        self.state = {a: AlgState(a) for a in ALGS}
        self._rebuild = None                       # (geometry, source name): rebuild in the worker
        self._build()
        self.main_thread = threading.Thread(target=self._run, daemon=True)
        self.main_thread.start()
        if self.b_proc:
            threading.Thread(target=self._read_b, daemon=True).start()
            threading.Thread(target=self._order_b, daemon=True).start()

    # ------------------------------------------------------------------ setup
    def _build(self):
        from daq_sim.music import Music
        from daq_sim.tdoa import TdoaEstimator
        from daq_sim.track import MODES, Tracker
        g = self.geometry
        self.est = TdoaEstimator(g)
        self.music = Music(g) if "C" in self.algs else None
        self.trackers = {a: Tracker(g.fs, MODES[self.track_mode]) for a in ALGS}
        self.buf = np.zeros((0, 6), dtype=np.int16)
        self.buf_start = None                      # capture frame index of buf[0]
        self.next_frame = None                     # expected first_frame of the next block
        if getattr(self, "b_proc", None):
            with self.b_cond:
                self.b_gen += 1
                self.b_order.clear()
                self.b_results.clear()
                self.b_cond.notify_all()
            self.b_jobs.put(("geom", self.b_gen, g.mics_m.tolist(), g.c, g.fs))

    def set_geometry_from_header(self, header):
        """Geometry, speed of sound and fs the device actually uses (NVS calibration), from the
        detection log header '# daq-detector ... fs=48000 c=343.0 mics=x,y,z;...'."""
        from daq_sim.geometry import parse_csv_header
        try:
            g, _ = parse_csv_header(header.split("\n", 1)[0])
        except (KeyError, ValueError):
            return
        if g.mics_m.shape != (6, 3):
            return
        same = (np.allclose(g.mics_m, self.geometry.mics_m) and abs(g.c - self.geometry.c) < 1e-6
                and g.fs == self.geometry.fs)
        if same:
            self.geometry_src = "device (NVS)"
        else:
            self._rebuild = (g, "device (NVS)")

    # ------------------------------------------------------------------ input (USB reader thread)
    def feed(self, first_frame, rate, ch, block):
        """block: (frames, ch) int16. Never blocks: a full queue drops the block (and restarts the window)."""
        if not self.enabled or ch != 6:
            return
        try:
            self.q.put_nowait((first_frame, rate, block))
        except queue.Full:
            self.dropped_blocks += 1

    # ------------------------------------------------------------------ main worker
    def _run(self):
        try:
            while self.running:
                try:
                    first_frame, rate, block = self.q.get(timeout=0.2)
                except queue.Empty:
                    continue
                if self._rebuild is not None:
                    with self.lock:
                        (self.geometry, self.geometry_src), self._rebuild = self._rebuild, None
                        self._build()
                if rate != self.geometry.fs:
                    self.error = f"stream fs {rate} Hz != geometry fs {self.geometry.fs} Hz"
                    continue
                if self.next_frame is None or first_frame != self.next_frame:
                    if self.next_frame is not None:
                        self.resets += 1
                    self.buf = block.copy()
                    self.buf_start = first_frame
                else:
                    self.buf = np.concatenate([self.buf, block])
                self.next_frame = first_frame + block.shape[0]
                self._frames()
        except Exception as e:                     # keep the UI alive, show the error
            self.error = f"PC localization: {e!r}"

    def _frames(self):
        from daq_sim.tdoa import FRAME_LEN, HOP
        while self.buf.shape[0] >= FRAME_LEN:
            frame = self.buf_start
            r = self.est.process(self.buf[:FRAME_LEN], frame)
            self.buf = self.buf[HOP:]
            self.buf_start += HOP
            if "A" in self.algs:
                self._alg_a(r)
            if "C" in self.algs:
                self._alg_c(r)
            if self.b_proc:
                job = None
                with self.b_cond:
                    if r.voice and self.b_inflight < B_IN_FLIGHT:
                        job = self.b_next_id
                        self.b_next_id += 1
                        self.b_inflight += 1
                    self.b_order.append((self.b_gen, frame, r.voice, job))
                    self.b_cond.notify_all()
                if job is not None:
                    self.b_jobs.put(("job", self.b_gen, job, r.corr, r.cross))

    def _publish(self, alg, r, kind, sources, ms, computed=True, residual=None):
        tracks = self.trackers[alg].update(r.frame if r is not None else None, sources)
        with self.lock:
            s = self.state[alg]
            s.frames += 1
            if r is not None and r.voice:
                s.voice += 1
                if computed:
                    s.computed += 1
                    s.busy_s += ms / 1000
                    s.last_ms = ms
                    if sources:
                        s.with_result += 1
                        s.kind, s.sources, s.residual_us = kind, list(sources), residual
                        s.last_frame, s.last_t = r.frame, time.monotonic()
            s.tracks = [(t.id, t.az_deg, t.el_deg, t.dist_m, t.confidence) for t in tracks]

    def _alg_a(self, r):
        from daq_sim.geometry import to_spherical
        from daq_sim.ls import confidence, direction, locate
        if not r.voice:
            self._publish("A", r, "-", [], 0.0)
            return
        t0 = time.perf_counter()
        res = locate(self.geometry, r.tau_s)
        if res.ok:
            az, el, dist = to_spherical(res.position_m)
            kind, sources, resid = "P", [(az, el, dist, confidence(res, r.peak))], res.rms_residual_us
        else:
            d = direction(self.geometry, r.tau_s)
            if d.ok:
                kind, sources, resid = "D", [(d.az_deg, d.el_deg, 0.0, confidence(d, r.peak))], d.rms_residual_us
            else:
                kind, sources, resid = "-", [], None
        self._publish("A", r, kind, sources, (time.perf_counter() - t0) * 1000, residual=resid)

    def _alg_c(self, r):
        t0 = time.perf_counter()
        out = self.music.process(r)          # every frame: the covariance also decays without voice
        ms = (time.perf_counter() - t0) * 1000
        if out is None or not r.voice:
            self._publish("C", r, "-", [], ms, computed=r.voice)
            return
        sources = [(s.az_deg, s.el_deg, s.dist_m, s.confidence) for s in out.sources]
        self._publish("C", r, "D" if sources else "-", sources, ms)

    # ------------------------------------------------------------------ B process
    def _read_b(self):
        """Results of the B process -> b_results (stale generations dropped)."""
        while self.running:
            try:
                gen, job_id, ms, kind, sources, err = self.b_out.get(timeout=0.5)
            except queue.Empty:
                if self.b_proc and not self.b_proc.is_alive() and self.running:
                    self.error = "PC localization B: process stopped"
                    return
                continue
            except (EOFError, OSError):
                return
            if err:
                self.error = f"PC localization B: {err}"
            with self.b_cond:
                self.b_inflight = max(0, self.b_inflight - 1)
                if gen == self.b_gen:
                    self.b_results[job_id] = (ms, kind, sources)
                self.b_cond.notify_all()

    def _order_b(self):
        """Tracker B and the B statistics, frame by frame in order."""
        try:
            while self.running:
                with self.b_cond:
                    while self.running and not (self.b_order and (self.b_order[0][3] is None
                                                                  or self.b_order[0][3] in self.b_results)):
                        self.b_cond.wait(0.2)
                    if not self.running:
                        return
                    gen, frame, voice, job = self.b_order.popleft()
                    res = self.b_results.pop(job) if job is not None else None
                if gen != self.b_gen:
                    continue
                if res is None:
                    self._publish_b(frame, voice, [], 0.0, False)
                else:
                    ms, kind, sources = res
                    self._publish_b(frame, voice, sources, ms, True, kind)
        except Exception as e:
            self.error = f"PC localization B: {e!r}"

    def _publish_b(self, frame, voice, sources, ms, computed, kind="-"):
        tracks = self.trackers["B"].update(frame, sources)
        with self.lock:
            s = self.state["B"]
            s.frames += 1
            if voice:
                s.voice += 1
                if computed:
                    s.computed += 1
                    s.busy_s += ms / 1000
                    s.last_ms = ms
                    if sources:
                        s.with_result += 1
                        s.kind, s.sources, s.residual_us = kind, list(sources), None
                        s.last_frame, s.last_t = frame, time.monotonic()
            s.tracks = [(t.id, t.az_deg, t.el_deg, t.dist_m, t.confidence) for t in tracks]

    # ------------------------------------------------------------------ output
    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            out = {}
            for a in self.algs:
                s = self.state[a]
                fresh = s.last_t and now - s.last_t <= SHOW_S
                out[a] = dict(
                    name=ALG_NAMES[a], frames=s.frames, voice=s.voice, computed=s.computed,
                    with_result=s.with_result, avg_ms=(s.busy_s * 1000 / s.computed) if s.computed else 0.0,
                    last_ms=s.last_ms, kind=s.kind if fresh else "-", sources=list(s.sources) if fresh else [],
                    tracks=list(s.tracks), residual_us=s.residual_us if fresh else None)
            return dict(algs=out, geometry=self.geometry_src, c=self.geometry.c, track_mode=self.track_mode,
                        queue=self.q.qsize(), dropped=self.dropped_blocks, resets=self.resets,
                        error=self.error, enabled=self.enabled)

    def reset(self):
        """Clear the statistics and the trackers (key c)."""
        with self.lock:
            self.state = {a: AlgState(a) for a in ALGS}
            self.dropped_blocks = self.resets = 0
            if self._rebuild is None:
                self._rebuild = (self.geometry, self.geometry_src)   # trackers restart in the worker thread

    def close(self):
        self.running = False
        with self.b_cond:
            self.b_cond.notify_all()
        if self.b_proc:
            try:
                self.b_jobs.put(None)
                self.b_proc.join(timeout=1.0)
            except (OSError, ValueError):
                pass
            if self.b_proc.is_alive():
                self.b_proc.terminate()


# ---------------------------------------------------------------------- text rendering

def radar(sources, tracks, w=31, h=15):
    """ASCII top view: azimuth 0 to the right (x), 90 up (y), as in the array coordinates.
    Ring = direction; arms M4 (150), M5 (270), M6 (30) marked 4 5 6 inside; frame sources '*',
    confirmed tracks by their id digit (last digit). Returns a list of strings."""
    grid = [[" "] * w for _ in range(h)]
    cx, cy = (w - 1) / 2, (h - 1) / 2
    rx, ry = cx - 1, cy - 1

    def put(az, frac, ch):
        a = np.radians(az)
        x = int(round(cx + frac * rx * np.cos(a)))
        y = int(round(cy - frac * ry * np.sin(a)))
        if 0 <= x < w and 0 <= y < h:
            grid[y][x] = ch

    for az in range(0, 360, 6):
        put(az, 1.0, "·")
    for az, label in ((0, "0"), (90, "90"), (180, "180"), (270, "270")):
        a = np.radians(az)
        x = int(round(cx + (rx + 0.5) * np.cos(a)))
        y = int(round(cy - (ry + 0.5) * np.sin(a)))
        x = min(max(x - (len(label) - 1 if az == 0 else 0), 0), w - len(label))
        if az == 180:
            x = 0
        for i, c in enumerate(label):
            grid[y][x + i] = c
    for az, label in ((150, "4"), (270, "5"), (30, "6")):
        put(az, 0.45, label)
    grid[int(cy)][int(cx)] = "+"
    for az, el, dist, conf in sources:
        put(az, 0.8, "*")
    for tid, az, el, dist, conf in tracks:
        put(az, 1.0, str(tid % 10))
    return ["".join(r) for r in grid]


def fmt_source(az, el, dist, conf):
    d = f"{dist:4.1f} m" if dist > 0 else "  ?  "
    return f"az {az:5.1f}  el {el:5.1f}  d {d}  c {conf:.2f}"


def plain_line(snap):
    parts = []
    for a, s in snap["algs"].items():
        txt = f"{a}:{s['kind']}"
        if s["sources"]:
            txt += " " + " / ".join(f"az {x[0]:.0f} el {x[1]:.0f} d {x[2]:.1f}" for x in s["sources"])
        txt += f" spk={len(s['tracks'])}"
        if s["tracks"]:
            txt += " (" + ",".join(f"{t[1]:.0f}" for t in s["tracks"]) + ")"
        txt += f" {s['avg_ms']:.1f}ms {s['computed']}/{s['voice']}"
        parts.append(txt)
    return "pc " + " | ".join(parts) + f" | q={snap['queue']} drop={snap['dropped']}"
