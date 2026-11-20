"""PC-side localization for daq_term: algorithms A, B and C at the same time on the audio of the USB stream.

Uses the Python reference of the firmware (`daq-sim`, repository daq-simulator): the same front end
(daq_sim.tdoa, 32 ms frames, VAD, GCC-PHAT), algorithm A (daq_sim.ls, GCC-PHAT + least squares),
B (daq_sim.srp, SRP-PHAT) and C (daq_sim.music, MUSIC on M1-M3), each followed by its own tracker
(daq_sim.track), so the results can be compared with each other and with the ESP.

Group mode (G, R, H; the default while the relative position of the two mic groups is not known): see GROUP_*.

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
import csv
import multiprocessing as mp
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ALGS = ("A", "B", "C", "G", "R", "H", "K")
ALG_NAMES = {"A": "GCC-PHAT + LS", "B": "SRP-PHAT", "C": "MUSIC M1-M3",
             "G": "group M1-M3", "R": "group M4-M6", "H": "hybrid M1-M3 + M4-M6", "K": "Kalman B+R+G"}

# K: one azimuth Kalman filter (constant velocity) fed with the per-frame directions of B, R and G as separate
# measurements, each with its own accuracy and its own outlier gate. Tuned on a 112 s session of this array with
# a moving source (pc_20261120_140412): against the consensus of the methods it covered 99.8 % of the frames with a
# median error of 2.9 deg and 1.7 % of errors > 30 deg, where the old tracker had 10.5 deg / 12 % and raw B jumped
# ~25 deg per frame. Directions of the groups are used as they are (no rotation between the groups).
K_NOISE_DEG = {"B": 6.9, "R": 7.7, "G": 14.3, "A": 6.0}   # robust error of each method on that session
K_Q = 6400.0                 # acceleration noise, (deg/s^2)^2: allows a person walking around the array
K_GATE_SIGMA, K_GATE_MIN_DEG = 3.5, 20.0
K_REACQUIRE = 5              # frames in a row outside the gate that agree with each other -> jump to them
K_HOLD_S = 0.8               # keep showing the estimate this long without accepted measurements
K_RESET_S = 3.0              # forget the source after this long


class AzKalman:
    def __init__(self, dt):
        self.dt = dt
        self.F = np.array([[1.0, dt], [0.0, 1.0]])
        self.Q = K_Q * np.array([[dt ** 4 / 4, dt ** 3 / 2], [dt ** 3 / 2, dt ** 2]])
        self.x = None
        self.P = None
        self.since = 1e9          # frames since the last accepted measurement
        self.pending = []

    def step(self, meas):
        """meas: list of (az_deg, el_deg, sigma_deg). Returns (az, el) or None (nothing to show)."""
        if self.x is not None:
            self.x = self.F @ self.x
            self.P = self.F @ self.P @ self.F.T + self.Q
        self.since += 1
        if meas:
            if self.x is None:
                self._start(meas)
            else:
                used = 0
                for az, el, sig in sorted(meas, key=lambda m: m[2]):
                    y = wrap180(az - self.x[0])
                    s = self.P[0, 0] + sig ** 2
                    if abs(y) <= max(K_GATE_MIN_DEG, K_GATE_SIGMA * np.sqrt(s)):
                        k = self.P[:, 0] / s
                        self.x = self.x + k * y
                        self.P = self.P - np.outer(k, self.P[0, :])
                        if not used:
                            self.el = el
                        used += 1
                if used:
                    self.since = 0
                    self.pending = []
                else:
                    self.pending.append(circ_mean([m[0] for m in meas])[0])
                    if (len(self.pending) >= K_REACQUIRE and
                            max(abs(wrap180(p - circ_mean(self.pending)[0])) for p in self.pending) < K_GATE_MIN_DEG):
                        self._start(meas, circ_mean(self.pending)[0])
            self.x[0] %= 360.0
            self.x[1] = float(np.clip(self.x[1], -180.0, 180.0))
        if self.x is None:
            return None
        if self.since * self.dt > K_RESET_S:
            self.x = None
            self.pending = []
            return None
        return (float(self.x[0]), float(self.el)) if self.since * self.dt <= K_HOLD_S else None

    def _start(self, meas, az=None):
        best = min(meas, key=lambda m: m[2])
        self.x = np.array([best[0] if az is None else az, 0.0])
        self.P = np.diag([best[2] ** 2, 60.0 ** 2])
        self.el = best[1]
        self.since = 0
        self.pending = []

# Group mode (G, R, H): the shape of each group (body M1-M3, arms M4-M6) is known, their relative position is not.
# Each group gives a far-field direction from its own 3 pairs only (plane wave in the group plane: azimuth and
# |elevation|), which does not depend on where the group is. The two azimuths are compared: their running
# difference is the rotation of the arms group relative to the body (MATCH_*), and the hybrid (H) combines both
# directions in the body frame, weighted by their accuracy (aperture and closure error).
GROUP_BODY, GROUP_ARMS = (0, 1, 2), (3, 4, 5)
CLOSURE_MAX_US = {"G": 25.0, "R": 80.0}       # |tau_ab - tau_ac + tau_bc| above this = inconsistent frame
CLOSURE_FLOOR_US = {"G": 5.0, "R": 10.0}       # accuracy floor for the weights
MATCH_FRAMES = 150                             # frames with both groups valid used for the offset (~5 s of voice)
MATCH_MIN = 10                                 # before this many, the hybrid does not rotate the arms result
DISAGREE_DEG = 30.0                            # larger difference after the offset: keep the more accurate group


def group_direction(geometry, r, grp):
    """Plane-wave direction from the 3 pairs of one group (mics assumed in a horizontal plane).
    Returns dict(az, el, closure_us, sigma_deg, peak, ok) - el is |elevation| (the sign is not observable)."""
    from daq_sim.geometry import PAIRS
    m = geometry.mics_m
    pairs = [(grp[0], grp[1]), (grp[0], grp[2]), (grp[1], grp[2])]
    a = np.array([(m[j] - m[i])[:2] for i, j in pairs])
    y = np.array([r.tau_s[PAIRS.index(p)] for p in pairs])
    v, *_ = np.linalg.lstsq(a, y, rcond=None)
    closure = (y[0] - y[1] + y[2]) * 1e6
    cos_el = float(np.linalg.norm(v) * geometry.c)
    az = float(np.degrees(np.arctan2(v[1], v[0])) % 360.0)
    el = float(np.degrees(np.arccos(min(cos_el, 1.0))))
    peak = float(min(r.peak[PAIRS.index(p)] for p in pairs))
    aperture_us = max(np.linalg.norm(m[j] - m[i]) for i, j in pairs) / geometry.c * 1e6
    return dict(az=az, el=el, cos_el=cos_el, closure_us=closure, peak=peak, aperture_us=aperture_us)


def circ_mean(deg, w=None):
    z = np.exp(1j * np.radians(np.asarray(deg, dtype=float)))
    m = np.average(z, weights=w) if w is not None else np.mean(z)
    return float(np.degrees(np.angle(m)) % 360.0), float(abs(m))


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0
SHOW_S = 1.0                 # a frame result is shown for this long after the frame
HISTORY_S = 3.0              # earlier results stay on the radar as dots for this long
STEADY_VAD_WINDOW = 940      # frames (~30 s): steady VAD floor = minimum band energy over this window
VAD_MARGIN_DB = 9.0          # same margin as the device
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
    history: collections.deque = field(default_factory=lambda: collections.deque(maxlen=400))   # (t, az)


class SessionLog:
    """CSV logs of one session (key g in daq_term), all in one folder:
      frames.csv   every frame: voice decision, band energy, 15 time differences (us) and GCC-PHAT peaks
                   (enough to rerun any localization / smoothing method offline)
      results.csv  every frame x algorithm: frame result (up to 2 sources), fit residual / closure, time,
                   and the tracker output after the frame (up to 2 speakers)
      groups.csv   group mode: direction of each group (also when rejected), closure, accuracy, matched rotation
      info.txt     geometry, speed of sound, algorithms, tracker and VAD mode
    pc_time_s is the PC clock (Unix seconds), frame the capture sample index (same as the ESP detection log)."""
    def __init__(self, folder, loc):
        from daq_sim.geometry import PAIRS
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        names = [f"{i + 1}{j + 1}" for i, j in PAIRS]
        self.f = {k: open(self.folder / f"{k}.csv", "w", newline="", encoding="utf-8")
                  for k in ("frames", "results", "groups")}
        self.w = {k: csv.writer(f) for k, f in self.f.items()}
        self.w["frames"].writerow(["pc_time_s", "frame", "voice", "energy_db"] + [f"t{n}_us" for n in names]
                                  + [f"p{n}" for n in names])
        self.w["results"].writerow(["pc_time_s", "frame", "alg", "voice", "computed", "kind", "n",
                                    "az1", "el1", "dist1", "conf1", "az2", "el2", "dist2", "conf2", "resid_us", "ms",
                                    "speakers", "s1_id", "s1_az", "s1_el", "s1_dist", "s1_conf",
                                    "s2_id", "s2_az", "s2_el", "s2_dist", "s2_conf"])
        self.w["groups"].writerow(["pc_time_s", "frame",
                                   "G_az", "G_el", "G_cos_el", "G_closure_us", "G_sigma_deg", "G_peak", "G_ok",
                                   "R_az", "R_el", "R_cos_el", "R_closure_us", "R_sigma_deg", "R_peak", "R_ok",
                                   "match_deg", "match_frames", "match_spread_deg"])
        g = loc.geometry
        with open(self.folder / "info.txt", "w", encoding="utf-8") as f:
            f.write(f"created {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"algorithms {''.join(loc.algs)}\ntracker {loc.track_mode}\nvad {loc.vad_mode}\n")
            f.write(f"geometry {loc.geometry_src}\nc {g.c}\nfs {g.fs}\n")
            f.write(f"group_offset {loc.fixed_offset}\n")
            for m in range(6):
                f.write(f"M{m + 1} " + " ".join(f"{v * 1000:.1f}" for v in g.mics_m[m]) + " mm\n")
        self.rows = 0
        self.last_flush = time.monotonic()

    def write(self, kind, row):
        with self.lock:
            if not self.f:
                return
            self.w[kind].writerow([f"{time.time():.3f}"] + [_fmt(v) for v in row])
            self.rows += 1
            if time.monotonic() - self.last_flush > 1.0:
                for f in self.f.values():
                    f.flush()
                self.last_flush = time.monotonic()

    def close(self):
        with self.lock:
            for f in self.f.values():
                f.close()
            self.f = {}


def _fmt(v):
    if v is None:
        return ""
    if isinstance(v, (bool, np.bool_)):
        return int(v)
    if isinstance(v, (float, np.floating)):
        return f"{v:.3f}"
    return v


def _src_cols(sources):
    out = []
    for k in range(2):
        out += list(sources[k]) if k < len(sources) else [None] * 4
    return out


def _track_cols(tracks):
    out = [len(tracks)]
    for k in range(2):
        out += [tracks[k].id, tracks[k].az_deg, tracks[k].el_deg, tracks[k].dist_m, tracks[k].confidence] \
            if k < len(tracks) else [None] * 5
    return out


class PcLocalizer:
    def __init__(self, sim_path=None, config_h=None, track_mode="AZ", algs="BGRK", group_offset=None, vad="steady"):
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
        self.match = collections.deque(maxlen=MATCH_FRAMES)   # az(arms) - az(body), frames with both valid
        self.fixed_offset = group_offset                       # --group-offset: no matching, use this rotation
        if vad not in ("steady", "device"):
            raise ValueError("vad must be 'steady' or 'device'")
        self.vad_mode = vad
        self.energy_hist = collections.deque(maxlen=STEADY_VAD_WINDOW)
        self.log = None                                        # SessionLog while logging
        self.kalman = AzKalman(1536 / 48000)
        self.k_meas = {}                                       # frame -> (voice, measurements) waiting for B
        self._grp = None                                       # (G, R) of the current frame
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
        if hasattr(self, "kalman"):
            self.kalman = AzKalman(1536 / 48000)
            self.k_meas = {}
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
            if self.vad_mode == "steady":
                # device VAD: the floor rises 0.05 dB per frame, so a steady sound stops being "voice" after
                # ~6 s; here the floor is the quietest frame of the last ~30 s, so a steady source stays on
                self.energy_hist.append(r.energy_db)
                r.voice = r.energy_db > min(self.energy_hist) + VAD_MARGIN_DB
            if self.log:
                self.log.write("frames", [frame, r.voice, r.energy_db] + list(r.tau_s * 1e6) + list(r.peak))
            if any(a in self.algs for a in "GRH"):
                self._groups(r)
            if "A" in self.algs:
                self._alg_a(r)
            if "C" in self.algs:
                self._alg_c(r)
            if "K" in self.algs:
                meas = []
                if r.voice and self._grp:
                    for key, d in zip("GR", self._grp):
                        if d["ok"]:
                            meas.append((d["az"], d["el"], K_NOISE_DEG[key]))
                if r.voice and "A" in self.algs and self.state["A"].last_frame == frame and self.state["A"].sources:
                    src = self.state["A"].sources[0]
                    meas.append((src[0], src[1], K_NOISE_DEG["A"]))
                self._grp = None
                if self.b_proc:
                    self.k_meas[frame] = (r.voice, meas)     # finished by the B order thread with B's result
                else:
                    self._k_step(frame, r.voice, meas)
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
        if self.log and r is not None:
            self.log.write("results", [r.frame, alg, r.voice, computed, kind, len(sources)] + _src_cols(sources)
                           + [residual, ms] + _track_cols(tracks))
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
                        s.history.extend((s.last_t, src[0]) for src in sources)
            s.tracks = [(t.id, t.az_deg, t.el_deg, t.dist_m, t.confidence) for t in tracks]

    def _group(self, key, r):
        d = group_direction(self.geometry, r, GROUP_BODY if key == "G" else GROUP_ARMS)
        d["ok"] = (abs(d["closure_us"]) <= CLOSURE_MAX_US[key] and 0.2 <= d["cos_el"] <= 1.3)
        # angular accuracy ~ timing error / aperture (rad -> deg)
        d["sigma_deg"] = float(np.degrees(max(abs(d["closure_us"]), CLOSURE_FLOOR_US[key]) / d["aperture_us"]))
        return d

    def mic_layout(self, alg):
        """(label, az, radius fraction) of the mics drawn on this algorithm's radar, from the geometry in use.
        Radius to scale with the farthest mic at 0.55; the small body triangle is drawn at >= 0.16 (enlarged)."""
        m = self.geometry.mics_m
        show = {"G": (0, 1, 2), "R": (3, 4, 5)}.get(alg, range(6))
        rmax = max(float(np.hypot(*m[i][:2])) for i in range(6)) or 1.0
        out = []
        for i in show:
            r = float(np.hypot(*m[i][:2]))
            az = float(np.degrees(np.arctan2(m[i][1], m[i][0])) % 360.0)
            out.append((str(i + 1), az, max(0.55 * r / rmax, 0.16)))
        return out

    def match_offset(self):
        """Rotation of the arms group relative to the body: (offset deg, frames, spread deg) or None."""
        if self.fixed_offset is not None:
            return self.fixed_offset, -1, 0.0
        if len(self.match) < MATCH_MIN:
            return None
        mean, rlen = circ_mean(self.match)
        spread = float(np.degrees(np.sqrt(-2 * np.log(max(rlen, 1e-9)))))
        return wrap180(mean), len(self.match), spread

    def _groups(self, r):
        t0 = time.perf_counter()
        if not r.voice:
            for a in "GRH":
                if a in self.algs:
                    self._publish(a, r, "-", [], 0.0)
            return
        g, a = self._group("G", r), self._group("R", r)
        self._grp = (g, a)
        if g["ok"] and a["ok"]:
            self.match.append(wrap180(a["az"] - g["az"]))
        off = self.match_offset()
        if self.log:
            self.log.write("groups", [r.frame] + [d[k] for d in (g, a) for k in
                                                  ("az", "el", "cos_el", "closure_us", "sigma_deg", "peak", "ok")]
                           + (list(off) if off else [None] * 3))
        ms = (time.perf_counter() - t0) * 1000
        for key, d in (("G", g), ("R", a)):
            if key in self.algs:
                src = [(d["az"], d["el"], 0.0, d["peak"])] if d["ok"] else []
                self._publish(key, r, "D" if src else "-", src, ms, residual=d["closure_us"] if src else None)
        if "H" not in self.algs:
            return
        cands = []
        if g["ok"]:
            cands.append((g["az"], g["el"], g["sigma_deg"], g["peak"]))
        if a["ok"] and off is not None:          # arms direction in the body frame
            cands.append(((a["az"] - off[0]) % 360.0, a["el"], a["sigma_deg"], a["peak"]))
        elif a["ok"] and not g["ok"]:
            cands.append((a["az"], a["el"], a["sigma_deg"], a["peak"]))   # no offset yet: arms as they are
        if not cands:
            self._publish("H", r, "-", [], ms)
            return
        if len(cands) == 2 and abs(wrap180(cands[0][0] - cands[1][0])) > DISAGREE_DEG:
            cands = [min(cands, key=lambda c: c[2])]     # disagree: the more accurate one
        w = [1.0 / c[2] ** 2 for c in cands]
        az, _ = circ_mean([c[0] for c in cands], w)
        el = float(np.average([c[1] for c in cands], weights=w))
        self._publish("H", r, "D", [(az, el, 0.0, max(c[3] for c in cands))], ms)

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
                    sources = []
                    self._publish_b(frame, voice, [], 0.0, False)
                else:
                    ms, kind, sources = res
                    self._publish_b(frame, voice, sources, ms, True, kind)
                if "K" in self.algs:
                    kv, meas = self.k_meas.pop(frame, (voice, []))
                    if sources:
                        meas = meas + [(sources[0][0], sources[0][1], K_NOISE_DEG["B"])]
                    self._k_step(frame, kv, meas)
        except Exception as e:
            self.error = f"PC localization B: {e!r}"

    def _k_step(self, frame, voice, meas):
        t0 = time.perf_counter()
        est = self.kalman.step(meas)
        src = [(est[0], est[1], 0.0, 1.0)] if est else []
        self._publish_b(frame, voice, src, (time.perf_counter() - t0) * 1000, True, "D" if src else "-", alg="K")

    def _publish_b(self, frame, voice, sources, ms, computed, kind="-", alg="B"):
        tracks = self.trackers[alg].update(frame, sources)
        if self.log:
            self.log.write("results", [frame, alg, voice, computed, kind, len(sources)] + _src_cols(sources)
                           + [None, ms] + _track_cols(tracks))
        with self.lock:
            s = self.state[alg]
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
                        s.history.extend((s.last_t, src[0]) for src in sources)
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
                    tracks=list(s.tracks), residual_us=s.residual_us if fresh else None,
                    history=[az for t, az in s.history if now - t <= HISTORY_S],
                    mics=self.mic_layout(a))
            match = self.match_offset() if any(a in self.algs for a in "GRH") else None
            return dict(algs=out, match=match, vad=self.vad_mode, geometry=self.geometry_src, c=self.geometry.c, track_mode=self.track_mode,
                        queue=self.q.qsize(), dropped=self.dropped_blocks, resets=self.resets,
                        error=self.error, enabled=self.enabled)

    def reset(self):
        """Clear the statistics and the trackers (key c)."""
        with self.lock:
            self.state = {a: AlgState(a) for a in ALGS}
            self.match.clear()
            self.dropped_blocks = self.resets = 0
            if self._rebuild is None:
                self._rebuild = (self.geometry, self.geometry_src)   # trackers restart in the worker thread

    def start_log(self, folder):
        self.stop_log()
        self.log = SessionLog(folder, self)
        return str(self.log.folder)

    def stop_log(self):
        log, self.log = self.log, None
        if log:
            log.close()
            return log.rows
        return 0

    def close(self):
        self.stop_log()
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

def track_mark(tid):
    return chr(ord("a") + (tid - 1) % 26)


def radar(sources, tracks, history=(), mics=(), w=31, h=15):
    """ASCII top view: azimuth 0 to the right (x), 90 up (y), as in the array coordinates.
    Returns rows of (char, kind): kind ring | label | mic | centre | history | source | track.
    Ring = direction; digits = mics (from the geometry; the small body triangle enlarged), '.' = results of the
    last HISTORY_S seconds, '*' = last frame, letters = confirmed speaker tracks (a = #1, b = #2, ...)."""
    grid = [[(" ", "")] * w for _ in range(h)]
    cx, cy = (w - 1) / 2, (h - 1) / 2
    rx, ry = cx - 1, cy - 1

    def put(az, frac, ch, kind):
        a = np.radians(az)
        x = int(round(cx + frac * rx * np.cos(a)))
        y = int(round(cy - frac * ry * np.sin(a)))
        if 0 <= x < w and 0 <= y < h:
            grid[y][x] = (ch, kind)

    for az in range(0, 360, 4):
        put(az, 1.0, "·", "ring")
    for az, label in ((0, "0"), (90, "90"), (180, "180"), (270, "270")):
        a = np.radians(az)
        x = int(round(cx + (rx + 0.5) * np.cos(a)))
        y = int(round(cy - (ry + 0.5) * np.sin(a)))
        x = min(max(x - (len(label) - 1 if az == 0 else 0), 0), w - len(label))
        if az == 180:
            x = 0
        for i, c in enumerate(label):
            grid[y][x + i] = (c, "label")
    grid[int(cy)][int(cx)] = ("+", "centre")
    for label, az, frac in mics:
        put(az, frac, label, "mic")
    for az in history:
        put(az, 0.8, ".", "history")
    for az, el, dist, conf in sources:
        put(az, 0.8, "*", "source")
    for tid, az, el, dist, conf in tracks:
        put(az, 1.0, track_mark(tid), "track")
    return grid


def radar_text(*args, **kw):
    return ["".join(c for c, _ in row) for row in radar(*args, **kw)]


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
    m = snap.get("match")
    if m:
        parts.append(f"match arms-body {m[0]:+.0f} deg" + (f" ({m[1]} frames, spread {m[2]:.0f})" if m[1] >= 0 else " (fixed)"))
    return "pc " + " | ".join(parts) + f" | q={snap['queue']} drop={snap['dropped']}"


# ---------------------------------------------------------------------- offline: same code on a recording

def _summary(loc, per_frame):
    print(f"\n{'alg':4s} {'name':22s} {'voice':>5s} {'results':>7s}  {'az mean':>7s} {'spread':>6s}  {'|el| med':>8s}  speakers (last)")
    for a in loc.algs:
        rows = [p for p in per_frame if p[0] == a]
        az = [p[1] for p in rows]
        s = loc.state[a]
        if az:
            m, rl = circ_mean(az)
            spread = np.degrees(np.sqrt(-2 * np.log(max(rl, 1e-9))))
            el = np.median([p[2] for p in rows])
            print(f"{a:4s} {ALG_NAMES[a]:22s} {s.voice:5d} {len(rows):7d}  {m:7.1f} {spread:6.1f}  {el:8.1f}  "
                  + (", ".join(f"#{t[0]} az {t[1]:.0f}" for t in s.tracks) or "-"))
        else:
            print(f"{a:4s} {ALG_NAMES[a]:22s} {s.voice:5d} {0:7d}")
    m = loc.match_offset()
    if m:
        print(f"\nmatch: M4-M6 rotated {m[0]:+.1f} deg relative to M1-M3"
              + (f" (last {m[1]} frames with both groups valid, spread {m[2]:.1f} deg)" if m[1] >= 0 else " (fixed)"))
    elif any(a in loc.algs for a in "GRH"):
        print("\nmatch: too few frames with both groups valid")


def main():
    import argparse
    import wave
    ap = argparse.ArgumentParser(description="PC localization on a recording (folder with M1..M6.wav from daq_term)")
    ap.add_argument("rec")
    ap.add_argument("--sim")
    ap.add_argument("--config")
    ap.add_argument("--pc-alg", default="BGRK")
    ap.add_argument("--track", default="AZ")
    ap.add_argument("--group-offset", type=float)
    ap.add_argument("--vad", default="steady", choices=["steady", "device"])
    ap.add_argument("--log", help="write the session logs (frames/results/groups.csv) to this folder")
    args = ap.parse_args()
    p = Path(args.rec)
    ch = []
    for m in range(1, 7):
        with wave.open(str(p / f"M{m}.wav")) as w:
            ch.append(np.frombuffer(w.readframes(w.getnframes()), "<i2"))
    n = min(len(c) for c in ch)
    x = np.stack([c[:n] for c in ch], axis=1)
    loc = PcLocalizer(args.sim, args.config, args.track.upper(), args.pc_alg, args.group_offset, args.vad)
    if args.log:
        loc.start_log(args.log)
    per_frame = []
    orig = loc._publish

    def spy(alg, r, kind, sources, ms, computed=True, residual=None):
        if sources and r is not None and r.voice:
            per_frame.append((alg, sources[0][0], sources[0][1]))
        orig(alg, r, kind, sources, ms, computed, residual)
    loc._publish = spy
    blk = 320
    for k in range(0, n - blk + 1, blk):
        while loc.q.qsize() > 300:
            time.sleep(0.01)
        loc.feed(k, 48000, 6, x[k:k + blk])
    while loc.q.qsize() or (loc.b_proc and (loc.b_order or loc.b_inflight)):
        time.sleep(0.05)
    time.sleep(0.3)
    print(f"{p}: {n / 48000:.1f} s, geometry {loc.geometry_src}")
    _summary(loc, per_frame)
    loc.close()


if __name__ == "__main__":
    main()
