#!/usr/bin/env python3
"""Clap test: channel order, real mic distances, digital time offsets and geometry from claps.

How to record (daq_term.py, key w):
  clap (or snap fingers / tap with a pen on the table) 2-3 times right next to M1, then M2, ... M6,
  about 1-2 cm from the microphone, ~1 s between claps, quiet room.

    python tools/clap_test.py recordings/usb_20261120_133000
    python tools/clap_test.py recordings/usb_20261120_133000 --order 1,2,3,4,5,6   # order you clapped in

What it does:
1. finds the claps (sudden broadband energy rise),
2. for every clap: arrival time at each mic from GCC-PHAT between channels in a short window (+-3 ms search,
   no geometry needed) -> the mic that hears it first is the one you clapped at,
3. channel order: with --order (default 1..6, groups of claps in time) it checks that the first channel of each
   group is the expected mic (a mismatch = wrong DAQ_CHANNEL_MAP / swapped wires),
4. distances: a clap at mic k gives t_j - t_k = |m_j - m_k| / c + (o_j - o_k), where o = digital offset of the
   channel. Claps at k and at j together separate the two: distance = c * mean, offset = half the difference.
   A real acoustic delay is symmetric, a digital offset is not,
5. geometry: positions from the distance matrix (classical MDS), rotated so that the M1-M3 centroid is the origin
   and M1 is at azimuth 90 deg (as in config.h), printed as console commands `mic N X Y Z` (mm).
   The sign of z cannot be told from distances alone: arms are put below (z <= 0) as in the pinout.
"""
import argparse
import sys
import wave
from pathlib import Path

import numpy as np

C = 343.0
FS = 48000
BAND = (500.0, 8000.0)       # claps are broadband; low frequencies carry room hum, high ones aliasing
MAX_LAG_S = 3e-3             # +-1 m path difference
WIN_PRE_S, WIN_POST_S = 0.004, 0.012


def load(path):
    p = Path(path)
    if p.is_dir():
        chans = []
        for m in range(1, 7):
            with wave.open(str(p / f"M{m}.wav")) as w:
                if w.getframerate() != FS or w.getsampwidth() != 2:
                    sys.exit(f"M{m}.wav: expected 16-bit {FS} Hz")
                chans.append(np.frombuffer(w.readframes(w.getnframes()), "<i2").astype(float))
        n = min(len(c) for c in chans)
        return np.array([c[:n] for c in chans])
    with wave.open(str(p)) as w:
        if w.getnchannels() != 6:
            sys.exit("give a folder with M1..M6.wav or a 6-channel WAV")
        x = np.frombuffer(w.readframes(w.getnframes()), "<i2").reshape(-1, 6).astype(float)
    return x.T


def bandpass(x, lo, hi):
    f = np.fft.rfftfreq(x.shape[-1], 1 / FS)
    X = np.fft.rfft(x, axis=-1)
    X[..., (f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, x.shape[-1], axis=-1)


def find_claps(x, min_gap_s=0.25, rise_db=15.0):
    """Onsets (sample index) where the 2 ms band energy (max over mics) jumps rise_db above the running floor."""
    y = bandpass(x - x.mean(axis=1, keepdims=True), *BAND)
    hop = int(0.002 * FS)
    n = y.shape[1] // hop
    e = (y[:, :n * hop].reshape(6, n, hop) ** 2).mean(axis=2).max(axis=0)
    edb = 10 * np.log10(e + 1e-9)
    floor = np.percentile(edb, 30)
    onsets, last = [], -10 ** 9
    for i in range(1, n):
        if edb[i] > floor + rise_db and edb[i] - edb[max(0, i - 5):i].mean() > rise_db * 0.6 and i - last > min_gap_s / 0.002:
            onsets.append(i * hop)
            last = i
    return onsets, y


def arrival_times(y, onset):
    """Relative arrival time of each mic (s, mean 0) from GCC-PHAT of all pairs in a window around the onset."""
    a = max(0, onset - int(WIN_PRE_S * FS))
    b = min(y.shape[1], onset + int(WIN_POST_S * FS))
    seg = y[:, a:b] * np.hanning(b - a)
    nfft = 4096
    S = np.fft.rfft(seg, nfft, axis=1)
    f = np.fft.rfftfreq(nfft, 1 / FS)
    band = (f >= BAND[0]) & (f <= BAND[1])
    maxlag = int(MAX_LAG_S * FS)
    rows, vals, peaks = [], [], []
    for i in range(6):
        for j in range(i + 1, 6):
            cr = S[i] * np.conj(S[j])
            cr = np.where(band, cr / (np.abs(cr) + 1e-20), 0)
            r = np.fft.irfft(cr, nfft)
            r = np.concatenate([r[-maxlag:], r[:maxlag + 1]])
            m = int(np.argmax(r))
            d = 0.0
            if 0 < m < len(r) - 1:
                den = r[m - 1] - 2 * r[m] + r[m + 1]
                d = 0.5 * (r[m - 1] - r[m + 1]) / den if den else 0.0
            tau = (m - maxlag + d) / FS          # t_i - t_j
            row = np.zeros(6)
            row[i], row[j] = 1, -1
            w = max(r[m], 1e-3)
            rows.append(row * w)
            vals.append(tau * w)
            peaks.append(r[m] / band.sum())
    rows.append(np.ones(6))
    vals.append(0.0)
    t, *_ = np.linalg.lstsq(np.array(rows), np.array(vals), rcond=None)
    return t, float(np.median(peaks))


def mds(d, dim=3):
    n = d.shape[0]
    j = np.eye(n) - np.ones((n, n)) / n
    b = -0.5 * j @ (d ** 2) @ j
    w, v = np.linalg.eigh(b)
    idx = np.argsort(w)[::-1][:dim]
    return v[:, idx] * np.sqrt(np.maximum(w[idx], 0)), w[np.argsort(w)[::-1]]


def orient(p):
    """Origin = centroid of M1..M3, M1 at azimuth 90 deg, M1-M3 plane = z 0, M2 on the left (x < 0), arms z <= 0."""
    p = p - p[:3].mean(axis=0)
    n = np.cross(p[1] - p[0], p[2] - p[0])
    n /= np.linalg.norm(n) or 1
    y = p[0] - (p[0] @ n) * n
    y /= np.linalg.norm(y) or 1
    x = np.cross(y, n)
    q = p @ np.array([x, y, n]).T
    if q[1, 0] > 0:                 # mirror so that M2 is at x < 0 (210 deg) as in the pinout
        q[:, 0] *= -1
    if q[3:, 2].mean() > 0:         # z sign is not observable: arms below
        q[:, 2] *= -1
    return q


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rec", help="folder with M1.wav..M6.wav (daq_term) or a 6-channel WAV")
    ap.add_argument("--order", default="1,2,3,4,5,6", help="mics in the order you clapped at them")
    ap.add_argument("--rise", type=float, default=15.0, help="clap detection threshold, dB above the floor")
    args = ap.parse_args()
    order = [int(v) - 1 for v in args.order.split(",")]

    x = load(args.rec)
    onsets, y = find_claps(x, rise_db=args.rise)
    print(f"{x.shape[1] / FS:.1f} s, {len(onsets)} claps found")
    if not onsets:
        sys.exit("no claps: clap louder / closer, or lower --rise")

    claps = []
    print("\n clap   time   first   arrival after the first mic [us]  (M1..M6)              quality")
    for k, o in enumerate(onsets):
        t, q = arrival_times(y, o)
        first = int(np.argmin(t))
        rel = (t - t[first]) * 1e6
        claps.append((o, first, t, q))
        print(f" {k + 1:4d} {o / FS:6.2f}s   M{first + 1}    " + " ".join(f"{v:7.0f}" for v in rel) + f"    {q:.2f}")

    # groups of consecutive claps at the same first mic
    groups = []
    for o, first, t, q in claps:
        if groups and groups[-1][0] == first and o - groups[-1][1][-1][0] < 3 * FS:
            groups[-1][1].append((o, t))
        else:
            groups.append((first, [(o, t)]))
    seq = [g[0] + 1 for g in groups]
    print(f"\nnearest mic per group of claps (in time order): {', '.join(f'M{s}' for s in seq)}")
    if len(groups) == len(order):
        bad = [(o + 1, g[0] + 1) for o, g in zip(order, groups) if o != g[0]]
        if bad:
            print("CHANNEL ORDER MISMATCH: " + ", ".join(f"clapped at M{a} but channel M{b} heard it first" for a, b in bad))
            print("  -> either the channel order is wrong (fix DAQ_CHANNEL_MAP in main/config.h or the wiring), or that channel\n"
                  "     has a digital time offset (its sound arrives too early / late whatever the source): repeat the test\n"
                  "     and look at the offset column below")
        else:
            print("channel order OK (each group is heard first by the mic you clapped at)")
    else:
        print(f"({len(groups)} groups, expected {len(order)}: clap 2-3 times at each mic, ~2 s pause between mics)")

    # per mic: mean relative arrival times of claps where it was first
    rel = {}
    for first, items in groups:
        ts = np.array([t - t[first] for _, t in items])
        rel.setdefault(first, []).append(ts.mean(axis=0))
    rel = {k: np.mean(v, axis=0) for k, v in rel.items()}
    have = sorted(rel)
    if len(have) < 2:
        return
    print("\npairwise (both mics clapped): distance and digital offset")
    print("  pair   t(j|clap at i)  t(i|clap at j)    distance    offset o_j - o_i")
    dist = np.full((6, 6), np.nan)
    np.fill_diagonal(dist, 0)
    offsets_ok = True
    for a in have:
        for b in have:
            if b <= a:
                continue
            tab, tba = rel[a][b], rel[b][a]
            dmm = C * (tab + tba) / 2 * 1000
            off = (tab - tba) / 2 * 1e6
            dist[a, b] = dist[b, a] = dmm
            flag = "  <-- not acoustic" if abs(off) > 60 else ""
            offsets_ok &= abs(off) <= 60
            print(f"  M{a + 1}-M{b + 1}  {tab * 1e6:9.0f} us   {tba * 1e6:9.0f} us   {dmm:7.0f} mm   {off:8.0f} us{flag}")
    if not offsets_ok:
        print("Some channels have a time offset that is not acoustic (the delay depends on which side the sound comes"
              " from by more than +-60 us): look for lost / shifted samples in that channel (DMA, slot order).")

    if len(have) == 6 and not np.isnan(dist).any():
        pos, ev = mds(dist / 1000.0)
        pos = orient(pos) * 1000
        print("\nestimated positions [mm] (origin = centre of M1-M3, M1 at 90 deg, arms below):")
        for m in range(6):
            r = np.hypot(pos[m, 0], pos[m, 1])
            az = np.degrees(np.arctan2(pos[m, 1], pos[m, 0])) % 360
            print(f"  M{m + 1}: x {pos[m, 0]:7.1f}  y {pos[m, 1]:7.1f}  z {pos[m, 2]:7.1f}   (r {r:5.0f} mm, az {az:5.0f} deg)")
        fit = np.sqrt(np.nanmean([(np.linalg.norm(pos[i] - pos[j]) - dist[i, j]) ** 2
                                  for i in range(6) for j in range(i + 1, 6)]))
        print(f"  distance fit error {fit:.0f} mm (a few mm is good; tens of mm = claps not close enough to the mics)")
        print("\nconsole commands (idf.py monitor), saved to NVS:")
        for m in range(6):
            print(f"  mic {m + 1} {pos[m, 0]:.1f} {pos[m, 1]:.1f} {pos[m, 2]:.1f}")
    else:
        miss = [f"M{m + 1}" for m in range(6) if m not in have]
        print(f"\nno geometry: no claps heard first by {', '.join(miss) or '-'} (clap at every mic)")


if __name__ == "__main__":
    main()
