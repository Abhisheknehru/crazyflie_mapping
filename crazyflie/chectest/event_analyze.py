#!/usr/bin/env python3
"""Characterise a wall-change event from a recorded trial CSV.

Direction-agnostic. For each face it finds the LARGEST SUSTAINED RUN: the
biggest cumulative distance change over a stretch of consecutive same-direction
samples (tiny noise wiggles are ignored, not counted as direction changes).
That run is the real size of the event -- unlike the start-to-end difference,
it survives the drone drifting back to where it began, and unlike a single
sample step it captures the whole move.

The face with the biggest run is the trigger. Each face is reported with a
plain-language label (moved CLOSER / moved FARTHER / steady), its magnitude in
metres, and the event duration is the time that winning run took.

Usage:
    python3 event_analyze.py trial1.csv trial2.csv trial3.csv

CSV columns expected: t,front,left,back,right  (as written by event_recorder.py)
"""
import csv
import statistics
import sys

FACES = ('front', 'left', 'back', 'right')

SMOOTH = 5           # moving-average width: tames per-sample jitter so a slow
                     # ramp's small real steps aren't lost in the noise.
NOISE_TOL = 0.001    # 1 mm: residual jitter below this doesn't break a run.
EVENT_MIN = 0.05     # 5 cm: runs below this are reported as "steady".


def load(path):
    t, cols = [], {f: [] for f in FACES}
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row['t']))
            for k in cols:
                cols[k].append(float(row[k]))
    return t, cols


def diffs(series):
    return [b - a for a, b in zip(series, series[1:])]


def smooth(series, w):
    if w <= 1:
        return list(series)
    out, half = [], w // 2
    for i in range(len(series)):
        lo, hi = max(0, i - half), min(len(series), i + half + 1)
        out.append(sum(series[lo:hi]) / (hi - lo))
    return out


def largest_run(series):
    """Biggest cumulative change over consecutive same-direction steps.

    Returns (signed_magnitude, i_start, i_end) where i_start/i_end index into
    `series`. The series is smoothed first; steps under NOISE_TOL are skipped
    so jitter doesn't end a run.
    """
    best = (0.0, 0, 0)
    run, sign, start = 0.0, 0, 0
    for i, d in enumerate(diffs(smooth(series, SMOOTH))):
        if abs(d) < NOISE_TOL:
            continue
        s = 1 if d > 0 else -1
        if s == sign:
            run += d
        else:
            sign, run, start = s, d, i
        if abs(run) > abs(best[0]):
            best = (run, start, i + 1)   # diff i spans samples i..i+1
    return best


def label(face, signed):
    """Plain-language meaning of a sustained range change on one face.

    Range DECREASING => the wall got nearer (came closer / appeared);
    range INCREASING => it moved away (receded / disappeared).
    """
    if abs(signed) < EVENT_MIN:
        return f"{face.capitalize():5} wall: steady ({signed:+.3f} m)"
    verb = "moved CLOSER" if signed < 0 else "moved FARTHER"
    return f"{face.capitalize():5} wall {verb} by {abs(signed):.3f} m"


def analyze(path):
    t, cols = load(path)
    if len(t) < 3:
        print(f"{path}: too few samples ({len(t)})")
        return None

    runs = {f: largest_run(cols[f]) for f in FACES}
    trigger = max(FACES, key=lambda f: abs(runs[f][0]))

    mag, i0, i1 = runs[trigger]
    feats = {'trigger': trigger, 'duration': t[i1] - t[i0]}
    for f in FACES:
        feats[f'change_{f}'] = runs[f][0]
    return feats


def main():
    paths = sys.argv[1:]
    if not paths:
        print(__doc__)
        sys.exit(1)

    results = []
    for p in paths:
        r = analyze(p)
        if r is None:
            continue
        results.append(r)
        trig = r['trigger']
        print(f"\n=== {p} ===")
        if abs(r['change_' + trig]) < EVENT_MIN:
            print("  No clear event (all faces steady).")
        print(f"  Trigger sensor      : {trig.upper()} "
              f"(largest change: {r['change_' + trig]:+.3f} m)")
        print(f"  Duration            : {r['duration']:.3f} s")
        for face in FACES:
            print("  " + label(face, r['change_' + face]))

    if len(results) > 1:
        print("\n=== Repeatability across trials ===")
        triggers = {r['trigger'] for r in results}
        consistent = (next(iter(triggers)).upper() if len(triggers) == 1
                      else 'VARIES: ' + ','.join(triggers))
        print(f"  Trigger sensor      : {consistent}")
        durs = [r['duration'] for r in results]
        print(f"  Duration            : mean {statistics.mean(durs):+.3f}, "
              f"std {statistics.pstdev(durs):.3f} "
              f"(min {min(durs):+.3f}, max {max(durs):+.3f})")
        for face in FACES:
            vals = [r['change_' + face] for r in results]
            print(f"  change {face.capitalize():13}: "
                  f"mean {statistics.mean(vals):+.3f}, "
                  f"std {statistics.pstdev(vals):.3f}")
        print("\n  Low std on the trigger => Repeatable: Yes / Confidence: High")


if __name__ == '__main__':
    main()
