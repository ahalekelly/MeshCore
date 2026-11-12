# /// script
# requires-python = ">=3.12"
# ///
"""Summarise bursttest.py output: uv run burststats.py PREFIX

Prints lock counts per (bit, arm) and, per locked burst, when the RSSI froze: the last
sample below -90 dBm before the trace stays at or above -95 dBm to the lock decision 1.5 s after the carrier (ms after the
carrier command; 'onset' if within 20 ms of the carrier's onset), with the carrier onset and the AGC toggle count (dips below -90) before that.
"""
import csv
import sys
from collections import defaultdict
from math import comb

prefix = sys.argv[1]
bursts = list(csv.DictReader(open(f"{prefix}-bursts.csv")))
traces = defaultdict(list)
for r in csv.DictReader(open(f"{prefix}-traces.csv")):
    traces[int(r["burst"])].append((int(r["t_ms"]), float(r["dbm"]) if r["dbm"] else None))

def interval(k, n):
    lo = 0.0 if k == 0 else beta_quantile(0.025, k, n - k + 1)
    hi = 1.0 if k == n else beta_quantile(0.975, k + 1, n - k)
    return lo, hi


def beta_quantile(q, a, b):
    lo, hi = 0.0, 1.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if beta_cdf(mid, a, b) < q:
            lo = mid
        else:
            hi = mid
    return lo


def beta_cdf(x, a, b):
    # integer a, b: P(Binomial(a + b - 1, x) >= a)
    n = a + b - 1
    return sum(comb(n, j) * x**j * (1 - x) ** (n - j) for j in range(a, n + 1))


def needed_fix(b):
    return b["cure"] in ("rx", "agc", "FAILED")


def is_lock(b):
    return needed_fix(b) and b["reg_8bf"] == "0812"


counts = defaultdict(lambda: [0, 0, 0, 0])
for b in bursts:
    key = (b["bit"], b["arm"] or "none", b["burst_ms"], b["khz"], b["dbm"])
    counts[key][0] += 1
    counts[key][1] += is_lock(b)
    counts[key][2] += b["cure"] == "self"
    counts[key][3] += needed_fix(b) and not is_lock(b)
for (bit, arm, ms, khz, dbm), (n, k, selfclear, other) in sorted(counts.items(), key=lambda kv: (kv[0][:4], int(kv[0][4]))):
    lo, hi = interval(k, n)
    print(f"bit {bit} arm {arm} {ms} ms {khz} kHz {dbm} dBm: {k}/{n} locked ({k / n:.2%}, 95% CI {lo:.2%}-{hi:.2%}), "
          f"{k / (n * int(ms) / 1000):.3f} per carrier-second, {selfclear} self-clearing, {other} other")

timing = defaultdict(lambda: {"onset": 0, "mid": 0})
for b in bursts:
    if not is_lock(b):
        continue
    i, carrier_end = int(b["burst"]), int(b["tx_reply_ms"])
    trace = [(t, v) for t, v in traces[i] if v is not None and t <= carrier_end + 1500]
    stuck = len(trace)
    while stuck > 0 and trace[stuck - 1][1] >= -95:
        stuck -= 1
    freeze = trace[stuck - 1][0] if stuck else None
    dips = sum(1 for (t0, a), (t1, c) in zip(trace, trace[1:]) if a >= -90 and c < -90 and t1 <= (freeze or 0))
    when = "onset" if freeze is None or freeze - float(b["onset_ms"]) <= 20 else "mid"
    timing[(b["bit"], b["burst_ms"], b["khz"], b["dbm"])][when] += 1
    print(f"{when:5} burst {i} bit {b['bit']} arm {b['arm'] or 'none'} {b['arm_reply']}: onset {b['onset_ms']} ms, froze after {freeze} ms "
          f"(carrier ends ~{carrier_end} ms), {dips} dips before, victim_since {b['victim_since_ms']} ms, "
          f"ref_since {b['ref_since_ms']} ref_next {b['ref_next_ms']}, regs {b['reg_8bf']}, cure {b['cure']}")
for (bit, ms, khz, dbm), t in sorted(timing.items()):
    print(f"bit {bit} {ms} ms {khz} kHz {dbm} dBm: froze at onset (within 20 ms) {t['onset']}, mid-carrier {t['mid']}")
