#!/usr/bin/env python3
"""Compare two consumers that watched the same stream, frame by frame.

Absolute stamp-to-arrival latency is not usable here: the RealSense frame stamp and the consumer's
clock are different domains, so it can even come out negative. Pairing the two consumers' rows by
stamp_ns removes the stamp from the comparison entirely -- what is left is how much later one
transport delivered the same frame than the other, plus each path's delivery jitter.
"""
import csv
import statistics
import sys


def load(path):
    rows = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            rows[int(r["stamp_ns"])] = (int(r["arrival_ns"]), float(r["prep_us"]), float(r["kernel_us"]))
    return rows


def pct(values, p):
    values = sorted(values)
    return values[min(len(values) - 1, int(len(values) * p))]


def jitter_ms(rows):
    arrivals = [a for _, (a, _, _) in sorted(rows.items())]
    gaps = [(b - a) / 1e6 for a, b in zip(arrivals, arrivals[1:])]
    return gaps


def main(nitros_path, cpu_path):
    nitros, cpu = load(nitros_path), load(cpu_path)
    common = sorted(set(nitros) & set(cpu))
    print(f"PAIRED frames: nitros={len(nitros)} cpu={len(cpu)} common={len(common)}")
    if not common:
        print("PAIRED no frames in common -- cannot compare")
        return
    deltas = [(nitros[s][0] - cpu[s][0]) / 1e6 for s in common]
    print(
        "PAIRED nitros-minus-cpu arrival delta (ms): "
        f"mean {statistics.mean(deltas):+.2f}  p50 {pct(deltas, 0.5):+.2f}  "
        f"p99 {pct(deltas, 0.99):+.2f}   (negative = NITROS delivered it first)"
    )
    for name, rows in (("nitros", nitros), ("cpu", cpu)):
        gaps = jitter_ms(rows)
        if gaps:
            print(
                f"PAIRED {name} inter-arrival gap (ms): mean {statistics.mean(gaps):.2f}  "
                f"p50 {pct(gaps, 0.5):.2f}  p99 {pct(gaps, 0.99):.2f}  max {max(gaps):.2f}"
            )


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
