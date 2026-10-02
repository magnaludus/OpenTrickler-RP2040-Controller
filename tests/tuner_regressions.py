"""Regression tests for the live tuner (src/learn_tuner.c) on the simulated tube (tests/tuner_sim.c).

Run: python tests/tuner_regressions.py        (CC="python -m ziglang cc" to pick a compiler)

Three kinds of check:
  * Invariants. Whatever the plant does, no knob may leave its allowed range and nothing may go
    NaN. The simulator asserts this after every throw; here it is fuzzed over random tubes and
    random starting profiles.
  * Outcomes. On the bench-calibrated plant the new tuner must beat the tuner it replaced, must not
    spend accuracy to do it, and must pull misses back after the powder changes under it.
  * Determinism. Same seed, same answer.

The plant is a model, not the hardware: these numbers show the control logic behaves, not that a
particular powder will gain a particular number of seconds.
"""
import random
import re
import statistics
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tuner_eval as ev  # noqa: E402

CONSERVATIVE = ["--handoff", "2.5", "--fkp", "1.0", "--cmax", "0.9", "--fmin", "0.25", "--ckp", "0.04"]
SEEDS = range(1, 9)


def agg(mode, extra, throws=300):
    rows = [ev.sim(mode, s, throws, extra) for s in SEEDS]
    return statistics.mean(r[0] for r in rows), 100.0 * sum(r[1] for r in rows) / (throws * len(rows))


def test_invariants_fuzz():
    rng = random.Random(20251001)
    for case in range(40):
        extra = [
            "--kc", f"{rng.uniform(4, 20):.2f}", "--kf", f"{rng.uniform(0.15, 0.6):.3f}",
            "--lagc", f"{rng.uniform(0.3, 0.9):.2f}", "--lagf", f"{rng.uniform(0.4, 1.1):.2f}",
            "--etaf", f"{rng.uniform(0.02, 0.25):.3f}", "--etac", f"{rng.uniform(0.01, 0.08):.3f}",
            "--handoff", f"{rng.uniform(0.4, 6):.2f}", "--fkp", f"{rng.uniform(0.6, 3):.2f}",
            "--cmax", f"{rng.uniform(0.5, 2.5):.2f}", "--ckp", f"{rng.uniform(0.03, 0.6):.3f}",
            "--fmin", f"{rng.uniform(0.15, 0.8):.2f}", "--target", f"{rng.choice([10, 26.5, 42.5, 60]):.1f}",
        ]
        out = subprocess.run([str(ev.EXE), "--throws", "250", "--tuner", "1", "--seed", str(case + 100),
                              "--block", "250", *extra], capture_output=True, text=True)
        assert out.returncode == 0, f"fuzz case {case} {extra}\n{out.stdout[-400:]}"
        assert "nan" not in out.stdout.lower(), f"NaN in fuzz case {case}"
    print("invariants over 40 random plants and profiles: passed")


def test_beats_the_old_tuner():
    for name, extra, max_time_vs_old in [("conservative", CONSERVATIVE, 0.97),
                                         ("very conservative",
                                          ["--handoff", "4", "--fkp", "0.8", "--cmax", "0.6", "--fmin", "0.18", "--ckp", "0.03"], 0.92),
                                         ("already good", [], 1.00)]:
        static, old, new = agg(0, extra), agg(3, extra), agg(1, extra)
        print(f"  {name:18s} static {static[0]:5.2f}s {static[1]:4.1f}% | old {old[0]:5.2f}s {old[1]:4.1f}% | new {new[0]:5.2f}s {new[1]:4.1f}%")
        assert new[0] <= old[0] * max_time_vs_old, f"{name}: new tuner no faster than the one it replaced"
        # Never buy speed with accuracy beyond what the Landing Sigma allows for
        assert new[1] <= 7.0, f"{name}: miss rate {new[1]:.1f}% too high"
    print("new tuner faster than the old one, misses held: passed")


def test_conservative_start_gains():
    static, new = agg(0, CONSERVATIVE), agg(1, CONSERVATIVE)
    assert new[0] < static[0] * 0.92, "a conservative fit should get at least 8% faster"
    print("conservative fit speeds up >= 8%: passed")


def test_recovers_from_drift():
    def last_block_misses(mode):
        total = 0
        for s in SEEDS:
            out = subprocess.run([str(ev.EXE), "--throws", "400", "--tuner", str(mode), "--seed", str(s), "--block", "100",
                                  "--etaf", "0.12", "--lagfsd", "0.17", "--drift", "200"],
                                 capture_output=True, text=True, check=True).stdout
            rows = [l for l in out.splitlines() if re.match(r"\s*\d+\s+\d", l)]
            total += int(re.search(r"(\d+)/\d+", rows[-1]).group(1))
        return total / len(SEEDS)
    static, tuned = last_block_misses(0), last_block_misses(1)
    print(f"  misses per 100 throws after drift: static {static:.1f}, tuned {tuned:.1f}")
    assert tuned < 0.6 * static, "tuner should claw back most of the misses a plant change causes"
    print("recovers from a mid-run plant change: passed")


def test_unit_edge_cases():
    exe = ev.BUILD / ("tuner_unit.exe" if ev.os.name == "nt" else "tuner_unit")
    cc = ev.os.environ.get("CC")
    cmd = ([cc] if cc else [sys.executable, "-m", "ziglang", "cc"]) + [
        "-std=c11", "-Wall", "-Wextra", "-Werror", "-fsanitize=undefined", "-fno-sanitize-recover=all",
        str(ev.ROOT / "tests/tuner_unit.c"), str(ev.ROOT / "src/learn_tuner.c"), "-I" + str(ev.ROOT / "src"),
        "-lm", "-o", str(exe)]
    subprocess.run(cmd, check=True)
    subprocess.run([str(exe)], check=True)


def test_deterministic():
    a = ev.sim(1, 3, 120, [])
    b = ev.sim(1, 3, 120, [])
    assert a == b
    print("deterministic: passed")


if __name__ == "__main__":
    ev.compile_sim()
    test_unit_edge_cases()
    test_deterministic()
    test_invariants_fuzz()
    test_beats_the_old_tuner()
    test_conservative_start_gains()
    test_recovers_from_drift()
    print("all tuner regressions passed")
