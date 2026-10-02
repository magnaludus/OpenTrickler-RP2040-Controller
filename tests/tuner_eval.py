"""Evaluate the live tuner against a static profile across seeds, on the simulated plant.
Run: python tests/tuner_eval.py   (needs `python -m ziglang`, or set CC)
"""
import os, re, subprocess, sys, statistics
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "build_host"
EXE = BUILD / ("tuner_sim.exe" if os.name == "nt" else "tuner_sim")


def compile_sim():
    BUILD.mkdir(exist_ok=True)
    cc = os.environ.get("CC")
    cmd = ([cc] if cc else [sys.executable, "-m", "ziglang", "cc"]) + [
        "-O2", "-Wall", "-Wextra", "-Werror", "-Wno-gnu-folding-constant",
        str(ROOT / "tests/tuner_sim.c"), str(ROOT / "src/learn_tuner.c"), "-I" + str(ROOT / "src"),
        "-lm", "-o", str(EXE)]
    subprocess.run(cmd, check=True)


def sim(tuner, seed, throws, extra):
    out = subprocess.run([str(EXE), "--throws", str(throws), "--tuner", str(tuner), "--seed", str(seed),
                          "--block", str(throws), "--etaf", "0.12", "--lagfsd", "0.17", *extra],
                         capture_output=True, text=True, check=True).stdout
    m = re.search(r"SUMMARY mean_time=([\d.]+) misses=(\d+) overs=(\d+) throws=(\d+)", out)
    last = out.strip().splitlines()[-1]
    return float(m.group(1)), int(m.group(2)), int(m.group(3)), last


def compare_styles(name, extra, seeds=range(1, 9), throws=300):
    def agg(aggr):
        rs = [sim(1, s, throws, extra + ["--aggr", str(aggr)]) for s in seeds]
        return (statistics.mean(r[0] for r in rs), 100.0 * sum(r[1] for r in rs) / (throws * len(rs)))
    normal, aggressive = agg(0), agg(1)
    print(f"{name:26s} normal {normal[0]:5.2f}s {normal[1]:4.1f}%  | aggressive {aggressive[0]:5.2f}s {aggressive[1]:4.1f}%")
    return normal, aggressive


def compare(name, extra, seeds=range(1, 9), throws=300):
    def agg(mode):
        rs = [sim(mode, s, throws, extra) for s in seeds]
        return (statistics.mean(r[0] for r in rs), 100.0 * sum(r[1] for r in rs) / (throws * len(rs)))
    off, old, new = agg(0), agg(3), agg(1)
    print(f"{name:26s} static {off[0]:5.2f}s {off[1]:4.1f}%  | old tuner {old[0]:5.2f}s {old[1]:4.1f}%  | new tuner {new[0]:5.2f}s {new[1]:4.1f}%")
    return off, old, new


if __name__ == "__main__":
    compile_sim()
    compare("NewProfile7 (already good)", [])
    compare("conservative fit", ["--handoff", "2.5", "--fkp", "1.0", "--cmax", "0.9", "--fmin", "0.25", "--ckp", "0.04"])
    compare("very conservative fit", ["--handoff", "4", "--fkp", "0.8", "--cmax", "0.6", "--fmin", "0.18", "--ckp", "0.03"])
