"""Microbenchmark: one load combination over a synthetic element field, numpy vs native vs wasm.

Default size: 131072 elements x 10 result points x 4 components (~40 floats per element, a 20 MiB
stride) and 6 terms, plus the von Mises re-derivation -- one combination of a large shell model.

    python tools/bench_fea.py [--elements N] [--ips N] [--comps N] [--terms N] [--no-wasm]

Reports the median of 7 runs for:
  numpy           in-memory: out = c0*x0; out += ct*xt (float32) + plane_von_mises (float64)
  native-kernel   in-memory: adacpp.fea.combine_strides + adacpp.fea.derive
  numpy-file      the whole case from a baked AFEL file: read strides, combine, derive, write blob
  native-file     adacpp.fea.combine_field (the same, in C++)
  wasm-file       adacpp_fea.combineField under node (WASMFS in-heap; tools/test_fea_wasm.mjs --bench)
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import shutil
import statistics
import subprocess
import tempfile
import time

import numpy as np

from adacpp import fea

ROOT = pathlib.Path(__file__).resolve().parents[1]


def median_ms(fn, reps=7):
    times = []
    for _ in range(reps):
        t0 = time.perf_counter()
        fn()
        times.append((time.perf_counter() - t0) * 1000.0)
    return statistics.median(times)


def von_mises(sx, sy, t):
    sx, sy, t = (np.asarray(a, dtype=float) for a in (sx, sy, t))
    return np.sqrt(sx * sx + sy * sy - sx * sy + 3.0 * t * t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--elements", type=int, default=131072)
    ap.add_argument("--ips", type=int, default=10)
    ap.add_argument("--comps", type=int, default=4)
    ap.add_argument("--terms", type=int, default=6)
    ap.add_argument("--no-wasm", action="store_true")
    a = ap.parse_args()

    ne, nip, nc, nt = a.elements, a.ips, a.comps, a.terms
    rng = np.random.default_rng(0)
    data = (rng.standard_normal((nt, ne, nip, nc)) * 1e3).astype(np.float32)
    factors = np.asarray([1.1 + 0.1 * i for i in range(nt)], dtype=np.float32)
    per = ne * nip * nc
    print(f"{ne} elements x {nip} ips x {nc} comps = {per} floats/stride ({per * 4 / 2**20:.1f} MiB), {nt} terms")

    results = {}
    strides = [data[t] for t in range(nt)]

    def numpy_mem():
        out = factors[0] * strides[0]
        for c, x in zip(factors[1:], strides[1:]):
            out += c * x
        out[..., 3] = von_mises(out[..., 0], out[..., 1], out[..., 2])
        return out

    def native_mem():
        out = fea.combine_strides(strides, factors)
        fea.derive("plane_von_mises", out, [0, 1, 2], [3])
        return out

    assert numpy_mem().tobytes() == native_mem().tobytes()
    results["numpy (in-memory)"] = median_ms(numpy_mem)
    results["native kernel (in-memory)"] = median_ms(native_mem)

    tmp = pathlib.Path(tempfile.mkdtemp(prefix="fea_bench_"))
    try:
        base = tmp / "base.bin"
        fea.write_afel(base, "G-STRESS", "QUAD4", data)
        header = fea.encode_afel_header("G-STRESS", "QUAD4", 1, ne, nip, nc)

        def numpy_file():
            mm = np.memmap(base, dtype=np.float32, mode="r", offset=1024, shape=(nt, ne, nip, nc))
            out = factors[0] * np.array(mm[0])
            for t in range(1, nt):
                out += factors[t] * np.array(mm[t])
            out[..., 3] = von_mises(out[..., 0], out[..., 1], out[..., 2])
            with open(tmp / "np.bin", "wb") as fh:
                fh.write(header)
                fh.write(out.tobytes())

        vm = [{"op": "plane_von_mises", "args": [0, 1, 2], "out": [3]}]

        def native_file():
            return fea.combine_field([base] * nt, list(range(nt)), factors, tmp / "native.bin", derive=vm)

        numpy_file()
        stats = native_file()
        assert (tmp / "np.bin").read_bytes() == (tmp / "native.bin").read_bytes()
        results["numpy (file -> file)"] = median_ms(numpy_file)
        results["native combine_field (file -> file)"] = median_ms(native_file)
        print("native split (last run, ms):", json.dumps(stats["timing_ms"]))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if not a.no_wasm and shutil.which("node"):
        mod = os.environ.get("ADACPP_FEA_WASM") or str(ROOT / "build-wasm-fea" / "wasm_output" / "adacpp_fea.js")
        if pathlib.Path(mod).exists():
            proc = subprocess.run(
                ["node", str(ROOT / "tools" / "test_fea_wasm.mjs"), "--bench", str(ne), str(nip), str(nc), str(nt)],
                capture_output=True,
                check=False,
                text=True,
                env=dict(os.environ, ADACPP_FEA_WASM=mod),
            )
            line = [ln for ln in proc.stdout.splitlines() if ln.startswith("{")]
            if proc.returncode == 0 and line:
                r = json.loads(line[-1])
                results["wasm combineField (node, WASMFS in-heap, file -> file)"] = r["median_ms"]
                print("wasm split (last run, ms):", json.dumps(r["timing_ms"]))
            else:
                print("wasm bench failed:", proc.stdout, proc.stderr)

    for k, v in results.items():
        print(f"  {k:<58s} {v:8.1f} ms")


if __name__ == "__main__":
    main()
