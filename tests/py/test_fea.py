"""adacpp.fea against numpy references, and against the adacpp_fea wasm module.

The references below are the numpy expressions the kernels port, written out verbatim (adapy's
derived_values.py / derived_fields.py / artefacts/fields.py), so a change of operation order on either
side shows up as a bit difference. Where the spec says "bit-identical", the comparison is on the raw
uint32 bits, not a tolerance.
"""

from __future__ import annotations

import json
import os
import pathlib
import shutil
import struct
import subprocess

import numpy as np
import pytest

fea = pytest.importorskip("adacpp.fea")

ROOT = pathlib.Path(__file__).resolve().parents[2]


def bits(a: np.ndarray) -> np.ndarray:
    return np.ascontiguousarray(a, dtype=np.float32).view(np.uint32)


def assert_bits_equal(a, b):
    a = np.asarray(a, dtype=np.float32)
    b = np.asarray(b, dtype=np.float32)
    assert a.shape == b.shape
    diff = np.flatnonzero(bits(a).ravel() != bits(b).ravel())
    assert (
        diff.size == 0
    ), f"{diff.size} values differ, first at {diff[0]}: {a.ravel()[diff[0]]!r} vs {b.ravel()[diff[0]]!r}"


def rand(shape, seed, nan_every=0):
    rng = np.random.default_rng(seed)
    out = (rng.standard_normal(shape) * 10.0 ** rng.uniform(-3, 6, shape)).astype(np.float32)
    if nan_every:
        out.reshape(-1)[::nan_every] = np.nan
    return out


# --- numpy references (verbatim from adapy) -----------------------------------------------------------


def ref_combine(strides, factors):
    c = np.asarray(factors, dtype=np.float32)
    out = c[0] * strides[0]
    for ci, xi in zip(c[1:], strides[1:]):
        out += ci * xi
    return out


def plane_von_mises(sig_x, sig_y, tau_xy):
    sig_x = np.asarray(sig_x, dtype=float)
    sig_y = np.asarray(sig_y, dtype=float)
    tau_xy = np.asarray(tau_xy, dtype=float)
    return np.sqrt(sig_x * sig_x + sig_y * sig_y - sig_x * sig_y + 3.0 * tau_xy * tau_xy)


def plane_principal(sig_x, sig_y, tau_xy):
    sig_x = np.asarray(sig_x, dtype=float)
    sig_y = np.asarray(sig_y, dtype=float)
    tau_xy = np.asarray(tau_xy, dtype=float)
    centre = 0.5 * (sig_x + sig_y)
    radius = np.sqrt((0.5 * (sig_x - sig_y)) ** 2 + tau_xy * tau_xy)
    return np.stack((centre + radius, centre - radius), axis=-1)


def decompose_shell(bottom, top):
    bottom = np.asarray(bottom, dtype=float)
    top = np.asarray(top, dtype=float)
    membrane = 0.5 * (top + bottom)
    bending = 0.5 * (top - bottom)
    mv = plane_von_mises(membrane[..., 0], membrane[..., 1], membrane[..., 2])
    return np.stack(
        (membrane[..., 0], membrane[..., 1], bending[..., 0], bending[..., 1], membrane[..., 2], bending[..., 2], mv),
        axis=-1,
    )


def stress_resultants(decomposed, thickness):
    d = np.asarray(decomposed, dtype=float)
    t = np.asarray(thickness, dtype=float)
    while t.ndim < d.ndim - 1:
        t = t[..., None]
    t2_over_6 = t * t / 6.0
    return np.stack(
        (
            d[..., 0] * t,
            d[..., 4] * t,
            d[..., 1] * t,
            d[..., 5] * t2_over_6,
            d[..., 2] * t2_over_6,
            d[..., 3] * t2_over_6,
        ),
        axis=-1,
    )


def ref_envelope(strides):
    """NaN-skipping max/min with the earliest governing case (np.nanargmax semantics, all-NaN -> 0)."""
    mx, mn = strides[0].copy(), strides[0].copy()
    gx = np.zeros(mx.shape, np.uint16)
    gn = np.zeros(mx.shape, np.uint16)
    for k, x in enumerate(strides[1:], start=1):
        up = (x > mx) | (np.isnan(mx) & ~np.isnan(x))
        down = (x < mn) | (np.isnan(mn) & ~np.isnan(x))
        mx[up], gx[up] = x[up], k
        mn[down], gn[down] = x[down], k
    return mx, mn, gx, gn


def ref_header(name, n_steps, n_points, n_components, elem_type=None, n_ips=None):
    """adapy artefacts/fields.py _encode_blob_header / _encode_elem_field_blob_header."""
    if elem_type is None:
        obj = {
            "name": name,
            "n_steps": n_steps,
            "n_points": n_points,
            "n_components": n_components,
            "dtype": "float32",
            "stride_bytes": n_points * n_components * 4,
        }
        magic = b"AFBL"
    else:
        obj = {
            "name": name,
            "elem_type": elem_type,
            "n_steps": n_steps,
            "n_elements": n_points,
            "n_ips": n_ips,
            "n_components": n_components,
            "dtype": "float32",
            "stride_bytes": n_points * n_ips * n_components * 4,
        }
        magic = b"AFEL"
    js = json.dumps(obj, separators=(",", ":")).encode("utf-8")
    prefix = magic + struct.pack("<II", 1, len(js)) + js
    return prefix + b"\x00" * (1024 - len(prefix))


def ref_stats(arr):
    """artefacts/fields.py FieldBlobWriter.add + finish, for one step."""
    arr = np.asarray(arr, dtype=np.float32).reshape(-1, arr.shape[-1])
    finite = np.isfinite(arr)
    comps = []
    for c in range(arr.shape[1]):
        col = arr[:, c][finite[:, c]]
        comps.append([float(col.min()), float(col.max())] if col.size else [0.0, 0.0])
    mag = [0.0, 0.0]
    if arr.shape[1] >= 3:
        m = np.linalg.norm(arr[:, :3], axis=1)
        m = m[np.isfinite(m)]
        if m.size:
            mag = [float(m.min()), float(m.max())]
    return comps, mag


# --- array kernels --------------------------------------------------------------------------------------


@pytest.mark.parametrize("n_terms", [1, 2, 6])
def test_combine_strides_bit_identical_to_numpy(n_terms):
    strides = [rand((1009, 10, 4), 100 + t, nan_every=97) for t in range(n_terms)]
    factors = [1.2, -1.1, 0.35, 2.0, -0.7, 1e-3][:n_terms]
    assert_bits_equal(fea.combine_strides(strides, factors), ref_combine(strides, factors))


def test_combine_strides_is_not_fused():
    a = np.float32(1.0 + 2.0**-12)
    # a*a rounds before the add: exactly 2**-12. An FMA would give 2**-12 + 2**-24.
    out = fea.combine_strides([np.array([a], np.float32), np.array([a], np.float32)], [a, -1.0])
    assert out[0] == np.float32(2.0**-12)


def test_combine_strides_refuses_float64():
    with pytest.raises(TypeError):
        fea.combine_strides([np.zeros(3)], [1.0])


def test_derive_von_mises_and_principal():
    v = rand((2000, 4), 1)
    fea.derive("plane_von_mises", v, [0, 1, 2], [3])
    assert_bits_equal(v[:, 3], plane_von_mises(v[:, 0], v[:, 1], v[:, 2]).astype(np.float32))

    p = np.empty((2000, 2), np.float32)
    fea.derive("plane_principal", v, [0, 1, 2], [0, 1], out=p)
    assert_bits_equal(p, plane_principal(v[:, 0], v[:, 1], v[:, 2]).astype(np.float32))


def test_derive_magnitude_matches_linalg_norm():
    v = rand((3000, 7), 2)
    m = np.empty((3000, 1), np.float32)
    fea.derive("magnitude", v, [0, 1, 2], [0], out=m)
    values = v.astype(float)  # build_nodal_kinematics: float64 values, then np.linalg.norm
    assert_bits_equal(m[:, 0], np.linalg.norm(values[:, :3], axis=1).astype(np.float32))


def test_derive_shell_decompose_and_resultants():
    bottom, top = rand((500, 4, 3), 3), rand((500, 4, 3), 4)
    stacked = np.concatenate((bottom, top), axis=-1)  # (..., 6): bottom then top
    d = np.empty((500, 4, 7), np.float32)
    fea.derive("shell_decompose", np.ascontiguousarray(stacked), [0, 1, 2, 3, 4, 5], list(range(7)), out=d)
    assert_bits_equal(d, decompose_shell(bottom, top).astype(np.float32))

    thickness = np.linspace(0.008, 0.04, 500)
    r = np.empty((500, 4, 6), np.float32)
    fea.derive("shell_resultants", d, [0, 1, 2, 3, 4, 5], list(range(6)), out=r, thickness=thickness, thickness_rows=4)
    # Tier A re-derives from the stored (float32) D-STRESS, so the reference starts from those too.
    assert_bits_equal(r, stress_resultants(d, thickness[:, None]).astype(np.float32))


def test_derive_validates():
    v = rand((10, 4), 6)
    with pytest.raises(ValueError):
        fea.derive("plane_von_mises", v, [0, 1], [3])
    with pytest.raises(ValueError):
        fea.derive("nope", v, [0], [0])
    with pytest.raises(TypeError):
        fea.derive("copy", v.astype(float), [0], [1])


def test_envelope_matches_reference():
    strides = [rand((777, 3), 10 + k, nan_every=13 + k) for k in range(9)]
    for got, want in zip(fea.envelope(strides), ref_envelope(strides)):
        assert got.dtype == want.dtype
        if got.dtype == np.float32:
            assert_bits_equal(got, want)
        else:
            np.testing.assert_array_equal(got, want)


def test_step_stats_match_blob_writer():
    v = rand((4000, 5), 11, nan_every=31)
    s = fea.step_stats(v)
    comps, mag = ref_stats(v)
    assert s["scalar_range_per_component"] == comps
    assert s["scalar_range_magnitude"] == mag
    two = fea.step_stats(rand((10, 2), 12))
    assert two["scalar_range_magnitude"] == [0.0, 0.0]


# --- artefact format ------------------------------------------------------------------------------------


@pytest.mark.parametrize("name", ["DISPLACEMENT", "G-STRESS.lower", 'we"ird\\naïve\U0001f600\x7f'])
def test_headers_byte_identical(name):
    assert fea.encode_afbl_header(name, 13, 1234, 7) == ref_header(name, 13, 1234, 7)
    assert fea.encode_afel_header(name, "QUAD4", 1, 10, 8, 4) == ref_header(name, 1, 10, 4, elem_type="QUAD4", n_ips=8)


def test_header_too_long_raises():
    with pytest.raises(ValueError):
        fea.encode_afbl_header("x" * 1100, 1, 1, 1)


def test_write_blobs_byte_identical(tmp_path):
    nodal = rand((3, 50, 7), 20)
    fea.write_afbl(tmp_path / "n.bin", "DISPLACEMENT", nodal)
    assert (tmp_path / "n.bin").read_bytes() == ref_header("DISPLACEMENT", 3, 50, 7) + nodal.tobytes()
    elem = rand((2, 30, 8, 4), 21)
    fea.write_afel(tmp_path / "e.bin", "G-STRESS", "QUAD4", elem)
    want = ref_header("G-STRESS", 2, 30, 4, elem_type="QUAD4", n_ips=8) + elem.tobytes()
    assert (tmp_path / "e.bin").read_bytes() == want
    h = fea.read_header(tmp_path / "e.bin")
    assert h["kind"] == "AFEL" and h["n_elements"] == 30 and h["n_ips"] == 8 and h["elem_type"] == "QUAD4"


def _base_blob(tmp_path, n_steps=5):
    data = rand((n_steps, 257, 4, 4), 30, nan_every=101)
    path = tmp_path / "fea.G-STRESS.QUAD4.elements.bin"
    fea.write_afel(path, "G-STRESS", "QUAD4", data)
    return path, data


def test_combine_field_single_step_case(tmp_path):
    path, data = _base_blob(tmp_path)
    steps, factors = [3, 0, 4], [1.2, -1.1, 0.35]
    vm = [{"op": "plane_von_mises", "args": [0, 1, 2], "out": [3]}]
    stats = fea.combine_field([path] * 3, steps, factors, tmp_path / "case.bin", derive=vm)
    want = ref_combine([data[s] for s in steps], factors)
    want[..., 3] = plane_von_mises(want[..., 0], want[..., 1], want[..., 2]).astype(np.float32)
    got = (tmp_path / "case.bin").read_bytes()
    assert got == ref_header("G-STRESS", 1, 257, 4, elem_type="QUAD4", n_ips=4) + want.tobytes()
    comps, mag = ref_stats(want)
    assert stats["ok"] and stats["n_steps"] == 1
    assert stats["steps"][0]["scalar_range_per_component"] == comps
    assert stats["steps"][0]["scalar_range_magnitude"] == mag


def test_combine_field_new_layout(tmp_path):
    path, data = _base_blob(tmp_path)
    fea.combine_field(
        [path, path],
        [1, 2],
        [0.5, 2.0],
        tmp_path / "p.bin",
        derive=[{"op": "plane_principal", "args": [0, 1, 2], "out": [0, 1]}],
        n_components=2,
        name="P-STRESS",
    )
    comb = ref_combine([data[1], data[2]], [0.5, 2.0])
    want = plane_principal(comb[..., 0], comb[..., 1], comb[..., 2]).astype(np.float32)
    header = ref_header("P-STRESS", 1, 257, 2, elem_type="QUAD4", n_ips=4)
    assert (tmp_path / "p.bin").read_bytes() == header + want.tobytes()


def test_envelope_field(tmp_path):
    path, data = _base_blob(tmp_path)
    fea.envelope_field([path] * 5, [0, 1, 2, 3, 4], tmp_path / "env.bin", tmp_path / "env.gov")
    mx, mn, gx, gn = ref_envelope([data[s] for s in range(5)])
    header = ref_header("G-STRESS", 2, 257, 4, elem_type="QUAD4", n_ips=4)
    assert (tmp_path / "env.bin").read_bytes() == header + mx.tobytes() + mn.tobytes()
    # AFGV sidecar, as adapy's combine.write_envelope writes it: 16-byte header, then uint16 [2, *stride].
    gov_header = b"AFGV" + struct.pack("<III", 1, 5, 0)
    gov = (tmp_path / "env.gov").read_bytes()
    assert gov == gov_header + gx.astype("<u2").tobytes() + gn.astype("<u2").tobytes()


def test_manifest_op_names(tmp_path):
    """The lazy-case manifest's single-output op names (adapy artefacts/combine.py DERIVATION_OPS)."""
    v = rand((500, 4), 40)
    p = np.empty((500, 2), np.float32)
    fea.derive("plane_principal", v, [0, 1, 2], [0, 1], out=p)
    q = np.empty((500, 2), np.float32)
    fea.derive("plane_principal_1", v, [0, 1, 2], [0], out=q)
    fea.derive("plane_principal_2", v, [0, 1, 2], [1], out=q)
    assert_bits_equal(q, p)
    m = np.empty((500, 1), np.float32)
    fea.derive("magnitude3", v, [0, 1, 2], [0], out=m)
    assert_bits_equal(m[:, 0], np.linalg.norm(v[:, :3].astype(float), axis=1).astype(np.float32))


def test_matches_adapy_combine_reference_when_available(tmp_path):
    """Against adapy's own Tier A reference (artefacts/combine.py), when it is importable."""
    combine = pytest.importorskip("ada.fem.results.artefacts.combine")
    strides = [rand((300, 8, 4), 50 + t, nan_every=37) for t in range(5)]
    coefficients = [1.2, -1.1, 0.35, 2.0, -0.7]
    assert_bits_equal(fea.combine_strides(strides, coefficients), combine.superpose(strides, coefficients))
    x = rand((2000, 3), 51)
    for name, spec in combine.DERIVATION_OPS.items():
        out = np.empty((2000, 1), np.float32)
        fea.derive(name, x, [0, 1, 2][: spec["inputs"]], [0], out=out)
        assert_bits_equal(out[:, 0], spec["impl"](x[:, 0], x[:, 1], x[:, 2]))


def test_combine_field_errors(tmp_path):
    path, _ = _base_blob(tmp_path)
    with pytest.raises(IndexError):
        fea.combine_field([path], [9], [1.0], tmp_path / "x.bin")
    other = tmp_path / "other.bin"
    fea.write_afbl(other, "DISPLACEMENT", rand((1, 10, 7), 1))
    with pytest.raises(ValueError):
        fea.combine_field([path, other], [0, 0], [1.0, 1.0], tmp_path / "x.bin")


# --- cross-engine: the wasm module writes the same bytes --------------------------------------------------


def _wasm_module():
    explicit = os.environ.get("ADACPP_FEA_WASM")
    path = pathlib.Path(explicit) if explicit else ROOT / "build-wasm-fea" / "wasm_output" / "adacpp_fea.js"
    return path if path.exists() else None


# Resolved once, at collection: other tests in the suite rewrite PATH while they run.
NODE = shutil.which("node")


@pytest.mark.skipif(NODE is None, reason="node not on PATH")
def test_wasm_and_native_write_identical_bytes(tmp_path):
    mod = _wasm_module()
    if mod is None:
        pytest.skip("adacpp_fea wasm module not built (pixi run -e wasm wbuild-fea, or set ADACPP_FEA_WASM)")

    path, _ = _base_blob(tmp_path, n_steps=6)
    disp = tmp_path / "fea.DISPLACEMENT.bin"
    fea.write_afbl(disp, "DISPLACEMENT", rand((3, 1001, 7), 31))
    steps, factors = [5, 0, 2, 3, 1, 4], [1.2, -1.1, 0.35, 2.0, -0.7, 1e-3]
    vm = [{"op": "plane_von_mises", "args": [0, 1, 2], "out": [3]}]
    pstress = {
        "name": "P-STRESS",
        "n_components": 2,
        "ops": [{"op": "plane_principal", "args": [0, 1, 2], "out": [0, 1]}],
    }
    mag = [{"op": "magnitude", "args": [1, 2, 3], "out": [0]}]

    native = tmp_path / "native"
    native.mkdir()
    fea.combine_field([path] * 6, steps, factors, native / "case.bin", derive=vm)
    fea.combine_field([path] * 6, steps, factors, native / "p.bin", pstress["ops"], n_components=2, name="P-STRESS")
    fea.combine_field([disp, disp], [2, 1], [0.5, 2.0], native / "d.bin", derive=mag)
    fea.envelope_field([path] * 6, list(range(6)), native / "env.bin", native / "env.gov")

    wasm = tmp_path / "wasm"
    wasm.mkdir()
    names = ("case.bin", "p.bin", "d.bin", "env.bin", "env.gov")
    base6 = ["/job/base.bin"] * 6
    job = {
        "inputs": {"/job/base.bin": str(path), "/job/disp.bin": str(disp)},
        "ops": [
            {
                "verb": "combine",
                "paths": base6,
                "steps": steps,
                "factors": factors,
                "out": "/job/case.bin",
                "derive": vm,
            },
            {
                "verb": "combine",
                "paths": base6,
                "steps": steps,
                "factors": factors,
                "out": "/job/p.bin",
                "derive": pstress,
            },
            {
                "verb": "combine",
                "paths": ["/job/disp.bin"] * 2,
                "steps": [2, 1],
                "factors": [0.5, 2.0],
                "out": "/job/d.bin",
                "derive": mag,
            },
            {"verb": "envelope", "paths": base6, "steps": list(range(6)), "out": "/job/env.bin", "gov": "/job/env.gov"},
        ],
        "outputs": {f"/job/{n}": str(wasm / n) for n in names},
    }
    job_path = tmp_path / "job.json"
    job_path.write_text(json.dumps(job))
    proc = subprocess.run(
        [NODE, str(ROOT / "tools" / "test_fea_wasm.mjs"), "--job", str(job_path)],
        capture_output=True,
        check=False,
        text=True,
        env=dict(os.environ, ADACPP_FEA_WASM=str(mod)),
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    for n in names:
        assert (wasm / n).read_bytes() == (native / n).read_bytes(), f"{n}: wasm and native bytes differ"
