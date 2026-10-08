"""Format-neutral FEA result kernels (load combinations, derived components, envelopes, AFBL/AFEL).

The same C++ backs the ``adacpp_fea`` wasm module, so everything here is bit-identical to what a
browser computes from the same baked strides:

- :func:`combine_strides` -- Tier A superposition, ``out = c0*x0`` then ``out += ct*xt`` in float32,
  terms in the given order, no FMA. Bit-identical to the numpy expression on float32 arrays with
  float32 factors.
- :func:`derive` -- re-derive the non-linear components (``plane_von_mises``, ``plane_principal``,
  ``magnitude``, ``shell_decompose``, ``shell_resultants``, ``copy``) from superposed linear ones, with
  the operation order of adapy's ``derived_values.py`` (double precision, rounded to float32 once).
- :func:`envelope` -- element-wise max / min over cases with the governing case (uint16).
- :func:`combine_field` / :func:`envelope_field` -- the file verbs: read strides from baked AFBL/AFEL
  blobs and write a single-step case blob (or a two-step max/min envelope blob), byte-identical to
  the wasm module's ``combineField`` / ``envelopeField``.
- :func:`write_afbl` / :func:`write_afel` / :func:`encode_afbl_header` / :func:`encode_afel_header` /
  :func:`read_header` -- the artefact format, byte-identical to adapy's writers.

The array kernels take float32 arrays and REFUSE anything else rather than converting silently: a
float64 input rounded on the way in would no longer be the stored stride.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from ._ada_cpp_ext_impl import fea as _fea

BLOB_HEADER_BYTES = _fea.BLOB_HEADER_BYTES
DERIVE_OPS = _fea.DERIVE_OPS

encode_afbl_header = _fea.encode_afbl_header
encode_afel_header = _fea.encode_afel_header


def read_header(path) -> dict:
    """The AFBL/AFEL JSON header of the blob at ``path`` as a dict, plus ``kind``."""
    return _fea.read_header(str(path))


def _f32(a, what: str) -> np.ndarray:
    arr = np.asarray(a)
    if arr.dtype != np.float32:
        raise TypeError(f"{what} must be float32, got {arr.dtype}")
    return np.ascontiguousarray(arr)


def _factors(factors) -> list[float]:
    # Round to float32 once (np.float32(x)), then hand the exact float32 values over as Python floats.
    return [float(x) for x in np.asarray(factors, dtype=np.float32).reshape(-1)]


def _steps(steps) -> list[int]:
    return [int(steps)] if np.ndim(steps) == 0 else [int(s) for s in steps]


def combine_strides(inputs: Sequence[np.ndarray], factors) -> np.ndarray:
    """``sum(factors[t] * inputs[t])`` in float32, in term order; a new array shaped like ``inputs[0]``."""
    arrays = [_f32(a, "combine_strides inputs") for a in inputs]
    return _fea.combine_strides(arrays, _factors(factors))


def derive(
    op: str,
    values: np.ndarray,
    args: Sequence[int],
    out_cols: Sequence[int],
    out: np.ndarray | None = None,
    thickness: np.ndarray | None = None,
    thickness_rows: int = 1,
) -> np.ndarray:
    """Apply one derivation op over ``values`` (..., n_components) float32.

    Writes the op's outputs to columns ``out_cols`` (-1 drops one) of ``out`` -- or of ``values``
    itself when ``out`` is None -- and returns that array. ``thickness`` (float64, one value per
    ``thickness_rows`` rows) is only used by ``shell_resultants``.
    """
    if not isinstance(values, np.ndarray) or values.dtype != np.float32 or not values.flags.c_contiguous:
        raise TypeError("derive: values must be a C-contiguous float32 numpy array")
    if out is not None and (out.dtype != np.float32 or not out.flags.c_contiguous):
        raise TypeError("derive: out must be a C-contiguous float32 numpy array")
    th = None if thickness is None else np.ascontiguousarray(thickness, dtype=np.float64)
    _fea.derive(op, values, [int(a) for a in args], [int(o) for o in out_cols], out, th, int(thickness_rows))
    return values if out is None else out


def envelope(inputs: Sequence[np.ndarray]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """``(max, min, governing_max, governing_min)`` over the cases in ``inputs`` (NaN skipped, ties keep
    the earlier case, governing indices uint16)."""
    return _fea.envelope([_f32(a, "envelope inputs") for a in inputs])


def step_stats(values: np.ndarray) -> dict:
    """Per-step scalar ranges of ``values`` (..., n_components), as the artefact writers record them."""
    return _fea.step_stats(_f32(values, "step_stats values"))


def write_afbl(path, name: str, data: np.ndarray) -> None:
    """Write an AFBL blob from ``(n_steps, n_points, n_components)`` float32."""
    _fea.write_afbl(str(path), name, _f32(data, "write_afbl data"))


def write_afel(path, name: str, elem_type: str, data: np.ndarray) -> None:
    """Write an AFEL blob from ``(n_steps, n_elements, n_ips, n_components)`` float32."""
    _fea.write_afel(str(path), name, elem_type, _f32(data, "write_afel data"))


def combine_field(
    paths: Sequence,
    steps,
    factors,
    out_path,
    derive: Sequence[dict] | None = None,
    n_components: int = 0,
    name: str = "",
) -> dict:
    """Materialise one load combination from baked strides into a single-step AFBL/AFEL blob.

    Term ``t`` is step ``steps[t]`` of the blob at ``paths[t]`` (``steps`` may be a single int for
    every path). ``derive`` is a list of ``{"op", "args", "out"}`` dicts applied in order after
    superposition: in place, or into a new ``n_components``-column layout when ``n_components > 0``.
    Returns the stats dict (per-step scalar ranges, timings).
    """
    ops = [
        {"op": str(d["op"]), "args": [int(a) for a in d["args"]], "out": [int(o) for o in d["out"]]}
        for d in derive or []
    ]
    return _fea.combine_field(
        [str(p) for p in paths], _steps(steps), _factors(factors), str(out_path), ops, int(n_components), name
    )


def envelope_field(paths: Sequence, steps, out_path, gov_path="") -> dict:
    """Envelope over cases: a two-step (max, min) blob at ``out_path`` and, when ``gov_path`` is given,
    the governing case indices as an AFGV sidecar (16-byte header ``AFGV``, uint32 version 1, uint32
    n_cases, uint32 0; then little-endian uint16 ``[2, rows, n_components]``) -- the layout of adapy's
    ``write_envelope``."""
    return _fea.envelope_field([str(p) for p in paths], _steps(steps), str(out_path), str(gov_path) if gov_path else "")


__all__ = [
    "BLOB_HEADER_BYTES",
    "DERIVE_OPS",
    "combine_field",
    "combine_strides",
    "derive",
    "encode_afbl_header",
    "encode_afel_header",
    "envelope",
    "envelope_field",
    "read_header",
    "step_stats",
    "write_afbl",
    "write_afel",
]
