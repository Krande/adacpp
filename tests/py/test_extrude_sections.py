"""Section tables round-tripped through arrays, and sweeps without analysis data.

Two things a caller needs that building a section in place does not cover:
replaying a table it stored earlier, and sweeping profiles for ordinary display
geometry, where the node pairing and axial parameter an analysis result carries
are noise.
"""

import math

import numpy as np
import pytest

cad = pytest.importorskip("adacpp.cad")


def _rect(w, h):
    return [(-w, -h), (w, -h), (w, h), (-w, h)]


def _frames(n, length=1.0, spacing=0.5):
    """n instances swept along +x, stacked in z."""
    return {
        "label": np.arange(n, dtype=np.uint32),
        "section_idx": np.zeros(n, dtype=np.uint32),
        "origin": np.array([[0.0, 0.0, spacing * k] for k in range(n)]),
        "xvec": np.tile(np.array([[1.0, 0.0, 0.0]]), (n, 1)),
        "yvec": np.tile(np.array([[0.0, 1.0, 0.0]]), (n, 1)),
        "length": np.full(n, length),
    }


def _volume(positions, indices):
    """Signed volume by the divergence theorem -- positive only for a closed,
    outward-wound surface."""
    p = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
    t = np.asarray(indices, dtype=np.int64).reshape(-1, 3)
    a, b, c = p[t[:, 0]], p[t[:, 1]], p[t[:, 2]]
    return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)


def test_a_section_survives_a_round_trip_through_arrays():
    built = cad.build_extruded_section([_rect(0.15, 0.30)])
    assert built.ok

    replayed = cad.make_extruded_section(np.asarray(built.points), np.asarray(built.triangles))
    assert replayed.ok
    assert np.array_equal(np.asarray(replayed.points), np.asarray(built.points))
    assert np.array_equal(np.asarray(replayed.triangles), np.asarray(built.triangles))

    # And it sweeps to the same solid, which is the only reason to round-trip it.
    f = _frames(1, length=3.0)
    out_built = cad.expand_beam_solids([built], **f)
    out_replayed = cad.expand_beam_solids([replayed], **f)
    assert np.array_equal(np.asarray(out_built.positions), np.asarray(out_replayed.positions))
    assert np.array_equal(np.asarray(out_built.indices), np.asarray(out_replayed.indices))


def test_a_replayed_table_still_has_to_be_in_range():
    """A stored table is untrusted input: an index past the swept vertices would
    read off the end of the output buffer, so it is refused here instead."""

    points = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    # 3 points -> 6 swept vertices, so 6 is one past the end.
    bad = np.array([[0, 1, 6]], dtype=np.uint32)
    with pytest.raises(ValueError):
        cad.make_extruded_section(points, bad)


def test_sweeping_without_nodes_still_produces_the_solid():
    sec = cad.build_extruded_section([_rect(0.2, 0.2)])
    out = cad.expand_beam_solids([sec], **_frames(3, length=2.0))

    assert len(out.range_label) == 3
    assert len(out.positions) == 3 * 8 * 3
    # 0.4 x 0.4 swept 2.0, three times.
    assert math.isclose(_volume(out.positions, out.indices), 0.4 * 0.4 * 2.0 * 3, rel_tol=1e-5)

    # Nothing was asked about nodes, so nothing is reported about them.
    assert len(out.t) == 0
    assert len(out.node0) == 0 and len(out.node1) == 0


def test_node_data_is_all_or_nothing():
    sec = cad.build_extruded_section([_rect(0.2, 0.2)])
    f = _frames(1)
    with pytest.raises(ValueError):
        cad.expand_beam_solids([sec], **f, node0=np.zeros(1, dtype=np.uint32))


def test_with_nodes_the_axial_parameter_is_reported():
    sec = cad.build_extruded_section([_rect(0.1, 0.1)])
    f = _frames(1, length=4.0)
    points = np.array([[0.0, 0.0, 0.0], [4.0, 0.0, 0.0]])

    out = cad.expand_beam_solids(
        [sec],
        **f,
        node0=np.zeros(1, dtype=np.uint32),
        node1=np.ones(1, dtype=np.uint32),
        points=points,
    )

    n = sec.n_points
    t = np.asarray(out.t)
    assert len(t) == 2 * n
    # The nodes lie on the sweep axis here, so each ring reads a single value.
    assert np.allclose(t[:n], 0.0, atol=1e-6)
    assert np.allclose(t[n:], 1.0, atol=1e-6)
    assert np.array_equal(np.asarray(out.node0), np.zeros(2 * n, dtype=np.uint32))
    assert np.array_equal(np.asarray(out.node1), np.ones(2 * n, dtype=np.uint32))
