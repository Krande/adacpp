import numpy as np

import adacpp.cad as C


def test_mesh_spike_detects_crows_nest():
    # A compact unit square (2 body triangles) plus one vertex shot far out in +z,
    # referenced by a thin needle triangle — the classic tessellation "crows-nest".
    pos = np.array(
        [0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0.5, 0.5, 40.0],
        dtype=np.float32,
    )
    idx = np.array([0, 1, 2, 0, 2, 3, 0, 1, 4], dtype=np.uint32)
    s = C.mesh_spike_stats(pos, idx, 8.0, 4.0)
    assert s["triangles"] == 3
    assert s["max_spike"] > 4.0  # the outlier vertex is far past the body
    assert s["spike_tris"] >= 1  # the needle triangle touching it


def test_mesh_spike_clean_mesh_scores_low():
    # The same square WITHOUT the spike vertex — a compact body scores ~1 and flags nothing.
    pos = np.array([0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0], dtype=np.float32)
    idx = np.array([0, 1, 2, 0, 2, 3], dtype=np.uint32)
    s = C.mesh_spike_stats(pos, idx, 8.0, 4.0)
    assert s["triangles"] == 2
    assert s["max_spike"] < 4.0
    assert s["spike_tris"] == 0


def test_mesh_spike_stats_matches_mesh_method_on_stream(files_dir):
    # The free function and the Mesh.spike_stats method must agree on a real tessellated geom.
    import pathlib

    ifcs = sorted(pathlib.Path(files_dir).rglob("with_arc_boundary.ifc"))
    if not ifcs:
        return  # fixture not present in this checkout
    st = C.IfcNgeomStream(str(ifcs[0]))
    for nbytes, _meta in st:
        m = C.tessellate_stream(bytes(nbytes), "libtess2", 0.0, 10.0, {}, 1, 0.0)
        idx = np.asarray(m.indices)
        if not len(idx):
            continue
        method = m.spike_stats(8.0, 4.0)
        free = C.mesh_spike_stats(np.asarray(m.positions), idx, 8.0, 4.0)
        assert method == free
        break
