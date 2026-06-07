import random

import pytest

import adacpp


def create_box_grid(grid_size):
    # Randomized box sizes
    min_size, max_size = 0.5, 1.0
    origins = []
    dimensions = []

    for x in range(grid_size):
        for y in range(grid_size):
            for z in range(grid_size):
                width = random.uniform(min_size, max_size)
                height = random.uniform(min_size, max_size)
                depth = random.uniform(min_size, max_size)
                origins.append((x * max_size, y * max_size, z * max_size))
                dimensions.append((width, height, depth))

    return origins, dimensions


def test_basic_write_step(tmp_path):
    adacpp.cadit.write_box_to_step(str(tmp_path / "mybox.stp"), (0, 0, 0), (10, 10, 10))


def test_convert_step_to_glb(files_dir, tmp_path):
    adacpp.cadit.conversion.stp_to_glb(
        str(files_dir / "flat_plate_abaqus_10x10_m_wColors.stp"),
        str(tmp_path / "flat_plate_abaqus_10x10_m_wColors.glb"),
    )


def test_basic_write_list_of_boxes_to_step(tmp_path):
    origins, dimensions = create_box_grid(10)
    adacpp.cadit.write_boxes_to_step(str(tmp_path / "myboxes.stp"), origins, dimensions)

    adacpp.cadit.conversion.stp_to_glb(str(tmp_path / "myboxes.stp"), str(tmp_path / "myboxes_convert.glb"))


def test_basic_mesh(tmp_path):
    mesh = adacpp.visit.get_box_mesh((0, 0, 0), (10, 10, 10))
    assert mesh.__class__.__name__ == "Mesh"
    assert hasattr(mesh, "positions")
    assert hasattr(mesh, "normals")
    assert hasattr(mesh, "indices")

    adacpp.cadit.write_mesh_to_gltf(str(tmp_path / "mybox.gltf"), mesh)
    print(mesh)


def test_boxes_mesh_gltf(tmp_path):
    origins, dimensions = create_box_grid(2)
    adacpp.cadit.write_boxes_to_gltf(str(tmp_path / "myboxes.glb"), origins, dimensions)


def test_simple_gmsh(tmp_path):
    adacpp.fem.create_gmesh(str(tmp_path / "my_fem_mesh.msh"))


def test_tess_factory():
    tess_factory = adacpp.visit.TessellateFactory()
    num = 50
    boxes = []
    for x in range(num):
        for y in range(num):
            for z in range(num):
                boxes.append(adacpp.geom.Box((x * 10, y * 10, z * 10), 10, 10, 10))
    box0 = boxes[0]
    assert box0.id == 0
    assert tess_factory.algorithm == adacpp.visit.TessellationAlgorithm.OCCT_DEFAULT
    tess_factory.tessellate()


def test_ifc_file_read(files_dir):
    # Regression: previously segfaulted because we compiled against ifcopenshell
    # headers WITHOUT IFOPSH_WITH_ROCKSDB while linking a libIfcParse.a built
    # WITH it (conda -DWITH_ROCKSDB=ON). That macro gates members of IfcFile, so
    # the layouts disagreed (sizeof 688 vs 896) and the ctor smashed our stack.
    # Fixed by defining IFOPSH_WITH_ROCKSDB in cmake/deps_ifc.cmake.
    assert adacpp.cadit.ifc.read_ifc_file(str(files_dir / "my_test.ifc")) == 0


def test_cad_tessellate_box():
    mesh = adacpp.cad.tessellate_box(2.0, 3.0, 4.0)
    positions = list(mesh.positions)
    assert len(positions) % 3 == 0
    assert len(mesh.indices) % 3 == 0
    assert len(mesh.indices) >= 12 * 3  # at least 12 triangles for a 6-face box

    # Centered axis-aligned box: AABB should be (±dx/2, ±dy/2, ±dz/2).
    xs, ys, zs = positions[0::3], positions[1::3], positions[2::3]
    assert min(xs) == -1.0 and max(xs) == 1.0
    assert min(ys) == -1.5 and max(ys) == 1.5
    assert min(zs) == -2.0 and max(zs) == 2.0


def test_cad_mesh_is_visit_mesh():
    """adacpp.cad.Mesh and adacpp.visit.Mesh must be the same class."""
    assert adacpp.cad.Mesh is adacpp.visit.Mesh


def test_cad_make_box_and_tessellate():
    box = adacpp.cad.make_box(2.0, 3.0, 4.0)
    assert type(box).__name__ == "ShapeHandle"

    mesh = adacpp.cad.tessellate(box)
    positions = list(mesh.positions)
    xs, ys, zs = positions[0::3], positions[1::3], positions[2::3]
    assert min(xs) == -1.0 and max(xs) == 1.0
    assert min(ys) == -1.5 and max(ys) == 1.5
    assert min(zs) == -2.0 and max(zs) == 2.0
    assert len(mesh.indices) >= 12 * 3


def test_cad_tessellate_batch():
    # Batch tessellation returns ONE combined mesh with a GroupReference per
    # input shape; the combined buffer must equal the concatenation of the
    # individual meshes, demarcated by the group ranges.
    boxes = [adacpp.cad.make_box(1.0, 1.0, 1.0) for _ in range(4)]
    singles = [adacpp.cad.tessellate(b, 0.1) for b in boxes]
    batch = adacpp.cad.tessellate_batch(boxes, 0.1)

    import numpy as np

    assert len(batch.groups) == len(boxes)
    assert len(batch.indices) == sum(len(s.indices) for s in singles)
    pos = np.asarray(batch.positions)
    idx = np.asarray(batch.indices)
    cursor = 0
    vcursor = 0
    for i, (g, s) in enumerate(zip(batch.groups, singles)):
        assert g.node_id == i
        # index range
        assert g.start == cursor
        assert g.length == len(s.indices)
        # vertex range: slicing positions by the group must match the single mesh,
        # and the group's local indices (rebased by vstart) must match too.
        assert g.vstart == vcursor
        assert g.vlength == len(s.positions) // 3
        seg_pos = pos[g.vstart * 3 : (g.vstart + g.vlength) * 3]
        assert np.array_equal(seg_pos, np.asarray(s.positions))
        seg_idx = idx[g.start : g.start + g.length] - g.vstart
        assert np.array_equal(seg_idx, np.asarray(s.indices))
        cursor += g.length
        vcursor += g.vlength


def test_cad_shape_handle_is_opaque():
    """ShapeHandle must not leak its kernel internals to Python."""
    box = adacpp.cad.make_box(1.0, 1.0, 1.0)
    public_attrs = [a for a in dir(box) if not a.startswith("_")]
    assert public_attrs == []


def test_cad_make_cylinder():
    cyl = adacpp.cad.make_cylinder(2.0, 5.0)
    mesh = adacpp.cad.tessellate(cyl)
    positions = list(mesh.positions)
    xs, _, zs = positions[0::3], positions[1::3], positions[2::3]
    # Cylinder along +Z, radius 2, base at z=0, top at z=5.
    assert min(zs) == 0.0 and max(zs) == 5.0
    # XY radius is approximately 2 (tessellation chord-deflects slightly inward).
    assert abs(max(xs) - 2.0) < 0.2
    assert abs(min(xs) - -2.0) < 0.2


def test_cad_make_sphere():
    sph = adacpp.cad.make_sphere(3.0)
    mesh = adacpp.cad.tessellate(sph)
    positions = list(mesh.positions)
    xs, ys, zs = positions[0::3], positions[1::3], positions[2::3]
    # Sphere centered at origin, radius 3.
    assert abs(max(xs) - 3.0) < 0.3 and abs(min(xs) - -3.0) < 0.3
    assert abs(max(ys) - 3.0) < 0.3 and abs(min(ys) - -3.0) < 0.3
    assert abs(max(zs) - 3.0) < 0.3 and abs(min(zs) - -3.0) < 0.3


def test_cad_bbox_box():
    box = adacpp.cad.make_box(2.0, 3.0, 4.0)
    bb = adacpp.cad.bbox(box)
    assert tuple(bb) == (-1.0, -1.5, -2.0, 1.0, 1.5, 2.0)


def test_cad_bbox_cylinder():
    cyl = adacpp.cad.make_cylinder(2.0, 5.0)
    bb = adacpp.cad.bbox(cyl)
    # Native OCCT bbox is exact for primitives; +Z cylinder, base at 0.
    assert tuple(bb) == (-2.0, -2.0, 0.0, 2.0, 2.0, 5.0)


def test_cad_bbox_sphere():
    sph = adacpp.cad.make_sphere(3.0)
    bb = adacpp.cad.bbox(sph)
    assert tuple(bb) == (-3.0, -3.0, -3.0, 3.0, 3.0, 3.0)


def test_cad_extruded_curve_profile_is_open_shell():
    # CURVE profile (is_area=False): sweeping a circle wire +Z yields the open
    # lateral cylinder surface — no end caps. Outer circle r=0.5 in XY, depth 2.
    circle = [[2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.5]]
    shp = adacpp.cad.build_extruded_area_solid(
        circle, [], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 2.0, False
    )
    bb = tuple(round(v, 6) for v in adacpp.cad.bbox(shp))
    assert bb == (-0.5, -0.5, 0.0, 0.5, 0.5, 2.0)
    # Open lateral surface: a single cylindrical face, no caps.
    assert len(adacpp.cad.faces(shp)) == 1


def test_cad_extruded_area_solid_tapered():
    # Loft (ThruSections) between a 2x2 square base and a 1x1 square top over
    # height 2 → a frustum. Volume = h/3 * (A1 + A2 + sqrt(A1*A2))
    #          = 2/3 * (4 + 1 + 2) = 4.666667.
    def rect(half):
        pts = [(-half, -half, 0.0), (half, -half, 0.0), (half, half, 0.0), (-half, half, 0.0)]
        return [[0.0, *pts[i], *pts[(i + 1) % 4]] for i in range(4)]

    shp = adacpp.cad.build_extruded_area_solid_tapered(
        rect(1.0), rect(0.5), [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 2.0
    )
    assert adacpp.cad.is_valid(shp)
    assert len(adacpp.cad.solids(shp)) == 1
    assert round(adacpp.cad.volume(shp), 6) == round(2.0 / 3.0 * 7.0, 6)
    bb = tuple(round(v, 6) for v in adacpp.cad.bbox(shp))
    assert bb == (-1.0, -1.0, 0.0, 1.0, 1.0, 2.0)


def test_cad_loft_profiles_two_squares_is_box():
    # Loft two equal squares 1 apart in Z → a unit box: 6 faces, volume 1.
    def rect(half, z):
        return [[-half, -half, z], [half, -half, z], [half, half, z], [-half, half, z]]

    box = adacpp.cad.loft_profiles([rect(0.5, 0.0), rect(0.5, 1.0)], True, True)
    assert adacpp.cad.is_valid(box)
    assert len(adacpp.cad.faces(box)) == 6
    assert round(adacpp.cad.volume(box), 6) == 1.0


def test_cad_loft_profiles_rejects_single():
    with pytest.raises(Exception):
        adacpp.cad.loft_profiles([[[0, 0, 0], [1, 0, 0], [1, 1, 0]]], True, True)


def test_cad_section_with_plane():
    # Mid-span Z section of a tall box → a single planar cross-section face.
    def rect(half, z):
        return [[-half, -half, z], [half, -half, z], [half, half, z], [-half, half, z]]

    box = adacpp.cad.loft_profiles([rect(1.0, 0.0), rect(1.0, 4.0)], True, True)
    sec = adacpp.cad.section_with_plane(box, [0.0, 0.0, 2.0], [0.0, 0.0, 1.0], 1000.0)
    assert len(adacpp.cad.faces(sec)) == 1


def test_cad_write_step(tmp_path):
    # Write two named, colored boxes to STEP via the OCAF/XCAF document model.
    b1 = adacpp.cad.build_box([0, 0, 0], [0, 0, 1], [1, 0, 0], 1, 1, 1)
    b2 = adacpp.cad.build_box([2, 0, 0], [0, 0, 1], [1, 0, 0], 1, 2, 3)
    out = tmp_path / "two_boxes.stp"
    adacpp.cad.write_step([b1, b2], ["BoxA", "BoxB"], [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], str(out), "m", "AP214")
    assert out.exists() and out.stat().st_size > 0
    text = out.read_text()
    assert "ISO-10303" in text
    assert "BoxA" in text and "BoxB" in text


def test_cad_build_bspline_surface_face():
    # Degree-2 dome: 3x3 control net with the centre row lifted in Z. Clamped
    # knots [0,1] mult [3,3] each direction. Expect one valid trimmed face whose
    # actual surface peaks at z=0.375 (well below the z=1 centre control point).
    cps = [
        [[0, 0, 0], [0, 1, 0], [0, 2, 0]],
        [[1, 0, 0.5], [1, 1, 1.0], [1, 2, 0.5]],
        [[2, 0, 0], [2, 1, 0], [2, 2, 0]],
    ]
    face = adacpp.cad.build_bspline_surface_face(2, 2, cps, [0.0, 1.0], [0.0, 1.0], [3, 3], [3, 3], [], 1e-6)
    assert adacpp.cad.is_valid(face)
    assert len(adacpp.cad.faces(face)) == 1
    bb = tuple(round(v, 4) for v in adacpp.cad.bbox(face))
    assert bb == (0.0, 0.0, 0.0, 2.0, 2.0, 0.375)


def _quad_edges(pts):
    # Closed polygon as line edge records [0, p1, p2] for build_planar_face.
    return [[0.0, *pts[i], *pts[(i + 1) % len(pts)]] for i in range(len(pts))]


def test_cad_sew_faces_open_shell():
    # Two quads sharing an edge sew into a single connected shell (one handle, two
    # faces) — the open-shell case (IfcShellBasedSurfaceModel) where
    # make_volumes_from_faces would yield nothing because nothing bounds a volume.
    f1 = adacpp.cad.build_planar_face(
        _quad_edges([(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)]), [], [0, 0, 0], [0, 0, 1], [1, 0, 0]
    )
    f2 = adacpp.cad.build_planar_face(
        _quad_edges([(1, 0, 0), (2, 0, 0), (2, 1, 0), (1, 1, 0)]), [], [0, 0, 0], [0, 0, 1], [1, 0, 0]
    )
    faces = adacpp.cad.faces(f1) + adacpp.cad.faces(f2)
    shell = adacpp.cad.sew_faces(faces)
    assert adacpp.cad.shape_type(shell) == "shell"
    assert len(adacpp.cad.faces(shell)) == 2


def test_cad_polygon_face():
    # A closed quad polygon -> one planar face (divider face for make_volumes_from_faces).
    face = adacpp.cad.polygon_face([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]])
    assert adacpp.cad.shape_type(face) == "face"
    assert round(adacpp.cad.area(face), 6) == 1.0


def test_cad_introspection_helpers():
    # area / shape_type / face_surface_type — backend-neutral replacements for
    # GProp + TopAbs + BRep_Tool::Surface introspection.
    box = adacpp.cad.build_box([0, 0, 0], [0, 0, 1], [1, 0, 0], 2, 3, 4)
    assert adacpp.cad.shape_type(box) == "solid"
    assert round(adacpp.cad.area(box), 5) == 52.0  # 2*(2*3 + 3*4 + 2*4)
    assert round(adacpp.cad.volume(box), 5) == 24.0
    box_face = adacpp.cad.faces(box)[0]
    assert adacpp.cad.shape_type(box_face) == "face"
    assert adacpp.cad.face_surface_type(box_face) == "plane"

    cyl = adacpp.cad.build_cylinder([0, 0, 0], [0, 0, 1], 1.0, 2.0)
    assert {adacpp.cad.face_surface_type(f) for f in adacpp.cad.faces(cyl)} == {"cylinder", "plane"}

    bs = adacpp.cad.build_bspline_surface_face(
        2,
        2,
        [[[0, 0, 0], [0, 1, 0], [0, 2, 0]], [[1, 0, 0.5], [1, 1, 1.0], [1, 2, 0.5]], [[2, 0, 0], [2, 1, 0], [2, 2, 0]]],
        [0.0, 1.0],
        [0.0, 1.0],
        [3, 3],
        [3, 3],
        [],
        1e-6,
    )
    assert adacpp.cad.face_surface_type(bs) == "bspline"


def test_cad_extrude_face_along_normal():
    # Extrude a B-spline dome face by 0.1 along its normal → a thin solid.
    bs = adacpp.cad.build_bspline_surface_face(
        2,
        2,
        [[[0, 0, 0], [0, 1, 0], [0, 2, 0]], [[1, 0, 0.5], [1, 1, 1.0], [1, 2, 0.5]], [[2, 0, 0], [2, 1, 0], [2, 2, 0]]],
        [0.0, 1.0],
        [0.0, 1.0],
        [3, 3],
        [3, 3],
        [],
        1e-6,
    )
    solid = adacpp.cad.extrude_face_along_normal(bs, 0.1)
    assert adacpp.cad.shape_type(solid) == "solid"
    assert adacpp.cad.is_valid(solid)
    assert adacpp.cad.volume(solid) > 0.0
    # thickness 0 short-circuits to the bare face.
    assert adacpp.cad.shape_type(adacpp.cad.extrude_face_along_normal(bs, 0.0)) == "face"


def test_cad_build_wire_curve_zoo():
    # Wire from line + 3-point arc + lines (closed path).
    edges = [
        [0, 0, 0, 0, 1, 0, 0],
        [1, 1, 0, 0, 1.5, 0.5, 0, 1, 1, 0],
        [0, 1, 1, 0, 0, 1, 0],
        [0, 0, 1, 0, 0, 0, 0],
    ]
    w = adacpp.cad.build_wire(edges)
    assert adacpp.cad.shape_type(w) == "wire"

    # Degree-2 B-spline curve edge (3 poles, clamped knots) + a closing line.
    bs_edge = [3, 2, 0, 0, 0.0, 0.0, 3, 0, 0, 0, 1, 1, 0, 2, 0, 0, 2, 0.0, 1.0, 3, 3]
    w2 = adacpp.cad.build_wire([bs_edge, [0, 2, 0, 0, 0, 0, 0]])
    assert adacpp.cad.shape_type(w2) == "wire"
    assert len(adacpp.cad.edges(w2)) == 2


def test_cad_build_filled_face():
    # MakeFilling interpolates a smooth (B-spline) surface through a 4-edge
    # square boundary → one valid face.
    sq = [[0, 0, 0, 0, 1, 0, 0], [0, 1, 0, 0, 1, 1, 0], [0, 1, 1, 0, 0, 1, 0], [0, 0, 1, 0, 0, 0, 0]]
    ff = adacpp.cad.build_filled_face(sq)
    assert adacpp.cad.shape_type(ff) == "face"
    assert len(adacpp.cad.faces(ff)) == 1
    assert adacpp.cad.face_surface_type(ff) == "bspline"


def test_cad_build_advanced_face_bspline_with_pcurves():
    # Bilinear (degree-1) B-spline surface = unit-square plane in z=0, trimmed by
    # a UV-square boundary made of 4 kind-6 pcurve (2D line) edges → unit face.
    cps = [[[0, 0, 0], [0, 1, 0]], [[1, 0, 0], [1, 1, 0]]]  # cp[u][v] = (u, v, 0)
    uk = vk = [0.0, 1.0]
    um = vm = [2, 2]

    def pcurve_line(u0, v0, u1, v1):
        # kind 6: [6, degree, rational, closed, n_poles, <2*n uv>, n_knots, knots, mults]
        return [6, 1, 0, 0, 2, u0, v0, u1, v1, 2, 0.0, 1.0, 2, 2]

    loop = [
        pcurve_line(0, 0, 1, 0),
        pcurve_line(1, 0, 1, 1),
        pcurve_line(1, 1, 0, 1),
        pcurve_line(0, 1, 0, 0),
    ]
    face = adacpp.cad.build_advanced_face_bspline(1, 1, cps, uk, vk, um, vm, [], [loop])
    assert adacpp.cad.shape_type(face) == "face"
    assert adacpp.cad.face_surface_type(face) == "bspline"
    assert round(adacpp.cad.area(face), 6) == 1.0
    bb = tuple(round(v, 4) for v in adacpp.cad.bbox(face))
    assert bb == (0.0, 0.0, 0.0, 1.0, 1.0, 0.0)


def test_cad_face_to_advanced_face_roundtrip():
    # Build a trimmed B-spline face, decompose it, rebuild from the decomposed
    # surface + pcurves, and check the area survives the round-trip.
    cps = [[[0, 0, 0], [0, 1, 0]], [[1, 0, 0], [1, 1, 0]]]
    uk = vk = [0.0, 1.0]
    um = vm = [2, 2]

    def pcurve_line(u0, v0, u1, v1):
        return [6, 1, 0, 0, 2, u0, v0, u1, v1, 2, 0.0, 1.0, 2, 2]

    loop = [pcurve_line(0, 0, 1, 0), pcurve_line(1, 0, 1, 1), pcurve_line(1, 1, 0, 1), pcurve_line(0, 1, 0, 0)]
    face = adacpp.cad.build_advanced_face_bspline(1, 1, cps, uk, vk, um, vm, [], [loop])
    area0 = adacpp.cad.area(face)

    data = adacpp.cad.face_to_advanced_face(face)
    assert data.u_degree == 1 and data.v_degree == 1
    assert len(data.poles) == 2 and len(data.poles[0]) == 2
    assert len(data.bounds) == 1
    edges = data.bounds[0]
    assert len(edges) == 4
    assert all(e.has_pcurve for e in edges)
    assert all(e.degree >= 1 and len(e.control_points) >= 2 for e in edges)

    # Rebuild from the decomposed data and compare area.
    bounds2 = []
    for e in edges:
        rec = [6, e.degree, 1 if e.weights else 0, 1 if e.closed else 0, len(e.control_points)]
        for cp in e.control_points:
            rec += [cp[0], cp[1]]
        rec += [len(e.knots), *e.knots, *[float(m) for m in e.multiplicities]]
        rec += [float(w) for w in e.weights]
        bounds2.append(rec)
    face2 = adacpp.cad.build_advanced_face_bspline(
        data.u_degree,
        data.v_degree,
        data.poles,
        data.u_knots,
        data.v_knots,
        data.u_multiplicities,
        data.v_multiplicities,
        data.weights,
        [bounds2],
    )
    assert round(adacpp.cad.area(face2), 6) == round(area0, 6)


def test_cad_read_step_shapes_roundtrip(tmp_path):
    # Write two named/colored boxes, read them back via the OCAF reader.
    b1 = adacpp.cad.build_box([0, 0, 0], [0, 0, 1], [1, 0, 0], 1, 1, 1)
    b2 = adacpp.cad.build_box([2, 0, 0], [0, 0, 1], [1, 0, 0], 1, 2, 3)
    out = tmp_path / "two.stp"
    adacpp.cad.write_step([b1, b2], ["BoxA", "BoxB"], [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], str(out), "m", "AP214")

    data = adacpp.cad.read_step_shapes(out.read_bytes())
    named = {d.name: tuple(round(c, 3) for c in d.color) for d in data if d.name and d.has_color}
    assert named.get("BoxA") == (1.0, 0.0, 0.0)
    assert named.get("BoxB") == (0.0, 1.0, 0.0)
    # every returned entry carries a usable shape handle
    assert all(adacpp.cad.shape_type(d.shape) in ("solid", "compound", "shell", "face") for d in data)


def test_cad_revolved_curve_profile():
    # Revolve a circle wire (r=0.5, centered at x=2 in XY) a quarter turn about
    # the world Z axis through the origin → a curved pipe-elbow surface.
    circle = [[2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.5]]
    shp = adacpp.cad.build_revolved_area_solid(
        circle, [], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 90.0, False
    )
    assert adacpp.cad.is_valid(shp)
    bb = adacpp.cad.bbox(shp)
    # Swept quarter-arc of a tube whose centerline radius is 2, tube radius 0.5.
    assert round(bb[3], 6) == 2.5  # +x extent = outer centerline + tube radius
    assert round(bb[4], 6) == 2.5  # +y extent after the 90° sweep


def _signed_mesh_volume(mesh):
    # Signed volume via the divergence theorem; positive iff triangles are
    # consistently wound outward.
    pos = list(mesh.positions)
    idx = list(mesh.indices)
    vol = 0.0
    for t in range(0, len(idx), 3):
        a, b, c = idx[t], idx[t + 1], idx[t + 2]
        ax, ay, az = pos[3 * a], pos[3 * a + 1], pos[3 * a + 2]
        bx, by, bz = pos[3 * b], pos[3 * b + 1], pos[3 * b + 2]
        cx, cy, cz = pos[3 * c], pos[3 * c + 1], pos[3 * c + 2]
        vol += ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)
    return vol / 6.0


def test_cad_tessellation_winding_is_outward():
    # A closed solid must tessellate with outward-facing triangles, i.e. a
    # positive signed mesh volume. Reversed faces (common in solids) need their
    # winding flipped — this guards that fix. A centered box has reversed faces.
    box = adacpp.cad.make_box(2.0, 2.0, 2.0)
    mesh = adacpp.cad.tessellate(box, -1.0)
    assert _signed_mesh_volume(mesh) > 0.0


def test_cad_fixed_reference_swept_area_solid():
    # Sweep a square profile (in the XY plane at the directrix start) along a
    # straight directrix up +Z by 1.0 → a unit square prism of height 1.
    h = 0.2
    directrix = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]  # line (0,0,0)->(0,0,1)
    profile = [
        [0.0, 0.0, 0.0, 0.0, h, 0.0, 0.0],  # (0,0,0)->(h,0,0)
        [0.0, h, 0.0, 0.0, h, h, 0.0],  # (h,0,0)->(h,h,0)
        [0.0, h, h, 0.0, 0.0, h, 0.0],  # (h,h,0)->(0,h,0)
        [0.0, 0.0, h, 0.0, 0.0, 0.0, 0.0],  # (0,h,0)->(0,0,0)
    ]
    shp = adacpp.cad.build_fixed_reference_swept_area_solid(directrix, profile, [0.0, 0.0, 0.0])
    assert adacpp.cad.is_valid(shp)
    # Square h×h swept length 1 → volume h*h.
    assert round(adacpp.cad.volume(shp), 6) == round(h * h, 6)
    # Solid tessellates outward (positive signed volume).
    assert _signed_mesh_volume(adacpp.cad.tessellate(shp, -1.0)) > 0.0


def test_cad_make_halfspace_cuts_a_box():
    # A half-space at z=0 (solid below) cut from a centered 2³ box removes the
    # top half, leaving a planar cut face on z=0.
    box = adacpp.cad.make_box(2.0, 2.0, 2.0)
    hs = adacpp.cad.make_halfspace([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], False)
    surfs = adacpp.cad.cut_surfaces(box, [hs], 1e-3, 1e-4)
    assert len(surfs) >= 1
    # Each surface tuple: (type, normal, outer_edges, outer_polyline, inners).
    planar = [s for s in surfs if s[0] == "Plane"]
    assert planar, "expected at least one planar cut face"
    surface_type, normal, outer_edges, outer_polyline, inners = planar[0]
    # The cut face lies on z=0.
    assert all(abs(p[2]) < 1e-6 for p in outer_polyline)
    assert len(outer_polyline) >= 3
    # Edges are classified (a box cut → straight Line edges).
    assert all(e[0] == "Line" for e in outer_edges)


def test_cad_obb_box():
    # Oriented bbox of a centered axis-aligned box: barycenter at origin,
    # half-sizes equal to the box half-extents (axes aligned to world).
    box = adacpp.cad.make_box(2.0, 3.0, 4.0)
    center, half = adacpp.cad.obb(box)
    assert tuple(round(v, 6) for v in center) == (0.0, 0.0, 0.0)
    assert tuple(sorted(round(v, 6) for v in half)) == (1.0, 1.5, 2.0)


def test_cad_write_glb_bytes_box():
    """Box → glTF binary: header + non-empty body."""
    box = adacpp.cad.make_box(2.0, 3.0, 4.0)
    glb = adacpp.cad.write_glb_bytes(box)
    assert isinstance(glb, bytes)
    assert glb[:4] == b"glTF"
    # GLB version field is little-endian u32 right after the magic.
    assert int.from_bytes(glb[4:8], "little") == 2
    assert len(glb) > 200  # at least header + JSON chunk + BIN chunk


def test_cad_read_step_bytes(files_dir):
    """STEP bytes → ShapeHandle that has actual geometry (non-trivial bbox)."""
    data = (files_dir / "flat_plate_abaqus_10x10_m_wColors.stp").read_bytes()
    handle = adacpp.cad.read_step_bytes(data)
    assert type(handle).__name__ == "ShapeHandle"
    bb = adacpp.cad.bbox(handle)
    # The plate covers a noticeable area in X/Y; sanity-check it's non-empty.
    assert (bb[3] - bb[0]) > 1.0
    assert (bb[4] - bb[1]) > 1.0


def test_cad_step_to_glb_roundtrip(files_dir):
    """End-to-end: STEP bytes → ShapeHandle → glTF bytes."""
    data = (files_dir / "flat_plate_abaqus_10x10_m_wColors.stp").read_bytes()
    handle = adacpp.cad.read_step_bytes(data)
    glb = adacpp.cad.write_glb_bytes(handle)
    assert glb[:4] == b"glTF"
    assert len(glb) > 1000  # converted geometry produces a reasonable file


def test_cad_from_topods_pointer():
    """Native interop: round-trip an adacpp.cadit.occt.TopoDS_Shape into cad."""
    if adacpp.cad.from_topods_pointer is None:
        pytest.skip("from_topods_pointer is native-only")

    # Create a TopoDS_Shape on the C++ side via the existing cadit path, then
    # expose its address (cadit.occt.TopoDS_Shape stores a shared_ptr<TopoDS_Shape>
    # whose underlying pointer is the .get_ptr() — but we already have a Python
    # path that goes the other way. Easier: feed write_box_to_step a temporary
    # path so we know that path lights up the OCCT side, then build via pyocc.
    pyocc = pytest.importorskip("OCC.Core.BRepPrimAPI")
    BRepPrimAPI_MakeBox = pyocc.BRepPrimAPI_MakeBox
    from OCC.Core.gp import gp_Pnt

    pyocc_shape = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 1.0, 2.0, 3.0).Shape()
    handle = adacpp.cad.from_topods_pointer(int(pyocc_shape.this))

    mesh = adacpp.cad.tessellate(handle)
    positions = list(mesh.positions)
    xs, ys, zs = positions[0::3], positions[1::3], positions[2::3]
    # Corner at origin, dims 1×2×3.
    assert min(xs) == 0.0 and max(xs) == 1.0
    assert min(ys) == 0.0 and max(ys) == 2.0
    assert min(zs) == 0.0 and max(zs) == 3.0


# --- shape-algebra verbs (mirror adapy's CadBackend) ---


def test_cad_transform_translates_box():
    box = adacpp.cad.make_box(2.0, 2.0, 2.0)  # centered: [-1, 1]^3
    # top 3 rows of a translation-by-(1,1,1) 4x4, row-major
    moved = adacpp.cad.transform(box, [1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1], True)
    assert tuple(round(v, 6) for v in adacpp.cad.bbox(moved)) == (0.0, 0.0, 0.0, 2.0, 2.0, 2.0)


def test_cad_boolean_ops():
    a = adacpp.cad.make_box(2.0, 2.0, 2.0)  # [-1, 1]^3
    b = adacpp.cad.transform(adacpp.cad.make_box(2.0, 2.0, 2.0), [1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1], True)
    assert tuple(round(v, 6) for v in adacpp.cad.bbox(adacpp.cad.boolean("UNION", a, b))) == (-1, -1, -1, 2, 2, 2)
    assert tuple(round(v, 6) for v in adacpp.cad.bbox(adacpp.cad.boolean("INTERSECTION", a, b))) == (0, 0, 0, 1, 1, 1)
    # difference is non-empty and bounded by a
    assert adacpp.cad.is_valid(adacpp.cad.boolean("DIFFERENCE", a, b))
    with pytest.raises(Exception):
        adacpp.cad.boolean("NOPE", a, b)


def test_cad_distance():
    a = adacpp.cad.make_box(2.0, 2.0, 2.0)  # ends at x=1
    b = adacpp.cad.transform(
        adacpp.cad.make_box(2.0, 2.0, 2.0), [1, 0, 0, 5, 0, 1, 0, 0, 0, 0, 1, 0], True
    )  # x in [4,6]
    assert round(adacpp.cad.distance(a, b), 6) == 3.0


def test_cad_is_valid_and_serialize():
    box = adacpp.cad.make_box(1.0, 1.0, 1.0)
    assert adacpp.cad.is_valid(box) is True
    s = adacpp.cad.serialize(box)
    assert isinstance(s, str) and "CASCADE Topology" in s[:40] and len(s) > 100


def test_cad_faces_and_vertex_points():
    box = adacpp.cad.make_box(1.0, 1.0, 1.0)
    faces = adacpp.cad.faces(box)
    assert len(faces) == 6
    assert all(type(f).__name__ == "ShapeHandle" for f in faces)
    pts = adacpp.cad.vertex_points(box)
    assert len(pts) == 8  # unique vertices
    assert all(len(p) == 3 for p in pts)


def test_cad_face_plane():
    box = adacpp.cad.make_box(2.0, 2.0, 2.0)
    faces = adacpp.cad.faces(box)
    origin, normal = adacpp.cad.face_plane(faces[0])
    assert len(origin) == 3 and len(normal) == 3
    # a box face normal is axis-aligned (unit on one component)
    assert round(sum(c * c for c in normal), 6) == 1.0
    # a solid (non-face) shape returns None
    assert adacpp.cad.face_plane(box) is None


def test_cad_volume():
    # make_box(2,3,4) is centered; volume = 24
    assert round(adacpp.cad.volume(adacpp.cad.make_box(2.0, 3.0, 4.0)), 6) == 24.0


def test_cad_build_extruded_area_solid():
    # A unit square in XY (4 line edges), extruded +Z by 2 → 1×1×2 box.
    square = [
        [0.0, 0, 0, 0, 1, 0, 0],
        [0.0, 1, 0, 0, 1, 1, 0],
        [0.0, 1, 1, 0, 0, 1, 0],
        [0.0, 0, 1, 0, 0, 0, 0],
    ]
    solid = adacpp.cad.build_extruded_area_solid(square, [], [0, 0, 0], [0, 0, 1], [1, 0, 0], 2.0)
    assert tuple(round(v, 6) for v in adacpp.cad.bbox(solid)) == (0.0, 0.0, 0.0, 1.0, 1.0, 2.0)
    assert round(adacpp.cad.volume(solid), 6) == 2.0

    # Same outer with a 0.5×0.5 inner void at the centre → volume 2 - 0.25*2 = 1.5.
    inner = [
        [0.0, 0.25, 0.25, 0, 0.75, 0.25, 0],
        [0.0, 0.75, 0.25, 0, 0.75, 0.75, 0],
        [0.0, 0.75, 0.75, 0, 0.25, 0.75, 0],
        [0.0, 0.25, 0.75, 0, 0.25, 0.25, 0],
    ]
    hollow = adacpp.cad.build_extruded_area_solid(square, [inner], [0, 0, 0], [0, 0, 1], [1, 0, 0], 2.0)
    assert round(adacpp.cad.volume(hollow), 6) == 1.5


def _unit_box_faces(x0):
    # Unit box at x0..x0+1 in x, 0..1 in y/z; return its 6 faces.
    h = adacpp.cad.build_box((x0, 0, 0), (0, 0, 1), (1, 0, 0), 1.0, 1.0, 1.0)
    return list(adacpp.cad.faces(h))


def test_cad_make_volumes_from_faces_two_abutting_boxes():
    # Two unit cubes sharing the x=1 plane -> 2 cells, 10 free (envelope) faces.
    soup = _unit_box_faces(0) + _unit_box_faces(1)
    cells = adacpp.cad.make_volumes_from_faces(soup, 1e-6)
    assert len(cells) == 2
    assert sorted(round(adacpp.cad.volume(c), 6) for c in cells) == [1.0, 1.0]
    assert len(adacpp.cad.free_faces(cells)) == 10
    coms = sorted(tuple(round(v, 4) for v in adacpp.cad.center_of_mass(c)) for c in cells)
    assert coms == [(0.5, 0.5, 0.5), (1.5, 0.5, 0.5)]


def test_cad_point_in_solid():
    cells = sorted(
        adacpp.cad.make_volumes_from_faces(_unit_box_faces(0) + _unit_box_faces(1), 1e-6),
        key=lambda c: adacpp.cad.center_of_mass(c)[0],
    )
    a, b = cells
    assert adacpp.cad.point_in_solid(a, [0.5, 0.5, 0.5], 1e-6) == 0  # IN
    assert adacpp.cad.point_in_solid(a, [1.5, 0.5, 0.5], 1e-6) == 1  # OUT
    assert adacpp.cad.point_in_solid(b, [1.5, 0.5, 0.5], 1e-6) == 0  # IN
    assert adacpp.cad.point_in_solid(a, [3.0, 3.0, 3.0], 1e-6) == 1  # OUT


def test_cad_non_manifold_merge_keeps_shared_face():
    a = adacpp.cad.build_box((0, 0, 0), (0, 0, 1), (1, 0, 0), 1.0, 1.0, 1.0)
    b = adacpp.cad.build_box((1, 0, 0), (0, 0, 1), (1, 0, 0), 1.0, 1.0, 1.0)
    comp = adacpp.cad.non_manifold_merge([a, b], 1e-6, True)
    sols = list(adacpp.cad.solids(comp))
    assert len(sols) == 2
    assert len(adacpp.cad.free_faces(sols)) == 10
