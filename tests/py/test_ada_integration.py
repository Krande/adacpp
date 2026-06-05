import pytest

from adacpp.cadit import STEPCAFControl_Writer, step_writer_to_string

try:
    import ada
    import OCC.Core.Interface as OCCInterface
    from ada.occ.store import OCCStore
    from OCC.Core.STEPCAFControl import (
        STEPCAFControl_Writer as OCC_STEPCAFControl_Writer,
    )
    from OCC.Core.STEPControl import STEPControl_AsIs
    from OCC.Core.XSControl import XSControl_WorkSession

    has_ada = True
except ModuleNotFoundError:
    has_ada = False


@pytest.fixture
def beams_model():
    beams = []
    for i in range(0, 5):
        bm = ada.Beam(f"bm{i}", (i, 0, 0), (i + 1, 0, 0), "IPE300")
        beams.append(bm)
    return ada.Assembly() / beams


@pytest.mark.skipif(not has_ada, reason="ada is not installed")
def test_ada_integration(beams_model):
    step_writer = OCCStore.get_step_writer()
    shape_iter = OCCStore.shape_iterator(beams_model)
    for i, (obj, shape) in enumerate(shape_iter, start=1):
        step_writer.add_shape(shape, obj.name, rgb_color=obj.color.rgb)

    session = XSControl_WorkSession()
    SetCVal = OCCInterface.Interface_Static.SetCVal
    SetCVal("write.step.unit", step_writer.units.value.upper())
    SetCVal("write.step.schema", step_writer.schema.value.upper())

    writer = OCC_STEPCAFControl_Writer(session, False)
    writer.SetColorMode(True)
    writer.SetNameMode(True)
    writer.Transfer(step_writer.doc, STEPControl_AsIs)

    writer_pointer = int(writer.this)
    ada_cpp_writer = STEPCAFControl_Writer.from_ptr(writer_pointer)
    result = step_writer_to_string(ada_cpp_writer)

    # Structural checks rather than an exact byte count: the latter is brittle to
    # OCCT version changes and to any process-global Interface_Static drift left
    # by an earlier test (e.g. write.surfacecurve.mode), which made this assert
    # order-dependent. Verify instead that the bridge produced a valid, complete
    # STEP carrying all five beams.
    assert result.startswith("ISO-10303-21;")
    assert result.rstrip().endswith("END-ISO-10303-21;")
    assert "FILE_SCHEMA" in result
    for i in range(5):
        assert f"bm{i}" in result
    # A 5-beam AP214 STEP is on the order of ~200 KB; guard against truncated or
    # empty output without pinning an exact length.
    assert len(result) > 100_000


@pytest.mark.skipif(not has_ada, reason="ada is not installed")
def test_basic_occ_shapes_tessellated():
    """Create 3 boxes in pythonocc-core and move them to adacpp using pointer reference and tesselate the shapes"""
    from adacpp.cadit.occt import TopoDS_Solid

    box1 = ada.PrimBox("box1", (0, 0, 0), (1, 1, 1))
    box2 = ada.PrimBox("box2", (1.1, 0, 0), (2, 2, 2))
    box3 = ada.PrimBox("box3", (2.2, 0, 0), (3, 3, 3))

    shapes = []
    for box in [box1, box2, box3]:
        occ_geom = box.solid_occ()
        ptr = int(occ_geom.this)
        shape = TopoDS_Solid.from_ptr(ptr)
        shapes.append(shape)
