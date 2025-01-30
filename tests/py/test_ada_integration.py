
import OCC.Core.Interface as OCCInterface
import pytest
from OCC.Core.STEPCAFControl import STEPCAFControl_Writer as OCC_STEPCAFControl_Writer
from OCC.Core.STEPControl import STEPControl_AsIs
from OCC.Core.XSControl import XSControl_WorkSession

from adacpp.cadit import STEPCAFControl_Writer, step_writer_to_string

try:
    import ada
    from ada.occ.store import OCCStore

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
    assert len(result) == 227399

@pytest.mark.skipif(not has_ada, reason="ada is not installed")
def test_basic_occ_shapes_tessellated():
    """Create 3 boxes in pythonocc-core and move them to adacpp using pointer reference and tesselate the shapes"""
    from adacpp.cadit.occt import TopoDS_Shape, TopoDS_Solid

    box1 = ada.PrimBox('box1', (0, 0, 0), (1, 1, 1))
    box2 = ada.PrimBox('box2', (1.1, 0, 0), (2, 2, 2))
    box3 = ada.PrimBox('box3', (2.2, 0, 0), (3, 3, 3))

    shapes = []
    for box in [box1, box2, box3]:
        occ_geom = box.solid_occ()
        ptr = int(occ_geom.this)
        shape = TopoDS_Solid.from_ptr(ptr)
        shapes.append(shape)



