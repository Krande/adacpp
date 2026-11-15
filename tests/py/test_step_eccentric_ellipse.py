"""A short arc of a very eccentric ELLIPSE edge must stay short through the native STEP path.

The edge's end vertex sits 0.0003 off the ellipse and 0.0041 along it from the start vertex. The
eccentric-anomaly guess atan2(y/ry, x/rx) orders the two vertices backwards on a 165.8 x 3.86
ellipse, which used to wrap the edge round the whole ellipse and blow the tiny face up into a
~330-wide spike. The fix projects the vertices orthogonally onto the ellipse.
"""

import numpy as np

from adacpp import cad

SLIVER = """ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''),'2;1');
FILE_NAME('sliver','',(''),(''),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1=CARTESIAN_POINT('',(146.64043956,-1.80127146,0.));
#2=CARTESIAN_POINT('',(146.64453956,-1.80139846,0.));
#3=CARTESIAN_POINT('',(146.14043956,-1.30127146,0.));
#4=CARTESIAN_POINT('',(0.,0.,0.));
#11=VERTEX_POINT('',#1);
#12=VERTEX_POINT('',#2);
#13=VERTEX_POINT('',#3);
#20=DIRECTION('',(0.,0.,1.));
#21=DIRECTION('',(1.,0.,0.));
#22=AXIS2_PLACEMENT_3D('',#4,#20,#21);
#23=ELLIPSE('',#22,165.8,3.86);
#24=VECTOR('',#21,1.);
#25=LINE('',#4,#24);
#31=EDGE_CURVE('',#11,#12,#23,.T.);
#32=EDGE_CURVE('',#12,#13,#25,.T.);
#33=EDGE_CURVE('',#13,#11,#25,.T.);
#41=ORIENTED_EDGE('',*,*,#31,.T.);
#42=ORIENTED_EDGE('',*,*,#32,.T.);
#43=ORIENTED_EDGE('',*,*,#33,.T.);
#50=EDGE_LOOP('',(#41,#42,#43));
#51=FACE_OUTER_BOUND('',#50,.T.);
#52=PLANE('',#22);
#53=ADVANCED_FACE('sliver',(#51),#52,.T.);
#54=OPEN_SHELL('',(#53));
#55=SHELL_BASED_SURFACE_MODEL('',(#54));
ENDSEC;
END-ISO-10303-21;
"""


def test_eccentric_ellipse_arc_stays_local(tmp_path):
    p = tmp_path / "sliver.stp"
    p.write_text(SLIVER)
    m = cad.stream_step_to_meshes(str(p), "libtess2", 0.0, 20.0)
    v = np.asarray(m.positions, float).reshape(-1, 3)
    assert len(np.asarray(m.indices)) >= 3
    extent = v.max(0) - v.min(0)
    # The face is a ~0.5 x 0.5 sliver; wrapping round the ellipse made it ~330 x 7.7.
    assert extent[0] < 1.0 and extent[1] < 1.0, extent
