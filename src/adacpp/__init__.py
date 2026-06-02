from . import _ada_cpp_ext_impl as _ext
from . import cad

__doc__ = "A module with drop-in replacement functions for ada-py written in c++ to improve performance."
__all__ = ["cad"]

# Native (non-wasm) builds register the OCCT/CGAL/gmsh/IfcOpenShell-backed
# submodules. The wasm/pyodide build only ships the kernel-agnostic ``cad``
# surface, so guard the legacy facades on what the C++ extension actually
# exposes.
if hasattr(_ext, "cadit"):
    from . import cadit, fem, geom, visit
    from .utils import do_this

    __all__ += ["do_this", "cadit", "visit", "fem", "geom"]
