"""Backend-agnostic CAD operations.

This module is the future home of the CadBackend abstraction that will let
adapy swap between pythonocc-core and adacpp implementations. For now it
re-exports what the C++ side provides under ``adacpp._ada_cpp_ext_impl.cad``,
and the surface is identical between the native and wasm/pyodide builds.
"""

from .._ada_cpp_ext_impl.cad import (
    Color,
    GroupReference,
    Mesh,
    MeshType,
    tessellate_box,
)

__all__ = ["Color", "GroupReference", "Mesh", "MeshType", "tessellate_box"]
