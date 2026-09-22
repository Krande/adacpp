"""adacpp must not export the symbols of the static libraries it embeds.

adacpp statically links conda-forge ifcopenshell's libIfcParse.a / libIfcGeom.a. The conda
`ifcopenshell` Python extension carries its OWN static copy of the same code. If adacpp exports its
copy, the two collide when both load into one process -- and on macOS that aborts inside
ifcopenshell_wrapper.to_string (dyld coalesces exported weak C++ definitions across images, so the
copies share state neither was built to share). adapy hits that whenever the adacpp backend meets an
IFC write.

Two tests, because the failure is platform-specific:

* the EXPORT SURFACE check runs everywhere nm exists. On Linux the co-load happens to survive even
  with the leak, so this is the only guard Linux CI can actually observe;
* the CO-LOAD check reproduces the real scenario. It runs in a subprocess so an abort surfaces as a
  test failure instead of killing pytest.
"""

import shutil
import subprocess
import sys
from pathlib import Path

import pytest

import adacpp

EXT = Path(adacpp._ada_cpp_ext_impl.__file__)
INIT = "PyInit__ada_cpp_ext_impl"


def _exported_defined_symbols(path: Path) -> list[str]:
    if sys.platform == "darwin":
        cmd = ["nm", "-gU", str(path)]  # global, defined
    else:
        cmd = ["nm", "-D", "--defined-only", str(path)]
    out = subprocess.run(cmd, capture_output=True, text=True, check=True).stdout
    names = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[1] in ("T", "D", "B", "R", "W", "V", "S", "i", "u"):
            names.append(parts[2].lstrip("_") if sys.platform == "darwin" else parts[2])
    return names


@pytest.mark.skipif(sys.platform == "win32", reason="a DLL exports only what is marked dllexport")
@pytest.mark.skipif(shutil.which("nm") is None, reason="needs nm to read the export table")
def test_extension_exports_only_its_init_symbol():
    exported = _exported_defined_symbols(EXT)
    leaked = [s for s in exported if s.lstrip("_") != INIT]
    ifc = [s for s in leaked if any(k in s for k in ("IfcParse", "IfcSchema", "IfcGeom", "rocksdb"))]
    assert INIT in [s.lstrip("_") for s in exported], f"{INIT} must stay exported or Python cannot import the module"
    assert not ifc, f"{len(ifc)} embedded IfcOpenShell/rocksdb symbols are exported, e.g. {ifc[:3]}"
    assert not leaked, f"{len(leaked)} symbols besides {INIT} are exported, e.g. {leaked[:5]}"


def test_adacpp_and_ifcopenshell_coexist_in_one_process():
    """The exact sequence that aborted on macOS: adacpp loaded first, then an ifcopenshell write."""
    pytest.importorskip("ifcopenshell")
    script = (
        "import adacpp.cad\n"  # loads adacpp's embedded IfcOpenShell first
        "import ifcopenshell, ifcopenshell.guid\n"
        "f = ifcopenshell.file(schema='IFC4')\n"
        "f.createIfcProject(ifcopenshell.guid.new(), None, 'p', None, None, None, None, None, None)\n"
        "s = f.to_string()\n"  # the call that aborted
        "assert 'IFCPROJECT' in s, s[:200]\n"
        "print('ok')\n"
    )
    # check=False on purpose: a crash must become a descriptive assertion naming the signal, not a
    # bare CalledProcessError.
    proc = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, timeout=120, check=False)
    assert proc.returncode == 0, (
        f"adacpp + ifcopenshell in one process exited {proc.returncode} "
        f"(negative = killed by signal {-proc.returncode})\nstdout: {proc.stdout[-500:]}\nstderr: {proc.stderr[-1500:]}"
    )
    assert proc.stdout.strip().endswith("ok")
