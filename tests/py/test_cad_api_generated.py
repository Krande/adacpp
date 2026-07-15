"""``adacpp/cad/__init__.py`` must match the compiled extension, on every build target.

That module re-exports the C++ ``cad`` bindings into Python. It used to be two hand-maintained lists
(assignments + ``__all__``) and both had rotted:

  * 15 names were assigned but absent from ``__all__``, so ``from adacpp.cad import *`` silently
    omitted working functions;
  * ``glb_diff`` was re-exported unconditionally after its binding was compiled out of the wasm build
    — an ``AttributeError`` at *import* that took the whole ``adacpp`` package down under pyodide,
    and with it ``ada.cad.select_backend()``;
  * a new binding stayed invisible from Python until someone edited two places.

It is now generated (``pixi run gen-cad-api``). These tests are the guarantee: add a binding and
forget to regenerate, and CI says so instead of a user finding out.

These run on the NATIVE build. The wasm-surface tests below simulate a missing binding rather than
requiring emscripten, so the import-time regression is caught in the ordinary test run;
``tools/test_wheel_pyodide.js`` is the real-artifact counterpart.
"""

from __future__ import annotations

import importlib
import importlib.util
import pathlib
import re
import types

import pytest

import adacpp.cad as cad
from adacpp._ada_cpp_ext_impl import cad as _cad

_ROOT = pathlib.Path(__file__).resolve().parents[2]
_PUBLIC = {n for n in dir(_cad) if not n.startswith("_")}


def _load_generator():
    """Import tools/gen_cad_api.py by path.

    Deliberately NOT ``pytest.importorskip("tools.gen_cad_api")``: that silently skips unless the repo
    root happens to be on sys.path, which turned the staleness gate into a no-op in CI — the exact
    failure mode (a gate that passes by not running) this module exists to prevent. If the generator
    is genuinely missing, fail; do not skip.
    """
    path = _ROOT / "tools" / "gen_cad_api.py"
    assert path.exists(), f"generator missing at {path}"
    spec = importlib.util.spec_from_file_location("_gen_cad_api", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _optional_names() -> set[str]:
    """Bindings the C++ compiles only into the native build."""
    return _load_generator().native_only_bindings((_ROOT / "src" / "cad" / "cad_py_wrap.cpp").read_text())


def test_every_binding_is_re_exported():
    """A new C++ binding must be reachable as adacpp.cad.<name> without a second edit."""
    missing = sorted(n for n in _PUBLIC if not hasattr(cad, n))
    assert not missing, f"bindings not re-exported (run `pixi run gen-cad-api`): {missing}"


def test_all_matches_the_re_exports():
    """__all__ and the assignments are one list, not two that can drift."""
    exported = set(cad.__all__)
    assert exported == _PUBLIC, (
        f"__all__ does not match the extension (run `pixi run gen-cad-api`). "
        f"missing: {sorted(_PUBLIC - exported)} extra: {sorted(exported - _PUBLIC)}"
    )


def test_re_exports_are_identity_on_native():
    """Each export IS the binding — this module adds no wrappers, so nothing can subtly differ.

    Holds for the optional ones too on a native build: _optional() returns the real attribute when it
    is present, and only substitutes a stub when it is not.
    """
    for n in cad.__all__:
        assert getattr(cad, n) is getattr(_cad, n), f"{n} is not an identity re-export"


def test_generated_file_is_not_stale():
    """The checked-in file must equal what the generator emits for this extension."""
    gen = _load_generator()
    out = _ROOT / "src" / "adacpp" / "cad" / "__init__.py"
    assert out.read_text() == gen.generate(), "adacpp/cad/__init__.py is stale — run `pixi run gen-cad-api`"


def test_native_only_bindings_are_detected():
    """The guard parser must actually find something.

    If it silently returned an empty set (a bad regex, a moved file, a refactor of the #ifndef), every
    other wasm test here would vacuously pass while the real bug walked straight through. glb_diff is
    the known native-only binding; this pins that detection works at all.
    """
    assert "glb_diff" in _optional_names(), "the #ifndef __EMSCRIPTEN__ parser found no native-only bindings"


def test_conditional_bindings_are_not_bound_flat():
    """A binding compiled out of the wasm build must not be re-exported flat.

    Regression: `glb_diff = _cad.glb_diff` raised AttributeError at import under pyodide and killed the
    whole module. The generator emits _optional() for these; this pins that it keeps doing so.
    """
    src = (_ROOT / "src" / "adacpp" / "cad" / "__init__.py").read_text()
    for name in _optional_names():
        assert (
            f"\n{name} = _cad.{name}\n" not in src
        ), f"{name} is native-only but bound flat — that is an AttributeError at import on wasm"
        # Matched loosely across newlines: the generator emits the exploded, black-canonical call, and
        # pinning the exact layout here would just re-fight black every time a reason changes length.
        assert re.search(
            rf"^{re.escape(name)} = _optional\(\s*\"{re.escape(name)}\",", src, re.M
        ), f"{name} should be bound via _optional()"


def test_import_survives_a_missing_binding(monkeypatch):
    """THE regression test: importing must not depend on a binding the build may not have.

    Simulates the wasm build by deleting the native-only attributes and re-importing. Before the fix
    this raised at import; the whole package went down with it. Asserting on the reloaded module
    rather than a fresh interpreter keeps this a plain unit test.
    """
    for name in _optional_names():
        monkeypatch.delattr(_cad, name, raising=False)
    mod = importlib.reload(importlib.import_module("adacpp.cad"))
    try:
        # The surface is unchanged: the names are still bound, still in __all__.
        assert set(mod.__all__) == _PUBLIC, "__all__ must not depend on the build target"
        for name in _optional_names():
            assert hasattr(mod, name), f"{name} must stay bound (as a stub) so the surface is stable"
            with pytest.raises(NotImplementedError, match=name):
                getattr(mod, name)()
        # And the bindings that ARE present still work — the point of not dying at import.
        assert mod.tessellate_box is _cad.tessellate_box
    finally:
        importlib.reload(mod)  # restore the real bindings for the rest of the session


def test_stub_explains_itself():
    """A deferred failure is only better than AttributeError if it says why."""
    gen = _load_generator()
    src = (_ROOT / "src" / "adacpp" / "cad" / "__init__.py").read_text()
    for name in _optional_names():
        reason = gen._REASONS.get(name, gen._GENERIC_REASON)
        assert reason in src, f"{name}'s stub must carry a reason a reader can act on"
        assert len(reason) > 20, "the reason must be a sentence, not a shrug"


def _exec_generated_as_wasm():
    """Execute the generated module against a fake extension missing the native-only bindings.

    This is how the wasm build sees it. Done by exec rather than by purging sys.modules and
    re-importing: re-initialising a nanobind extension aborts the interpreter outright (verified —
    it takes pytest down with SIGABRT, not a failure), so a "does it import" test written the obvious
    way cannot run at all. Exec'ing the real file's source against a stand-in _cad tests the same
    code with no global mutation.
    """
    src = (_ROOT / "src" / "adacpp" / "cad" / "__init__.py").read_text()
    imp = "from .._ada_cpp_ext_impl import cad as _cad"
    assert imp in src, "generated import line changed — this fake-injection is no longer faithful"
    fake = types.SimpleNamespace(**{n: getattr(_cad, n) for n in _PUBLIC - _optional_names()})
    ns: dict = {"_cad": fake}
    exec(compile(src.replace(imp, ""), "<generated adacpp.cad as wasm>", "exec"), ns)
    return ns


def test_generated_module_executes_without_the_native_only_bindings():
    """THE regression test: the module must not fail to load when a binding is compiled out.

    `glb_diff = _cad.glb_diff` raised AttributeError here at import under pyodide and took the whole
    adacpp package down — and with it ada.cad.select_backend(), so a caller could not reach the
    operations that WERE present.
    """
    ns = _exec_generated_as_wasm()
    assert set(ns["__all__"]) == _PUBLIC, "__all__ must not depend on the build target"
    for name in _optional_names():
        assert name in ns, f"{name} must stay bound (as a stub) so the surface is build-independent"
        with pytest.raises(NotImplementedError, match=name):
            ns[name]()
    # The bindings that ARE present must still be the real ones — that is the point of not dying.
    assert ns["tessellate_box"] is _cad.tessellate_box
