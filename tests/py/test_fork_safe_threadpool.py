"""OCCT's global mesher thread pool must survive fork().

BRepMesh(parallel=true) lazily fills OSD_ThreadPool::DefaultPool() with real OS threads. fork()
copies only the calling thread, so without the pthread_atfork handlers in occt_fork_safety.h the
child's next parallel mesh blocks forever in OSD_ThreadPool::WaitIdle(), waiting on workers that
no longer exist. adapy's REST worker forks per job *after* converting, which is exactly this
order, so the regression is a production hang rather than a curiosity.

The test has to fork for real -- the bug only exists across a fork boundary -- so it is
POSIX-only and runs the child through os._exit to keep pytest's state out of it.
"""

import os

import pytest

import adacpp.cad

pytestmark = pytest.mark.skipif(not hasattr(os, "fork"), reason="fork() is POSIX-only")

# NOTE: there is deliberately no "cold parent" case. The pool is process-wide, so once any test
# in this module has meshed the process is warm for every test after it -- a cold case would pass
# or fail on test ORDER rather than on the fix. Every test below therefore warms on purpose.
#
# Generous enough to be a real hang rather than a slow machine: the meshes below are a few ms
# each, and the unfixed failure mode is an unbounded block, not a slowdown.
_CHILD_TIMEOUT_S = 60


def _mesh():
    """Parallel BRepMesh through adacpp -- what populates the global pool."""
    adacpp.cad.tessellate(adacpp.cad.make_cylinder(1.0, 5.0))


def _fork_and_mesh() -> int:
    """Mesh in the child, return its exit status. 0 = the pool worked after fork()."""
    pid = os.fork()
    if pid == 0:  # child
        try:
            _mesh()
            os._exit(0)
        except BaseException:  # noqa: BLE001 - any failure is a non-zero exit for the parent
            os._exit(1)

    # Poll rather than blocking on waitpid so a deadlocked child is a test failure, not a hung suite.
    import time

    deadline = time.monotonic() + _CHILD_TIMEOUT_S
    while time.monotonic() < deadline:
        done, status = os.waitpid(pid, os.WNOHANG)
        if done == pid:
            return status
        time.sleep(0.05)
    os.kill(pid, 9)
    os.waitpid(pid, 0)
    pytest.fail(f"child still meshing after {_CHILD_TIMEOUT_S}s -- OCCT thread pool deadlocked across fork()")


def test_child_can_mesh_after_parent_has_meshed():
    """The regression: a WARM parent (pool populated) forking a child that meshes."""
    _mesh()  # warm the global pool in the parent
    assert os.waitstatus_to_exitcode(_fork_and_mesh()) == 0


def test_parent_still_meshes_after_forking():
    """Parking the pool pre-fork must be restored in the parent, not left at one thread."""
    _mesh()
    assert os.waitstatus_to_exitcode(_fork_and_mesh()) == 0
    _mesh()  # parent's pool is usable again


def test_repeated_forks():
    """The worker forks once per job, so parking/restoring has to be stable across many cycles."""
    _mesh()
    for _ in range(3):
        assert os.waitstatus_to_exitcode(_fork_and_mesh()) == 0
    _mesh()
