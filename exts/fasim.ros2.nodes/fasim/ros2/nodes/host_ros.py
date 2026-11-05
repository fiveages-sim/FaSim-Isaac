"""Host-side ROS 2 process helpers shared by FaSim OmniGraph nodes.

Both ``ROS2ControlLaunch`` (in-process ControllerManager via ``emit_control``) and
``NavigationLaunch`` (host ``ros2 launch`` for Nav2) run ROS 2 outside Kit. They
need the same three things, which used to live only inside the control node:

* find the user's ``install/setup.bash`` (Kit strips ``/opt/ros`` from AMENT),
* reset the polluted path variables before sourcing it,
* spawn a process **group** that Stop can tear down.
"""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import sys


def find_workspace_setup(ament_prefix_path: str) -> str:
    """Return install/setup.bash from the first non-/opt/ros ament prefix."""
    for prefix in (ament_prefix_path or "").split(":"):
        prefix = prefix.strip().rstrip("/")
        if not prefix or prefix.startswith("/opt/ros"):
            continue
        parent = os.path.dirname(prefix)
        for candidate in (
            os.path.join(prefix, "setup.bash"),
            os.path.join(parent, "setup.bash") if os.path.basename(parent) == "install" else "",
        ):
            if candidate and os.path.isfile(candidate):
                return candidate
    return ""


def resolve_ros_setup(ros_setup: str, env: dict) -> str:
    setup = (ros_setup or "").strip()
    if setup and os.path.isfile(setup):
        return setup
    return find_workspace_setup(env.get("AMENT_PREFIX_PATH", ""))


def prepare_host_ros_env(env: dict) -> None:
    """Reset path vars so ``source install/setup.bash`` rebuilds overlay order.

    Kit launchers strip ``/opt/ros`` from AMENT (pinocchio/Kit NumPy crash), then
    leave a *partial* prefix list. Re-sourcing setup.bash on top of that can
    insert ``/opt/ros`` *before* some workspace packages. Clearing these lets
    colcon rebuild workspace-first, underlay-last.
    """
    for key in (
        "PYTHONPATH",
        "AMENT_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
        "CMAKE_PREFIX_PATH",
    ):
        env.pop(key, None)


def signal_process_group(proc: subprocess.Popen) -> None:
    """SIGTERM the child process group (best effort)."""
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError, OSError):
        try:
            proc.terminate()
        except Exception:
            pass


def host_python() -> str:
    """Host interpreter for ROS entry points (never Kit's bundled python)."""
    python = shutil.which("python3") or sys.executable
    if "isaacsim" in (python or "") or "kit/python" in (python or ""):
        candidate = shutil.which("python3", path="/usr/bin:/bin") or "/usr/bin/python3"
        if os.path.isfile(candidate):
            return candidate
    return python


def build_sourced_command(
    argv: list[str],
    *,
    ros_setup: str = "",
    env: dict | None = None,
) -> tuple[list[str], dict]:
    """Build a ``bash -c 'source <setup> && exec <argv>'`` command + env.

    ``env`` is copied and (when a setup script is used) has the Kit-polluted path
    variables cleared so ``setup.bash`` rebuilds them workspace-first.
    """
    environ = dict(env if env is not None else os.environ)
    setup = resolve_ros_setup(ros_setup, environ)
    if setup:
        prepare_host_ros_env(environ)
        quoted = subprocess.list2cmdline([str(a) for a in argv])
        return ["bash", "-c", f'source "{setup}" && exec {quoted}'], environ
    if environ.get("AMENT_PREFIX_PATH"):
        return [str(a) for a in argv], environ
    raise RuntimeError(
        "Cannot find ros2_ws install/setup.bash; source the workspace or set "
        "inputs:rosSetup (AMENT_PREFIX_PATH alone is not enough inside Kit)"
    )


__all__ = [
    "build_sourced_command",
    "find_workspace_setup",
    "host_python",
    "prepare_host_ros_env",
    "resolve_ros_setup",
    "signal_process_group",
]
