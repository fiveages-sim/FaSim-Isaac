"""OmniGraph node: launch Nav2 as a host ``ros2 launch`` on Play.

Companion to :mod:`OgnROS2ControlLaunch`. The scene USD owns the *what* -- which
map, which Nav2 profile, which topics -- while this node owns the *how*: build the
command, source the ROS workspace, and keep the process group alive until Stop.

The map is a USD ``asset`` input, so ``@@../../maps/<name>/map.yaml@@`` in an
``env/`` layer points at files stored inside the fa-project-usd project itself.
"""

from __future__ import annotations

import atexit
import os
import shlex
import signal
import subprocess
import threading
import time

import carb
import omni.graph.core as og
import omni.timeline

from fasim.ros2.nodes.host_ros import build_sourced_command, signal_process_group

_DEFAULT_LOG_DIR = "/tmp/fasim"
_DEFAULT_LOG_NAME = "nav2.log"
_REAP_TIMEOUT_S = 5.0
_PID_FILE_NAME = "nav2.pid"
_STALE_KILL_TIMEOUT_S = 3.0


def _root_layer_dir() -> str:
    """Directory of the opened stage's root layer (asset paths resolve against it)."""
    try:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return ""
        root = stage.GetRootLayer()
        identifier = root.realPath or root.identifier or ""
        if not identifier:
            return ""
        return os.path.dirname(os.path.abspath(identifier))
    except Exception:
        return ""


def _resolve_path(value: object) -> str:
    """Absolute filesystem path for a path input (``""`` when unset).

    Relative paths are resolved against the opened stage's root layer, so an
    ``env/`` layer can write ``../../maps/<name>/map.yaml``.
    """
    if value is None:
        return ""
    raw = getattr(value, "path", None)
    if raw is None:
        raw = value
    text = str(raw).strip()
    if not text:
        return ""
    text = os.path.expanduser(text)
    if os.path.isabs(text):
        return os.path.normpath(text)
    base = _root_layer_dir()
    if base:
        return os.path.normpath(os.path.join(base, text))
    return os.path.abspath(text)


def _robot_from_prim(prim_path: str) -> str:
    """Read the ``Robot`` variant from ``prim_path`` or its ancestors (W2 legacy alias)."""
    if not prim_path:
        return ""
    try:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return ""
        path = prim_path
        while path and path != "/":
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                vsets = prim.GetVariantSets()
                if vsets.HasVariantSet("Robot"):
                    selection = vsets.GetVariantSet("Robot").GetVariantSelection()
                    if selection:
                        return "fiveages_w2" if selection == "robot" else selection
            path = path.rsplit("/", 1)[0]
    except Exception:
        return ""
    return ""


def _bool_arg(value: object) -> str:
    return "true" if bool(value) else "false"


def _build_argv(db: og.Database) -> tuple[list[str], str]:
    """Assemble ``ros2 launch`` argv (only arguments the launch file declares)."""
    package = str(db.inputs.launchPackage or "").strip() or "robot_common_launch"
    launch_file = str(db.inputs.launchFile or "").strip() or "navigation_isaac_gt.launch.py"

    map_path = _resolve_path(db.inputs.map)
    params_path = _resolve_path(db.inputs.paramsFile)
    robot = str(db.inputs.robot or "").strip()
    if not robot:
        targets = db.inputs.targetPrim
        robot = _robot_from_prim(str(targets[0]) if targets else "")

    argv = ["ros2", "launch", package, launch_file]
    if robot:
        argv.append(f"robot:={robot}")
    if map_path:
        argv.append(f"map:={map_path}")
    if params_path:
        argv.append(f"params_file:={params_path}")
    argv.append(f"nav2_profile:={str(db.inputs.nav2Profile or 'map_only').strip()}")
    argv.append(f"scan_topic:={str(db.inputs.scanTopic or '/scan').strip()}")
    argv.append(f"odom_topic:={str(db.inputs.odomTopic or '/odom').strip()}")
    argv.append(f"use_sim_time:={_bool_arg(db.inputs.useSimTime)}")
    argv.append(f"use_rviz:={_bool_arg(db.inputs.useRviz)}")
    argv.append(f"publish_map_odom_tf:={_bool_arg(db.inputs.publishMapOdomTf)}")
    argv.append(f"publish_map_world_tf:={_bool_arg(db.inputs.publishMapWorldTf)}")

    extra = str(db.inputs.extraArgs or "").strip()
    if extra:
        argv.extend(shlex.split(extra))

    log_path = str(db.inputs.logFile or "").strip()
    if log_path:
        log_path = os.path.expanduser(log_path)
    else:
        log_dir = os.path.dirname(map_path) if map_path else _DEFAULT_LOG_DIR
        log_path = os.path.join(log_dir or _DEFAULT_LOG_DIR, _DEFAULT_LOG_NAME)
    return argv, log_path


def _pid_file_path(log_path: str) -> str:
    """PID file next to the log, so a later session can reap a leaked launch."""
    return os.path.join(os.path.dirname(log_path) or _DEFAULT_LOG_DIR, _PID_FILE_NAME)


def _read_pid(path: str) -> int | None:
    try:
        with open(path, encoding="utf-8") as handle:
            return int(handle.read().strip() or "0") or None
    except (OSError, ValueError):
        return None


def _proc_cmdline(pid: int) -> str:
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as handle:
            return handle.read().replace(b"\0", b" ").decode("utf-8", "replace").strip()
    except OSError:
        return ""


def _unlink_quietly(path: str) -> None:
    try:
        os.unlink(path)
    except OSError:
        pass


def _kill_stale_launch(pid_path: str) -> None:
    """Kill a Nav2 launch group leaked by a previous Isaac session.

    Closing the Isaac window (or a crash) skips timeline STOP, so the launched
    ``ros2 launch`` keeps a CycloneDDS participant alive. Enough of them and new
    nodes fail with "Failed to find a free participant index for domain 0".
    """
    pid = _read_pid(pid_path)
    if not pid:
        return
    command = _proc_cmdline(pid)
    if not command:
        _unlink_quietly(pid_path)
        return
    if "navigation_isaac_gt.launch.py" not in command and "nav2" not in command:
        _unlink_quietly(pid_path)
        return

    carb.log_warn(f"NavigationLaunch: killing stale Nav2 launch pid={pid} from a previous session")
    try:
        os.killpg(pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError, OSError):
        _unlink_quietly(pid_path)
        return
    deadline = time.monotonic() + _STALE_KILL_TIMEOUT_S
    while time.monotonic() < deadline:
        if not _proc_cmdline(pid):
            break
        time.sleep(0.1)
    else:
        try:
            os.killpg(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError, OSError):
            pass
    _unlink_quietly(pid_path)


def _reap(proc: subprocess.Popen, log_file, pid_path: str) -> None:
    """Wait for Nav2 to exit; escalate to SIGKILL if it ignores SIGTERM."""
    try:
        proc.wait(timeout=_REAP_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError, OSError):
            try:
                proc.kill()
            except Exception:
                pass
        try:
            proc.wait(timeout=2.0)
        except Exception:
            pass
    finally:
        if log_file is not None:
            try:
                log_file.close()
            except Exception:
                pass
        _unlink_quietly(pid_path)


class OgnNavigationLaunchInternalState:
    def __init__(self) -> None:
        self._proc: subprocess.Popen | None = None
        self._log_file = None
        self._log_path = ""
        self._pid_path = ""
        self._command = ""
        self._timeline_sub = None
        self._needs_reconfigure = False
        self._lock = threading.Lock()
        atexit.register(self.stop)

    def ensure_timeline_subscription(self) -> None:
        if self._timeline_sub is not None:
            return
        timeline = omni.timeline.get_timeline_interface()
        if timeline is None:
            carb.log_warn("NavigationLaunch: omni.timeline unavailable")
            return
        stream = timeline.get_timeline_event_stream()
        if stream is None:
            carb.log_warn("NavigationLaunch: timeline event stream unavailable")
            return
        self._timeline_sub = stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP),
            self._on_stop,
        )

    def _on_stop(self, _event) -> None:
        self.stop()
        self._needs_reconfigure = True

    def is_running(self) -> bool:
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    def start(self, argv: list[str], *, log_path: str, ros_setup: str) -> None:
        self.stop()
        log_dir = os.path.dirname(log_path) or _DEFAULT_LOG_DIR
        os.makedirs(log_dir, exist_ok=True)
        pid_path = _pid_file_path(log_path)
        # Isaac was killed/closed last time? Reap its Nav2 before adding another
        # DDS participant (CycloneDDS runs out of participant indices).
        _kill_stale_launch(pid_path)
        cmd, env = build_sourced_command(argv, ros_setup=ros_setup, env=os.environ.copy())

        log_file = open(log_path, "a", encoding="utf-8", buffering=1)
        command = " ".join(shlex.quote(part) for part in argv)
        log_file.write(f"\n=== NavigationLaunch {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n$ {command}\n")
        log_file.flush()

        proc = subprocess.Popen(
            cmd,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            env=env,
            start_new_session=True,
        )
        try:
            with open(pid_path, "w", encoding="utf-8") as handle:
                handle.write(str(proc.pid))
        except OSError as exc:
            carb.log_warn(f"NavigationLaunch: cannot write {pid_path}: {exc}")
        with self._lock:
            self._proc = proc
            self._log_file = log_file
            self._log_path = log_path
            self._pid_path = pid_path
            self._command = command
        carb.log_info(f"NavigationLaunch: pid={proc.pid} log={log_path}\n  {command}")

    def stop(self) -> None:
        with self._lock:
            proc, log_file = self._proc, self._log_file
            pid_path = self._pid_path
            self._proc = None
            self._log_file = None
            self._pid_path = ""
        if proc is None:
            if log_file is not None:
                try:
                    log_file.close()
                except Exception:
                    pass
            return
        carb.log_info(f"NavigationLaunch: stopping pid={proc.pid}")
        signal_process_group(proc)
        threading.Thread(
            target=_reap,
            args=(proc, log_file, pid_path),
            name="fasim_nav2_reap",
            daemon=True,
        ).start()


class OgnNavigationLaunch:
    @staticmethod
    def internal_state() -> OgnNavigationLaunchInternalState:
        return OgnNavigationLaunchInternalState()

    @staticmethod
    def compute(db: og.Database) -> bool:
        state = db.per_instance_state
        state.ensure_timeline_subscription()

        if state._needs_reconfigure:
            db.state.configured = False
            state._needs_reconfigure = False

        if db.state.configured:
            return True

        if not bool(db.inputs.enabled):
            state.stop()
            db.state.configured = True
            db.outputs.command = ""
            db.outputs.logPath = ""
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED
            return True

        try:
            argv, log_path = _build_argv(db)
            state.start(argv, log_path=log_path, ros_setup=str(db.inputs.rosSetup or ""))
        except Exception as exc:
            carb.log_error(f"NavigationLaunch start failed: {exc}")
            return False

        db.state.configured = True
        db.outputs.command = state._command
        db.outputs.logPath = log_path
        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        return True
