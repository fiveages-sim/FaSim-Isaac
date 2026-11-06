"""CLI: ``python -m fasim.replicator.sdg {record,capture,list,describe}``.

Must run under Isaac's Python (``/home/fiveages/isaacsim/python.sh``) because
``omni.replicator`` / ``pxr`` are only available there.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .annotators import describe_annotators, resolve_annotators
from .config_io import (
    CONFIG_ROOT,
    SdgConfigError,
    load_default_config,
    load_robot_config,
    load_task_config,
)


def _cmd_render(args: argparse.Namespace) -> int:
    from isaacsim import SimulationApp

    # 场景里带 OmniGraph script node（我们的 ROS 图），Isaac 默认会弹
    # 「ScriptNode Warning」对话框询问是否允许执行脚本。
    #
    # 在 --gui 模式下这个弹窗会**阻塞 Kit 的消息循环**，于是
    # 渲染线程照常出图（frames/*.npz 内容正确），但视窗完全不重绘 ——
    # 表现就是「脚本没报错、仿真自动播放、但画面一动不动」。
    #
    # 这里在启动前把 opt_in 打开、关掉询问弹窗：
    #   /app/omni.graph.scriptnode/opt_in = true
    #   /app/omni.graph.scriptnode/enable_opt_in = false
    launch_config = {
        "headless": not args.gui,
        "extra_args": [
            "--/app/omni.graph.scriptnode/opt_in=true",
            "--/app/omni.graph.scriptnode/enable_opt_in=false",
        ],
    }
    app = SimulationApp(launch_config=launch_config)
    try:
        from .renderer import render_episode

        root = Path(args.config_root) if args.config_root else CONFIG_ROOT
        robot = load_robot_config(args.robot, root)
        task = load_task_config(
            args.robot, args.task, root,
            output_override=Path(args.output) if args.output else None,
        )
        result = render_episode(
            episode_path=Path(args.episode),
            robot_config=robot,
            task_config=task,
            output_dir=Path(args.output) if args.output else None,
            app=app,
            start=args.start,
            end=args.end,
            steps_per_frame=args.steps_per_frame,
        )
        print("\n" + "=" * 60)
        print(f"output          : {result.output_dir}")
        print(f"frames          : {result.frames_written}")
        print(f"cameras         : {result.cameras}")
        print(f"driven joints   : {len(result.dof_names) - len(result.skipped_dofs)}")
        print(f"skipped (chassis): {result.skipped_dofs}")
        if result.missing_joints:
            print(f"MISSING joints  : {result.missing_joints}")
        print(f"joint max error : {result.joint_max_error * 1000:.3f} mrad")
        p95 = result.extras.get("joint_error_p95")
        if p95 is not None:
            print(f"joint p95 error : {p95 * 1000:.3f} mrad")
        print(f"wall            : {result.wall_seconds:.1f} s")
        print("=" * 60)
        print("\n下一步: 检查质量")
        print(
            f"  python.sh tools/check_quality.py "
            f"--episode {result.episode_path} --render-dir {result.output_dir}"
        )
        return 0
    except Exception:
        # 没有这层的话，异常会先在 finally 里关掉窗口，然后才冒泡到顶层，
        # 表现就是「Isaac 窗口一闪而过、没有任何报错」—— 排查时极易误判。
        import traceback

        print("\n[ERROR] render 失败：", file=sys.stderr)
        traceback.print_exc()
        return 1
    finally:
        app.close()


def _cmd_list(args: argparse.Namespace) -> int:
    root = Path(args.config_root) if args.config_root else CONFIG_ROOT
    if not root.is_dir():
        print(f"config root not found: {root}", file=sys.stderr)
        return 2
    print(f"config root: {root}\n")
    for robot_dir in sorted(p for p in root.iterdir() if p.is_dir() and not p.name.startswith("_")):
        print(f"[{robot_dir.name}]")
        tasks_dir = robot_dir / "tasks"
        if tasks_dir.is_dir():
            for task in sorted(tasks_dir.glob("*.yaml")):
                print(f"  task: {task.stem}")
        else:
            print("  (no tasks/)")
        print()
    return 0


def _cmd_describe(args: argparse.Namespace) -> int:
    root = Path(args.config_root) if args.config_root else CONFIG_ROOT
    robot = load_robot_config(args.robot, root)
    task = load_task_config(args.robot, args.task, root)

    print(f"robot      : {robot.robot_key}")
    print(f"articulation: {robot.articulation_prim}")
    print(f"base       : {robot.base_prim}")
    print(f"joint topic: {robot.joint_state_topic}")
    print(f"EE prims   : {robot.ee_prims}")
    print()
    print(f"task       : {task.task_key}")
    print(f"scene      : {task.scene_usda}")
    print(f"scene exists: {task.scene_usda.is_file()}")
    print(f"tracked    : {task.tracked_prims}")
    print(f"output     : {task.output_dir}")
    print(f"fps        : {task.capture_fps}")
    print()
    print("cameras:")
    for cam in task.cameras:
        kind = f"prim={cam.prim_path}" if cam.prim_path else f"create={cam.create}"
        print(f"  {cam.name:14s} {cam.resolution}  {kind}")
        print(describe_annotators(resolve_annotators(cam.effective_annotators())))
    return 0


def _cmd_record(args: argparse.Namespace) -> int:
    from isaacsim import SimulationApp

    app = SimulationApp(launch_config={"headless": args.headless})
    try:
        from .recorder import record_episode

        root = Path(args.config_root) if args.config_root else CONFIG_ROOT
        robot = load_robot_config(args.robot, root)
        task = load_task_config(
            args.robot, args.task, root,
            output_override=Path(args.output) if args.output else None,
        )
        from .recorder import _apply_variants, _open_stage

        _open_stage(task.scene_usda)
        _apply_variants(robot.articulation_prim, task.variant_selection)
        result = record_episode(
            robot=robot,
            task=task,
            app=app,
            duration_sec=args.duration,
        )
        print(f"\nHDF5: {result.hdf5_path}")
        print(f"frames: {result.frames}")
        return 0
    finally:
        app.close()


def _cmd_capture(args: argparse.Namespace) -> int:
    from isaacsim import SimulationApp

    # --gui: visible window so the replay can be watched; annotators still write.
    app = SimulationApp(launch_config={"headless": not args.gui})
    try:
        from .capturer import capture_session

        root = Path(args.config_root) if args.config_root else CONFIG_ROOT
        robot = load_robot_config(args.robot, root)
        task = load_task_config(
            args.robot, args.task, root,
            output_override=Path(args.output) if args.output else None,
        )
        result = capture_session(
            robot=robot,
            task=task,
            hdf5_path=Path(args.session),
            episode_index=args.episode,
            render_interval=args.render_interval,
        )
        print(f"\noutput: {result.output_dir}")
        print(f"frames: {result.frames_written}")
        print(f"cameras: {result.cameras}")
        if result.warnings:
            print("warnings:")
            for w in result.warnings:
                print(f"  - {w}")
        return 0
    finally:
        app.close()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="fasim.replicator.sdg", description=__doc__)
    p.add_argument("--config-root", default=None, help=f"default: {CONFIG_ROOT}")
    sub = p.add_subparsers(dest="cmd", required=True)

    ls = sub.add_parser("list", help="list configured robots and tasks")
    ls.set_defaults(func=_cmd_list)

    d = sub.add_parser("describe", help="print resolved config (no Isaac launch)")
    d.add_argument("--robot", required=True)
    d.add_argument("--task", required=True)
    d.set_defaults(func=_cmd_describe)

    r = sub.add_parser("record", help="phase 1: record live state to HDF5")
    r.add_argument("--robot", required=True)
    r.add_argument("--task", required=True)
    r.add_argument("--duration", type=float, default=30.0, help="recording seconds")
    r.add_argument("--output", default=None)
    r.add_argument("--headless", action="store_true")
    r.set_defaults(func=_cmd_record)

    c = sub.add_parser("capture", help="phase 2 (legacy): replay HDF5 link poses + render")
    c.add_argument("--robot", required=True)
    c.add_argument("--task", required=True)
    c.add_argument("--session", required=True, help="HDF5 from phase 1")
    c.add_argument("--episode", type=int, default=0)
    c.add_argument("--render-interval", type=int, default=1)
    c.add_argument("--output", default=None)
    c.add_argument(
        "--gui",
        action="store_true",
        help="open a visible window so the replay can be watched (default headless)",
    )
    c.set_defaults(func=_cmd_capture)

    rd = sub.add_parser(
        "render",
        help="phase 2 (current): replay episode NPZ joint angles + render cameras",
    )
    rd.add_argument("--robot", required=True)
    rd.add_argument("--task", required=True)
    rd.add_argument("--episode", required=True, help="episode .npz from the recorder")
    rd.add_argument("--output", default=None, help="override task output_dir")
    rd.add_argument("--start", type=int, default=0, help="first frame (inclusive)")
    rd.add_argument("--end", type=int, default=None, help="last frame (exclusive)")
    rd.add_argument(
        "--steps-per-frame",
        type=int,
        default=2,
        help="app.update() calls per frame; raise if joint tracking lags",
    )
    rd.add_argument(
        "--gui",
        action="store_true",
        help="open a visible window instead of headless (for eyeballing)",
    )
    rd.set_defaults(func=_cmd_render)

    return p


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except SdgConfigError as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
