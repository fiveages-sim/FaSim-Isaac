"""一键起编排 —— 从 sdg 侧拉起 motion-generation 跑一遍任务。

为什么放在这里
==============
日常数采的操作顺序是::

    ./run.sh                      # 1. 起 Isaac（运控随之进程内启动）
    ./sdg.sh randomize            # 2. 随机化物体
    ./sdg.sh capture              # 3. 开始采集
    cd ~/lerobot_ros2 && motion-generation ...   # 4. 切仓库跑编排
    ./sdg.sh stop                 # 5. 回 sdg 停采集

第 4 步要切仓库、source venv、敲一长串参数 —— 每次采集都来一遍，还容易
忘记 `--no-reset`（会导致编排把刚随机化的物体冲掉）。所以把这一步收进
sdg：`./sdg.sh motion`。

关键约束
========
1. **必须 ``--no-reset``**
   编排默认会 reset 环境（timeline STOP+PLAY），那会把随机化的物体
   打回 USD 初始位。采集循环依赖「随机化 → 不动场景 → 跑任务」。

2. **不自己起 ROS 栈**
   ``robots/Unitree/G1D/robot.yaml`` 里 ``ros2_stack.motion.required: false``
   —— 运控是随 Isaac 的 Play 在**进程内**拉起的。再加
   ``--ensure-ros2-stack`` 会启动第二套控制器，抢同一个 domain。
   所以固定传 ``--no-ensure-ros2-stack``。

3. **要在 lerobot_ros2 的 venv 里跑**
   ``motion-generation`` 是那个 venv 的 console script，且需要
   ``PYTHONPATH`` 含子模块。这里用 ``bash -lc`` 重新进一个 shell，
   先 source venv 再执行 —— 不要试图在 sdg 的 python 进程里 import。

4. **用子进程而不是同进程**
   编排会长时间阻塞（跑完整个 task_queue，几分钟），且自己装 SIGINT
   处理器。放子进程里，sdg 侧仍能正常响应 status/stop；用户 Ctrl+C
   也只是中断编排，不会连累 Isaac。

调用方式（实测可用的那一条）
============================
::

    cd ~/lerobot_ros2
    source .venv/bin/activate
    motion-generation --robot unitree_g1d --task-key pick_tube_full \\
        --scene default --no-reset --no-ensure-ros2-stack

``--workspace`` 默认取当前目录（``resolve_workspace_dir(None)`` = cwd），
所以在 lerobot_ros2 根目录执行即可。
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

#: 默认的编排仓库位置（用户环境）。
DEFAULT_REPO = Path.home() / "lerobot_ros2"

#: sdg 的 robot_key / task_key → 编排的 robot / task-key。
#:
#: 两边是**两套独立的任务定义**：
#:   sdg:      configs/<robot>/tasks/<task>.yaml          （采集/场景/相机）
#:   编排:      robots/<Robot>/task_configs/**/<task>.yaml （技能序列）
#: 名字不一定相同，所以显式映射。没有映射时按同名处理。
DEFAULT_MAP: dict[tuple[str, str], tuple[str, str]] = {
    ("unitree_g1d", "sinopec_test_tube"): ("unitree_g1d", "pick_tube_full"),
}


def resolve_target(robot_key: str, task_key: str) -> tuple[str, str]:
    """sdg 的 (robot, task) → 编排的 (--robot, --task-key)。"""
    return DEFAULT_MAP.get((robot_key, task_key), (robot_key, task_key))


def find_repo(explicit: str = "") -> Path | None:
    """定位编排仓库：显式路径 > 环境变量 > 默认 ~/lerobot_ros2。"""
    cands = []
    if str(explicit or "").strip():
        cands.append(Path(str(explicit)).expanduser())
    env = os.environ.get("LEROBOT_ROS2_DIR", "").strip()
    if env:
        cands.append(Path(env).expanduser())
    cands.append(DEFAULT_REPO)
    for p in cands:
        if (p / "submodules" / "robot_action_composer").is_dir():
            return p.resolve()
    return None


def find_activate(repo: Path) -> Path | None:
    """venv 的 activate 脚本（``.venv/bin/activate``）。"""
    for rel in (".venv/bin/activate", ".venv-train/bin/activate"):
        p = repo / rel
        if p.is_file():
            return p
    return None


def find_workspace(repo: Path, robot_key: str) -> Path | None:
    """定位编排的 workspace —— **必须含 ``robots/<...>/robot.yaml`` 且 key 匹配**。

    为什么不能只用仓库根目录（踩过）：
    编排的 ``--workspace`` 默认取 cwd（``resolve_workspace_dir(None)``）。
    在 ``~/lerobot_ros2`` 根目录跑会报::

        ValueError: Unknown robot key: 'unitree_g1d'

    因为机器人定义在 ``examples/IsaacSim/robots/`` 下，而不是仓库根目录。
    更坑的是**根目录也有个 ``robots/Dobot/CR5``** —— 从根目录跑能找到
    一个"workspace"，但里面没有我们的机器人，报错信息还看不出是路径问题。

    所以这里显式找「哪个目录下有 ``robots/**/robot.yaml`` 且 ``key:`` 等于
    目标机器人」，而不是猜一个固定相对路径。

    候选顺序（先精确后宽松）：
      1. ``<repo>/examples/IsaacSim``     ← 本仓库的实际布局
      2. ``<repo>``                       ← 兼容 robots_root 直接放根目录的布局
    """
    cands = [repo / "examples" / "IsaacSim", repo]
    for ws in cands:
        rb = ws / "robots"
        if not rb.is_dir():
            continue
        # 扫一层子目录（Unitree/G1D、FiveAges_W2/... 等）
        for yml in rb.glob("*/robot.yaml"):
            try:
                txt = yml.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            for line in txt.splitlines():
                if line.strip().startswith("key:"):
                    got = line.split(":", 1)[1].strip().strip("\"'")
                    if got == robot_key:
                        return ws.resolve()
                    break
        # 也支持 robots/<Robot>/<variant>/robot.yaml（如 Unitree/G1D）
        for yml in rb.glob("*/*/robot.yaml"):
            try:
                txt = yml.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            for line in txt.splitlines():
                if line.strip().startswith("key:"):
                    got = line.split(":", 1)[1].strip().strip("\"'")
                    if got == robot_key:
                        return ws.resolve()
                    break
    return None


def build_command(
    *,
    workspace: Path,
    robot: str,
    task_key: str,
    scene: str = "default",
    extra_args: list[str] | None = None,
) -> list[str]:
    """拼出 bash 命令行（**不含** venv activate —— 由调用方包一层）。

    **注意**：只有「切目录 / 执行」这几段用 ``&&`` 连接；
    ``motion-generation`` 的参数之间必须是空格 —— 曾经把参数也 join 进
    ``&&``，结果变成 ``motion-generation && --robot xxx``，等于先跑一个
    无参的 motion-generation（进交互模式）再执行 ``--robot`` 这个不存在的
    命令。
    """
    arg_list = [
        "motion-generation",
        f"--robot {shlex_quote(robot)}",
        f"--task-key {shlex_quote(task_key)}",
    ]
    if str(scene or "").strip():
        arg_list.append(f"--scene {shlex_quote(scene)}")
    # 两个关键开关：不 reset（保住随机化）、不起第二套 ROS 栈
    arg_list.append("--no-reset")
    arg_list.append("--no-ensure-ros2-stack")
    if extra_args:
        arg_list.extend(shlex_quote(a) for a in extra_args)

    # --workspace 显式传，不依赖 cwd —— 否则必须 cd 到 examples/IsaacSim
    # 才能找到机器人定义（见 find_workspace 的说明）。
    arg_list.append(f"--workspace {shlex_quote(str(workspace))}")
    return arg_list


def shlex_quote(s: str) -> str:
    import shlex

    return shlex.quote(str(s))


def preflight(
    *,
    repo_arg: str = "",
    robot_key: str,
    task_key: str,
    stream=None,
) -> tuple[bool, dict]:
    """检查能不能起编排。返回 ``(ok, info)``，info 里带定位结果与错误原因。"""
    out = stream or sys.stdout
    repo = find_repo(repo_arg)
    if repo is None:
        print(
            "[motion] ERROR 找不到编排仓库（要含 submodules/robot_action_composer）。\n"
            f"        试过: {repo_arg or '(未指定)'}, "
            f"$LEROBOT_ROS2_DIR, {DEFAULT_REPO}\n"
            "        用 --repo <path> 或设 LEROBOT_ROS2_DIR 指定。",
            file=out,
        )
        return False, {"repo": None}

    robot, task = resolve_target(robot_key, task_key)

    ws = find_workspace(repo, robot)
    if ws is None:
        print(
            f"[motion] ERROR {repo} 里找不到 key: {robot} 的机器人定义。\n"
            "        编排的 --workspace 必须指向含 robots/**/robot.yaml 的目录\n"
            f"        （本仓库是 {repo}/examples/IsaacSim）。\n"
            "        用 --workspace 显式指定，或检查 robot.yaml 里的 key 是否正确。",
            file=out,
        )
        return False, {"repo": repo, "robot": robot}

    activate = find_activate(repo)
    info = {
        "repo": str(repo),
        "workspace": str(ws),
        "activate": str(activate) if activate else "",
        "robot": robot,
        "task_key": task,
        "sdg_robot": robot_key,
        "sdg_task": task_key,
    }
    return True, info


def run_motion(
    *,
    repo_arg: str = "",
    robot_key: str,
    task_key: str,
    scene: str = "default",
    extra_args: list[str] | None = None,
    background: bool = False,
    log_path: str = "",
) -> int:
    """起编排。background=True 时阻塞到跑完；background=True 时立即返回。

    Returns:
        进程退出码（前台模式）；后台模式返回 0（启动成功）。
    """
    ok, info = preflight(repo_arg=repo_arg, robot_key=robot_key, task_key=task_key)
    if not ok:
        return 2

    args = build_command(
        workspace=Path(info["workspace"]),
        robot=info["robot"],
        task_key=info["task_key"],
        scene=scene,
        extra_args=extra_args,
    )
    cmd = _wrap_shell(info, args)

    print("[motion] 编排目标: "
          f"{info['sdg_robot']}/{info['sdg_task']} → "
          f"{info['robot']}/{info['task_key']}")
    print(f"[motion] 仓库: {info['repo']}")
    print(f"[motion] 工作区: {info['workspace']}"
          f"{'' if info['activate'] else '  (无 venv，用当前 python)'}")
    print("[motion] 关键开关: --no-reset（保住随机化）"
          " --no-ensure-ros2-stack（运控随 Isaac 进程内启动）")

    if background:
        log = Path(log_path).expanduser() if log_path else Path("/tmp/motion_generation.log")
        log.parent.mkdir(parents=True, exist_ok=True)
        fh = log.open("a", encoding="utf-8")
        proc = subprocess.Popen(
            cmd,
            stdout=fh,
            stderr=subprocess.STDOUT,
            start_new_session=True,   # 脱离当前进程组，Ctrl+C 不误伤
        )
        print(f"[motion] 已后台启动（pid {proc.pid}），日志: {log}")
        print(f"[motion] 跟踪进度: tail -f {log}")
        return 0

    print("[motion] 前台运行，Ctrl+C 可中断（不会影响 Isaac）\n" + "=" * 60)
    proc = None
    try:
        proc = subprocess.Popen(cmd)
        return proc.wait()
    except KeyboardInterrupt:
        print("\n[motion] 用户中断")
        try:
            if proc is not None:
                proc.terminate()
        except Exception:  # noqa: BLE001
            pass
        return 130


def _wrap_shell(info: dict, args: list[str]) -> list[str]:
    """把编排参数包成 bash 命令（切目录 + source venv + 执行）。

    两件事必须在这层做，而不是让用户自己准备环境：

    1. **source venv**：``motion-generation`` 是那个 venv 的 console script。
       实测：venv 里有 ``robot_action_composer``、``rclpy`` 等依赖，系统
       python 没有。所以不能指望外部 shell 已经激活对了环境。

    2. **切到 repo 根目录**：``motion-generation`` 是相对路径调用的
       console script，且它内部会用相对路径找 ``submodules/``。

    ``--workspace`` 已经显式传了（见 build_command），所以**不依赖 cwd
    来定位机器人定义** —— 但仍需要 cd 到 repo 才能找到 venv。
    """
    steps = [f"cd {shlex_quote(info['repo'])}"]
    if info.get("activate"):
        steps.append(f"source {shlex_quote(info['activate'])}")
    steps.append(" ".join(args))
    return ["bash", "-lc", " && ".join(steps)]


def is_running() -> list[int]:
    """找出正在跑的编排进程（用于提醒"已经在跑了"）。"""
    try:
        r = subprocess.run(
            ["pgrep", "-f", "motion-generation|cli.motion_main"],
            capture_output=True, text=True, timeout=5,
        )
        return [int(x) for x in r.stdout.split() if x.isdigit()]
    except Exception:  # noqa: BLE001
        return []


def resolve_sdg_python() -> str:
    """sdg 侧用的 python（只为文档一致，实际不需要）。"""
    return shutil.which("python3") or sys.executable
