"""交互式主菜单 —— 不记参数，靠选择。

为什么不继续用子命令
====================
子命令 ``sdg.sh capture --what camera,joint,ee,object --camera all ...`` 适合
脚本/CI，但对日常调试不友好：

- 参数多且互相依赖（选了 camera 才该问相机名、才该问分辨率）
- 记不住上次选了什么，每次重敲
- 「转化格式」这种要先选会话目录的，得先 `ls` 再复制路径

所以主入口改成菜单式：先选**机器人和任务**（形成隔离的任务包上下文），
再选**功能**，每个功能进各自的分支，只问该问的。选择记在
``~/.cache/fasim_sdg/menu_state.json``，下次回车就沿用。

子命令仍然保留 —— 两者共用同一套底层函数，参数化入口适合自动化。

菜单结构
========
::

    主菜单
    ├── 采集        capture  （选采集项/相机/频率/输出）
    ├── 查看状态    status
    ├── 结束采集    stop
    ├── 转数据      pack / trim / shots / diag / fk-diag
    ├── 回放        live（关节 / 末端）
    ├── 自检        doctor
    └── 切换机器人/任务

上下文（机器人/任务）隔离
========================
所有功能都在「机器人 + 任务」的上下文里跑，对应 ``configs/<robot>/`` 目录。
迁移到新机器人＝新建目录 + 填配置，不需要改代码（见 README）。
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
EXT_ROOT = SCRIPT_DIR.parent
SDG_SH = SCRIPT_DIR / "sdg.sh"

try:
    sys.path.insert(0, str(EXT_ROOT))
    from fasim.replicator.sdg.config_io import CONFIG_ROOT, load_robot_config
    from fasim.replicator.sdg.config_io import load_task_config
    from fasim.replicator.sdg import runtime
except Exception as exc:  # noqa: BLE001
    print(f"[menu] ERROR 无法加载 sdg 配置模块: {exc}")
    print(f"       检查 PYTHONPATH 是否包含 {EXT_ROOT}")
    raise SystemExit(1)


STATE_PATH = Path.home() / ".cache" / "fasim_sdg" / "menu_state.json"

BOLD = "\033[1m"
DIM = "\033[2m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
CYAN = "\033[36m"
NC = "\033[0m"


def _c(text: str, color: str) -> str:
    return f"{color}{text}{NC}" if sys.stdout.isatty() else text


def _log(msg: str) -> None:
    print(f"{_c('[menu]', CYAN)} {msg}", flush=True)


# ---------------------------------------------------------------- 状态记忆


def load_state() -> dict[str, Any]:
    try:
        return json.loads(STATE_PATH.read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001
        return {}


def save_state(state: dict[str, Any]) -> None:
    try:
        STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
        STATE_PATH.write_text(json.dumps(state, indent=2, ensure_ascii=False),
                              encoding="utf-8")
    except Exception:  # noqa: BLE001
        pass


# ---------------------------------------------------------------- 交互原语


def ask(prompt: str, default: str = "") -> str:
    suffix = f" [{default}]" if default else ""
    try:
        ans = input(f"{prompt}{suffix}: ")
    except (EOFError, KeyboardInterrupt):
        print()
        raise SystemExit(0)
    return ans.strip() or default


def ask_yes(prompt: str, default: bool = True) -> bool:
    d = "Y/n" if default else "y/N"
    ans = ask(f"{prompt} [{d}]", "y" if default else "n").lower()
    return ans in ("y", "yes", "是")


def pick(title: str, options: list[tuple[str, str]], default_idx: int = 1) -> str:
    """options: [(value, 显示文本)]。返回 value。

    除了序号，也接受直接输入 value（如 ``quit``）—— 交互时手快更顺手。
    """
    if not options:
        raise SystemExit(f"[menu] ERROR {title}: 没有可选项")
    print(f"\n{_c(title, BOLD)}")
    for i, (_v, desc) in enumerate(options, 1):
        mark = _c("  (上次)", DIM) if i == default_idx else ""
        print(f"  {i}) {desc}{mark}")
    vals = [v for v, _ in options]
    while True:
        ans = ask("选择", str(default_idx))
        if ans.isdigit() and 1 <= int(ans) <= len(options):
            return options[int(ans) - 1][0]
        if ans in vals:
            return ans
        # 常见退出别名
        if ans.lower() in ("q", "quit", "exit"):
            return "quit" if "quit" in vals else vals[-1]
        _log(f"无效选择 {ans!r}（可输序号或 {vals[:3]}…）")


def pick_multi(title: str, options: list[tuple[str, str]],
               default: list[str] | None = None) -> list[str]:
    """多选。输入如 ``1,3`` 或回车=默认。"""
    print(f"\n{_c(title, BOLD)}")
    for i, (_v, desc) in enumerate(options, 1):
        mark = _c("  (上次)", DIM) if default and _v in default else ""
        print(f"  {i}) {desc}{mark}")
    d = ",".join(str(i + 1) for i, (v, _) in enumerate(options)
                 if default and v in default) or "全部"
    ans = ask("选择（逗号分隔，回车=默认）", d)
    if ans.strip().lower() in ("", "全部", "all"):
        return [v for v, _ in options]
    idx: list[int] = []
    for tok in ans.replace(" ", "").split(","):
        if tok.isdigit() and 1 <= int(tok) <= len(options):
            idx.append(int(tok) - 1)
    return [options[i][0] for i in idx] or [v for v, _ in options]


def run(cmd: list[str]) -> int:
    """执行 sdg.sh 子命令（继承终端，交互命令也能用）。"""
    print(f"\n{_c('$', DIM)} {' '.join(cmd)}\n")
    try:
        return subprocess.call(cmd)
    except KeyboardInterrupt:
        print()
        return 130


# ---------------------------------------------------------------- 上下文


def list_tasks(robot: str) -> list[str]:
    d = CONFIG_ROOT / robot / "tasks"
    if not d.is_dir():
        return []
    return sorted(p.stem for p in d.glob("*.yaml"))


def choose_context(state: dict) -> tuple[str, str]:
    """选机器人 + 任务。记住上次的选择。只有一个候选时自动选中不再问。"""
    robots = sorted(p.name for p in CONFIG_ROOT.iterdir()
                    if p.is_dir() and not p.name.startswith("_"))
    if not robots:
        raise SystemExit(f"[menu] ERROR {CONFIG_ROOT} 下没有任何机器人配置目录")

    last_r = state.get("robot") or robots[0]
    if len(robots) == 1:
        robot = robots[0]
        _log(f"机器人: {robot}（唯一候选）")
    else:
        print(f"\n{_c('机器人', BOLD)}")
        for i, r in enumerate(robots, 1):
            try:
                spec = load_robot_config(r)
                desc = f"{r:20s} {spec.articulation_prim}"
            except Exception as exc:  # noqa: BLE001
                desc = f"{r:20s} {_c(f'配置有误: {exc}', RED)}"
            mark = _c("  (上次)", DIM) if r == last_r else ""
            print(f"  {i}) {desc}{mark}")
        d_idx = robots.index(last_r) + 1 if last_r in robots else 1
        robot = pick("选择机器人", [(r, r) for r in robots], d_idx)

    tasks = list_tasks(robot)
    if not tasks:
        _log(f"WARN {robot} 下没有 tasks/*.yaml")
        task = ""
    elif len(tasks) == 1:
        task = tasks[0]
        _log(f"任务: {task}（唯一候选）")
    else:
        last_t = state.get("task") or tasks[0]
        d_idx = tasks.index(last_t) + 1 if last_t in tasks else 1
        task = pick("选择任务", [(t, t) for t in tasks], d_idx)

    state["robot"], state["task"] = robot, task
    save_state(state)
    return robot, task


def show_context(robot: str, task: str) -> None:
    print(f"\n{_c('当前上下文', BOLD)}: {_c(robot, GREEN)} / {_c(task or '（无任务）', GREEN)}")
    try:
        spec = load_robot_config(robot)
        print(f"  {DIM}articulation{DIM}: {spec.articulation_prim}")
        print(f"  相机/物体等任务配置: {CONFIG_ROOT / robot / 'tasks' / (task + '.yaml')}")
    except Exception as exc:  # noqa: BLE001
        print(f"  {_c(f'配置有误: {exc}', RED)}")


# ---------------------------------------------------------------- 功能分支


def _default_out(robot: str, task: str) -> str:
    try:
        return str(runtime.data_root() / robot / task)
    except Exception:  # noqa: BLE001
        return str(Path.home() / "fasim_data" / robot / task)


def menu_capture(robot: str, task: str, state: dict) -> None:
    """采集：只问该问的。"""
    print(f"\n{_c('=== 采集 ===', BOLD)}")
    all_what = ["camera", "joint", "ee", "object"]
    desc = {
        "camera": "相机 RGB-D（同时采内参）",
        "joint": "关节角（回放必需）",
        "ee": "末端位姿（tcp 真值 + 指令）",
        "object": "交互物体位姿",
    }
    what = pick_multi("采集项", [(w, f"{w:8s} {desc[w]}") for w in all_what],
                      state.get("what") or all_what)
    state["what"] = what

    camera = ""
    if "camera" in what:
        try:
            task_cfg = load_task_config(robot, task)
            cams = [c.name for c in (task_cfg.cameras or ())]
        except Exception:  # noqa: BLE001
            cams = []
        if not cams:
            _log("WARN 该任务没配相机，跳过相机")
        else:
            opts = [(c, c) for c in cams]
            if len(cams) > 1:
                opts.append(("all", f"全部（{len(cams)} 个）"))
            camera = pick("选择相机", opts, 1 if len(cams) == 1 else len(opts))

    every = pick(
        "采集频率（每 N 个物理步采一帧）",
        [("5", "5   约 40fps 仿真时间（最细，开销大）"),
         ("10", "10  约 20fps 仿真时间（推荐）"),
         ("20", "20  约 10fps 仿真时间（省开销）")],
        2,
    )

    out = ask("输出目录", _default_out(robot, task) + "/" + _suggest_name())
    state["last_out"] = out

    args = ["capture", "--robot", robot, "--task", task,
            "--what", ",".join(what), "--every", every, "--out", out]
    if camera:
        args += ["--camera", camera]
    run([str(SDG_SH)] + args)


def _suggest_name() -> str:
    """建议输出目录名：run_<序号>，取已有最大序号+1。"""
    try:
        root = runtime.data_root()
        if root.is_dir():
            nums = []
            for p in root.rglob("run_*"):
                suf = p.name[len("run_"):]
                if suf.isdigit():
                    nums.append(int(suf))
            return f"run_{max(nums) + 1:02d}" if nums else "run_01"
    except Exception:  # noqa: BLE001
        pass
    return "run_01"


def menu_tools(robot: str, task: str, state: dict) -> None:
    """转数据：pack / trim / shots / diag / fk-diag。"""
    print(f"\n{_c('=== 数据工具 ===', BOLD)}")
    tool = pick(
        "选择工具",
        [
            ("pack", "pack      采集会话 → episode.npz（含末端指令位姿反算）"),
            ("shots", "shots     相机联络图（人工检查 RGB）"),
            ("diag", "diag      一键体检 episode（7 项）"),
            ("fkdiag", "fk-diag   回放体检（回放 vs 编排，mm 级 + 方向）"),
            ("trim", "trim      切成任务片段（剔除待机段）"),
            ("info", "info      看会话里有什么"),
        ],
        1,
    )
    if tool == "fkdiag":
        ep = ask("回放记录路径（*.replay_fN.npz）")
        src = ask("原始 episode（可留空）")
        args = ["fk-diag", ep] + (["--source", src] if src else [])
        run([str(SDG_SH)] + args)
        return

    sess = ask("会话/episode 路径", state.get("last_session", ""))
    if not sess:
        _log("需要路径（会话目录或 episode.npz）")
        return
    state["last_session"] = sess
    save_state(state)

    if tool == "pack":
        run([str(SDG_SH), "pack", sess, "--robot", robot])
    elif tool == "shots":
        seg = ask_yes("只看任务段（自动剔除待机）", True)
        run([str(SDG_SH), "shots", sess] + (["--segment"] if seg else []))
    elif tool == "diag":
        run([str(SDG_SH), "diag", sess])
    elif tool == "trim":
        run([str(SDG_SH), "trim", sess])
    elif tool == "info":
        run([str(SDG_SH), "info", sess])


def menu_replay(robot: str, task: str, state: dict) -> None:
    """回放：关节 / 末端。"""
    print(f"\n{_c('=== 回放 ===', BOLD)}")
    ep = ask("episode.npz 路径", state.get("last_episode", ""))
    if not ep:
        _log("需要 episode 路径（先跑 pack）")
        return
    state["last_episode"] = ep
    save_state(state)

    drive = pick(
        "驱动方式",
        [
            ("joints", "joints  逐帧驱关节（纯 Isaac，不需要运控）"),
            ("ee", "ee      末端位姿发运控（需运控在跑，会锁底盘+切 FSM）"),
        ],
        1,
    )
    if drive == "joints":
        src = pick(
            "关节角来源",
            [
                ("cmd", "cmd     下发目标（推荐，与编排一致）"),
                ("actual", "actual  实测值（会多偏一次稳态误差，仅对照用）"),
            ],
            1,
        )
        run([str(SDG_SH), "live", ep, "--drive", "joints", "--joint-src", src, "--wait"])
    else:
        run([str(SDG_SH), "live", ep, "--drive", "ee", "--wait"])


def menu_selfcheck(robot: str, task: str, state: dict) -> None:
    print(f"\n{_c('=== 环境自检 ===', BOLD)}")
    run([str(SDG_SH), "doctor"])
    print()
    try:
        spec = load_robot_config(robot)
        print(f"{_c('机器人配置', BOLD)}: {robot}")
        print(f"  articulation : {spec.articulation_prim}")
        print(f"  base         : {spec.base_prim}")
        print(f"  末端刚体     : {list((spec.ee_links or {}).keys())}")
        print(f"  运动学 URDF  : {(spec.planning or {}).get('urdf')}")
        print(f"  ee_frames    : {(spec.planning or {}).get('ee_frames')}")
    except Exception as exc:  # noqa: BLE001
        print(f"  {_c(f'读取失败: {exc}', RED)}")


# ---------------------------------------------------------------- 主循环


FUNCS = [
    ("capture", "采集", menu_capture),
    ("status", "查看采集进度", None),
    ("stop", "结束采集", None),
    ("tools", "数据工具（pack/shots/diag/trim）", menu_tools),
    ("replay", "回放（关节/末端）", menu_replay),
    ("selfcheck", "环境自检", menu_selfcheck),
    ("context", "切换机器人/任务", None),
    ("quit", "退出", None),
]


def main() -> int:
    state = load_state()
    print(_c("\n" + "=" * 62, CYAN))
    print(_c("  FaSim 合成数据采集 (sdg)", BOLD))
    print(_c("=" * 62, CYAN))

    robot = state.get("robot")
    task = state.get("task")
    if not (robot and task) or ask_yes("重新选择机器人/任务", False):
        robot, task = choose_context(state)
    else:
        show_context(robot, task)

    while True:
        print()
        print(_c(f"上下文: {robot} / {task or '（无任务）'}", GREEN))
        hint = {
            "status": "查看采集进度",
            "stop": "结束采集并写 session.json",
            "context": f"当前 {robot}/{task}",
        }
        disp = []
        for k, lbl, _f in FUNCS:
            mark = "  (上次)" if k == state.get("last_func") else ""
            extra = f"   {hint[k]}" if k in hint else ""
            disp.append((k, f"{lbl}{mark}{_c(extra, DIM) if extra else ''}"))
        choice = pick("主菜单", disp, 1)
        state["last_func"] = choice

        if choice == "quit":
            _log("再见")
            return 0
        if choice == "context":
            robot, task = choose_context(state)
            show_context(robot, task)
            continue
        if choice == "status":
            run([str(SDG_SH), "status"])
            continue
        if choice == "stop":
            run([str(SDG_SH), "stop"])
            continue
        fn = next((f for k, _l, f in FUNCS if k == choice), None)
        if fn is None:
            continue
        try:
            fn(robot, task, state)
        except SystemExit:
            raise
        except KeyboardInterrupt:
            print()
            _log("已中断，回到主菜单")
        except Exception as exc:  # noqa: BLE001
            _log(f"{_c(f'出错: {exc}', RED)}")
        save_state(state)


if __name__ == "__main__":
    sys.exit(main())
