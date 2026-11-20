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


BACK = "<<back>>"


def pick(title: str, options: list[tuple[str, str]], default_idx: int = 1,
         allow_back: bool = False) -> str:
    """options: [(value, 显示文本)]。返回 value。

    除了序号，也接受直接输入 value（如 ``quit``）。``allow_back`` 时追加一项
    「返回」，返回 ``BACK`` 哨兵值 —— 子菜单用它回上一级，不用 Ctrl+C 打断。
    """
    if not options:
        raise SystemExit(f"[menu] ERROR {title}: 没有可选项")
    opts = list(options)
    if allow_back:
        opts.append((BACK, "返回上一级"))
    print(f"\n{_c(title, BOLD)}")
    for i, (_v, desc) in enumerate(opts, 1):
        mark = _c("  (上次)", DIM) if i == default_idx and not (
            allow_back and i == len(opts)) else ""
        print(f"  {i}) {desc}{mark}")
    vals = [v for v, _ in opts]
    while True:
        ans = ask("选择", str(default_idx))
        if ans.isdigit() and 1 <= int(ans) <= len(opts):
            return opts[int(ans) - 1][0]
        if ans in vals:
            return ans
        if ans.lower() in ("q", "quit", "exit"):
            return "quit" if "quit" in vals else vals[-1]
        if allow_back and ans.lower() in ("b", "back", "..", ""):
            return BACK
        _log(f"无效选择 {ans!r}（可输序号或 {vals[:3]}…"
             + ("，b=返回" if allow_back else "") + "）")


def _is_tty() -> bool:
    try:
        return sys.stdin.isatty() and sys.stdout.isatty()
    except Exception:  # noqa: BLE001
        return False


def _read_key() -> str:
    """读一个按键，返回归一化的名字。

    返回: 'up' / 'down' / 'space' / 'enter' / 'a' / 'q' / 其他单字符。
    用 termios 进原始模式，避免方向键被 readline 吃掉。
    """
    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        ch = sys.stdin.read(1)
        if ch == "\x1b":                      # ESC 序列：方向键
            seq = sys.stdin.read(2)
            if seq == "[A":
                return "up"
            if seq == "[B":
                return "down"
            return "esc"
        if ch in ("\r", "\n"):
            return "enter"
        if ch == " ":
            return "space"
        if ch == "\x03":                      # Ctrl-C
            raise KeyboardInterrupt
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def pick_multi(title: str, options: list[tuple[str, str]],
               default: list[str] | None = None,
               allow_empty: bool = False) -> list[str]:
    """多选采集项。

    **交互式（TTY）**：上下箭头移动光标，空格勾选/取消，回车确认，a=全选，q=取消。
    屏幕上方就地重绘，有 ``[x]`` 框框。这是默认路径 —— 避免手输数字打错。

    **非 TTY**（管道/CI/日志）：退回「逗号分隔数字」输入，行为与旧版一致。

    Args:
        default: 预勾选的 value 列表；None 表示全选。
        allow_empty: 允许一个都不选（回车时为空则返回空列表，而不是回退默认）。
    """
    if not options:
        return []

    dflt = ([v for v, _ in options] if default is None
            else [v for v, _ in options if v in default])

    # ---- 非 TTY：保留原来的文本输入 ----
    if not _is_tty():
        print(f"\n{_c(title, BOLD)}")
        for i, (v, desc) in enumerate(options, 1):
            mark = _c("  ✓", GREEN) if v in dflt else ""
            print(f"  {i}) {desc}{mark}")
        d = ",".join(str(i + 1) for i, (v, _) in enumerate(options) if v in dflt)
        ans = ask("选择（逗号分隔，回车=用 ✓ 的项）", d)
        if ans.strip().lower() in ("", "全部", "all"):
            return dflt
        idx = [int(t) - 1 for t in ans.replace(" ", "").split(",")
               if t.isdigit() and 1 <= int(t) <= len(options)]
        return [options[i][0] for i in idx] or dflt

    # ---- TTY：方向键 + 空格 ----
    chosen = set(dflt)
    cursor = 0
    n = len(options)

    def render() -> None:
        # 回到标题行重绘：先上移 n 行（首次渲染前 n 行还没画，用 \r 兜底）
        lines = [f"\n{_c(title, BOLD)}"]
        for i, (_v, desc) in enumerate(options):
            box = _c("[x]", GREEN) if options[i][0] in chosen else "[ ]"
            arrow = _c("❯", CYAN) if i == cursor else " "
            lines.append(f" {arrow} {box} {desc}")
        lines.append(_c("  ↑↓ 移动  空格 勾选  a 全选  n 全不选  回车 确认  q 取消", DIM))
        sys.stdout.write("\r\033[K" + "\n\033[K".join(lines) + "\033[K")
        sys.stdout.flush()

    def clear(n_lines: int) -> None:
        # 光标上移并清空，供重绘/收尾
        sys.stdout.write(f"\033[{n_lines}A")
        for _ in range(n_lines):
            sys.stdout.write("\033[K\n")
        sys.stdout.write(f"\033[{n_lines}A")
        sys.stdout.flush()

    total_lines = n + 2
    render()
    try:
        while True:
            key = _read_key()
            if key == "up":
                cursor = (cursor - 1) % n
            elif key == "down":
                cursor = (cursor + 1) % n
            elif key == "space":
                v = options[cursor][0]
                chosen.symmetric_difference_update({v})
            elif key == "a":
                chosen = {v for v, _ in options}
            elif key == "n":
                chosen = set()
            elif key == "enter":
                break
            elif key in ("q", "esc"):
                chosen = set(dflt)   # 取消 = 回到默认
                break
            clear(total_lines)
            render()
    finally:
        clear(total_lines)

    result = [v for v, _ in options if v in chosen]
    if not result and not allow_empty:
        _log("一个都没选，回退为全部")
        result = [v for v, _ in options]
    _log(f"已选: {result if result else '（空）'}")
    return result


def run(cmd: list[str]) -> int:
    """执行 sdg.sh 子命令（继承终端，交互命令也能用）。"""
    print(f"\n{_c('$', DIM)} {' '.join(cmd)}\n")
    try:
        return subprocess.call(cmd)
    except KeyboardInterrupt:
        print()
        return 130


def _fmt_age(mtime: float) -> str:
    import time

    d = max(0.0, time.time() - mtime)
    if d < 3600:
        return f"{d/60:.0f}分钟前"
    if d < 86400:
        return f"{d/3600:.0f}小时前"
    return f"{d/86400:.0f}天前"


def pick_session(
    state: dict,
    robot: str,
    task: str,
    *,
    need_episode: bool = False,
    kind: str = "会话",
) -> Path | None:
    """从数据根目录列出已有会话让用户选，而不是手输路径。

    手输路径最容易出错（``run_01`` vs ``run_1``），而且用户通常记不住完整
    路径。这里直接扫 ``data_root()`` 下的 ``session.json``，带帧数/时间/
    是否已 pack。也支持手输路径（列表里没有的，比如挂在别的盘）。

    Args:
        need_episode: True 时只列已 pack 出 ``episode.npz`` 的会话，
            并在标题里说明依赖（pack 之前不能 diag/fk-diag）。
        kind: 标题用词（``会话`` / ``episode``）。

    Returns:
        选中的路径；用户选「返回」或没有候选时返回 ``None``。
    """
    with_ep = need_episode
    sessions = runtime.list_sessions(robot=robot or None, task=task or None,
                                     with_episode_only=with_ep, limit=40)
    last = state.get("last_session", "")

    def _desc(s: dict) -> str:
        bits = [f"{s['name']:24s}"]
        bits.append(f"{s['frames']:4d}帧")
        if s["duration"]:
            bits.append(f"{s['duration']:5.1f}s")
        bits.append("有episode" if s["has_episode"] else "未pack")
        if s["cameras"]:
            bits.append(f"{len(s['cameras'])}相机")
        if s.get("incomplete"):
            bits.append(_c("⚠中断", YELLOW))
        bits.append(_fmt_age(s["mtime"]))
        return "  ".join(bits)

    if not sessions:
        _log(f"数据目录下没有可用{kind}: {runtime.data_root()}")
        if need_episode:
            _log("提示：先执行 pack，生成 episode.npz 后再回来")
        manual = ask("手动输入路径（回车=返回）", "")
        return Path(manual).expanduser() if manual else None

    opts = [(str(s["path"]), _desc(s)) for s in sessions]
    opts.append(("__manual__", "手动输入其他路径…"))
    d_idx = 1
    for i, (p, _d) in enumerate(opts, 1):
        if last and p == last:
            d_idx = i
            break
    title = f"选择{kind}"
    if need_episode:
        title += f"（只列已 pack 的；共 {len(sessions)} 个）"
    else:
        title += f"（共 {len(sessions)} 个，最新在前）"
    sel = pick(title, opts, d_idx, allow_back=True)
    if sel == BACK:
        return None
    if sel == "__manual__":
        manual = ask("路径", last or str(runtime.data_root()))
        p = Path(manual).expanduser()
        if not p.exists():
            _log(f"{_c(f'路径不存在: {p}', RED)}")
            return None
        return p
    p = Path(sel)
    state["last_session"] = sel
    save_state(state)
    return p


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


def _do_capture_with_preset(robot: str, task: str, name: str,
                            preset: dict, state: dict) -> None:
    """按预设直接开始采集（只问输出目录，其余参数全部锁定）。"""
    print(f"\n{_c(f'预设: {name}', BOLD)}  {preset.get('desc', '')}")

    base = _default_out(robot, task)
    suggest = f"{base}/{_suggest_name(base)}"
    print(f"{DIM}  数据根目录: {runtime.data_root()}{NC}")
    print(f"{DIM}  参数已锁定：what={preset.get('what')} "
          f"camera={preset.get('camera')} "
          f"target_fps={preset.get('target_fps')} "
          f"size={preset.get('width')}x{preset.get('height')} "
          f"depth={preset.get('depth')}{NC}")
    out = ask("输出目录（回车=用建议名）", suggest)
    if out.lower() in ("b", "back"):
        return
    state["last_out"] = out
    state["preset"] = name

    args = ["capture", "--robot", robot, "--task", task,
            "--preset", name, "--out", out]
    run([str(SDG_SH)] + args)


def _default_out(robot: str, task: str) -> str:
    try:
        return str(runtime.data_root() / robot / task)
    except Exception:  # noqa: BLE001
        return str(Path.home() / "fasim_data" / robot / task)


def menu_capture(robot: str, task: str, state: dict) -> None:
    """采集：只问该问的。每步都能 b 返回上一级。

    第一问是**采集预设**（如果任务配置里定义了）—— 选一次就锁定全部参数，
    保证多批次数据格式对齐。不选预设才逐项手选（保留给回放/针对性采集）。
    """
    print(f"\n{_c('=== 采集 ===', BOLD)}")

    try:
        task_cfg = load_task_config(robot, task)
    except Exception:  # noqa: BLE001
        task_cfg = None
    presets = dict(getattr(task_cfg, "capture_presets", None) or {}) if task_cfg else {}

    # ---- 预设优先 ----
    if presets:
        opts = [(k, f"{k:12s} {v.get('desc', '')}") for k, v in presets.items()]
        opts.append(("__manual__", "逐项手选（回放/针对性采集）"))
        sel = pick("采集预设（推荐：多批次数据格式一致）", opts,
                   state.get("preset_idx", 1), allow_back=True)
        if sel in (BACK, "quit"):
            return
        if sel != "__manual__":
            _do_capture_with_preset(robot, task, sel, presets[sel], state)
            return

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
            cams = [c.name for c in (task_cfg.cameras or ())] if task_cfg else []
        except Exception:  # noqa: BLE001
            cams = []
        if not cams:
            _log("WARN 该任务没配相机，跳过相机")
        else:
            opts_c = [(c, c) for c in cams]
            if len(cams) > 1:
                opts_c.append(("all", f"全部（{len(cams)} 个）"))
            camera = pick("选择相机", opts_c, 1 if len(cams) == 1 else len(opts_c),
                          allow_back=True)
            if camera in (BACK, "quit"):
                return

    # every 的实际帧率 = 物理步频 / every；步频来自任务配置（对应场景资产）
    phys_hz = getattr(task_cfg, "physics_hz", None) if task_cfg else None
    if phys_hz:
        opts = []
        seen = set()
        for tgt in (30, 20, 15, 10, 5):
            ev = max(1, int(round(phys_hz / tgt)))
            if ev in seen:
                continue
            seen.add(ev)
            fps = phys_hz / ev
            exact = abs(phys_hz / tgt - ev) < 1e-6
            opts.append((str(ev), f"{ev:3d}  约 {fps:5.1f} fps 仿真时间"
                                  f"{'' if exact else f'  (目标 {tgt}fps 取整)'}"))
        every = pick(
            f"采集频率（每 N 个物理步采一帧；本任务步频 {phys_hz:g} Hz）",
            opts, min(2, len(opts) - 1), allow_back=True,
        )
    else:
        every = pick(
            "采集频率（每 N 个物理步采一帧；步频未知，帧率取决于场景）",
            [("5", "5   每 5 个物理步 1 帧"),
             ("10", "10  每 10 个物理步 1 帧  (常用)"),
             ("20", "20  每 20 个物理步 1 帧")],
            2, allow_back=True,
        )
    if every in (BACK, "quit"):
        return

    base = _default_out(robot, task)
    suggest = f"{base}/{_suggest_name(base)}"
    print(f"\n{DIM}  数据根目录: {runtime.data_root()}{NC}")
    out = ask("输出目录（回车=用建议名）", suggest)
    if out.lower() in ("b", "back"):
        return
    state["last_out"] = out

    args = ["capture", "--robot", robot, "--task", task,
            "--what", ",".join(what), "--every", every, "--out", out]
    if camera:
        args += ["--camera", camera]
    run([str(SDG_SH)] + args)


def _suggest_name(base: str) -> str:
    """建议输出目录名：run_<序号>，取该 robot/task 下已有最大序号+1。

    只扫本 robot/task 目录（不是全局），避免不同任务互相干扰编号。
    """
    try:
        d = Path(base)
        if d.is_dir():
            nums = [int(p.name[4:]) for p in d.iterdir()
                    if p.is_dir() and p.name.startswith("run_")
                    and p.name[4:].isdigit()]
            return f"run_{max(nums) + 1:02d}" if nums else "run_01"
    except Exception:  # noqa: BLE001
        pass
    return "run_01"


def menu_tools(robot: str, task: str, state: dict) -> None:
    """转数据：pack / shots / diag / fk-diag / trim / info。

    流程有顺序：**采集 → pack → diag/shots/trim**。
    diag 要 episode.npz，所以这里只列已 pack 的会话，并在没 pack 过时明确
    提示先 pack —— 而不是让用户对着 "episode 不存在" 猜。
    """
    while True:
        print(f"\n{_c('=== 数据工具 ===', BOLD)}")
        if state.get("_tools_hint", True):
            print(f"{DIM}  顺序: 采集 → pack（生成 episode.npz）→ diag / shots / trim"
                  f"{NC}")
        tool = pick(
            "选择工具",
            [
                ("pack", "pack      采集会话 → episode.npz（含末端指令位姿反算）"),
                ("shots", "shots     相机联络图（人工检查 RGB）"),
                ("diag", "diag      一键体检 episode（7 项，需先 pack）"),
                ("fkdiag", "fk-diag   回放体检（回放 vs 编排，需先 live 回放）"),
                ("trim", "trim      切成任务片段（剔除待机段，需先 pack）"),
                ("export", "export    导出成 LeRobot 数据集（可直接训练 / 体积小很多）"),
                ("info", "info      看会话里有什么（不需要 pack）"),
            ],
            1, allow_back=True,
        )
        if tool == BACK or tool == "quit":
            return

        if tool == "export":
            sess = pick_session(state, robot, task, need_episode=False, kind="会话")
            if sess is None:
                continue
            out = ask("输出目录（回车=默认 ~/lerobot_datasets/<会话名>）", "")
            args = ["export", str(sess)]
            if out:
                args += ["--out", out]
            if ask_yes("输出目录已存在时覆盖", False):
                args.append("--overwrite")
            run([str(SDG_SH)] + args)
            continue

        if tool == "fkdiag":
            rec = ask("回放记录路径（*.replay_fN.npz，live 回放后生成，回车=返回）", "")
            if not rec:
                continue
            rp = Path(rec).expanduser()
            if not rp.is_file():
                _log(f"{_c(f'找不到文件: {rp}', RED)}")
                _log("提示：先跑 live 回放，结束时会在 episode 同目录生成 "
                     "<episode>.replay_f0.npz")
                continue
            src = ask("原始 episode（可留空；给了才能算「回放 vs 编排」）", "")
            args = ["fk-diag", str(rp)] + (["--source", src] if src else [])
            run([str(SDG_SH)] + args)
            continue

        # 需要 episode 的工具：只列已 pack 的会话
        need_ep = tool in ("diag", "trim")
        sess = pick_session(
            state, robot, task,
            need_episode=need_ep,
            kind="episode" if need_ep else "会话",
        )
        if sess is None:
            continue
        if not sess.exists():
            _log(f"{_c(f'路径不存在: {sess}', RED)}")
            continue

        # 目录给成 episode.npz（diag/fk-diag 要文件）
        target = sess
        if need_ep and sess.is_dir():
            ep = sess / "episode.npz"
            if not ep.is_file():
                _log(f"{_c('该会话还没有 episode.npz —— 先跑 pack', YELLOW)}")
                if ask_yes("现在就 pack 它", True):
                    run([str(SDG_SH), "pack", str(sess), "--robot", robot])
                    if not ep.is_file():
                        continue
                else:
                    continue
            target = ep
        state["last_session"] = str(sess)
        save_state(state)

        if tool == "pack":
            run([str(SDG_SH), "pack", str(sess), "--robot", robot])
        elif tool == "shots":
            seg = ask_yes("只看任务段（自动剔除待机）", True)
            run([str(SDG_SH), "shots", str(sess)] + (["--segment"] if seg else []))
        elif tool == "diag":
            run([str(SDG_SH), "diag", str(target)])
        elif tool == "trim":
            run([str(SDG_SH), "trim", str(target)])
        elif tool == "info":
            run([str(SDG_SH), "info", str(sess)])



def _request(payload: dict, timeout: float = 5.0) -> dict:
    """向 Isaac 扩展发一条指令（菜单内的轻量版，与 cli.py 的 _request 同协议）。

    为什么菜单里还要单独写一份：cli.py 的版本会打印一堆「Isaac 没在跑」
    之类的引导语，菜单顶部已经有一个环境状态条了，重复提示反而乱。
    这里只返回结果，由调用方决定怎么显示。
    """
    import socket as _sock

    from fasim.replicator.sdg import runtime

    path = runtime.socket_path()
    if not path.exists():
        return {"ok": False, "error": "socket 不存在（Isaac 未运行或扩展未就绪）"}
    s = _sock.socket(_sock.AF_UNIX, _sock.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect(str(path))
        s.sendall((json.dumps(payload, ensure_ascii=False) + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(65536)
            if not chunk:
                break
            buf += chunk
        if not buf.strip():
            return {"ok": False, "error": "扩展没有返回"}
        return json.loads(buf.split(b"\n", 1)[0].decode("utf-8", "replace"))
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
    finally:
        try:
            s.close()
        except OSError:
            pass


def prompt_positive_int(prompt: str, default: int = 1, min_value: int = 1) -> int:
    """问一个正整数（回车用默认值）。"""
    while True:
        raw = input(f"{prompt} [{default}]: ").strip()
        if not raw:
            return default
        try:
            v = int(raw)
        except ValueError:
            print(f"  {_c('请输入整数', YELLOW)}")
            continue
        if v < min_value:
            print(f"  {_c(f'至少 {min_value}', YELLOW)}")
            continue
        return v


def menu_auto(robot: str, task: str, state: dict) -> None:
    """一键数采：随机化 → 采集 → 编排 → 结束 → 裁剪 → 打包。

    每轮独立（各自随机化、各自 run_*/ 目录），产物：
      run_NNN/episode_trimmed.npz   留开头 home + 截到回 home 的单条轨迹
      run_NNN/episode.npz           完整原始数据（pack 格式）
    """
    from fasim.replicator.sdg import auto_collect, motion_launch as ml

    print(f"\n{_c('=== 一键数采 ===', BOLD)}")
    print(f"{DIM}  随机化 → 采集 → 编排 → 结束 → 裁剪 → 打包{NC}")

    # 预检
    st = _request({"cmd": "status"}, timeout=10)
    if not st.get("ok"):
        print(f"{_c('  Isaac 扩展没响应 —— 先 cd ~/FaSim-Isaac && ./run.sh', RED)}")
        return
    ok, info = ml.preflight(robot_key=robot, task_key=task)
    if not ok:
        return
    try:
        ctx = auto_collect._resolve_task_ctx(robot, task)
    except Exception as exc:  # noqa: BLE001
        print(f"{_c(f'  读任务配置失败: {exc}', RED)}")
        return

    print(f"  任务        {info['sdg_robot']}/{info['sdg_task']} → {info['robot']}/{info['task_key']}")
    print(f"  数据目录    {DIM}{ctx['data_dir']}{NC}")
    if ctx["has_randomize"]:
        snap = "已存在" if ctx["snapshot_file"].is_file() else "将自动记录"
        print(f"  随机化      有（快照{snap}）")
    else:
        print(f"  {_c('随机化      未配置（每轮物体位置不变）', YELLOW)}")

    rounds = prompt_positive_int("跑几轮", default=1, min_value=1)
    head = 2.0
    tail = 1.5
    ans = input(f"{DIM}  裁剪参数：开头留几秒 home [{head}] / 结尾留几秒 [{tail}]（回车用默认）: {NC}").strip()
    if ans:
        parts = ans.replace("/", " ").split()
        try:
            if len(parts) >= 1:
                head = float(parts[0])
            if len(parts) >= 2:
                tail = float(parts[1])
        except ValueError:
            print(f"{_c('  参数无效，用默认值', YELLOW)}")

    print()
    print(f"  预计每轮约 1~2 分钟（编排本身约 40s + 采集开销）")
    if input(f"  开始？[Y/n]: ").strip().lower() not in ("", "y", "yes"):
        return

    results = auto_collect.run_batch(
        robot_key=robot,
        task_key=task,
        rounds=rounds,
        head_sec=head,
        tail_sec=tail,
    )
    ok_n = sum(1 for r in results if r.get("ok"))
    print()
    if ok_n == len(results):
        print(f"{GREEN}  ✅ {ok_n} 轮全部成功{NC}")
    else:
        print(f"{_c(f'  {ok_n}/{len(results)} 轮成功', YELLOW)}")


def menu_motion(robot: str, task: str, state: dict) -> None:
    """一键起编排：让 motion-generation 跑一遍任务。

    把「切到 lerobot_ros2、source venv、敲一长串参数」收进菜单。
    两个关键开关固定带上（``--no-reset`` 保随机化、``--no-ensure-ros2-stack``
    避免起第二套运控），详见 ``fasim/replicator/sdg/motion_launch.py``。
    """
    from fasim.replicator.sdg import motion_launch as ml

    print(f"\n{_c('=== 一键起编排 ===', BOLD)}")
    print(f"{DIM}  让 motion-generation 跑一遍任务（在另一个进程里）{NC}")

    ok, info = ml.preflight(robot_key=robot, task_key=task)
    if not ok:
        return

    print(f"  sdg 任务    {info['sdg_robot']}/{info['sdg_task']}")
    print(f"  编排任务    {info['robot']}/{info['task_key']}")
    print(f"  仓库        {DIM}{info['repo']}{NC}")
    print(f"  工作区      {DIM}{info['workspace']}{NC}")
    if info.get("activate"):
        print(f"  venv        {DIM}{info['activate']}{NC}")
    else:
        print(f"{_c('  WARN 没找到 venv，可能缺 rclpy', YELLOW)}")
    print(f"{DIM}  固定参数: --no-reset（保随机化） "
          f"--no-ensure-ros2-stack（运控随 Isaac 启动）{NC}")

    # 采集状态提醒
    st = _request({"cmd": "status"}, timeout=5)
    if st.get("ok") and st.get("armed"):
        print(f"{GREEN}  采集进行中（{st.get('frames',0)} 帧）—— 编排跑的动作会被采下来{NC}")
        dc = st.get("done_checker")
        if dc:
            print(f"{DIM}    完成判据: {dc['object']}→{dc['rack']} "
                  f"xy<{dc['xy_tol_mm']:.0f}mm dz<{dc['dz_inside_mm']}mm{NC}")
    else:
        print(f"{_c('  当前未采集 —— 编排照跑，但采不到数据。建议先「开始采集」', YELLOW)}")

    running = ml.is_running()
    if running:
        print(f"{_c(f'  警告: 已有编排在跑 (pid {running[:3]})', YELLOW)}")

    while True:
        print()
        sel = pick(
            "起编排",
            [
                ("fg", "前台运行（看输出，Ctrl+C 可中断）"),
                ("bg", "后台运行（立即返回，日志到 /tmp）"),
                ("check", "只预检（不真的跑）"),
            ],
            1,
            allow_back=True,
        )
        if sel in (BACK, "quit"):
            return

        if sel == "check":
            print(f"{GREEN}  预检通过{NC}")
            continue

        if not st.get("ok") or not st.get("armed"):
            ans = input(f"{_c('  采集没开，仍要跑？[y/N]: ', YELLOW)}").strip().lower()
            if ans not in ("y", "yes"):
                continue

        rc = ml.run_motion(
            robot_key=robot,
            task_key=task,
            background=(sel == "bg"),
        )
        if sel == "bg":
            print(f"{DIM}  用 ./sdg.sh status 看采集进度；"
                  f"编排日志见上面的路径{NC}")
            return
        print(f"{DIM}  编排退出码 {rc}{NC}")
        return


def menu_randomize(robot: str, task: str, state: dict) -> None:
    """随机化物体位姿 —— 纯调试/半自动数采用，不进采集流程。

    设计意图：**足够独立**。不依赖采集是否在跑，可以反复按、反复看效果，
    用来验证「随机化后仿真会不会崩」「物体位置是否合理」，也可以人工
    挑位置后手动开始采集（半自动数采）。

    每次都从「初始快照位姿 + 随机偏移」算起，所以：
      - 第一次用必须先「记录初始位姿」（本菜单第一项会自动检测并提示）
      - 反复随机不会累积漂移（每次都从快照重新算）
    """
    from fasim.replicator.sdg import config_io

    try:
        task_cfg = config_io.load_task_config(robot, task)
    except Exception as exc:  # noqa: BLE001
        _log(f"{_c(f'读任务配置失败: {exc}', RED)}")
        return

    cfg = dict(getattr(task_cfg, "randomize", {}) or {})
    if not cfg:
        _log(f"{_c('该任务的 yaml 里没有 randomize: 段 —— 无可随机化的物体', YELLOW)}")
        _log(f"{DIM}  在 configs/{robot}/tasks/{task}.yaml 里加一段 randomize: 即可{NC}")
        return

    objs = cfg.get("objects") or []
    print(f"\n{_c('=== 随机化物体 ===', BOLD)}")
    print(f"{DIM}  任务: {robot}/{task}{NC}")
    want_w = max((len(str(o.get("prim", ""))) for o in objs), default=10)
    print(f"{DIM}  配置声明的物体（{len(objs)} 个）:{NC}")
    for o in objs:
        prim = str(o.get("prim", ""))
        ax = o.get("axes") or {}
        follow = o.get("follow")
        if follow:
            desc = f"follow {follow}"
        elif ax:
            desc = " ".join(f"{k}±{float(v)*1000:.0f}mm" for k, v in ax.items())
        else:
            desc = "不动"
        print(f"    {prim:<{want_w}}  {desc}")

    # 快照状态检查
    import os
    snap = os.path.join(str(getattr(task_cfg, "output_dir", "") or ""),
                        "scene_initial_poses.json")
    has_snap = os.path.isfile(snap)
    print()
    if has_snap:
        print(f"{GREEN}  初始位姿快照: 已有{NC} {DIM}({snap}){NC}")
    else:
        print(f"{_c('  初始位姿快照: 还没有 —— 先选「记录初始位姿」', YELLOW)}")

    while True:
        sel = pick(
            "操作",
            [
                ("snapshot", "记录初始位姿（第一轮前必须做一次）"),
                ("once", "随机一次（看效果）"),
                ("multi", "连续随机 N 次（验证会不会崩）"),
                ("range", "改随机范围（临时，不改 yaml）"),
            ],
            2 if has_snap else 1,
            allow_back=True,
        )
        if sel == BACK or sel == "quit":
            return

        if sel == "snapshot":
            run([str(SDG_SH), "randomize", "--robot", robot, "--task", task, "--snapshot"])
            has_snap = True
            continue

        if sel == "once":
            if not has_snap:
                _log(f"{_c('先记录初始位姿（第一项）', YELLOW)}")
                continue
            run([str(SDG_SH), "randomize", "--robot", robot, "--task", task])
            continue

        if sel == "multi":
            if not has_snap:
                _log(f"{_c('先记录初始位姿（第一项）', YELLOW)}")
                continue
            n = ask("连续随机几轮", "10")
            try:
                n_i = max(1, int(n))
            except ValueError:
                _log(f"{_c('要整数', RED)}")
                continue
            rng = ask("临时覆盖范围？如 0.04,0.03,0 表示 x±40 y±30 z±0（回车用配置）", "")
            cmd = [str(SDG_SH), "randomize", "--robot", robot, "--task", task, "--rounds", str(n_i)]
            if rng.strip():
                parts = [x.strip() for x in rng.replace("，", ",").split(",") if x.strip()]
                for name, val in zip(("--range-x", "--range-y", "--range-z"), parts):
                    cmd += [name, val]
            run(cmd)
            _log(f"{DIM}  想看更严格的报告（含读回误差/越界统计）就在 Isaac Script Editor 里跑：{NC}")
            _log(f"{DIM}    exec(open('{SCRIPT_DIR}/verify_randomize.py').read()){NC}")
            continue

        if sel == "range":
            _log("临时改范围只对本次「连续随机」有效；要长期生效请改任务 yaml 的 randomize.objects[].axes")
            continue


def menu_replay(robot: str, task: str, state: dict) -> None:
    """回放：关节 / 末端。"""
    ep = pick_session(state, robot, task, need_episode=True, kind="episode")
    if ep is None:
        return
    if ep.is_dir():
        ep = ep / "episode.npz"
    if not ep.is_file():
        _log(f"{_c(f'找不到 {ep}', RED)}")
        return
    state["last_episode"] = str(ep)
    save_state(state)

    drive = pick(
        "驱动方式",
        [
            ("joints", "joints  逐帧驱关节（纯 Isaac，不需要运控）"),
            ("ee", "ee      末端位姿发运控（需运控在跑，会锁底盘+切 FSM）"),
        ],
        1, allow_back=True,
    )
    if drive == BACK or drive == "quit":
        return
    print(f"{DIM}  注: 回放结束后会停在最后一帧，不交还运控"
          f"（如需要用 --restore-cm）{NC}")

    if drive == "joints":
        src = pick(
            "关节角来源",
            [
                ("cmd", "cmd     下发目标（推荐，与编排一致）"),
                ("actual", "actual  实测值（会多偏一次稳态误差，仅对照用）"),
            ],
            1, allow_back=True,
        )
        if src == BACK or src == "quit":
            return
        run([str(SDG_SH), "live", str(ep), "--robot", robot, "--task", task,
             "--drive", "joints", "--joint-src", src, "--wait"])
    else:
        if not ask_yes("确认运控已在跑，且 FSM 可切到 OCS2", True):
            return
        run([str(SDG_SH), "live", str(ep), "--robot", robot, "--task", task,
             "--drive", "ee", "--wait"])


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
    ("auto", "一键数采（随机化→采集→编排→裁剪→打包）", menu_auto),
    ("capture", "采集", menu_capture),
    ("status", "查看采集进度", None),
    ("stop", "结束采集", None),
    ("motion", "一键起编排（跑一遍任务）", menu_motion),
    ("tools", "数据工具（pack/shots/diag/trim）", menu_tools),
    ("replay", "回放（关节/末端）", menu_replay),
    ("random", "随机化物体位姿（调试/半自动数采）", menu_randomize),
    ("selfcheck", "环境自检", menu_selfcheck),
    ("context", "切换机器人/任务", None),
    ("quit", "退出", None),
]


def check_env() -> dict:
    """检查 Isaac / socket 状态，供主菜单顶部提示。

    崩溃是常态（实测采集中崩过），菜单不该假装一切正常 —— 启动时就说清
    现在是什么状态，比等用户点了采集才报错好。
    """
    import socket as _sock

    info: dict = {"isaac": False, "socket": False, "pids": []}
    try:
        import subprocess

        r = subprocess.run(["pgrep", "-f", "isaacsim/kit|isaac-sim.sh"],
                           capture_output=True, text=True, timeout=5)
        pids = [int(x) for x in r.stdout.split() if x.isdigit()]
        info["pids"] = pids
        info["isaac"] = bool(pids)
    except Exception:  # noqa: BLE001
        pass
    sp = runtime.socket_path()
    if sp.exists():
        s = _sock.socket(_sock.AF_UNIX, _sock.SOCK_STREAM)
        s.settimeout(1.0)
        try:
            s.connect(str(sp))
            info["socket"] = True
        except Exception:  # noqa: BLE001
            info["socket"] = False
        finally:
            try:
                s.close()
            except OSError:
                pass
    return info


def print_env_banner() -> dict:
    """打印环境状态条。返回 check_env() 结果，供主菜单决定是否提示。"""
    st = check_env()
    if st["isaac"] and st["socket"]:
        print(f"{DIM}  环境: Isaac 运行中（pid {st['pids'][0]}），采集扩展就绪{NC}")
    elif st["isaac"] and not st["socket"]:
        print(f"{YELLOW}  环境: Isaac 在跑但扩展未就绪"
              f"（等几秒；或查 Isaac 终端 [sdg] 日志）{NC}")
    elif not st["isaac"] and st["socket"]:
        print(f"{RED}  环境: Isaac 未运行，但残留了 socket 文件 —— "
              f"仿真可能崩溃过{NC}")
        print(f"{DIM}        重启 ./run.sh 即可；残留文件会被自动覆盖{NC}")
    else:
        print(f"{DIM}  环境: Isaac 未运行（采集/回放前先 cd ~/FaSim-Isaac && ./run.sh）{NC}")
    return st


def main() -> int:
    state = load_state()
    print(_c("\n" + "=" * 62, CYAN))
    print(_c("  FaSim 合成数据采集 (sdg)", BOLD))
    print(_c("=" * 62, CYAN))
    env = print_env_banner()

    robot = state.get("robot")
    task = state.get("task")
    # 每次启动都先确认上下文。
    #
    # 为什么不再"记住后直接进主菜单"：
    #   先选上下文、再选功能，否则用户会在一堆功能选项里凭印象点，
    #   很容易在上一台机器人（比如刚测完的 G1D）上误执行采集 ——
    #   采到一半才发现机器人不对，数据白采。
    # 上下文本身仍会记住：choose_context 把上次的选择排在默认位并标 (上次)，
    # 回车即沿用，多按一次回车的成本换来不会选错机器人。
    if robot and task:
        print(f"{DIM}  上次用过 {robot} / {task}（回车沿用）{NC}")
    robot, task = choose_context(state)
    show_context(robot, task)

    while True:
        print()
        print(_c("─" * 62, DIM))
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
            r = run([str(SDG_SH), "status"])
            if r != 0:
                print(f"{YELLOW}  （Isaac 可能未运行或已崩溃；"
                      f"选 6 自检可看详细状态）{NC}")
            continue
        if choice == "stop":
            r = run([str(SDG_SH), "stop"])
            if r != 0:
                print(f"{YELLOW}  （连不上 Isaac —— 若它已崩，数据仍在 frames/ 里，"
                      f"可以直接 pack 抢救）{NC}")
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
