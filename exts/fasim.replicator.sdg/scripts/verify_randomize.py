"""可行性验证：在**正在运行的仿真**里反复随机化物体，看会不会崩。

## 为什么单独做这个
自动化采集的前提是"随机化 100 次都不崩"。而 Isaac 里给 dynamic 刚体
``set_world_poses`` 有两个已知坑（见 renderer.py 注释）：
  - kinematic=True  → 设置完全无效
  - kinematic=False → 会被物理改写（位置飞走）
所以在把它接进自动采集循环之前，必须先**单独验证**：设完位姿后物理能否
稳定下来、会不会穿模/飞走/崩溃。

## 用法（Isaac 仿真已打开、场景已加载）

在 **Isaac 的 Script Editor** 里执行::

    exec(open("/path/to/exts/fasim.replicator.sdg/scripts/verify_randomize.py").read())

或从外部通过 socket（脚本化时用）::

    ./scripts/sdg.sh randomize-check

## 它做什么
1. 抓一次初始位姿快照
2. 循环 N 轮：归位 → 随机偏移 → 推进物理 settle → 读回验证
3. 每轮记录：实际偏移量、settle 后误差、是否越界、是否异常
4. 最后给一份报告：能否安全用于自动采集

**不改任何配置、不写数据**，纯验证。
"""

from __future__ import annotations

import json
import time
from pathlib import Path

#: 默认验证的物体（可被 task 配置覆盖）
DEFAULT_TARGETS = [
    "/World/room/scene/test_tube_rack",
    "/World/room/scene/test_tube_rack_01",
]

#: 默认随机范围（±米）。试管任务只动 xy。
DEFAULT_AXES = {"x": 0.04, "y": 0.03}


def _log(msg: str) -> None:
    print(f"[verify] {msg}", flush=True)


def run_check(
    *,
    targets: list[dict] | None = None,
    rounds: int = 10,
    settle_steps: int = 60,
    out_dir: str | Path | None = None,
) -> dict:
    """执行验证循环，返回报告 dict。

    Args:
        targets: ``[{"prim": ..., "axes": {...}}, ...]``；None 用 DEFAULT_TARGETS。
        rounds: 随机化轮数。
        settle_steps: 每轮设完位姿后推进多少物理步。
        out_dir: 报告输出目录。
    """
    import omni.kit.app
    import numpy as np

    from fasim.replicator.sdg import randomize as R

    app = omni.kit.app.get_app()
    targets = targets or [{"prim": p, "axes": DEFAULT_AXES} for p in DEFAULT_TARGETS]

    out = Path(out_dir) if out_dir else Path("/tmp/sdg_randomize_check")
    out.mkdir(parents=True, exist_ok=True)
    data_dir = out  # 快照放这里，不污染真实数据目录

    cfg = {"objects": targets, "settle_steps": settle_steps, "verify_tol_m": 0.05}

    # ---- 0) 初始快照 ----
    _log("抓初始位姿快照 ...")
    snap = R.save_snapshot(cfg, data_dir)
    base = {k: (v["pos"] if v else None) for k, v in (snap.get("objects") or {}).items()}
    _log(f"  快照: { {k: (np.round(v,3).tolist() if v else None) for k,v in base.items()} }")

    # ---- 物理推进函数 ----
    def settle(n_steps: int) -> None:
        for _ in range(max(0, int(n_steps))):
            app.update()

    # ---- 1) 循环 ----
    rows: list[dict] = []
    crashes = 0
    for i in range(rounds):
        t0 = time.time()
        try:
            res = R.restore_and_randomize(
                cfg, data_dir=data_dir,
                rng=np.random.default_rng(1000 + i),
                settle_fn=settle,
            )
            # 统计实际偏移量（相对快照）
            deltas = {}
            for a in res.get("applied") or []:
                p = a["prim"]
                off = a.get("offset") or [0, 0, 0]
                deltas[p] = off
            row = {
                "round": i,
                "ok": res.get("ok"),
                "seed": res.get("seed"),
                "applied": len(res.get("applied") or []),
                "warnings": res.get("warnings") or [],
                "deltas": deltas,
                "elapsed_ms": round((time.time() - t0) * 1000, 1),
            }
        except Exception as exc:  # noqa: BLE001
            import traceback
            traceback.print_exc()
            crashes += 1
            row = {"round": i, "ok": False, "error": f"{type(exc).__name__}: {exc}"}
        rows.append(row)
        _log(f"第 {i+1}/{rounds} 轮: ok={row.get('ok')} "
             f"warnings={len(row.get('warnings') or [])} "
             f"{row.get('elapsed_ms','?')}ms")

    # ---- 2) 统计 ----
    ok_rounds = sum(1 for r in rows if r.get("ok"))
    all_warn = [w for r in rows for w in (r.get("warnings") or [])]
    # 每轮实际偏移量是否在声明范围内
    oob = []
    for r in rows:
        for prim, off in (r.get("deltas") or {}).items():
            ax = next((t.get("axes") or {} for t in targets if t["prim"] == prim), {})
            for j, name in enumerate(("x", "y", "z")):
                lim = ax.get(name)
                if lim and abs(off[j]) > lim + 1e-6:
                    oob.append(f"round{r['round']} {prim} {name}={off[j]:+.4f} > ±{lim}")

    report = {
        "rounds": rounds,
        "ok_rounds": ok_rounds,
        "crashes": crashes,
        "settle_steps": settle_steps,
        "targets": targets,
        "out_of_range": oob,
        "warnings": all_warn,
        "rows": rows,
        "verdict": "",
    }

    # ---- 3) 结论 ----
    if crashes > 0:
        verdict = f"❌ 出现 {crashes} 次异常 —— 不能直接用于自动采集，看 traceback"
    elif ok_rounds < rounds:
        verdict = (f"⚠️ {rounds - ok_rounds}/{rounds} 轮有告警 —— "
                   "物体可能被物理改写/碰撞，需要收窄范围或加 settle 步数")
    elif all_warn:
        verdict = f"⚠️ 全部完成但有 {len(all_warn)} 条告警，逐条看"
    else:
        verdict = f"✅ {rounds} 轮全部正常 —— 可以接入自动采集"
    report["verdict"] = verdict

    (out / "report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    _log("=" * 66)
    _log(f"验证 {rounds} 轮：成功 {ok_rounds}，异常 {crashes}")
    _log(f"越界 {len(oob)} 条，告警 {len(all_warn)} 条")
    _log(verdict)
    _log(f"报告: {out/'report.json'}")
    _log("=" * 66)
    return report


def main() -> int:
    """Script Editor / 直接执行入口。"""
    import argparse

    ap = argparse.ArgumentParser(description="随机化可行性验证")
    ap.add_argument("--rounds", type=int, default=10)
    ap.add_argument("--settle-steps", type=int, default=60,
                    help="每轮设完位姿后推进多少物理步（200Hz 下 60 步=0.3s）")
    ap.add_argument("--prim", action="append", default=None,
                    help="要随机化的 prim，可重复。默认用试管架两个")
    ap.add_argument("--range-x", type=float, default=DEFAULT_AXES["x"])
    ap.add_argument("--range-y", type=float, default=DEFAULT_AXES["y"])
    ap.add_argument("--out", default="")
    args, _unknown = ap.parse_known_args()

    prims = args.prim or DEFAULT_TARGETS
    targets = [{"prim": p, "axes": {"x": args.range_x, "y": args.range_y}}
               for p in prims]

    rep = run_check(
        targets=targets,
        rounds=args.rounds,
        settle_steps=args.settle_steps,
        out_dir=args.out or None,
    )
    # **不要 sys.exit / os._exit** —— 会把 Isaac 一起关掉（DEBUG_LOG #12）
    return 0 if rep.get("crashes", 1) == 0 else 1


if __name__ == "__main__":
    # 在 Isaac 内执行时不要退出进程
    main()
