#!/usr/bin/env python3
"""Audit (and optionally fix) mimic joints that have no active drive.

Why
---
``isaacsim.ros2.control`` (IsaacSimSystem) tracks a mimic joint through a
position drive on the mimic joint itself.  Without one it logs at startup::

    IsaacSimSystem: mimic joint '<j>' has no position drive (stiffness=0);
    it cannot track leader '<l>'. Author a position DriveAPI on the mimic joint.

``isaacsim.asset.exporter.urdf`` also warns about mimic joints without a drive
("has no finite effort limit and no active drive").

Usage
-----
    python3 tools/audit_mimic_joint_drives.py                 # report only
    python3 tools/audit_mimic_joint_drives.py --apply         # write drives
    python3 tools/audit_mimic_joint_drives.py --apply --dry-run

The fix mirrors the leader joint's ``stiffness`` / ``damping`` / ``maxForce``
onto the mimic joint (``targetPosition`` stays 0; IsaacSimSystem overwrites it
with ``gearing * leader + offset`` every tick).
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import sys
from dataclasses import dataclass

DRIVE_API_TOKENS = ("PhysicsDriveAPI",)
LEDGER: list[str] = []


@dataclass
class Joint:
    path: str
    name: str
    kind: str  # "Revolute" | "Prismatic" | ...
    start: int
    end: int
    lines: list[str]

    @property
    def text(self) -> str:
        return "\n".join(self.lines)

    @property
    def axis(self) -> str:
        return "angular" if self.kind == "Revolute" else "linear"

    @property
    def has_mimic(self) -> bool:
        t = self.text
        return (
            "PhysxMimicJointAPI" in t
            or "NewtonMimicAPI" in t
            or "mimicJoint" in t
        )

    @property
    def drive_values(self) -> tuple[float, float, float]:
        def num(pattern: str) -> float | None:
            m = re.search(pattern, self.text)
            return float(m.group(1)) if m else None

        stiff = num(r"drive:\w+:physics:stiffness\s*=\s*([0-9.eE+-]+)") or 0.0
        damp = num(r"drive:\w+:physics:damping\s*=\s*([0-9.eE+-]+)") or 0.0
        force = num(r"drive:\w+:physics:maxForce\s*=\s*([0-9.eE+-]+)") or 0.0
        return stiff, damp, force

    @property
    def has_drive(self) -> bool:
        api = re.search(r"apiSchemas\s*=\s*\[(.*?)\]", self.text, re.S)
        if not api or not any(tok in api.group(1) for tok in DRIVE_API_TOKENS):
            return False
        stiff, damp, _ = self.drive_values
        return bool(stiff or damp)

    @property
    def leader(self) -> str | None:
        m = re.search(r"mimicJoint\s*=\s*<[^>]*?/([^/>]+)>", self.text)
        if not m:
            m = re.search(
                r"physxMimicJoint:\w+:referenceJoint\s*=\s*<[^>]*?/([^/>]+)>",
                self.text,
            )
        return m.group(1) if m else None


def parse_joints(path: str) -> list[Joint]:
    lines = open(path, encoding="utf-8", errors="replace").read().splitlines()
    out: list[Joint] = []
    i = 0
    while i < len(lines):
        m = re.match(r'^(\s*)def\s+(\w*?)(Revolute|Prismatic)Joint\s+"([^"]+)"', lines[i])
        if not m:
            i += 1
            continue
        indent = len(m.group(1))
        start = i
        i += 1
        while i < len(lines) and not (
            lines[i].strip() == "}"
            and len(lines[i]) - len(lines[i].lstrip()) == indent
        ):
            i += 1
        out.append(Joint(path, m.group(4), m.group(3), start, i, lines[start : i + 1]))
        i += 1
    return out


def plan_fix(joint: Joint, by_name: dict[str, Joint]) -> tuple[list[str], str] | None:
    """Return (new_lines, note) for one mimic joint, or None if not fixable."""
    leader_name = joint.leader
    if not leader_name:
        return None
    leader = by_name.get(leader_name)
    if leader is None or leader is joint:
        return None
    stiff, damp, force = leader.drive_values
    if not (stiff or damp):
        return None
    force = force or 20.0

    lines = list(joint.lines)
    axis = joint.axis

    # 1) add DriveAPI to apiSchemas
    for idx, line in enumerate(lines):
        m = re.search(r"(apiSchemas\s*=\s*\[)(.*?)(\])", line)
        if not m or f"PhysicsDriveAPI:{axis}" in m.group(2):
            continue
        list_body = m.group(2)
        token = f'"PhysicsDriveAPI:{axis}"'
        if '"NewtonMimicAPI"' in list_body:
            list_body = list_body.replace('"NewtonMimicAPI"', f'"NewtonMimicAPI", {token}')
        else:
            list_body = f"{list_body.rstrip().rstrip(',')}, {token}"
        lines[idx] = line[: m.start()] + m.group(1) + list_body + m.group(3) + line[m.end() :]
        break
    else:
        return None  # apiSchemas is not on one line -> leave it to a human

    # 2) insert drive attributes right after the opening brace
    body_indent = " " * (len(lines[0]) - len(lines[0].lstrip()) + 4)
    attrs = [
        f"{body_indent}float drive:{axis}:physics:damping = {_fmt(damp)}",
        f"{body_indent}float drive:{axis}:physics:maxForce = {_fmt(force)}",
        f"{body_indent}float drive:{axis}:physics:stiffness = {_fmt(stiff)}",
        f"{body_indent}float drive:{axis}:physics:targetPosition = 0",
        f'{body_indent}uniform token drive:{axis}:physics:type = "force"',
    ]
    brace = next(i for i, l in enumerate(lines) if l.strip() == "{")
    lines[brace + 1 : brace + 1] = attrs
    note = f"mirror {leader_name} stiff={_fmt(stiff)} damp={_fmt(damp)} force={_fmt(force)}"
    return lines, note


def _fmt(v: float) -> str:
    return str(int(v)) if float(v).is_integer() else str(v)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "robots"))
    ap.add_argument("--apply", action="store_true", help="write the missing drives")
    ap.add_argument("--dry-run", action="store_true", help="with --apply: print but do not write")
    ap.add_argument(
        "--include",
        action="append",
        default=[],
        metavar="GLOB",
        help="only touch paths containing this substring (repeatable), e.g. grippers/",
    )
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.root, "**", "*.usda"), recursive=True))
    if args.include:
        files = [
            f
            for f in files
            if any(pat in os.path.relpath(f, args.root) for pat in args.include)
        ]
    total = bad = fixable = skipped = changed_files = 0
    report: dict[str, list[str]] = {}

    for path in files:
        joints = parse_joints(path)
        by_name = {j.name: j for j in joints}
        pending: list[str] = []
        fixes: list[tuple[Joint, list[str]]] = []
        for j in joints:
            if not j.has_mimic:
                continue
            total += 1
            if j.has_drive:
                continue
            bad += 1
            plan = plan_fix(j, by_name)
            if plan is None:
                skipped += 1
                pending.append(f"  SKIP {j.name} (leader={j.leader or '?'})")
                continue
            new_lines, note = plan
            fixable += 1
            pending.append(f"  FIX  {j.name:26} {note}")
            fixes.append((j, new_lines))
        if pending:
            report[os.path.relpath(path, args.root)] = pending
        if not (args.apply and fixes):
            continue
        lines = open(path, encoding="utf-8", errors="replace").read().splitlines()
        # apply bottom-up so earlier line numbers stay valid
        for joint, new_lines in sorted(fixes, key=lambda x: x[0].start, reverse=True):
            lines = lines[: joint.start] + new_lines + lines[joint.end + 1 :]
        if not args.dry_run:
            with open(path, "w", encoding="utf-8") as f:
                f.write("\n".join(lines) + "\n")
        changed_files += 1

    for path, items in report.items():
        print(path)
        print("\n".join(items))

    print()
    print(
        f"mimic joints: {total} | without active drive: {bad} | "
        f"fixable: {fixable} | skipped: {skipped} | files touched: {changed_files}"
    )
    if args.apply and args.dry_run:
        print("(dry-run: nothing written)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

