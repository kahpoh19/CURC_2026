#!/usr/bin/env python3
"""
json_to_motion.py — 将调试平台导出的动作 JSON 转为 C++ 动作帧代码，并可直接更新 Motions.cpp

用法:
    # 输出 C++ 代码到终端
    python3 tools/json_to_motion.py tools/try.json

    # 保存到文件
    python3 tools/json_to_motion.py tools/try.json > tools/generated_motion_code.cpp

    # 直接更新 src/Motions.cpp（替换标记区域内的帧数据）
    python3 tools/json_to_motion.py tools/try.json --apply

Motivations.cpp 中的标记注释（由本脚本识别）:
    // ═══ AUTO-GENERATED POSES BEGIN (tools/json_to_motion.py --apply) ═══
    // ... STAND / SQUAD 帧数据 ...
    // ═══ AUTO-GENERATED POSES END ═══

    // ═══ AUTO-GENERATED GAIT MOTIONS BEGIN (tools/json_to_motion.py --apply) ═══
    // ... FORWARD / BACKWARD / MOVELEFT / MOVERIGHT / ROTATELEFT / ROTATERIGHT ...
    // ═══ AUTO-GENERATED GAIT MOTIONS END ═══

    LB / RB / LT / RT / X / B 的手写帧数据在标记外，不会被覆盖。

JSON 格式 (来自调试平台):
    {
      "groups": [
        {
          "name": "Stand",
          "frames": [
            {
              "angles": [{"id": 0, "angle": -30.0}, ...],
              "time": 500,       // → moveMs
              "delay": 0         // → delayMs (null → 0)
            }
          ]
        }
      ]
    }
"""

import json
import re
import os
import sys
from typing import Optional

# ── config ──────────────────────────────────────────────────────────
SERVOS_PER_LINE = 5
SERVO_COUNT = 17

# 中文名 → C++ 变量前缀
CN_NAME_MAP = {"默认": "DEFAULT"}

# JSON group name → Motions.cpp 中实际使用的 C++ 变量前缀
# (有些名字不符合纯 camel→snake 规则，需要手动对齐)
NAME_OVERRIDE = {
    # JSON 里的 camelCase 名需要对齐 Motions.cpp 中的实际变量前缀
    "MoveLeft": "MOVELEFT",
    "MoveRight": "MOVERIGHT",
    "RotateLeft": "ROTATELEFT",
    "RotateRight": "ROTATERIGHT",
    # v2.0 新动作组名
    "stand 2.0": "STAND",
    "squad 2.0": "SQUAD",
    "Moveleft 2.0": "MOVELEFT",
    "Moveright 2.0": "MOVERIGHT",
    "backward 2.0": "BACKWARD",
    "Rotate Left 2.0": "ROTATELEFT",
    " Rotate Right 2.0": "ROTATERIGHT",
    "Defence": "DEFENCE",
    "Final Kick": "FINAL_KICK",
    "Punch": "PUNCH",
    "Dance": "DANCE",
    "standup back": "STANDUP_BACK",
    "standup front": "STANDUP_FRONT",
}

# 哪些 group 放入 "poses" 区块，哪些放入 "gait" 区块
POSE_GROUPS = {"默认", "stand 2.0", "squad 2.0",
               "Defence", "Final Kick", "Punch", "Dance",
               "standup back", "standup front"}
GAIT_GROUPS = {"Forward", "backward 2.0", "Moveleft 2.0", "Moveright 2.0",
               "Rotate Left 2.0", " Rotate Right 2.0"}

# Motions.cpp 路径（相对于脚本所在 tools/ 目录）
MOTIONS_CPP = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           os.pardir, "src", "Motions.cpp")

MARKER_POSES_BEGIN = "// ═══ AUTO-GENERATED POSES BEGIN (tools/json_to_motion.py --apply) ═══"
MARKER_POSES_END   = "// ═══ AUTO-GENERATED POSES END ═══"
MARKER_GAIT_BEGIN  = "// ═══ AUTO-GENERATED GAIT MOTIONS BEGIN (tools/json_to_motion.py --apply) ═══"
MARKER_GAIT_END    = "// ═══ AUTO-GENERATED GAIT MOTIONS END ═══"


# ═════════════════════════════════════════════════════════════════════
# naming
# ═════════════════════════════════════════════════════════════════════

def name_to_cpp(name: str) -> Optional[str]:
    """JSON group name → C++ identifier prefix (e.g. 'RotateLeft' → 'ROTATELEFT')"""
    if name in CN_NAME_MAP:
        return CN_NAME_MAP[name]
    if name in NAME_OVERRIDE:
        return NAME_OVERRIDE[name]
    if not name.isascii():
        return None
    s = re.sub(r'([A-Z])', r'_\1', name)
    return s.strip('_').upper()


# ═════════════════════════════════════════════════════════════════════
# C++ code generators
# ═════════════════════════════════════════════════════════════════════

def fmt_angle(a: float) -> str:
    return f"{a:.1f}f"


def gen_frame_array(cpp_name: str, frame_idx: int, angles: list) -> str:
    """Generate a single ServoAngle[] definition."""
    var = f"{cpp_name}_FRAME_{frame_idx}"
    lines = [f"static constexpr ServoAngle {var}[] = {{"]
    for i in range(0, len(angles), SERVOS_PER_LINE):
        batch = angles[i:i + SERVOS_PER_LINE]
        parts = [f"{{{a['id']}, {fmt_angle(a['angle'])}}}" for a in batch]
        lines.append("    " + ", ".join(parts) + ",")
    lines.append("};")
    return "\n".join(lines)


def gen_motion_array(cpp_name: str, frames: list) -> str:
    """Generate a MotionFrame[] definition."""
    var = f"{cpp_name}_MOTION"
    lines = [f"static constexpr MotionFrame {var}[] = {{"]
    for i, f in enumerate(frames):
        fvar = f"{cpp_name}_FRAME_{i}"
        ms = f.get("time", 200)
        dly = f.get("delay") or 0
        lines.append(f"    {{{fvar}, ARRAY_COUNT({fvar}), {ms}, {dly}}},")
    lines.append("};")
    return "\n".join(lines)


def gen_group(group: dict) -> Optional[str]:
    """Generate all C++ frame data for one motion group."""
    name = group["name"]
    cpp = name_to_cpp(name)
    if cpp is None:
        return None
    frames = group.get("frames", [])
    if not frames:
        return None

    parts = [f"// {name}"]
    for i, frame in enumerate(frames):
        angles = frame.get("angles", [])
        # fill missing servo IDs with angle=0
        seen = {a["id"] for a in angles}
        full = list(angles)
        for mid in range(SERVO_COUNT):
            if mid not in seen:
                full.append({"id": mid, "angle": 0.0})
        full.sort(key=lambda a: a["id"])
        parts.append(gen_frame_array(cpp, i, full))
        parts.append("")
    parts.append(gen_motion_array(cpp, frames))
    parts.append("")
    return "\n".join(parts)


# ═════════════════════════════════════════════════════════════════════
# marker-based replacement in Motions.cpp
# ═════════════════════════════════════════════════════════════════════

def replace_between_markers(text: str, begin: str, end: str,
                            replacement: str) -> str:
    """Replace everything between `begin` and `end` markers.
    Markers themselves are preserved. Raises ValueError if not found."""
    lines = text.splitlines(keepends=True)
    bi = next(i for i, ln in enumerate(lines) if begin in ln)
    ei = next(i for i, ln in enumerate(lines) if end in ln and i > bi)

    # Keep the marker lines, replace only the content between them
    new_lines = lines[:bi + 1] + [replacement] + lines[ei:]
    return "".join(new_lines)


def apply_to_motions(groups: list):
    """Read Motions.cpp, replace marked sections, write back."""
    if not os.path.exists(MOTIONS_CPP):
        print(f"Error: {MOTIONS_CPP} not found", file=sys.stderr)
        sys.exit(1)

    with open(MOTIONS_CPP, "r", encoding="utf-8") as f:
        original = f.read()

    # ── build pose section ──────────────────────────────────────────
    pose_blocks = []
    for g in groups:
        name = g["name"]
        cpp = name_to_cpp(name)
        if cpp is None or name not in POSE_GROUPS:
            continue
        block = gen_group(g)
        if block:
            pose_blocks.append(block)

    # ── build gait section ──────────────────────────────────────────
    gait_blocks = []
    for g in groups:
        name = g["name"]
        cpp = name_to_cpp(name)
        if cpp is None or name not in GAIT_GROUPS:
            continue
        block = gen_group(g)
        if block:
            gait_blocks.append(block)

    # ── replace in Motions.cpp ──────────────────────────────────────
    modified = original

    if pose_blocks:
        replacement = "\n" + "\n\n".join(pose_blocks) + "\n"
        modified = replace_between_markers(
            modified, MARKER_POSES_BEGIN, MARKER_POSES_END, replacement)
        pose_count = len(pose_blocks)
    else:
        pose_count = 0

    if gait_blocks:
        replacement = "\n" + "\n\n".join(gait_blocks) + "\n"
        modified = replace_between_markers(
            modified, MARKER_GAIT_BEGIN, MARKER_GAIT_END, replacement)
        gait_count = len(gait_blocks)
    else:
        gait_count = 0

    # ── write back ──────────────────────────────────────────────────
    with open(MOTIONS_CPP, "w", encoding="utf-8") as f:
        f.write(modified)

    return pose_count, gait_count


# ═════════════════════════════════════════════════════════════════════
# main
# ═════════════════════════════════════════════════════════════════════

def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = [a for a in sys.argv[1:] if a.startswith("--")]
    do_apply = "--apply" in flags

    script_dir = os.path.dirname(os.path.abspath(__file__))

    # ── resolve JSON path ───────────────────────────────────────────
    if args:
        json_path = args[0]
        if not os.path.isabs(json_path):
            json_path = os.path.abspath(json_path)
    else:
        candidates = [
            os.path.join(os.getcwd(), "try.json"),
            os.path.join(script_dir, os.pardir,
                         "fashionstar_project_v5.6 movement v1.0.json"),
        ]
        json_path = next((c for c in candidates if os.path.exists(c)), "")

    if not json_path or not os.path.exists(json_path):
        print(f"Error: JSON not found", file=sys.stderr)
        print(f"Usage: python3 {__file__} [path/to/motion.json] [--apply]",
              file=sys.stderr)
        sys.exit(1)

    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    groups = data.get("groups", [])
    if not groups:
        print("Error: no groups in JSON", file=sys.stderr)
        sys.exit(1)

    # ── --apply mode: directly update Motions.cpp ───────────────────
    if do_apply:
        p, g = apply_to_motions(groups)
        src = os.path.basename(json_path)
        print(f">>> Motions.cpp updated from {src}: "
              f"{p} pose groups, {g} gait groups.",
              file=sys.stderr)
        return

    # ── stdout mode: print C++ code ─────────────────────────────────
    print("// 自动生成的动作帧代码")
    print(f"// 来源: {os.path.basename(json_path)}")
    print(f"// 工具: tools/json_to_motion.py")
    print(f"// 动作组数: {len(groups)}")
    print(f"// 舵机数: {SERVO_COUNT} (ID 0-{SERVO_COUNT - 1})")
    print(f"//")
    print(f"// 用法: 粘贴到 src/Motions.cpp 中对应标记区域")
    print(f"//       或运行: python3 tools/json_to_motion.py --apply 自动替换")
    print()

    for group in groups:
        block = gen_group(group)
        if block:
            print(block)

    frame_count = sum(len(g["frames"]) for g in groups)
    print(f"\n>>> Done. {len(groups)} groups, {frame_count} frames. "
          f"Use --apply to auto-update Motions.cpp.",
          file=sys.stderr)


if __name__ == "__main__":
    main()
