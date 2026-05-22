#!/usr/bin/env python3
"""
apply_button_map.py — 将按钮→动作映射表写入 Motions.cpp 的 handleRemoteActions()

用法:
    # 查看当前映射会生成什么
    python3 tools/apply_button_map.py tools/button_map.json

    # 预览输出（不修改文件）
    python3 tools/apply_button_map.py tools/button_map.json --dry-run

    # 直接更新 src/Motions.cpp
    python3 tools/apply_button_map.py tools/button_map.json --apply

配置文件格式 (button_map.json):
    {
      "buttons": {
        "Start":  {"action": "applyStandPose", "arm": true},
        "Select": {"action": "unloadAllServos", "special": "unload"},
        "Y":      {"action": "applyStandPose", "arm": true},
        "A":      {"action": "playSquadMotion"},
        "B":      {"action": "playBackGetUpMotion"},
        "X":      {"action": "playFrontGetUpMotion"},
        "LB":     {"action": "playLeftPunch"},
        "RB":     {"action": "playRightPunch"},
        "LT":     {"action": "playLeftHookPunch"},
        "RT":     {"action": "playRightHookPunch"}
      }
    }

每个按键的字段:
    action  — 要调用的动作函数名 (必需)
    arm     — 是否设置 motionArmed = true (可选，默认 false)
    special — 特殊模式: "unload" 会加上状态重置 + logPrintln (可选)

FlyDigi Apex 5 按键→通道/区间对照（工具内部已知，无需在 JSON 中指定）:
    Start  → SYSTEM HIGH    Select → SYSTEM LOW
    Y      → POSE HIGH      A      → POSE LOW
    B      → GETUP HIGH     X      → GETUP LOW
    LB     → PUNCH LOW      RB     → PUNCH HIGH
    LT     → HOOK LOW       RT     → HOOK HIGH

Motions.cpp 中的标记（由本工具识别和替换）:
    // ═══ AUTO-GENERATED BUTTON MAP BEGIN (tools/apply_button_map.py --apply) ═══
    // ... handleRemoteActions() 函数体 ...
    // ═══ AUTO-GENERATED BUTTON MAP END ═══
"""

import json
import os
import sys
from typing import Optional

# ── 按键 → (通道常量, 区间) ───────────────────────────────────────
# FlyDigi Apex 5 / 标准 XInput 映射
BUTTON_CHANNEL = {
    "Start":  ("REMOTE_SYSTEM_CHANNEL", "SWITCH_HIGH_MIN_US", "SWITCH_HIGH_MAX_US", "1"),
    "Select": ("REMOTE_SYSTEM_CHANNEL", "SWITCH_LOW_MIN_US",  "SWITCH_LOW_MAX_US",  "0"),
    "Y":      ("REMOTE_POSE_CHANNEL",   "SWITCH_HIGH_MIN_US", "SWITCH_HIGH_MAX_US", "1"),
    "A":      ("REMOTE_POSE_CHANNEL",   "SWITCH_LOW_MIN_US",  "SWITCH_LOW_MAX_US",  "0"),
    "B":      ("REMOTE_GETUP_CHANNEL",  "SWITCH_HIGH_MIN_US", "SWITCH_HIGH_MAX_US", "1"),
    "X":      ("REMOTE_GETUP_CHANNEL",  "SWITCH_LOW_MIN_US",  "SWITCH_LOW_MAX_US",  "0"),
    "LB":     ("REMOTE_PUNCH_CHANNEL",  "SWITCH_LOW_MIN_US",  "SWITCH_LOW_MAX_US",  "0"),
    "RB":     ("REMOTE_PUNCH_CHANNEL",  "SWITCH_HIGH_MIN_US", "SWITCH_HIGH_MAX_US", "1"),
    "LT":     ("REMOTE_HOOK_CHANNEL",   "SWITCH_LOW_MIN_US",  "SWITCH_LOW_MAX_US",  "0"),
    "RT":     ("REMOTE_HOOK_CHANNEL",   "SWITCH_HIGH_MIN_US", "SWITCH_HIGH_MAX_US", "1"),
}

# Motions.cpp 路径
MOTIONS_CPP = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           os.pardir, "src", "Motions.cpp")

BEGIN_MARKER = "// ═══ AUTO-GENERATED BUTTON MAP BEGIN (tools/apply_button_map.py --apply) ═══"
END_MARKER   = "// ═══ AUTO-GENERATED BUTTON MAP END ═══"


def generate_handle_remote_actions(buttons: dict) -> str:
    """Generate the complete handleRemoteActions() function body."""

    lines = [
        "bool handleRemoteActions(const RemoteSnapshot &snapshot) {",
        "  bool actionRan = false;",
        "",
    ]

    for btn_name in BUTTON_CHANNEL:
        if btn_name not in buttons:
            continue

        cfg = buttons[btn_name]
        action = cfg.get("action", "")
        arm = cfg.get("arm", False)
        special = cfg.get("special", "")

        channel, lo_min, lo_max, zone = BUTTON_CHANNEL[btn_name]

        # consumeSwitchZone call
        lines.append(
            f"  if (consumeSwitchZone({channel},"
        )
        lines.append(
            f"                        snapshot.channels[{channel}],"
        )
        lines.append(
            f"                        {lo_min}, {lo_max}, {zone})) {{"
        )

        # preamble for special modes
        if special == "unload":
            lines.append(f'    logPrintln("Remote action: unload servos");')
            lines.append(f"    motionArmed = false;")
            lines.append(f"    idlePoseApplied = false;")
            lines.append(f"    returnToStandPending = false;")
            lines.append(f"    {action}();")
        elif arm:
            lines.append(f"    motionArmed = true;")
            lines.append(f"    {action}();")
        else:
            lines.append(f"    {action}();")

        lines.append(f"    actionRan = true;")
        lines.append(f"  }}")
        lines.append("")

    lines.extend([
        "  return actionRan;",
        "}",
    ])

    return "\n".join(lines)


def replace_between_markers(text: str, replacement: str) -> str:
    """Replace everything between BEGIN and END markers.
    Markers themselves are preserved."""
    content_lines = text.splitlines(keepends=True)
    bi = next(i for i, ln in enumerate(content_lines) if BEGIN_MARKER in ln)
    ei = next(i for i, ln in enumerate(content_lines) if END_MARKER in ln and i > bi)
    new_lines = content_lines[:bi + 1] + ["\n", replacement, "\n"] + content_lines[ei:]
    return "".join(new_lines)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = [a for a in sys.argv[1:] if a.startswith("--")]
    do_apply = "--apply" in flags
    dry_run = "--dry-run" in flags

    # ── resolve config path ────────────────────────────────────────
    if args:
        cfg_path = args[0]
        if not os.path.isabs(cfg_path):
            cfg_path = os.path.abspath(cfg_path)
    else:
        default = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "button_map.json")
        cfg_path = default

    if not os.path.exists(cfg_path):
        print(f"Error: config not found: {cfg_path}", file=sys.stderr)
        print(f"Usage: python3 {__file__} [button_map.json] [--apply | --dry-run]",
              file=sys.stderr)
        sys.exit(1)

    with open(cfg_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    buttons = data.get("buttons", {})
    if not buttons:
        print("Error: no buttons in config", file=sys.stderr)
        sys.exit(1)

    # ── generate ───────────────────────────────────────────────────
    generated = generate_handle_remote_actions(buttons)

    if dry_run:
        print(generated)
        return

    if do_apply:
        if not os.path.exists(MOTIONS_CPP):
            print(f"Error: {MOTIONS_CPP} not found", file=sys.stderr)
            sys.exit(1)

        with open(MOTIONS_CPP, "r", encoding="utf-8") as f:
            original = f.read()

        modified = replace_between_markers(original, generated)

        with open(MOTIONS_CPP, "w", encoding="utf-8") as f:
            f.write(modified)

        actions = [b.get("action", "?") for b in buttons.values()]
        print(f">>> Motions.cpp updated: {len(buttons)} buttons mapped → "
              f"{len(set(actions))} actions.",
              file=sys.stderr)
    else:
        print(generated)


if __name__ == "__main__":
    main()
