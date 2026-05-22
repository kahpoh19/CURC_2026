# Tools 使用说明

本目录包含两个代码生成工具，配合 [src/Motions.cpp](../src/Motions.cpp) 中的标记注释使用。

---

## 工具 1：动作帧数据更新

> `json_to_motion.py` — 将调试平台导出的舵机角度 JSON 转为 C++ 帧代码

### 用法

```bash
# 从调试平台导出 JSON（如 try.json）放到 tools/ 目录，然后：
python3 tools/json_to_motion.py tools/try.json --apply
```

### 更新范围

只替换 Motions.cpp 中标记区域内的内容，包括：

| 类型 | 动作 | 变量前缀 |
|------|------|----------|
| 姿势 | Stand | `STAND` |
| 姿势 | Squad | `SQUAD` |
| 步态 | Forward | `FORWARD` |
| 步态 | Backward | `BACKWARD` |
| 步态 | Moveleft | `MOVELEFT` |
| 步态 | MoveRight | `MOVERIGHT` |
| 步态 | RotateLeft | `ROTATELEFT` |
| 步态 | RotateRight | `ROTATERIGHT` |

> LB/RB/LT/RT/X/B 的拳头和起身动作不在 JSON 中管理，不会受影响。

### 其他用法

```bash
# 只看输出，不修改文件
python3 tools/json_to_motion.py tools/try.json

# 保存到文件方便查看
python3 tools/json_to_motion.py tools/try.json > tools/generated_motion_code.cpp
```

---

## 工具 2：按键映射更新

> `apply_button_map.py` — 将 `button_map.json` 中的按键→动作映射写入 handleRemoteActions()

### 用法

1. 编辑 [button_map.json](button_map.json) 中按键的 `action` 字段
2. 运行：

```bash
python3 tools/apply_button_map.py tools/button_map.json --apply
```

### button_map.json 格式

```json
{
  "buttons": {
    "按键名": {"action": "动作函数名", "arm": true/false}
  }
}
```

每个按键的字段：

| 字段 | 必填 | 说明 |
|------|------|------|
| `action` | 是 | 动作函数名，见下方列表 |
| `arm` | 否 | `true` 表示按下后激活 `motionArmed`，摇杆可接管 |
| `special` | 否 | `"unload"` 仅用于 Select 的卸载舵机功能 |

### 可用动作函数

| 函数 | 效果 |
|------|------|
| `applyStandPose` | 站立 |
| `playSquadMotion` | 蹲姿 |
| `playLeftPunch` | 左直拳 |
| `playRightPunch` | 右直拳 |
| `playLeftHookPunch` | 左勾拳 |
| `playRightHookPunch` | 右勾拳 |
| `playFrontGetUpMotion` | 正面起身 |
| `playBackGetUpMotion` | 背面起身 |
| `unloadAllServos` | 卸载所有舵机 |

### 按键名与手柄物理按键对照

| JSON 键名 | 手柄按键 | 通道 |
|-----------|----------|------|
| `Start` | 右小键 (三横) | SYSTEM HIGH |
| `Select` | 左小键 (双框) | SYSTEM LOW |
| `Y` | Y 键 (上) | POSE HIGH |
| `A` | A 键 (下) | POSE LOW |
| `B` | B 键 (右) | GETUP HIGH |
| `X` | X 键 (左) | GETUP LOW |
| `LB` | 左肩键 | PUNCH LOW |
| `RB` | 右肩键 | PUNCH HIGH |
| `LT` | 左扳机 | HOOK LOW |
| `RT` | 右扳机 | HOOK HIGH |

> 按键→通道的对应关系由工具内部按 FlyDigi Apex 5 / XInput 协议处理，无需在 JSON 中指定。

### 示例：把 B 键改为蹲姿

```json
"B": {"action": "playSquadMotion"}
```

运行 `--apply` 后重新编译上传即可。

### 其他用法

```bash
# 预览生成的代码，不修改文件
python3 tools/apply_button_map.py tools/button_map.json --dry-run
```

---

## 文件结构

```
tools/
├── README.md                 ← 本文件
├── json_to_motion.py         # 工具1：舵机角度转换
├── button_map.json           # 工具2：按键映射配置（你编辑这个）
├── apply_button_map.py       # 工具2：按键映射应用
├── try.json                  # 调试平台导出的动作数据
└── generated_motion_code.cpp # 工具1 生成的预览文件（可选）
```

## Motions.cpp 中的标记区域

三个标记区域互不重叠，各自独立管理：

```
Line ~30:  ═══ AUTO-GENERATED POSES BEGIN ═══
           ... 工具1 管理：STAND + SQUAD 帧数据 ...
Line ~57:  ═══ AUTO-GENERATED POSES END ═══

           ... 手写代码：LB/LT/RB/RT/X/B 的拳头动作 ...

Line ~206: ═══ AUTO-GENERATED GAIT MOTIONS BEGIN ═══
           ... 工具1 管理：FORWARD/BACKWARD/MOVELEFT/MOVERIGHT/ROTATELEFT/ROTATERIGHT 帧数据 ...
Line ~477: ═══ AUTO-GENERATED GAIT MOTIONS END ═══

           ... applyFrame / playMotion / playXxxMotion 等辅助函数 ...

Line ~587: ═══ AUTO-GENERATED BUTTON MAP BEGIN ═══
           ... 工具2 管理：handleRemoteActions() 函数体 ...
Line ~669: ═══ AUTO-GENERATED BUTTON MAP END ═══

           ... handleMotionCommand() 摇杆控制 ...
```

## 典型工作流

```bash
# 1. 调试平台调好动作 → 导出 JSON → 放到 tools/
# 2. 更新舵机角度
python3 tools/json_to_motion.py tools/try.json --apply

# 3. 如果需要改键位映射 → 编辑 tools/button_map.json
# 4. 更新键位
python3 tools/apply_button_map.py tools/button_map.json --apply

# 5. 编译上传
pio run -t upload
```
