# REASONIX.md — FMCW Radar Target Simulator

## Stack

- **MATLAB** ≥R2020a + Phased Array Toolbox — radar signal simulation
  [FMCWradar.m](Matlab/FMCW_Radar_Target_Simulator/FMCWradar.m:1)
- **Python 3** (Miniconda3) — COCO conversion + ML training
  [JSONCoco.py](Matlab/FMCW_Radar_Target_Simulator/JSONCoco.py:1)
- **PyTorch + effdet** — EfficientDet-D0 on Range-Doppler maps
  [train_efficientdet.py](Matlab/FMCW_Radar_Target_Simulator/train_efficientDet/train_efficientdet.py)
- **numpy, scipy, opencv-python** — Python data deps
  [JSONCoco.py](Matlab/FMCW_Radar_Target_Simulator/JSONCoco.py:5)

## Layout

- `Matlab/FMCW_Radar_Target_Simulator/` — radar config (`FMCWradar.m`), target models (`Car.m`, `Bicyclist.m`, `Pedestrian.m`), entry points (`SimulateTargetList.m`, `TargetSimulation.m`), signal chain
- `.../train_efficientDet/` + `.../radar_detection/` — training script & Python package
- `.../SimulationData/` — generated `.mat` output (gitignored)
- `.../COCO/` — generated COCO JSON + JPGs (gitignored)
- `docs/` — IEEE paper notes
- Root `efficientdet_d0_epoch*.pth` — trained weights (~45 MB each)

## Commands

```bash
# MATLAB multi-target simulation
matlab -nodisplay -r "SimulateTargetList; exit"
# MATLAB single-target test
matlab -nodisplay -r "TargetSimulation('Pedestrian'); exit"
# Convert .mat to COCO
python Matlab/FMCW_Radar_Target_Simulator/JSONCoco.py
# Train EfficientDet-D0
python Matlab/FMCW_Radar_Target_Simulator/train_efficientDet/train_efficientdet.py --epochs 10
```

## Conventions

- **MATLAB classdef** — `FMCWradar`, `Car`, `Bicyclist`, `Pedestrian` defined as `classdef` with `properties` for config [Car.m](Matlab/FMCW_Radar_Target_Simulator/Car.m:1)
- **Python: Chinese comments + `# -*- coding: utf-8 -*-`** — all project `.py` files [JSONCoco.py](Matlab/FMCW_Radar_Target_Simulator/JSONCoco.py:1)
- **Target label format** — `[Range, velocity, azimuth, RadarVelocity, xPos, yPos, width, height, heading, obstruction]` [README.md](Matlab/FMCW_Radar_Target_Simulator/README.md:48)
- **Radar defaults** — 76.5 GHz, 1 GHz BW, 256 chirps/cycle, 8 RX [FMCWradar.m](Matlab/FMCW_Radar_Target_Simulator/FMCWradar.m:18)
- **.gitignore** — `*.asv`, `SimulationData/`, model weights `.pth` [.gitignore](Matlab/FMCW_Radar_Target_Simulator/.gitignore:1)

## Watch out for

- **SimulationData/ is gitignored** — re-run sim if data missing [.gitignore](Matlab/FMCW_Radar_Target_Simulator/.gitignore:2)
- **GPU acceleration** — manual `for` → `parfor` change in `TargetSimulation.m` [README.md](Matlab/FMCW_Radar_Target_Simulator/README.md:82)
- **Global `c_0`** — `global c_0; c_0 = 299792458;` in entry scripts [TargetSimulation.m](Matlab/FMCW_Radar_Target_Simulator/TargetSimulation.m:8)
- **This is a home dir, not a single repo** — `.claude/`, `.codebuddy/`, `miniconda3/` are personal tooling


# CLAUDE.md

行为准则，用于减少大语言模型（LLM）常见的编码错误。可根据需要与项目特定说明进行合并。

**权衡：** 这些准则偏向于谨慎而非速度。对于简单任务，请自行判断。

## 1. 编码前思考

**不要假设。不要隐藏困惑。呈现权衡方案。**

在实施之前：
- 明确陈述你的假设。如果不确定，请询问。
- 如果存在多种解释，请呈现出来——不要默默地选择。
- 如果存在更简单的方法，请说明。必要时提出反对意见。
- 如果某些内容不清楚，请停下来。指出令人困惑的地方。询问。

## 2. 简洁优先

**用最少的代码解决问题。不做任何推测性的工作。**

- 不添加超出需求的功能。
- 不为一次性代码创建抽象。
- 不添加未被要求的"灵活性"或"可配置性"。
- 不处理不可能发生的场景的错误。
- 如果你写了 200 行代码而其实可以只用 50 行，请重写。

问自己："资深工程师会说这过于复杂吗？"如果是，请简化。

## 3. 精准修改

**只修改必须修改的部分。只清理自己造成的混乱。**

编辑现有代码时：
- 不要"改进"相邻的代码、注释或格式。
- 不要重构未损坏的东西。
- 遵循现有风格，即使你会有不同的做法。
- 如果你注意到无关的死代码，请提及——但不要删除它。

当你的更改产生孤立代码时：
- 删除因你的更改而变得未使用的导入/变量/函数。
- 除非被要求，否则不要删除预先存在的死代码。

检验标准：每一行更改的代码都应该能直接追溯到用户的请求。

## 4. 目标驱动执行

**定义成功标准。循环验证直至通过。**

将任务转化为可验证的目标：
- "添加验证" → "为无效输入编写测试，然后使其通过"
- "修复错误" → "编写重现该错误的测试，然后使其通过"
- "重构 X" → "确保测试在重构前后都能通过"

对于多步骤任务，陈述简要计划：
```
1. [步骤] → 验证：[检查点]
2. [步骤] → 验证：[检查点]
3. [步骤] → 验证：[检查点]
```

明确的成功标准让你能够独立迭代。模糊的标准（"让它工作"）需要不断的澄清。

---

**这些准则有效的表现：** 差异对比中不必要更改更少，因过度复杂而重写的情况更少，澄清问题出现在实施前而非出错后。
