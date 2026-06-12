# CLAUDE.md

本文件为 Claude Code 提供此代码库的操作指引。

## 项目概述

车载毫米波雷达仿真与深度学习目标检测科研项目，基于 Thomas Wengerter 等人 IEEE 论文 "Simulation of Urban Automotive Radar Measurements for Deep Learning Target Detection"。核心流程：**MATLAB 仿真 FMCW 雷达信号 → Python 转换为 COCO 格式 → 训练 EfficientDet-D0 → 目标检测验证**。

## 数据流架构

```
MATLAB 仿真层
  FMCWradar.m (76.5GHz, 1GHz带宽, 256chirps/cycle, 8RX天线)
    → SimulateTargetList.m (多目标仿真入口, 随机生成0-2个/类)
      → Car.m / Bicyclist.m / Pedestrian.m (多点散射模型+微多普勒)
        → TrajectoryPlanner.m (运动轨迹规划)
          → generateObstructionMap.m (目标间遮挡检测)
            → modelBasebandSignal.m + modelSignal.m (基带信号计算与叠加)
              → .mat输出 (160R × 256V × 16天线 3D数据立方)
                ↓
Python 数据转换层
  JSONCoco.py / JSONCoco_3SeqRGB.py
    → .mat → COCO JSON标注 + JPG图像 (RD图谱 256×160px)
      ↓
深度学习层
  EfficientDet-D0
    → RD图谱上检测 Vehicle / Pedestrian / Bicycle
```

## 关键文件位置

### MATLAB 仿真 (核心项目)
- `Matlab/FMCW_Radar_Target_Simulator/FMCWradar.m` — 雷达参数配置（载频、带宽、chirp数、噪声与杂波）
- `Matlab/FMCW_Radar_Target_Simulator/SimulateTargetList.m` — 多目标仿真主入口
- `Matlab/FMCW_Radar_Target_Simulator/TargetSimulation.m` — 单目标仿真入口
- `Matlab/FMCW_Radar_Target_Simulator/Car.m` — 车辆多点反射模型（高斯+RCS采样+轮毂微多普勒）
- `Matlab/FMCW_Radar_Target_Simulator/Bicyclist.m` — 自行车反射模型（框架+旋转车轮）
- `Matlab/FMCW_Radar_Target_Simulator/Pedestrian.m` — 行人反射模型（12个身体部位散射点）
- `Matlab/FMCW_Radar_Target_Simulator/TrajectoryPlanner.m` — 目标运动轨迹生成
- `Matlab/FMCW_Radar_Target_Simulator/modelBasebandSignal.m` — 单目标基带信号计算
- `Matlab/FMCW_Radar_Target_Simulator/modelSignal.m` — 多目标反射信号叠加
- `Matlab/FMCW_Radar_Target_Simulator/SimulationData/` — 仿真输出 .mat 文件（Szenario1~50）

### Python 数据转换
- `Matlab/FMCW_Radar_Target_Simulator/JSONCoco.py` — .mat → COCO JSON + 灰度JPG
- `Matlab/FMCW_Radar_Target_Simulator/JSONCoco_3SeqRGB.py` — 3帧序列叠加为RGB图像的COCO转换

### 模型权重
- `efficientdet_d0_epoch5.pth` — 5轮训练权重
- `efficientdet_d0_epoch10.pth` — 10轮训练权重
- `Matlab/FMCW_Radar_Target_Simulator/efficientdet_d0_epoch5.pth` — 副本

## 常用命令

### 运行 MATLAB 仿真
```bash
cd /home/TangXuebiao/Matlab/FMCW_Radar_Target_Simulator
matlab -nodisplay -r "SimulateTargetList; exit"
```

### 单目标仿真
```bash
matlab -nodisplay -r "TargetSimulation('Pedestrian'); exit"
```

### 转换数据为 COCO 格式
```bash
cd /home/TangXuebiao/Matlab/FMCW_Radar_Target_Simulator
python JSONCoco.py
```

### 训练 EfficientDet
```bash
python train_efficientdet.py --epochs 10
```

## 配置入口

| 配置项 | 位置 | 说明 |
|--------|------|------|
| 雷达参数 | `FMCWradar.m` properties 段 | 载频、带宽、chirp数、天线数、噪声杂波 |
| 场景数量/时长 | `SimulateTargetList.m` 中 Szenarios 和 duration | 默认50场景，每场景多帧 |
| 目标概率 | `SimulateTargetList.m` 中 `floor(2.3*rand())` | 每类最多2个目标 |
| COCO输出控制 | `JSONCoco.py` | `uniformBoxes`、`writeJPGs`、`drawBoxes` |

## 标签格式

多目标仿真标签：`[Range, velocity, azimuth, RadarVelocity, xPosition, yPosition, width, height, heading, obstruction]`

## 环境依赖

- **MATLAB** ≥ R2020a + Phased Array Toolbox
- **Python** (Miniconda3, `/home/TangXuebiao/miniconda3/`): numpy, scipy, opencv-python, matplotlib, torch, effdet
- **Conda** 初始化已在 `.bashrc` 中配置

## 详细文档

- 更完整的项目说明见 [CODEBUDDY.md](CODEBUDDY.md)
- MATLAB 项目原始README见 [Matlab项目README](Matlab/FMCW_Radar_Target_Simulator/README.md)
- 论文笔记见 [docs/Simulation_of_Urban_Automotive_Radar_Measurements_for_Deep_Learning_Target_Detection.md](docs/Simulation_of_Urban_Automotive_Radar_Measurements_for_Deep_Learning_Target_Detection.md)

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
