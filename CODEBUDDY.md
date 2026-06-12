# CODEBUDDY.md This file provides guidance to CodeBuddy when working with code in this repository.

## 项目概述

车载毫米波雷达仿真与深度学习目标检测科研项目。核心流程：**MATLAB 仿真 FMCW 雷达信号 → 生成 Range-Doppler 图谱 → Python 转换为 COCO 格式 → 训练 EfficientDet-D0 神经网络 → 目标检测验证**。

项目基于 Thomas Wengerter 等人的 IEEE 论文 "Simulation of Urban Automotive Radar Measurements for Deep Learning Target Detection" (2022)，模拟 76.5 GHz 车载雷达在城市场景中对行人、自行车和车辆的探测。

## 架构概览

### 数据流

```
MATLAB 仿真层
  FMCWradar.m (雷达参数配置: 76.5GHz, 1GHz 带宽, 256 chirps/cycle, 8 RX 天线)
    → SimulateTargetList.m (多目标仿真入口, 随机生成 0-2 个/类目标)
      → Car.m / Bicyclist.m / Pedestrian.m (多点散射模型, 含微多普勒)
        → TrajectoryPlanner.m (运动轨迹规划)
          → generateObstructionMap.m (目标间遮挡检测)
            → modelBasebandSignal.m → modelSignal.m (基带信号模型)
              → RDmap (160R × 256V × 16 天线 3D 数据立方)
                → saveMat.m (.mat 文件输出)
                  ↓
Python 数据转换层
  JSONCoco.py / JSONCoco_3SeqRGB.py
    → 读取 .mat 文件 → 计算 Range-Doppler 边界框
      → 生成 COCO JSON 标注 + JPG 图像
        ↓
深度学习层
  EfficientDet-D0 (train_efficientdet.py)
    → RD 图谱上检测 Vehicle / Pedestrian / Bicycle
```

### 关键组件关系

| 组件 | 位置 | 职责 |
|------|------|------|
| 雷达参数定义 | `Matlab/.../FMCWradar.m` | 配置载频、带宽、chirp 数、天线数、噪声与杂波参数 |
| 目标模型 | `Matlab/.../Car.m` | 车辆多点反射模型（高斯 + RCS 采样，含轮毂微多普勒） |
| | `Matlab/.../Bicyclist.m` | 自行车反射模型（框架 + 旋转车轮） |
| | `Matlab/.../Pedestrian.m` | 行人反射模型（12 个身体部位散射点） |
| 系统仿真 | `Matlab/.../TargetSimulation.m` | 单目标仿真入口（支持 Pedestrian/Bicycle/Car/Synthetic/Noise 模式） |
| | `Matlab/.../SimulateTargetList.m` | 多目标仿真入口（主要工作流） |
| 轨迹规划 | `Matlab/.../TrajectoryPlanner.m` | 为每个目标生成运动路径 |
| 信号模型 | `Matlab/.../modelBasebandSignal.m` | 单目标基带信号计算 |
| | `Matlab/.../modelSignal.m` | 多目标反射叠加 |
| 数据转换 | `Matlab/.../JSONCoco.py` | MATLAB .mat → COCO JSON + JPG |
| | `Matlab/.../JSONCoco_3SeqRGB.py` | 3 帧序列叠加为 RGB 图像的 COCO 转换 |
| 训练 | `train_efficientdet.py` | EfficientDet-D0 训练脚本 |
| 模型权重 | `efficientdet_d0_epoch5.pth` | 5 轮训练权重 (44.8 MB) |
| | `efficientdet_d0_epoch10.pth` | 10 轮训练权重 (44.8 MB) |

### 仿真数据组织

- `Matlab/.../SimulationData/`: 已生成 50 个场景 (Szenario1~50)，每场景含多帧 .mat 文件
- `Matlab/.../COCO/annotations/instances_train2017.json`: COCO 标注文件
- `Matlab/.../COCO/images/train2017/`: RD 图谱灰度 JPG (256×160 px)
- 论文中生成 600 个交通场景（各 1 秒），最多 6 个目标/场景，约 30,000+ 帧

### 目标配置入口

- **雷达参数**: `FMCWradar.m` properties 段
- **场景数量/时长**: `SimulateTargetList.m` 中 `Szenarios` 和 `duration`
- **目标概率**: `SimulateTargetList.m` 中 `floor(2.3*rand())`
- **COCO 输出控制**: `JSONCoco.py` 中 `uniformBoxes`, `writeJPGs`, `drawBoxes`

## 编码规范

### 特殊约定
- **不可变性原则**: 禁止直接修改现有对象，始终返回新对象；禁止原地操作数组/对象；优先使用纯函数
- **错误处理**: 分层处理，禁止静默处理（禁止空 except 块），用户友好提示
- **输入验证**: 边界验证、快速失败、零信任原则
- **文件限制**: 单文件不超过 800 行，单函数不超过 50 行，嵌套不超过 4 层

### MATLAB 编码规则
- **初学者友好**: 完整实现不省略，详细注释
- **线性编码优先**: 避免过度函数封装
- **维度安全**: 显式指定 `mean/sum/std` 维度，校验向量方向
- **数值稳定性**: 使用 `eps` 保护，`log(0)` 需防护
- **注释规范**: 每个段落、维度变化、物理公式都需注释

### Python 编码规则
- 使用中文注释和 docstring
- 复杂逻辑必须注释，公共函数添加 docstring

## 环境与依赖

### Python 环境
- 通过 Miniconda3 管理 (`/home/TangXuebiao/miniconda3/`)
- Conda 初始化已在 `.bashrc` 中配置
- 主要依赖: `numpy`, `scipy`, `opencv-python`, `matplotlib`, `torch`, `effdet`

### MATLAB 依赖
- MATLAB ≥ R2020a
- Phased Array Toolbox (`backscatterPedestrian`)

## 常见命令

### 运行 MATLAB 仿真
```bash
# 启动 MATLAB 并运行仿真
cd /home/TangXuebiao/Matlab/FMCW_Radar_Target_Simulator
matlab -nodisplay -r "SimulateTargetList; exit"
```
### 转换数据为 COCO 格式
```bash
cd /home/TangXuebiao/Matlab/FMCW_Radar_Target_Simulator
python JSONCoco.py
```
### 训练 EfficientDet
```bash
python train_efficientdet.py
# 或指定 epoch 数
python train_efficientdet.py --epochs 10
```
### 单目标仿真
```bash
# 在 MATLAB 中运行单目标测试
matlab -nodisplay -r "TargetSimulation('Pedestrian'); exit"
```


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
