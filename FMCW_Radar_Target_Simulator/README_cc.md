# 车载毫米波雷达仿真与深度学习目标检测 — 使用说明

## 1. 项目概述与功能说明

### 1.1 项目简介

本项目基于 Thomas Wengerter 等人的 IEEE 论文 *"Simulation of Urban Automotive Radar Measurements for Deep Learning Target Detection"*，实现了一个**车载 FMCW（调频连续波）雷达仿真系统**。它能够模拟城市交通场景下的雷达回波信号，生成包含车辆（Vehicle）、行人（Pedestrian）和自行车（Bicycle）三类目标的距离-多普勒（RD）图谱，并支持将仿真数据转换为 COCO 格式，用于训练深度学习目标检测模型（如 EfficientDet-D0）。

### 1.2 核心工作流程

```
┌─────────────────────────────────────────────────────────────────┐
│                    MATLAB 仿真层                                 │
│                                                                 │
│  FMCWradar.m (76.5GHz雷达参数配置)                               │
│      → SimulateTargetList.m (随机生成多目标场景)                 │
│          → Car.m / Bicyclist.m / Pedestrian.m (多点散射模型)    │
│              → TrajectoryPlanner.m (运动轨迹规划)               │
│                  → generateObstructionMap.m (目标间遮挡检测)     │
│                      → modelBasebandSignal.m (基带信号计算)     │
│                          → .mat输出 (160×256×16 数据立方)        │
│                              ↓                                  │
│                    Python 数据转换层                              │
│                                                                 │
│  JSONCoco.py / JSONCoco_3SeqRGB.py                              │
│      → .mat → COCO JSON标注 + JPG图像 (RD图谱 256×160px)       │
│          ↓                                                      │
│                    深度学习层                                    │
│                                                                 │
│  EfficientDet-D0                                                │
│      → RD图谱上检测 Vehicle / Pedestrian / Bicycle              │
└─────────────────────────────────────────────────────────────────┘
```

### 1.3 功能特性

| 功能 | 说明 |
|------|------|
| **多目标仿真** | 随机生成0-2个/类的车辆、行人、自行车目标 |
| **多点散射模型** | 每类目标用多个散射点模拟雷达回波，体现目标几何形状 |
| **微多普勒效应** | 行人手臂/腿部摆动、车轮旋转产生微多普勒特征 |
| **轨迹运动规划** | 带随机加速/减速/转弯的目标运动轨迹 |
| **目标间遮挡** | 前方目标对后方目标的雷达信号遮挡检测 |
| **噪声与杂波** | 高斯噪声 + 静态杂波模拟真实环境 |
| **8天线阵列** | 均匀线阵（ULA）支持到达角（DOA）估计 |
| **多场景连续仿真** | 支持连续生成50个随机场景，每场景含多个测量帧 |
| **COCO数据转换** | 仿真结果转为深度学习标准数据集格式 |

---

## 2. 环境依赖与安装配置

### 2.1 MATLAB 环境

| 依赖项 | 版本要求 | 说明 |
|--------|---------|------|
| MATLAB | ≥ R2020a | 核心仿真平台 |
| Phased Array System Toolbox | — | 必需工具箱，提供雷达发射/接收/信道/天线模型 |

### 2.2 Python 环境

| 依赖项 | 安装命令 | 说明 |
|--------|---------|------|
| numpy | `pip install numpy` | 数值计算 |
| scipy | `pip install scipy` | .mat 文件读取 |
| opencv-python | `pip install opencv-python` | 图像处理与 JPG 写入 |
| json | 内置 | JSON 文件生成 |
| os | 内置 | 文件路径管理 |

> **注意**：项目实测使用 Miniconda3 环境，路径为 `/home/TangXuebiao/miniconda3/`，已在 `.bashrc` 中配置 conda 初始化。

### 2.3 目录结构说明

```
FMCW_Radar_Target_Simulator/
├── FMCWradar.m                # 雷达类定义（核心参数与信号处理）
├── SimulateTargetList.m       # 多目标场景仿真主入口
├── TargetSimulation.m         # 单目标仿真入口
├── Car.m                      # 车辆多点散射模型
├── Pedestrian.m               # 行人多点散射模型
├── Bicyclist.m                # 自行车多点散射模型
├── TrajectoryPlanner.m        # 目标运动轨迹规划器
├── generateObstructionMap.m   # 目标间遮挡地图生成
├── modelBasebandSignal.m      # 单目标基带信号计算（优化版）
├── modelSignal.m              # 多目标基带信号计算（原始版）
├── simulateSignal.m           # 合成点目标信号仿真
├── saveMat.m                  # 仿真结果保存函数
├── JSONCoco.py                # .mat → COCO JSON + 灰度JPG
├── JSONCoco_3SeqRGB.py        # 3帧序列叠加为RGB图像的COCO转换
├── SimulationData/            # 仿真输出 .mat 文件目录
│   ├── Szenario1/
│   ├── Szenario2/
│   └── ...
└── COCO/                      # Python转换后的COCO数据集目录
    ├── annotations/
    └── images/
```

---

## 3. 输入参数定义

### 3.1 雷达参数（`FMCWradar.m` 类属性）

| 参数名 | 类型 | 默认值 | 取值范围 | 说明 |
|--------|------|--------|---------|------|
| `chirpShape` | 字符串 | `'SAWgap'` | `'TRI'`, `'SAW1'`, `'SAWgap'` | 调频波形：三角波、锯齿波、带间隙锯齿波 |
| `sweepBw` | 浮点数 | `1e9` | — | 扫频带宽 (Hz)，决定距离分辨率 |
| `chirpTime` | 浮点数 | `32e-6` | — | 单个chirp持续时间 (s) |
| `fs` | 浮点数 | `10e6` | — | ADC 采样频率 (Hz) |
| `f0` | 浮点数 | `76.5e9` | — | 雷达载频 (Hz)，77GHz频段 |
| `chirpsCycle` | 整数 | `256` | — | 每个测量周期的 chirp 数量（多普勒维长度） |
| `height` | 浮点数 | `0.5` | — | 雷达安装高度 (m) |
| `egoMotion` | 布尔/浮点数 | `0` | `false`, `true`, 数值 | 自车运动状态；`true` 时随机生成速度 |
| `TXpeakPower` | 浮点数 | `0.01` | — | 发射峰值功率 (W)，约 10dBm |
| `TXgain` | 浮点数 | `17` | — | 发射天线增益 (dB) |
| `RXgain` | 浮点数 | `15` | — | 接收天线增益 (dB) |
| `RXNF` | 浮点数 | `10` | — | 接收机噪声系数 (dB) |
| `RXant` | 整数 | `8` | — | 接收天线数（均匀线阵） |
| `NoiseFloor` | 浮点数 | `-130` | — | 噪声基底 (dB) |
| `dynamicNoise` | 浮点数 | `10` | — | 噪声动态范围 (±dB) |
| `backscatterStatClutter` | 布尔 | `false` | `true`/`false` | 是否启用后向散射静态杂波 |
| `numStatTargets` | 浮点数 | `60` | — | 静态杂波目标数的瑞利均值 |
| `dBoffset` | 浮点数 | `30` | — | RD 图谱显示偏移 (dB) |
| `printNoiseCharacteristics` | 布尔 | `false` | `true`/`false` | 是否打印噪声特性信息 |

### 3.2 仿真控制参数（`SimulateTargetList.m`）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `Szenarios` | 整数 | `50` | 生成的场景总数 |
| `duration` | 浮点数 | `0.5` | 每个场景持续时间 (秒) |
| `plotAntennas` | 整数数组 | `[]` | 要绘图的天线索引；`[]`=不绘图，`0`=合并图，`1:8`=各天线 |
| `add_files` | 布尔 | `false` | 是否追加到已有文件 |
| `file_offset` | 整数 | `0` | 文件编号偏移量 |
| `SimDataPath` | 字符串 | `'SimulationData/'` | 仿真输出路径 |

**目标随机生成规则**：每类目标数量为 `floor(2.3*rand())`，即 0 到 2 个。每场景至少有一个目标（`checksum > 0`）。

### 3.3 目标模型参数

#### 3.3.1 车辆参数（`Car.m`）

| 参数 | 类型 | 默认值（typeNr=0 / 1） | 说明 |
|------|------|----------------------|------|
| `typeNr` | 整数 | 0 / 1 | 车型：0=标准轿车，1=SUV/货车 |
| `width` | 浮点数 | 1.8 / 2.01 | 车宽 (m) |
| `length` | 浮点数 | 4.5 / 5.8 | 车长 (m) |
| `Height` | 浮点数 | 1.5 / 2.1 | 车高 (m) |
| `heightAxis` | 浮点数 | 0.3 / 0.375 | 车轴离地高度 (m) |
| `cornerRadius` | 浮点数 | 0.8 / 0.7 | 轮廓圆角半径 (m) |
| `rTire` | 浮点数 | 0.3 / 0.375 | 轮胎半径 (m) |
| `ReceptionAngle` | 浮点数 | 160 | 散射点接收角度范围 (度) |
| `ReflectionsPerContourPoint` | 整数 | 1 | 每个轮廓点的散射采样数 |
| `WheelReflectionsFactor` | 整数 | 4 | 车轮散射点倍增系数 |

#### 3.3.2 行人参数（`Pedestrian.m`）

| 参数 | 类型 | 默认值/范围 | 说明 |
|------|------|-------------|------|
| `Height` | 浮点数 | 1.0~2.3 m | 身高，随机 `1.3+rand()` |
| `WalkingSpeed` | 浮点数 | ~1.0~1.7 m/s | 步行速度，随机 `1+0.7*rand()` |
| `width` | 浮点数 | 自动计算 | 身宽 = `Height/(2.5+rand()*1.3)` |
| `length` | 浮点数 | 0.25~0.5 | 身长 (m)，随机 |
| `ReceptionAngle` | 浮点数 | 180 | 散射点接收角度范围 (度) |
| `RCSsigma` | 浮点数 | -6 dBsm | 总雷达散射截面 |

**12个身体散射点**：脚（FR/FL）、膝（KR/KL）、髋（HR/HL）、肘（ER/EL）、肩（SR/SL）、躯干（C）、头（H）。各部位分配不同 RCS（雷达散射截面）权重，模拟行走时的肢体摆动微多普勒。

#### 3.3.3 自行车参数（`Bicyclist.m`）

| 参数 | 类型 | 默认值（typeNr=0 / 1） | 说明 |
|------|------|----------------------|------|
| `typeNr` | 整数 | 0 / 1 | 车型：0=26寸，1=29寸 |
| `width` | 浮点数 | 0.78 / 0.8 | 车把宽 (m) |
| `length` | 浮点数 | 1.70 / 1.80 | 车长 (m) |
| `frameHeight` | 浮点数 | 0.55 / 0.65 | 车架高 (m) |
| `rTire` | 浮点数 | 0.35 / 0.39 | 轮胎半径 (m) |
| `ReceptionAngle` | 浮点数 | 150 | 散射点接收角度范围 (度) |

### 3.4 运动轨迹参数（`TrajectoryPlanner.m`）

| 参数 | 说明 |
|------|------|
| `tstep` | 时间步长 = `chirpsCycle * chirpInterval`（约 0.016 秒） |
| `eventduration` | 加/减速事件持续时间：车辆4s、自行车4s、行人2s |
| 最大速度 | 车辆 `fmcw.velBins(end)` ≈ 15 m/s、自行车 10 m/s、行人 1.4×身高 |
| 转弯角度 | 车辆 ±45°、自行车 ±40°、行人 ±30° |

---

## 4. 执行运行命令与操作流程

### 4.1 多目标场景仿真（主入口）

**运行命令**：
```bash
cd /home/TangXuebiao/Matlab/FMCW_Radar_Target_Simulator
matlab -nodisplay -r "SimulateTargetList; exit"
```

**操作流程**：
1. 打开 `SimulateTargetList.m`
2. 调整 `Szenarios`（场景数）和 `duration`（每场景时长）
3. 运行脚本，自动完成以下步骤：
   - 初始化 FMCW 雷达对象
   - 循环生成 `Szenarios` 个随机场景
   - 每个场景随机选择 0-2 个车辆/行人/自行车
   - 为每个目标规划运动轨迹
   - 对每个时间步：更新目标位置 → 遮挡检测 → 信号仿真 → 加噪声/杂波 → RD 图谱计算 → 保存
4. 输出文件保存在 `SimulationData/Szenario{N}/` 目录

### 4.2 单目标仿真

**运行命令**：
```bash
# 单人
matlab -nodisplay -r "TargetSimulation; exit"

# 或指定目标类型（需修改文件内的变量）
matlab -nodisplay -r "TargetSimulation('Pedestrian'); exit"
```

**操作流程**：
1. 打开 `TargetSimulation.m`
2. 设置 `Pedestrians`、`Bicycles`、`Cars` 变量为目标数量
3. 运行脚本，生成对应数量的独立目标仿真数据

### 4.3 转换为 COCO 格式

**灰度 JPG 格式**：
```bash
cd /home/TangXuebiao/Matlab/FMCW_Radar_Target_Simulator
python JSONCoco.py
```
- 读取 `SimulationData/Szenario{N}/` 下所有 .mat 文件
- 生成 256×160px 灰度 RD 图谱 JPG
- 输出 COCO 格式 JSON 标注文件

**RGB 三帧叠加格式**：
```bash
cd /home/TangXuebiao/Matlab/FMCW_Radar_Target_Simulator
python JSONCoco_3SeqRGB.py
```
- 将连续 3 帧 RD 图谱叠加为 RGB 三通道图像
- 利用时序信息增强目标检测

**Python 转换关键配置**：
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `uniformBoxes` | `True` | 使用统一尺寸的边界框（按类别设置固定多普勒展宽） |
| `writeJPGs` | `True` | 是否输出 JPG 图像 |
| `drawBoxes` | `False` | 是否绘制标注框调试图 |
| `SimDataPath` | `'./SimulationData0910_Train/'` | 仿真数据路径（**使用前需修改为实际路径**） |
| `trainvalname` | `'train2017'` | 数据集名称，影响输出目录 |

### 4.4 训练深度学习模型

```bash
cd /home/TangXuebiao
python train_efficientdet.py --epochs 10
```

预训练权重：
- `efficientdet_d0_epoch5.pth` — 5 轮训练
- `efficientdet_d0_epoch10.pth` — 10 轮训练

---

## 5. 输出结果解析

### 5.1 文件格式

仿真输出为 `.mat` 文件，每帧两个文件：

| 文件 | 命名规则 | 说明 |
|------|---------|------|
| RD 图谱 | `Szenario{N}_{frame}.mat` | 距离-多普勒图谱（3D 数据立方） |
| 标签 | `Szenario{N}_Label_{frame}.mat` | 目标标注信息 |

### 5.2 RD 图谱数据格式

**变量名**：`RD`

**维度**：`[160, 256, 16]`

| 维度 | 大小 | 说明 |
|------|------|------|
| 第 1 维 | 160 | 距离门（Range bins），有效范围 ≈ 0~77 m |
| 第 2 维 | 256 | 速度门（Doppler/Velocity bins），范围 ≈ ±58 km/h |
| 第 3 维 | 16 | 天线通道（8 RX × 2 虚拟阵元），支持到达角估计 |

**数值**：RD 图谱功率值，以 dB 为单位（对数刻度）。

**分辨率**：
- **距离分辨率** `dR` = c₀ / (2 × sweepBw) ≈ 0.15 m
- **速度分辨率** `dV` = 1/(L × chirpInterval) × c₀/(2 × f₀) ≈ 0.127 m/s

### 5.3 标签格式

**变量名**：`label`

**多目标格式**：cell 数组 `{TargetID; [数值向量]}`

```
TargetID: 字符串，如 'Pedestrian1', 'Bicycle0', 'Vehicle0'

数值向量 [10个元素]:
索引  字段             类型      说明
─────────────────────────────────────────────
1     targetR          浮点数    径向距离 (m)，目标到雷达的直线距离
2     targetV          浮点数    径向速度 (m/s)，>0 远离雷达，<0 靠近雷达
3     azi              浮点数    方位角 (度)，目标相对于雷达视线的角度
4     egoMotion        浮点数    自车运动速度 (m/s)
5     xPos             浮点数    目标 x 坐标 (m)，雷达视线方向
6     yPos             浮点数    目标 y 坐标 (m)，雷达视线左侧90°
7     width            浮点数    目标宽度 (m)
8     length           浮点数    目标长度 (m)
9     heading          浮点数    目标朝向角 (度)，相对于 x 轴
10    obstruction      整数      遮挡等级：0=可见，1=部分遮挡，2=过半遮挡，
                                 3=严重遮挡(>75%)，4=完全遮挡
```

**单目标格式**：直接为数值向量 `[targetR, targetV, azi, egoMotion, xPos, yPos, width, length, heading]`（无第10项遮挡信息，即 `TargetSimulation.m` 输出）。

### 5.4 COCO 格式输出

Python 转换脚本生成的 COCO 数据集结构：

```
COCO/
├── annotations/
│   └── instances_train2017.json   # COCO 格式标注文件
└── images/
    └── train2017/
        ├── 1.jpg                  # RD 图谱图像 (256×160px)
        ├── 2.jpg
        └── ...
```

**类别映射**：

| category_id | 名称 | 说明 |
|-------------|------|------|
| 0 | Vehicle | 车辆 |
| 1 | Pedestrian | 行人 |
| 2 | Bicycle | 自行车 |

**COCO 标注格式**：
```json
{
  "segmentation": [[x1,y1,x2,y2,...]],
  "iscrowd": 0,
  "image_id": 1,
  "category_id": 0,
  "id": 1,
  "bbox": [x, y, width, height],
  "area": 480
}
```

边界框在 RD 图谱坐标系中，`bbox = [x, y, width, height]`，其中：
- x, y 为左上角坐标
- width 为多普勒维宽度（对应速度展宽）
- height 为距离维高度（对应距离展宽）

---

## 6. 常见错误排查与注意事项

### 6.1 常见错误

| 错误现象 | 可能原因 | 解决方法 |
|---------|---------|---------|
| `Undefined function 'phased.FMCWWaveform'` | 缺少 Phased Array System Toolbox | `ver` 检查工具箱安装；重新安装 toolbox |
| 运行时间长 | 场景数过多或 `duration` 过长 | 减小 `Szenarios`（如设为 5），或改用 `parfor`（需 Parallel Computing Toolbox） |
| 噪声淹没问题 | `NoiseFloor` 设置过高 | 降低 `NoiseFloor`（如 -135dB），或增大 `TXpeakPower` |
| `Array formation` 相关错误 | MATLAB 版本差异 | 确保 MATLAB ≥ R2020a |
| Python 找不到 .mat 文件 | `SimDataPath` 路径不匹配 | 修改 `JSONCoco.py` 中 `SimDataPath` 为实际路径 |
| `cv2` 模块导入失败 | 未安装 opencv-python | `pip install opencv-python` |
| `scipy.io.loadmat` 报错 | .mat 文件版本不适配 | 确保 .mat 文件为 v7.3 格式，或用 `savemat` 保存 |

### 6.2 关键注意事项

1. **路径修改**：`JSONCoco.py` 中 `SimDataPath` 默认值为 `'./SimulationData0910_Train/'`，需根据实际仿真输出路径修改。

2. **数据清理**：`SimulateTargetList.m` 中 `add_files == false` 时会清空 `SimulationData/` 目录，如需保留数据请先备份。

3. **MATLAB 工作目录**：运行仿真前确保 MATLAB 当前工作目录为 `FMCW_Radar_Target_Simulator/`，否则 `SimulationData/` 路径会不正确。

4. **随机性控制**：若需复现结果，取消 `TargetSimulation.m` 中 `rng('default')` 的注释，或使用 `rng(seed)` 设置随机种子。

5. **内存消耗**：仿真的 RD 图谱 `[160×256×16]` 占内存较小，但 `modelSignal.m`（原始版）中 `xRX` 的临时变量 `[fmcw.chirpInterval × Propagation_fs × chirpsCycle × RXant]` 会占用大量内存。推荐使用 `modelBasebandSignal.m`（优化版，直接计算基带信号）。

6. **`modelSignal` vs `modelBasebandSignal`**：
   - `modelBasebandSignal.m`（新）：直接计算差频信号，无需求解完整传播路径，速度快。
   - `modelSignal.m`（旧）：使用 `phased.FreeSpace` 信道和 `dechirp` 混合，更精确但速度慢。
   - 两版代码在 `SimulateTargetList.m` 中调用的是 `modelBasebandSignal`。

7. **天线方向图**：`FMCWradar.m` 中的天线方向图数据（`Vpattern` / `Hpattern`）为近似值，如需精确仿真应参考具体雷达芯片的 datasheet。

8. **COCO 转换前的路径校对**：运行 `JSONCoco.py` 前务必检查 `SimDataPath` 是否指向正确的仿真输出目录，否则脚本可能因路径不存在而崩溃。
