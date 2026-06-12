# 车载毫米波雷达仿真与深度学习目标检测 — 完整使用手册

## 项目概述

本项目基于 Thomas Wengerter 等人的 IEEE 论文 *"Simulation of Urban Automotive Radar Measurements for Deep Learning Target Detection"*，实现了一个**车载 FMCW（调频连续波）雷达仿真系统**，模拟城市交通场景下的雷达回波信号，生成包含车辆（Vehicle）、行人（Pedestrian）和自行车（Bicycle）三类目标的距离-多普勒（RD）图谱，并支持将仿真数据转换为 COCO 格式，用于训练深度学习目标检测模型（如 EfficientDet-D0）。

---

## 目录

1. [核心工作流程](#1-核心工作流程)
2. [环境依赖与安装](#2-环境依赖与安装)
3. [文件目录结构](#3-文件目录结构)
4. [完整使用步骤](#4-完整使用步骤)
5. [核心参数速查表](#5-核心参数速查表)
6. [输出数据格式详解](#6-输出数据格式详解)
7. [训练与推理](#7-训练与推理)
8. [常见问题排查](#8-常见问题排查)

---

## 1. 核心工作流程

```
┌─────────────────────────────────────────────────────────────────────┐
│                      MATLAB 仿真层                                   │
│                                                                     │
│  Step 1: FMCWradar.m —— 配置雷达参数（载频、带宽、chirp数等）         │
│      ↓                                                              │
│  Step 2: SimulateTargetList.m —— 随机生成多目标场景（主入口）         │
│      ├── 随机选择 0-2 个/类的目标（Pedestrian/Bicycle/Car）           │
│      ├── 调用 Pedestrian.m / Bicyclist.m / Car.m 初始化目标模型       │
│      ├── 调用 TrajectoryPlanner.m 规划目标的随机运动轨迹               │
│      └── 每时间步：                                                   │
│          ├── 更新目标位置 → generateObstructionMap.m（遮挡检测）      │
│          ├── modelBasebandSignal.m（高效版）/ modelSignal.m（精确版）  │
│          ├── addGaussNoise + addStaticClutter（噪声与杂波）            │
│          └── RDmap() 计算 RD 图谱 → saveMat.m 保存 .mat 文件          │
│              ↓                                                       │
│  输出目录: SimulationData/Szenario{N}/                                │
│      ├── Szenario{N}_{frame}.mat        (RD 图谱数据立方)             │
│      └── Szenario{N}_Label_{frame}.mat  (目标标注信息)                │
│              ↓                                                       │
│                      Python 数据转换层                                │
│                                                                     │
│  Step 3: JSONCoco.py 或 JSONCoco_3SeqRGB.py                         │
│      ├── 读取所有 .mat 仿真文件                                       │
│      ├── 计算目标在 RD 图谱上的边界框                                  │
│      ├── 生成 JPG 图像（256×160px RD 图谱）                          │
│      └── 输出 COCO 格式 JSON 标注文件                                 │
│              ↓                                                       │
│  输出目录: COCO/                                                     │
│      ├── annotations/instances_train2017.json                       │
│      └── images/train2017/{id}.jpg                                   │
│              ↓                                                       │
│                     深度学习层                                        │
│                                                                     │
│  Step 4: train_efficientdet.py —— 训练 EfficientDet-D0 模型          │
│  Step 5: detect_radar.py —— 推理检测                                │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 2. 环境依赖与安装

### 2.1 MATLAB 环境

| 依赖项 | 版本要求 | 说明 |
|--------|---------|------|
| MATLAB | ≥ R2020a | 核心仿真平台 |
| Phased Array System Toolbox | — | 提供雷达发射/接收/信道/天线模型（`phased.FMCWWaveform`、`phased.FreeSpace`、`phased.RadarTarget`、`phased.ULA` 等） |
| Parallel Computing Toolbox | 可选 | 若需使用 `parfor` 并行加速运算 |

### 2.2 Python 环境

| 依赖项 | 安装命令 | 说明 |
|--------|---------|------|
| numpy | `pip install numpy` | 数值计算 |
| scipy | `pip install scipy` | .mat 文件读取 |
| opencv-python | `pip install opencv-python` | 图像处理与 JPG 写入 |
| torch | `pip install torch` | PyTorch 深度学习框架 |
| effdet | `pip install effdet` | EfficientDet 模型库 |
| timm | `pip install timm` | PyTorch 图像模型库（effdet 依赖） |
| pycocotools | `pip install pycocotools` | COCO 评估工具 |

> **注意**：本项目实测使用 Miniconda3 环境，路径为 `/home/TangXuebiao/miniconda3/`，已在 `.bashrc` 中配置 conda 初始化。

---

## 3. 文件目录结构

```
FMCW_Radar_Target_Simulator/
│
├── 【核心控制】── 仿真入口与配置
│   ├── SimulateTargetList.m     # 多目标场景仿真主入口（随机生成多个目标）
│   ├── TargetSimulation.m       # 单目标类型仿真入口（单一类别批量生成）
│   └── FMCWradar.m              # 雷达类定义（核心参数与信号处理）
│
├── 【目标模型】── 多点散射模型
│   ├── Car.m                    # 车辆散射模型（轮廓+车轮微多普勒）
│   ├── Pedestrian.m             # 行人散射模型（12个身体散射点+行走微多普勒）
│   └── Bicyclist.m              # 自行车散射模型（框架+旋转车轮）
│
├── 【运动规划】── 轨迹与遮挡
│   ├── TrajectoryPlanner.m      # 目标运动轨迹规划（随机加速/减速/转弯）
│   └── generateObstructionMap.m # 目标间遮挡地图生成
│
├── 【信号仿真】── 基带信号计算
│   ├── modelBasebandSignal.m    # 高效版基带信号计算（直接计算差频信号，推荐）
│   ├── modelSignal.m            # 原始版基带信号计算（使用 phased.FreeSpace，更精确但慢）
│   └── simulateSignal.m         # 合成点目标信号仿真（简单点目标模型）
│
├── 【数据输出】── 保存与后处理
│   ├── saveMat.m                # 仿真结果保存函数
│   ├── JSONCoco.py              # .mat → COCO JSON + 灰度 JPG
│   └── JSONCoco_3SeqRGB.py      # 3帧序列叠加为RGB图像的COCO转换
│
├── 【深度学习】── 训练与推理
│   ├── train_efficientdet.py    # EfficientDet-D0 训练脚本
│   └── detect_radar.py          # EfficientDet-D0 推理检测脚本
│
├── 【文档】
│   ├── README_cc.md             # 本文件（中文使用手册）
│   └── doc/                     # 论文笔记等文档
│
├── 【仿真数据目录】（已被 .gitignore 忽略）
│   ├── SimulationData/          # MATLAB 仿真输出的 .mat 文件
│   ├── COCO/                    # Python 转换后的 COCO 数据集
│   ├── checkpoints/             # 模型权重检查点
│   ├── logs/                    # TensorBoard 训练日志
│   └── results/                 # 检测结果图像
│
└── 【配置文件】
    └── .gitignore               # Git 忽略规则
```

---

## 4. 完整使用步骤

### 4.1 第一步：配置雷达参数

打开 `FMCWradar.m`，在 `properties` 段修改雷达核心参数（详细参数见第 5 节）。

关键检查项：
- `sweepBw`（扫频带宽）决定距离分辨率
- `chirpsCycle`（chirp 数）决定多普勒分辨率
- `f0`（载频）通常为 76.5~77 GHz
- `NoiseFloor`（噪声基底）影响信噪比，需根据实测数据调整

### 4.2 第二步：配置仿真场景参数

打开 `SimulateTargetList.m`，在文件头部修改：

```matlab
% 仿真场景数（每场景包含多个时间帧）
Szenarios = 50;    % 场景总数，建议范围 5~100

% 每场景持续时间（秒）
duration = 0.5;    % 每场景时长，1 帧 = 256×64µs ≈ 0.0164s

% 是否绘图（空数组 = 不绘图，0 = 合并图，1:8 = 各天线单独图）
plotAntennas = [];

% 仿真数据保存路径
SimDataPath = 'SimulationData/';
```

每场景中目标数量的生成规则（`SimulateTargetList.m` 第 82-84 行）：
```matlab
Pedestrians = floor(2.3*rand());  % 0~2 个行人
Bicycles    = floor(2.3*rand());  % 0~2 辆自行车
Cars        = floor(2.3*rand());  % 0~2 辆车
```
当 `Pedestrians + Bicycles + Cars == 0` 时会重新生成，确保每场景至少有一个目标。

### 4.3 第三步：运行 MATLAB 仿真

```bash
cd /home/TangXuebiao/Matlab/FMCW_Radar_Target_Simulator
matlab -nodisplay -r "SimulateTargetList; exit"
```

**仿真时间与性能**：
- 50 场景 × 0.5s/场景 ≈ 30 帧，使用 `parfor`（需 Parallel Computing Toolbox）约 2 分钟
- 当前版本已改为 `for`（串行），时间会延长
- 如需加速：减小 `Szenarios` 和 `duration`，或安装 Parallel Computing Toolbox 改回 `parfor`

仿真完成后，输出文件保存在 `SimulationData/Szenario{N}/` 目录下。

### 4.4 第四步：转换为 COCO 格式

**灰度 JPG 格式（单帧）**：
```bash
cd /home/TangXuebiao/Matlab/FMCW_Radar_Target_Simulator
python JSONCoco.py
```

**RGB 三帧叠加格式（3帧合成RGB三通道）**：
```bash
python JSONCoco_3SeqRGB.py
```

**转换前必须修改**：在 `JSONCoco.py` 或 `JSONCoco_3SeqRGB.py` 中设置正确的 `SimDataPath` 变量，使其指向实际的仿真输出目录。

转换完成后，数据输出在 `COCO/` 目录：
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

### 4.5 第五步：训练深度学习模型

```bash
cd /home/TangXuebiao
python FMCW_Radar_Target_Simulator/train_efficientdet.py \
    --epochs 50 \
    --batch-size 4 \
    --lr 1e-4
```

关键命令行参数：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--epochs` | 训练轮数 | 50 |
| `--batch-size` | 批处理大小 | 4 |
| `--lr` | 初始学习率 | 1e-4 |
| `--annotations` | COCO 标注 JSON 路径 | `./COCO/annotations/instances_train2017.json` |
| `--image-dir` | 图像目录 | `./COCO/images/train2017` |
| `--save-dir` | 模型保存目录 | `./checkpoints` |
| `--log-dir` | TensorBoard 日志目录 | `./logs` |
| `--no-pretrain` | 不使用 COCO 预训练权重 | （默认使用预训练） |
| `--resume` | 恢复训练的检查点路径 | None |
| `--val-split` | 从训练集划分验证集比例 | 0.0 |

训练过程会：
- 每 5 个 epoch 保存一次检查点（`efficientdet_d0_epoch{N}.pth`）
- 保存最佳模型（`efficientdet_d0_best.pth`）
- 记录 TensorBoard 日志到 `logs/` 目录

### 4.6 第六步：推理检测

**单张图片推理**：
```bash
python FMCW_Radar_Target_Simulator/detect_radar.py \
    --weights checkpoints/efficientdet_d0_best.pth \
    --image COCO/images/train2017/1.jpg \
    --output ./results/
```

**批量推理**：
```bash
python FMCW_Radar_Target_Simulator/detect_radar.py \
    --weights checkpoints/efficientdet_d0_epoch50.pth \
    --image-dir ./test_images/ \
    --output ./results/
```

推理结果包括：
- 标注了检测框的图像（保存至 `results/` 目录）
- 检测结果的 JSON 文件（`results/detections.json`）
- 物理坐标输出（距离、速度、范围展宽等）

---

## 5. 核心参数速查表

### 5.1 雷达参数（`FMCWradar.m` — properties）

| 参数名 | 功能说明 | 默认值 | 可调范围 | 影响 |
|--------|---------|--------|---------|------|
| `chirpShape` | 调频波形选择 | `'SAWgap'` | `'TRI'`（三角波）, `'SAW1'`（锯齿波）, `'SAWgap'`（带间隙锯齿波） | chirp 间隔计算方式不同 |
| `sweepBw` | 扫频带宽 (Hz) | `1e9` (1 GHz) | 0.5~4 GHz | 决定距离分辨率 `dR = c/(2×Bw)` |
| `chirpTime` | 单个 chirp 持续时间 (s) | `32e-6` (32 µs) | 16~64 µs | 影响最大不模糊距离 |
| `fs` | ADC 采样频率 (Hz) | `10e6` (10 MHz) | 5~20 MHz | 决定采样点数 `K = chirpTime×fs` |
| `f0` | 雷达载频 (Hz) | `76.5e9` (76.5 GHz) | 76~77 GHz（车载雷达频段） | 影响多普勒分辨率和天线间距 |
| `chirpsCycle` | 每测量周期 chirp 数 | `256` | 64~512 | 决定多普勒分辨率 `dV = 1/(L×Tci)×c/(2×f0)` |
| `height` | 雷达安装高度 (m) | `0.5` | 0.3~1.0 m | 影响车轮遮挡因子和目标俯仰角 |
| `egoMotion` | 自车运动状态 | `false` (0) | `false`, `true`, 数值 | `true` 时随机生成速度 (0~max) |
| `TXpeakPower` | 发射峰值功率 (W) | `0.01` (10 dBm) | 0.001~0.1 W | 影响信噪比 |
| `TXgain` | 发射天线增益 (dB) | `17` | 10~20 dB | 影响接收信号功率 |
| `RXgain` | 接收天线增益 (dB) | `15` | 10~20 dB | 影响接收信号功率 |
| `RXNF` | 接收机噪声系数 (dB) | `10` | 5~15 dB | 影响噪声基底 |
| `RXant` | 接收天线数（ULA 均匀线阵） | `8` | 4~16（2 的幂） | 决定角度维 FFT 长度 |
| `NoiseFloor` | 噪声基底 (dB) | `-130` | -140~-120 dB | 噪声功率水平，过高会淹没问题 |
| `dynamicNoise` | 噪声动态范围 (±dB) | `10` | 5~20 dB | 每帧噪声水平随机变化范围 |
| `backscatterStatClutter` | 是否启用后向散射静态杂波 | `false` | `true`/`false` | 推荐 `false`（用 `addStaticClutter` 替代） |
| `numStatTargets` | 静态杂波目标数的瑞利均值 | `60` | 20~100 | 静态杂波点数量 |
| `dBoffset` | RD 图谱显示偏移 (dB) | `30` | 20~60 dB | 仅影响显示，不影响数据 |
| `printNoiseCharacteristics` | 是否打印噪声特性 | `false` | `true`/`false` | 调试用，打印 SNR 和噪声水平 |

### 5.2 仿真控制参数（`SimulateTargetList.m` — 文件头部）

| 参数名 | 功能说明 | 默认值 | 可调范围 | 位置 |
|--------|---------|--------|---------|------|
| `Szenarios` | 生成的场景总数 | `50` | 5~200 | 第 44 行 |
| `duration` | 每场景持续时间 (s) | `0.5` | 0.1~2.0 s | 第 47 行 |
| `plotAntennas` | 绘图天线索引 | `[]` | `[]`（不绘图）, `0`（合并图）, `1:8`（各天线） | 第 43 行 |
| `SimDataPath` | 仿真输出路径 | `'SimulationData/'` | 任意有效路径 | 第 50 行 |
| `add_files` | 是否追加到已有文件 | `false` | `true`/`false` | 第 51 行 |
| `file_offset` | 文件编号偏移量 | `0` | ≥0 | 第 60 行 |

### 5.3 目标随机生成概率（`SimulateTargetList.m` — 第 82-84 行）

| 代码 | 目标类型 | 生成规则 | 每场景数量 |
|------|---------|---------|-----------|
| `floor(2.3*rand())` | Pedestrian | 均匀随机取整 | 0~2 个 |
| `floor(2.3*rand())` | Bicycle | 均匀随机取整 | 0~2 个 |
| `floor(2.3*rand())` | Car | 均匀随机取整 | 0~2 个 |

总计为 0 时会重新生成，确保每场景 ≥1 个目标。

### 5.4 车辆模型参数（`Car.m` — initCar 方法）

| 参数名 | 功能说明 | 默认值 (typeNr=0) | 默认值 (typeNr=1) |
|--------|---------|------------------|------------------|
| `typeNr` | 车型 | 0（标准轿车） | 1（SUV/货车） |
| `width` | 车宽 (m) | 1.8 | 2.01 |
| `length` | 车长 (m) | 4.5 | 5.8 |
| `Height` | 车高 (m) | 1.5 | 2.1 |
| `heightAxis` | 车轴离地高度 (m) | 0.3 | 0.375 |
| `cornerRadius` | 轮廓圆角半径 (m) | 0.8 | 0.7 |
| `rTire` | 轮胎半径 (m) | 0.3 | 0.375 |
| `ReceptionAngle` | 散射点接收角度范围 (度) | 160 | 160 |
| `ReflectionsPerContourPoint` | 每轮廓点散射采样数 | 1 | 1 |
| `WheelReflectionsFactor` | 车轮散射点倍增系数 | 4 | 4 |

### 5.5 行人模型参数（`Pedestrian.m` — 属性与 initPedestrian 方法）

| 参数名 | 功能说明 | 默认值/范围 | 说明 |
|--------|---------|-------------|------|
| `Height` | 身高 (m) | `1.3+rand()` | 范围 1.0~2.3 m |
| `WalkingSpeed` | 步行速度 (m/s) | `1+0.7*rand()` | 范围 1.0~1.7 m/s |
| `width` | 身宽 (m) | `Height/(2.5+rand()*1.3)` | 自动根据身高计算 |
| `length` | 身长 (m) | `0.25+rand()*0.25` | 范围 0.25~0.5 m |
| `ReceptionAngle` | 散射点接收角度范围 (度) | 180 | 180 度（全向） |
| `RCSsigma` | 平均总 RCS (dBsm) | `-6` | -6 dBsm |
| `StepLength` | 步长 (m) | `Height×0.3871` | 自动根据身高计算 |
| `StepDuration` | 步幅时间 (s) | `StepLength/WalkingSpeed` | 自动计算 |

**12 个身体散射点**：脚(FR/FL)、膝(KR/KL)、髋(HR/HL)、肘(ER/EL)、肩(SR/SL)、躯干(C)、头(H)。各部位 RCS 权重详见 `Pedestrian.m` 第 144-151 行。

### 5.6 自行车模型参数（`Bicyclist.m` — initBicycle 方法）

| 参数名 | 功能说明 | 默认值 (typeNr=0) | 默认值 (typeNr=1) |
|--------|---------|------------------|------------------|
| `typeNr` | 车型 | 0（26寸） | 1（29寸） |
| `width` | 车把宽 (m) | 0.78 | 0.8 |
| `length` | 车长 (m) | 1.70 | 1.80 |
| `frameHeight` | 车架高 (m) | 0.55 | 0.65 |
| `rTire` | 轮胎半径 (m) | 0.35 | 0.39 |
| `Height` | 骑车人身高 (m) | `1+rand()*0.3` | `1+rand()*0.6` |
| `ReceptionAngle` | 散射点接收角度范围 (度) | 150 | 150 |

### 5.7 运动轨迹参数（`TrajectoryPlanner.m` — init_TrajectoryPlanner 方法）

| 参数/规则 | 车辆 | 自行车 | 行人 |
|-----------|------|--------|------|
| 最大速度 (m/s) | `fmcw.velBins(end)` ≈ 15 | 10 | `1.4×Height` |
| 加/减速事件时长 (s) | 4 | 4 | 2 |
| 转弯角度范围 | ±45° | ±40° | ±30° |
| 速度变化逻辑 | 高速→减速，低速→加速，中速→随机 | 同车辆 | 高速→减速，低速→加速 |

### 5.8 噪声与杂波参数（`FMCWradar.m` — addGaussNoise / addStaticClutter 方法）

| 参数 | 功能说明 | 默认值 | 说明 |
|------|---------|--------|------|
| `NoiseFloor` | 噪声基底 (dB) | -130 | 噪声功率基准水平 |
| `dynamicNoise` | 动态噪声范围 (±dB) | 10 | 每帧在此范围内随机偏移 |
| `FFToffset` | FFT 处理偏移 (dB) | 10 | 固定偏移补偿 FFT 处理增益（`addGaussNoise` 内硬编码） |
| `AmpMargin` | 静态杂波幅度方差 (dB) | 15 | 杂波幅度动态范围（`addStaticClutter` 内硬编码） |

### 5.9 COCO 数据转换参数（`JSONCoco.py` / `JSONCoco_3SeqRGB.py` — 文件头部）

| 参数名 | 功能说明 | 默认值 | 可调范围 |
|--------|---------|--------|---------|
| `SimDataPath` | 仿真数据路径 | `'./SimulationData0910_Train/'` | 任意有效路径（**使用前必须修改**） |
| `trainvalname` | 数据集名称 | `'train2017'` | 任意字符串，影响输出目录名 |
| `uniformBoxes` | 使用统一尺寸边界框 | `True` | `True`/`False`；`True` 时按类别设置固定多普勒展宽 |
| `writeJPGs` | 是否输出 JPG 图像 | `True` | `True`/`False` |
| `drawBoxes` | 是否绘制标注框调试图 | `False` | `True`/`False`；调试用 |

`uniformBoxes` 为 `True` 时的多普勒展宽设置（`generate_annotation` 函数内）：

| 目标类别 | 多普勒展宽 (m/s) | 距离展宽 (m) |
|---------|-----------------|-------------|
| Pedestrian | ±1.2 | 1.2 |
| Bicycle | ±3 | 1.5 |
| Vehicle | ±3 | 2.5 |

### 5.10 训练脚本参数（`train_efficientdet.py` — 命令行参数 / get_default_config）

| 参数名 | 功能说明 | 默认值 | 可调范围 |
|--------|---------|--------|---------|
| `--epochs` | 训练轮数 | 50 | ≥1 |
| `--batch-size` | 批处理大小 | 4 | 1~64（受 GPU 显存限制） |
| `--lr` | 初始学习率 | 1e-4 | 1e-5~1e-3 |
| `lr_min` | 最小学习率（余弦退火） | 1e-6 | 1e-7~1e-5（配置字典内） |
| `weight_decay` | 权重衰减 | 1e-4 | 1e-5~1e-3 |
| `clip_grad` | 梯度裁剪阈值 | 10.0 | 1.0~20.0 |
| `save_interval` | 保存间隔（epoch） | 5 | 1~20 |
| `--no-pretrain` | 不使用 COCO 预训练 | `False` | 添加标志即不使用 |
| `pretrained` | 是否使用 COCO 预训练 | `True` | 配置字典内 |
| `image_size` | 输入图像尺寸 (px) | 256 | 可被 128 整除（BiFPN 要求） |
| `num_classes` | 类别数 | 3 | Vehicle/Pedestrian/Bicycle |

### 5.11 推理脚本参数（`detect_radar.py` — 命令行参数）

| 参数名 | 功能说明 | 默认值 | 可调范围 |
|--------|---------|--------|---------|
| `--weights` | 模型权重路径 | `./checkpoints/efficientdet_d0_best.pth` | 任意 .pth 文件 |
| `--image` | 单张图像路径 | `None` | JPG/PNG 文件路径 |
| `--image-dir` | 图像目录（批量推理） | `None` | 目录路径 |
| `--output` | 结果输出目录 | `./results` | 任意有效路径 |
| `--threshold` | 置信度阈值 | `0.3` | 0.1~0.9 |

---

## 6. 输出数据格式详解

### 6.1 RD 图谱格式（`RD` 变量）

- **变量名**：`RD`
- **维度**：`[160, 256, 16]`
  - 第 1 维（160）：距离门（Range bins），有效范围 ≈ 0~77 m
  - 第 2 维（256）：速度门（Doppler/Velocity bins），范围 ≈ ±58 km/h
  - 第 3 维（16）：天线通道（8 RX，补齐到 16）
- **数值**：RD 图谱功率值，以 dB 为单位（对数刻度）
- **生成方式**：2D FFT（距离维 Hamming 窗 → 多普勒维 Hamming 窗 → 角度维 Hamming 窗）

**分辨率计算公式**：
- **距离分辨率** `dR = c₀ / (2 × sweepBw)` = 299792458 / (2 × 1e9) ≈ **0.15 m**
- **速度分辨率** `dV = 1/(L × Tci) × c₀/(2 × f₀)` = 1/(256 × 64e-6) × 299792458/(2 × 76.5e9) ≈ **0.127 m/s**

### 6.2 标签格式（`label` 变量）

**多目标格式**（`SimulateTargetList.m` 输出）：cell 数组 `{TargetID; [数值向量]}`

```
TargetID: 字符串，如 'Pedestrian1', 'Bicycle0', 'Vehicle0'

数值向量 [10 个元素]:
索引  字段          类型      单位    说明
─────────────────────────────────────────────────
1     targetR       浮点数    m       径向距离，目标到雷达的直线距离
2     targetV       浮点数    m/s     径向速度，>0 远离雷达，<0 靠近雷达
3     azi           浮点数    度      方位角，目标相对于雷达视线的角度
4     egoMotion     浮点数    m/s     自车运动速度
5     xPos          浮点数    m       目标 x 坐标，雷达视线方向
6     yPos          浮点数    m       目标 y 坐标，雷达视线左侧 90°
7     width         浮点数    m       目标宽度
8     length        浮点数    m       目标长度
9     heading       浮点数    度      目标朝向角，相对于 x 轴
10    obstruction   整数      —       遮挡等级（见下表）
```

**遮挡等级**：

| obstruction 值 | 说明 |
|:---:|------|
| 0 | 可见，无遮挡 |
| 1 | 部分遮挡（部分散射点被遮挡） |
| 2 | 过半遮挡（>1/2 散射点被遮挡） |
| 3 | 严重遮挡（>3/4 散射点被遮挡） |
| 4 | 完全遮挡（全部散射点被遮挡） |

**单目标格式**（`TargetSimulation.m` 输出）：直接为数值向量 `[targetR, targetV, azi, egoMotion, xPos, yPos, width, length, heading]`（无第 10 项遮挡信息）。

### 6.3 COCO 标注格式

```json
{
  "segmentation": [[x1, y1, x2, y2, ...]],
  "iscrowd": 0,
  "image_id": 1,
  "category_id": 0,     // 0=Vehicle, 1=Pedestrian, 2=Bicycle
  "id": 1,
  "bbox": [x, y, width, height],   // 在 RD 图谱坐标系中
  "area": 480
}
```

**类别映射**：

| category_id | 名称 | 说明 |
|:---:|------|------|
| 0 | Vehicle | 车辆 |
| 1 | Pedestrian | 行人 |
| 2 | Bicycle | 自行车 |

---

## 7. 训练与推理

### 7.1 模型架构

- **骨干网络**：EfficientNet-B0
- **特征金字塔**：BiFPN（双向特征金字塔网络）
- **检测头**：分类 + 回归分支
- **输入尺寸**：256×256（RD 图谱从 256×160 填充至 256×256）
- **输出**：目标边界框、类别、置信度

### 7.2 训练流程详解

`train_efficientdet.py` 的训练流程：

1. **数据加载**：`RadarRDCOCODataset` 类加载 COCO 格式的 RD 图谱
   - 读取 JSON 标注文件
   - 图像从 256×160 填充至 256×256（BiFPN 要求 H/W 可被 128 整除）
   - 边界框坐标相应调整
2. **模型构建**：`build_efficientdet_d0()` 使用 `effdet` 库创建模型
   - 配置 `image_size = (256, 256)`
   - 配置 `num_classes = 3`
   - 可选加载 COCO 预训练权重
3. **训练循环**：`Trainer` 类管理
   - 优化器：AdamW（`lr=1e-4`, `weight_decay=1e-4`）
   - 调度器：余弦退火（`T_max=epochs`, `eta_min=1e-6`）
   - 梯度裁剪：阈值 10.0
   - TensorBoard 日志记录
4. **检查点保存**：每 5 个 epoch 保存一次，同时保存最佳模型

### 7.3 推理流程详解

`detect_radar.py` 的推理流程：

1. **模型加载**：加载训练好的 `.pth` 权重文件
2. **图像预处理**：`preprocess_image()`
   - 读取 JPG → BGR → RGB → float32 [0,1]
   - 填充至 256×256
   - HWC → CHW → 添加 batch 维度
3. **模型推理**：DetBenchPredict 处理
   - 输出格式 `[batch, max_det, 6]`：`[x1, y1, x2, y2, score, class_id]`
4. **结果解码**：`decode_predictions()`
   - 按置信度阈值过滤（默认 0.3）
   - 将填充后坐标转换回原始图像坐标
5. **结果可视化**：`draw_detections()`
   - Vehicle 绿色、Pedestrian 红色、Bicycle 蓝色
   - 绘制边界框 + 置信度标签
6. **物理坐标转换**：`print_physical_coords()`
   - 将检测框像素坐标转换为物理量（距离、速度）

---

## 8. 常见问题排查

### 8.1 常见错误

| 错误现象 | 可能原因 | 解决方法 |
|---------|---------|---------|
| `Undefined function 'phased.FMCWWaveform'` | 缺少 Phased Array System Toolbox | 运行 `ver` 检查工具箱安装；若缺失需安装 |
| `Array formation` 相关错误 | MATLAB 版本过旧 | 确保 MATLAB ≥ R2020a |
| 运行时间过长 | `Szenarios` 或 `duration` 设置过大 | 减小参数值，或安装 Parallel Computing Toolbox 使用 `parfor` |
| 噪声淹没问题（RD 图中无目标可见） | `NoiseFloor` 设置过高 | 降低 `NoiseFloor`（如 -135dB），或增大 `TXpeakPower` |
| Python 找不到 .mat 文件 | `SimDataPath` 路径不匹配 | 修改 `JSONCoco.py` 中 `SimDataPath` 为实际路径 |
| `cv2` 模块导入失败 | 未安装 opencv-python | `pip install opencv-python` |
| `scipy.io.loadmat` 报错 | .mat 文件版本不适配 | 确保 .mat 文件为 v7.3 格式 |
| `effdet` 导入失败 | 未安装 effdet | `pip install effdet` |
| 训练时 GLIBCXX 版本错误 | conda 环境 libstdc++ 不匹配 | 脚本已内置自动修复（通过 `LD_LIBRARY_PATH` 预加载 conda 的 libstdc++） |

### 8.2 关键注意事项

1. **路径修改**：每次运行 `JSONCoco.py` 或 `JSONCoco_3SeqRGB.py` 前，务必检查 `SimDataPath` 是否指向正确的仿真输出目录，否则脚本可能因路径不存在而崩溃。

2. **数据清理**：`SimulateTargetList.m` 中 `add_files == false` 时会清空 `SimulationData/` 目录，如需保留数据请先备份或设置 `add_files = true`。

3. **MATLAB 工作目录**：运行仿真前确保 MATLAB 当前工作目录为 `FMCW_Radar_Target_Simulator/`，否则 `SimulationData/` 路径会不正确。

4. **随机性控制**：若需复现结果，取消 `TargetSimulation.m` 或 `SimulateTargetList.m` 中 `rng('default')` 的注释，或使用 `rng(seed)` 设置随机种子。

5. **内存消耗**：
   - 新版 `modelBasebandSignal.m`（优化版）：直接计算差频信号，内存占用小，速度快
   - 旧版 `modelSignal.m`（原始版）：使用 `phased.FreeSpace` 信道和 `dechirp` 混频，更精确但速度慢，且临时变量 `xRX` 会占用大量内存
   - `SimulateTargetList.m` 当前调用的是 `modelBasebandSignal`（高效版）

6. **天线方向图**：`FMCWradar.m` 中的天线方向图数据（`Vpattern` / `Hpattern`）为近似值，精确仿真应参考具体雷达芯片的 datasheet。

7. **遮挡检测**：`generateObstructionMap.m` 基于目标的 `Height` 属性进行遮挡判断。更高大的目标（如车辆）会遮挡后方的较小目标（如行人）。自行车对后方目标仅产生部分遮挡（`obstruction = 0.5`）。

8. **数据增强**：当前训练脚本未包含数据增强（如随机翻转、旋转等），如需提高模型泛化能力，可以在 `RadarRDCOCODataset` 中增加 `transforms` 参数。

9. **权重文件**：预训练权重存放在项目根目录和 `checkpoints/` 目录下：
   - `efficientdet_d0_epoch5.pth` — 5 轮训练权重
   - `efficientdet_d0_epoch10.pth` — 10 轮训练权重
   - `checkpoints/efficientdet_d0_best.pth` — 最佳模型
   - `checkpoints/efficientdet_d0_epoch50.pth` — 50 轮训练完成

---

## 附录：参数修改路线图

一次完整的参数修改通常遵循以下路径：

```
修改需求                    → 修改位置                       → 影响范围
─────────────────────────────────────────────────────────────────────
改变距离/速度分辨率         → FMCWradar.m: sweepBw/chirpsCycle   → 所有场景
改变信噪比                 → FMCWradar.m: NoiseFloor            → 所有场景
改变场景数量               → SimulateTargetList.m: Szenarios    → 仿真时长
改变目标密度               → SimulateTargetList.m: 目标生成行   → 每场景目标数
修改目标物理尺寸           → Car.m/Pedestrian.m/Bicyclist.m     → 特定类型目标
修改运动行为               → TrajectoryPlanner.m                → 目标轨迹
修改数据集路径             → JSONCoco.py: SimDataPath          → 数据转换
修改训练超参数             → train_efficientdet.py 命令行参数   → 模型训练
修改推理阈值               → detect_radar.py: --threshold       → 检测结果
```
