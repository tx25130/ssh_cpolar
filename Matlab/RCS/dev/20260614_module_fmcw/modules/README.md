# FMCW MIMO 雷达仿真模块

> 77GHz TDM-MIMO FMCW 雷达信号处理仿真，基于 TI AWR2243 4芯片级联配置。
> 支持方位角 + 俯仰角二维角度估计（4元俯仰虚拟阵列 `[0, 0.5, 2, 3]` λ）。

---

## 目录

- [模块总览](#模块总览)
- [阵列配置](#阵列配置)
- [流水线架构](#流水线架构)
- [各模块详解](#各模块详解)
- [主程序使用规则](#主程序使用规则)
- [配置参数速查](#配置参数速查)
- [测试](#测试)
- [注意事项](#注意事项)

---

## 模块总览

| 序号 | 模块文件 | 功能 | 输入 → 输出 |
|:---:|---------|------|------------|
| 1 | `fmcw_sim_config.m` | 参数集中配置 | `varargin` → `cfg` |
| 2 | `fmcw_sim_signal.m` | TDM-MIMO 差频信号生成 | `cfg, targets` → `if_signal` |
| 3 | `fmcw_sim_clutter.m` | 杂波生成与叠加 | `cfg, if_signal` → `if_signal` |
| 4 | `fmcw_sim_noise.m` | 复高斯白噪声叠加 | `cfg, if_signal` → `if_signal` |
| 5 | `fmcw_sim_range_fft.m` | 快时间 FFT 测距 | `cfg, if_signal` → `range_fft, cfg` |
| 6 | `fmcw_sim_mti.m` | MTI 两脉冲对消 | `cfg, range_fft` → `mti_data, cfg` |
| 7 | `fmcw_sim_doppler_fft.m` | 慢时间 FFT 测速 | `cfg, mti_data` → `rd_data, cfg` |
| 8 | `fmcw_sim_virtual_array.m` | MIMO 虚拟阵列重组（方位+俯仰） | `cfg, rd_data` → `rd_virtual, rd_el_virtual, cfg` |
| 9 | `fmcw_sim_cfar.m` | 二维 CA-CFAR 检测 | `cfg, rd_virtual` → `det_mask, det_snr, cfg` |
| 10 | `fmcw_sim_angle.m` | 角度估计（方位+俯仰） | `cfg, rd_virtual, det_mask, rd_el_virtual` → `estimates, cfg` |
| 11 | `fmcw_sim_pointcloud.m` | 点云结构化与可视化（3D） | `cfg, estimates, targets` → `pointcloud, fig_handles` |
| 12 | `fmcw_sim_verify.m` | 仿真验证（含俯仰） | `targets, estimates, cfg` → `verify_result` |
| — | `fmcw_sim_main.m` | **主入口**（串联以上全部步骤） | `targets, varargin` → `estimates, verify_result, cfg` |
| — | `init_path.m` | 将模块目录添加到搜索路径 | 无输入输出 |

---

## 阵列配置

### 天线布局（AWR2243 4芯片级联）

```
方位维（水平面）:
  TX1~9: 间距 2λ, 覆盖 0~16λ
  RX1~16: 4芯片级联布局, 覆盖 0~26.5λ

俯仰维（垂直面）:
  TX10~12: 垂直位置 [0.5λ, 2λ, 3λ]
  TX1(借用): 俯仰位置 0λ（从方位维借用）
  → 4元俯仰虚拟阵列 [0, 0.5, 2, 3] λ
```

### 虚拟阵列参数

| 参数 | 方位维 | 俯仰维 |
|------|--------|--------|
| **物理TX** | TX1~9（9个） | TX10~12（3个） |
| **RX** | 16（水平排列） | 16（水平排列，俯仰位置=0） |
| **原始虚拟阵元** | 9×16 = 144 | 3×16 = 48 |
| **去重叠后** | **86** | **4**（借用TX1 + 3俯仰TX） |
| **阵列孔径** | 42.5 λ | 3 λ |
| **3dB波束宽度** | ≈ 1.2° | ≈ 16.9° |
| **角分辨率** | 高（86元大孔径） | 低（4元稀疏阵） |

### 俯仰维借用机制

AWR2243 中所有 16 个 RX 都在水平面排列，俯仰维无空间采样。为提升俯仰角分辨力：

1. **借用方位 TX1**（俯仰位置=0λ），与 3 个俯仰 TX10~12 组成 4 元虚拟阵列
2. 物理发射天线仍为 **12 个**（9 方位 + 3 俯仰），不额外占用 chirp 资源
3. 虚拟阵列构建时，TX1 的数据同时参与方位维和俯仰维处理

```
TX12 ── 3λ ──┐
              │
TX11 ── 2λ ──┤   4元俯仰虚拟阵列
              │
TX10 ── 0.5λ ─┤
              │
TX1(借用) ── 0λ ─┘
```

---

## 流水线架构

```
目标定义 [R, v, θ_az, θ_el, RCS]
     │
     ▼
┌─────────────────┐
│  fmcw_sim_signal │  TDM-MIMO 差频信号生成（12TX: 9方位+3俯仰）
└────────┬────────┘
         │
    ┌────▼─────┐
    │ clutter   │  杂波叠加（可选）
    └────┬─────┘
         │
    ┌────▼─────┐
    │  noise    │  噪声叠加
    └────┬─────┘
         │
    ┌────▼──────────┐
    │  Range-FFT     │  快时间 FFT → 距离
    └────┬──────────┘
         │
    ┌────▼─────┐
    │   MTI     │  两脉冲对消（可选）
    └────┬─────┘
         │
    ┌────▼──────────┐
    │ Doppler-FFT   │  慢时间 FFT → 速度
    └────┬──────────┘
         │
    ┌────▼──────────────┐
    │ 虚拟阵列重组       │  方位: TDM补偿+重叠平均(144→86)
    │ (方位+俯仰)        │  俯仰: 借用TX1+3俯仰TX → 4元阵
    └────┬──────────────┘
         │
    ┌────▼──────────┐
    │  CA-CFAR      │  恒虚警检测
    └────┬──────────┘
         │
    ┌────▼──────────┐
    │  角度估计      │  方位: FFT/DBF/MUSIC (86元)
    │ (方位+俯仰)    │  俯仰: DBF (4元 [0,0.5,2,3]λ)
    └────┬──────────┘
         │
    ┌────▼──────────┐
    │  点云可视化    │  BEV鸟瞰图 + 3D点云（含高度Z）
    └────┬──────────┘
         │
    ┌────▼──────────┐
    │  仿真验证      │  RMSE(R,V,Az,El) + PASS/FAIL
    └───────────────┘
```

---

## 各模块详解

### 1. `fmcw_sim_config.m` — 参数配置

集中管理所有仿真参数，返回结构体 `cfg`。

```matlab
cfg = fmcw_sim_config()                          % 默认参数
cfg = fmcw_sim_config('fc', 79e9, 'SNR_dB', 30)  % 覆盖指定参数
```

**核心参数分组**：

| 分组 | 关键字段 | 默认值 | 说明 |
|------|---------|--------|------|
| FMCW 波形 | `fc`, `gama`, `fs`, `Ns` | 77e9, 24e12, 4e6, 512 | 载频、斜率、采样率、采样点 |
| 信号维度 | `numLoops`, `numChirpPerLoop`, `Nchirp` | 64, 12, 768 | 慢时间点数、轮询数、总chirp数 |
| MIMO 阵列 | `Ntx`, `Nrx`, `Nv`, `Ntx_el`, `Nv_el` | 9, 16, 144, 3, 48 | 方位/俯仰收发天线数 |
| 俯仰阵列 | `d_el_tx_pos`, `el_borrowed_tx` | `[0;0.5;2;3]`, 1 | 俯仰4元阵位置/借用TX索引 |
| 处理开关 | `enable_clutter`, `enable_MTI`, `enable_pointcloud` | true, true, true | 杂波/MTI/点云开关 |
| 角度方法 | `angle_method` | `'DBF'` | `'FFT'`/`'DBF'`/`'MUSIC'` |
| 杂波参数 | `clutter_dist`, `clutter_power`, `clutter_shape` | `'Rayleigh'`, 1e-15, 1.5 | 分布/功率/Weibull形状 |
| 噪声 | `SNR_dB` | 20 | 信噪比 |
| 图像保存 | `save_fig`, `fig_dir` | false, `''` | 保存PNG/输出目录 |
| CFAR | `cfar2_Pfa`, `cfar2_Tr`, `cfar2_Gr` | 1e-6, 3, 2 | 虚警概率/参考/保护单元 |
| 角度扫描 | `az_angle_range`, `el_angle_range`, `angle_fft_n` | -60:0.1:60, -30:0.1:30, 256 | 方位/俯仰DBF扫描角/FFT点数 |

> **注意**：不支持的字段名会直接报错，请检查拼写。

---

### 2. `fmcw_sim_signal.m` — 信号生成

生成 TDM-MIMO 差频（IF，中频）信号数据立方体。

```matlab
if_signal = fmcw_sim_signal(cfg, targets)
```

- **输入**：`cfg` 配置，`targets` 目标矩阵 `[N×4]` 或 `[N×5]`
  - `[R(m), v(m/s), θ_az(°), RCS(m²)]` — 无俯仰角
  - `[R(m), v(m/s), θ_az(°), θ_el(°), RCS(m²)]` — 含俯仰角
- **输出**：`if_signal` 差频信号 `[Ns × Nchirp_per_tx × Nrx × Ntx_total]`
- **信号模型**：去斜接收（Dechirp），包含自由空间路径衰减
- **TX 结构**：TX1~9 方位（水平相位），TX10~12 俯仰（垂直相位）

---

### 3. `fmcw_sim_clutter.m` — 杂波生成

生成并叠加杂波信号。由 `cfg.enable_clutter` 控制开关。

```matlab
if_signal = fmcw_sim_clutter(cfg, if_signal)
```

- **支持的杂波分布**：
  - `'Rayleigh'` — 复高斯散斑（默认）
  - `'Weibull'` — ZMNL 变换，形状参数 `cfg.clutter_shape`
  - `'K'` — Gamma 纹理 × 复高斯散斑（SIRP 模型）
- **杂波特性**：高斯多普勒谱塑形（频域滤波），CNR 功率控制

---

### 4. `fmcw_sim_noise.m` — 噪声叠加

叠加循环对称复高斯白噪声。

```matlab
if_signal = fmcw_sim_noise(cfg, if_signal)
```

- 噪声功率 = 信号功率 / 10^(SNR_dB/10)
- 实部虚部独立同分布 N(0, noise_power/2)

---

### 5. `fmcw_sim_range_fft.m` — Range-FFT

快时间维 Hamming 窗 + FFT，实现测距。

```matlab
[range_fft, cfg] = fmcw_sim_range_fft(cfg, if_signal)
```

- **输出**：`range_fft` 距离 FFT 结果 `[Ns × Nchirp_per_tx × Nrx × Ntx]`
- 更新 `cfg.R_axis`（距离轴，单位 cm）

---

### 6. `fmcw_sim_mti.m` — MTI 对消

两脉冲对消动目标指示。由 `cfg.enable_MTI` 控制开关。

```matlab
[mti_data, cfg] = fmcw_sim_mti(cfg, range_fft)
```

- **输出**：`mti_data` 对消后数据 `[Ns × (Nchirp_per_tx-1) × Nrx × Ntx]`
- 更新 `cfg.V_axis_tdm`（MTI 后速度轴）
- 对消公式：`y[n] = x[n] - x[n-1]`

> **注意**：MTI 会抑制零速附近的低速目标，使用时需确保目标速度远离零速。

---

### 7. `fmcw_sim_doppler_fft.m` — Doppler-FFT

慢时间维 Hamming 窗 + FFT + fftshift，实现测速。

```matlab
[rd_data, cfg] = fmcw_sim_doppler_fft(cfg, mti_data)
```

- **输出**：`rd_data` R-D 谱 `[Ns × Ndop × Nrx × Ntx]`
- 更新 `cfg.V_axis_tdm`（TDM 速度轴，单位 m/s）

---

### 8. `fmcw_sim_virtual_array.m` — 虚拟阵列重组

MIMO 虚拟孔径重组：TDM 多普勒补偿 + 重叠阵元相干平均，输出方位+俯仰两路虚拟阵列。

```matlab
[rd_virtual, rd_el_virtual, cfg] = fmcw_sim_virtual_array(cfg, rd_data)
```

- **输出**：
  - `rd_virtual` 方位虚拟阵列 R-D 数据 `[Ns × Ndop × Nv_unique]`（86维）
  - `rd_el_virtual` 俯仰虚拟阵列 R-D 数据 `[Ns × Ndop × 4]`
  - 更新 `cfg.Nv_unique`、`cfg.Nv_el_unique`、`cfg.virtual_az_pos_unique`、`cfg.virtual_el_pos_unique`
- **方位维处理**：
  - TDM 相位补偿：`exp(-j·4π·v_est·(tx-1)·Tp/λ)`
  - 在 λ 单位下做 `unique()` 去重叠（9×16=144 → 86）
  - 同位置多个 TX-RX 对的信号做相干平均
- **俯仰维处理**：
  - 借用方位 TX1（俯仰位置=0λ）+ 3 个俯仰 TX10~12
  - 每个 TX 对应所有 RX 做相干平均（16次平均，SNR 增益 ≈ 12dB）
  - 输出 4 元快拍，位置 `[0, 0.5, 2, 3]` λ

---

### 9. `fmcw_sim_cfar.m` — CFAR 检测

二维 CA-CFAR 恒虚警检测。

```matlab
[det_mask, det_snr, cfg] = fmcw_sim_cfar(cfg, rd_virtual)
```

- **输出**：
  - `det_mask` 检测掩码 `[Ns × Ndop]`（布尔）
  - `det_snr` 检测 SNR `[Ns × Ndop]`
- 处理前对所有虚拟阵元做非相干功率积累

---

### 10. `fmcw_sim_angle.m` — 角度估计

对 CFAR 检测点进行方位角 + 俯仰角估计。

```matlab
[estimates, cfg] = fmcw_sim_angle(cfg, rd_virtual, det_mask, rd_el_virtual)
```

- **输出**：`estimates` 结构体数组，每点含 `range`, `velocity`, `angle`, `elevation`, `snr_dB`
- **方位角**：使用 86 元方位虚拟阵列，支持三种算法

| 算法 | 速度 | 精度 | 适用场景 |
|------|------|------|---------|
| `'FFT'` | 快 | 低 | 快速初估 |
| `'DBF'` | 中 | 中 | 通用场景（默认） |
| `'MUSIC'` | 慢 | 高 | 超分辨需求 |

- **俯仰角**：使用 4 元俯仰虚拟阵列，固定使用 DBF 扫描
  - 扫描范围由 `cfg.el_angle_range` 控制（默认 `-30:0.1:30`°）
  - 4 元非均匀阵 `[0, 0.5, 2, 3]` λ，3dB 波束宽度 ≈ 16.9°

---

### 11. `fmcw_sim_pointcloud.m` — 点云可视化

将检测结果结构化并绘制 2D/3D 点云图。由 `cfg.enable_pointcloud` 控制。

```matlab
[pointcloud, fig_handles] = fmcw_sim_pointcloud(cfg, estimates, targets)
```

- **输出**：
  - `pointcloud` 结构体数组：`range`, `velocity`, `angle`, `elevation`, `snr_dB`, `power`, `x`, `y`, `z`
  - `fig_handles` 图形句柄
- `targets` 为真值矩阵 `[N×4]` 或 `[N×5]`（可选，用于对比标记）
- **3D 坐标转换**：
  ```
  x = R × cos(El) × sin(Az)    → 横向
  y = R × cos(El) × cos(Az)    → 纵向
  z = R × sin(El)               → 高度
  ```
- **可视化**：BEV 鸟瞰图（X-Y）+ 3D 点云图（X-Y-Z）

---

### 12. `fmcw_sim_verify.m` — 仿真验证

将估计结果与真值对比，输出 RMSE 和 PASS/FAIL 判定（含俯仰角）。

```matlab
verify_result = fmcw_sim_verify(targets, estimates, cfg)
```

- **输出**：`verify_result` 结构体，含 `matched`, `errors`, `rmse`, `all_pass`
- **判定阈值**：距离 ≤ 1m，速度 ≤ 1m/s，方位角 ≤ 5°，**俯仰角 ≤ 5°**
- 目标矩阵无俯仰角（`[N×4]`）时，俯仰真值默认 0°

---

## 主程序使用规则

### `fmcw_sim_main.m` — 主入口

一站式调用完整 12 步流水线。

```matlab
% 最简调用（使用默认参数，目标含俯仰角）
targets = [5, 0, 10, 5, 10; 10, 0, -15, 0, 20];  % [R, v, Az, El, RCS]
[estimates, verify_result, cfg] = fmcw_sim_main(targets);

% 覆盖参数调用（静态目标，关闭MTI）
[estimates, verify_result, cfg] = fmcw_sim_main(targets, ...
    'enable_MTI', false, ...
    'enable_clutter', false, ...
    'SNR_dB', 30, ...
    'angle_method', 'FFT', ...
    'az_angle_range', -60:2:60, ...
    'el_angle_range', -30:0.5:30, ...
    'enable_pointcloud', false);
```

### `-batch` 命令行模式

在 PowerShell 中使用 `matlab -batch` 运行仿真，需搭配 `save_fig` 保存图像（`-batch` 模式无图形界面，无法交互查看）：

```powershell
# 双目标测试（含俯仰角，关闭MTI，保存图像）
matlab -batch "addpath('D:\Python\Matlab_1\RCS\dev\20260614\modules'); targets=[5.0,0,10.0,5.0,10; 10.0,0,-15.0,0.0,20]; fmcw_sim_main(targets,'enable_MTI',false,'save_fig',true,'fig_dir','D:\Python\Matlab_1\RCS\dev\20260614\output')"

# 运动目标（启用MTI，速度须远离零速）
matlab -batch "addpath('D:\Python\Matlab_1\RCS\dev\20260614\modules'); targets=[2.16,5,0,0,10; 2.79,-3,-29.1,0,10]; fmcw_sim_main(targets,'save_fig',true,'fig_dir','D:\Python\Matlab_1\RCS\dev\20260614\output')"

# 仅运行流水线不绘图不保存（快速验证）
matlab -batch "addpath('D:\Python\Matlab_1\RCS\dev\20260614\modules'); targets=[5,0,10,5,10; 10,0,-15,0,20]; fmcw_sim_main(targets,'enable_MTI',false,'enable_pointcloud',false)"
```

**`-batch` 模式关键参数**：

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `enable_MTI` | 静态目标(v=0)必须关闭 | `false`（静态）/ `true`（运动） |
| `enable_pointcloud` | 控制是否绘图 | `true`（配合 `save_fig`）/ `false` |
| `save_fig` | 自动保存所有图像为 PNG | `true` |
| `fig_dir` | 图像输出目录 | 绝对路径，如 `'D:\...\output'` |

> **注意**：`save_fig` 保存所有打开的 figure 窗口（包括非本模块创建的），文件名取自 figure 的 `Name` 属性。

**输入**：
- `targets` — 目标矩阵 `[N×4]`，每行 `[R(m), v(m/s), θ_az(°), RCS(m²)]`
- `varargin` — 名称-值对，直接透传给 `fmcw_sim_config`

**输出**：
- `estimates` — 检测估计结果结构体数组
- `verify_result` — 验证结果结构体
- `cfg` — 完整配置

### 手动分步调用

也可不使用主入口，手动分步调用各模块：

```matlab
addpath('modules');

% 1. 配置
cfg = fmcw_sim_config('enable_MTI', false, 'SNR_dB', 30);
targets = [5.0, 0, 10.0, 5.0, 10; 10.0, 0, -15.0, 0.0, 20];  % 含俯仰角

% 2-4. 信号 + 杂波 + 噪声
if_sig = fmcw_sim_signal(cfg, targets);
if_sig = fmcw_sim_clutter(cfg, if_sig);
if_sig = fmcw_sim_noise(cfg, if_sig);

% 5-7. FFT 链
[rf, cfg] = fmcw_sim_range_fft(cfg, if_sig);
[mti, cfg] = fmcw_sim_mti(cfg, rf);
[rd, cfg]  = fmcw_sim_doppler_fft(cfg, mti);

% 8-10. 虚拟阵列(方位+俯仰) + 检测 + 角度
[rd_v, rd_el, cfg] = fmcw_sim_virtual_array(cfg, rd);
[mask, ~, cfg]     = fmcw_sim_cfar(cfg, rd_v);
[est, cfg]         = fmcw_sim_angle(cfg, rd_v, mask, rd_el);  % 传入俯仰数据

% 11-12. 可视化 + 验证
fmcw_sim_pointcloud(cfg, est, targets);
vr = fmcw_sim_verify(targets, est, cfg);
```

---

## 配置参数速查

### 常用覆盖参数

```matlab
cfg = fmcw_sim_config(...
    'fc', 77e9,              ...  % 载频 (Hz)
    'gama', 24e12,           ...  % 调频斜率 (Hz/s)
    'fs', 4e6,               ...  % 采样率 (Hz)
    'Ns', 512,               ...  % 快时间采样点
    'Ntx', 9,                ...  % 方位发射天线数
    'Nrx', 16,               ...  % 接收天线数
    'Ntx_el', 3,             ...  % 俯仰发射天线数
    'd_el_tx_pos', [0;0.5;2;3], ... % 俯仰虚拟阵列位置(λ)
    'enable_clutter', true,  ...  % 杂波开关
    'enable_MTI', true,      ...  % MTI 开关
    'enable_pointcloud', true, ... % 点云可视化开关
    'clutter_dist', 'Rayleigh', ...% 杂波分布
    'clutter_power', 1e-15,  ...  % 杂波功率 (W)
    'clutter_shape', 1.5,    ...  % Weibull 形状参数
    'SNR_dB', 20,            ...  % 信噪比 (dB)
    'angle_method', 'DBF',   ...  % 角度方法: 'FFT'/'DBF'/'MUSIC'
    'az_angle_range', -60:0.1:60, ... % 方位 DBF 扫描角度
    'el_angle_range', -30:0.1:30, ... % 俯仰 DBF 扫描角度
    'cfar2_Pfa', 1e-6,       ...  % CFAR 虚警概率
    'cfar2_Tr', 3,           ...  % CFAR 距离维参考单元
    'cfar2_Gr', 2            ...  % CFAR 距离维保护单元
);
```

### 导出参数（自动计算，只读）

| 字段 | 含义 | 计算公式 |
|------|------|---------|
| `lambda` | 波长 | c / fc |
| `Rres` | 距离分辨率 | c·fs / (2·gama·Ns) |
| `Vres` | 速度分辨率 | λ / (2·numLoops·Tp) |
| `Rmax` | 最大无模糊距离 | c·Tp / 2 |
| `Vmax` | 最大无模糊速度 | λ / (4·Tp) |
| `Nv_unique` | 方位去重叠虚拟阵元数 | 86（9×16=144 去重叠） |
| `Nv_el_unique` | 俯仰去重叠虚拟阵元数 | 4（3俯仰TX + 1借用TX） |
| `el_borrowed_tx` | 借用的方位TX索引 | 1（TX1，俯仰位置=0λ） |
| `el_tx_pos_physical` | 物理俯仰TX位置(λ) | `[0.5, 2, 3]`（不含借用） |
| `virtual_el_pos_unique` | 俯仰虚拟阵元位置(m) | `[0, 0.5, 2, 3]` × λ |
| `R_axis` | 距离轴 (cm) | 自动生成 |
| `V_axis` | 速度轴 (m/s) | 自动生成 |

---

## 测试

### 完整测试套件（5项）

```powershell
cd D:\Python\Matlab_1\RCS\dev\20260614
matlab -batch "run('test_fmcw_sim.m')"
```

**测试项**：

| # | 场景 | 验证内容 |
|---|------|---------|
| 1 | DBF + 静态双目标 | 基础流水线正确性 |
| 2 | FFT 角度估计 | FFT 角度方法 |
| 3 | Rayleigh 杂波 + MTI 运动目标 | 杂波生成 + MTI 抑制 |
| 4 | Weibull 杂波 + MTI | 非高斯杂波 |
| 5 | `fmcw_sim_main` 主入口 | 端到端集成 |

### 双目标俯仰角测试

```powershell
cd D:\Python\Matlab_1\RCS\dev\20260614
matlab -batch "run('test_two_targets.m')"
```

**测试目标**：验证 4 元俯仰虚拟阵列 `[0, 0.5, 2, 3]` λ 的角度估计能力

| 目标 | R (m) | v (m/s) | Az (°) | El (°) | RCS (m²) |
|------|-------|---------|--------|--------|----------|
| 1 | 5.0 | 0 | 10.0 | 5.0 | 10 |
| 2 | 10.0 | 0 | -15.0 | 0.0 | 20 |

> 俯仰角误差应 < 0.1°（高 SNR 无杂波条件）

---

## 注意事项

1. **TDM-MIMO 速度模糊**：有效最大速度 `Vmax_tdm ≈ 0.22 m/s`，目标速度超出此范围将发生速度模糊。使用 MTI 时目标速度须在 `Vmax_tdm` 内且远离零速。

2. **杂波功率量级**：信号功率约 `5.77e-15 W`，`clutter_power` 需与信号量级匹配（推荐 `1e-15`~`1e-14`）。设置过大会淹没目标。

3. **DBF 扫描步进**：`az_angle_range` 默认 `-60:0.1:60`（1201 点），`el_angle_range` 默认 `-30:0.1:30`（601 点），扫描精细但耗时。快速测试可用 `-60:2:60` 和 `-30:1:30`。

4. **方位虚拟阵元去重叠**：9×16=144 个虚拟阵元在 λ 单位下去重叠后为 86 个。内部在整数尺度下做 `unique()`，避免浮点精度问题。

5. **俯仰虚拟阵列借用机制**：4 元俯仰阵 `[0, 0.5, 2, 3]` λ 中，位置 0λ 借用方位 TX1。物理发射天线仍为 12 个（9+3），不额外占用 chirp 资源。

6. **俯仰角分辨率限制**：4 元非均匀阵列孔径仅 3λ，3dB 波束宽度约 16.9°，无法分辨间距 <16° 的两个俯仰目标。

7. **目标矩阵格式**：支持 `[N×4]`（无俯仰角，El 默认 0°）和 `[N×5]`（含俯仰角 `[R, v, Az, El, RCS]`）两种格式。

8. **MTI 与静态目标**：静态目标（v=0）会被 MTI 对消，测试静态目标时需关闭 MTI（`enable_MTI=false`）。

9. **`-batch` 模式图像保存**：使用 `save_fig=true` + `fig_dir='路径'` 自动保存所有图像为 PNG（150 DPI），无需图形界面即可查看。推荐配合 `enable_pointcloud=true` 使用。
