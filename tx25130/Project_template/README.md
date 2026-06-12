# Project

> 项目标准目录模板，适用于车载毫米波雷达信号处理及通用 Python/MATLAB 工程项目。

---

## 目录结构

```
Project/
├── src/                # 源代码（source）
│   ├── core/           #   核心算法
│   ├── features/       #   功能模块
│   ├── utils/          #   工具函数
│   ├── models/         #   数据模型/类定义
│   └── __init__.py     #   包入口
├── config/             # 配置文件
│   ├── default.yaml    #   默认配置
│   ├── development.yaml#   开发环境配置
│   ├── production.yaml #   生产环境配置
│   └── schema.py       #   配置校验模式
├── data/               # 数据文件
│   ├── raw/            #   原始数据（只读）
│   ├── interim/        #   中间处理结果
│   ├── processed/      #   最终处理结果
│   └── external/       #   第三方数据
├── test/               # 测试
│   ├── unit/           #   单元测试
│   ├── integration/    #   集成测试
│   └── fixtures/       #   测试数据与模拟对象
├── doc/                # 文档
│   ├── api/            #   API 文档
│   ├── design/         #   设计文档
│   └── tutorials/      #   教程与示例
├── assets/             # 静态资源
│   ├── images/         #   图片
│   └── diagrams/       #   架构图/流程图
├── dev/                # 开发工作区（development）
└── README.md           # 本文件
```

---

## 目录说明

### `src/` — 源代码

项目运行的正式代码，功能稳定、可复用。按功能模块划分子目录。

- **入 Git**：✅
- **命名规范**：模块名使用小写下划线，如 `signal_processing/`、`clutter_model/`

#### 可创建的子目录

| 子目录 | 说明 | 示例文件 |
|--------|------|----------|
| `core/` | 核心算法与主流程 | `core/range_fft.py`、`core/doppler_fft.py`、`core/cfar.py` |
| `features/` | 独立功能模块 | `features/dbf.py`、`features/mti.py`、`features/rcs_calc.py` |
| `utils/` | 通用工具函数 | `utils/io.py`、`utils/plot.py`、`utils/constants.py` |
| `models/` | 数据模型/类定义 | `models/radar_config.py`、`models/target.py` |
| `pipelines/` | 数据处理流水线 | `pipelines/calibration.py`、`pipelines/detection.py` |

#### 可创建的文件

| 文件 | 说明 |
|------|------|
| `__init__.py` | 包入口，导出公共接口 |
| `__main__.py` | 模块直接运行入口（`python -m src`） |
| `types.py` | 类型别名与 Protocol 定义 |
| `exceptions.py` | 自定义异常类 |

#### 使用说明

```python
# 从 src 导入模块
from src.core.range_fft import compute_range_fft
from src.features.cfar import cfar_2d
from src.utils.constants import SPEED_OF_LIGHT
```

**迁移规则**：`dev/` 中验证通过的功能，提取至 `src/` 对应模块，移除临时调试代码，补充 docstring 和类型注解。

---

### `config/` — 配置文件

与代码分离的参数配置，改配置不改代码。按环境区分（开发/生产）。

- **入 Git**：✅（敏感信息使用环境变量，不入库）
- **命名规范**：`{环境名}.yaml` 或 `{环境名}.json`

#### 可创建的文件

| 文件 | 说明 | 格式 |
|------|------|------|
| `default.yaml` | 全局默认配置，所有环境共享 | YAML |
| `development.yaml` | 开发环境覆盖配置 | YAML |
| `production.yaml` | 生产环境覆盖配置 | YAML |
| `logging.yaml` | 日志配置（级别、格式、输出） | YAML |
| `schema.py` | 配置字段校验模式（Pydantic/Schema） | Python |
| `radar_params.yaml` | 雷达参数配置（中心频率、带宽、chirp 等） | YAML |

#### 配置文件示例

```yaml
# config/default.yaml
radar:
  center_freq_ghz: 77.0
  bandwidth_ghz: 4.0
  num_chirps: 128
  num_samples: 512
  num_tx: 3
  num_rx: 4

processing:
  range_fft_size: 512
  doppler_fft_size: 128
  cfar_method: "ca"       # CA-CFAR
  cfar_guard_cells: 4
  cfar_train_cells: 8

logging:
  level: "INFO"
  format: "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
```

#### 使用说明

```python
from src.utils.config import load_config

# 加载配置：default.yaml + 环境覆盖
cfg = load_config(env="development")
center_freq = cfg.radar.center_freq_ghz
```

**优先级**：`default.yaml` < `{环境}.yaml` < 环境变量

---

### `data/` — 数据文件

代码消费和产出的数据，按处理阶段划分子目录。

- **入 Git**：❌（通过 `.gitignore` 排除，仅保留目录结构占位文件）
- **命名规范**：`{日期}_{描述}.{ext}`，如 `20260603_calibration_adc.bin`

#### 子目录说明

| 子目录 | 说明 | 可重建？ | 可创建的文件 |
|--------|------|----------|-------------|
| `raw/` | 原始数据，只读不可修改 | ❌ | `.bin`（ADC 二进制）、`.h5`（HDF5）、`.mat`（MATLAB） |
| `interim/` | 中间处理结果 | ✅ | `.npy`、`.csv`、`.h5` |
| `processed/` | 最终处理结果 | ✅ | `.csv`（RCS 表）、`.png`（图表）、`.h5`（点云） |
| `external/` | 第三方数据 | ❌ | `.zip`、`.tar.gz`、数据集索引文件 |

#### 占位文件

每个子目录放置 `.gitkeep` 文件以确保空目录能被 Git 追踪：

```
data/raw/.gitkeep
data/interim/.gitkeep
data/processed/.gitkeep
data/external/.gitkeep
```

#### 使用说明

```python
from src.utils.io import read_adc_bin, save_processed

# 读取原始数据
raw_data = read_adc_bin("data/raw/20260603_calibration_adc.bin")

# 保存处理结果
save_processed(detection_result, "data/processed/20260603_detection.csv")
```

**数据管理原则**：
- `raw/` 中的文件**绝不修改**，任何处理结果写入 `interim/` 或 `processed/`
- 大文件（>100MB）不入 Git，使用 `.gitignore` 排除
- 在 `data/external/` 中放置 `README.md` 记录数据来源与下载方式

---

### `test/` — 测试

验证代码正确性的测试代码，按测试层级划分。

- **入 Git**：✅
- **命名规范**：`test_{模块名}.py`，如 `test_range_fft.py`

#### 子目录说明

| 子目录 | 说明 | 可创建的文件 |
|--------|------|-------------|
| `unit/` | 单元测试，验证单个函数/模块 | `test_range_fft.py`、`test_cfar.py`、`test_dbf.py` |
| `integration/` | 集成测试，验证模块间协作 | `test_pipeline.py`、`test_calibration_flow.py` |
| `fixtures/` | 测试数据与模拟对象 | `mock_adc_data.npy`、`sample_config.yaml`、`conftest.py` |

#### 可创建的文件

| 文件 | 说明 |
|------|------|
| `conftest.py` | pytest 公共 fixture 定义 |
| `test_*.py` | 测试用例文件 |
| `fixtures/*.npy` | 小型测试数据（NumPy 格式） |
| `fixtures/*.yaml` | 测试专用配置 |

#### 使用说明

```python
# test/unit/test_range_fft.py
import numpy as np
from src.core.range_fft import compute_range_fft

def test_range_fft_output_shape():
    """测试 Range FFT 输出维度正确性。"""
    signal = np.random.randn(128, 512)  # 128 chirps × 512 samples
    result = compute_range_fft(signal)
    assert result.shape == (128, 512)
```

```powershell
# 运行全部测试
python -m pytest test/ -v

# 仅运行单元测试
python -m pytest test/unit/ -v

# 运行指定测试文件
python -m pytest test/unit/test_range_fft.py -v
```

---

### `doc/` — 文档

项目说明文档，让人看懂项目的设计与用法。

- **入 Git**：✅
- **命名规范**：Markdown 文件使用小写连字符，如 `signal-flow.md`

#### 子目录说明

| 子目录 | 说明 | 可创建的文件 |
|--------|------|-------------|
| `api/` | API（应用程序编程接口）文档 | `range_fft_api.md`、`cfar_api.md` |
| `design/` | 设计文档（架构、算法选型） | `architecture.md`、`clutter_model_design.md` |
| `tutorials/` | 教程与示例 | `quickstart.md`、`data_pipeline_walkthrough.md` |

#### 可创建的文件

| 文件 | 说明 |
|------|------|
| `changelog.md` | 版本变更日志 |
| `contributing.md` | 贡献指南 |
| `faq.md` | 常见问题解答 |
| `glossary.md` | 术语表（如 RCS、CFAR、DBF 等） |

#### 使用说明

文档采用 Markdown 格式，结构如下：

```markdown
# 功能名称

## 概述
简要描述功能用途。

## 接口
### 函数签名
`def compute_range_fft(signal: np.ndarray) -> np.ndarray`

### 参数
| 参数 | 类型 | 说明 |
|------|------|------|
| signal | np.ndarray | 输入信号，形状 (chirps, samples) |

### 返回值
| 返回 | 类型 | 说明 |
|------|------|------|
| result | np.ndarray | Range FFT 结果 |

## 示例
（代码示例）
```

---

### `assets/` — 静态资源

文档引用的图片、字体、模型等辅助资源。

- **入 Git**：✅（仅限小文件，>5MB 用 `.gitignore` 排除）
- **命名规范**：小写下划线，如 `range_doppler_map.png`

#### 子目录说明

| 子目录 | 说明 | 可创建的文件 |
|--------|------|-------------|
| `images/` | 文档插图、结果截图 | `range_doppler_map.png`、`beam_pattern.png` |
| `diagrams/` | 架构图、流程图（源文件+导出） | `pipeline_flow.drawio`、`pipeline_flow.png` |

#### 使用说明

在 Markdown 文档中引用资源：

```markdown
![距离-多普勒图](../assets/images/range_doppler_map.png)
```

**绘图工具推荐**：
- 流程图/架构图：Draw.io（`.drawio` 源文件放入 `diagrams/`）
- 数据图表：由代码直接生成，保存至 `images/`

---

### `dev/` — 开发工作区

开发阶段的临时草稿、实验脚本，功能验证通过后迁移至 `src/`。

- **入 Git**：❌（通过 `.gitignore` 排除）
- **命名规范**：`YYYY-MM-DD_英文主题小写`

#### 可创建的子目录与文件

```
dev/
├── 2026-06-03_feature_alpha/     # 按日期+主题创建文件夹
│   ├── experiment.py             #   实验脚本
│   ├── notes.md                  #   实验笔记
│   └── temp_output/              #   临时输出（可删）
├── 2026-06-04_bug_fix/           # Bug 修复调试
│   └── debug_script.py           #   调试脚本
└── scratch/                      # 临时草稿（随时清理）
    └── test_idea.py              #   快速验证想法
```

| 文件/目录 | 说明 |
|-----------|------|
| `YYYY-MM-DD_topic/` | 日期主题文件夹，每次独立任务一个 |
| `*.py` / `*.m` | 实验/调试脚本 |
| `notes.md` | 实验笔记、思路记录 |
| `temp_output/` | 临时输出目录（验证后删除） |
| `scratch/` | 快速草稿区（无主题约束） |

#### 使用说明

```powershell
# 创建新的开发工作区
mkdir dev\2026-06-03_clutter_simulation
cd dev\2026-06-03_clutter_simulation

# 完成后迁移到 src/
# 1. 提取稳定代码，移除调试代码
# 2. 添加 docstring 和类型注解
# 3. 移入 src/ 对应模块
# 4. 删除 dev/ 中的临时文件
```

**迁移检查清单**：
- [ ] 代码已提取至 `src/` 对应模块
- [ ] 已添加 docstring 和类型注解
- [ ] 已在 `test/` 中补充单元测试
- [ ] `dev/` 临时文件已清理

---

## 核心原则

1. **代码与配置分离** — 改参数改 `config/`，不改 `src/`
2. **代码与数据分离** — 数据不入版本控制，用 `.gitignore` 排除
3. **正式与开发分离** — `src/` 是成品，`dev/` 是草稿
4. **原始数据不可变** — `data/raw/` 只读，任何处理结果写入下游目录

## 迁移流程

```
dev/YYYY-MM-DD_topic  →  验证通过  →  src/对应模块  →  补充测试到 test/
                      →  废弃      →  删除
```

1. 在 `dev/` 下创建日期主题文件夹开发
2. 功能验证通过后，迁移至 `src/` 对应模块
3. 在 `test/` 中补充对应的单元测试
4. 清理 `dev/` 中的临时文件

## 项目初始化

新建项目时，创建目录结构和占位文件：

```powershell
# 创建目录
mkdir src, config, data\raw, data\interim, data\processed, data\external
mkdir test\unit, test\integration, test\fixtures
mkdir doc\api, doc\design, doc\tutorials
mkdir assets\images, assets\diagrams
mkdir dev

# 创建占位文件
New-Item -ItemType File -Path data\raw\.gitkeep, data\interim\.gitkeep, data\processed\.gitkeep, data\external\.gitkeep
New-Item -ItemType File -Path src\__init__.py, src\core\__init__.py, src\features\__init__.py, src\utils\__init__.py
New-Item -ItemType File -Path config\default.yaml
New-Item -ItemType File -Path test\conftest.py

# 配置 .gitignore
@"
# 数据目录（保留结构，排除内容）
data/raw/*
data/interim/*
data/processed/*
data/external/*
!data/*/.gitkeep

# 开发工作区
dev/

# Python
__pycache__/
*.pyc
.venv/

# 大文件
*.bin
*.h5
*.mat
"@ | Out-File -Encoding utf8 .gitignore
```
