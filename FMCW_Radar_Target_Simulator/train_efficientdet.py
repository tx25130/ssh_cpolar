# -*- coding: utf-8 -*-
"""
车载毫米波雷达 RD 图谱目标检测 - EfficientDet-D0 训练脚本

基于论文:
    "Simulation of Urban Automotive Radar Measurements for Deep
     Learning Target Detection" (Wengerter et al.)

功能:
    1. 加载 COCO 格式的雷达 RD 图谱数据集
    2. 构建 EfficientDet-D0 检测网络（3 类目标：Vehicle, Pedestrian, Bicycle）
    3. 训练并保存模型权重

数据集格式:
    - 图像: 256×160 px 灰度/三通道 RD 图谱
    - 标注: COCO JSON 格式
    - 类别: Vehicle(0), Pedestrian(1), Bicycle(2)

环境要求:
    Python >= 3.8
    torch >= 2.0
    torchvision
    timm
    effdet
    Pillow
    numpy
    opencv-python
    pycocotools
"""

import os
import sys
import json
import time
import math
import argparse
import logging
from datetime import datetime
from collections import defaultdict

# ---------------------------------------------------------------------------
# 环境修复：优先使用 conda 环境的 libstdc++（避免 GLIBCXX 版本不匹配）
# ---------------------------------------------------------------------------
_conda_lib = os.path.join(os.path.dirname(sys.executable), '..', 'lib')
_conda_lib = os.path.abspath(_conda_lib)
_conda_libstdcxx = os.path.join(_conda_lib, 'libstdc++.so.6')
if os.path.exists(_conda_libstdcxx):
    # 将 conda lib 插入到 LD_LIBRARY_PATH 最前面（仅影响当前进程）
    os.environ.setdefault('LD_LIBRARY_PATH', '')
    if _conda_lib not in os.environ.get('LD_LIBRARY_PATH', ''):
        os.environ['LD_LIBRARY_PATH'] = f"{_conda_lib}:{os.environ.get('LD_LIBRARY_PATH', '')}"
        # ctypes 方式预加载，确保 numpy/torch 引入前生效
        try:
            import ctypes
            ctypes.CDLL(_conda_libstdcxx, mode=ctypes.RTLD_GLOBAL)
        except Exception:
            pass

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from torch.utils.tensorboard import SummaryWriter
from PIL import Image, ImageDraw

import cv2

# ---------------------------------------------------------------------------
# 日志配置
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s | %(levelname)s | %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)


# ===========================================================================
# 常量定义
# ===========================================================================

# RD 图谱尺寸（来自 MATLAB 仿真参数）
FMCW_L = 256        # 速度门（Velocity bins），图像宽度
FMCW_K = 320        # 2 × 距离门（Range bins），图像原始高度的一半=160
FMCW_H = 160        # 实际图像高度

# 雷达物理参数（用于标签坐标与物理量换算）
SWEEP_BW = 1e9          # 扫频带宽 1 GHz
CHIRP_INTERVAL = 64e-6  # chirp 间隔 64 μs
C_0 = 299792458         # 光速
F0 = 76.5e9             # 载频 76.5 GHz

# 分辨率
FMCW_DR = C_0 / (2 * SWEEP_BW)              # 距离分辨率 ≈ 0.15 m
FMCW_DV = 1 / (FMCW_L * CHIRP_INTERVAL) * C_0 / (2 * F0)  # 速度分辨率 ≈ 0.127 m/s

# 类别映射（与 JSONCoco_3SeqRGB.py 保持一致）
COCO_CATEGORIES = {
    0: 'Vehicle',
    1: 'Pedestrian',
    2: 'Bicycle'
}
NUM_CLASSES = len(COCO_CATEGORIES)

# 数据集路径（可根据需要修改）
DEFAULT_COCO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'COCO')
DEFAULT_ANNOTATIONS_FILE = os.path.join(DEFAULT_COCO_DIR, 'annotations', 'instances_train2017.json')
DEFAULT_IMAGE_DIR = os.path.join(DEFAULT_COCO_DIR, 'images', 'train2017')


# ===========================================================================
# 数据集类
# ===========================================================================

class RadarRDCOCODataset(Dataset):
    """
    雷达 RD 图谱 COCO 格式数据集

    加载 256×160 RD 图谱图像及其对应的 COCO 标注，
    转换为 EfficientDet 所需的输入格式。
    """

    # EfficientDet BiFPN 要求 H/W 可被 128 整除
    target_size = (256, 256)  # (H, W) 填充目标尺寸

    def __init__(self, annotation_file, image_dir, transforms=None):
        """
        参数:
            annotation_file: COCO JSON 标注文件路径
            image_dir: JPG 图像所在目录
            transforms: torchvision 数据增强/变换
        """
        self.image_dir = image_dir
        self.transforms = transforms

        # 加载 COCO 标注
        with open(annotation_file, 'r') as f:
            self.coco_data = json.load(f)

        # 建立图像 ID → 图像信息的映射
        self.images = {}
        for img in self.coco_data['images']:
            self.images[img['id']] = img

        # 建立图像 ID → 标注列表的映射
        self.img_annotations = defaultdict(list)
        for ann in self.coco_data['annotations']:
            self.img_annotations[ann['image_id']].append(ann)

        # 获取所有有标注的图像 ID 列表
        self.image_ids = list(self.images.keys())

        logger.info(f"加载数据集: {len(self.image_ids)} 张图像, "
                    f"{len(self.coco_data['annotations'])} 个标注")
        logger.info(f"类别: {COCO_CATEGORIES}")

    def __len__(self):
        return len(self.image_ids)

    def __getitem__(self, idx):
        """
        返回:
            image: [3, H, W] RGB 图像张量，归一化到 [0,1]
                  图像会被填充至 256×256（EfficientDet 要求尺寸可被 128 整除）
            targets: 字典，包含:
                - 'bbox': [N, 4] Tensor，格式 [x, y, w, h]（COCO 格式，相对于原图）
                - 'cls': [N] Tensor，类别 ID
                - 'img_scale': float，图像缩放因子
                - 'img_size': (H, W) 填充后的图像尺寸
                - 'ori_size': (H, W) 原始图像尺寸（用于还原坐标）
        """
        # 获取图像信息
        img_id = self.image_ids[idx]
        img_info = self.images[img_id]
        img_path = os.path.join(self.image_dir, img_info['file_name'])
        orig_h = img_info['height']  # 160（Range 维度）
        orig_w = img_info['width']   # 256（Velocity 维度）

        # 加载图像（RD 图谱为灰度图，但 COCO 中存为 3 通道 JPG）
        image = cv2.imread(img_path, cv2.IMREAD_COLOR)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # BGR → RGB

        if image is None:
            raise FileNotFoundError(f"图像未找到: {img_path}")

        # 转换为 [0,1] 范围的 float32
        image = image.astype(np.float32) / 255.0

        # 填充至 256×256（EfficientDet BiFPN 需要 H 和 W 都能被 128 整除）
        # 原图 256×160，在下方填充 96 行 0，得到 256×256
        pad_h = self.target_size[0] - orig_h  # 需要填充的行数
        pad_w = self.target_size[1] - orig_w  # 需要填充的列数
        if pad_h > 0 or pad_w > 0:
            # 填充格式: (top, bottom, left, right)
            image = np.pad(
                image,
                ((0, pad_h), (0, pad_w), (0, 0)),
                mode='constant',
                constant_values=0
            )

        # 获取标注
        annotations = self.img_annotations[img_id]
        boxes = []
        classes = []

        for ann in annotations:
            # COCO 边界框格式: [x_min, y_min, width, height]
            # 注：bbox 坐标相对于原图，填充后 y 坐标不变（填充在下方）
            x, y, w, h = ann['bbox']
            # 确保框在原始图像范围内
            x = max(0, x)
            y = max(0, y)
            w = min(w, orig_w - x)
            h = min(h, orig_h - y)

            if w > 0 and h > 0:
                # 转换为 [x1, y1, x2, y2] 格式（effdet 内部使用）
                boxes.append([x, y, x + w, y + h])
                classes.append(ann['category_id'])

        # 转换为张量
        if len(boxes) > 0:
            boxes = torch.as_tensor(boxes, dtype=torch.float32)
            classes = torch.as_tensor(classes, dtype=torch.long)
        else:
            boxes = torch.zeros((0, 4), dtype=torch.float32)
            classes = torch.zeros((0,), dtype=torch.long)

        # 图像维度: HWC → CHW
        image = torch.as_tensor(image).permute(2, 0, 1).contiguous()

        target = {
            'bbox': boxes,
            'cls': classes,
            'img_scale': torch.tensor([1.0]),
            'img_size': torch.as_tensor([self.target_size[0], self.target_size[1]]),
            'ori_size': torch.as_tensor([orig_h, orig_w]),
        }

        return image, target

    def get_image_info(self, img_id):
        """获取指定图像的文件名和尺寸信息"""
        return self.images[img_id]


# ===========================================================================
# EfficientDet-D0 模型构建
# ===========================================================================

def build_efficientdet_d0(num_classes=NUM_CLASSES, pretrained=True,
                          image_size=256):
    """
    构建 EfficientDet-D0 检测模型。

    使用 effdet 库，采用 EfficientNet-B0 作为骨干网络，
    BiFPN 特征金字塔和检测头。

    参数:
        num_classes: 目标类别数（含背景，实际传入 3）
        pretrained: 是否加载 COCO 预训练权重
        image_size: 输入图像尺寸（宽和高相同，需被 128 整除）

    返回:
        model: EfficientDet 模型
    """
    try:
        from effdet import create_model_from_config
        from effdet.config import get_efficientdet_config
    except ImportError:
        logger.error("effdet 库未安装，请运行: pip install effdet")
        sys.exit(1)

    # 从配置文件读取默认结构，覆盖 image_size 和 num_classes
    config = get_efficientdet_config('tf_efficientdet_d0')
    config.image_size = (image_size, image_size)  # 设置为 (256, 256) 正方形输入
    config.num_classes = num_classes
    # 显式指定骨干网络特征索引（EfficientNet-B0 的输出层索引 2,3,4 对应 P3,P4,P5）
    config.backbone_indices = [2, 3, 4]

    # 创建 EfficientDet-D0 模型
    # 注意: pretrained_backbone 必须显式为 False，否则即使 pretrained=False
    #       骨干网络仍会尝试从 HuggingFace 下载预训练权重
    try:
        model = create_model_from_config(
            config,
            pretrained=pretrained,
            pretrained_backbone=pretrained,
            num_classes=num_classes,
            bench_task='train',
            bench_labeler=True,  # 创建 AnchorLabeler，自动将 bbox/cls 转为 anchor 标签
        )
    except Exception as e:
        if pretrained:
            logger.warning(f"加载预训练权重失败: {e}")
            logger.warning("将使用随机初始化权重（无预训练）继续...")
            model = create_model_from_config(
                config,
                pretrained=False,
                pretrained_backbone=False,
                num_classes=num_classes,
                bench_task='train',
                bench_labeler=True,
            )
        else:
            logger.error(f"模型创建失败: {e}")
            raise

    return model


# ===========================================================================
# 训练器类
# ===========================================================================

class Trainer:
    """
    EfficientDet-D0 训练器

    封装训练循环、验证、模型保存和日志记录。
    """

    def __init__(self, model, device, config):
        """
        参数:
            model: EfficientDet 模型
            device: torch.device
            config: 配置字典
        """
        self.model = model.to(device)
        self.device = device
        self.config = config

        # 优化器
        self.optimizer = optim.AdamW(
            model.parameters(),
            lr=config.get('lr', 1e-4),
            weight_decay=config.get('weight_decay', 1e-4)
        )

        # 学习率调度器（余弦退火）
        self.scheduler = optim.lr_scheduler.CosineAnnealingLR(
            self.optimizer,
            T_max=config.get('epochs', 50),
            eta_min=config.get('lr_min', 1e-6)
        )

        # TensorBoard 日志
        log_dir = config.get('log_dir', './logs')
        os.makedirs(log_dir, exist_ok=True)
        self.writer = SummaryWriter(log_dir)

        # 训练状态
        self.start_epoch = 0
        self.global_step = 0
        self.best_loss = float('inf')
        self.best_epoch = -1

        # 损失历史
        self.loss_history = []

    def train_one_epoch(self, dataloader, epoch):
        """
        训练一个 epoch

        参数:
            dataloader: 训练数据加载器
            epoch: 当前 epoch 编号

        返回:
            avg_loss: 本 epoch 平均损失
        """
        self.model.train()
        total_loss = 0.0
        num_batches = len(dataloader)

        # 进度条
        print_freq = max(1, num_batches // 10)

        for batch_idx, (images, targets) in enumerate(dataloader):
            images = images.to(self.device)

            # target 已经是单个字典格式（collate_fn 已合并），直接移到 GPU
            targets_gpu = {
                'bbox': targets['bbox'].to(self.device),
                'cls': targets['cls'].to(self.device),
                'img_scale': targets['img_scale'].to(self.device),
                'img_size': targets['img_size'].to(self.device),
            }

            # 前向传播
            loss_dict = self.model(images, targets_gpu)

            # effdet 返回 {'loss': scalar, 'class_loss': ..., 'box_loss': ...}
            loss = loss_dict['loss'] if isinstance(loss_dict, dict) else loss_dict

            # 反向传播
            self.optimizer.zero_grad()
            loss.backward()

            # 梯度裁剪
            torch.nn.utils.clip_grad_norm_(self.model.parameters(),
                                          self.config.get('clip_grad', 10.0))
            self.optimizer.step()

            # 记录
            loss_val = loss.item()
            total_loss += loss_val
            self.global_step += 1

            # 记录到 TensorBoard
            self.writer.add_scalar('Loss/train', loss_val, self.global_step)

            # 打印进度
            if (batch_idx + 1) % print_freq == 0 or batch_idx == 0:
                lr_current = self.optimizer.param_groups[0]['lr']
                logger.info(
                    f'Epoch [{epoch}] Batch [{batch_idx+1}/{num_batches}] '
                    f'Loss: {loss_val:.4f} LR: {lr_current:.2e}'
                )

        avg_loss = total_loss / num_batches
        logger.info(f'Epoch [{epoch}] 平均损失: {avg_loss:.4f}')

        return avg_loss

    @torch.no_grad()
    def validate(self, dataloader, epoch):
        """
        验证集评估

        参数:
            dataloader: 验证数据加载器
            epoch: 当前 epoch

        返回:
            avg_loss: 验证集平均损失
        """
        self.model.eval()
        total_loss = 0.0
        num_batches = len(dataloader)

        for batch_idx, (images, targets) in enumerate(dataloader):
            images = images.to(self.device)

            # 合并成单个字典并移到 GPU
            targets_gpu = {
                'bbox': targets['bbox'].to(self.device),
                'cls': targets['cls'].to(self.device),
                'img_scale': targets['img_scale'].to(self.device),
                'img_size': targets['img_size'].to(self.device),
            }

            # 前向传播（验证模式会同时输出 loss 和 detections）
            output = self.model(images, targets_gpu)

            # 只取标量 loss（忽略 detections 等非标量输出）
            loss = output['loss'] if isinstance(output, dict) else output
            if isinstance(loss, torch.Tensor) and loss.dim() > 0:
                loss_val = loss.mean().item()
            else:
                loss_val = loss.item() if isinstance(loss, torch.Tensor) else float(loss)

            total_loss += loss_val

        avg_loss = total_loss / max(num_batches, 1)

        # 记录到 TensorBoard
        self.writer.add_scalar('Loss/val', avg_loss, epoch)

        logger.info(f'验证集 | Epoch [{epoch}] 平均损失: {avg_loss:.4f}')

        return avg_loss

    def save_checkpoint(self, epoch, loss, is_best=False, save_dir='./checkpoints'):
        """
        保存模型检查点

        参数:
            epoch: 当前 epoch
            loss: 当前损失
            is_best: 是否为最佳模型
            save_dir: 保存目录
        """
        os.makedirs(save_dir, exist_ok=True)

        checkpoint = {
            'epoch': epoch,
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'scheduler_state_dict': self.scheduler.state_dict(),
            'loss': loss,
            'config': self.config,
        }

        # 定期保存
        save_interval = self.config.get('save_interval', 5)
        if epoch % save_interval == 0:
            filename = f'efficientdet_d0_epoch{epoch}.pth'
            filepath = os.path.join(save_dir, filename)
            torch.save(checkpoint, filepath)
            logger.info(f'检查点已保存: {filepath}')

        # 最佳模型
        if is_best:
            filepath = os.path.join(save_dir, 'efficientdet_d0_best.pth')
            torch.save(checkpoint, filepath)
            logger.info(f'最佳模型已保存: {filepath}')

    def fit(self, train_loader, val_loader=None, num_epochs=50, save_dir='./checkpoints'):
        """
        完整训练流程

        参数:
            train_loader: 训练数据加载器
            val_loader: 验证数据加载器（可选）
            num_epochs: 训练 epoch 数
            save_dir: 模型保存目录
        """
        logger.info(f'开始训练，共 {num_epochs} 个 epoch')
        logger.info(f'训练集大小: {len(train_loader.dataset)}')
        if val_loader:
            logger.info(f'验证集大小: {len(val_loader.dataset)}')

        for epoch in range(self.start_epoch, num_epochs):
            epoch_start = time.time()

            # 训练一个 epoch
            train_loss = self.train_one_epoch(train_loader, epoch)
            self.loss_history.append(train_loss)

            # 验证
            if val_loader is not None:
                val_loss = self.validate(val_loader, epoch)
            else:
                val_loss = train_loss

            # 更新学习率
            self.scheduler.step()

            # 检查是否为最佳
            is_best = val_loss < self.best_loss
            if is_best:
                self.best_loss = val_loss
                self.best_epoch = epoch

            # 保存检查点
            self.save_checkpoint(epoch, val_loss, is_best=is_best, save_dir=save_dir)

            epoch_time = time.time() - epoch_start
            logger.info(
                f'Epoch [{epoch}] 完成 | 耗时: {epoch_time:.1f}s | '
                f'Train Loss: {train_loss:.4f} | Val Loss: {val_loss:.4f} | '
                f'LR: {self.optimizer.param_groups[0]["lr"]:.2e}'
            )
            logger.info('-' * 60)

        logger.info(f'训练完成！最佳模型: Epoch {self.best_epoch}, Loss: {self.best_loss:.4f}')
        self.writer.close()


# ===========================================================================
# 辅助函数
# ===========================================================================

def collate_fn(batch):
    """
    自定义批处理函数。

    EfficientDet 的 DetBenchTrain 要求 targets 合并为单个字典，
    其中 bbox 和 cls 为带 batch 维度的填充张量。
    """
    images = torch.stack([item[0] for item in batch], dim=0)

    # 收集每张图像的 target 字段
    all_boxes = [item[1]['bbox'] for item in batch]
    all_cls = [item[1]['cls'] for item in batch]
    all_img_scale = [item[1]['img_scale'] for item in batch]
    all_img_size = [item[1]['img_size'] for item in batch]

    # 找出当前 batch 中最大的目标数，用于填充
    max_boxes = max(b.shape[0] for b in all_boxes)
    batch_size = len(batch)
    num_classes_item = NUM_CLASSES

    # 填充 bbox 和 cls 到相同大小
    padded_boxes = torch.zeros((batch_size, max_boxes, 4), dtype=torch.float32)
    padded_cls = torch.full((batch_size, max_boxes), fill_value=-1, dtype=torch.long)

    for i, (boxes, cls) in enumerate(zip(all_boxes, all_cls)):
        num = boxes.shape[0]
        if num > 0:
            padded_boxes[i, :num] = boxes
            padded_cls[i, :num] = cls

    # 合并到单个字典（DetBenchTrain 要求）
    targets = {
        'bbox': padded_boxes,
        'cls': padded_cls,
        'img_scale': torch.stack(all_img_scale),
        'img_size': torch.stack(all_img_size),
    }

    return images, targets


def get_default_config():
    """获取默认训练配置"""
    return {
        'lr': 1e-4,
        'lr_min': 1e-6,
        'weight_decay': 1e-4,
        'clip_grad': 10.0,
        'batch_size': 4,
        'num_workers': 2,
        'epochs': 50,
        'save_interval': 5,
        'log_dir': './logs',
        'checkpoint_dir': './checkpoints',
        'pretrained': True,
    }


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description='EfficientDet-D0 训练 - 车载雷达 RD 图谱目标检测'
    )
    parser.add_argument('--annotations', type=str, default=DEFAULT_ANNOTATIONS_FILE,
                        help=f'COCO 标注 JSON 路径 (默认: {DEFAULT_ANNOTATIONS_FILE})')
    parser.add_argument('--image-dir', type=str, default=DEFAULT_IMAGE_DIR,
                        help=f'图像目录 (默认: {DEFAULT_IMAGE_DIR})')
    parser.add_argument('--val-annotations', type=str, default=None,
                        help='验证集标注 JSON (可选)')
    parser.add_argument('--val-image-dir', type=str, default=None,
                        help='验证集图像目录 (可选)')
    parser.add_argument('--epochs', type=int, default=50,
                        help='训练 epoch 数 (默认: 50)')
    parser.add_argument('--batch-size', type=int, default=4,
                        help='批大小 (默认: 4)')
    parser.add_argument('--lr', type=float, default=1e-4,
                        help='初始学习率 (默认: 1e-4)')
    parser.add_argument('--save-dir', type=str, default='./checkpoints',
                        help='模型保存目录 (默认: ./checkpoints)')
    parser.add_argument('--log-dir', type=str, default='./logs',
                        help='TensorBoard 日志目录 (默认: ./logs)')
    parser.add_argument('--resume', type=str, default=None,
                        help='恢复训练的检查点路径')
    parser.add_argument('--no-pretrain', action='store_true',
                        help='不使用 COCO 预训练权重')
    parser.add_argument('--val-split', type=float, default=0.0,
                        help='从训练集中划分验证集比例 (0.0-1.0, 默认: 0.0)')
    return parser.parse_args()


# ===========================================================================
# 入口函数
# ===========================================================================

def main():
    """主训练入口"""
    args = parse_args()

    # 设备配置
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    logger.info(f'使用设备: {device}')
    if device.type == 'cuda':
        logger.info(f'GPU: {torch.cuda.get_device_name(0)}')
        logger.info(f'显存: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB')

    # 训练配置
    config = get_default_config()
    config.update({
        'epochs': args.epochs,
        'batch_size': args.batch_size,
        'lr': args.lr,
        'pretrained': not args.no_pretrain,
        'checkpoint_dir': args.save_dir,
        'log_dir': args.log_dir,
    })

    # 创建保存目录
    os.makedirs(config['checkpoint_dir'], exist_ok=True)
    os.makedirs(config['log_dir'], exist_ok=True)

    # 加载数据集
    logger.info('加载训练集...')
    full_dataset = RadarRDCOCODataset(
        annotation_file=args.annotations,
        image_dir=args.image_dir,
    )

    # 划分训练集和验证集
    val_split = args.val_split
    if val_split > 0:
        val_size = int(len(full_dataset) * val_split)
        train_size = len(full_dataset) - val_size
        train_dataset, val_dataset = torch.utils.data.random_split(
            full_dataset, [train_size, val_size],
            generator=torch.Generator().manual_seed(42)
        )
        logger.info(f'数据集划分: 训练集 {train_size} 张, 验证集 {val_size} 张')
    else:
        train_dataset = full_dataset
        val_dataset = None

    # 数据加载器
    train_loader = DataLoader(
        train_dataset,
        batch_size=config['batch_size'],
        shuffle=True,
        num_workers=min(config['num_workers'], os.cpu_count() or 1),
        collate_fn=collate_fn,
        pin_memory=(device.type == 'cuda'),
    )

    val_loader = None
    if val_dataset is not None:
        val_loader = DataLoader(
            val_dataset,
            batch_size=config['batch_size'],
            shuffle=False,
            num_workers=min(config['num_workers'], os.cpu_count() or 1),
            collate_fn=collate_fn,
            pin_memory=(device.type == 'cuda'),
        )

    # 支持独立的验证集标注文件
    if args.val_annotations is not None and args.val_image_dir is not None:
        val_dataset = RadarRDCOCODataset(
            annotation_file=args.val_annotations,
            image_dir=args.val_image_dir,
        )
        val_loader = DataLoader(
            val_dataset,
            batch_size=config['batch_size'],
            shuffle=False,
            num_workers=min(config['num_workers'], os.cpu_count() or 1),
            collate_fn=collate_fn,
            pin_memory=(device.type == 'cuda'),
        )

    # 构建模型
    logger.info('构建 EfficientDet-D0 模型...')
    model = build_efficientdet_d0(
        num_classes=NUM_CLASSES,
        pretrained=config['pretrained'],
    )

    # 模型参数量统计
    total_params = sum(p.numel() for p in model.parameters())
    trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    logger.info(f'模型参数量: {total_params:,} (可训练: {trainable_params:,})')

    # 创建训练器
    trainer = Trainer(model, device, config)

    # 恢复检查点
    if args.resume:
        if os.path.isfile(args.resume):
            logger.info(f'恢复检查点: {args.resume}')
            checkpoint = torch.load(args.resume, map_location=device)
            model.load_state_dict(checkpoint['model_state_dict'])
            trainer.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
            trainer.scheduler.load_state_dict(checkpoint['scheduler_state_dict'])
            trainer.start_epoch = checkpoint['epoch'] + 1
            trainer.best_loss = checkpoint.get('loss', float('inf'))
            logger.info(f'从 Epoch {checkpoint["epoch"]} 恢复训练')
        else:
            logger.warning(f'检查点未找到: {args.resume}')

    # 开始训练
    trainer.fit(
        train_loader=train_loader,
        val_loader=val_loader,
        num_epochs=config['epochs'],
        save_dir=config['checkpoint_dir'],
    )

    logger.info('训练完成！')

    # 保存最终模型（epoch 数命名的副本）
    final_path = os.path.join(
        config['checkpoint_dir'],
        f'efficientdet_d0_epoch{config["epochs"]}.pth'
    )
    torch.save({
        'model_state_dict': model.state_dict(),
    }, final_path)
    logger.info(f'最终模型已保存: {final_path}')


if __name__ == '__main__':
    main()
