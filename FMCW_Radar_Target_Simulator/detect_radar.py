# -*- coding: utf-8 -*-
"""
EfficientDet-D0 RD 图谱目标检测推理脚本

加载训练好的权重，对单张或多张 RD 图谱进行推理，
输出目标检测结果（类别、边界框、置信度）。

用法:
    # 单张图片推理
    python detect_radar.py --weights checkpoints/efficientdet_d0_best.pth \
                           --image COCO/images/train2017/1.jpg

    # 批量推理并保存结果
    python detect_radar.py --weights checkpoints/efficientdet_d0_epoch50.pth \
                           --image-dir ./test_images/ \
                           --output ./results/
"""

import os
import sys
import argparse
import json
import logging
from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader

logging.basicConfig(level=logging.INFO, format='%(asctime)s | %(levelname)s | %(message)s')
logger = logging.getLogger(__name__)

# 类别映射（与训练时一致）
COCO_CATEGORIES = {0: 'Vehicle', 1: 'Pedestrian', 2: 'Bicycle'}
NUM_CLASSES = 3

# RD 图谱物理参数（用于可视化）
FMCW_L = 256          # 速度门数
FMCW_K = 320          # 2 × 距离门数
SWEEP_BW = 1e9
CHIRP_INTERVAL = 64e-6
C_0 = 299792458
F0 = 76.5e9
FMCW_DR = C_0 / (2 * SWEEP_BW)        # 距离分辨率 ≈ 0.15 m
FMCW_DV = 1 / (FMCW_L * CHIRP_INTERVAL) * C_0 / (2 * F0)  # 速度分辨率 ≈ 0.127 m/s


def build_model(num_classes=NUM_CLASSES, checkpoint_path=None):
    """
    构建模型并加载权重。

    参数:
        num_classes: 类别数
        checkpoint_path: 权重文件路径 (.pth)

    返回:
        model: EfficientDet 模型（评估模式）
        device: torch 设备
    """
    try:
        from effdet import create_model_from_config
        from effdet.config import get_efficientdet_config
    except ImportError:
        logger.error("effdet 未安装: pip install effdet")
        sys.exit(1)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    logger.info(f'使用设备: {device}')

    # 构建模型（与训练时的配置一致）
    config = get_efficientdet_config('tf_efficientdet_d0')
    config.image_size = (256, 256)
    config.num_classes = num_classes
    config.backbone_indices = [2, 3, 4]

    model = create_model_from_config(
        config,
        pretrained=False,
        pretrained_backbone=False,
        num_classes=num_classes,
        bench_task='predict',  # 推理模式
    )

    # 加载权重
    if checkpoint_path and os.path.isfile(checkpoint_path):
        logger.info(f'加载权重: {checkpoint_path}')
        checkpoint = torch.load(checkpoint_path, map_location=device)
        state_dict = checkpoint.get('model_state_dict', checkpoint)
        model.load_state_dict(state_dict, strict=False)
        logger.info('权重加载成功')
    else:
        logger.warning('未加载权重，使用随机初始化的模型')

    # ⚠️ 重要：从 bench_predict 中取出原始模型供自定义预处理
    # bench_predict 内部封装了 EfficientDet，其 forward 输出
    # 已经是解码后的检测结果（框、分数、类别）
    if hasattr(model, 'model'):
        inner_model = model  # bench_predict 自动处理解码
    else:
        inner_model = model

    model = model.to(device)
    model.eval()

    return model, device


def preprocess_image(image_path, target_size=(256, 256)):
    """
    加载并预处理单张 RD 图谱图像。

    参数:
        image_path: 图像路径
        target_size: (H, W) 模型输入尺寸

    返回:
        image_tensor: [1, 3, H, W] 张量
        orig_size: (H, W) 原始图像尺寸
        image_bgr: BGR 格式的图像（用于可视化）
    """
    # 读取图像
    image_bgr = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise FileNotFoundError(f'无法读取图像: {image_path}')

    orig_h, orig_w = image_bgr.shape[:2]
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    image_float = image_rgb.astype(np.float32) / 255.0

    # 填充至 target_size（与训练一致）
    pad_h = target_size[0] - orig_h
    pad_w = target_size[1] - orig_w
    if pad_h > 0 or pad_w > 0:
        image_float = np.pad(
            image_float,
            ((0, pad_h), (0, pad_w), (0, 0)),
            mode='constant', constant_values=0
        )

    # HWC → CHW → 添加 batch 维度
    image_tensor = torch.as_tensor(image_float).permute(2, 0, 1).unsqueeze(0).contiguous()

    return image_tensor, (orig_h, orig_w), image_bgr


def decode_predictions(raw_output, orig_size, score_threshold=0.3):
    """
    解码 DetBenchPredict 输出，按置信度阈值过滤。

    参数:
        raw_output: DetBenchPredict 返回的 tensor，形状 [batch, max_det, 6]
                    每行: [x1, y1, x2, y2, score, class_id]
        orig_size: (H, W) 原始图像尺寸
        score_threshold: 置信度阈值

    返回:
        results: 字典列表，每个字典包含:
                 - 'bbox': [x1, y1, x2, y2] 填充后坐标
                 - 'bbox_orig': [x1, y1, x2, y2] 原图坐标
                 - 'score': 置信度
                 - 'class_id': 类别 ID
                 - 'class_name': 类别名称
    """
    results = []
    org_h, org_w = orig_size

    if raw_output is None:
        return results

    if isinstance(raw_output, torch.Tensor):
        raw_output = raw_output.cpu().numpy()

    # extract batch=0 results
    if raw_output.ndim == 3:
        raw_output = raw_output[0]
    elif isinstance(raw_output, (list, tuple)) and len(raw_output) > 0:
        raw_output = raw_output[0]

    if not isinstance(raw_output, np.ndarray) or raw_output.shape[0] == 0:
        return results

    for det in raw_output:
        score = float(det[4])
        if score < score_threshold:
            continue

        class_id = int(det[5])
        x1, y1, x2, y2 = det[:4]

        # 转换为原图坐标（裁剪填充区域）
        x1_orig = max(0, min(x1, org_w))
        y1_orig = max(0, min(y1, org_h))
        x2_orig = max(0, min(x2, org_w))
        y2_orig = max(0, min(y2, org_h))

        results.append({
            'bbox': [float(x1), float(y1), float(x2), float(y2)],
            'bbox_orig': [x1_orig, y1_orig, x2_orig, y2_orig],
            'score': score,
            'class_id': class_id,
            'class_name': COCO_CATEGORIES.get(class_id, f'Unknown({class_id})'),
        })

    # 按置信度降序排列
    results.sort(key=lambda r: r['score'], reverse=True)
    return results


def draw_detections(image, detections, output_path=None):
    """
    在图像上绘制检测结果。

    参数:
        image: BGR 格式图像
        detections: 检测结果列表
        output_path: 保存路径（为 None 时不保存）

    返回:
        image: 绘制后的图像
    """
    img = image.copy()
    colors = {
        0: (0, 255, 0),    # Vehicle → 绿色
        1: (255, 0, 0),    # Pedestrian → 红色
        2: (0, 0, 255),    # Bicycle → 蓝色
    }

    for det in detections:
        x1, y1, x2, y2 = map(int, det['bbox_orig'])
        score = det['score']
        class_name = det['class_name']
        color = colors.get(det['class_id'], (255, 255, 255))

        # 绘制边界框
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

        # 绘制标签背景
        label = f'{class_name} {score:.2f}'
        (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img, (x1, y1 - label_h - 4), (x1 + label_w + 4, y1), color, -1)
        cv2.putText(img, label, (x1 + 2, y1 - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    if output_path:
        cv2.imwrite(output_path, img)
        logger.info(f'检测结果已保存: {output_path}')

    return img


def detect_single_image(model, device, image_path, score_threshold=0.3, output_path=None):
    """
    对单张图像进行检测。

    参数:
        model: EfficientDet 模型
        device: torch 设备
        image_path: 图像路径
        score_threshold: 置信度阈值
        output_path: 结果保存路径

    返回:
        detections: 检测结果列表
    """
    # 预处理
    image_tensor, orig_size, image_bgr = preprocess_image(image_path)
    image_tensor = image_tensor.to(device)

    logger.info(f'图像: {image_path} ({orig_size[1]}x{orig_size[0]})')

    # 推理
    with torch.no_grad():
        # DetBenchPredict 需要 img_info 字典（含 img_scale 和 img_size）
        img_info = {
            'img_scale': torch.tensor([1.0], device=device).unsqueeze(0),
            'img_size': torch.tensor([orig_size[0], orig_size[1]], device=device).unsqueeze(0),
        }
        raw_output = model(image_tensor, img_info)

    # 解码检测结果
    detections = decode_predictions(raw_output, orig_size, score_threshold=score_threshold)

    logger.info(f'检测到 {len(detections)} 个目标:')
    for det in detections:
        logger.info(f'  {det["class_name"]}: '
                    f'[{det["bbox_orig"][0]:.0f}, {det["bbox_orig"][1]:.0f}, '
                    f'{det["bbox_orig"][2]:.0f}, {det["bbox_orig"][3]:.0f}] '
                    f'置信度: {det["score"]:.3f}')

    # 绘制结果
    if output_path:
        os.makedirs(os.path.dirname(output_path) if os.path.dirname(output_path) else '.', exist_ok=True)
        draw_detections(image_bgr, detections, output_path)

        # 同时打印物理坐标
        print_physical_coords(detections)

    return detections


def print_physical_coords(detections):
    """将检测框转换为物理坐标（距离和速度）"""
    print('\n--- 物理坐标 ---')
    print(f'  距离分辨率: {FMCW_DR:.2f} m, 速度分辨率: {FMCW_DV:.2f} m/s')
    for det in detections:
        x1, y1, x2, y2 = det['bbox_orig']
        # 图像坐标: x=速度轴, y=距离轴
        v_center = (x1 + x2) / 2
        r_center = (y1 + y2) / 2
        # 转换为物理量
        range_m = r_center * FMCW_DR
        vel_ms = (v_center - FMCW_L / 2) * FMCW_DV
        range_extent = (y2 - y1) * FMCW_DR
        vel_extent = (x2 - x1) * FMCW_DV
        print(f'  {det["class_name"]}: '
              f'距离={range_m:.1f} m, 速度={vel_ms:.1f} m/s '
              f'(ΔR={range_extent:.1f}m, ΔV={vel_extent:.1f}m/s)')


def detect_batch(model, device, image_dir, output_dir, score_threshold=0.3):
    """批量检测目录中所有图像"""
    image_extensions = ('.jpg', '.jpeg', '.png', '.bmp')
    image_paths = sorted([
        os.path.join(image_dir, f) for f in os.listdir(image_dir)
        if f.lower().endswith(image_extensions)
    ])

    if not image_paths:
        logger.warning(f'{image_dir} 中未找到图像')
        return

    os.makedirs(output_dir, exist_ok=True)
    logger.info(f'批量检测 {len(image_paths)} 张图像...')

    all_results = {}
    for img_path in image_paths:
        basename = os.path.splitext(os.path.basename(img_path))[0]
        out_path = os.path.join(output_dir, f'{basename}_detected.jpg')
        detections = detect_single_image(
            model, device, img_path,
            score_threshold=score_threshold,
            output_path=out_path,
        )
        all_results[basename] = detections

    # 保存 JSON 结果
    json_path = os.path.join(output_dir, 'detections.json')
    json_safe = {}
    for name, dets in all_results.items():
        json_safe[name] = [
            {k: float(v) if isinstance(v, (int, float, np.floating)) else v
             for k, v in d.items()}
            for d in dets
        ]
    with open(json_path, 'w') as f:
        json.dump(json_safe, f, indent=2, ensure_ascii=False)
    logger.info(f'检测结果 JSON 已保存: {json_path}')


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='EfficientDet-D0 RD 图谱目标检测推理')
    parser.add_argument('--weights', type=str, default='./checkpoints/efficientdet_d0_best.pth',
                        help='模型权重路径')
    parser.add_argument('--image', type=str, default=None,
                        help='单张图像路径')
    parser.add_argument('--image-dir', type=str, default=None,
                        help='图像目录（批量推理）')
    parser.add_argument('--output', type=str, default='./results',
                        help='输出目录')
    parser.add_argument('--threshold', type=float, default=0.3,
                        help='置信度阈值 (默认: 0.3)')
    return parser.parse_args()


def main():
    args = parse_args()

    # 检查输入
    if not args.image and not args.image_dir:
        logger.error('请指定 --image 或 --image-dir')
        sys.exit(1)

    # 构建模型
    model, device = build_model(
        num_classes=NUM_CLASSES,
        checkpoint_path=args.weights,
    )

    # 推理
    if args.image:
        output_path = os.path.join(args.output, 'detected_' + os.path.basename(args.image))
        detect_single_image(
            model, device, args.image,
            score_threshold=args.threshold,
            output_path=output_path,
        )
    elif args.image_dir:
        detect_batch(
            model, device, args.image_dir, args.output,
            score_threshold=args.threshold,
        )


if __name__ == '__main__':
    main()
