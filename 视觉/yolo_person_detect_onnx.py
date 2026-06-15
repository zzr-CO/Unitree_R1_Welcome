#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLOv8 ONNX person-detection smoke test for R1 videohub images.

这个脚本只做一件事：对一张 R1 videohub 拍到的 JPG 做“人”检测。
它不会连接 DDS，不会调用 Unitree SDK，也不会控制 R1 的任何电机。

为什么要有这个 ONNX 版本:
  - R1 不能联网时，安装 ultralytics + torch 比较麻烦。
  - ONNX 版本只依赖 OpenCV 的 dnn 模块，离线测试更简单。
  - 你只需要把 yolov8n.onnx 和这个脚本放到 R1 同一个目录即可。

你可以改:
  - --model: ONNX 模型文件，例如 yolov8n.onnx
  - --conf: 置信度阈值，越高越保守
  - --min-height-ratio: 人框高度占整张图高度的最小比例，用于过滤太远/太小的人
  - --center-band: 只保留画面中间区域的人，适合迎宾正前方触发

暂时不要改:
  - person 类别 ID = 0，这是 COCO 预训练模型里“人”的类别
  - 这个脚本只做识别，不要在这里直接加动作控制

你应该学会:
  - videohub 负责从 R1 摄像头取图
  - YOLO 负责从图里找人
  - OpenCV dnn 负责在没有 ultralytics 的情况下运行 ONNX 模型
"""

import argparse
from pathlib import Path

import cv2
import numpy as np


PERSON_CLASS_ID = 0
MODEL_INPUT_SIZE = 640


def parse_args():
    parser = argparse.ArgumentParser(
        description="Detect people in one R1 videohub image with YOLOv8 ONNX."
    )
    parser.add_argument("input", help="input JPG path, for example r1_probe_videohub.jpg")
    parser.add_argument(
        "-o",
        "--output",
        default="r1_yolo_person_detect.jpg",
        help="output annotated JPG path",
    )
    parser.add_argument(
        "--model",
        default="yolov8n.onnx",
        help="YOLOv8 ONNX model path, default: yolov8n.onnx",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.35,
        help="confidence threshold",
    )
    parser.add_argument(
        "--nms",
        type=float,
        default=0.45,
        help="NMS threshold",
    )
    parser.add_argument(
        "--min-height-ratio",
        type=float,
        default=0.12,
        help="ignore person boxes shorter than this ratio of image height",
    )
    parser.add_argument(
        "--center-band",
        type=float,
        default=1.0,
        help="horizontal center band ratio in [0,1]; 0.6 means only middle 60%%",
    )
    return parser.parse_args()


def letterbox(image, new_size=MODEL_INPUT_SIZE):
    # 你应该学会: letterbox 是“等比例缩放 + 补边”，避免鱼眼画面被强行拉伸变形。
    height, width = image.shape[:2]
    scale = min(new_size / width, new_size / height)
    resized_width = int(round(width * scale))
    resized_height = int(round(height * scale))

    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    padded = np.full((new_size, new_size, 3), 114, dtype=np.uint8)

    pad_x = (new_size - resized_width) // 2
    pad_y = (new_size - resized_height) // 2
    padded[pad_y : pad_y + resized_height, pad_x : pad_x + resized_width] = resized
    return padded, scale, pad_x, pad_y


def parse_yolov8_output(output, image_shape, scale, pad_x, pad_y, conf_threshold):
    # 暂时不要改: YOLOv8 ONNX 常见输出是 [1, 84, 8400] 或 [1, 8400, 84]。
    predictions = output[0]
    if predictions.shape[0] < predictions.shape[1]:
        predictions = predictions.T

    image_height, image_width = image_shape[:2]
    boxes = []
    scores = []

    for row in predictions:
        class_scores = row[4:]
        class_id = int(np.argmax(class_scores))
        score = float(class_scores[class_id])

        if class_id != PERSON_CLASS_ID or score < conf_threshold:
            continue

        cx, cy, w, h = row[:4]
        x1 = (cx - w / 2 - pad_x) / scale
        y1 = (cy - h / 2 - pad_y) / scale
        x2 = (cx + w / 2 - pad_x) / scale
        y2 = (cy + h / 2 - pad_y) / scale

        x1 = max(0, min(image_width - 1, int(round(x1))))
        y1 = max(0, min(image_height - 1, int(round(y1))))
        x2 = max(0, min(image_width - 1, int(round(x2))))
        y2 = max(0, min(image_height - 1, int(round(y2))))

        box_w = x2 - x1
        box_h = y2 - y1
        if box_w <= 0 or box_h <= 0:
            continue

        boxes.append([x1, y1, box_w, box_h])
        scores.append(score)

    return boxes, scores


def filter_for_greeting_zone(box, image_shape, min_height_ratio, center_band):
    # 你可以改: 这里决定“多近的人才算迎宾对象”。
    image_height, image_width = image_shape[:2]
    x, y, w, h = box

    if h / image_height < min_height_ratio:
        return False

    if center_band < 1.0:
        center_x = x + w / 2
        band_width = image_width * center_band
        left = (image_width - band_width) / 2
        right = (image_width + band_width) / 2
        if center_x < left or center_x > right:
            return False

    return True


def draw_detection(image, box, score, is_trigger_candidate):
    x, y, w, h = box
    color = (0, 220, 0) if is_trigger_candidate else (0, 180, 255)
    label = f"person {score:.2f}"
    if is_trigger_candidate:
        label += " greeting"

    cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
    cv2.putText(
        image,
        label,
        (x, max(20, y - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2,
        cv2.LINE_AA,
    )


def main():
    args = parse_args()
    input_path = Path(args.input)
    model_path = Path(args.model)
    output_path = Path(args.output)

    if not input_path.exists():
        raise FileNotFoundError(f"Input image not found: {input_path}")
    if not model_path.exists():
        raise FileNotFoundError(f"ONNX model not found: {model_path}")

    image = cv2.imread(str(input_path))
    if image is None:
        raise RuntimeError(f"Failed to read image: {input_path}")

    # 暂时不要改: 这里只加载 ONNX 模型，不连接机器人，也不发任何控制指令。
    net = cv2.dnn.readNetFromONNX(str(model_path))

    model_input, scale, pad_x, pad_y = letterbox(image)
    blob = cv2.dnn.blobFromImage(
        model_input,
        scalefactor=1.0 / 255.0,
        size=(MODEL_INPUT_SIZE, MODEL_INPUT_SIZE),
        swapRB=True,
        crop=False,
    )

    net.setInput(blob)
    output = net.forward()

    boxes, scores = parse_yolov8_output(
        output,
        image.shape,
        scale,
        pad_x,
        pad_y,
        args.conf,
    )

    indices = cv2.dnn.NMSBoxes(boxes, scores, args.conf, args.nms)
    indices = np.array(indices).reshape(-1).tolist() if len(indices) > 0 else []

    trigger_count = 0
    for idx in indices:
        box = boxes[idx]
        score = scores[idx]
        is_trigger_candidate = filter_for_greeting_zone(
            box,
            image.shape,
            args.min_height_ratio,
            args.center_band,
        )
        if is_trigger_candidate:
            trigger_count += 1
        draw_detection(image, box, score, is_trigger_candidate)

    cv2.imwrite(str(output_path), image)

    print(f"Input: {input_path}")
    print(f"Model: {model_path}")
    print(f"Output: {output_path}")
    print(f"Detected people: {len(indices)}")
    print(f"Greeting trigger candidates: {trigger_count}")
    print("Greeting trigger candidate:", "YES" if trigger_count > 0 else "NO")


if __name__ == "__main__":
    main()
