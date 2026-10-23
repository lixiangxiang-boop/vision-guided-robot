"""
Vision Service — 视觉服务层
==========================
提供目标检测与颜色识别的 HTTP API，供 Agent 调用。
基于 YOLOv8 + OpenCV 传统视觉，覆盖工业场景常用需求。

API:
  POST /detect          目标检测 (YOLOv8)
  POST /color           颜色识别 (OpenCV HSV)
  POST /analyze         综合视觉分析 (检测 + 颜色 + 空间关系)
  GET  /health          健康检查
"""
import io
import os
import json
import base64
import logging
from typing import Optional

import cv2
import numpy as np
from PIL import Image
from flask import Flask, request, jsonify
from flask_cors import CORS

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("vision_service")

app = Flask(__name__)
CORS(app)

# ── 模型加载 ──
DEVICE = "cuda" if __import__("torch").cuda.is_available() else "cpu"
yolo_model = None


def get_yolo():
    global yolo_model
    if yolo_model is None:
        try:
            from ultralytics import YOLO
            yolo_model = YOLO("yolov8n.pt")
            logger.info("YOLOv8n loaded on %s", DEVICE)
        except Exception as e:
            logger.warning("YOLO not available: %s, using fallback", e)
    return yolo_model


# ── 颜色定义 ──
COLOR_RANGES = {
    "红色": [((0, 50, 50), (10, 255, 255)), ((170, 50, 50), (180, 255, 255))],
    "橙色": [((11, 50, 50), (25, 255, 255))],
    "黄色": [((26, 50, 50), (35, 255, 255))],
    "绿色": [((36, 50, 50), (85, 255, 255))],
    "蓝色": [((86, 50, 50), (130, 255, 255))],
    "紫色": [((131, 50, 50), (160, 255, 255))],
    "白色": [((0, 0, 200), (180, 30, 255))],
    "黑色": [((0, 0, 0), (180, 255, 50))],
    "灰色": [((0, 0, 50), (180, 30, 200))],
}

COLOR_BGR = {
    "红色": (0, 0, 255), "橙色": (0, 165, 255), "黄色": (0, 255, 255),
    "绿色": (0, 255, 0), "蓝色": (255, 0, 0), "紫色": (255, 0, 255),
    "白色": (255, 255, 255), "黑色": (0, 0, 0), "灰色": (128, 128, 128),
}


def _decode_image(data: dict) -> np.ndarray:
    """从请求中解码图像: b64/file/array 三种格式"""
    if "image_b64" in data:
        img_bytes = base64.b64decode(data["image_b64"])
        return cv2.imdecode(np.frombuffer(img_bytes, np.uint8), cv2.IMREAD_COLOR)
    if "image_path" in data:
        path = data["image_path"]
        if not os.path.exists(path):
            raise FileNotFoundError(f"Image not found: {path}")
        return cv2.imread(path)
    if "image_array" in data:
        arr = np.array(data["image_array"], dtype=np.uint8)
        return arr
    raise ValueError("请提供 image_b64 / image_path / image_array 之一")


def _encode_image(img: np.ndarray) -> str:
    _, buf = cv2.imencode(".jpg", img)
    return base64.b64encode(buf).decode()


# ══════════════════════════════════════════════════════════════
# API: 目标检测
# ══════════════════════════════════════════════════════════════

@app.route("/detect", methods=["POST"])
def detect():
    """
    目标检测接口。
    请求: { "image_b64": "...", "conf": 0.5, "classes": null }
    返回: { "detections": [{ bbox, label, confidence, center }], "count", "annotated_b64" }
    """
    try:
        data = request.get_json(force=True)
        img = _decode_image(data)
        conf = data.get("conf", 0.35)
        classes = data.get("classes", None)

        model = get_yolo()
        if model is None:
            return _fallback_detection(img, conf)

        results = model(img, conf=conf, classes=classes, verbose=False)
        detections = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                cls = int(box.cls[0])
                detections.append({
                    "bbox": [x1, y1, x2, y2],
                    "label": model.names.get(cls, str(cls)),
                    "confidence": float(box.conf[0]),
                    "center": [cx, cy],
                })

        annotated = results[0].plot() if results else img

        return jsonify({
            "success": True,
            "detections": detections,
            "count": len(detections),
            "annotated_b64": _encode_image(annotated),
        })

    except Exception as e:
        logger.exception("detect error")
        return jsonify({"success": False, "error": str(e)}), 400


def _fallback_detection(img: np.ndarray, conf: float) -> tuple:
    """无 YOLO 时的纯 OpenCV 轮廓检测回退方案。
    返回 (dict, status_code) 便于 Flask 路由和无 Flask 上下文测试两种场景使用。"""
    h, w = img.shape[:2]
    # 基于颜色差异的前景分割: 背景为均匀灰色时最有效
    bg_color = np.mean(img.reshape(-1, 3), axis=0)
    diff = np.sqrt(np.sum((img.astype(float) - bg_color) ** 2, axis=2))
    # 固定阈值: 与背景色差异 > 40 视为前景物体
    thresh_val = 40.0
    diff_clipped = np.clip(diff, 0, 255).astype(np.uint8)
    _, fg_mask = cv2.threshold(diff_clipped, thresh_val, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel)
    fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = (w * h) * 0.003

    detections = []
    annotated = img.copy()
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        detections.append({
            "bbox": [float(x), float(y), float(x + bw), float(y + bh)],
            "label": "object",
            "confidence": round(min(area / (w * h) * 10, 0.99), 2),
            "center": [float(x + bw / 2), float(y + bh / 2)],
        })
        cv2.rectangle(annotated, (x, y), (x + bw, y + bh), (0, 255, 0), 2)

    return {
        "success": True,
        "detections": detections,
        "count": len(detections),
        "annotated_b64": _encode_image(annotated),
        "method": "opencv_contour_fallback",
    }, 200


# ══════════════════════════════════════════════════════════════
# API: 颜色识别
# ══════════════════════════════════════════════════════════════

@app.route("/color", methods=["POST"])
def color_detect():
    """
    颜色识别接口。
    请求: { "image_b64": "...", "roi": [x1,y1,x2,y2] (可选) }
    返回: { "dominant_color", "color_percentages", "mask_b64" }
    """
    try:
        data = request.get_json(force=True)
        img = _decode_image(data)
        h, w = img.shape[:2]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        roi = data.get("roi")
        crop_hsv = False
        if roi:
            x1, y1, x2, y2 = map(int, roi)
            # 边界检查：裁剪 roi 到图像范围内
            x1 = max(0, min(x1, w - 1))
            y1 = max(0, min(y1, h - 1))
            x2 = max(x1 + 1, min(x2, w))
            y2 = max(y1 + 1, min(y2, h))
            hsv = hsv[y1:y2, x1:x2]
            crop_hsv = True

        results = {}
        masks = {}
        for name, ranges in COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                lower = np.array(lower)
                upper = np.array(upper)
                mask |= cv2.inRange(hsv, lower, upper)
            pct = float(np.sum(mask > 0) / mask.size * 100)
            results[name] = round(pct, 1)
            masks[name] = mask

        dominant = max(results, key=results.get)

        # 生成可视化 — 仅在不裁剪时，或使用全图 roi 区域
        if crop_hsv:
            mask_h, mask_w = hsv.shape[:2]
            mask_vis = np.zeros((mask_h, mask_w, 3), dtype=np.uint8)
        else:
            mask_vis = np.zeros((h, w, 3), dtype=np.uint8)
        for name, mask in masks.items():
            if results[name] > 1:
                # 确保 mask 与 mask_vis 前两维尺寸一致
                if mask.shape != mask_vis.shape[:2]:
                    mask = cv2.resize(mask.astype(np.uint8),
                                      (mask_vis.shape[1], mask_vis.shape[0]))
                    mask = mask > 0
                mask_vis[mask > 0] = COLOR_BGR.get(name, (255, 255, 255))

        return jsonify({
            "success": True,
            "dominant_color": dominant,
            "confidence": results[dominant],
            "color_percentages": {k: v for k, v in results.items() if v > 0},
            "mask_b64": _encode_image(mask_vis),
        })

    except Exception as e:
        logger.exception("color error")
        return jsonify({"success": False, "error": str(e)}), 400


# ══════════════════════════════════════════════════════════════
# API: 综合视觉分析
# ══════════════════════════════════════════════════════════════

@app.route("/analyze", methods=["POST"])
def analyze():
    """
    综合视觉分析：检测 + 颜色 + 空间关系。
    请求: { "image_b64": "..." }
    返回: { "objects": [{ bbox, label, color, center, size }], "summary", "annotated_b64" }
    """
    try:
        data = request.get_json(force=True)
        img = _decode_image(data)

        # Step 1: 目标检测 (YOLO 优先，无结果时回退到轮廓检测)
        model = get_yolo()
        detections = []
        if model is not None:
            results = model(img, conf=0.35, verbose=False)
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
                    cls = int(box.cls[0])
                    detections.append({
                        "bbox": [x1, y1, x2, y2],
                        "label": model.names.get(cls, str(cls)),
                        "confidence": float(box.conf[0]),
                    })
        if not detections:
            detect_result, code = _fallback_detection(img, 0.35)
            detections = detect_result.get("detections", [])
        det_data = {"detections": detections}

        # Step 2: 对每个检测对象做颜色识别
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        objects = []

        for det in det_data.get("detections", []):
            x1, y1, x2, y2 = map(int, det["bbox"])
            roi_hsv = hsv[y1:y2, x1:x2]

            color_scores = {}
            for name, ranges in COLOR_RANGES.items():
                mask = np.zeros(roi_hsv.shape[:2], dtype=np.uint8)
                for lower, upper in ranges:
                    mask |= cv2.inRange(roi_hsv, np.array(lower), np.array(upper))
                color_scores[name] = float(np.sum(mask > 0) / mask.size * 100)
            dominant_color = max(color_scores, key=color_scores.get)

            w, h_box = x2 - x1, y2 - y1
            objects.append({
                "bbox": det["bbox"],
                "label": det["label"],
                "confidence": det.get("confidence", 0.0),
                "color": dominant_color,
                "color_confidence": color_scores[dominant_color],
                "center": [float((x1 + x2) / 2), float((y1 + y2) / 2)],
                "size": [w, h_box],
            })

        # Step 3: 空间关系分析
        spatial = _analyze_spatial(objects)

        # 生成可视化
        annotated = img.copy()
        for obj in objects:
            x1, y1, x2, y2 = map(int, obj["bbox"])
            color_bgr = COLOR_BGR.get(obj["color"], (0, 255, 0))
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color_bgr, 2)
            label = f"{obj['label']} ({obj['color']})"
            cv2.putText(annotated, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)

        summary = _generate_summary(objects, spatial)

        return jsonify({
            "success": True,
            "objects": objects,
            "count": len(objects),
            "spatial_relations": spatial,
            "summary": summary,
            "annotated_b64": _encode_image(annotated),
        })

    except Exception as e:
        logger.exception("analyze error")
        return jsonify({"success": False, "error": str(e)}), 400


def _analyze_spatial(objects: list) -> dict:
    """分析物体间的空间关系 (左右/上下/前后)"""
    if len(objects) < 2:
        return {}
    relations = []
    for i, a in enumerate(objects):
        for j, b in enumerate(objects):
            if i >= j:
                continue
            ax, ay = a["center"]
            bx, by = b["center"]
            dx, dy = bx - ax, by - ay
            rel = {
                "between": [a["label"] + str(i), b["label"] + str(j)],
                "horizontal": "右侧" if dx > 0 else "左侧",
                "vertical": "下方" if dy > 0 else "上方",
                "distance_px": round(np.sqrt(dx ** 2 + dy ** 2), 1),
            }
            relations.append(rel)
    return {"pairs": relations}


def _generate_summary(objects: list, spatial: dict) -> str:
    """生成场景自然语言描述"""
    if not objects:
        return "场景中未检测到任何物体。"
    parts = []
    for i, obj in enumerate(objects):
        parts.append(f"物体{i + 1}: {obj['color']}的{obj['label']}，"
                     f"位置({obj['center'][0]:.0f}, {obj['center'][1]:.0f})，"
                     f"大小({obj['size'][0]:.0f}x{obj['size'][1]:.0f})")
    if spatial.get("pairs"):
        relations = spatial["pairs"]
        for r in relations:
            a_name = r["between"][0].rstrip("0123456789")
            b_name = r["between"][1].rstrip("0123456789")
            parts.append(f"{a_name}在{b_name}的{r['horizontal']}{r['vertical']}，"
                         f"距离{r['distance_px']:.0f}像素")
    return "；".join(parts)


# ══════════════════════════════════════════════════════════════
# 健康检查
# ══════════════════════════════════════════════════════════════

@app.route("/health", methods=["GET"])
def health():
    model = get_yolo()
    return jsonify({
        "status": "ok",
        "service": "vision",
        "yolo_available": model is not None,
        "device": DEVICE,
    })


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8801)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()
    logger.info("Vision Service starting on port %d", args.port)
    app.run(host="0.0.0.0", port=args.port, debug=args.debug)
