#!/usr/bin/env python3

import argparse
import json
import math
import sys
import time

import cv2
import numpy as np
import xir
import vart


MODEL_W = 480
MODEL_H = 360

MEAN_BGR = np.array([104.0, 117.0, 123.0], dtype=np.float32)
SCALE_BGR = np.array([1.0, 1.0, 1.0], dtype=np.float32)

LABELS = {
    0: "background",
    1: "car",
    2: "bicycle",
    3: "person",
}

CLASS_THRESHOLDS = {
    1: 0.6,
    2: 0.4,
    3: 0.3,
}

NMS_THRESHOLD = 0.4
KEEP_TOP_K = 200

PRIOR_LAYERS = [
    {"layer_w": 60, "layer_h": 45, "min": 21.0, "max": 45.0, "ars": [2.0], "step_w": 8.0, "step_h": 8.0},
    {"layer_w": 30, "layer_h": 23, "min": 45.0, "max": 99.0, "ars": [2.0, 3.0], "step_w": 16.0, "step_h": 16.0},
    {"layer_w": 15, "layer_h": 12, "min": 99.0, "max": 153.0, "ars": [2.0, 3.0], "step_w": 32.0, "step_h": 32.0},
    {"layer_w": 8, "layer_h": 6, "min": 153.0, "max": 207.0, "ars": [2.0, 3.0], "step_w": 64.0, "step_h": 64.0},
    {"layer_w": 6, "layer_h": 4, "min": 207.0, "max": 261.0, "ars": [2.0], "step_w": 100.0, "step_h": 100.0},
    {"layer_w": 4, "layer_h": 2, "min": 261.0, "max": 315.0, "ars": [2.0], "step_w": 300.0, "step_h": 300.0},
]


def get_dpu_subgraphs(graph):
    root = graph.get_root_subgraph()
    children = root.toposort_child_subgraph()
    return [
        sg for sg in children
        if sg.has_attr("device") and str(sg.get_attr("device")).upper() == "DPU"
    ]


def tensor_shape(tensor):
    if hasattr(tensor, "dims"):
        return tuple(tensor.dims)
    if hasattr(tensor, "get_shape"):
        return tuple(tensor.get_shape())
    raise RuntimeError("Cannot get tensor shape")


def tensor_fix_point(tensor):
    if tensor.has_attr("fix_point"):
        return int(tensor.get_attr("fix_point"))
    return 0


def generate_priors():
    priors = []

    for layer in PRIOR_LAYERS:
        layer_w = layer["layer_w"]
        layer_h = layer["layer_h"]
        min_size = layer["min"]
        max_size = layer["max"]
        aspect_ratios = layer["ars"]
        step_w = layer["step_w"]
        step_h = layer["step_h"]

        for y in range(layer_h):
            for x in range(layer_w):
                cx = (x + 0.5) * step_w / MODEL_W
                cy = (y + 0.5) * step_h / MODEL_H

                w = min_size / MODEL_W
                h = min_size / MODEL_H
                priors.append([cx, cy, w, h])

                size = math.sqrt(min_size * max_size)
                w = size / MODEL_W
                h = size / MODEL_H
                priors.append([cx, cy, w, h])

                for ar in aspect_ratios:
                    ar_sqrt = math.sqrt(ar)

                    w = min_size * ar_sqrt / MODEL_W
                    h = min_size / ar_sqrt / MODEL_H
                    priors.append([cx, cy, w, h])

                    w = min_size / ar_sqrt / MODEL_W
                    h = min_size * ar_sqrt / MODEL_H
                    priors.append([cx, cy, w, h])

    priors = np.array(priors, dtype=np.float32)
    if priors.shape[0] != 16436:
        raise RuntimeError(f"Prior count mismatch: {priors.shape[0]} != 16436")
    return priors


def softmax(x, axis=1):
    x = x - np.max(x, axis=axis, keepdims=True)
    e = np.exp(x)
    return e / np.sum(e, axis=axis, keepdims=True)


def decode_boxes(loc, priors):
    variances = [0.1, 0.1, 0.2, 0.2]

    cx = priors[:, 0] + loc[:, 0] * variances[0] * priors[:, 2]
    cy = priors[:, 1] + loc[:, 1] * variances[1] * priors[:, 3]
    w = priors[:, 2] * np.exp(np.clip(loc[:, 2] * variances[2], -20.0, 20.0))
    h = priors[:, 3] * np.exp(np.clip(loc[:, 3] * variances[3], -20.0, 20.0))

    boxes = np.zeros((loc.shape[0], 4), dtype=np.float32)
    boxes[:, 0] = cx - w / 2.0
    boxes[:, 1] = cy - h / 2.0
    boxes[:, 2] = cx + w / 2.0
    boxes[:, 3] = cy + h / 2.0

    return np.clip(boxes, 0.0, 1.0)


def iou(box, boxes):
    x1 = np.maximum(box[0], boxes[:, 0])
    y1 = np.maximum(box[1], boxes[:, 1])
    x2 = np.minimum(box[2], boxes[:, 2])
    y2 = np.minimum(box[3], boxes[:, 3])

    inter_w = np.maximum(0.0, x2 - x1)
    inter_h = np.maximum(0.0, y2 - y1)
    inter = inter_w * inter_h

    area_box = np.maximum(0.0, box[2] - box[0]) * np.maximum(0.0, box[3] - box[1])
    area_boxes = np.maximum(0.0, boxes[:, 2] - boxes[:, 0]) * np.maximum(0.0, boxes[:, 3] - boxes[:, 1])

    union = area_box + area_boxes - inter
    return inter / np.maximum(union, 1e-9)


def nms(boxes, scores, threshold):
    if len(boxes) == 0:
        return []

    order = np.argsort(scores)[::-1]
    keep = []

    while order.size > 0:
        idx = order[0]
        keep.append(idx)

        if order.size == 1:
            break

        rest = order[1:]
        overlaps = iou(boxes[idx], boxes[rest])
        order = rest[overlaps <= threshold]

    return keep


def read_exact(stream, size):
    chunks = bytearray()
    while len(chunks) < size:
        chunk = stream.read(size - len(chunks))
        if not chunk:
            return None
        chunks.extend(chunk)
    return bytes(chunks)


class VitisAiWorker:
    def __init__(self, model_path, softmax_mode, log_file=None):
        self.model_path = model_path
        self.softmax_mode = softmax_mode
        self.log_file = log_file
        self.last_timing = None
        self.input_lut = None
        self.log("worker init")
        self.priors = generate_priors()

        # Pre-filter thresholds (used to skip background priors before the
        # expensive softmax/decode/NMS; see postprocess()).
        self.min_class_threshold = min(CLASS_THRESHOLDS.values())
        # When conf are logits (softmax applied here), prob_c >= t_c implies
        # z_c - z_background >= logit(t_c). This is a safe necessary condition.
        # Subtract a small epsilon so floating-point borderline cases are never
        # dropped (they get cut later by the exact per-class threshold).
        _eps = 1e-3
        self.fg_logit_thresholds = np.array(
            [
                math.log(CLASS_THRESHOLDS[c] / (1.0 - CLASS_THRESHOLDS[c])) - _eps
                for c in (1, 2, 3)
            ],
            dtype=np.float32,
        )

        self.load_model()

    def log(self, message):
        if not self.log_file:
            return
        with open(self.log_file, "a", encoding="utf-8") as f:
            f.write(message + "\n")
            f.flush()

    def load_model(self):
        self.log(f"load_model start: {self.model_path}")
        self.graph = xir.Graph.deserialize(self.model_path)
        self.dpu_subgraphs = get_dpu_subgraphs(self.graph)

        if not self.dpu_subgraphs:
            raise RuntimeError("No DPU subgraph found in xmodel")

        self.log("create_runner start")
        # Keep graph/subgraph objects on self for the runner lifetime. On this
        # Vitis-AI Python runtime, execute_async can segfault if the runner is
        # used after the XIR graph/subgraph wrapper objects have been released.
        self.runner = vart.Runner.create_runner(self.dpu_subgraphs[0], "run")
        self.log("create_runner done")
        self.input_tensors = self.runner.get_input_tensors()
        self.output_tensors = self.runner.get_output_tensors()

        self.input_tensor = self.input_tensors[0]
        self.input_shape = tensor_shape(self.input_tensor)
        self.input_fix = tensor_fix_point(self.input_tensor)
        self.build_input_lut()

        self.conf_output_index = None
        self.loc_output_index = None
        self.conf_fix = None
        self.loc_fix = None

        for i, tensor in enumerate(self.output_tensors):
            name = tensor.name
            if "mbox_conf_reshape_fix" in name:
                self.conf_output_index = i
                self.conf_fix = tensor_fix_point(tensor)
            elif "mbox_loc_fixed" in name:
                self.loc_output_index = i
                self.loc_fix = tensor_fix_point(tensor)

        if self.conf_output_index is None:
            raise RuntimeError("Cannot find output tensor: mbox_conf_reshape_fix")
        if self.loc_output_index is None:
            raise RuntimeError("Cannot find output tensor: mbox_loc_fixed")

        self.output_data = [
            np.empty(tensor_shape(tensor), dtype=np.int8, order="C")
            for tensor in self.output_tensors
        ]
        self.log(f"load_model done: input_shape={self.input_shape}")

    def detect(self, image, output_width=None, output_height=None):
        self.log(f"detect start: image_shape={image.shape}, dtype={image.dtype}")
        t_start = time.perf_counter()
        image_h, image_w = image.shape[:2]
        if output_width is None:
            output_width = image_w
        if output_height is None:
            output_height = image_h
        input_data = self.preprocess_image(image)
        t_pre = time.perf_counter()
        self.log(
            f"preprocess done: input_shape={input_data.shape}, "
            f"dtype={input_data.dtype}, "
            f"c_contiguous={input_data.flags['C_CONTIGUOUS']}"
        )

        for i, data in enumerate(self.output_data):
            self.log(
                f"output[{i}] buffer: name={self.output_tensors[i].name}, "
                f"shape={data.shape}, dtype={data.dtype}, "
                f"c_contiguous={data.flags['C_CONTIGUOUS']}"
            )
        self.log("output buffers allocated")

        self.log("execute_async start")
        job_id = self.runner.execute_async([input_data], self.output_data)
        self.log("execute_async submitted")
        self.runner.wait(job_id)
        t_dpu = time.perf_counter()
        self.log("runner wait done")

        conf_raw = self.output_data[self.conf_output_index]
        loc_raw = self.output_data[self.loc_output_index]
        self.log("raw outputs selected")

        conf = conf_raw.reshape(-1, 4).astype(np.float32) / float(2 ** self.conf_fix)
        loc_q = loc_raw.reshape(-1, 4)  # keep int8; dequantize only candidates

        if conf.shape != (16436, 4):
            raise RuntimeError(f"Unexpected conf shape: {conf.shape}")
        if loc_q.shape != (16436, 4):
            raise RuntimeError(f"Unexpected loc shape: {loc_q.shape}")

        detections = self.postprocess(
            conf,
            loc_q,
            int(output_width),
            int(output_height),
        )
        t_post = time.perf_counter()
        self.last_timing = {
            "pre_ms": (t_pre - t_start) * 1e3,
            "dpu_ms": (t_dpu - t_pre) * 1e3,
            "post_ms": (t_post - t_dpu) * 1e3,
            "worker_ms": (t_post - t_start) * 1e3,
        }
        self.log("postprocess done")

        return detections

    def build_input_lut(self):
        # Precompute the int8 preprocessing result for every possible uint8 input
        # value, per channel. The transform depends only on (value, channel), and
        # the input is uint8 (256 values), so we can replace the per-frame float
        # math with a table lookup. This is numerically identical to the original
        # formula below (same mean/scale/round/clip), just precomputed once:
        #     int8 = clip(round((v - mean[c]) * scale[c] * 2**input_fix), -128, 127)
        scale = float(2 ** self.input_fix)
        values = np.arange(256, dtype=np.float32)
        lut = np.empty((3, 256), dtype=np.int8)
        for c in range(3):
            t = (values - MEAN_BGR[c]) * SCALE_BGR[c] * scale
            t = np.round(t)
            t = np.clip(t, -128, 127)
            lut[c] = t.astype(np.int8)
        self.input_lut = lut

    def preprocess_image(self, image):
        if image.shape[1] == MODEL_W and image.shape[0] == MODEL_H:
            resized = image
        else:
            resized = cv2.resize(image, (MODEL_W, MODEL_H))
        if resized.dtype != np.uint8:
            resized = resized.astype(np.uint8)

        # LUT lookup per channel (no float math, no per-frame temporaries).
        arr = np.empty((MODEL_H, MODEL_W, 3), dtype=np.int8)
        arr[:, :, 0] = self.input_lut[0][resized[:, :, 0]]
        arr[:, :, 1] = self.input_lut[1][resized[:, :, 1]]
        arr[:, :, 2] = self.input_lut[2][resized[:, :, 2]]

        arr = np.ascontiguousarray(arr[np.newaxis, ...], dtype=np.int8)

        if tuple(arr.shape) != self.input_shape:
            raise RuntimeError(
                f"Input shape mismatch: image={arr.shape}, tensor={self.input_shape}"
            )

        return arr

    def _should_softmax(self, conf):
        if self.softmax_mode == "yes":
            return True
        if self.softmax_mode == "no":
            return False
        if self.softmax_mode == "auto":
            return float(np.max(conf)) > 1.2 or float(np.min(conf)) < -0.01
        raise RuntimeError(f"Unknown softmax_mode: {self.softmax_mode}")

    def postprocess(self, conf, loc_q, image_w, image_h):
        # Most of the 16436 priors are background. Keep only priors that could
        # possibly pass a foreground class threshold, then run softmax / decode /
        # NMS on that small subset. The filter is a safe necessary condition, so
        # the result is identical to scoring every prior.
        if self._should_softmax(conf):
            # conf are logits: prob_c >= t_c  =>  z_c - z_background >= logit(t_c)
            margin = conf[:, 1:4] - conf[:, 0:1]
            candidate = (margin >= self.fg_logit_thresholds).any(axis=1)
            idx = np.nonzero(candidate)[0]
            if idx.size == 0:
                return []
            scores = softmax(conf[idx], axis=1)
        else:
            # conf are already probabilities
            candidate = (conf[:, 1:4] >= self.min_class_threshold).any(axis=1)
            idx = np.nonzero(candidate)[0]
            if idx.size == 0:
                return []
            scores = conf[idx]

        loc = loc_q[idx].astype(np.float32) / float(2 ** self.loc_fix)
        boxes = decode_boxes(loc, self.priors[idx])
        return self.make_detections(scores, boxes, image_w, image_h)

    def make_detections(self, conf, boxes, image_w, image_h):
        detections = []

        for class_id, threshold in CLASS_THRESHOLDS.items():
            scores = conf[:, class_id]
            mask = scores >= threshold

            class_boxes = boxes[mask]
            class_scores = scores[mask]

            if class_boxes.shape[0] == 0:
                continue

            keep = nms(class_boxes, class_scores, NMS_THRESHOLD)

            for k in keep:
                box = class_boxes[k]
                score = float(class_scores[k])

                xmin = int(round(box[0] * image_w))
                ymin = int(round(box[1] * image_h))
                xmax = int(round(box[2] * image_w))
                ymax = int(round(box[3] * image_h))

                width = max(0, xmax - xmin)
                height = max(0, ymax - ymin)

                if width <= 0 or height <= 0:
                    continue

                detections.append({
                    "class_id": int(class_id),
                    "class_name": LABELS[class_id],
                    "confidence": score,
                    "center_x": float(xmin + width / 2.0),
                    "center_y": float(ymin + height / 2.0),
                    "width": float(width),
                    "height": float(height),
                })

        detections.sort(key=lambda item: item["confidence"], reverse=True)
        return detections[:KEEP_TOP_K]


def write_response(response):
    sys.stdout.buffer.write(json.dumps(response).encode("utf-8") + b"\n")
    sys.stdout.buffer.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--softmax", choices=["auto", "yes", "no"], default="auto")
    parser.add_argument("--log-file", default=None)
    args = parser.parse_args()

    if args.log_file:
        with open(args.log_file, "w", encoding="utf-8") as f:
            f.write("worker log start\n")

    worker = VitisAiWorker(args.model, args.softmax, args.log_file)
    write_response({
        "status": "ready",
        "input_shape": list(worker.input_shape),
    })
    worker.log("ready sent")

    while True:
        line = sys.stdin.buffer.readline()
        if not line:
            worker.log("stdin closed")
            break

        try:
            worker.log("request header received")
            request = json.loads(line.decode("utf-8"))
            data = read_exact(sys.stdin.buffer, int(request["data_len"]))
            if data is None:
                worker.log("request data missing")
                break

            height = int(request["height"])
            width = int(request["width"])
            channels = int(request["channels"])
            worker.log(
                f"request data received: width={width}, height={height}, "
                f"channels={channels}, data_len={len(data)}"
            )

            image = np.frombuffer(data, dtype=np.uint8).reshape(
                (height, width, channels)
            )
            worker.log("image reshaped")

            source_width = int(request.get("source_width", width))
            source_height = int(request.get("source_height", height))
            detections = worker.detect(
                image,
                output_width=source_width,
                output_height=source_height,
            )
            worker.log(f"detections done: count={len(detections)}")
            write_response({
                "detections": detections,
                "timing": worker.last_timing,
                "error": None,
            })
            worker.log("response sent")
        except Exception as exc:
            if "worker" in locals():
                worker.log(f"python exception: {exc}")
            write_response({
                "detections": [],
                "error": str(exc),
            })


if __name__ == "__main__":
    main()
