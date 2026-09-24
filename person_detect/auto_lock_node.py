#!/usr/bin/env python3
"""
==========================================================================
  AERIAL HUMAN DETECTION — AUTO-LOCK NODE
  Raspberry Pi 5 + Hailo-8 AI HAT (26 TOPS) + Pi Camera
==========================================================================

  Detects humans from an aerial (drone) camera feed using a YOLO .hef
  model on the Hailo-8 accelerator. When a person is detected, the system
  automatically LOCKS onto them and continuously tracks that person
  frame-to-frame.

  FEATURES
  ────────
  • Real-time human detection via Hailo-8 NPU
  • Automatic lock-on to the highest-confidence / most-centered person
  • Continuous tracking with IoU-based association (survives brief occlusion)
  • Lock status overlay with bounding box, trail, and confidence
  • LOCKED / SCANNING / NO HUMAN status output (can be read by any
    external system — Pixhawk, GPIO, UART, MQTT, etc.)
  • Graceful fallback to CPU/OpenCV DNN if Hailo is not available

  USAGE
  ─────
  python auto_lock_node.py \
      --hef models/hailo/yolo26s_visdrone_best.hef \
      --source 0 \
      --imgsz 640 \
      --conf 0.35

  DEPENDENCIES (on Pi 5)
  ──────────────────────
  pip install opencv-python numpy

  HailoRT must be installed separately via the Hailo apt repository:
    sudo apt install hailort hailort-pcie-driver python3-hailort

==========================================================================
"""

import argparse
import time
import sys
import os
from collections import deque

import cv2
import numpy as np

# ──────────────────────────────────────────────────────────────────────
#  Hailo Runtime Import (graceful fallback)
# ──────────────────────────────────────────────────────────────────────
HAILO_AVAILABLE = False
try:
    from hailo_platform import (
        HEF, VDevice, HailoStreamInterface,
        ConfigureParams, InferVStreams,
        InputVStreamParams, OutputVStreamParams, FormatType,
    )
    HAILO_AVAILABLE = True
    print("[INFO] HailoRT loaded successfully.")
except ImportError:
    print("[WARN] HailoRT not found — will use OpenCV DNN fallback (CPU, slower).")


# ──────────────────────────────────────────────────────────────────────
#  UTILITIES
# ──────────────────────────────────────────────────────────────────────
def iou(box_a, box_b):
    """Compute Intersection-over-Union between two [x1,y1,x2,y2] boxes."""
    x1 = max(box_a[0], box_b[0])
    y1 = max(box_a[1], box_b[1])
    x2 = min(box_a[2], box_b[2])
    y2 = min(box_a[3], box_b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
    area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def nms(boxes, scores, iou_threshold=0.45):
    """Non-Maximum Suppression."""
    if len(boxes) == 0:
        return []
    indices = np.argsort(scores)[::-1]
    keep = []
    while len(indices) > 0:
        current = indices[0]
        keep.append(current)
        if len(indices) == 1:
            break
        rest = indices[1:]
        ious = np.array([iou(boxes[current], boxes[r]) for r in rest])
        indices = rest[ious < iou_threshold]
    return keep


def letterbox(img, new_shape=640):
    """Resize image with padding to maintain aspect ratio (YOLO-style)."""
    h, w = img.shape[:2]
    scale = min(new_shape / h, new_shape / w)
    new_h, new_w = int(h * scale), int(w * scale)
    img_resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    # Pad to square
    dw = (new_shape - new_w) // 2
    dh = (new_shape - new_h) // 2
    padded = cv2.copyMakeBorder(img_resized, dh, new_shape - new_h - dh,
                                dw, new_shape - new_w - dw,
                                cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return padded, scale, dw, dh


# ──────────────────────────────────────────────────────────────────────
#  SIMPLE TRACKER (IoU-based, lightweight for edge)
# ──────────────────────────────────────────────────────────────────────
class Track:
    _next_id = 1

    def __init__(self, box, conf):
        self.id = Track._next_id
        Track._next_id += 1
        self.box = box              # [x1, y1, x2, y2]
        self.conf = conf
        self.age = 0                # frames since creation
        self.lost_frames = 0        # consecutive frames without match
        self.trail = deque(maxlen=60)  # center-point trail for visualization
        self._update_center()

    def _update_center(self):
        cx = int((self.box[0] + self.box[2]) / 2)
        cy = int((self.box[1] + self.box[3]) / 2)
        self.trail.append((cx, cy))

    def update(self, box, conf):
        self.box = box
        self.conf = conf
        self.lost_frames = 0
        self.age += 1
        self._update_center()

    def mark_lost(self):
        self.lost_frames += 1
        self.age += 1

    @property
    def center(self):
        cx = (self.box[0] + self.box[2]) / 2
        cy = (self.box[1] + self.box[3]) / 2
        return cx, cy


class SimpleTracker:
    """
    Lightweight IoU-based multi-object tracker designed for edge deployment.
    Good enough for aerial single-class (person) tracking on a drone.
    """
    def __init__(self, iou_threshold=0.3, max_lost=15):
        self.tracks: list[Track] = []
        self.iou_threshold = iou_threshold
        self.max_lost = max_lost   # drop track after N frames lost

    def update(self, detections):
        """
        Args:
            detections: list of [x1, y1, x2, y2, conf]
        Returns:
            list of active Track objects
        """
        if len(detections) == 0:
            for t in self.tracks:
                t.mark_lost()
            self.tracks = [t for t in self.tracks if t.lost_frames <= self.max_lost]
            return self.tracks

        det_boxes = [d[:4] for d in detections]
        det_confs = [d[4] for d in detections]

        matched_det = set()
        matched_trk = set()

        # Match existing tracks to detections by IoU
        if len(self.tracks) > 0:
            iou_matrix = np.zeros((len(self.tracks), len(det_boxes)))
            for ti, trk in enumerate(self.tracks):
                for di, det in enumerate(det_boxes):
                    iou_matrix[ti, di] = iou(trk.box, det)

            # Greedy matching (highest IoU first)
            while True:
                idx = np.unravel_index(np.argmax(iou_matrix), iou_matrix.shape)
                if iou_matrix[idx] < self.iou_threshold:
                    break
                ti, di = idx
                self.tracks[ti].update(det_boxes[di], det_confs[di])
                matched_trk.add(ti)
                matched_det.add(di)
                iou_matrix[ti, :] = 0
                iou_matrix[:, di] = 0

        # Mark unmatched tracks as lost
        for ti, trk in enumerate(self.tracks):
            if ti not in matched_trk:
                trk.mark_lost()

        # Create new tracks for unmatched detections
        for di in range(len(det_boxes)):
            if di not in matched_det:
                self.tracks.append(Track(det_boxes[di], det_confs[di]))

        # Prune dead tracks
        self.tracks = [t for t in self.tracks if t.lost_frames <= self.max_lost]
        return self.tracks


# ──────────────────────────────────────────────────────────────────────
#  HAILO INFERENCE ENGINE
# ──────────────────────────────────────────────────────────────────────
class HailoDetector:
    """Run YOLO detection on the Hailo-8 NPU."""

    def __init__(self, hef_path, conf_threshold=0.35, iou_threshold=0.45, imgsz=640):
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.imgsz = imgsz

        if not HAILO_AVAILABLE:
            raise RuntimeError("HailoRT is required. Install it on the Pi 5.")

        print(f"[INFO] Loading HEF: {hef_path}")
        self.hef = HEF(hef_path)
        self.target = VDevice()

        configure_params = ConfigureParams.create_from_hef(
            hef=self.hef, interface=HailoStreamInterface.PCIe
        )
        self.network_group = self.target.configure(self.hef, configure_params)[0]
        self.network_group_params = self.network_group.create_params()

        # Get input/output metadata
        self.input_vstream_info = self.hef.get_input_vstream_infos()
        self.output_vstream_info = self.hef.get_output_vstream_infos()

        input_shape = self.input_vstream_info[0].shape
        print(f"[INFO] Model input shape: {input_shape}")
        print(f"[INFO] Model outputs: {[o.name for o in self.output_vstream_info]}")
        print(f"[INFO] Hailo detector ready.")

    def detect(self, frame):
        """
        Run detection on a BGR frame.
        Returns: list of [x1, y1, x2, y2, confidence] in original frame coords.
        """
        orig_h, orig_w = frame.shape[:2]
        img, scale, dw, dh = letterbox(frame, self.imgsz)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Prepare input (NHWC uint8 for Hailo quantized models)
        input_data = np.expand_dims(img_rgb, axis=0).astype(np.uint8)

        # Configure vstreams
        input_vstream_params = InputVStreamParams.make(
            self.network_group, quantized=True,
            format_type=FormatType.UINT8
        )
        output_vstream_params = OutputVStreamParams.make(
            self.network_group, quantized=False,
            format_type=FormatType.FLOAT32
        )

        # Infer
        input_dict = {self.input_vstream_info[0].name: input_data}
        with InferVStreams(self.network_group, input_vstream_params, output_vstream_params) as pipeline:
            results = pipeline.infer(input_dict)

        # Post-process: parse YOLO output
        detections = self._postprocess(results, scale, dw, dh, orig_w, orig_h)
        return detections

    def _postprocess(self, results, scale, dw, dh, orig_w, orig_h):
        """Parse raw network output into [x1,y1,x2,y2,conf] detections."""
        detections = []

        # Handle different YOLO output formats
        for name, output in results.items():
            output = np.squeeze(output)  # Remove batch dim

            # Common YOLO output shape: (num_predictions, 5) or (num_predictions, 6)
            # where columns are [cx, cy, w, h, conf] or [cx, cy, w, h, conf, class_conf]
            if output.ndim == 2:
                for row in output:
                    if len(row) >= 5:
                        # Single-class: conf is objectness * class_conf or just objectness
                        if len(row) == 5:
                            cx, cy, w, h, conf = row
                        else:
                            cx, cy, w, h = row[:4]
                            conf = row[4] * row[5]  # obj_conf * cls_conf

                        if conf < self.conf_threshold:
                            continue

                        # Convert from letterbox coords to original frame coords
                        x1 = (cx - w / 2 - dw) / scale
                        y1 = (cy - h / 2 - dh) / scale
                        x2 = (cx + w / 2 - dw) / scale
                        y2 = (cy + h / 2 - dh) / scale

                        # Clip to frame
                        x1 = max(0, min(orig_w, x1))
                        y1 = max(0, min(orig_h, y1))
                        x2 = max(0, min(orig_w, x2))
                        y2 = max(0, min(orig_h, y2))

                        if (x2 - x1) > 2 and (y2 - y1) > 2:
                            detections.append([x1, y1, x2, y2, conf])

        # Apply NMS
        if len(detections) > 0:
            dets = np.array(detections)
            keep = nms(dets[:, :4], dets[:, 4], self.iou_threshold)
            detections = dets[keep].tolist()

        return detections


# ──────────────────────────────────────────────────────────────────────
#  OPENCV DNN FALLBACK (for testing without Hailo hardware)
# ──────────────────────────────────────────────────────────────────────
class OpenCVDetector:
    """Fallback detector using OpenCV DNN with an ONNX model (CPU)."""

    def __init__(self, onnx_path, conf_threshold=0.35, iou_threshold=0.45, imgsz=640):
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.imgsz = imgsz

        print(f"[INFO] Loading ONNX model for CPU fallback: {onnx_path}")
        self.net = cv2.dnn.readNetFromONNX(onnx_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        print("[INFO] OpenCV DNN detector ready (CPU fallback).")

    def detect(self, frame):
        orig_h, orig_w = frame.shape[:2]
        img, scale, dw, dh = letterbox(frame, self.imgsz)

        blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (self.imgsz, self.imgsz),
                                     swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.net.getUnconnectedOutLayersNames())

        detections = []
        for output in outputs:
            output = np.squeeze(output)  # e.g. (5, 8400) or (8400, 5)

            # YOLOv8/YOLO26 format: (num_features, num_detections)
            # where num_features = 4 + num_classes
            # We need rows = detections, so transpose if features < detections
            if output.ndim == 2 and output.shape[0] < output.shape[1]:
                output = output.T  # now (8400, 5) = (detections, features)

            if output.ndim != 2 or output.shape[1] < 5:
                continue

            # Columns: [cx, cy, w, h, class0_score, class1_score, ...]
            # For single-class model: [cx, cy, w, h, person_score]
            boxes_cxcywh = output[:, :4]
            class_scores = output[:, 4:]
            confs = class_scores.max(axis=1)

            # Filter by confidence
            mask = confs >= self.conf_threshold
            if not mask.any():
                continue

            boxes_cxcywh = boxes_cxcywh[mask]
            confs = confs[mask]

            # Convert from letterboxed coords to original image coords
            for i in range(len(boxes_cxcywh)):
                cx, cy, w, h = boxes_cxcywh[i]
                x1 = (cx - w / 2 - dw) / scale
                y1 = (cy - h / 2 - dh) / scale
                x2 = (cx + w / 2 - dw) / scale
                y2 = (cy + h / 2 - dh) / scale
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(orig_w, x2), min(orig_h, y2)
                if (x2 - x1) > 2 and (y2 - y1) > 2:
                    detections.append([x1, y1, x2, y2, float(confs[i])])

        if len(detections) > 0:
            dets = np.array(detections)
            keep = nms(dets[:, :4], dets[:, 4], self.iou_threshold)
            detections = dets[keep].tolist()

        return detections


# ──────────────────────────────────────────────────────────────────────
#  AUTO-LOCK CONTROLLER
# ──────────────────────────────────────────────────────────────────────
class AutoLockController:
    """
    Manages the lock-on state machine:
      SCANNING  →  person found  →  LOCKED
      LOCKED    →  target lost for N frames  →  SCANNING
      *         →  no humans at all  →  NO_HUMAN
    """

    # ── States ──
    SCANNING = "SCANNING"
    LOCKED   = "LOCKED"
    NO_HUMAN = "NO_HUMAN"

    def __init__(self, lock_lost_patience=20):
        self.state = self.SCANNING
        self.locked_track_id = None
        self.lock_lost_patience = lock_lost_patience
        self._lost_counter = 0
        self._skip_ids = set()  # IDs to skip when cycling with 'u'

    def update(self, tracks: list[Track], frame_w: int, frame_h: int):
        """
        Process tracker output and return (state, locked_track_or_None).
        """
        active_tracks = [t for t in tracks if t.lost_frames == 0]

        # ── No humans visible at all ──
        if len(active_tracks) == 0 and self.locked_track_id is None:
            self.state = self.NO_HUMAN
            return self.state, None

        # ── Currently locked — try to find our target ──
        if self.locked_track_id is not None:
            locked_track = None
            for t in tracks:
                if t.id == self.locked_track_id:
                    locked_track = t
                    break

            if locked_track is not None and locked_track.lost_frames == 0:
                # Target still visible — stay locked
                self._lost_counter = 0
                self.state = self.LOCKED
                return self.state, locked_track
            else:
                # Target missing this frame
                self._lost_counter += 1
                if self._lost_counter >= self.lock_lost_patience:
                    # Lost for too long — go back to scanning
                    print(f"[AUTO-LOCK] Lost target ID {self.locked_track_id} for "
                          f"{self._lost_counter} frames. Releasing lock.")
                    self.locked_track_id = None
                    self._lost_counter = 0
                    self.state = self.SCANNING
                    return self.state, None
                else:
                    # Still within patience — report locked but missing
                    self.state = self.LOCKED
                    if locked_track:
                        return self.state, locked_track
                    return self.state, None

        # ── Scanning — wait for user click ──
        self.state = self.SCANNING
        return self.state, None

        self.state = self.SCANNING
        return self.state, None

    def force_unlock(self):
        """Manually release the lock (e.g., via a button press)."""
        print(f"[AUTO-LOCK] Manual unlock from ID {self.locked_track_id}")
        if self.locked_track_id is not None:
            self._skip_ids.add(self.locked_track_id)
        self.locked_track_id = None
        self._lost_counter = 0
        self.state = self.SCANNING

    def force_lock(self, track_id):
        """Manually lock onto a specific track ID."""
        print(f"[AUTO-LOCK] 🔒 Manual LOCK onto person ID {track_id}")
        self.locked_track_id = track_id
        self._lost_counter = 0
        self.state = self.LOCKED# ──────────────────────────────────────────────────────────────────────
#  VISUALIZATION
# ──────────────────────────────────────────────────────────────────────
def draw_overlay(frame, tracks, lock_state, locked_track, fps):
    """Draw detection boxes, lock indicator, trail, and HUD."""
    h, w = frame.shape[:2]

    for t in tracks:
        if t.lost_frames > 0:
            continue  # Don't draw lost tracks

        x1, y1, x2, y2 = [int(v) for v in t.box]
        is_locked = (locked_track is not None and t.id == locked_track.id)

        if is_locked:
            # ── Locked target: RED box + crosshair + trail ──
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)

            # Crosshair at center
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            cross_size = 15
            cv2.line(frame, (cx - cross_size, cy), (cx + cross_size, cy), (0, 0, 255), 2)
            cv2.line(frame, (cx, cy - cross_size), (cx, cy + cross_size), (0, 0, 255), 2)

            # Trail
            trail = list(t.trail)
            for i in range(1, len(trail)):
                alpha = i / len(trail)
                color = (0, 0, int(255 * alpha))
                cv2.line(frame, trail[i - 1], trail[i], color, 2)

            # Label
            label = f"LOCKED ID:{t.id} {t.conf:.0%}"
            cv2.putText(frame, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            # ── Other detections: dim gray box ──
            overlay = frame.copy()
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (100, 100, 100), 1)
            cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

    # ── HUD (top bar) ──
    # Status color
    status_colors = {
        AutoLockController.LOCKED: (0, 0, 255),
        AutoLockController.SCANNING: (0, 200, 255),
        AutoLockController.NO_HUMAN: (128, 128, 128),
    }
    status_color = status_colors.get(lock_state, (255, 255, 255))

    cv2.rectangle(frame, (0, 0), (w, 40), (20, 20, 20), -1)
    cv2.putText(frame, f"STATUS: {lock_state}", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
    cv2.putText(frame, f"FPS: {fps:.1f}", (w - 130, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    persons_count = sum(1 for t in tracks if t.lost_frames == 0)
    cv2.putText(frame, f"Persons: {persons_count}", (w // 2 - 50, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # ── Lock indicator (bottom bar) ──
    if locked_track is not None:
        cx, cy = locked_track.center
        bar_text = f"Target ID:{locked_track.id}  Pos:({int(cx)},{int(cy)})  Conf:{locked_track.conf:.0%}"
        cv2.rectangle(frame, (0, h - 35), (w, h), (20, 20, 20), -1)
        cv2.putText(frame, bar_text, (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)

    return frame


# ──────────────────────────────────────────────────────────────────────
#  MAIN LOOP
# ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Auto-Lock Human Detection for Pi 5 + Hailo-8 AI HAT",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--hef", type=str, default=None,
                        help="Path to compiled .hef model for Hailo-8")
    parser.add_argument("--onnx", type=str, default=None,
                        help="Path to .onnx model (CPU fallback if no --hef)")
    parser.add_argument("--source", type=str, default="0",
                        help="Camera index (0) or video file path")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="Inference image size (default: 640)")
    parser.add_argument("--conf", type=float, default=0.35,
                        help="Confidence threshold (default: 0.35)")
    parser.add_argument("--iou", type=float, default=0.45,
                        help="NMS IoU threshold (default: 0.45)")
    parser.add_argument("--no-display", action="store_true",
                        help="Disable GUI window (headless mode)")
    parser.add_argument("--save", type=str, default=None,
                        help="Save output video to this path")
    args = parser.parse_args()

    # ── Initialize detector ──
    if args.hef and HAILO_AVAILABLE:
        detector = HailoDetector(args.hef, args.conf, args.iou, args.imgsz)
    elif args.onnx:
        detector = OpenCVDetector(args.onnx, args.conf, args.iou, args.imgsz)
    else:
        print("ERROR: Provide either --hef (for Hailo) or --onnx (for CPU fallback).")
        print("       On Pi 5 with Hailo HAT: python auto_lock_node.py --hef model.hef")
        sys.exit(1)

    # ── Initialize camera ──
    source = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"ERROR: Cannot open video source: {args.source}")
        sys.exit(1)

    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[INFO] Video source: {args.source} ({frame_w}x{frame_h})")

    # ── Initialize tracker and auto-lock ──
    tracker = SimpleTracker(iou_threshold=0.3, max_lost=15)
    auto_lock = AutoLockController(lock_lost_patience=20)

    # ── Optional video writer ──
    writer = None
    if args.save:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.save, fourcc, 25, (frame_w, frame_h))
        print(f"[INFO] Saving output to: {args.save}")

    # ── Main loop ──
    print("\n" + "=" * 50)
    print("  CLICK-TO-LOCK DETECTION RUNNING")
    print("  Click on a person to lock | 'u' to unlock | 'q' to quit")
    print("=" * 50 + "\n")

    # ── Mouse Callback ──
    click_target = None
    def mouse_callback(event, x, y, flags, param):
        nonlocal click_target
        if event == cv2.EVENT_LBUTTONDOWN:
            click_target = (x, y)

    if not args.no_display:
        cv2.namedWindow("Aerial Human Auto-Lock")
        cv2.setMouseCallback("Aerial Human Auto-Lock", mouse_callback)

    fps = 0.0
    frame_count = 0

    # ── Detect if source is a static image ──
    is_image = args.source.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tiff', '.webp'))

    try:
        while True:
            t_start = time.perf_counter()

            ret, frame = cap.read()
            if not ret:
                if is_image and frame_count > 0:
                    # Already processed the image — just wait for keypress
                    break
                print("[INFO] End of video stream.")
                if frame_count > 0 and not args.no_display:
                    print("[INFO] Press any key in the image window to exit...")
                    cv2.waitKey(0)
                break

            # 1. Detect
            detections = detector.detect(frame)

            # 2. Track
            tracks = tracker.update(detections)

            # Handle mouse click for locking
            if click_target is not None:
                cx, cy = click_target
                click_target = None
                # Find the track whose bounding box contains the click
                for t in tracks:
                    x1, y1, x2, y2 = t.box
                    if x1 <= cx <= x2 and y1 <= cy <= y2:
                        auto_lock.force_lock(t.id)
                        break

            # 3. Auto-lock
            lock_state, locked_track = auto_lock.update(tracks, frame_w, frame_h)

            # 4. Print status to stdout (machine-readable for external systems)
            if locked_track:
                cx, cy = locked_track.center
                print(f"[{lock_state}] ID={locked_track.id} "
                      f"pos=({int(cx)},{int(cy)}) conf={locked_track.conf:.2f} "
                      f"fps={fps:.1f}", end="\r")
            else:
                print(f"[{lock_state}] fps={fps:.1f}", end="\r")

            # 5. Draw overlay
            display_frame = draw_overlay(frame.copy(), tracks, lock_state, locked_track, fps)

            # 6. Display / save
            if not args.no_display:
                cv2.imshow("Aerial Human Auto-Lock", display_frame)

                if is_image:
                    # For images: wait indefinitely, let user press 'u' or 'q'
                    print(f"\n[INFO] Detected {len(detections)} person(s). "
                          f"Press 'u' to unlock/cycle, 'q' to quit.")
                    while True:
                        key = cv2.waitKey(0) & 0xFF
                        if key == ord('q'):
                            frame_count += 1
                            raise KeyboardInterrupt  # clean exit
                        elif key == ord('u'):
                            auto_lock.force_unlock()
                            # Re-run lock logic on same detections
                            tracks = tracker.update(detections)
                            lock_state, locked_track = auto_lock.update(tracks, frame_w, frame_h)
                            display_frame = draw_overlay(frame.copy(), tracks, lock_state, locked_track, fps)
                            cv2.imshow("Aerial Human Auto-Lock", display_frame)
                            if locked_track:
                                cx, cy = locked_track.center
                                print(f"[{lock_state}] ID={locked_track.id} "
                                      f"pos=({int(cx)},{int(cy)}) conf={locked_track.conf:.2f}")
                            else:
                                print(f"[{lock_state}] No target locked")
                else:
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('u'):
                        auto_lock.force_unlock()

            if writer:
                writer.write(display_frame if not args.no_display else frame)

            # FPS calculation
            t_end = time.perf_counter()
            fps = 1.0 / max(t_end - t_start, 1e-6)
            frame_count += 1

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")

    finally:
        cap.release()
        if writer:
            writer.release()
        cv2.destroyAllWindows()
        print(f"\n[INFO] Processed {frame_count} frames. Done.")


if __name__ == "__main__":
    main()
