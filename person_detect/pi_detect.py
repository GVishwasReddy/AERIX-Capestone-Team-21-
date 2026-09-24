#!/usr/bin/env python3
"""
Run the .hef on the Pi AI HAT+ (Hailo-8).

    python3 pi_detect.py model.hef image.jpg
    python3 pi_detect.py model.hef folder/ --out results/
    python3 pi_detect.py model.hef video.mp4 --out out.mp4

The HEF has NMS compiled in (nms_postprocess, engine=cpu), so HailoRT hands
back decoded detections - there is no DFL/anchor maths to redo here.

Input contract: the network does its own /255 on-chip, so we feed raw uint8.
Do not scale to 0-1 first; that would divide twice and everything vanishes.
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

from hailo_platform import (HEF, VDevice, HailoStreamInterface, InferVStreams,
                            ConfigureParams, InputVStreamParams,
                            OutputVStreamParams, FormatType)

IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp", ".webp")
VID_EXT = (".mp4", ".avi", ".mov", ".mkv")


def letterbox(bgr, size):
    """Match make_calib.py exactly: grey 114 pad, aspect preserved, RGB out."""
    h0, w0 = bgr.shape[:2]
    r = min(size / h0, size / w0)
    nw, nh = round(w0 * r), round(h0 * r)
    resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, np.uint8)
    padx, pady = (size - nw) // 2, (size - nh) // 2
    canvas[pady:pady + nh, padx:padx + nw] = resized
    return cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB), r, padx, pady


def unletterbox(x1, y1, x2, y2, r, padx, pady, w0, h0):
    """Undo the letterbox so boxes land on the original frame."""
    x1 = (x1 - padx) / r
    x2 = (x2 - padx) / r
    y1 = (y1 - pady) / r
    y2 = (y2 - pady) / r
    return (max(0.0, min(x1, w0 - 1)), max(0.0, min(y1, h0 - 1)),
            max(0.0, min(x2, w0 - 1)), max(0.0, min(y2, h0 - 1)))


def parse_nms(raw, size):
    """
    HailoRT NMS output -> [(x1, y1, x2, y2, score), ...] in network pixels.

    With one class the payload is a list whose single entry is an (n, 5)
    array of (y_min, x_min, y_max, x_max, score), normalised 0-1.
    Shapes vary a little across HailoRT versions, so be tolerant.
    """
    dets = []
    per_class = raw
    while isinstance(per_class, (list, tuple)) and len(per_class) == 1 \
            and not isinstance(per_class[0], np.ndarray):
        per_class = per_class[0]

    if isinstance(per_class, np.ndarray) and per_class.ndim == 2:
        per_class = [per_class]

    for arr in per_class:
        arr = np.asarray(arr)
        if arr.size == 0:
            continue
        arr = arr.reshape(-1, arr.shape[-1])
        for row in arr:
            ymin, xmin, ymax, xmax, score = row[:5]
            dets.append((float(xmin) * size, float(ymin) * size,
                         float(xmax) * size, float(ymax) * size, float(score)))
    return dets


def host_nms(dets, iou_th):
    """
    Second-pass NMS on the host.

    The HEF's built-in NMS only merges boxes overlapping by IoU >= 0.70. Int8
    rounding in the box-regression heads makes coordinates jitter by a few
    pixels, leaving near-duplicate boxes on one person that 0.70 never merges.
    Measured on 40 aerial frames / 1025 people (int8 emulation, conf 0.25):
        HEF NMS only (0.70)   recall 65.6%  precision 42.4%
        + host NMS 0.50       recall 64.4%  precision 57.4%
        + host NMS 0.40       recall 63.4%  precision 63.5%
    Pass 0 to disable. Keep it at or above 0.4 for crowds: lower values start
    merging genuinely adjacent people.
    """
    if iou_th <= 0 or len(dets) < 2:
        return dets
    a = np.asarray(dets, np.float32)
    a = a[np.argsort(-a[:, 4])]
    x1, y1, x2, y2 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    area = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    alive = np.ones(len(a), bool)
    keep = []
    for i in range(len(a)):
        if not alive[i]:
            continue
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[i + 1:]); yy1 = np.maximum(y1[i], y1[i + 1:])
        xx2 = np.minimum(x2[i], x2[i + 1:]); yy2 = np.minimum(y2[i], y2[i + 1:])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        iou = inter / np.maximum(area[i] + area[i + 1:] - inter, 1e-9)
        alive[i + 1:] &= iou < iou_th
    return [tuple(float(v) for v in a[k]) for k in keep]


class Detector:
    def __init__(self, hef_path):
        self.hef = HEF(hef_path)
        self.vdev = VDevice()
        cfg = ConfigureParams.create_from_hef(
            self.hef, interface=HailoStreamInterface.PCIe)
        self.ng = self.vdev.configure(self.hef, cfg)[0]
        self.ng_params = self.ng.create_params()

        self.in_info = self.hef.get_input_vstream_infos()[0]
        self.out_info = self.hef.get_output_vstream_infos()[0]
        self.size = self.in_info.shape[0]          # 1280 (square)

        self.in_params = InputVStreamParams.make(
            self.ng, format_type=FormatType.UINT8)
        self.out_params = OutputVStreamParams.make(
            self.ng, format_type=FormatType.FLOAT32)

        self._act = self.ng.activate(self.ng_params)
        self._act.__enter__()
        self._pipe = InferVStreams(self.ng, self.in_params, self.out_params)
        self._pipe.__enter__()

    def close(self):
        self._pipe.__exit__(None, None, None)
        self._act.__exit__(None, None, None)
        self.vdev.release()

    def __call__(self, bgr, conf, nms_iou=0.5):
        h0, w0 = bgr.shape[:2]
        rgb, r, padx, pady = letterbox(bgr, self.size)
        batch = np.expand_dims(rgb, 0)             # uint8 NHWC, no /255
        res = self._pipe.infer({self.in_info.name: batch})
        dets = parse_nms(res[self.out_info.name], self.size)
        dets = host_nms([d for d in dets if d[4] >= conf], nms_iou)

        out = []
        for x1, y1, x2, y2, score in dets:
            if score < conf:
                continue
            out.append(unletterbox(x1, y1, x2, y2, r, padx, pady, w0, h0)
                       + (score,))
        return out


def draw(bgr, dets):
    for x1, y1, x2, y2, s in dets:
        p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
        cv2.rectangle(bgr, p1, p2, (0, 255, 0), 2)
        cv2.putText(bgr, f"{s:.2f}", (p1[0], max(12, p1[1] - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    return bgr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("hef")
    ap.add_argument("source", help="image, folder, or video")
    ap.add_argument("--out", default="hailo_out")
    ap.add_argument("--conf", type=float, default=0.25,
                    help="display threshold; the HEF's own floor is 0.20 "
                         "and is compiled in - you cannot raise it past that "
                         "without recompiling, only filter above it")
    ap.add_argument("--nms-iou", type=float, default=0.5,
                    help="extra host-side NMS IoU to remove int8 duplicate "
                         "boxes (0 = off). 0.5 default, 0.4 for fewer duplicates")
    args = ap.parse_args()

    if args.conf < 0.20:
        print(f"[warn] --conf {args.conf} is below the compiled-in "
              f"nms_scores_th of 0.20; nothing under 0.20 ever leaves the chip.",
              file=sys.stderr)

    det = Detector(args.hef)
    print(f"[info] {os.path.basename(args.hef)}  input {det.size}x{det.size} uint8")

    try:
        src = args.source
        if os.path.isdir(src):
            os.makedirs(args.out, exist_ok=True)
            files = sorted(f for f in os.listdir(src)
                           if f.lower().endswith(IMG_EXT))
            t0, total = time.time(), 0
            for f in files:
                img = cv2.imread(os.path.join(src, f))
                if img is None:
                    continue
                d = det(img, args.conf, args.nms_iou)
                total += len(d)
                cv2.imwrite(os.path.join(args.out, f), draw(img, d))
            dt = time.time() - t0
            print(f"[done] {len(files)} images, {total} detections, "
                  f"{len(files)/max(dt,1e-9):.2f} img/s -> {args.out}/")

        elif src.lower().endswith(VID_EXT) or str(src).isdigit() or str(src).startswith("/dev/video"):
            cap = cv2.VideoCapture(int(src) if str(src).isdigit() else src)
            fps = cap.get(cv2.CAP_PROP_FPS) or 30
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            outp = args.out if args.out.lower().endswith(".mp4") else args.out + ".mp4"
            vw = cv2.VideoWriter(outp, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
            n, t0 = 0, time.time()
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                vw.write(draw(frame, det(frame, args.conf, args.nms_iou)))
                n += 1
                if n % 30 == 0:
                    print(f"  {n} frames  {n/(time.time()-t0):.2f} fps", end="\r")
            cap.release()
            vw.release()
            print(f"\n[done] {n} frames, {n/max(time.time()-t0,1e-9):.2f} fps -> {outp}")

        else:
            img = cv2.imread(src)
            if img is None:
                sys.exit(f"cannot read {src}")
            d = det(img, args.conf, args.nms_iou)
            outp = args.out if args.out.lower().endswith(IMG_EXT) else args.out + ".jpg"
            cv2.imwrite(outp, draw(img, d))
            print(f"[done] {len(d)} detections -> {outp}")
            for x1, y1, x2, y2, s in d:
                print(f"   {s:.3f}  ({x1:.0f},{y1:.0f})-({x2:.0f},{y2:.0f})")
    finally:
        det.close()


if __name__ == "__main__":
    main()
