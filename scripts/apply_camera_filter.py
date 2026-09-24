#!/usr/bin/env python3
"""Apply the camera de-noising / low-latency changes to an existing checkout.

Runs ON the Pi, in place, and is safe to run more than once: every edit is
anchored to the exact text it replaces and is skipped if already applied. If an
anchor is missing the script stops and says which one rather than guessing -
the Pi's working copy is authoritative and has diverged from the desktop mirror
before (avoidance_vfh_enabled, takeoff_altitude_m), so nothing here overwrites
a whole file.

    python3 scripts/apply_camera_filter.py [--check]

--check reports what would change and touches nothing.
"""
from __future__ import annotations

import argparse
import re
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STAMP = time.strftime("%Y%m%d_%H%M%S")

CAMERAS_BLOCK = 'cameras:\n  enabled: true\n\n  # JPEG quality for the MJPEG stream (1-100). 60 is the point where the\n  # camera\'s own sensor noise costs more bandwidth than the extra quality buys.\n  jpeg_quality: 60\n\n  # ------------------------------------------------------------------ picam --\n  picam:\n    width: 1280\n    height: 720\n    fps: 30\n    # ISP (hardware) denoise: off | minimal | fast | high_quality.\n    # Runs on the image pipeline, not on a CPU core the flight stack needs, so\n    # it is the cheapest noise reduction available and the first thing to use.\n    isp_denoise: fast\n    # Auto-exposure bias: normal | short | long. "short" trades gain for a\n    # shorter exposure. On a moving aircraft this cuts motion blur AND the\n    # rolling-shutter skew that makes the frame appear to tear horizontally,\n    # because the top and bottom rows are sampled closer together in time.\n    ae_mode: short\n    # Apply the CPU frame filter below to this camera. On by default: this is\n    # the camera whose CSI ribbon runs past the ESCs.\n    filter: true\n    # The camera is mounted upside down on the airframe; the ISP does\n    # this flip in hardware via libcamera Transform.\n    rotate_180: true\n\n  # ----------------------------------------------------------------- filter --\n  # CPU image conditioning. Deliberately narrow: it fixes what the ISP cannot\n  # see, which is row-correlated interference from the ESCs coupling into the\n  # CSI ribbon in flight (horizontal lines drifting through the frame).\n  # See drone_stack/gcs/frame_filter.py for the full rationale.\n  filter:\n    enabled: true\n\n    # -- stage 1: destripe (~1 ms at 720p) --\n    destripe: true\n    # Rows compared against a median of this many neighbours. The window wants\n    # to sit at or above the interference period but BELOW the finest real\n    # horizontal texture worth keeping. Measured at 720p: 21 removes 81% of a\n    # continuous band (15 removes 34%) while keeping the horizon at 100% and\n    # real horizontal texture at 97%; 31 removes only 69% and destroys half of\n    # that texture. Note that banding slower than the window is indistinguishable\n    # from vertical shading and is deliberately left alone.\n    destripe_window: 21\n    # Shrinkage floor in DN. A row within this much of where its neighbours say\n    # it belongs is left bit-for-bit alone, so a clean frame passes through\n    # untouched and nothing is ever invented. Raise if fine horizontal detail\n    # looks softened; lower to catch fainter banding.\n    destripe_floor: 0.6\n    # Largest correction any single row may receive, in DN. A residual bigger\n    # than this is more likely a real edge the median could not follow exactly\n    # than interference - the rows either side of a hard horizon are where that\n    # happens. Measured on a banded scene with an 89 DN horizon: capping at 10\n    # keeps 90% of the horizon against 85% uncapped while removing slightly MORE\n    # of the band. Raise only if you genuinely have banding stronger than this.\n    destripe_ceiling: 10.0\n    destripe_gain: 1.0\n    # Sample every Nth column when measuring row brightness. 4 is 4x cheaper\n    # and statistically identical - a 720p row still contributes 320 samples.\n    column_step: 4\n\n    # -- stage 2: row repair --\n    # A row displaced this far (DN) is destroyed rather than merely offset, so\n    # it is redrawn from the nearest surviving rows instead of being corrected.\n    row_repair: true\n    row_repair_floor: 14.0\n\n    # -- stage 3: temporal denoise (~8 ms at 720p on a Mac; measured and\n    #    auto-disabled on hardware if it does not fit - see budget_ms) --\n    temporal: true\n    # Blend weight against the previous frame where nothing moved. Higher is\n    # cleaner but slower to respond; 0.95 is the hard ceiling.\n    temporal_alpha: 0.6\n    # Frame-to-frame change (DN) at or below which motion is indistinguishable\n    # from sensor noise - blend at full strength. Without this deadband the\n    # noise floor sits partway up the ramp and halves the filter for nothing.\n    temporal_deadband: 6.0\n    # Change (DN) at which the blend drops to zero, so genuine movement passes\n    # through untouched and never ghosts or smears.\n    temporal_motion: 20.0\n    # Motion gate evaluated on s x s blocks. Bigger is cheaper and less\n    # noise-sensitive, but localises motion less precisely.\n    temporal_gate_scale: 4\n\n    # Latency guard. If the smoothed per-frame filter cost stays above this,\n    # the temporal stage is dropped (and the reason logged) rather than letting\n    # the video fall behind. Destriping is never dropped: it is the stage the\n    # artefact actually needs and it costs almost nothing.\n    budget_ms: 22.0\n'

SERVER_OLD = '        async def gen():\n            last = None\n            try:\n                while True:\n                    jpeg = cam.jpeg()\n                    if jpeg is not last:\n                        last = jpeg\n                        yield (\n                            b"--" + boundary.encode() + b"\\r\\n"\n                            b"Content-Type: image/jpeg\\r\\n"\n                            b"Content-Length: " + str(len(jpeg)).encode()\n                            + b"\\r\\n\\r\\n" + jpeg + b"\\r\\n"\n                        )\n                    await asyncio.sleep(1 / 30)\n            except asyncio.CancelledError:\n                return'

SERVER_NEW = '        async def gen():\n            # Event-driven, not polled. The original loop woke on a 1/30 s timer\n            # and re-checked, so a frame finishing just after a tick waited out\n            # the rest of the period before being sent - up to 33 ms added to\n            # every frame, plus a wakeup 30 times a second per viewer whether or\n            # not anything had changed. This awaits the camera\'s own publish\n            # notification instead, so each frame goes out the instant it is\n            # encoded and an idle stream costs nothing at all.\n            last_seq = -1\n            new_frame = cam.subscribe()\n            try:\n                while True:\n                    jpeg, seq = cam.jpeg_seq()\n                    if seq != last_seq:\n                        last_seq = seq\n                        yield (\n                            b"--" + boundary.encode() + b"\\r\\n"\n                            b"Content-Type: image/jpeg\\r\\n"\n                            b"Content-Length: " + str(len(jpeg)).encode()\n                            + b"\\r\\n\\r\\n" + jpeg + b"\\r\\n"\n                        )\n                        continue\n                    new_frame.clear()\n                    # Re-read after clearing: a frame published between the read\n                    # above and the clear would otherwise be lost and we would\n                    # wait a full second through it.\n                    if cam.jpeg_seq()[1] != last_seq:\n                        continue\n                    try:\n                        # Bounded, so a camera that stops delivering cannot wedge\n                        # the request open forever.\n                        await asyncio.wait_for(new_frame.wait(), timeout=1.0)\n                    except asyncio.TimeoutError:\n                        pass\n            except asyncio.CancelledError:\n                return\n            finally:\n                cam.unsubscribe(new_frame)'

HUB_OLD = '        self.cameras = CameraManager(\n            bus=self.bus,\n            enabled=bool(config.get("cameras.enabled", True)),\n        )'

HUB_NEW = '        self.cameras = CameraManager(\n            bus=self.bus,\n            enabled=bool(config.get("cameras.enabled", True)),\n            settings=config.get("cameras", {}) or {},\n        )'


class Abort(Exception):
    pass


def patch_file(path: Path, old: str, new: str, tag: str, check: bool) -> str:
    """Anchored replace. Already-applied is success, not an error."""
    if not path.exists():
        return "absent - skipped"
    text = path.read_text()
    if new in text:
        return "already current"
    if old not in text:
        raise Abort(f"{path.name}: anchor for '{tag}' not found. The Pi's copy has "
                    f"diverged from what was tested. Apply this edit by hand, or "
                    f"re-pull the file, then re-run.")
    if not check:
        shutil.copy2(path, Path(str(path) + f".bak_{STAMP}"))
        path.write_text(text.replace(old, new, 1))
    return tag


def patch_yaml(path: Path, check: bool) -> str:
    text = path.read_text()
    existing = re.search(r"^cameras:\n(?:[ \t#].*\n|\n)*", text, re.M)
    if existing and existing.group(0).strip() == CAMERAS_BLOCK.strip():
        return "already current"
    if existing:
        new = text[:existing.start()] + CAMERAS_BLOCK + text[existing.end():]
        what = "cameras block replaced"
    else:
        new = text.rstrip("\n") + "\n\n" + CAMERAS_BLOCK
        what = "cameras block appended"
    if not check:
        shutil.copy2(path, Path(str(path) + f".bak_{STAMP}"))
        path.write_text(new)
    return what


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="report what would change, touch nothing")
    args = ap.parse_args()
    lead = "would " if args.check else ""

    try:
        for name in ("config/default.yaml", "config/real.yaml"):
            path = ROOT / name
            if not path.exists():
                print(f"  skip   {name} (absent)")
                continue
            print(f"  {lead}{patch_yaml(path, args.check):24s} {name}")

        print(f"  {lead}{patch_file(ROOT / 'drone_stack/gcs/server.py', SERVER_OLD, SERVER_NEW, 'event-driven MJPEG', args.check):24s} gcs/server.py")
        print(f"  {lead}{patch_file(ROOT / 'drone_stack/gcs/hub.py', HUB_OLD, HUB_NEW, 'pass cameras config', args.check):24s} gcs/hub.py")

        # cameras.py is copied wholesale by the deploy script, not patched -
        # too many edits to anchor safely. Verify the copy actually landed.
        cam = (ROOT / "drone_stack/gcs/cameras.py").read_text()
        if "_condition" not in cam or "fastNlMeans" in cam:
            raise Abort("drone_stack/gcs/cameras.py is still the old version "
                        "(NLM denoise present / _condition missing) - copy the "
                        "new cameras.py across before running this")
        print(f"  {'':6s}{'verified':24s} gcs/cameras.py")

        if not (ROOT / "drone_stack/gcs/frame_filter.py").exists():
            raise Abort("drone_stack/gcs/frame_filter.py is missing - copy it "
                        "across before running this")
        print(f"  {'':6s}{'present':24s} gcs/frame_filter.py")
    except Abort as exc:
        print(f"ABORT: {exc}", file=sys.stderr)
        return 2

    print("\nNext:  .venv/bin/python -m pytest tests/test_camera_filter.py -q")
    print("       sudo systemctl restart aerix-gcs")
    print("       python3 scripts/camera_latency.py --cam 1")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
