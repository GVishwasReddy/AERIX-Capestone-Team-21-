# Aerial person detector on Pi 5 + AI HAT+ (Hailo-8, 26 TOPS)

`aerix_person_y8m_1280_pass2_biascorr.hef` - YOLOv8m, 1280x1280 input, int8,
compiled for **Hailo-8** (will not load on a 13-TOPS Hailo-8L).

## 1. Install (Raspberry Pi OS Bookworm, 64-bit)

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y hailo-all python3-opencv
sudo reboot
```

```bash
hailortcli fw-control identify                                   # expect Board Name: Hailo-8
hailortcli run aerix_person_y8m_1280_pass2_biascorr.hef          # synthetic FPS ceiling
```

If PCIe isn't detected, add `dtparam=pciex1` and `dtparam=pciex1_gen=3` to
`/boot/firmware/config.txt` and reboot.

## 2. Run

```bash
python3 pi_detect.py aerix_person_y8m_1280_pass2_biascorr.hef photo.jpg
python3 pi_detect.py aerix_person_y8m_1280_pass2_biascorr.hef frames/ --out annotated/
python3 pi_detect.py aerix_person_y8m_1280_pass2_biascorr.hef flight.mp4 --out flight_out.mp4
```

Feed raw uint8 frames (the /255 is on-chip; `pi_detect.py` handles this).

## 3. Settings - measured, not guessed

Measured in int8 emulation on 40 aerial frames containing 1,025 labelled people
(666 of them under 12 px). Precision here counts duplicate boxes as errors.

| use | flags | recall | precision |
|---|---|---|---|
| default | `--conf 0.25 --nms-iou 0.5` | 64.4% | 57.4% |
| fewer duplicate boxes (counting, reporting) | `--conf 0.25 --nms-iou 0.4` | 63.4% | 63.5% |
| HEF alone, no extra NMS | `--conf 0.25 --nms-iou 0` | 65.6% | 42.4% |

Why `--nms-iou` exists: most of int8's "false positives" are not phantom
people. 704 of 912 were shifted or duplicate boxes on *real* people - int8
rounding makes box edges jitter, and the HEF's built-in NMS only merges boxes
overlapping >= 70%. The host pass merges the rest. Don't go below 0.4 in
crowds or adjacent people get merged.

`--conf` cannot go below 0.20: that floor is compiled into the HEF.

## 4. What it can actually see - by person size in the model's input

The model sees every frame shrunk to 1280 px across. Size = sqrt(box width x
height) in those pixels. Recall at `--conf 0.25`:

| person size | int8 recall | verdict |
|---|---|---|
| 24 px and up | ~94% | reliable |
| 12-24 px | ~84% | good |
| 8-12 px | ~75% | usable, will miss 1 in 4 |
| under 8 px | ~38% | **not usable** |

## 5. Maximum height - Pi Camera Module 3 (66 deg horizontal FOV)

Heights are for a person at the **centre** of the frame. In a forward-tilted
view, people near the top of the frame are much farther away and look smaller.

**Single frame (`pi_detect.py` as shipped):**

| camera angle | ~94% (>=24 px) | ~84% (>=12 px) | ~75% (>=8 px) | beyond this: unusable |
|---|---|---|---|---|
| straight down | 18 m | 37 m | 55 m | > 55 m |
| 30 deg tilt | 28 m | 55 m | 83 m | > 83 m |
| 45 deg tilt | 25 m | 50 m | 75 m | > 75 m |
| 60 deg (forward) | 19 m | 38 m | 56 m | > 56 m |

Straight down is the worst case: from above a person is only about 0.5 m of
head and shoulders.

**Split each frame into two halves** (2304x1296 camera mode, each half run
through the model separately): about 1.8x the heights above, at about half
the frame rate. The sensor is 4608 px wide and the single-frame pipeline throws
72% of that detail away; tiling keeps it. `pi_detect.py` does not tile yet.

## 6. Realistic way to use it

- **Search / overwatch:** fly 30-50 m with the camera tilted 30-45 deg. You
  will catch most people at 12 px+, and miss many very small or distant ones.
  Treat "no detection" at this height as *not checked*, not *clear*.
- **Confirm before any decision that matters:** descend until people would be
  24 px+ (below ~18 m straight down, ~25 m at 45 deg) and look again.
- **Landing zone check:** do it on final approach below ~18 m, camera straight
  down, `--conf 0.20`. Treat any detection in the zone as "do not land". A
  false alarm only costs a go-around; a miss is the dangerous error, so bias
  toward recall here.
- **Use several frames, not one.** Hover over the zone and require it to be
  clear across a second or two of video. Misses in consecutive frames are
  correlated (same occlusion, same angle), so move or change height slightly
  rather than just waiting.
- **Not validated:** night, IR, heavy rain/fog, motion blur at speed, people
  under trees. These numbers come from daylight drone footage similar to
  VisDrone/AeroScapes.

## Honest limits of these numbers

- Measured in software emulation of the int8 model, not on the physical chip.
- The 40 frames are from the calibration set, so absolute numbers are somewhat
  optimistic. Validate on your own footage before relying on it.
- Under 8 px even the original float model only finds ~54% - that limit is the
  model and the camera resolution, not the Hailo conversion.
