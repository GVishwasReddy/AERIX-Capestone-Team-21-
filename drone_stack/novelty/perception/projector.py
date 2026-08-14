"""Pixel -> ground-plane projection (flat-earth pinhole model).

Every metric threshold in §2.1/§2.2/§2.4 (recipient position in metres,
displacement rate in m/s, zone radii in metres) needs a pixel-to-ground
conversion. This module is the ONE place that conversion happens; every
other novelty module receives already-projected :class:`~drone_stack.
novelty.types.GroundPoint` values and never touches a camera intrinsic.

-------------------------------------------------------------------------
Camera mount convention (read this before touching the algebra below)
-------------------------------------------------------------------------
Body frame (matches drone_stack/utils/geometry.py exactly):
    x_b = forward, y_b = left, z_b = up.

Camera frame (OpenCV pinhole convention):
    x_c = image right, y_c = image down, z_c = optical axis (into scene).

Mount pose is ONE parameter, ``tilt_from_nadir_deg``:
    0 deg  -> camera points straight down  (nadir; the expected mount for a
              delivery drone that must see the landing zone/recipient below it)
    90 deg -> camera points straight forward (horizontal)
Roll about the optical axis is assumed 0 (image "up" edge stays aligned with
the drone's forward direction) - this is a documented limitation, not an
oversight; see "Known limitations" in docs/novelty/landing_zone.md.

Deriving the body-frame camera basis as a function of tilt (theta), by the
two boundary conditions above and completing an orthonormal frame:

    f(theta) = ( sin(theta),  0, -cos(theta) )   # optical axis
    r(theta) = ( 0,          -1,  0          )   # image +x (right)
    d(theta) = f(theta) x r(theta)
             = ( -cos(theta), 0, -sin(theta) )   # image +y (down)

(f(0)=(0,0,-1)=straight down; f(90deg)=(1,0,0)=straight forward; r is fixed
along the body's left/right axis since we only rotate about it; d follows
from the right-handed camera frame. See docs/novelty/landing_zone.md for the
full derivation and a worked numeric example.)

-------------------------------------------------------------------------
Pixel -> ground formula
-------------------------------------------------------------------------
For a pixel (px, py) in a frame of intrinsics (fx, fy, cx, cy):

    rx = (px - cx) / fx
    ry = (py - cy) / fy
    dir_cam  = (rx, ry, 1)                                 # un-normalised
    dir_body = rx * r(theta) + ry * d(theta) + 1 * f(theta)

Intersect the ray (from the camera, assumed coincident with the body
origin - see limitations) with the flat ground plane at body-relative
height ``-altitude_m``:

    t = -altitude_m / dir_body.z          (undefined/None if dir_body.z >= 0,
                                            i.e. the ray points at or above
                                            the horizon - no ground intersection)
    ground_x_m = t * dir_body.x            # forward
    ground_y_m = t * dir_body.y            # left

This is exact for a flat ground plane and a pinhole camera; it is NOT exact
where the terrain itself has slope (see landing_zone.py's separate slope
handling) or where the camera is offset from the body origin (assumed
negligible for a small multirotor - flagged as a limitation).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from drone_stack.novelty.types import GroundPoint


class GroundProjector(Protocol):
    def pixel_to_ground(
        self, px: float, py: float, altitude_m: float, frame_shape: tuple[int, int]
    ) -> GroundPoint | None:
        """Project one pixel to a body-relative ground point, or None if the
        ray does not intersect the ground (points at/above the horizon)."""
        ...

    def pixels_to_ground(
        self, px: np.ndarray, py: np.ndarray, altitude_m: float, frame_shape: tuple[int, int]
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Vectorised batch form of :meth:`pixel_to_ground`, for callers that
        must project every pixel of a frame (landing_zone.py's grid
        rasteriser - a Python-level loop over ~240k segmentation pixels is
        too slow). Returns ``(ground_x_m, ground_y_m, valid)`` arrays shaped
        like ``px``/``py``; ``valid[i]`` is False (x/y are 0.0 there)
        wherever the ray misses the ground, matching ``pixel_to_ground``'s
        ``None`` case elementwise. Must implement the identical formula as
        ``pixel_to_ground`` - this stays the ONE place pixel->ground math
        lives, per the module docstring."""
        ...


@dataclass(frozen=True)
class FlatEarthPinhole:
    """Default :class:`GroundProjector` - see module docstring for the
    derivation. ``fx``/``fy``/``cx``/``cy`` and ``tilt_from_nadir_deg`` come
    from ``config/novelty/models.yaml`` and are GUESSED placeholders until
    the delivery camera is calibrated."""

    fx: float
    fy: float
    cx: float
    cy: float
    tilt_from_nadir_deg: float = 0.0

    def _body_basis(self) -> tuple[tuple[float, float, float], ...]:
        theta = math.radians(self.tilt_from_nadir_deg)
        f = (math.sin(theta), 0.0, -math.cos(theta))
        r = (0.0, -1.0, 0.0)
        d = (
            f[1] * r[2] - f[2] * r[1],
            f[2] * r[0] - f[0] * r[2],
            f[0] * r[1] - f[1] * r[0],
        )
        return f, r, d

    def pixel_to_ground(
        self, px: float, py: float, altitude_m: float, frame_shape: tuple[int, int]
    ) -> GroundPoint | None:
        if altitude_m <= 0:
            return None
        rx = (px - self.cx) / self.fx
        ry = (py - self.cy) / self.fy
        f, r, d = self._body_basis()
        dir_body = (
            rx * r[0] + ry * d[0] + 1.0 * f[0],
            rx * r[1] + ry * d[1] + 1.0 * f[1],
            rx * r[2] + ry * d[2] + 1.0 * f[2],
        )
        if dir_body[2] >= 0.0:
            return None  # ray points at/above the horizon - no ground hit
        t = -altitude_m / dir_body[2]
        return GroundPoint(x_m=t * dir_body[0], y_m=t * dir_body[1])

    def velocity_mps(
        self,
        px_prev: tuple[float, float],
        px_now: tuple[float, float],
        dt_s: float,
        altitude_m: float,
        frame_shape: tuple[int, int],
    ) -> float | None:
        """Convenience used by motion_monitor.py: displacement rate between
        two pixel centroids, in m/s. Returns None if either projection
        misses the ground or dt_s is non-positive."""
        if dt_s <= 0:
            return None
        p0 = self.pixel_to_ground(px_prev[0], px_prev[1], altitude_m, frame_shape)
        p1 = self.pixel_to_ground(px_now[0], px_now[1], altitude_m, frame_shape)
        if p0 is None or p1 is None:
            return None
        return p0.distance_to(p1) / dt_s

    def pixels_to_ground(
        self, px: np.ndarray, py: np.ndarray, altitude_m: float, frame_shape: tuple[int, int]
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        px = np.asarray(px, dtype=float)
        py = np.asarray(py, dtype=float)
        if altitude_m <= 0:
            zeros = np.zeros_like(px)
            return zeros, zeros, np.zeros_like(px, dtype=bool)
        rx = (px - self.cx) / self.fx
        ry = (py - self.cy) / self.fy
        f, r, d = self._body_basis()
        dir_x = rx * r[0] + ry * d[0] + f[0]
        dir_y = rx * r[1] + ry * d[1] + f[1]
        dir_z = rx * r[2] + ry * d[2] + f[2]
        valid = dir_z < 0.0
        # Avoid dividing by zero for invalid rays; their outputs are masked
        # out below regardless of what safe_dir_z produces.
        safe_dir_z = np.where(valid, dir_z, -1.0)
        t = -altitude_m / safe_dir_z
        x_m = np.where(valid, t * dir_x, 0.0)
        y_m = np.where(valid, t * dir_y, 0.0)
        return x_m, y_m, valid
