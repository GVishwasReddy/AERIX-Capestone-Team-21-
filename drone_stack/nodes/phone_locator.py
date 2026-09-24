"""Where is the recipient's phone? BLE RSSI + the phone's own GPS, fused.

Requirement (2026-09-24): with two or more people under the aircraft, use the
phone's BLE signal to find the one who ordered the parcel.

What each source can and cannot do - measured expectations, not hopes:

* The phone's GPS (HMAC-signed, written by the app for the micro-geofence) is
  unbiased-ish but coarse: 3-5 m on a good day, and correlated from one fix
  to the next, so averaging many fixes does NOT shrink the error much.
* RSSI of the live BLE link is fine-grained but noisy: +/-6 dB from body
  shadowing, grip and the aircraft's own yaw, and its absolute level depends
  on the phone model. One reading says almost nothing about position. What
  carries information is how it CHANGES as the aircraft moves - which is why
  the drop-point scan flies an orbit.

So neither is trusted alone. The estimate is a Bayesian grid posterior over
ground positions around the drop point:

    prior      N(drop point, prior_sigma)      the order said "here"
    x GPS      N(mean recent fix, gps_sigma)   one term, not one per fix
    x RSSI     log-distance path loss with a reference level A that is
               solved per cell in closed form under a weak prior
               (ref_dbm +/- ref_sigma_db) - a quiet phone or a phone in a
               pocket mostly shifts A, not the position.

Samples are binned by where the aircraft was, and each bin's error is modelled
as shadowing shared by the whole bin plus sample noise that averages down:
RSSI noise is correlated, and 40 readings from one spot are not 40
independent measurements. With the aircraft parked, every sample lands in one
bin and the RSSI term correctly contributes nothing - A absorbs it.

The posterior's spread is reported as ``std_m``. Callers are expected to act
on the estimate only when that is small enough to matter; a GPS-only estimate
is honest about being ~5 m wide and will not pick between two people 2 m
apart.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np


# --------------------------------------------------------------------------- #
# Camera geometry (downward-tilted camera, flat ground)
# --------------------------------------------------------------------------- #
# Frames: body forward/right (metres, horizontal), camera "normalised image
# plane" nx (image right) / ny (image up), i.e. the tangent of the angle off
# the optical axis. Image up is body forward because the camera pitches down
# about the body's lateral axis on the AUX6 tilt servo. pitch_from_nadir_deg
# is how far the optical axis sits FORWARD of straight down.

def ground_to_image(forward_m: float, right_m: float, height_m: float,
                    pitch_from_nadir_deg: float) -> tuple[float, float, float] | None:
    """Project a point ``height_m`` below the camera onto the image plane.

    Returns (nx, ny, depth_m), or None if the point is behind the camera.
    """
    th = math.radians(pitch_from_nadir_deg)
    depth = height_m * math.cos(th) + forward_m * math.sin(th)
    if depth <= 0.05:
        return None
    up = forward_m * math.cos(th) - height_m * math.sin(th)
    return right_m / depth, up / depth, depth


def image_to_ground(nx: float, ny: float, height_m: float,
                    pitch_from_nadir_deg: float) -> tuple[float, float] | None:
    """Inverse of ground_to_image: which ground point (forward, right) is seen
    at (nx, ny)? None when that ray never comes down (above the horizon)."""
    th = math.radians(pitch_from_nadir_deg)
    # Ray in (forward, right, down) for camera coords (right=nx, up=ny, axis=1).
    down = math.cos(th) - ny * math.sin(th)
    if down <= 1e-3:
        return None
    forward = math.sin(th) + ny * math.cos(th)
    s = height_m / down
    return forward * s, nx * s


def body_to_enu(forward_m: float, right_m: float, yaw_rad: float) -> tuple[float, float]:
    """Body (forward, right) -> ENU (east, north); yaw 0 = North, clockwise."""
    s, c = math.sin(yaw_rad), math.cos(yaw_rad)
    return forward_m * s + right_m * c, forward_m * c - right_m * s


def enu_to_body_fr(east_m: float, north_m: float, yaw_rad: float) -> tuple[float, float]:
    """ENU (east, north) -> body (forward, right); inverse of body_to_enu."""
    s, c = math.sin(yaw_rad), math.cos(yaw_rad)
    return east_m * s + north_m * c, east_m * c - north_m * s


# --------------------------------------------------------------------------- #
# Estimator
# --------------------------------------------------------------------------- #
@dataclass
class PhoneFix:
    east: float          # local ENU metres (same origin as FusedState: home)
    north: float
    std_m: float         # 1-sigma radius of the posterior
    rssi_bins: int       # distinct aircraft positions RSSI was heard from
    gps_fixes: int


class PhoneLocator:
    def __init__(self, *, grid_radius_m: float = 12.0, grid_step_m: float = 0.5,
                 path_loss_n: float = 2.0, rssi_sigma_db: float = 6.0,
                 gps_sigma_m: float = 5.0, prior_sigma_m: float = 8.0,
                 phone_height_m: float = 1.2, bin_m: float = 0.75,
                 shadow_sigma_db: float = 4.5, sample_ttl_s: float = 180.0,
                 max_samples: int = 2000, use_rssi: bool = True,
                 ref_dbm: float = -58.0, ref_sigma_db: float = 10.0) -> None:
        self.grid_radius_m = float(grid_radius_m)
        self.grid_step_m = max(0.1, float(grid_step_m))
        self.path_loss_n = float(path_loss_n)
        self.rssi_sigma_db = max(0.5, float(rssi_sigma_db))
        self.gps_sigma_m = max(0.5, float(gps_sigma_m))
        self.prior_sigma_m = max(0.5, float(prior_sigma_m))
        self.phone_height_m = float(phone_height_m)
        self.bin_m = max(0.1, float(bin_m))
        self.shadow_sigma_db = max(0.0, float(shadow_sigma_db))
        self.sample_ttl_s = float(sample_ttl_s)
        self.use_rssi = bool(use_rssi)
        self.ref_dbm = float(ref_dbm)
        self.ref_sigma_db = max(1.0, float(ref_sigma_db))
        # (t, rssi, drone_e, drone_n, drone_alt) and (t, e, n)
        self._rssi: deque = deque(maxlen=max(10, int(max_samples)))
        self._gps: deque = deque(maxlen=64)
        r = self.grid_radius_m
        ax = np.arange(-r, r + 1e-9, self.grid_step_m)
        ge, gn = np.meshgrid(ax, ax)
        keep = ge ** 2 + gn ** 2 <= r * r
        self._offsets = np.stack([ge[keep], gn[keep]], axis=1)   # (M, 2)

    def reset(self) -> None:
        self._rssi.clear()
        self._gps.clear()

    def add_rssi(self, rssi_dbm: float, drone_e: float, drone_n: float,
                 drone_alt: float, t: float) -> None:
        if not math.isfinite(rssi_dbm) or rssi_dbm >= 0 or rssi_dbm < -127:
            return                       # 127 = "not available" on HCI
        self._rssi.append((float(t), float(rssi_dbm), float(drone_e),
                           float(drone_n), float(drone_alt)))

    def add_gps(self, east: float, north: float, t: float) -> None:
        self._gps.append((float(t), float(east), float(north)))

    @property
    def rssi_count(self) -> int:
        return len(self._rssi)

    def _bins(self, now: float) -> list[tuple[float, float, float, float, int]]:
        """(east, north, alt, mean rssi, n samples) per aircraft-position cell."""
        cells: dict = {}
        for t, r, e, n, a in self._rssi:
            if now - t > self.sample_ttl_s:
                continue
            key = (round(e / self.bin_m), round(n / self.bin_m), round(a / self.bin_m))
            c = cells.setdefault(key, [0.0, 0.0, 0.0, 0.0, 0])
            c[0] += e; c[1] += n; c[2] += a; c[3] += r; c[4] += 1
        return [(c[0] / c[4], c[1] / c[4], c[2] / c[4], c[3] / c[4], c[4])
                for c in cells.values()]

    def _log_post(self, pts: np.ndarray, origin_e: float, origin_n: float,
                  now: float) -> tuple[np.ndarray, int, int]:
        """Unnormalised log posterior at ``pts`` (M, 2) ENU. Also returns the
        number of RSSI bins and GPS fixes that went into it."""
        gps = [(e, n) for t, e, n in self._gps if now - t <= self.sample_ttl_s]
        bins = self._bins(now) if self.use_rssi else []
        off = pts - np.array([origin_e, origin_n])
        logp = -0.5 * np.sum(off ** 2, axis=1) / self.prior_sigma_m ** 2
        if gps:
            g = np.array(gps[-5:]).mean(axis=0)
            logp = logp - 0.5 * np.sum((pts - g) ** 2, axis=1) / self.gps_sigma_m ** 2
        if len(bins) >= 2:
            b = np.array(bins)                       # (K, 5)
            dh = np.maximum(b[:, 2] - self.phone_height_m, 0.3)
            d2 = ((pts[:, None, 0] - b[None, :, 0]) ** 2
                  + (pts[:, None, 1] - b[None, :, 1]) ** 2 + dh[None, :] ** 2)
            loss = 5.0 * self.path_loss_n * np.log10(np.maximum(d2, 0.09))  # 10n*log10(d)
            # A bin mean's error: shadowing shared by every sample taken from
            # that spot (a body or a hand between phone and aircraft), plus
            # sample noise that averages down. Without the shared term, 40
            # readings from one spot count as 40 independent looks and the
            # test goes overconfident: 11-17% wrong picks at 5 dB shadowing
            # in simulation, vs ~3% with it.
            w = 1.0 / (self.shadow_sigma_db ** 2 + self.rssi_sigma_db ** 2 / b[:, 4])
            wa = 1.0 / self.ref_sigma_db ** 2
            # Reference level A (RSSI at 1 m), solved per point as a MAP
            # estimate under a weak prior. Left fully free, A trades against
            # range: a phone 6 m out along the right bearing fits almost as
            # well as one 2 m out. The prior only rules out implausibly loud
            # or quiet phones; it does not make range well determined.
            a_hat = ((w * (b[:, 3] + loss)).sum(axis=1) + wa * self.ref_dbm) / (w.sum() + wa)
            resid = b[None, :, 3] - a_hat[:, None] + loss
            logp = logp - 0.5 * ((w * resid ** 2).sum(axis=1) + wa * (a_hat - self.ref_dbm) ** 2)
        return logp, len(bins), len(gps)

    def estimate(self, origin_e: float, origin_n: float, now: float) -> PhoneFix | None:
        """Posterior mean and spread over a grid centred on ``origin`` (the
        drop point). None when there is nothing at all to go on.

        Measured on synthetic orbits: the BEARING to the phone comes out well,
        the RANGE does not (the posterior is a banana along the bearing, and
        its mean sits long). Use this to steer the search toward the phone,
        and ``score_candidates`` to choose between people actually seen."""
        n_gps = sum(1 for t, _, _ in self._gps if now - t <= self.sample_ttl_s)
        if not n_gps and (not self.use_rssi or len(self._bins(now)) < 2):
            return None
        grid = self._offsets + np.array([origin_e, origin_n])
        logp, n_bins, n_gps = self._log_post(grid, origin_e, origin_n, now)
        p = np.exp(logp - logp.max())
        p /= p.sum()
        mean = p @ grid
        var = p @ np.sum((grid - mean) ** 2, axis=1)
        return PhoneFix(east=float(mean[0]), north=float(mean[1]),
                        std_m=float(math.sqrt(max(var, 0.0) / 2.0)),
                        rssi_bins=n_bins, gps_fixes=n_gps)

    def score_candidates(self, points: list[tuple[float, float]], origin_e: float,
                         origin_n: float, now: float) -> list[float] | None:
        """Which of these ground points (people the camera sees) holds the
        phone? Returns a probability per point, summing to 1, or None when
        there is no evidence beyond the prior.

        This is a discrete hypothesis test, and it is much stronger than the
        grid estimate: two people a couple of metres apart see clearly
        different RSSI trends across an orbit, even when the phone's absolute
        range is ambiguous."""
        if not points:
            return None
        n_gps = sum(1 for t, _, _ in self._gps if now - t <= self.sample_ttl_s)
        if not n_gps and (not self.use_rssi or len(self._bins(now)) < 2):
            return None
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        logp, _, _ = self._log_post(pts, origin_e, origin_n, now)
        p = np.exp(logp - logp.max())
        return [float(v) for v in p / p.sum()]
