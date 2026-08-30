import math

import pytest

from mission_validator import (
    haversine_distance_m,
    validate_altitude,
    validate_delivery_request,
    validate_geofence,
    validate_lat_lon_bounds,
)

# Canberra SITL default home.
HOME_LAT, HOME_LON = -35.363262, 149.165237


class TestLatLonBounds:
    def test_accepts_valid_coordinates(self):
        assert validate_lat_lon_bounds(12.9716, 77.5946).ok

    def test_accepts_extremes(self):
        assert validate_lat_lon_bounds(90.0, 180.0).ok
        assert validate_lat_lon_bounds(-90.0, -180.0).ok

    @pytest.mark.parametrize("lat,lon", [(91.0, 0.0), (-90.1, 0.0)])
    def test_rejects_out_of_range_latitude(self, lat, lon):
        result = validate_lat_lon_bounds(lat, lon)
        assert not result.ok
        assert "latitude" in result.reason

    @pytest.mark.parametrize("lat,lon", [(0.0, 181.0), (0.0, -180.5)])
    def test_rejects_out_of_range_longitude(self, lat, lon):
        result = validate_lat_lon_bounds(lat, lon)
        assert not result.ok
        assert "longitude" in result.reason

    def test_rejects_nan(self):
        assert not validate_lat_lon_bounds(float("nan"), 0.0).ok

    def test_rejects_non_numeric(self):
        assert not validate_lat_lon_bounds("12.97", 77.59).ok


class TestHaversine:
    def test_zero_distance(self):
        assert haversine_distance_m(HOME_LAT, HOME_LON, HOME_LAT, HOME_LON) == 0.0

    def test_known_distance_one_degree_latitude(self):
        # One degree of latitude is ~111.19 km anywhere on the globe.
        distance = haversine_distance_m(0.0, 0.0, 1.0, 0.0)
        assert math.isclose(distance, 111_195, rel_tol=0.001)

    def test_symmetric(self):
        a = haversine_distance_m(HOME_LAT, HOME_LON, HOME_LAT + 0.01, HOME_LON + 0.01)
        b = haversine_distance_m(HOME_LAT + 0.01, HOME_LON + 0.01, HOME_LAT, HOME_LON)
        assert math.isclose(a, b)


class TestGeofence:
    def test_accepts_point_inside(self):
        # ~111m north of home.
        assert validate_geofence(HOME_LAT + 0.001, HOME_LON, HOME_LAT, HOME_LON, 300).ok

    def test_rejects_point_outside(self):
        # ~1.1km north of home, geofence is 300m.
        result = validate_geofence(HOME_LAT + 0.01, HOME_LON, HOME_LAT, HOME_LON, 300)
        assert not result.ok
        assert "geofence" in result.reason

    def test_boundary_is_inclusive(self):
        radius = haversine_distance_m(HOME_LAT, HOME_LON, HOME_LAT + 0.001, HOME_LON)
        assert validate_geofence(HOME_LAT + 0.001, HOME_LON, HOME_LAT, HOME_LON, radius).ok


class TestAltitude:
    def test_accepts_in_range(self):
        assert validate_altitude(15.0, 2.0, 50.0).ok

    def test_rejects_too_low(self):
        assert not validate_altitude(1.0, 2.0, 50.0).ok

    def test_rejects_too_high(self):
        result = validate_altitude(120.0, 2.0, 50.0)
        assert not result.ok
        assert "altitude" in result.reason

    def test_boundaries_inclusive(self):
        assert validate_altitude(2.0, 2.0, 50.0).ok
        assert validate_altitude(50.0, 2.0, 50.0).ok


class TestFullValidation:
    def _validate(self, lat, lon, alt):
        return validate_delivery_request(
            lat, lon, alt, HOME_LAT, HOME_LON,
            geofence_radius_m=300.0, min_altitude_m=2.0, max_altitude_m=50.0,
        )

    def test_accepts_good_request(self):
        assert self._validate(HOME_LAT + 0.001, HOME_LON, 15.0).ok

    def test_rejects_bad_coordinates_first(self):
        # Bad lat AND out of geofence — coordinate bounds should be reported.
        result = self._validate(999.0, HOME_LON, 15.0)
        assert not result.ok
        assert "latitude" in result.reason

    def test_rejects_far_away_destination(self):
        result = self._validate(12.9716, 77.5946, 15.0)
        assert not result.ok
        assert "geofence" in result.reason

    def test_rejects_bad_altitude(self):
        result = self._validate(HOME_LAT + 0.001, HOME_LON, 200.0)
        assert not result.ok
        assert "altitude" in result.reason
