"""sight.py: where no ring meets a car body, by pitch."""
import math

from foscar_lidar.sight import blind_bands, pitch_deg


def plane_for(pitch):
    return (-math.tan(math.radians(pitch)), 0., 0.)


def test_steady_pitch_sees_cars_out_to_about_ninety_metres():
    bands = blind_bands(plane_for(-.16), 2., 2.11)
    assert all(lo > 85. for lo, _ in bands)


def test_nose_up_from_hard_acceleration_hides_a_car_from_about_44_m_to_the_end():
    bands = blind_bands(plane_for(.6), 2., 2.11)
    assert any(40. < lo < 50. and hi >= 99.5 for lo, hi in bands)


def test_a_gap_shorter_than_a_car_is_not_a_blind_band():
    assert blind_bands(plane_for(0.), 2., 2.11, min_length=7.) == []


def test_pitch_sign():
    assert pitch_deg(plane_for(.5)) > 0 and pitch_deg(None) == 0.
