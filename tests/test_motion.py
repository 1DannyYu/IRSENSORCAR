"""Tests for the time-based motion model (``carbot.motion``)."""

import pytest

from carbot.motion import DistanceIntegrator, MotionModel


def test_forward_time():
    model = MotionModel(forward_mps=0.117, spin_degps=53.5)
    assert model.seconds_for_distance(0.10) == pytest.approx(0.10 / 0.117)
    assert model.seconds_for_distance(0.585) == pytest.approx(0.585 / 0.117)


def test_spin_time():
    model = MotionModel(forward_mps=0.117, spin_degps=53.5)
    assert model.seconds_for_angle(90.0) == pytest.approx(90.0 / 53.5)
    assert model.seconds_for_angle(-90.0) == pytest.approx(90.0 / 53.5)
    assert model.seconds_for_angle(270.0) == pytest.approx(270.0 / 53.5)


def test_integrator_distance():
    model = MotionModel(forward_mps=0.100, spin_degps=50.0)
    integ = DistanceIntegrator(model)
    dist = 0.0
    for _ in range(10):
        dist += integ.distance_delta(0.05)
    assert dist == pytest.approx(0.05)  # 10 x 0.05 s at 0.1 m/s


def test_integrator_angle():
    model = MotionModel(forward_mps=0.100, spin_degps=90.0)
    integ = DistanceIntegrator(model)
    ang = 0.0
    for _ in range(10):
        ang += integ.angle_delta(0.1)
    assert ang == pytest.approx(90.0)


def test_invalid_model():
    with pytest.raises(ValueError):
        MotionModel(forward_mps=0.0, spin_degps=53.5).seconds_for_distance(1.0)
    with pytest.raises(ValueError):
        MotionModel(forward_mps=0.117, spin_degps=0.0).seconds_for_angle(90.0)
