import numpy as np
import pytest

from src.vehicle.aerodynamics import Aerodynamics
from src.vehicle.powertrain import Powertrain
from src.vehicle.tires import Tires
from src.vehicle.brakes import Brakes
from src.vehicle.vehicle_model import VehicleModel
from src.track.track_model import TrackModel
from src.solver.quasi_static import (
    calculate_cornering_limits,
    forward_pass,
    backward_pass,
    solver,
)


@pytest.fixture
def car_and_track():
    # data from Formula SAE
    aero = Aerodynamics(0.3, 0.6, 0.5, 1.225)
    power = Powertrain(50000)
    tires = Tires(1)
    brakes = Brakes(8000)

    car = VehicleModel(
        250,
        320 / 3.6,
        aero,
        power,
        tires,
        brakes,
    )

    track = TrackModel.create_simple_track(1)

    return car, track

def test_cornering_limits(car_and_track):
    car, track = car_and_track

    limits = calculate_cornering_limits(car, track)

    assert len(limits) == track.n_points
    assert np.all(limits >= 0)
    assert np.all(limits <= car.v_max)

    straight_points = track.curvature == 0
    assert np.all(limits[straight_points] == car.v_max)

def test_forward_pass_respects_limits(car_and_track):
    car, track = car_and_track

    limits = calculate_cornering_limits(car, track)
    v_forward = forward_pass(car, track, 0.0, limits)

    assert len(v_forward) == track.n_points
    assert np.all(v_forward >= 0)
    assert np.all(v_forward <= limits)

def test_backward_pass_respects_limits(car_and_track):
    car, track = car_and_track

    limits = calculate_cornering_limits(car, track)
    v_backward = backward_pass(car, track, limits)

    assert len(v_backward) == track.n_points
    assert np.all(v_backward >= 0)
    assert np.all(v_backward <= limits)

def test_solver_returns_lap_time_and_profile(car_and_track):
    car, track = car_and_track

    lap_time, v_final = solver(car, track, 0.0)

    assert lap_time > 0
    assert len(v_final) == track.n_points
    assert np.all(v_final >= 0)
    assert np.all(v_final <= car.v_max)