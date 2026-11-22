from src.track.track_model import TrackModel
from src.vehicle.vehicle_model import VehicleModel
import numpy as np

def calculate_cornering_limits(car: VehicleModel, track: TrackModel) -> np.ndarray:
        cornering_limits = np.zeros(track.n_points)

        mask_straights = (track.curvature == 0)
        cornering_limits[mask_straights] = car.v_max

        mask_turns = (abs(track.curvature) > 0)
        k = track.curvature[mask_turns]
        mu = car.tires.mu
        m = car.mass
        g = 9.81
        rho = car.aero.air_density
        cl = car.aero.cl
        area = car.aero.frontal_area

        # equation on paper
        num = (mu*m*g)
        den = ((m*k)-(0.5*rho*cl*area*mu))

        limit = den > 0

        # assumption of max velocity in every point
        v = np.full_like(k, car.v_max)

        v[limit] = np.sqrt(num/den[limit])
        v = np.minimum(v, car.v_max)

        cornering_limits[mask_turns] = v

        return cornering_limits

def forward_pass(car: VehicleModel, track: TrackModel, v_o: float, limits: np.ndarray) -> np.ndarray:
        v_forward = np.zeros(track.n_points)

        v_forward[0] = v_o

        for i in range(track.n_points - 1):
                F = car.max_acceleration_force(v_forward[i])
                a = F / car.mass

                delta = track.distance[i+1] - track.distance[i]
                v_sqrt = ((v_forward[i])**2) + 2*a*delta
                v = np.sqrt(max(0.0, v_sqrt))
                v_forward[i+1] = min(v,limits[i+1])

        return v_forward

def backward_pass(car: VehicleModel, track: TrackModel, limits: np.ndarray) -> np.ndarray:
        v_backward = np.zeros(track.n_points)

        v_backward[-1] = limits[-1]

        for i in range(track.n_points - 1, 0, -1):
                F = car.max_braking_force(v_backward[i])
                a = F / car.mass

                delta = track.distance[i] - track.distance[i-1]
                v_sqrt = ((v_backward[i])**2) + 2*a*delta
                v = np.sqrt(max(0.0, v_sqrt))
                v_backward[i-1] = min(v, limits[i-1])

        return v_backward

def solver(car: VehicleModel, track: TrackModel, v_o: float):
        corner_limits = calculate_cornering_limits(car, track)
        v_forward = forward_pass(car, track, v_o, corner_limits)
        v_backward = backward_pass(car, track, corner_limits)

        v_final = np.minimum.reduce([
                v_forward,
                v_backward,
                corner_limits
        ])

        delta_s = np.diff(track.distance)
        v_avg = 0.5*(v_final[:-1] + v_final[1:])
        dt = delta_s/np.maximum(v_avg,0.01)

        lap_time = np.sum(dt)

        return lap_time, v_final