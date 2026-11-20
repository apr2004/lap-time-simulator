from src.vehicle.aerodynamics import Aerodynamics
from src.vehicle.powertrain import Powertrain
from src.vehicle.tires import Tires
from src.vehicle.brakes import Brakes
from src.vehicle.vehicle_model import VehicleModel
from src.track.track_model import TrackModel
from src.solver.quasi_static import *
import numpy as np

# data from Formula SAE
aero = Aerodynamics(0.3, 0.6, 0.5, 1.225)
power = Powertrain(50000)