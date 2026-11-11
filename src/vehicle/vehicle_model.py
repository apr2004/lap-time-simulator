from dataclasses import dataclass
from src.vehicle.aerodynamics import Aerodynamics
from src.vehicle.powertrain import Powertrain
from src.vehicle.tires import Tires
from src.vehicle.brakes import Brakes

@dataclass
class VehicleModel:
        mass: float  
        aero: Aerodynamics
        powertrain: Powertrain
        tires: Tires
        brakes: Brakes

        def calculate_vertical_load(self, v: float) -> float:
                return (self.mass*9.81) + self.aero.downforce(v)

        def max_lateral_force(self, v: float) -> float:
                return self.tires.max_grip(self.calculate_vertical_load(v))

        def max_acceleration_force(self, v: float) -> float:
                return min(self.powertrain.engine(v), self.max_lateral_force(v)) - self.aero.drag(v)

        def max_braking_force(self, v: float) -> float:
                fz = self.calculate_vertical_load(v)
                f_tire_limit = self.tires.max_grip(fz)
                f_brake_wheels = self.brakes.freno_max(f_tire_limit)
                f_drag = self.aero.drag(v)
                return f_brake_wheels + f_drag