from dataclasses import dataclass

@dataclass
class Brakes:
        mechanical_limit: float

        def max_braking_force(self, tire_limit: float) -> float:
                return min(self.mechanical_limit, tire_limit)