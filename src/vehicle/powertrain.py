from dataclasses import dataclass

@dataclass
class Powertrain:
        power: float

        def engine(self, velocity: float) -> float:
                return self.power / max(velocity, 0.1)