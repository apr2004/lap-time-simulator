from dataclasses import dataclass

@dataclass
class Tires:
        mu: float

        def max_grip(self, total_vertical_load: float) -> float:
                return self.mu * total_vertical_load