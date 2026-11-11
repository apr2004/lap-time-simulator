from dataclasses import dataclass
import numpy as np

@dataclass
class TrackModel:
        distance: np.ndarray
        curvature: np.ndarray

        # calls the method like if it was an attribute without ()
        @property
        def total_lenght(self) -> float:
                return float(self.distance[-1])
        
        @property
        def n_points(self) -> int:
                return len(self.distance)

        @classmethod
        def create_simple_track(cls, step_size: float = 1.0) -> "TrackModel":
                distance = np.arange(0, 501, step_size)
                curvature = np.zeros(len(distance))

                mask_turn = (distance >= 200) & (distance <= 300)
                curvature[mask_turn] = 1.0 / 50.0
                return cls(distance=distance, curvature=curvature)