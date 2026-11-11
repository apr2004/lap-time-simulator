from dataclasses import dataclass

@dataclass
class Aerodynamics:
    cl: float        
    cd: float        
    frontal_area: float  
    air_density: float = 1.225  

    def downforce(self, velocity: float) -> float:
        return 0.5 * self.air_density * self.cl * self.frontal_area * (velocity ** 2)

    def drag(self, velocity: float) -> float:
        return 0.5 * self.air_density * self.cd * self.frontal_area * (velocity ** 2)