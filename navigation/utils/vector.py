import numpy as np
from pydantic import BaseModel


class Vec3(BaseModel):
    # For angles x, y, z are pitch, yaw, roll
    x: float
    y: float
    z: float

    @property
    def raw(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    def to_array(self):
        return np.array([self.x, self.y, self.z])

    @property
    def roll(self) -> float:
        return self.x

    @roll.setter
    def roll(self, value: float):
        self.x = value

    @property
    def pitch(self) -> float:
        return self.y

    @pitch.setter
    def pitch(self, value: float):
        self.y = value

    @property
    def yaw(self) -> float:
        return self.z

    @yaw.setter
    def yaw(self, value: float):
        self.z = value
