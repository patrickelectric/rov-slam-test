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

    def unit(self) -> "Vec3":
        return Vec3(x=self.x / np.linalg.norm(self.raw), y=self.y / np.linalg.norm(self.raw), z=self.z / np.linalg.norm(self.raw))

    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3(x=self.x + other.x, y=self.y + other.y, z=self.z + other.z)

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(x=self.x - other.x, y=self.y - other.y, z=self.z - other.z)

    def dot(self, other: "Vec3") -> float:
        return np.dot(self.raw, other.raw)

    def __str__(self) -> str:
        return f"x={np.round(self.x, 3)}, y={np.round(self.y, 3)}, z={np.round(self.z, 3)}"
