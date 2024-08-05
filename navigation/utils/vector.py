import numpy as np
from pydantic import BaseModel

# Roll ? Are you drunk ? Use stabilize mode :)
class Angles(BaseModel):
    pitch: float
    yaw: float

    def to_array(self):
        return np.array([self.pitch, self.yaw])


class Position(BaseModel):
    x: float
    y: float
    z: float

    def to_array(self):
        return np.array([self.x, self.y, self.z])

    def __add__(self, o):
        return Position(x=self.x + o.x, y=self.y + o.y, z=self.z + o.z)

    def __mul__(self, o: float):
        return Position(x=self.x * o, y=self.y * o, z=self.z * o)
