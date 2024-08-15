import time
import importlib
from vehicle import Vehicle
from utils.vector import Vec3
from typing import Callable, Any

class MissionNavigator:
    @property
    def is_complete(self) -> bool:
        return self.step is None

    def __init__(self, mission_file: str, vehicle: Vehicle) -> None:
        self.vehicle = vehicle
        self.step = importlib.import_module(mission_file).start
        self.delay: float = 0.0
        self.last_time = time.time()
        self.first_execution = True

    def next(self, step: Callable[[Vehicle, Vec3, Vec3, bool, Any], None], delay: float = 0.0) -> None:
        self.step = step
        self.delay = delay
        self.first_execution = True

    def update(self, position: Vec3, rotation: Vec3) -> None:
        if self.step is None or time.time() - self.last_time < self.delay:
            return
        self.delay = 0.0
        self.last_time = time.time()

        step = self.step
        step(self.vehicle, position, rotation, self.first_execution, self.next)

        if step is self.step:
            self.first_execution = False
