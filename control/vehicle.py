import time
from pymavlink import mavutil


class Vehicle:
    def __init__(self):
        self.mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
        self.last_hearthbeat = time.time()

    def send_hearthbeat(self):
        if self.last_hearthbeat + 1 < time.time():
            self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def send_vision_position_estimate(self, x, y, z, roll, pitch, yaw):
        self.mav.mav.vision_position_estimate_send(0, x, y, z, roll, pitch, yaw)