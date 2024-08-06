from functools import wraps
from pymavlink import mavutil
from utils.vector import Vec3


class Vehicle:
    def __init__(self, vehicle_connection: str) -> None:
        # Create the connection
        self.master = mavutil.mavlink_connection(vehicle_connection)
        # Wait a heartbeat before sending commands
        self.master.wait_heartbeat()
        self.target_position: Vec3 = Vec3(x=0.34, y=0.2, z=0.54)

    def is_vehicle_armed(self):
        heartbeat = self.master.recv_match(type="HEARTBEAT", blocking=False)
        if heartbeat is not None:
            return heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

    def set_tag_position(self, position: Vec3):
        self.target_position = position

    def force_arm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 21196, 0, 0, 0, 0, 0
        )

    # Make a decorator to make sure vehicle is armed
    def ensure_armed(func):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            if not self.is_vehicle_armed():
                self.force_arm()
            return func(self, *args, **kwargs)
        return wrapper

    @ensure_armed
    def actuate(self, position: Vec3, yaw: float):
        yaw_error = yaw - 0

        x_error = position.x - self.target_position.x
        y_error = position.y - self.target_position.y
        z_error = position.z - self.target_position.z
        # TODO: PIDs and custom gains
        yaw_output = yaw_error * 15
        x_output = x_error * 100
        y_output = y_error * 400
        z_output = z_error * 100
        self.master.mav.manual_control_send(
            self.master.target_system,
            int(z_output), # sideways?
            -int(x_output),
            500 -int(y_output), # 500 means neutral throttle
            int(yaw_output),
            0
        )
