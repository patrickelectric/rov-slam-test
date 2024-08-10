from functools import wraps
from pymavlink import mavutil
from utils.vector import Vec3
import rerun as rr
import numpy as np


class PIDController:
    def __init__(self, kp, ki, kd, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return np.clip(output, -self.max_output, self.max_output)


class Vehicle:
    def __init__(self, vehicle_connection: str) -> None:
        self.master = mavutil.mavlink_connection(vehicle_connection)
        self.send_heartbeat()
        self.master.wait_heartbeat()
        self.target_position = Vec3(x=0.0, y=0.0, z=0.4)

        self.x_pid = PIDController(kp=650, ki=0.1, kd=50, max_output=300)
        self.y_pid = PIDController(kp=700, ki=0.1, kd=50, max_output=300)
        self.z_pid = PIDController(kp=650, ki=0.1, kd=50, max_output=300)
        self.yaw_pid = PIDController(kp=10, ki=0.1, kd=5, max_output=300)

    def send_heartbeat(self):
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0
        )

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

    def ensure_armed(func):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            if not self.is_vehicle_armed():
                self.force_arm()
            return func(self, *args, **kwargs)
        return wrapper

    def manual_control(self, x: int, y: int, z: int, yaw: int):
        self.master.mav.manual_control_send(
            self.master.target_system,
            z, -x, 500 - y, yaw, 0
        )

    @ensure_armed
    def deactivate(self):
        self.manual_control(0, 0, 0, 0)

    def _plot_vectors(self, position: Vec3, output: Vec3, yaw: float) -> None:
        rr.log(
            f"world/target",
            rr.Points3D(
                translation=[self.target_position.x, self.target_position.y, self.target_position.z],
            ),
        )

        target_vector = Vec3(
            x=self.target_position.x - position.x,
            y=self.target_position.y - position.y,
            z=self.target_position.z - position.z,
        ).unit()
        control_vector = Vec3(
            x=output.x,
            y=output.y,
            z=output.z,
        ).unit()

        rr.log(
            f"world/target_vector",
            rr.Arrows3D(
                start=[position.x, position.y, position.z],
                end=[target_vector.x, target_vector.y, target_vector.z],
            ),
        )

        rr.log(
            f"world/control_vector",
            rr.Arrows3D(
                start=[position.x, position.y, position.z],
                end=[control_vector.x, control_vector.y, control_vector.z],
            ),
        )

    @ensure_armed
    def actuate(self, position: Vec3, yaw: float):
        dt = 0.1  # Time step, adjust as necessary for your system

        x_error = position.x - self.target_position.x
        y_error = position.y - self.target_position.y
        z_error = position.z - self.target_position.z
        yaw_error = yaw - 0  # Assuming the target yaw is 0 for simplicity

        x_output = self.x_pid.update(x_error, dt)
        y_output = self.y_pid.update(y_error, dt)
        z_output = self.z_pid.update(z_error, dt)
        yaw_output = self.yaw_pid.update(yaw_error, dt)

        print(f"Yaw: {yaw_output}, X: {x_output}, Y: {y_output}, Z: {z_output}")

        self.manual_control(int(x_output), int(y_output), int(z_output), int(yaw_output))
