from functools import wraps
from pymavlink import mavutil
from utils.vector import Vec3
import rerun as rr
import numpy as np
import time

import cv2

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
    x_pid = PIDController(kp=1783, ki=30, kd=2, max_output=900)
    y_pid = PIDController(kp=560, ki=15, kd=2, max_output=500)
    z_pid = PIDController(kp=1783, ki=30, kd=2, max_output=900)
    yaw_pid = PIDController(kp=500, ki=10, kd=8, max_output=300)

    def __init__(self, vehicle_connection: str) -> None:
        self.master = mavutil.mavlink_connection(vehicle_connection)
        self.send_heartbeat()
        self.master.wait_heartbeat()
        self.target_position: Vec3 | None = None
        self.target_yaw: float | None = None
        self.home_position: Vec3 | None = None
        self.home_yaw: float | None = None
        self.average_positioning_error: float = 100000

        cv2.namedWindow('Aruco Marker Detection')
        cv2.createTrackbar("kp_x", "Aruco Marker Detection", 1783, 5000, lambda x: Vehicle.x_pid.__setattr__("kp", x))
        cv2.createTrackbar("ki_x", "Aruco Marker Detection", 30, 100, lambda x: Vehicle.x_pid.__setattr__("ki", x))
        cv2.createTrackbar("kd_x", "Aruco Marker Detection", 2, 100, lambda x: Vehicle.x_pid.__setattr__("kd", x))

        cv2.createTrackbar("kp_y", "Aruco Marker Detection", 560, 5000, lambda x: Vehicle.y_pid.__setattr__("kp", x))
        cv2.createTrackbar("ki_y", "Aruco Marker Detection", 15, 100, lambda x: Vehicle.y_pid.__setattr__("ki", x))
        cv2.createTrackbar("kd_y", "Aruco Marker Detection", 2, 100, lambda x: Vehicle.y_pid.__setattr__("kd", x))

        cv2.createTrackbar("kp_z", "Aruco Marker Detection", 1783, 5000, lambda x: Vehicle.z_pid.__setattr__("kp", x))
        cv2.createTrackbar("ki_z", "Aruco Marker Detection", 30, 100, lambda x: Vehicle.z_pid.__setattr__("ki", x))
        cv2.createTrackbar("kd_z", "Aruco Marker Detection", 2, 100, lambda x: Vehicle.z_pid.__setattr__("kd", x))

        cv2.createTrackbar("kp_yaw", "Aruco Marker Detection", 500, 2000, lambda x: Vehicle.yaw_pid.__setattr__("kp", x))
        cv2.createTrackbar("ki_yaw", "Aruco Marker Detection", 10, 100, lambda x: Vehicle.yaw_pid.__setattr__("ki", x))
        cv2.createTrackbar("kd_yaw", "Aruco Marker Detection", 8, 100, lambda x: Vehicle.yaw_pid.__setattr__("kd", x))

        self.gripper_reset_state(62)

    def __del__(self):
        self.gripper_reset_state(0)

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

    def is_target_stable(self, threshold: float):
        if self.target_position is None:
            return False

        print(f"Average Position Error: {self.average_positioning_error}")

        return np.abs(self.average_positioning_error) <= threshold

    def go_home(self):
        if self.home_position is not None:
            self.move_abs(self.home_position, self.home_yaw)

    def set_home(self, position: Vec3, yaw: float = 0):
        self.home_yaw = yaw
        self.home_position = position

    def gripper_reset_state(self, val: int) -> None:
        self.master.mav.param_set_send(
            self.master.target_system, self.master.target_component,
            b'SERVO9_FUNCTION',
            val,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

    def set_gripper_pwm(self, pwm: int) -> None:
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[12 - 1] = pwm
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_channel_values
        )

    def set_camera_gimbal(self, pitch: int, yaw: int) -> None:
        # Set small pitch rate
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            0,
            pitch, yaw, 0, 0, 0, 0, 0
        )

    def look_down(self) -> None:
        self.set_camera_gimbal(-65, 0)

    def look_forward(self) -> None:
        self.set_camera_gimbal(10, 0)

    def open_gripper(self) -> None:
        self.set_gripper_pwm(2000)

    def close_gripper(self) -> None:
        self.set_gripper_pwm(1000)

    def move_rel(self, position: Vec3, yaw: float = 0):
        if self.target_position is None:
            self.target_position = Vec3(0, 0, 0)
        if self.target_yaw is None:
            self.target_yaw = 0
        self.target_yaw += yaw
        self.target_position += position
        self.average_positioning_error = 100000

    def move_abs(self, position: Vec3, yaw: float = 0):
        self.target_yaw = yaw
        self.target_position = position
        self.average_positioning_error = 100000

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

    @ensure_armed
    def actuate(self, position: Vec3, yaw: float):
        if self.target_position is None:
            return

        rr.log(
            f"world/target",
            rr.Transform3D(
                translation=[self.target_position.x, self.target_position.y, self.target_position.z],
            ),
        )

        #print(f"Target Position: {self.target_position}")
        #print(f"Current Position: {position}")

        dt = 0.1

        x_error = position.x - self.target_position.x
        y_error = position.y - self.target_position.y
        z_error = position.z - self.target_position.z
        yaw_error = self.target_yaw - yaw

        x_output = int(self.x_pid.update(x_error, dt))
        y_output = int(self.y_pid.update(y_error, dt))
        z_output = int(self.z_pid.update(z_error, dt))
        yaw_output = int(self.yaw_pid.update(yaw_error, dt))

        self.average_positioning_error = np.linalg.norm([x_error, y_error, z_error])

        #print(f"Yaw Error: {yaw_error}")
        #print(f"Output: {x_output}, {y_output}, {z_output}, {yaw_output}")

        self.manual_control(x_output, y_output, z_output, 0)
