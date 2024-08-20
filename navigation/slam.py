import cv2
import numpy as np
from typing import List, Optional

from camera import Camera
from utils.position import estimate_pose_single_markers, get_positioning_from_estimation, get_camera_position_from_estimations
from utils.vector import Vec3
from world import World
from scipy.spatial.transform import Rotation as R
import rerun as rr


class SLAM:
    def __init__(self, world: World, camera: Camera) -> None:
        self.camera = camera
        self.world = world
        self.old_coord: Optional[Vec3] = None
        self.old_rot: Optional[Vec3] = None
        self.succeeded = False

    def _slam(self, camera_positions: List[Vec3], camera_quats: List[Vec3]) -> None:
        threshold = 1.0
        filtered_positions = []
        filtered_quats = []
        if self.old_coord is not None:
            for camera_pos, camera_quat in zip(camera_positions, camera_quats):
                distance = np.linalg.norm(np.array(camera_pos) - np.array(self.old_coord))

                if distance <= threshold:
                    filtered_positions.append(camera_pos)
                    filtered_quats.append(camera_quat)
            if not filtered_positions:
                filtered_positions = camera_positions
                filtered_quats = camera_quats
        else:
            filtered_positions = camera_positions
            filtered_quats = camera_quats

        average_position = np.average(filtered_positions, axis=0)
        average_quat = np.average(filtered_quats, axis=0)

        if self.old_coord is None:
            self.old_coord = average_position
            self.old_rot = average_quat

        filtered_position = average_position * 0.1 + self.old_coord * 0.9
        filtered_quat = average_quat * 0.1 + self.old_rot * 0.9
        self.old_coord = filtered_position
        self.old_rot = filtered_quat
        ## print quat as euler
        r = R.from_quat(filtered_quat)
        euler_angles = r.as_euler('xyz', degrees=True)

        yaw = euler_angles[1]

        self.camera.position = Vec3(x=filtered_position[0], y=filtered_position[1], z=filtered_position[2])
        self.camera.rotation = Vec3(x=euler_angles[0], y=yaw, z=euler_angles[2])

        rr.log(
            "world/cam",
            rr.Transform3D(
                translation=filtered_position,
                rotation=rr.Quaternion(xyzw=filtered_quat),
            ),
        )

    def detect(self) -> None:
        frame, corners, ids = self.camera.get_frame_markers()
        if corners is None or ids is None:
            cv2.imshow("Aruco Marker Detection", frame)
            return

        try:
            rvecs, tvecs, _ = estimate_pose_single_markers(
                corners, self.world.tags_size, self.camera.matrix, np.zeros((1,5)), ids
            )
        except cv2.error:
            return

        if len(ids) == 0 or len(rvecs) != len(ids) or len(tvecs) != len(ids):
            self.succeeded = False
            return

        camera_quats = []
        camera_positions = []
        for i in range(len(ids)):
            marker_id = ids[i][0]
            if marker_id not in self.world.absolute_tags:
                continue
            cv2.drawFrameAxes(
                frame, self.camera.matrix, np.zeros((1,5)), rvecs[i], tvecs[i], 0.1, 1
            )
            distance, angles = get_positioning_from_estimation(rvecs[i], tvecs[i])
            # Rest of the processing code remains the same
            corner = corners[i][0]

            text = f"ID: {marker_id}, D:{distance:.2f}m, A: {angles.yaw:.0f}, {angles.pitch:.0f}"
            cv2.putText(
                frame,
                text,
                (int(corner[0][0]), int(corner[0][1]) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 0, 0),
                1,
            )

            world_tag = self.world.get_abs_tag(marker_id)

            if world_tag is None:
                continue

            camera_pos, camera_quat = get_camera_position_from_estimations(rvecs[i], tvecs[i], world_tag.position)
            camera_positions.append(camera_pos)
            camera_quats.append(camera_quat)

        if len(camera_positions) == 0 or len(camera_quats) == 0:
            return

        self._slam(camera_positions, camera_quats)
        self.succeeded = True
        cv2.imshow("Aruco Marker Detection", frame)
