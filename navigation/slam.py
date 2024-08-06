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

    def _slam(self, camera_positions: List[Vec3], camera_quats: List[Vec3]) -> None:
        if self.old_coord is None:
            self.old_coord = camera_positions[0]
            self.old_rot = camera_quats[0]

        average_position = np.mean(camera_positions, axis=0)
        average_quat = np.mean(camera_quats, axis=0)
        filtered_position = average_position * 0.1 + self.old_coord * 0.9
        filtered_quat = average_quat * 0.1 + self.old_rot * 0.9
        self.old_coord = filtered_position
        self.old_rot = filtered_quat
        ## print quat as euler
        r = R.from_quat(filtered_quat)
        euler_angles = r.as_euler('xyz', degrees=True)

        yaw = euler_angles[1]

        self.camera.position = filtered_position
        self.camera.angles = Vec3(x=euler_angles[0], y=yaw, z=euler_angles[2])

        rr.log(
            "world/cam",
            rr.Transform3D(
                translation=filtered_position,
                rotation=rr.Quaternion(xyzw=filtered_quat),
            ),
        )

    def detect(self) -> None:
        markers = self.camera.get_frame_markers()
        if not markers:
            return
        frame, corners, ids = markers

        try:
            rvecs, tvecs, _ = estimate_pose_single_markers(
                corners, self.world.tags_size, self.camera.matrix, self.camera.distortion, ids
            )
        except cv2.error:
            return

        if len(rvecs) != len(ids) or len(tvecs) != len(ids):
            return

        for i in range(len(ids)):
            marker_id = ids[i][0]
            if marker_id not in self.world.absolute_tags:
                continue
            cv2.drawFrameAxes(
                frame, self.camera.matrix, self.camera.distortion, rvecs[i], tvecs[i], 0.1, 1
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

            camera_quats = []
            camera_positions = []
            # Tag with known position
            for tag in self.world.tags:
                if tag.id != marker_id:
                    continue

                camera_pos, camera_quat = get_camera_position_from_estimations(rvecs[i], tvecs[i], tag.position)
                camera_positions.append(camera_pos)
                camera_quats.append(camera_quat)

            self._slam(camera_positions, camera_quats)

        cv2.imshow("Aruco Marker Detection", frame)
