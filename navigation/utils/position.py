import cv2
import numpy as np
from typing import List, Tuple, Dict
from utils.vector import Vec3
from scipy.spatial.transform import Rotation as R

# This is just a helper function to estimate the pose of a single marker
def estimate_pose_single_markers(
    corners: List[np.ndarray],
    sizes: Dict[int, float],
    mtx: np.ndarray,
    distortion: np.ndarray,
    ids: List[int],
) -> Tuple[List[np.ndarray], List[np.ndarray], List]:
    """
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    Thanks: https://stackoverflow.com/questions/76802576/how-to-estimate-pose-of-single-marker-in-opencv-python-4-8-0
    """
    rvecs = []
    tvecs = []
    trash = []
    for i, c in enumerate(corners):
        marker_size = 0.36
        if ids[i][0] in sizes:
            marker_size = sizes[ids[i][0]]
        marker_points = np.array(
        [
            [-marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, -marker_size / 2, 0],
            [-marker_size / 2, -marker_size / 2, 0],
        ],
            dtype=np.float32,
        )
        _, R, t = cv2.solvePnP(marker_points, c, mtx, distortion)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(_)
    return rvecs, tvecs, trash

def get_positioning_from_estimation(rvec: np.ndarray, tvec: np.ndarray) -> Tuple[float, Vec3]:
    distance = np.linalg.norm(tvec)
    rmat, _ = cv2.Rodrigues(rvec)
    proj_matrix = np.hstack((rmat, tvec))
    _, _, _, _, _, _, angles = cv2.decomposeProjectionMatrix(proj_matrix)
    pitch, yaw, row = angles.flatten()

    return distance, Vec3(x=pitch, y=yaw, z=row)

def get_camera_position_from_estimations(rvec: np.ndarray, tvec: np.ndarray, tag_position: Vec3) -> Tuple[np.ndarray, np.ndarray]:
    position = tvec.flatten()

    # Store the rotation information
    rotation_matrix = np.eye(4)
    rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvec))[0]
    r = R.from_matrix(rotation_matrix[0:3, 0:3])
    quat = r.as_quat()
    camera_position = (
        -np.dot(rotation_matrix[0:3, 0:3].T, position)
        + tag_position.to_array()
    )
    camera_quat = r.inv().as_quat()

    return camera_position, camera_quat
