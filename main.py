import cv2
import numpy as np
from typing import Dict, List, Tuple, Union
from pydantic import BaseModel, Field, validator
from pydantic_core import from_json

import rerun as rr

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

class TagData(BaseModel):
    id: int
    # Real world position and angles, not relative
    angles: Angles
    position: Position

OUR_WORLD = [
    {
        "id": 666,
        "position": {
            "x": 0,
            "y": 0,
            "z": 0,
        },
        "angles": {
            "pitch": 0,
            "yaw": 0,
        },
    },
]

KNOWN_TAGS: List[TagData] = [TagData.parse_obj(tag) for tag in OUR_WORLD]
positions: Dict[int, Union[np.ndarray, Tuple[float, Tuple[float, float]]]] = {}
known_position_received: bool = False

# We should consider the vehicle orientation here as well
def compute_position_from(position: Position, angles: Angles, distance: float) -> np.ndarray:
    pitch, yaw = angles.pitch, angles.yaw
    x = position.x - distance * np.cos(np.radians(pitch)) * np.sin(np.radians(yaw))
    y = position.y - distance * np.cos(np.radians(pitch)) * np.cos(np.radians(yaw))
    z = position.z + distance * np.sin(np.radians(pitch))
    return Position(x=x, y=y, z=z)

# DO SLAM
def process_data(data: TagData) -> None:
    global positions
    global known_position_received

    feature_id = data.id
    distance = data.distance
    angles = data.angles

    if feature_id == FEATURE_ID_KNOWN:
        positions[feature_id] = KNOWN_FEATURE_POSITION
        known_position_received = True
        for tag_id in positions:
            if tag_id != FEATURE_ID_KNOWN:
                _id, rel_distance, rel_angles = positions[tag_id]
                positions[tag_id] = compute_position(rel_distance, rel_angles, KNOWN_FEATURE_POSITION)
    else:
        if known_position_received:
            positions[feature_id] = compute_position(distance, angles, KNOWN_FEATURE_POSITION)
        else:
            positions[feature_id] = (distance, angles)

    print(f"Positions: {positions}")

# This is just a helper function to estimate the pose of a single marker
def estimatePoseSingleMarkers(corners: List[np.ndarray], marker_size: float, mtx: np.ndarray, distortion: np.ndarray) -> Tuple[List[np.ndarray], List[np.ndarray], List]:
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    Thanks: https://stackoverflow.com/questions/76802576/how-to-estimate-pose-of-single-marker-in-opencv-python-4-8-0
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    rvecs = []
    tvecs = []
    trash = []
    for c in corners:
        _, R, t = cv2.solvePnP(marker_points, c, mtx, distortion)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(_)
    return rvecs, tvecs, trash

def main():
    CAMERA_ID = 2
    CAMMERA_WIDTH = 1920
    CAMERA_HEIGHT = 1080
    MARKER_SIZE_M = 0.2

    # Camera calibration parameters for the following configuration, check calib.py
    camera_matrix = np.array([[1.49752903e+03, 0.00000000e+00, 9.22278508e+02],
                              [0.00000000e+00, 1.49970497e+03, 5.56573802e+02],
                              [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]], dtype=float)
    dist_coeffs = np.array([[0.0346], [-0.194], [0], [0.0], [0.186]], dtype=float)

    rr.init("Camera and Tag Positions", spawn=True)
    rr.log("world/cam/image", rr.Pinhole(focal_length=300, width=CAMMERA_WIDTH, height=CAMERA_HEIGHT))

    # We should use image_from_camera property here
    for tags in KNOWN_TAGS:
        rr.log(f"world/tag_{tags.id}/image", rr.Pinhole(focal_length=300, width=MARKER_SIZE_M*1000, height=MARKER_SIZE_M*1000))
        rr.log(f"world/tag_{tags.id}", rr.Transform3D(translation=[tags.position.x, tags.position.y, tags.position.z]))

    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    old_coord = None

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        rr.log("world/cam/image", rr.Image(frame))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = estimatePoseSingleMarkers(corners, MARKER_SIZE_M, camera_matrix, dist_coeffs)
            for i in range(len(ids)):
                corner = corners[i][0]
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1, 1)
                distance = np.linalg.norm(tvecs[i])
                rmat, _ = cv2.Rodrigues(rvecs[i])
                proj_matrix = np.hstack((rmat, tvecs[i]))
                _, _, _, _, _, _, angles = cv2.decomposeProjectionMatrix(proj_matrix)
                pitch, yaw, roll = angles.flatten()

                marker_id = ids[i][0]
                text = f"ID: {marker_id}, D:{distance:.2f}m, A: {yaw:.0f}, {pitch:.0f}, {roll:.0f}"
                print(text)
                cv2.putText(frame, text, (int(corner[0][0]), int(corner[0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

                x_min = int(corner[:, 0].min())
                x_max = int(corner[:, 0].max())
                y_min = int(corner[:, 1].min())
                y_max = int(corner[:, 1].max())
                tag_image = frame[y_min:y_max, x_min:x_max]
                rr.log(f"world/tag_{marker_id}/image", rr.Image(tag_image))

                # Tag with known position
                for tag in KNOWN_TAGS:
                    position = compute_position_from(tag.position, Angles(pitch=pitch, yaw=yaw), distance)

                    # poor man's low pass filter
                    if not old_coord:
                        old_coord = position
                    position = old_coord * 0.9 + position * 0.1
                    old_coord = position

                    # Paticks test env: 0.9743828787711633 2.2215185425872606 0.2750221709072308
                    print(position)
                    rr.log("world/cam", rr.Transform3D(translation=position.to_array()))


        cv2.imshow('Aruco Marker Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
