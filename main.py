import cv2
import numpy as np
from typing import Dict, List, Tuple, Union
from pydantic import BaseModel, Field, validator
from pydantic_core import from_json
from scipy.spatial.transform import Rotation as R
from math import degrees
from pymavlink import mavutil
import rerun as rr
from rerun.datatypes import Angle


target_position = [0.34, 0.2, 0.54]

# Create the connection
master = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
# Wait a heartbeat before sending commands
master.wait_heartbeat()
print("connected")


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
    size: float


OUR_WORLD = [
    {
        "id": 333,
        "position": {
            "x": 0,
            "y": 0,
            "z": 0,
        },
        "angles": {
            "pitch": 0,
            "yaw": 0,
        },
        "size": 0.4,
    },
    {
        "id": 666,
        "position": {
            "x": 0,
            "y": -1,
            "z": 0,
        },
        "angles": {
            "pitch": 0,
            "yaw": 0,
        },
        "size": 0.4,
    },
    {
        "id": 999,
        "position": {
            "x": 2,
            "y": 0,
            "z": 0,
        },
        "angles": {
            "pitch": 0,
            "yaw": 0,
        },
        "size": 0.4,
    },
    {
        "id": 222,
        "position": {
            "x": 0.4,
            "y": -0.15,
            "z": 0,
        },
        "angles": {
            "pitch": 0,
            "yaw": 0,
        },
        "size": 0.1
    },
]

KNOWN_TAGS: List[TagData] = [TagData.parse_obj(tag) for tag in OUR_WORLD]
positions: Dict[int, Union[np.ndarray, Tuple[float, Tuple[float, float]]]] = {}
known_position_received: bool = False


# This is just a helper function to estimate the pose of a single marker
def estimatePoseSingleMarkers(
    corners: List[np.ndarray],
    sizes: float,
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
  
  
def force_arm():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 21196, 0, 0, 0, 0, 0)

def actuate(position, yaw):
    # force_arm()
    error = yaw - 0
    yaw_output = error * 5
    x_error = position[0] - target_position[0]
    y_error = position[1] - target_position[1]
    z_error = position[2] - target_position[2]
    x_output = x_error * 5
    y_output = y_error * 5
    z_output = z_error * 5
    
    #print(f"sending {channel_4}")
    master.mav.manual_control_send(
    master.target_system,
    0,
    0,
    500, # 500 means neutral throttle
    int(yaw_output),
    0)


def main():
    CAMMERA_WIDTH = 1920
    CAMERA_HEIGHT = 1080
    print("armed")
    # Camera calibration parameters remain the same
    camera_matrix = np.array(
        [[334.77497062, 0, 594.55840979], [0, 353.7880701, 322.79266094], [0, 0, 1]],
        dtype=float,
    )
    dist_coeffs = np.array(
        [[0.00758054, -0.00838534, -0.00029538, 0.00270836, 0.00415058]], dtype=float
    )

    rr.init("Camera and Tag Positions", spawn=True)
    rr.log(
        "world/cam/image",
        rr.Pinhole(focal_length=300, width=CAMMERA_WIDTH, height=CAMERA_HEIGHT),
    )

    # Initialize known tags (code remains the same)
    for tags in KNOWN_TAGS:
        rr.log(
            f"world/tag_{tags.id}/image",
            rr.Pinhole(
                focal_length=300,
                width=tags.size * 1000,
                height=tags.size * 1000,
            ),
        )
        rr.log(
            f"world/tag_{tags.id}",
            rr.Transform3D(
                translation=[tags.position.x, tags.position.y, tags.position.z]
            ),
        )

    cap = cv2.VideoCapture("rtsp://127.0.0.1:8554/test")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters()
    parameters.minMarkerPerimeterRate = 0.08
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    old_coord = None
    old_rot = None
    known_ids = [tag.id for tag in KNOWN_TAGS]
    sizes = {tag.id: tag.size for tag in KNOWN_TAGS}

    while cap.isOpened():
        heartbeat = master.recv_match(type="HEARTBEAT", blocking=False)
        if heartbeat is not None:
            armed = heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if not armed:
                print("Arming")
                force_arm()
    
        # Use grab() instead of read()
        if not cap.grab():
            break

        # Retrieve the frame only when we're ready to process it
        ret, frame = cap.retrieve()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = estimatePoseSingleMarkers(
                corners, sizes, camera_matrix, dist_coeffs, ids
            )
            for i in range(len(ids)):
                # Rest of the processing code remains the same
                corner = corners[i][0]
                if ids[i][0] not in known_ids:
                    continue
                cv2.drawFrameAxes(
                    frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1, 1
                )
                distance = np.linalg.norm(tvecs[i])
                rmat, _ = cv2.Rodrigues(rvecs[i])
                proj_matrix = np.hstack((rmat, tvecs[i]))
                _, _, _, _, _, _, angles = cv2.decomposeProjectionMatrix(proj_matrix)
                pitch, yaw, roll = angles.flatten()

                marker_id = ids[i][0]
                text = f"ID: {marker_id}, D:{distance:.2f}m, A: {yaw:.0f}, {pitch:.0f}, {roll:.0f}"
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
                for tag in KNOWN_TAGS:
                    if tag.id != marker_id:
                        continue

                    position = tvecs[i].flatten()

                    # Store the rotation information
                    rotation_matrix = np.eye(4)
                    rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i]))[0]
                    r = R.from_matrix(rotation_matrix[0:3, 0:3])
                    quat = r.as_quat()
                    camera_position = (
                        -np.dot(rotation_matrix[0:3, 0:3].T, position)
                        + tag.position.to_array()
                    )
                    camera_quat = r.inv().as_quat()
                    camera_positions.append(camera_position)
                    camera_quats.append(camera_quat)

                if old_coord is None:
                    old_coord = camera_positions[0]
                    old_rot = camera_quats[0]
                average_position = np.mean(camera_positions, axis=0)
                average_quat = np.mean(camera_quats, axis=0)
                filtered_position = average_position * 0.1 + old_coord * 0.9
                filtered_quat = average_quat * 0.1 + old_rot * 0.9
                old_coord = filtered_position
                old_rot = filtered_quat
                ## print quat as euler
                r = R.from_quat(filtered_quat)
                euler_angles = r.as_euler('xyz', degrees=True)
                
                yaw = euler_angles[1]
                actuate(filtered_position, yaw)
                rr.log(
                    "world/cam",
                    rr.Transform3D(
                        translation=filtered_position,
                        rotation=rr.Quaternion(xyzw=filtered_quat),
                    ),
                )

        cv2.imshow("Aruco Marker Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()