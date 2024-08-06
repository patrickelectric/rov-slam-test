import time
import json
import os
from typing import List, Tuple, Optional
import rerun as rr
import cv2
import numpy as np
from pydantic import BaseModel
from utils.vector import Vec3

DETECTION_RATE_S = 0.2

class CameraResolution(BaseModel):
    width: int
    height: int


class CameraConfiguration(BaseModel):
    id: int | str
    resolution: CameraResolution
    camera_matrix: List[List[float]]
    distortion_coefficients: List[List[float]]
    rotation_vectors: List[List[List[float]]]
    translation_vectors: List[List[List[float]]]


class Camera:
    @property
    def matrix(self) -> np.ndarray:
        return np.array(self.configuration.camera_matrix)

    @property
    def distortion(self) -> np.ndarray:
        return np.array(self.configuration.distortion_coefficients)

    def __init__(self, configuration_file: str) -> None:
        self.last_frame = time.time()
        try:
            with open(configuration_file, "r") as file:
                self.configuration = CameraConfiguration(**json.load(file))
        except (FileNotFoundError, json.JSONDecodeError) as e:
            raise ValueError(f"Error reading configuration file: {e}")

        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"

        self.capture: Optional[cv2.VideoCapture] = None
        print(f"Camera Source: {self.configuration.id}")
        self.capture = cv2.VideoCapture(self.configuration.id, cv2.CAP_FFMPEG)
        if not self.capture.isOpened():
            raise ValueError("Error opening video capture")

        rr.init("Camera Position", spawn=True)
        rr.log(
            "world/cam/image",
            rr.Pinhole(
                focal_length=300,
                width=self.configuration.resolution.width,
                height=self.configuration.resolution.height
            ),
        )

        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'H264'))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.configuration.resolution.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.configuration.resolution.height)

        self.optimal_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            self.matrix,
            self.distortion,
            (self.configuration.resolution.width, self.configuration.resolution.height),
            1,
            (self.configuration.resolution.width, self.configuration.resolution.height)
        )

        parameters = cv2.aruco.DetectorParameters()
        parameters.minMarkerPerimeterRate = 0.08

        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL),
            parameters
        )

        self.position: Vec3 = Vec3(x=0, y=0, z=0)
        self.angles: Vec3 = Vec3(x=0, y=0, z=0)

    def __del__(self):
        if self.capture and self.capture.isOpened():
            cv2.destroyAllWindows()
            self.capture.release()

    def get_frame_markers(self) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        if self.capture.isOpened():
            if not self.capture.grab():
                return None

            ret, frame = self.capture.retrieve()
            if not ret:
                return None
            if time.time() - self.last_frame < DETECTION_RATE_S:
                return None

            undistorted_frame = cv2.undistort(
                frame, self.matrix, self.distortion, None, self.optimal_camera_matrix
            )

            gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                return frame, corners, ids
            self.last_frame = time.time()

        return None
