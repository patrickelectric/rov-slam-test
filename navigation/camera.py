import time
import abc
import json
import os
from typing import List, Tuple, Optional
import rerun as rr
import cv2
import numpy as np
from pydantic import BaseModel
from utils.vector import Vec3


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

    def __init__(self, config: CameraConfiguration) -> None:
        self.optimal_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            self.matrix,
            self.distortion,
            (config.resolution.width, config.resolution.height),
            1,
            (config.resolution.width, config.resolution.height)
        )

        parameters = cv2.aruco.DetectorParameters()
        parameters.minMarkerPerimeterRate = 0.08

        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL),
            parameters
        )

        self.position: Vec3 = Vec3(x=0, y=0, z=0)
        self.angles: Vec3 = Vec3(x=0, y=0, z=0)

    @abc.abstractmethod
    def get_frame(self) -> Optional[np.ndarray]:
        raise NotImplementedError("Method should be implemented in subclass")

    def get_frame_markers(self) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        frame = self.get_frame()

        if frame is None:
            return None

        undistorted_frame = cv2.undistort(
            frame, self.matrix, self.distortion, None, self.optimal_camera_matrix
        )

        gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            return frame, corners, ids


class VideoCamera(Camera):
    def __init__(self, configuration_file: str) -> None:
        try:
            with open(configuration_file, "r") as file:
                self.configuration = CameraConfiguration(**json.load(file))
        except (FileNotFoundError, json.JSONDecodeError) as e:
            raise ValueError(f"Error reading configuration file: {e}")

        self.capture: Optional[cv2.VideoCapture] = None
        print(f"Camera Source: {self.configuration.id}")

        if isinstance(self.configuration.id, int):
            self.capture = cv2.VideoCapture(int(self.configuration.id))
        else:
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"

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
        super().__init__(self.configuration)

    def __del__(self):
        if self.capture and self.capture.isOpened():
            cv2.destroyAllWindows()
            self.capture.release()

    def get_frame(self) -> Optional[np.ndarray]:
        if self.capture.isOpened():
            if not self.capture.grab():
                return None

            ret, frame = self.capture.retrieve()
            if not ret:
                return None

            return frame
        return None


class ImageCamera(Camera):
    def __init__(self, configuration_file: str) -> None:
        try:
            with open(configuration_file, "r") as file:
                self.configuration = CameraConfiguration(**json.load(file))
        except (FileNotFoundError, json.JSONDecodeError) as e:
            raise ValueError(f"Error reading configuration file: {e}")

        self.frame: np.ndarray = cv2.imaread(self.configuration.id)

        rr.init("Camera Position", spawn=True)
        rr.log(
            "world/cam/image",
            rr.Pinhole(
                focal_length=300,
                width=self.configuration.resolution.width,
                height=self.configuration.resolution.height
            ),
        )

        super().__init__(self.configuration)

    def get_frame(self) -> Optional[np.ndarray]:
        return self.frame


class GstreamerCamera(Camera):
    def __init__(self, configuration_file: str) -> None:
        from capture import Video

        try:
            with open(configuration_file, "r") as file:
                self.configuration = CameraConfiguration(**json.load(file))
        except (FileNotFoundError, json.JSONDecodeError) as e:
            raise ValueError(f"Error reading configuration file: {e}")

        self.video = Video(self.configuration.id)

        rr.init("Camera Position", spawn=True)
        rr.log(
            "world/cam/image",
            rr.Pinhole(
                focal_length=300,
                width=self.configuration.resolution.width,
                height=self.configuration.resolution.height
            ),
        )

        super().__init__(self.configuration)

    def get_frame(self) -> Optional[np.ndarray]:
        if not self.video.frame_available():
            return None

        return self.video.frame()
