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
    calibration_matrix: Optional[List[List[float]]] = np.array([[600, 0.0, 400], [0.0, 600, 260], [0.0, 0.0, 1.0]])
    distortion_coefficients: Optional[List[List[float]]] = np.array([[0.0],[0.0],[0.0],[0.0], [0.0]])
    rotation_vectors: Optional[List[List[List[float]]]] = None
    translation_vectors: Optional[List[List[List[float]]]] = None


class Camera:
    @property
    def matrix(self) -> np.ndarray:
        return np.array(self.configuration.calibration_matrix)

    @property
    def resolution(self) -> CameraResolution:
        if self.__class__.__name__ == "ImageCamera":
            return CameraResolution(
                width = self.frames[0].shape[1],
                height = self.frames[0].shape[0],
            )
        return CameraResolution(
            width = self.capture.get(cv2.CAP_PROP_FRAME_WIDTH),
            height = self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT),
        )

    @property
    def distortion(self) -> np.ndarray:
        return np.array(self.configuration.distortion_coefficients)

    def __init__(self, configuration_path: str) -> None:
        self.optimal_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            self.matrix,
            self.distortion,
            (self.resolution.width, self.resolution.height),
            1,
            (self.resolution.width, self.resolution.height)
        )
        self.configuration_path = configuration_path

        if os.environ.get("RERUN_DISABLE_UI") != "1":
            rr.init("Camera Position", spawn=True)
            rr.log(
                "world/cam/image",
                rr.Pinhole(
                    focal_length=350,
                    width=self.resolution.width,
                    height=self.resolution.height,
                ),
            )

        parameters = cv2.aruco.DetectorParameters()
        parameters.minMarkerPerimeterRate = 0.08
        parameters.minDistanceToBorder = 0
        parameters.minGroupDistance = 0
        parameters.cornerRefinementMinAccuracy = 0.5
        parameters.errorCorrectionRate = 0.9

        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL),
            parameters
        )

        self.position: Vec3 = Vec3(x=0, y=0, z=0)
        self.rotation: Vec3 = Vec3(x=0, y=0, z=0)

    @abc.abstractmethod
    def get_frame(self) -> Optional[np.ndarray]:
        raise NotImplementedError("Method should be implemented in subclass")

    def get_frame_markers(self) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        frame = self.get_frame()

        if frame is None:
            return None, None, None

        # For normal cameras
        #undistorted_frame = cv2.undistort(
        #    frame, self.matrix, self.distortion, None, self.optimal_camera_matrix
        #)

        # For fisheye cameras
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            self.matrix,
            self.distortion,
            np.eye(3),
            self.matrix,
            (self.resolution.width, self.resolution.height),
            cv2.CV_16SC2
        )
        undistorted_frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        # Convert to hsv and only use hue
        undistorted_frame = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2HSV)

        # Only maintains hue
        undistorted_frame[:,:,1] = 0

        undistorted_frame = cv2.cvtColor(undistorted_frame, cv2.COLOR_HSV2BGR)

        gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            return undistorted_frame, None, None

        cv2.aruco.drawDetectedMarkers(undistorted_frame, corners, ids)
        return undistorted_frame, corners, ids


class VideoCamera(Camera):
    def __init__(self, configuration_file: str) -> None:
        self.configuration_path = configuration_file
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

        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'H264'))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.configuration.resolution.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.configuration.resolution.height)
        super().__init__(configuration_file)

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
    def __init__(
        self,
        configuration_file: str,
        frames: List[np.ndarray]
    ) -> None:
        try:
            with open(configuration_file, "r") as file:
                self.configuration = CameraConfiguration(**json.load(file))
        except (FileNotFoundError, json.JSONDecodeError) as e:
            raise ValueError(f"Error reading configuration file: {e}")

        self.frames: np.ndarray = frames
        self.last_frame = 0

        super().__init__(configuration_file)

    def get_frame(self) -> Optional[np.ndarray]:
        frame = self.frames[self.last_frame]
        self.last_frame = (self.last_frame + 1) % len(self.frames)
        return frame


class GstreamerCamera(Camera):
    def __init__(self, configuration_file: str) -> None:
        from capture import Video

        try:
            with open(configuration_file, "r") as file:
                self.configuration = CameraConfiguration(**json.load(file))
        except (FileNotFoundError, json.JSONDecodeError) as e:
            raise ValueError(f"Error reading configuration file: {e}")

        self.video = Video(self.configuration.id, configuration_file)

        super().__init__(self.configuration)

    def get_frame(self) -> Optional[np.ndarray]:
        if not self.video.frame_available():
            return None

        return self.video.frame()
