import json
import cv2
import time
import numpy as np
from camera import Camera


class CameraCalibrator:
    """
    This class is used to calibrate a camera using a set of images of a checkerboard pattern.
    """

    def __init__(
        self, camera: Camera, samples_needed: int = 200
    ) -> None:
        """
        Args:
            references_dir (str): The directory containing the images of the checkerboard pattern.
            chessboard_size (Vec2): The size of the chessboard pattern in the format (rows, cols).
            show_imgs (bool): Whether to display the images of the checkerboard pattern while calibrating
        """

        self.camera = camera
        self.chessboard_size = (6, 9)
        self.samples_needed = samples_needed

        # stop the iteration when specified
        # accuracy, epsilon, is reached or
        # specified number of iterations are completed.
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.0005)

        # Vector for 3D points
        self.points_3d = []
        # Vector for 2D points
        self.points_2d = []

        self.object_points = np.zeros((1, self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        self.object_points[0, :, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)

    def save_calibration(self, matrix: np.ndarray, distortion: np.ndarray) -> None:
        # Convert numpy arrays to lists for JSON serialization
        calibration_data = {
            "id": self.camera.configuration.id,
            "resolution": {
                "width": self.camera.configuration.resolution.width,
                "height": self.camera.configuration.resolution.height
            },
            "calibration_matrix": matrix.tolist(),
            "distortion_coefficients": [[dist] for dist in distortion.tolist()[0]]
        }

        # Save calibration data to a JSON file
        with open(f"./{self.camera.configuration_path}", "w") as file:
            json.dump(calibration_data, file, indent=4)

        print("Calibration completed successfully")

    def calibrate(self) -> None:
        counter = 0
        while counter < self.samples_needed:
            # Capture frame-by-frame
            frame = self.camera.get_frame()

            if frame is not None:
                gray_color = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # Find the chess board corners
                ret, corners = cv2.findChessboardCorners(
                    gray_color, self.chessboard_size,
                    cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
                )

                if ret:
                    self.points_3d.append(self.object_points)

                    # Refining pixel coordinates for given 2d points.
                    corners2 = cv2.cornerSubPix(gray_color, corners, (11, 11), (-1, -1), self.criteria)
                    self.points_2d.append(corners2)

                    # Draw and display the corners
                    frame = cv2.drawChessboardCorners(frame, self.chessboard_size, corners2, ret)
                    counter += 1
                    print(f"Sample {counter} of {self.samples_needed} collected")
                    time.sleep(0.15)

            cv2.imshow('Calibration Table', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

        # Perform camera calibration by
        # passing the value of above found out 3D points (threedpoints)
        # and its corresponding pixel coordinates of the
        # detected corners (twodpoints)
        ret, matrix, distortion, _, _ = cv2.calibrateCamera(
            self.points_3d, self.points_2d, gray_color.shape[::-1], None, None
        )

        self.save_calibration(matrix, distortion)
