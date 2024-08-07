import argparse
import json
import cv2
import glob
import numpy as np
from typing import Tuple


class CameraCalibrator:
    """
    This class is used to calibrate a camera using a set of images of a checkerboard pattern.
    """

    def __init__(
        self,
        references_dir: str,
        chessboard_size: Tuple[int, int],
        show_imgs: bool = False,
        output_file: str = "calibration.json"
    ) -> None:
        """
        Args:
            references_dir (str): The directory containing the images of the checkerboard pattern.
            chessboard_size (Vec2): The size of the chessboard pattern in the format (rows, cols).
            show_imgs (bool): Whether to display the images of the checkerboard pattern while calibrating
        """

        self.references_dir = references_dir
        self.chessboard_size = chessboard_size
        self.show_imgs = show_imgs
        self.output_file = output_file

        with open(f"{self.references_dir}/camera.json") as file:
            self.metadata = json.load(file)

        # stop the iteration when specified
        # accuracy, epsilon, is reached or
        # specified number of iterations are completed.
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Vector for 3D points
        self.points_3d = []
        # Vector for 2D points
        self.points_2d = []

        self.object_points = np.zeros((1, chessboard_size[0] * chessboard_size[1], 3), np.float32)
        self.object_points[0, :, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

    def calibrate(self) -> None:
        images = glob.glob(self.references_dir + "/*.jpeg")

        if not images:
            raise ValueError("No images found in the specified directory")

        for filename in images:
            image = cv2.imread(filename)
            gray_color = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            # If desired number of corners are
            # found in the image then ret = true
            ret, corners = cv2.findChessboardCorners(
                            gray_color, self.chessboard_size,
                            cv2.CALIB_CB_ADAPTIVE_THRESH
                            + cv2.CALIB_CB_FAST_CHECK +
                            cv2.CALIB_CB_NORMALIZE_IMAGE)

            # If desired number of corners can be detected then,
            # refine the pixel coordinates and display
            # them on the images of checker board
            if ret == True:
                self.points_3d.append(self.object_points)

                # Refining pixel coordinates
                # for given 2d points.
                corners2 = cv2.cornerSubPix(
                    gray_color, corners, (11, 11), (-1, -1), self.criteria)

                self.points_2d.append(corners2)

                # Draw and display the corners
                image = cv2.drawChessboardCorners(image, self.chessboard_size, corners2, ret)

            if self.show_imgs:
                cv2.imshow('img', image)
                cv2.waitKey(0)

        cv2.destroyAllWindows()

        # Perform camera calibration by
        # passing the value of above found out 3D points (threedpoints)
        # and its corresponding pixel coordinates of the
        # detected corners (twodpoints)
        ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(
            self.points_3d, self.points_2d, gray_color.shape[::-1], None, None
        )

        # Convert numpy arrays to lists for JSON serialization
        calibration_data = {
            "id": self.metadata["id"],
            "resolution": {
                "width": self.metadata["resolution"]["width"],
                "height": self.metadata["resolution"]["height"]
            },
            "calibration_matrix": matrix.tolist(),
            "distortion_coefficients": [[dist] for dist in distortion.tolist()[0]],
            "rotation_vectors": [rvec.tolist() for rvec in r_vecs],
            "translation_vectors": [tvec.tolist() for tvec in t_vecs]
        }

        # Save calibration data to a JSON file
        with open(f"./{self.output_file}", "w") as file:
            json.dump(calibration_data, file, indent=4)

        print("Calibration completed successfully")


def main() -> None:
    parser = argparse.ArgumentParser(description="Camera calibration tool")

    parser.add_argument(
        "--reference-dir", type=str, default="./data/cameras/default.json", help="Directory containing images of the checkerboard pattern"
    )
    parser.add_argument(
        "--output-file", type=str, default="calibration.json", help="Name of the output file containing the calibration data"
    )
    parser.add_argument(
        "--chessboard-size", type=int, nargs=2, default=[6, 9], help="Size of the chessboard pattern in the format (rows, cols)"
    )
    parser.add_argument(
        "--show-imgs", action="store_true", help="Display images of the checkerboard pattern while calibrating"
    )

    args = parser.parse_args()

    calibrator = CameraCalibrator(
        references_dir=args.reference_dir,
        chessboard_size=(args.chessboard_size[0], args.chessboard_size[1]),
        show_imgs=args.show_imgs,
        output_file=args.output_file
    )
    calibrator.calibrate()

if __name__ == "__main__":
    main()

# Example usage:
# python tools/calibration.py --reference-dir ./calib_images --output-file ./camera_calib.json --chessboard-size 9 6 --show-imgs
