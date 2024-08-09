
import glob
import json
import os
import cv2
import re
import time
import numpy as np
from camera import ImageCamera
from world import World
from slam import SLAM
from utils.vector import Vec3
from calibration import CameraCalibrator
from typing import List, Optional

def natural_sort(list: List[str]):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(list, key=alphanum_key)


def run_test_set(test_set_path: str) -> None:
    images = glob.glob(os.path.join(test_set_path, "world_*.png"))

    skipped = []
    calibrated_camera_path: Optional[str] = None
    for image_path in images:
        print("================================")
        print(f"Running test on {image_path}...")
        json_path = image_path.replace(".png", "_source.json")
        camera_json_path = image_path.replace(".png", "_camera.json")
        tags_json_path = image_path.replace(".png", "_tags.json")

        if not os.path.exists(json_path):
            error = f" - Skipping {image_path} because it has no associated JSON file."
            print(error)
            skipped.append(error)
            continue
        if not os.path.exists(camera_json_path):
            error = f" - Skipping {image_path} because it has no associated camera JSON file."
            print(error)
            skipped.append(error)
            continue
        if not os.path.exists(tags_json_path):
            error = f" - Skipping {image_path} because it has no associated tags JSON file."
            print(error)
            skipped.append(error)
            continue

        print(f" - JSON file: {json_path}")
        with open(json_path, "r") as f:
            data = json.load(f)

        if not calibrated_camera_path:
            calibration_frames = []
            for chess_file in natural_sort(glob.glob(os.path.join(test_set_path, "chess_*.png"))):
                calibration_frames.append(cv2.imread(chess_file))
            calibrator = CameraCalibrator(ImageCamera(camera_json_path, calibration_frames), 100)
            calibrator.calibrate()
            calibrated_camera_path = camera_json_path

        frame = cv2.imread(image_path)
        camera = ImageCamera(calibrated_camera_path, [frame])
        world = World(tags_json_path)
        detector = SLAM(world, camera)

        detector.detect()
        cv2.imshow("Frame", frame)
        cv2.waitKey(10)

        time.sleep(2)

        position = Vec3(**data["position"])
        angles = Vec3(**data["rotation"])

        pos_diff = camera.position - position
        angles_diff = camera.rotation - angles
        pos_similarity = camera.position.dot(position)
        angles_similarity = camera.rotation.dot(angles)

        print("Positions:")
        print(f" - Expected: {position}")
        print(f" - Detected: {camera.position}")
        print(f" - Difference: {pos_diff}")
        print(f" - Similarity: {np.round(pos_similarity, 3)}")
        print("Angles:")
        print(f" - Expected: {angles}")
        print(f" - Detected: {camera.rotation}")
        print(f" - Difference: {angles_diff}")
        print(f" - Similarity: {np.round(angles_similarity, 3)}")

        del camera
        del world
        del detector
        cv2.destroyAllWindows()

    if len(skipped) > 0:
        print("================================")
        print("Following errors were encountered:")
        for error in skipped:
            print(error)
        exit(1)

    print("================================")
    print("All tests passed!")
