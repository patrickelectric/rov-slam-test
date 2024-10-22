import os
import argparse
from dataclasses import dataclass
from typing import Optional


@dataclass
class CommandLineArgs:
    camera: str
    world: str
    vehicle: Optional[str] = None
    mission: Optional[str] = None
    test_set: Optional[str] = None
    calibrate_camera: Optional[bool] = None
    ideal_calibration: Optional[bool] = None

    @staticmethod
    def from_args() -> "CommandLineArgs":
        parser = argparse.ArgumentParser(description="Slam Navigation System")

        parser.add_argument(
            "--camera", type=str, default="./navigation/data/cameras/default.json", help="Camera configuration file to use"
        )
        parser.add_argument(
            "--world", type=str, default="./navigation/data/world.json", help="World configuration file to use"
        )
        parser.add_argument(
            "--vehicle", type=str, required=False, default=None, help="Vehicle connection string"
        )
        parser.add_argument(
            "--test-set", type=str, required=False, default=None, help="Directory containing test set images and jsons"
        )
        parser.add_argument(
            "--calibrate-camera", default=None, action="store_true", help="Calibrate the camera"
        )
        parser.add_argument(
            "--ideal-calibration", type=bool, default=False, help="Create an ideal calibration matrix",
            action=argparse.BooleanOptionalAction
        )
        parser.add_argument(
            "--mission", type=str, default=None, help="Mission file to run"
        )

        args = parser.parse_args()
        navigation_args = CommandLineArgs(
            camera=args.camera,
            world=args.world,
            vehicle=args.vehicle,
            mission=args.mission,
            test_set=args.test_set,
            calibrate_camera=args.calibrate_camera,
            ideal_calibration=args.ideal_calibration
        )

        if navigation_args.mission and not navigation_args.vehicle:
            raise ValueError("Mission file requires a vehicle connection string")

        if navigation_args.test_set or navigation_args.calibrate_camera:
            os.environ["RERUN_DISABLE_UI"] = "1"

        return navigation_args
