import argparse
from dataclasses import dataclass
from typing import Optional


@dataclass
class CommandLineArgs:
    camera: str
    world: str
    vehicle: Optional[str] = None
    test_set: Optional[str] = None

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
            "--vehicle", type=str, required=False, default= None, help="Vehicle connection string"
        )
        parser.add_argument(
            "--test-set", type=str, required=False, default= None, help="Directory containing test set images and jsons"
        )

        args = parser.parse_args()
        navigation_args = CommandLineArgs(
            camera=args.camera, world=args.world, vehicle=args.vehicle, test_set=args.test_set
        )

        return navigation_args
