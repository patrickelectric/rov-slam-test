import argparse
from dataclasses import dataclass


@dataclass
class CommandLineArgs:
    camera: str
    world: str
    vehicle: str

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
            "--vehicle", type=str, help="Vehicle connection string"
        )

        args = parser.parse_args()
        navigation_args = CommandLineArgs(camera=args.camera, world=args.world, vehicle=args.vehicle)

        return navigation_args
