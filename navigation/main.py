import cv2
from args import CommandLineArgs
from camera import VideoCamera
from world import World
from slam import SLAM
from vehicle import Vehicle


def main() -> None:
    args = CommandLineArgs.from_args()

    camera = VideoCamera(args.camera)
    world = World(args.world)
    detector = SLAM(world, camera)

    if args.vehicle:
        vehicle = Vehicle(args.vehicle)

    while True:
        detector.detect()

        if args.vehicle:
            vehicle.actuate(camera.position, camera.angles.yaw)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

if __name__ == "__main__":
    main()
