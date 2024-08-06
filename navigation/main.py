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
    vehicle = Vehicle(args.vehicle)

    while True:
        if not vehicle.is_vehicle_armed():
            vehicle.force_arm()

        detector.detect()
        vehicle.actuate(camera.position, camera.angles.yaw)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

if __name__ == "__main__":
    main()
