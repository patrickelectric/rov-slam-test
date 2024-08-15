import cv2
from args import CommandLineArgs
from camera import VideoCamera
from world import World
from slam import SLAM
from vehicle import Vehicle
from test_set import run_test_set
from calibration import CameraCalibrator
from mission import MissionNavigator


def main() -> None:
    args = CommandLineArgs.from_args()

    if args.test_set:
        run_test_set(args.test_set)
        return

    camera = VideoCamera(args.camera)

    if args.calibrate_camera:
        calibrator = CameraCalibrator(camera, args.ideal_calibration)
        calibrator.calibrate()
        return

    world = World(args.world)
    detector = SLAM(world, camera)

    if args.vehicle:
        vehicle = Vehicle(args.vehicle)

    if args.mission:
        mission = MissionNavigator(args.mission, vehicle)

    while True:
        detector.detect()

        if args.mission:
            mission.update(camera.position, camera.rotation)

        if args.vehicle:
            vehicle.actuate(camera.position, camera.rotation.yaw)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

if __name__ == "__main__":
    main()
