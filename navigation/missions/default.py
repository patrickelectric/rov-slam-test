from utils.vector import Vec3
from vehicle import Vehicle

dock_station = Vec3(x=-0.03, y=-0.193, z=0.655)

#rotation Quaternion([-0.9967669, -0.0069931247, -0.0069350293, 0.051607076]

def end(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # End Mission
    vehicle.deactivate()
    vehicle.look_forward()
    print("Mission Ended")

    # Close Gripper
    next(None)

def close_gripper(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Close Gripper
    print("Closing Gripper")
    vehicle.open_gripper()
    vehicle.close_gripper()

    next(end, 5)

def look_down(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Look Down
    print("Looking Down")
    vehicle.look_down()

    next(close_gripper, 1)

def move_to_dock(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Move to dock station
    if first_execution:
        print("Moving to dock station")
        vehicle.move_abs(Vec3(x=vehicle.home_position.x, y=vehicle.home_position.y + 0.03, z=vehicle.home_position.z + 0.01))

    print("Checking if target is stable")

    if vehicle.is_target_stable(0.05):
        print("Target stable")
        next(look_down)

def move_to_pre_dock_1(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Move to pre dock station
    if first_execution:
        print("Moving to pre dock station")
        vehicle.move_abs(Vec3(x=vehicle.home_position.x, y=vehicle.home_position.y + 0.03, z=vehicle.home_position.z + 0.3))

    print("Checking if target is stable")

    if vehicle.is_target_stable(0.03):
        print("Target stable")
        next(move_to_dock)

def move_to_pre_dock(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Move to pre dock station
    if first_execution:
        print("Moving to pre dock station")
        vehicle.move_abs(Vec3(x=vehicle.home_position.x, y=vehicle.home_position.y + 0.05, z=vehicle.home_position.z + 0.5))

    print("Checking if target is stable")

    if vehicle.is_target_stable(0.05):
        print("Target stable")
        next(move_to_pre_dock_1)

def move_back_2(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Navigate 50 cm back
    if first_execution:
        print("Moving back, relative z -2")
        vehicle.move_rel(Vec3(x=-0.8, y=0.0, z=0.0))
        print(f"Target position: {vehicle.target_position}")
        print(f"Homing to {vehicle.home_position}")
        print(f"Current position: {position}")

    print("Checking if target is stable")

    if vehicle.is_target_stable(0.05):
        print("Target stable")
        next(move_to_pre_dock)

def move_back_1(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Navigate 50 cm back
    if first_execution:
        print("Moving back, relative z -2")
        vehicle.move_rel(Vec3(x=0.4, y=0.0, z=0.0))
        print(f"Target position: {vehicle.target_position}")
        print(f"Homing to {vehicle.home_position}")
        print(f"Current position: {position}")

    print("Checking if target is stable")

    if vehicle.is_target_stable(0.05):
        print("Target stable")
        next(move_back_2)

def move_back(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Navigate 50 cm back
    if first_execution:
        print("Moving back, relative z -2")
        vehicle.move_rel(Vec3(x=0.0, y=0.01, z=0.5))
        print(f"Target position: {vehicle.target_position}")
        print(f"Homing to {vehicle.home_position}")
        print(f"Current position: {position}")

    print("Checking if target is stable")

    if vehicle.is_target_stable(0.05):
        print("Target stable")
        next(move_back_1)

def set_origin(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Set Home
    vehicle.set_home(position, rotation.yaw)
    print(f"Setting Home to {position} and yaw {rotation.yaw}")
    vehicle.go_home()
    print("Going Home")

    # Move away from dock station in 2 seconds
    next(move_back, 1)

def calibrate(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Set Home
    target = Vec3(x=-0.0, y=-0.090, z=1.057)

    if first_execution:
        vehicle.set_home(target, 0.0)
        vehicle.go_home()
        print(f"Setting Home to {position} and yaw {rotation.yaw}")

    if vehicle.is_target_stable(0.001):
        print("Target stable")
        next(None)

def look_forward(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Look forward
    print("Looking Forward")
    vehicle.look_forward()

    next(set_origin, 1)

def release_gripper(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Open Gripper
    print("Releasing Gripper")
    vehicle.close_gripper()
    vehicle.open_gripper()

    next(look_forward, 10)

def start(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Open Gripper
    print("Starting Mission, looking down")

    #vehicle.look_forward()
    vehicle.look_down()

    next(release_gripper, 1)
