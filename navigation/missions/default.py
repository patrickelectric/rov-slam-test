from utils.vector import Vec3
from vehicle import Vehicle

dock_station = Vec3(x=-0.03, y=-0.193, z=0.655)

#rotation Quaternion([-0.9967669, -0.0069931247, -0.0069350293, 0.051607076]

def end(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # End Mission
    vehicle.deactivate()
    print("Mission Ended")

    # Close Gripper
    next(None)

def move_to_dock(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Move to dock station
    if first_execution:
        print("Moving to dock station")
        vehicle.go_home()

    print("Checking if target is stable")

    if vehicle.is_target_stable(0.03):
        print("Target stable")
        next(end)

def move_to_pre_dock(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Move to pre dock station
    if first_execution:
        print("Moving to pre dock station")
        vehicle.move_abs(Vec3(x=vehicle.home_position.x, y=vehicle.home_position.y, z=vehicle.home_position.z + 0.3))

    print("Checking if target is stable")

    if vehicle.is_target_stable(0.05):
        print("Target stable")
        next(move_to_dock)

def move_back_1(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Navigate 50 cm back
    if first_execution:
        print("Moving back, relative z -2")
        vehicle.move_rel(Vec3(x=0.3, y=0.0, z=0.0))
        print(f"Target position: {vehicle.target_position}")
        print(f"Homing to {vehicle.home_position}")
        print(f"Current position: {position}")

    print("Checking if target is stable")

    if vehicle.is_target_stable(0.05):
        print("Target stable")
        next(move_to_pre_dock)

def move_back(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Navigate 50 cm back
    if first_execution:
        print("Moving back, relative z -2")
        vehicle.move_rel(Vec3(x=0.0, y=0.01, z=0.3))
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
    next(move_back)

def start(vehicle: Vehicle, position: Vec3, rotation: Vec3, first_execution: bool, next) -> None:
    # Open Gripper
    print("Starting Mission, opening gripper")

    next(set_origin, 7)
