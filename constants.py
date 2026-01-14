from wpimath.geometry import Transform3d, Translation2d, Translation3d, Rotation3d
from wpimath.units import inchesToMeters
from wpimath.kinematics import SwerveDrive4Kinematics
from pathplannerlib.path import PathConstraints

from math import pi

class NeoMotorConstants:
    free_speed_rpm = 5676

class DriveConstants:
    max_speed_meters_per_second = 4.8
    max_angular_speed = 2 * pi
    slow_mode_speed_percentage = 0.3

    track_width = 0.622
    wheel_base = 0.622
    wheel_translations = [
        Translation2d(wheel_base / 2, track_width / 2),
        Translation2d(wheel_base / 2, -track_width / 2),
        Translation2d(-wheel_base / 2, track_width / 2),
        Translation2d(-wheel_base / 2, -track_width / 2),
    ]

    drive_kinematics = SwerveDrive4Kinematics(*wheel_translations)

    front_left_angular_offset = -pi / 2
    front_right_angular_offset = 0
    back_left_angular_offset = pi
    back_right_angular_offset = pi / 2

    front_left_driving_id = 2
    front_right_driving_id = 4
    back_left_driving_id = 6
    back_right_driving_id = 8

    front_left_turning_id = 1
    front_right_turning_id = 3
    back_left_turning_id = 5
    back_right_turning_id = 7

    gyro_reversed = False

class ModuleConstants:
    driving_motor_pinion_teeth = 13

    driving_motor_free_speed_rps = NeoMotorConstants.free_speed_rpm / 60.0
    wheel_diameter_meters = 0.0753
    wheel_circumference_meters = wheel_diameter_meters * pi

    driving_motor_reduction = (45.0 * 22.0) / (driving_motor_pinion_teeth * 15.0)
    drive_wheel_free_speed_rps = (
        driving_motor_free_speed_rps
        * wheel_circumference_meters
        / driving_motor_reduction
    )

class OIConstants:
    driver_controller_port = 0
    copilot_controller_port = 1

    drive_deadband = 0.05