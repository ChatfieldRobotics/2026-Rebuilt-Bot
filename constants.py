from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry import Pose2d, Transform3d, Translation2d, Translation3d, Rotation3d
from wpimath.units import inchesToMeters
from wpimath.kinematics import SwerveDrive4Kinematics

from math import pi


class CANConstants:
    pigeon_id: int = 0
    right_intake_motor: int = 1
    left_intake_motor: int = 2
    upper_roller_motor: int = 3
    lower_roller_motor: int = 4
    conveyor_motor: int = 5
    trigger_motor: int = 6
    intake_motor: int = 7


class ShooterConstants:
    advancement_motor_rps: float = -90
    conveyor_motor_rps: float = -90

    minimum_acceptable_closed_loop_error: float = 1


class NeoMotorConstants:
    free_speed_rpm = 5676


class DriveConstants:
    max_speed_meters_per_second = 15
    max_angular_speed = 5 * pi
    slow_mode_speed_percentage = 0.3

    track_width = 0.5969
    wheel_base = 0.5969
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

    x_duration = 0.25


class ModuleConstants:
    driving_motor_pinion_teeth = 12

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

    drive_deadband = 0.05


class VisionConstants:
    robot_to_camera = Transform3d(
        Translation3d(inchesToMeters(-12.5), 0, inchesToMeters(15.0)),
        Rotation3d(pi / 6, pi, 0),
    )


class HopperConstants:
    extended_position: float = 15.5
    retracted_position: float = 0.0

    intake_speed: float = -0.9


class FieldConstants:
    red_hub_pose = Pose2d(11.84, 4.035, Rotation3d(0))
    blue_hub_pose = Pose2d(4.606, 4.035, Rotation3d(pi))