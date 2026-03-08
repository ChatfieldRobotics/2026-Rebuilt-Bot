from math import cos, pi, sin
from typing import List
from numpy import atan2
import wpilib

from constants import (
    CANConstants,
    DriveConstants,
    FieldConstants,
    OIConstants,
    AutoConstants,
)
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.util import DriveFeedforwards
from phoenix6.hardware import Pigeon2
import robot
from simple_state_system import *
from subsystems.swerve_module_subsystem import SwerveModuleSubsystem
from subsystems.vision_subsystem import VisionSubsystem
from wpilib import DriverStation, SmartDashboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModuleState
from commands2.button import CommandXboxController
from commands2.subsystem import Subsystem
from wpimath.controller import ProfiledPIDControllerRadians


class SwerveDriveSubsystem(Subsystem):
    """Subsystem for controlling a swerve drive. Contains four swerve modules, a gyro, and odometry. Also contains functions for driving the robot and setting the module states."""

    # Define the four swerve modules. The IDs and angular offsets are defined in constants.py
    front_left = SwerveModuleSubsystem(
        DriveConstants.front_left_driving_id,
        DriveConstants.front_left_turning_id,
        DriveConstants.front_left_angular_offset,
    )
    front_right = SwerveModuleSubsystem(
        DriveConstants.front_right_driving_id,
        DriveConstants.front_right_turning_id,
        DriveConstants.front_right_angular_offset,
    )
    back_left = SwerveModuleSubsystem(
        DriveConstants.back_left_driving_id,
        DriveConstants.back_left_turning_id,
        DriveConstants.back_left_angular_offset,
    )
    back_right = SwerveModuleSubsystem(
        DriveConstants.back_right_driving_id,
        DriveConstants.back_right_turning_id,
        DriveConstants.back_right_angular_offset,
    )

    # Define the gyro. The ID is defined in constants.py
    gyro = Pigeon2(CANConstants.pigeon_id)

    # Define the odometry. The kinematics and initial pose are defined in constants.py
    odometry = SwerveDrive4PoseEstimator(
        DriveConstants.drive_kinematics,
        gyro.getRotation2d(),
        [
            front_left.getPosition(),
            front_right.getPosition(),
            back_left.getPosition(),
            back_right.getPosition(),
        ],
        Pose2d(),
    )

    def __init__(self, vision_subsystem: VisionSubsystem):
        # Initialize the state machine
        super().__init__()

        # Store the vision subsystem and initialize the x timer to None
        self.vision_subsystem = vision_subsystem
        self.x_timer = None

        # Create a PID controller for rotating towards the hub in the drive_hub_relative function. The constraints are defined in constants.py and the proportional gain was determined through testing to provide good responsiveness without excessive overshoot.
        self.theta_pid_controller = ProfiledPIDControllerRadians(
            0.7, 0.1, 0.0, AutoConstants.theta_pid_controller.getConstraints()
        )
        self.theta_pid_controller.enableContinuousInput(-pi, pi)

        # Try to create the robot config from the GUI settings and configure the auto builder with the appropriate functions and constants. If there is an error, report it to the driver station.
        try:
            robot_config = RobotConfig.fromGUISettings()

            AutoBuilder.configure(
                self.get_pose,
                self.reset_pose,
                self.get_robot_relative_speeds,
                lambda speeds, feedforwards: self.drive_robot_relative(
                    speeds, feedforwards
                ),
                PPHolonomicDriveController(
                    PIDConstants(13.50, 5.6, 1.9), PIDConstants(9.75, 1.6, 0.6)
                ),
                robot_config,
                lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
                self,
            )
        except Exception as e:
            wpilib.reportError("Error creating configs!")

    def periodic(self):
        # Run internal periodic functions
        super().periodic()

        # Update odometry with the latest gyro and module positions.
        self.odometry.update(
            self.gyro.getRotation2d(),
            [
                self.front_left.getPosition(),
                self.front_right.getPosition(),
                self.back_left.getPosition(),
                self.back_right.getPosition(),
            ],
        )

        # If the vision subsystem has a valid pose estimate, update the odometry with that as well
        if self.vision_subsystem.robot_pose != None:
            robot_pose = self.vision_subsystem.robot_pose
            self.odometry.addVisionMeasurement(
                robot_pose.estimatedPose.toPose2d(), robot_pose.timestampSeconds
            )

    def get_hub_dist(self):
        """Returns the distance from the robot to the hub in meters. The hub position is defined based on the alliance color in the field coordinate system."""
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            hub_x = FieldConstants.red_hub_pose.X()
            hub_y = FieldConstants.red_hub_pose.Y()
        else:
            hub_x = FieldConstants.blue_hub_pose.X()
            hub_y = FieldConstants.blue_hub_pose.Y()

        return (
            (self.get_pose().X() - hub_x) ** 2 + (self.get_pose().Y() - hub_y) ** 2
        ) ** 0.5

    def get_pose(self) -> Pose2d:
        """Returns the current estimated pose of the robot."""
        return self.odometry.getEstimatedPosition()

    def reset_pose(self, new_pose: Pose2d):
        """Resets the robot's pose to the given pose."""
        self.odometry.resetPose(new_pose)

    def reset_odometry(self, pose: Pose2d):
        """Resets the robot's odometry to the given pose. This should be used when the robot's position on the field is known with certainty, such as at the start of a match or after being picked up by the field staff."""
        self.odometry.resetPosition(
            self.gyro.getRotation2d(),
            [
                self.front_left.getPosition(),
                self.front_right.getPosition(),
                self.back_left.getPosition(),
                self.back_right.getPosition(),
            ],
            pose,
        )

    def set_module_states(self, desired_states: List[SwerveModuleState]):
        """Sets the desired states for all four swerve modules. The states are desaturated to ensure that the wheel speeds do not exceed the maximum speed."""
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desired_states, DriveConstants.max_speed_meters_per_second
        )
        self.front_left.setDesiredState(desired_states[0])
        self.front_right.setDesiredState(desired_states[1])
        self.back_left.setDesiredState(desired_states[2])
        self.back_right.setDesiredState(desired_states[3])

    def drive(
        self,
        x_speed: float,
        y_speed: float,
        rot: float,
        field_relative: bool,
        slow_mode: bool = False,
    ):
        # Calculate the speeds to deliver to the modules based on the input speeds, the maximum speed defined in constants.py, whether slow mode is active, and whether the gyro is reversed. If field relative is true, the speeds are converted from field relative to robot relative using the current gyro angle.
        x_speed_delivered = (
            x_speed
            * DriveConstants.max_speed_meters_per_second
            * (DriveConstants.slow_mode_speed_percentage if slow_mode else 1.0)
            * (-1.0 if DriveConstants.gyro_reversed else 1.0)
        )
        y_speed_delivered = (
            y_speed
            * DriveConstants.max_speed_meters_per_second
            * (DriveConstants.slow_mode_speed_percentage if slow_mode else 1.0)
            * (-1.0 if DriveConstants.gyro_reversed else 1.0)
        )
        rot_delivered = (
            rot
            * DriveConstants.max_angular_speed
            * (DriveConstants.slow_mode_speed_percentage if slow_mode else 1.0)
            * (-1.0 if DriveConstants.gyro_reversed else 1.0)
        )

        # If all speeds are zero, start the x timer if it hasn't been started already. If the x timer has been running for longer than the duration defined in constants.py, set the modules to an X formation to prevent movement. If any speed is not zero, reset the x timer.
        if x_speed_delivered == 0 and y_speed_delivered == 0 and rot_delivered == 0:
            if self.x_timer is None:
                self.x_timer = wpilib.Timer.getFPGATimestamp()
                self.set_module_states(
                    DriveConstants.drive_kinematics.toSwerveModuleStates(
                        ChassisSpeeds(0, 0, 0)
                    )
                )
            if (
                wpilib.Timer.getFPGATimestamp() - self.x_timer
                > DriveConstants.x_duration
            ):
                self.set_x()
            return

        self.x_timer = None

        # Convert the desired chassis speeds to individual module states and set the modules to those states.
        swerve_module_states = DriveConstants.drive_kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed_delivered,
                y_speed_delivered,
                rot_delivered,
                self.gyro.getRotation2d(),
            )
            if field_relative
            else ChassisSpeeds(x_speed_delivered, y_speed_delivered, rot_delivered)
        )
        self.set_module_states(swerve_module_states)

    def drive_hub_relative(
        self,
        x_speed: float,
        y_speed: float,
        slow_mode: bool = False,
    ):

        # Calculate the speeds to deliver to the modules based on the input speeds, the maximum speed defined in constants.py, whether slow mode is active, and whether the gyro is reversed. The speeds are converted from field relative to robot relative using the current gyro angle and the angle to the hub.
        x_speed_delivered = (
            x_speed
            * DriveConstants.max_speed_meters_per_second
            * (DriveConstants.slow_mode_speed_percentage if slow_mode else 1.0)
            * (-1.0 if DriveConstants.gyro_reversed else 1.0)
        )
        y_speed_delivered = (
            y_speed
            * DriveConstants.max_speed_meters_per_second
            * (DriveConstants.slow_mode_speed_percentage if slow_mode else 1.0)
            * (-1.0 if DriveConstants.gyro_reversed else 1.0)
        )

        # Define the hub position based on the alliance color in the field coordinate system.
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            hub_x = FieldConstants.red_hub_pose.X()
            hub_y = FieldConstants.red_hub_pose.Y()
        else:
            hub_x = FieldConstants.blue_hub_pose.X()
            hub_y = FieldConstants.blue_hub_pose.Y()

        # Calculate the angle to the hub from the robot's current position and create a PID controller to rotate towards that angle. The PID controller is configured with the constraints defined in constants.py and a proportional gain of 3.0, which was determined through testing to provide good responsiveness without excessive overshoot.
        robot_pose = self.get_pose()
        angle_to_hub = -atan2(hub_y - robot_pose.Y(), hub_x - robot_pose.X())

        theta_pid_output = self.theta_pid_controller.calculate(
            robot_pose.rotation().radians(), pi - angle_to_hub
        )

        rot_delivered = (
            theta_pid_output
            * DriveConstants.max_angular_speed
            * (DriveConstants.slow_mode_speed_percentage if slow_mode else 1.0)
        )

        # Convert the desired chassis speeds to individual module states and set the modules to those states.
        swerve_module_states = DriveConstants.drive_kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed_delivered,
                y_speed_delivered,
                rot_delivered,
                self.gyro.getRotation2d(),
            )
        )
        self.set_module_states(swerve_module_states)

    def drive_robot_relative(
        self, speeds: ChassisSpeeds, feedforwards: DriveFeedforwards
    ):
        # Convert the desired chassis speeds to individual module states and set the modules to those states. The feedforwards are currently not used, but could be implemented in the future for more accurate control.
        swerve_module_states = DriveConstants.drive_kinematics.toSwerveModuleStates(
            speeds
        )
        self.set_module_states(swerve_module_states)

    def get_robot_relative_speeds(self) -> ChassisSpeeds:
        # Get the current module states and convert them to robot relative chassis speeds using the kinematics.
        module_states = self.get_module_states()
        robot_relative_speeds = DriveConstants.drive_kinematics.toChassisSpeeds(
            module_states
        )
        return robot_relative_speeds

    def set_x(self):
        # Set the modules to an X formation to prevent movement when the robot is disabled or when the input speeds are zero for an extended period of time.
        self.front_left.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(45))
        )
        self.front_right.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        )
        self.back_left.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        )
        self.back_right.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(45))
        )

    def get_module_states(self) -> List[SwerveModuleState]:
        # Get the current states of all four swerve modules and return them as a list.
        return [
            self.front_left.getState(),
            self.front_right.getState(),
            self.back_left.getState(),
            self.back_right.getState(),
        ]

    def reset_encoders(self):
        # Reset the encoders on all four swerve modules. This should be used when the robot's position on the field is known with certainty, such as at the start of a match or after being picked up by the field staff.
        self.front_left.reset_encoders()
        self.back_left.reset_encoders()
        self.front_right.reset_encoders()
        self.back_right.reset_encoders()

    def apply_deadband(self, value: float, deadband: float) -> float:
        """Applies a deadband to the given value. If the absolute value of the input is less than the deadband, returns 0. Otherwise, scales the input so that it starts from 0 at the edge of the deadband and reaches 1 at the maximum input."""
        if abs(value) < deadband:
            return 0.0
        else:
            return (value - deadband * (1 if value > 0 else -1)) / (1 - deadband)

    def default_drive(
        self, driver_controller: CommandXboxController, field_relative: bool
    ):
        """Default drive function that can be used as the default command for the swerve drive subsystem. Takes input from the given Xbox controller and drives the robot accordingly. The left stick controls translation, while the right stick controls rotation. If the right bumper is held, the robot will drive hub relative instead of field relative. If the left trigger is held beyond 0.2, slow mode will be activated, which reduces the maximum speed for more precise control."""
        if DriverStation.isDisabled():
            return False

        if driver_controller.leftTrigger().getAsBoolean():
            self.drive_hub_relative(
                x_speed=-self.apply_deadband(
                    driver_controller.getLeftY(), OIConstants.drive_deadband
                ),
                y_speed=-self.apply_deadband(
                    driver_controller.getLeftX(), OIConstants.drive_deadband
                ),
                slow_mode=driver_controller.rightBumper().getAsBoolean(),
            )
        else:
            self.drive(
                x_speed=-self.apply_deadband(
                    driver_controller.getLeftY(), OIConstants.drive_deadband
                ),
                y_speed=-self.apply_deadband(
                    driver_controller.getLeftX(), OIConstants.drive_deadband
                ),
                rot=-self.apply_deadband(
                    driver_controller.getRightX(), OIConstants.drive_deadband
                ),
                field_relative=field_relative,
                slow_mode=driver_controller.rightBumper().getAsBoolean(),
            )

    def zero_heading(self):
        """Resets the gyro heading to zero. This should be used when the robot's orientation is known with certainty, such as at the start of a match or after being picked up by the field staff."""
        self.gyro.reset()

    def smart_zero_heading(self):
        """Resets the gyro heading to zero based on the alliance color and the current estimated pose."""
        alliance = DriverStation.getAlliance()

        if alliance != None:
            self.gyro.set_yaw(
                self.get_pose().rotation().degrees()
                + 180.0 * (alliance == DriverStation.Alliance.kRed)
            )
        else:
            wpilib.reportError("Couldn't get alliance!")
