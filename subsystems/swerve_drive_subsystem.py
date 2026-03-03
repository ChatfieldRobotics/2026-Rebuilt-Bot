from math import cos, pi, sin
from typing import List
from numpy import arctan
import wpilib

from constants import DriveConstants, ShooterConstants, AutoConstants
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.util import DriveFeedforwards
from phoenix6.hardware import Pigeon2
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

    vision_subsystem: VisionSubsystem

    gyro = Pigeon2(0)

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

    slow_mode = False

    x_pid_controller = AutoConstants.x_pid_controller
    y_pid_controller = AutoConstants.y_pid_controller
    theta_pid_controller = AutoConstants.theta_pid_controller

    x_timer = None

    # The parametric value used for orbiting the hub
    t: float = 0

    orbiting = False

    def __init__(self, vision_subsystem: VisionSubsystem):
        # Initialize the state machine
        super().__init__()

        self.vision_subsystem = vision_subsystem

        robot_config: RobotConfig

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
                lambda: DriverStation.getAlliance()
                == DriverStation.Alliance.kRed,
                self,
            )
        except Exception as e:
            wpilib.reportError("Error creating configs!")

    def periodic(self):
        # Run internal periodic functions
        super().periodic()

        # if DriverStation.isEnabled():
        robot_pose = self.get_pose()
        SmartDashboard.putNumber("Red Hub Dist", ((robot_pose.X() - (11.84))**2 + (robot_pose.Y() - 4.035)**2) ** 0.5)
        SmartDashboard.putNumber("Blue Hub Dist", ((robot_pose.X() - (4.606))**2 + (robot_pose.Y() - 4.035)**2) ** 0.5)

        self.odometry.update(
            self.gyro.getRotation2d(),
            [
                self.front_left.getPosition(),
                self.front_right.getPosition(),
                self.back_left.getPosition(),
                self.back_right.getPosition(),
            ],
        )

        if self.vision_subsystem.robot_pose != None:
            robot_pose = self.vision_subsystem.robot_pose
            self.odometry.addVisionMeasurement(
                robot_pose.estimatedPose.toPose2d(), robot_pose.timestampSeconds
            )

    def get_hub_dist(self): 
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            hub_x = 11.84
            hub_y = 4.035
        else: 
            hub_x = 4.606
            hub_y = 4.035
        
        return ((self.get_pose().X() - hub_x)**2 + (self.get_pose().Y() - hub_y)**2) ** 0.5

    def get_pose(self) -> Pose2d:
        return self.odometry.getEstimatedPosition()

    def reset_pose(self, new_pose: Pose2d):
        self.odometry.resetPose(new_pose)

    def reset_odometry(self, pose: Pose2d):
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
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desired_states, DriveConstants.max_speed_meters_per_second
        )
        self.front_left.setDesiredState(desired_states[0])
        self.front_right.setDesiredState(desired_states[1])
        self.back_left.setDesiredState(desired_states[2])
        self.back_right.setDesiredState(desired_states[3])

    def drive(self, x_speed: float, y_speed: float, rot: float, field_relative: bool):
        x_speed_delivered = (
            x_speed
            * DriveConstants.max_speed_meters_per_second
            * (DriveConstants.slow_mode_speed_percentage if self.slow_mode else 1.0)
            * (-1.0 if DriveConstants.gyro_reversed else 1.0)
        )
        y_speed_delivered = (
            y_speed
            * DriveConstants.max_speed_meters_per_second
            * (DriveConstants.slow_mode_speed_percentage if self.slow_mode else 1.0)
            * (-1.0 if DriveConstants.gyro_reversed else 1.0)
        )
        rot_delivered = (
            rot
            * DriveConstants.max_angular_speed
            * (DriveConstants.slow_mode_speed_percentage if self.slow_mode else 1.0)
            * (-1.0 if DriveConstants.gyro_reversed else 1.0)
        )

        if x_speed_delivered == 0 and y_speed_delivered == 0 and rot_delivered == 0:
            if self.x_timer is None:
              self.x_timer = wpilib.Timer.getFPGATimestamp()
            if wpilib.Timer.getFPGATimestamp() - self.x_timer > DriveConstants.x_duration:
                self.set_x()
            return

        self.x_timer = None

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

    def drive_robot_relative(
        self, speeds: ChassisSpeeds, feedforwards: DriveFeedforwards
    ):
        swerve_module_states = DriveConstants.drive_kinematics.toSwerveModuleStates(
            speeds
        )
        self.set_module_states(swerve_module_states)

    def get_robot_relative_speeds(self) -> ChassisSpeeds:
        module_states = self.get_module_states()
        robot_relative_speeds = DriveConstants.drive_kinematics.toChassisSpeeds(
            module_states
        )
        return robot_relative_speeds

    def set_x(self):
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
        return [
            self.front_left.getState(),
            self.front_right.getState(),
            self.back_left.getState(),
            self.back_right.getState(),
        ]

    def reset_encoders(self):
        self.front_left.reset_encoders()
        self.back_left.reset_encoders()
        self.front_right.reset_encoders()
        self.back_right.reset_encoders()

    def get_parametric_position(self) -> List[float]:
        return [
            182.11
            + ShooterConstants.optimal_shooter_distance
            * cos(5 * pi / 6 * self.t + 7 * pi / 12),
            317.69 / 2
            + ShooterConstants.optimal_shooter_distance
            * sin(5 * pi / 6 * self.t + 7 * pi / 12),
            5 * pi / 6 * self.t + 7 * pi / 12,
        ]
    
    def apply_deadband(self, value: float, deadband: float) -> float:
        if abs(value) < deadband:
            return 0.0
        else:
            return (value - deadband * (1 if value > 0 else -1)) / (1 - deadband)

    def default_drive(
        self, driver_controller: CommandXboxController, field_relative: bool
    ):
        if DriverStation.isDisabled():
            return False

        self.drive(
            -self.apply_deadband(driver_controller.getLeftY(), 0.1),
            -self.apply_deadband(driver_controller.getLeftX(), 0.1),
            -self.apply_deadband(driver_controller.getRightX(), 0.1),
            field_relative,
        )
        return False
    
    def point_towards_hub(self, driver_controller: CommandXboxController): 
        x_speed_delivered = (
            -self.apply_deadband(driver_controller.getLeftY(), 0.1)
            * DriveConstants.max_speed_meters_per_second
            * (DriveConstants.slow_mode_speed_percentage if self.slow_mode else 1.0)
            * (-1.0 if DriveConstants.gyro_reversed else 1.0)
        )
        y_speed_delivered = (
            -self.apply_deadband(driver_controller.getLeftX(), 0.1)
            * DriveConstants.max_speed_meters_per_second
            * (DriveConstants.slow_mode_speed_percentage if self.slow_mode else 1.0)
            * (-1.0 if DriveConstants.gyro_reversed else 1.0)
        )

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            hub_x = 11.84
            hub_y = 4.035 
        else:
            hub_x = 4.606
            hub_y = 4.035

        robot_pose = self.get_pose()
        angle_to_hub = arctan(hub_y - robot_pose.Y(), hub_x - robot_pose.X())

        theta_pid_controller = ProfiledPIDControllerRadians(3.0, 0.0, 0.0, AutoConstants.theta_pid_controller.constraints)
        theta_pid_controller.setGoal(angle_to_hub)

        theta_pid_output = theta_pid_controller.calculate(robot_pose.rotation().radians())

        swerve_module_states = DriveConstants.drive_kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed_delivered,
                y_speed_delivered,
                theta_pid_output,
                Rotation2d(angle_to_hub),
            )
        )

        self.set_module_states(swerve_module_states)
        return abs(theta_pid_controller.getPositionError()) < 0.05            

    def pre_orbit(self):
        # NEED TO IMPLEMENT T CALCULATION
        self.orbiting = True
        return True

    def orbit_hub(self, driver_controller: CommandXboxController):
        self.t = min(max(self.t + driver_controller.getLeftX(), 0.0), 1.0)

        alliance = DriverStation.getAlliance()

        if alliance == None:
            return True

        target_position = self.get_parametric_position()

        if alliance == DriverStation.Alliance.kBlue:
            target_position[0] = 651.22 - target_position[0]
            target_position[2] = pi - target_position[2]

        target_position[0] *= 0.0254
        target_position[1] *= 0.0254

        self.x_pid_controller.setGoal(target_position[0])
        self.y_pid_controller.setGoal(target_position[1])
        self.theta_pid_controller.setGoal(target_position[2])

        x_pid_output = self.x_pid_controller.calculate(
            self.get_pose().X(), target_position[0]
        )
        y_pid_output = self.y_pid_controller.calculate(
            self.get_pose().Y(), target_position[1]
        )
        theta_pid_output = self.theta_pid_controller.calculate(
            self.get_pose().rotation().radians(), target_position[2]
        )

        self.drive(x_pid_output, y_pid_output, theta_pid_output, True)
        return not self.orbiting

    def stop_orbiting(self):
        self.orbiting = False
        return True

    def enable_slow_mode(self):
        self.slow_mode = True
        return True

    def disable_slow_mode(self):
        self.slow_mode = False
        return True

    def zero_heading(self):
        self.gyro.reset()
        return True

    def smart_zero_heading(self):
        alliance = DriverStation.getAlliance()

        if alliance != None:
            self.gyro.set_yaw(
                self.get_pose().rotation().degrees()
                + 180.0 * (alliance == DriverStation.Alliance.kRed)
            )
        else:
            wpilib.reportError("Couldn't get alliance!")

        return True
