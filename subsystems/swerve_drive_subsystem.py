from re import L
from typing import List
import wpilib

from constants import DriveConstants
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.util import DriveFeedforwards
from phoenix6.hardware import Pigeon2
from simple_state_system import *
from subsystems.swerve_module_subsystem import SwerveModuleSubsystem
from subsystems.vision_subsystem import VisionSubsystem
from wpilib import DriverStation
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModuleState

class SwerveDriveSubsystem(StateSystem):
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
        Pose2d()
    )

    slow_mode = False

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
                    PIDConstants(2.25, 0.85, 0.31), PIDConstants(3.7, 1.6, 0.6)
                ),
                robot_config,
                lambda: (DriverStation.getAlliance() | DriverStation.Alliance.kBlue)
                == DriverStation.Alliance.kRed,
                self,
            )
        except Exception as e:
            wpilib.reportError("Error creating configs!")

    def periodic(self):
        # Run internal periodic functions
        super().periodic()

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
        )
        y_speed_delivered = (
            y_speed
            * DriveConstants.max_speed_meters_per_second
            * (DriveConstants.slow_mode_speed_percentage if self.slow_mode else 1.0)
        )
        rot_delivered = (
            rot
            * DriveConstants.max_angular_speed
            * (DriveConstants.slow_mode_speed_percentage if self.slow_mode else 1.0)
        )

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
    
    def drive_robot_relative(self, speeds: ChassisSpeeds, feedforwards: DriveFeedforwards):
        swerve_module_states = DriveConstants.drive_kinematics.toSwerveModuleStates(speeds)
        self.set_module_states(swerve_module_states)

    def get_robot_relative_speeds(self) -> ChassisSpeeds:
        module_states = self.get_module_states()
        robot_relative_speeds = DriveConstants.drive_kinematics.toChassisSpeeds(*module_states)
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

    @state
    def enable_slow_mode(self):
        self.slow_mode = True
        return True
    
    @state
    def disable_slow_mode(self):
        self.slow_mode = False
        return True

    @state
    def zero_heading(self):
        self.gyro.reset()
        return True

    @state 
    def smart_zero_heading(self):
        alliance = DriverStation.getAlliance()

        if alliance != None:
            self.gyro.set_yaw(self.get_pose().rotation().degrees() + 180.0 * (alliance == DriverStation.Alliance.kRed))
        else:
            wpilib.reportError("Couldn't get alliance!")

        return True