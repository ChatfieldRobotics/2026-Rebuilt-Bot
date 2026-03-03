from math import pi

import commands2.sysid
from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import VelocityVoltage, VoltageOut
from wpilib import DriverStation, SmartDashboard
from constants import CANConstants, ShooterConstants
from configs import ShooterConfigs
from simple_state_system import *
from time import sleep

import commands2

from subsystems.swerve_drive_subsystem import SwerveDriveSubsystem


class ShooterSubsytem(StateSystem):
    upper_roller_motor = TalonFX(CANConstants.upper_roller_motor)
    lower_roller_motor = TalonFX(CANConstants.lower_roller_motor)
    conveyor_motor = TalonFX(CANConstants.conveyor_motor)
    trigger_motor = TalonFX(CANConstants.trigger_motor)

    def __init__(self, robot_drive: SwerveDriveSubsystem):
        # Initialize the state machine
        super().__init__()

        self.robot_drive = robot_drive

        self.upper_roller_motor.setNeutralMode(NeutralModeValue.COAST)
        self.lower_roller_motor.setNeutralMode(NeutralModeValue.COAST)
        self.conveyor_motor.setNeutralMode(NeutralModeValue.COAST)
        self.trigger_motor.setNeutralMode(NeutralModeValue.BRAKE)

        self.upper_roller_motor.configurator.apply(ShooterConfigs.roller_config)
        self.lower_roller_motor.configurator.apply(ShooterConfigs.roller_config)
        self.conveyor_motor.configurator.apply(ShooterConfigs.roller_config)
        self.trigger_motor.configurator.apply(ShooterConfigs.roller_config)

    def periodic(self):
        # Run internal periodic functions
        super().periodic()

    def get_shooter_rps_from_dist(self, dist: float) -> float:
        return 8.5 * dist**2 - 40.35 * dist + 79.575

    @state
    def start_conveyor(self):
        self.conveyor_motor.set_control(VelocityVoltage(-20))
        return True

    @state
    def init_shooter(self):
        self.upper_roller_motor.set_control(
            VelocityVoltage(
                self.get_shooter_rps_from_dist(self.robot_drive.get_hub_dist())
            )
        )
        self.lower_roller_motor.set_control(
            VelocityVoltage(
                -self.get_shooter_rps_from_dist(self.robot_drive.get_hub_dist())
            )
        )

        sleep(1.0)

        return True

    @state
    def ensure_velocity(self):
        # print(self.upper_roller_motor.get_velocity().value_as_double, self.lower_roller_motor.get_velocity().value_as_double)
        target_rps = self.get_shooter_rps_from_dist(self.robot_drive.get_hub_dist())
        return (
            abs(self.upper_roller_motor.get_velocity().value_as_double - target_rps)
            < ShooterConstants.minimum_acceptable_closed_loop_error
            and abs(self.lower_roller_motor.get_velocity().value_as_double - target_rps)
            < ShooterConstants.minimum_acceptable_closed_loop_error
        )

    @state
    def advance_balls(self):
        self.trigger_motor.set_control(VelocityVoltage(-90))
        self.conveyor_motor.set_control(
            VelocityVoltage(
                -90
            )
        )
        return True

    @state
    def shoot(self):
        target_rps = self.get_shooter_rps_from_dist(self.robot_drive.get_hub_dist())

        self.upper_roller_motor.set_control(
            VelocityVoltage(
                target_rps
            )
        )
        self.lower_roller_motor.set_control(
            VelocityVoltage(
                -target_rps
            )
        )

        if not (abs(self.upper_roller_motor.get_velocity().value_as_double - target_rps)
            < ShooterConstants.minimum_acceptable_closed_loop_error
            and abs(self.lower_roller_motor.get_velocity().value_as_double - target_rps)
            < ShooterConstants.minimum_acceptable_closed_loop_error):
            return False
        
        self.trigger_motor.set_control(VelocityVoltage(-90))
        self.conveyor_motor.set_control(
            VelocityVoltage(
                -90
            )
        )

        return False

    @state
    def disable_shooter(self):
        self.upper_roller_motor.stopMotor()
        self.lower_roller_motor.stopMotor()
        self.conveyor_motor.stopMotor()
        self.trigger_motor.stopMotor()
        self.clear_queue()
        return True
