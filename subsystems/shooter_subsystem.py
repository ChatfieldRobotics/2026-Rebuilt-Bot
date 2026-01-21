from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import MotionMagicVelocityVoltage
from constants import CANConstants, ShooterConstants
from simple_state_system import *


class ShooterSubsytem(StateSystem):
    upper_roller_motor = TalonFX(CANConstants.upper_roller_motor)
    lower_roller_motor = TalonFX(CANConstants.lower_roller_motor)
    advancement_motor = TalonFX(CANConstants.advancement_motor)

    motion_magic_velocity_voltage: MotionMagicVelocityVoltage = (
        MotionMagicVelocityVoltage.with_slot(0)
    )

    def __init__(self):
        # Initialize the state machine
        super().__init__()

        roller_config = TalonFXConfiguration()
        motion_magic_config = roller_config.motion_magic
        intake_slot0 = roller_config.slot0

        intake_slot0.k_p = 0.5
        intake_slot0.k_i = 0.1
        intake_slot0.k_d = 0.0001

        motion_magic_config.motion_magic_cruise_velocity = 60
        motion_magic_config.motion_magic_acceleration = 90
        motion_magic_config.motion_magic_jerk = 135

        self.upper_roller_motor.setNeutralMode(NeutralModeValue.COAST)
        self.lower_roller_motor.setNeutralMode(NeutralModeValue.COAST)
        self.advancement_motor.setNeutralMode(NeutralModeValue.COAST)

        self.upper_roller_motor.configurator.apply(roller_config)
        self.lower_roller_motor.configurator.apply(roller_config)
        self.advancement_motor.configurator.apply(roller_config)

    def periodic(self):
        # Run internal periodic functions
        super().periodic()

    @state
    def set_shooter_speeds(self):
        self.upper_roller_motor.set_control(
            self.motion_magic_velocity_voltage.with_velocity(
                ShooterConstants.optimal_upper_roller_rps
            )
        )
        self.lower_roller_motor.set_control(
            self.motion_magic_velocity_voltage.with_velocity(
                ShooterConstants.optimal_lower_roller_rps
            )
        )
        return True

    @state
    def ensure_velocity(self):
        return (
            abs(self.upper_roller_motor.get_closed_loop_error())
            < ShooterConstants.minimum_acceptable_closed_loop_error
            and abs(self.lower_roller_motor.get_closed_loop_error())
            < ShooterConstants.minimum_acceptable_closed_loop_error
        )

    @state
    def advance_balls(self):
        self.advancement_motor.set(ShooterConstants.advancement_motor_percentage)
        return True

    @state
    def disable_shooter(self):
        self.upper_roller_motor.stopMotor()
        self.lower_roller_motor.stopMotor()
        self.advancement_motor.stopMotor()
        return True
