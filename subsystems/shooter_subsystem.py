from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import MotionMagicVelocityVoltage
from wpilib import DriverStation
from constants import CANConstants, ShooterConstants
from simple_state_system import *
from time import sleep


class ShooterSubsytem(StateSystem):
    upper_roller_motor = TalonFX(CANConstants.upper_roller_motor)
    lower_roller_motor = TalonFX(CANConstants.lower_roller_motor)
    conveyor_motor = TalonFX(CANConstants.conveyor_motor)
    trigger_motor = TalonFX(CANConstants.trigger_motor)

    def __init__(self):
        # Initialize the state machine
        super().__init__()

        roller_config = TalonFXConfiguration()
        motion_magic_config = roller_config.motion_magic
        intake_slot0 = roller_config.slot0

        intake_slot0.k_p = 0.475
        intake_slot0.k_i = 0.4
        intake_slot0.k_d = 0.0

        motion_magic_config.motion_magic_cruise_velocity = 8000
        motion_magic_config.motion_magic_acceleration = 14000
        motion_magic_config.motion_magic_jerk = 20000

        self.upper_roller_motor.setNeutralMode(NeutralModeValue.COAST)
        self.lower_roller_motor.setNeutralMode(NeutralModeValue.COAST)
        self.conveyor_motor.setNeutralMode(NeutralModeValue.COAST)
        self.trigger_motor.setNeutralMode(NeutralModeValue.BRAKE)

        self.upper_roller_motor.configurator.apply(roller_config)
        self.lower_roller_motor.configurator.apply(roller_config)
        self.conveyor_motor.configurator.apply(roller_config)
        self.trigger_motor.configurator.apply(roller_config)

        self.start_timer = None

    def periodic(self):
        # Run internal periodic functions
        super().periodic()

    @state
    def start_conveyor(self):
        self.conveyor_motor.set_control(MotionMagicVelocityVoltage(-20))
        return True

    @state
    def init_shooter(self):
        self.upper_roller_motor.set_control(
            MotionMagicVelocityVoltage(
                ShooterConstants.optimal_upper_roller_rps
            )
        )
        self.lower_roller_motor.set_control(
            MotionMagicVelocityVoltage(
                ShooterConstants.optimal_lower_roller_rps
            )
        )

        sleep(0.25)

        return True

    @state
    def ensure_velocity(self):
        # print(self.upper_roller_motor.get_velocity().value_as_double, self.lower_roller_motor.get_velocity().value_as_double)
        return (
            abs(self.upper_roller_motor.get_closed_loop_error().value_as_double)
            < ShooterConstants.minimum_acceptable_closed_loop_error
            and abs(self.lower_roller_motor.get_closed_loop_error().value_as_double)
            < ShooterConstants.minimum_acceptable_closed_loop_error
        )

    @state
    def advance_balls(self):
        self.trigger_motor.set_control(MotionMagicVelocityVoltage(-90))
        self.conveyor_motor.set_control(
            MotionMagicVelocityVoltage(
                -90
            )
        )
        return True

    @state
    def disable_shooter(self):
        self.upper_roller_motor.stopMotor()
        self.lower_roller_motor.stopMotor()
        self.conveyor_motor.stopMotor()
        self.trigger_motor.stopMotor()
        self.clear_queue()
        return True
