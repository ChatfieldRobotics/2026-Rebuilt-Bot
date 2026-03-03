from phoenix6.configs import TalonFXSConfiguration, TalonFXConfiguration
from phoenix6.hardware import TalonFX, TalonFXS
from phoenix6.signals import MotorArrangementValue, NeutralModeValue
from phoenix6.controls import MotionMagicVoltage, MotionMagicVelocityVoltage
from constants import CANConstants, ShooterConstants
from configs import HopperConfigs
from simple_state_system import *


class HopperSubsystem(StateSystem):
    left_intake = TalonFXS(CANConstants.left_intake)
    right_intake = TalonFXS(CANConstants.right_intake)
    intake_motor = TalonFX(CANConstants.intake_motor)

    hopper_toggle = False

    def __init__(self):
        # Initialize the state machine
        super().__init__()

        hopper_motor_config = TalonFXSConfiguration()
        hopper_slot0 = hopper_motor_config.slot0
        motion_magic_configs = hopper_motor_config.motion_magic
        limit_configs = hopper_motor_config.current_limits

        hopper_motor_config.commutation.motor_arrangement = (
            MotorArrangementValue.MINION_JST
        )

        hopper_slot0.k_p = HopperConfigs.hopper_k_p
        hopper_slot0.k_i = HopperConfigs.hopper_k_i
        hopper_slot0.k_d = HopperConfigs.hopper_k_d

        motion_magic_configs.motion_magic_cruise_velocity = HopperConfigs.motion_magic_cruise_velocity
        motion_magic_configs.motion_magic_acceleration = HopperConfigs.motion_magic_acceleration
        motion_magic_configs.motion_magic_jerk = HopperConfigs.motion_magic_jerk
        
        limit_configs.stator_current_limit_enable = True
        limit_configs.stator_current_limit = 15

        self.left_intake.setNeutralMode(NeutralModeValue.COAST)
        self.right_intake.setNeutralMode(NeutralModeValue.COAST)

        self.left_intake.configurator.apply(hopper_motor_config)
        self.right_intake.configurator.apply(hopper_motor_config)

        intake_motor_config = TalonFXConfiguration()
        intake_slot0 = intake_motor_config.slot0

        intake_slot0.k_p = HopperConfigs.intake_k_p
        intake_slot0.k_i = HopperConfigs.hopper_k_i
        intake_slot0.k_d = HopperConfigs.intake_k_d

        self.intake_motor.setNeutralMode(NeutralModeValue.COAST)

        self.intake_motor.configurator.apply(intake_motor_config)

    def periodic(self):
        # Run internal periodic functions
        super().periodic()

    @state
    def toggle_hopper_position(self):
        self.hopper_toggle = not self.hopper_toggle

        if self.hopper_toggle:
            self.extend_hopper()
        else:
            self.retract_hopper()

        return True

    @state
    def ensure_position(self):
        return (
            abs(self.left_intake.get_velocity().value_as_double) < 0.1
            and abs(self.right_intake.get_velocity().value_as_double) < 0.1
        )

    @state
    def toggle_intake_speed(self):
        if self.hopper_toggle:
            self.intake_motor.set(-0.9)
        else:
            self.intake_motor.set(0.0)

        return True

    def extend_hopper(self):
        self.left_intake.set_control(MotionMagicVoltage(15.5))
        self.right_intake.set_control(MotionMagicVoltage(-15.5))

    def retract_hopper(self):
        self.left_intake.set_control(MotionMagicVoltage(0.0))
        self.right_intake.set_control(MotionMagicVoltage(0.0))

    @state
    def run_intake(self, speed: float):
        self.intake_motor.set(speed)
        return True
