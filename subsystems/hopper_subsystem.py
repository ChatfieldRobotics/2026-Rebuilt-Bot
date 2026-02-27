from phoenix6.configs import TalonFXSConfiguration
from phoenix6.hardware import TalonFXS
from phoenix6.signals import MotorArrangementValue, NeutralModeValue
from phoenix6.controls import MotionMagicVoltage
from constants import CANConstants
from simple_state_system import *


class HopperSubsystem(StateSystem):
    left_intake = TalonFXS(CANConstants.left_intake)
    right_intake = TalonFXS(CANConstants.right_intake)

    def __init__(self):
        # Initialize the state machine
        super().__init__()

        hopper_motor_config = TalonFXSConfiguration()
        hopper_slot0 = hopper_motor_config.slot0
        motion_magic_configs = hopper_motor_config.motion_magic

        hopper_motor_config.commutation.motor_arrangement = MotorArrangementValue.MINION_JST    

        hopper_slot0.k_p = 0.1
        hopper_slot0.k_i = 0.0
        hopper_slot0.k_d = 0.0

        motion_magic_configs.motion_magic_cruise_velocity = 20
        motion_magic_configs.motion_magic_acceleration = 30
        motion_magic_configs.motion_magic_jerk = 45

        self.left_intake.setNeutralMode(NeutralModeValue.COAST)
        self.right_intake.setNeutralMode(NeutralModeValue.COAST)

        self.left_intake.configurator.apply(hopper_motor_config)
        self.right_intake.configurator.apply(hopper_motor_config)

    def periodic(self):
        # Run internal periodic functions
        super().periodic()

        print(self.left_intake.get_position(), self.right_intake.get_position())

    @state
    def extend_hopper(self):
        self.left_intake.set_control(MotionMagicVoltage(15.25))
        self.right_intake.set_control(MotionMagicVoltage(-15.25))
        return True