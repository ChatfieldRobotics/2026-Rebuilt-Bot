from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from constants import CANConstants
from simple_state_system import *


class HopperSubsystem(StateSystem):
    hopper_motor = TalonFX(CANConstants.hopper_motor)

    def __init__(self):
        # Initialize the state machine
        super().__init__()

        hopper_motor_config = TalonFXConfiguration()
        hopper_slot0 = hopper_motor_config.slot0

        hopper_slot0.k_p = 0.1
        hopper_slot0.k_i = 0.0
        hopper_slot0.k_d = 0.0

        self.hopper_motor.setNeutralMode(NeutralModeValue.COAST)

        self.hopper_motor.configurator.apply(hopper_motor_config)

    def periodic(self):
        # Run internal periodic functions
        super().periodic()

    @state
    def run_hopper(self, speed: float):
        self.hopper_motor.set(speed)
        return True
