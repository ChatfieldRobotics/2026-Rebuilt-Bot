from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from constants import CANConstants
from simple_state_system import *


class IntakeSubsystem(StateSystem):
    intake_motor = TalonFX(CANConstants.intake_motor)

    def __init__(self):
        # Initialize the state machine
        super().__init__()

        intake_motor_config = TalonFXConfiguration()
        intake_slot0 = intake_motor_config.slot0

        intake_slot0.k_p = 0.1
        intake_slot0.k_i = 0.0
        intake_slot0.k_d = 0.0

        self.intake_motor.setNeutralMode(NeutralModeValue.COAST)

        self.intake_motor.configurator.apply(intake_motor_config)

    def periodic(self):
        # Run internal periodic functions
        super().periodic()

    @state
    def run_intake(self, speed: float):
        self.intake_motor.set(speed)
        return True
