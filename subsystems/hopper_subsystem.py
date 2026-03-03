from phoenix6.hardware import TalonFX, TalonFXS
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import MotionMagicVoltage
from configs import HopperConfigs
from constants import CANConstants, HopperConstants
from simple_state_system import *
from time import sleep
from commands2.button import CommandXboxController

class HopperSubsystem(StateSystem):
    """Subsystem responsible for controlling the hopper mechanism, which includes the intake and the positioning of the hopper itself."""
    left_intake = TalonFXS(CANConstants.left_intake_motor)
    right_intake = TalonFXS(CANConstants.right_intake_motor)
    intake_motor = TalonFX(CANConstants.intake_motor)

    target_hopper_position = 0.0
    hopper_toggle = False

    def __init__(self):
        # Initialize the state machine
        super().__init__()
        self.left_intake.setNeutralMode(NeutralModeValue.COAST)
        self.right_intake.setNeutralMode(NeutralModeValue.COAST)
        self.intake_motor.setNeutralMode(NeutralModeValue.COAST)

        self.left_intake.configurator.apply(HopperConfigs.hopper_motor_config)
        self.right_intake.configurator.apply(HopperConfigs.hopper_motor_config)
        self.intake_motor.configurator.apply(HopperConfigs.intake_motor_config)

    def periodic(self):
        # Run internal periodic functions
        super().periodic()

    def wait_for_hopper_position(self, target_position: float, tolerance: float = 0.1):
        """Waits until the hopper reaches the target position within a specified tolerance."""
        while (
            abs(self.left_intake.get_position().value_as_double - target_position) > tolerance
            and abs(self.right_intake.get_position().value_as_double - target_position) > tolerance
        ):
            sleep(0.02)

    def toggle_hopper(self):
        """Toggles the hopper between its extended and retracted positions. When toggled, it will move the hopper to the target position and then set the intake motor speed accordingly.
        """
        self.hopper_toggle = not self.hopper_toggle

        if self.hopper_toggle:
            self.target_hopper_position = HopperConstants.extended_position
        else:
            self.target_hopper_position = HopperConstants.retracted_position

        self.left_intake.set_control(MotionMagicVoltage(self.target_hopper_position))
        self.right_intake.set_control(MotionMagicVoltage(-self.target_hopper_position))

        self.wait_for_hopper_position(self.target_hopper_position)

        if self.hopper_toggle:
            self.intake_motor.set(HopperConstants.intake_speed)
        else:
            self.intake_motor.set(0.0)
    
    def stop_intake_rollers(self):
        """Stops the intake rollers by setting the intake motor speed to zero."""
        self.intake_motor.set(0.0)

    def outtake(self):
        """Sets the intake motor to outtake speed, which is defined in constants.py. This can be used to eject balls from the hopper."""
        self.target_hopper_position = HopperConstants.extended_position

        self.left_intake.set_control(MotionMagicVoltage(self.target_hopper_position))
        self.right_intake.set_control(MotionMagicVoltage(-self.target_hopper_position))

        self.wait_for_hopper_position(self.target_hopper_position)

        self.intake_motor.set(-HopperConstants.intake_speed)