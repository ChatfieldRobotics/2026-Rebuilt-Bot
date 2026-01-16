from commands2.button import CommandXboxController
from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.swerve_drive_subsystem import SwerveDriveSubsystem
from subsystems.vision_subsystem import VisionSubsystem


class RobotContainer:
    vision_subsystem = VisionSubsystem("Right_APT_CAM")

    robot_drive: SwerveDriveSubsystem = SwerveDriveSubsystem(vision_subsystem)
    intake_subsystem: IntakeSubsystem = IntakeSubsystem()

    driver_controller = CommandXboxController(0)

    def __init__(self):
        self.set_controller_bindings()

    def set_controller_bindings(self):
        self.robot_drive.queue_state(
            (
                "default_drive",
                {"driver_controller": self.driver_controller, "field_relative": True},
            )
        )

        self.driver_controller.a().onTrue(
            lambda: self.intake_subsystem.queue_state(("run_intake", {"speed": 0.6}))
        )
        self.driver_controller.a().onFalse(
            lambda: self.intake_subsystem.queue_state(("run_intake", {"speed": 0.0}))
        )
