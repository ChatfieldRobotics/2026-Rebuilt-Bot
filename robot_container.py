from commands2 import InstantCommand
from commands2.button import CommandXboxController

# from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.hopper_subsystem import HopperSubsystem
from subsystems.swerve_drive_subsystem import SwerveDriveSubsystem
from subsystems.vision_subsystem import VisionSubsystem

# from subsystems.hopper_subsystem import HopperSubsystem
# from subsystems.shooter_subsystem import ShooterSubsytem
from wpilib.interfaces import GenericHID


class RobotContainer:
    # intake_subsystem: IntakeSubsystem = IntakeSubsystem()
    # hopper_subsystem: HopperSubsystem = HopperSubsystem()
    # shooter_subsystem: ShooterSubsytem = ShooterSubsytem()

    driver_controller = CommandXboxController(0)

    def __init__(self):
        self.vision_subsystem = VisionSubsystem("APTCam")
        self.robot_drive = SwerveDriveSubsystem(self.vision_subsystem)
        self.hopper_subsystem = HopperSubsystem()

        self.set_controller_bindings()

    def active_rumble(self):
        self.driver_controller.getHID().setRumble(
            GenericHID.RumbleType.kBothRumble, 1.0
        )

    # def start_intake(self):
    #     self.intake_subsystem.queue_state(("run_intake", {"speed": 0.6}))
    #     self.hopper_subsystem.queue_state(("run_hopper", {"speed": 0.6}))

    # def stop_intake(self):
    #     self.intake_subsystem.queue_state(("run_intake", {"speed": 0.0}))
    #     self.hopper_subsystem.queue_state(("run_hopper", {"speed": 0.0}))

    # def start_shooting(self):
    #     self.shooter_subsystem.clear_queue()
    #     self.shooter_subsystem.queue_states(
    #         "set_shooter_speeds", "ensure_velocity", "advance_balls"
    #     )

    # def stop_shooting(self):
    #     self.shooter_subsystem.clear_queue()
    #     self.shooter_subsystem.queue_state("disable_shooter")

    def set_controller_bindings(self):
        self.robot_drive.queue_state(
            (
                "default_drive",
                {"driver_controller": self.driver_controller, "field_relative": True},
            )
        )

        self.driver_controller.leftBumper().onTrue(
            InstantCommand(lambda: self.robot_drive.queue_state("zero_heading", 0))
        )

        self.driver_controller.a().onTrue(
            InstantCommand(lambda: self.hopper_subsystem.extend_hopper())
        )

        # self.driver_controller.a().onTrue(self.start_intake)
        # self.driver_controller.a().onFalse(self.stop_intake)
