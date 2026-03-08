from commands2 import InstantCommand, ProxyCommand, RunCommand, SequentialCommandGroup
from commands2.button import CommandXboxController
from wpilib import SendableChooser, SmartDashboard
from wpilib.interfaces import GenericHID

from constants import OIConstants
from subsystems.hopper_subsystem import HopperSubsystem
from subsystems.shooter_subsystem import ShooterSubsytem
from subsystems.swerve_drive_subsystem import SwerveDriveSubsystem
from subsystems.vision_subsystem import VisionSubsystem

from pathplannerlib.auto import AutoBuilder, NamedCommands


class RobotContainer:
    driver_controller = CommandXboxController(OIConstants.driver_controller_port)
    sendable_chooser: SendableChooser | None = None

    def __init__(self):
        self.vision_subsystem = VisionSubsystem("APTCam")
        self.robot_drive = SwerveDriveSubsystem(self.vision_subsystem)
        self.shooter_subsystem = ShooterSubsytem(self.robot_drive)
        self.hopper_subsystem = HopperSubsystem()

        self.set_controller_bindings()
        self.configure_named_commands()

        self.sendable_chooser = AutoBuilder.buildAutoChooser()

        SmartDashboard.putData(self.sendable_chooser)

    def configure_named_commands(self):
        NamedCommands.registerCommand(
            "toggle_intake",
            InstantCommand(lambda: self.hopper_subsystem.queue_state("toggle_hopper")),
        )
        NamedCommands.registerCommand(
            "shoot",
            InstantCommand(lambda: self.shooter_subsystem.queue_state("shoot")),
        )
        NamedCommands.registerCommand(
            "disable_shooter",
            InstantCommand(
                lambda: self.shooter_subsystem.queue_state("disable_shooter", 0)
            )
        )

    def set_controller_bindings(self):
        self.robot_drive.setDefaultCommand(
            RunCommand(
                lambda: self.robot_drive.default_drive(self.driver_controller, True),
                self.robot_drive,
            )
        )

        self.driver_controller.leftBumper().onTrue(
            InstantCommand(lambda: self.robot_drive.zero_heading(), self.robot_drive)
        )

        self.driver_controller.povRight().onTrue(
            InstantCommand(lambda: self.hopper_subsystem.queue_state("toggle_hopper"))
        )

        self.driver_controller.a().onTrue(
            InstantCommand(
                lambda: [
                    self.hopper_subsystem.queue_state("outtake", 0),
                    self.shooter_subsystem.outtake(),
                ]
            )
        )

        self.driver_controller.a().onFalse(
            InstantCommand(
                lambda: [
                    self.hopper_subsystem.stop_intake_rollers(),
                    self.shooter_subsystem.queue_state("disable_shooter", 0),
                ]
            )
        )

        self.driver_controller.b().onTrue(
            InstantCommand(lambda: self.hopper_subsystem.toggle_intake_roller())
        )

        self.driver_controller.rightTrigger().onTrue(
            InstantCommand(lambda: self.shooter_subsystem.queue_states("shoot"))
        )

        self.driver_controller.rightTrigger().onFalse(
            InstantCommand(
                lambda: self.shooter_subsystem.queue_state("disable_shooter", 0)
            )
        )

    def get_autonomous_command(self):
        return SequentialCommandGroup(
            ProxyCommand(lambda: self.sendable_chooser.getSelected())
        )
