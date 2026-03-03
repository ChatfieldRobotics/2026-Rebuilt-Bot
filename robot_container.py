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
        self.hopper_subsystem = HopperSubsystem()
        self.shooter_subsystem = ShooterSubsytem(self.robot_drive)

        self.set_controller_bindings()
        self.configure_named_commands()

        self.sendable_chooser = AutoBuilder.buildAutoChooser()

        SmartDashboard.putData(self.sendable_chooser)

    def active_rumble(self):
        self.driver_controller.getHID().setRumble(
            GenericHID.RumbleType.kBothRumble, 1.0
        )

    def configure_named_commands(self):
        NamedCommands.registerCommand(
            "toggle_intake",
            InstantCommand(
                lambda: self.hopper_subsystem.queue_states(
                    "toggle_hopper_position", "ensure_position", "toggle_intake_speed"
                )
            ),
        )
        NamedCommands.registerCommand(
            "shoot",
            InstantCommand(
                lambda: self.shooter_subsystem.queue_states(
                    "init_shooter", "ensure_velocity", "advance_balls"
                )
            ),
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
            InstantCommand(
                lambda: self.hopper_subsystem.queue_state("toggle_hopper")
            )
        )
        
        self.driver_controller.a().onTrue(
            InstantCommand(
                lambda: self.hopper_subsystem.outtake()
            )
        )

        self.driver_controller.b().onTrue(
            InstantCommand(
                lambda: self.hopper_subsystem.stop_intake_rollers()
            )
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
