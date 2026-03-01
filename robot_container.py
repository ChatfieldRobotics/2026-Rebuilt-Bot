from commands2 import InstantCommand, ProxyCommand, RunCommand, SequentialCommandGroup
from commands2.button import CommandXboxController
from wpilib import SendableChooser, SmartDashboard

# from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.hopper_subsystem import HopperSubsystem
from subsystems.shooter_subsystem import ShooterSubsytem
from subsystems.swerve_drive_subsystem import SwerveDriveSubsystem
from subsystems.vision_subsystem import VisionSubsystem

# from subsystems.hopper_subsystem import HopperSubsystem
# from subsystems.shooter_subsystem import ShooterSubsytem
from wpilib.interfaces import GenericHID

from pathplannerlib.auto import AutoBuilder, NamedCommands


class RobotContainer:
    driver_controller = CommandXboxController(0)
    sendable_chooser: SendableChooser | None = None

    def __init__(self):
        self.vision_subsystem = VisionSubsystem("APTCam")
        self.robot_drive = SwerveDriveSubsystem(self.vision_subsystem)
        self.hopper_subsystem = HopperSubsystem()
        self.shooter_subsystem = ShooterSubsytem()

        self.set_controller_bindings()
        self.configure_named_commands()

        self.sendable_chooser = AutoBuilder.buildAutoChooser()

        SmartDashboard.putData(self.sendable_chooser)

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

    def configure_named_commands(self):
        NamedCommands.registerCommand("toggle_intake", 
            InstantCommand(lambda: self.hopper_subsystem.queue_states(
                "toggle_hopper_position", "ensure_position", "toggle_intake_speed"
        )))
        NamedCommands.registerCommand("shoot", 
            InstantCommand(lambda: self.shooter_subsystem.queue_states(
                "init_shooter", "ensure_velocity", "advance_balls"
        )))

    def set_controller_bindings(self):
        self.robot_drive.setDefaultCommand(
            RunCommand(lambda: self.robot_drive.default_drive(self.driver_controller, True), self.robot_drive)
        )

        self.driver_controller.leftBumper().onTrue(
            InstantCommand(lambda: self.robot_drive.zero_heading(), self.robot_drive)
        )

        self.driver_controller.povRight().onTrue(
            InstantCommand(lambda: self.hopper_subsystem.queue_states(
                "toggle_hopper_position", "ensure_position", "toggle_intake_speed"
            ))
        )
        
        self.driver_controller.rightTrigger().onTrue(
            InstantCommand(lambda: self.shooter_subsystem.queue_states(
                "init_shooter", "ensure_velocity", "advance_balls"
            ))
        )

        self.driver_controller.rightTrigger().onFalse(
            InstantCommand(lambda: self.shooter_subsystem.queue_state("disable_shooter", 0))
        )

        # self.driver_controller.a().onTrue(self.start_intake)
        # self.driver_controller.a().onFalse(self.stop_intake)

    def get_autonomous_command(self):
        return SequentialCommandGroup(
            ProxyCommand(lambda: self.sendable_chooser.getSelected())
        )