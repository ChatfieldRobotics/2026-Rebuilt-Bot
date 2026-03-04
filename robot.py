from robot_container import RobotContainer
from commands2 import Command, CommandScheduler
from wpilib import DriverStation, RobotController
from wpilib import SmartDashboard, Field2d

import wpilib


class Robot(wpilib.TimedRobot):
    robot_container = RobotContainer()
    field = Field2d()
    autonomous_command: Command | None = None

    def robotInit(self):
        SmartDashboard.putData("Field", self.field)
        RobotController.setBrownoutVoltage(7.0)

    def robotPeriodic(self):
        self.field.setRobotPose(self.robot_container.robot_drive.get_pose())

        data = wpilib.DriverStation.getGameSpecificMessage()
        if data:
            if (
                data == "R"
                and DriverStation.getAlliance() == DriverStation.Alliance.kRed
            ):
                self.robot_container.active_rumble()
            elif (
                data == "B"
                and DriverStation.getAlliance() == DriverStation.Alliance.kBlue
            ):
                self.robot_container.active_rumble()

        CommandScheduler.getInstance().run()

    def teleopInit(self):
        if self.autonomous_command:
            self.autonomous_command.cancel()

    def autonomousInit(self):
        self.autonomous_command = self.robot_container.get_autonomous_command()

        if self.autonomous_command:
            self.autonomous_command.schedule()


if __name__ == "__main__":
    wpilib.run(Robot)
