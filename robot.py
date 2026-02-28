from robot_container import RobotContainer
from commands2 import CommandScheduler
from wpilib import DriverStation
from wpilib import SmartDashboard, Field2d

import wpilib


class Robot(wpilib.TimedRobot):
    robot_container = RobotContainer()
    field = Field2d()

    def robotInit(self):
        SmartDashboard.putData("Field", self.field)

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


if __name__ == "__main__":
    wpilib.run(Robot)
