from robot_container import RobotContainer
from commands2 import CommandScheduler
from wpilib import DriverStation

import wpilib


class Robot(wpilib.TimedRobot):
    robot_container = RobotContainer()

    def robotPeriodic(self):
        data = wpilib.DriverStation.getGameSpecificMessage()
        if data:
            if (
                data == "R"
                and (DriverStation.getAlliance() | DriverStation.Alliance.kBlue)
                == DriverStation.Alliance.kRed
            ):
                self.robot_container.active_rumble()
            elif (
                data == "B"
                and (DriverStation.getAlliance() | DriverStation.Alliance.kRed)
                == DriverStation.Alliance.kBlue
            ):
                self.robot_container.active_rumble()

        CommandScheduler.getInstance().run()


if __name__ == "__main__":
    wpilib.run(Robot)
