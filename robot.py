from robot_container import RobotContainer
from commands2 import CommandScheduler

import wpilib


class Robot(wpilib.TimedRobot):
    robot_container = RobotContainer()

    def robotPeriodic(self):
        CommandScheduler.getInstance().run()


if __name__ == "__main__":
    wpilib.run(Robot)
