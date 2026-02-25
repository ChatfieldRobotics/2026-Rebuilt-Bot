from rev import (
    SparkMax,
    SparkAbsoluteEncoder,
    SparkRelativeEncoder,
    SparkClosedLoopController,
    SparkLowLevel,
    SparkBase,
    ResetMode,
    PersistMode,
)

from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from wpimath.geometry import Rotation2d

import configs


class SwerveModuleSubsystem:
    def __init__(self, driving_id: int, turning_id: int, chassis_angular_offset: float):
        self.driving_spark = SparkMax(driving_id, SparkLowLevel.MotorType.kBrushless)
        self.turning_spark = SparkMax(turning_id, SparkLowLevel.MotorType.kBrushless)

        self.driving_encoder: SparkRelativeEncoder = self.driving_spark.getEncoder()
        self.turning_encoder: SparkAbsoluteEncoder = (
            self.turning_spark.getAbsoluteEncoder()
        )

        self.driving_closed_loop_controller: SparkClosedLoopController = (
            self.driving_spark.getClosedLoopController()
        )
        self.turning_closed_loop_controller: SparkClosedLoopController = (
            self.turning_spark.getClosedLoopController()
        )

        self.driving_spark.configure(
            configs.driving_config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
        self.turning_spark.configure(
            configs.turning_config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        self.chassis_angular_offset = chassis_angular_offset
        self.desired_state = SwerveModuleState(
            0.0, Rotation2d(self.turning_encoder.getPosition())
        )

        self.driving_encoder.setPosition(0.0)

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.driving_encoder.getVelocity(),
            Rotation2d(
                self.turning_encoder.getPosition() - self.chassis_angular_offset
            ),
        )

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self.driving_encoder.getPosition(),
            Rotation2d(
                self.turning_encoder.getPosition() - self.chassis_angular_offset
            ),
        )

    def setDesiredState(self, desired_state: SwerveModuleState):
        corrected_desired_state = SwerveModuleState()
        corrected_desired_state.speed = desired_state.speed
        corrected_desired_state.angle = desired_state.angle.rotateBy(
            Rotation2d(self.chassis_angular_offset)
        )

        corrected_desired_state.optimize(Rotation2d(self.turning_encoder.getPosition()))

        self.driving_closed_loop_controller.setReference(
            corrected_desired_state.speed,
            SparkBase.ControlType.kVelocity,
        )
        self.turning_closed_loop_controller.setReference(
            corrected_desired_state.angle.radians(), SparkBase.ControlType.kPosition
        )

        self.desired_state = corrected_desired_state

    def reset_encoders(self):
        self.driving_encoder.setPosition(0)
