from rev import SparkMaxConfig, FeedbackSensor
from phoenix6.configs import TalonFXSConfiguration, TalonFXConfiguration
from phoenix6.signals import MotorArrangementValue

from constants import ModuleConstants

from math import pi

class DriveConfigs:
    driving_config = SparkMaxConfig()
    turning_config = SparkMaxConfig()

    driving_factor = (
        ModuleConstants.wheel_diameter_meters * pi / ModuleConstants.driving_motor_reduction
    )
    turning_factor = 2 * pi
    driving_velocity_feed_forward = 1 / ModuleConstants.drive_wheel_free_speed_rps

    driving_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake).smartCurrentLimit(50)
    driving_config.encoder.positionConversionFactor(
        driving_factor
    ).velocityConversionFactor(driving_factor / 60.0)
    driving_config.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
        2.25,
        0.85,
        0.31,
    ).velocityFF(driving_velocity_feed_forward).outputRange(-1.0, 1.0)

    turning_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake).smartCurrentLimit(20)
    turning_config.absoluteEncoder.inverted(True).positionConversionFactor(
        turning_factor
    ).velocityConversionFactor(turning_factor / 60.0)
    turning_config.closedLoop.setFeedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(
        3.70,
        1.60,
        0.60,
    ).outputRange(-1.0, 1.0).positionWrappingEnabled(True).positionWrappingInputRange(
        0, turning_factor
    )



class HopperConfigs:
    hopper_motor_config = TalonFXSConfiguration()
    hopper_slot0 = hopper_motor_config.slot0
    motion_magic_configs = hopper_motor_config.motion_magic
    limit_configs = hopper_motor_config.current_limits

    hopper_motor_config.commutation.motor_arrangement = (
        MotorArrangementValue.MINION_JST
    )

    hopper_slot0.k_p = 0.65
    hopper_slot0.k_i = 0.15
    hopper_slot0.k_d = 0.0

    motion_magic_configs.motion_magic_cruise_velocity = 1600
    motion_magic_configs.motion_magic_acceleration = 2400
    motion_magic_configs.motion_magic_jerk = 3600
    
    limit_configs.stator_current_limit_enable = True
    limit_configs.stator_current_limit = 15

    intake_motor_config = TalonFXConfiguration()
    intake_slot0 = intake_motor_config.slot0

    intake_slot0.k_p = 0.1
    intake_slot0.k_i = 0.0
    intake_slot0.k_d = 0.0


class ShooterConfigs:
    roller_config = TalonFXConfiguration()
    intake_slot0 = roller_config.slot0

    intake_slot0.k_s = 0.18
    intake_slot0.k_v = 0.123
    intake_slot0.k_p = 0.4
    intake_slot0.k_d = 0.006



