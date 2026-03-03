from rev import SparkBaseConfig, SparkMaxConfig, ClosedLoopConfig, FeedbackSensor

from constants import ModuleConstants

from math import pi

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
    0.08, 0.0, 0.0
).velocityFF(driving_velocity_feed_forward).outputRange(-1.0, 1.0)

turning_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake).smartCurrentLimit(20)
turning_config.absoluteEncoder.inverted(True).positionConversionFactor(
    turning_factor
).velocityConversionFactor(turning_factor / 60.0)
turning_config.closedLoop.setFeedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(
    1.0, 0.0, 0.0
).outputRange(-1.0, 1.0).positionWrappingEnabled(True).positionWrappingInputRange(
    0, turning_factor
)



class HopperConfigs:
    hopper_k_p = 0.65
    hopper_k_i = 0.15
    hopper_k_d = 0.0

    motion_magic_cruise_velocity = 1600
    motion_magic_acceleration = 2400
    motion_magic_jerk = 3600

    intake_k_p = 0.1
    intake_k_i = 0.0
    intake_k_d = 0.0

    

