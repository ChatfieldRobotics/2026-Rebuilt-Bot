from rev import SparkBaseConfig, SparkMaxConfig, ClosedLoopConfig

from constants import ModuleConstants

from math import pi

neo_config = SparkMaxConfig()

neo_config.setIdleMode(SparkBaseConfig.IdleMode.kCoast).smartCurrentLimit(10)
neo_config.closedLoop.setFeedbackSensor(
    ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
).pid(0.0, 0.0, 0.0).velocityFF(0.00045).outputRange(-1, 1)
neo_config.closedLoop.maxMotion.maxVelocity(700).maxAcceleration(
    1800
).allowedClosedLoopError(0.01)

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
driving_config.closedLoop.setFeedbackSensor(
    ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
).pid(0.04, 0.0, 0.0).velocityFF(driving_velocity_feed_forward).outputRange(-1.0, 1.0)
driving_config.closedLoop.maxMotion.maxVelocity(10).maxAcceleration(
    20
).allowedClosedLoopError(0.01)

turning_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake).smartCurrentLimit(20)
turning_config.absoluteEncoder.inverted(True).positionConversionFactor(
    turning_factor
).velocityConversionFactor(turning_factor / 60.0)
turning_config.closedLoop.setFeedbackSensor(
    ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder
).pid(1.0, 0.0, 0.0).outputRange(-1.0, 1.0).positionWrappingEnabled(
    True
).positionWrappingInputRange(
    0, turning_factor
)
turning_config.closedLoop.maxMotion.maxVelocity(10).maxAcceleration(
    20
).allowedClosedLoopError(0.01)
