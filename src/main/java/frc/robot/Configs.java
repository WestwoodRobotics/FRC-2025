package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PortConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = ModuleConstants.kTurningEncoderPositionFactor;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor/60); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingP)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(ModuleConstants.kTurningP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

        public static final class Elevator {
                public static final SparkMaxConfig elevator1Config = new SparkMaxConfig();
                public static final SparkMaxConfig elevator2Config = new SparkMaxConfig();
        
                static {
                        elevator1Config
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(60);
                        elevator1Config.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(0.1, 0.0, 0.0)
                                .outputRange(-1, 1);
                        //elevator1Config.softLimit.reverseSoftLimitEnabled(true);
                        //elevator1Config.softLimit.forwardSoftLimitEnabled(true);
                        //elevator1Config.softLimit.reverseSoftLimit(-71);
                        //elevator1Config.softLimit.forwardSoftLimit(0);
                }

                static {
                        elevator2Config
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(60);
                                
                        elevator2Config.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(0.1, 0.0, 0.0)
                                .outputRange(-1, 1);
                        elevator2Config.follow(PortConstants.kElevatorMotor1Port, true);
                        
                }

                
        }


}