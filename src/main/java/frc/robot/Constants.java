package frc.robot;






import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 6.77;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = Math.PI/3; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26);
    // Distance between front and back wheels on robot
    public static final Translation2d frontLeftPos = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
    public static final Translation2d frontRightPos = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
    public static final Translation2d rearLeftPos = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
    public static final Translation2d rearRightPos = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        frontLeftPos,
        frontRightPos,
        rearLeftPos,
        rearRightPos);

    // Angular offsets of the modules relative to the chassis in radians
    // These values are the angle offset of the wheels when the robot is facing forwards (Absolute Encoders)
    //DO NOT CHANGE THESE VALUES UNLESS YOU KNOW WHAT YOU'RE DOING!!
    public static final double kFrontLeftChassisAngularOffset = -Math.PI/2;
    public static final double kFrontRightChassisAngularOffset = 0; 
    public static final double kRearLeftChassisAngularOffset = Math.PI;
    public static final double kRearRightChassisAngularOffset = Math.PI/2;

    




    public static final boolean kGyroReversed = false;

    public static final double slowModeMultiplier = 0.25;

    // PID constants for the new PIDController in the driveCommand class
    public static final double kP = 0.015;
    public static final double kI = 0.0;
    public static final double kD = 0.0005;
  }

  public static final class PortConstants{
    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 17;
    public static final int kRearLeftDrivingCanId = 11;
    public static final int kFrontRightDrivingCanId = 13;
    public static final int kRearRightDrivingCanId = 15;

    public static final int kFrontLeftTurningCanId = 12;
    public static final int kRearLeftTurningCanId = 14;
    public static final int kFrontRightTurningCanId = 10;
    public static final int kRearRightTurningCanId = 16;




    public static final int testCANId = 0; //TODO: Change this to the CAN ID of the motor you want to test



    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kElevatorMotor1Port = 50;
    public static final int kElevatorMotor2Port = 34;
    public static final int kIntakeMotorSidePort = 33; //TODO: Change this value
    public static final int kIntakeMotorTopPort = 57;
    public static final int outtakeMotorPort = 59;
    public static final int kCANdiPort = 20;
    

  }

  public static final class TransportConstants{

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = VortexMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 12 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 20) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians


    // These PID Gains have been tested
    public static final double kDrivingP = 0.0000002; // 0.2 original value
    public static final double kDrivingI = 0; 
    public static final double kDrivingD = 0.0001; // 0.0001
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps; // 1

    public static final double kDrivingMinOutput = 0; //-1
    public static final double kDrivingMaxOutput = 0; //1

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0.2;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

  }

  public static final class ControllerConstants {
    public static final double kDriveDeadband = 0.2;
  }

  public static final class AutoConstants {
    //These constants need to tuned when setting up Auton Paths
    public static final double kMaxModuleSpeedMetersPerSecond = 7;
     //Distance from the center of the robot to the farthest SwerveModule
    public static final double kDriveBaseRadius = 0.4; //meters


    public static final double kMaxAccelerationMetersPerSecondSquared = 6;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;


    //These control x, y, and theta corrections when doing path following, 
    //basically like a joystick input to correct for misalignment, 
    //Units are m/s per meter of offset or rad/s per radian of offset

    public static final double kPXController = 2;
    public static final double kPYController = 2;
    public static final double kPThetaController = 2;
    
    public static final double kIXController = 0;
    public static final double kIYController = 0;
    public static final double kIThetaController = 0;

    public static final double kDXController = 0;
    public static final double kDYController = 0;
    public static final double kDThetaController = 0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class VortexMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }
  public static final class IntakeConstants {

    public static final double kRPMConversionFactor = 1.0;


    public static final double kP = 0.0002;
    public static final double kI = 0;
    public static final double kD = 0;


    public static final double kSideI = 0;


    public static final double kSideP = 0;


    public static final double kSideD = 0;


    public static final double kTopP = 0;


    public static final double kTopI = 0;
 

    public static final double kTopD = 0;
  } 

  
  public static final class UtilityConstants {
    public static final boolean debugMode = true;
  }


  public static final class ElevatorConstants{
    // public static final double kElevatorHomePosition = 0;
    // public static final double kElevatorL2Position = -17.5;
    // public static final double kElevatorL3Position = -37.5;
    // public static final double kElevatorL4Position = -73;

    //OG GR is 15:1
    //NEW GR IS 9:1

    //OLD POSES

    // public static final double kElevatorHomePosition = 0;
    // public static final double kElevatorL2Position = -21.875;
    // public static final double kElevatorL3Position = -46.875;
    // public static final double kElevatorL35Position = -80.25; //-91.25 OG
    // public static final double kElevatorL4Position = -90.25; //-91.25 OG
    // public static final double kElevatorBargePosition = -93.5;

    //NEW POSES
    public static final double kElevatorHomePosition = 0;
    public static final double kElevatorL2Position = -13.125; // (-21.875/15)*9
    public static final double kElevatorL3Position = -28.125; // (-46.875/15)*9
    public static final double kElevatorL35Position = -48.15; // (-80.25/15)*9
    public static final double kElevatorL4Position = -53.9; // (-90.25/15)*9
    public static final double kElevatorBargePosition = -50.283062; // (-93.5/15)*9




    public static final double kP = 0.075;
    public static final double kI = 0;
    public static final double kD = 0;


    // Current detection constants
    public static final double kCurrentDetectionThreshold = 30.0; // Threshold for detecting a current spike
    public static final int kCurrentWarmupSamples = 30; // Number of initial samples to ignore
    public static final int kCurrentWindowSize = 5; // Size of the rolling average window
  }

  public static final class OuttakeConstants{
    //PID
    public static final double kP = 100;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static final class CameraConstants{
    public static final double kReefCameraToRobotX = -0.25019; 
    public static final double kReefCameraToRobotY = 0.18685;
    public static final double kReefCameraToRobotZ = -0.3604;

    public static final double kReefCameraToRobotPitch = 0;
    public static final double kReefCameraToRobotYaw = 0;
    public static final double kReefCameraToRobotRoll = 0;

    public static final Transform3d kReefCameraToRobotTransform = new Transform3d(
      new Translation3d(kReefCameraToRobotX, kReefCameraToRobotY, kReefCameraToRobotZ),
      new Rotation3d(kReefCameraToRobotPitch, kReefCameraToRobotYaw, kReefCameraToRobotRoll)
    );



    public static final double kHumanPlayerCameraToRobotX = -0.25019;
    public static final double kHumanPlayerCameraToRobotY = -0.18685;
    public static final double kHumanPlayerCameraToRobotZ = -0.3604;

    public static final double kHumanPlayerCameraToRobotPitch = 0;
    public static final double kHumanPlayerCameraToRobotYaw = 0;
    public static final double kHumanPlayerCameraToRobotRoll = 0;

    public static final Transform3d kHumanPlayerCameraToRobotTransform = new Transform3d(
      new Translation3d(kHumanPlayerCameraToRobotX, kHumanPlayerCameraToRobotY, kHumanPlayerCameraToRobotZ),
      new Rotation3d(kHumanPlayerCameraToRobotPitch, kHumanPlayerCameraToRobotYaw, kHumanPlayerCameraToRobotRoll)
    );

  }

  public class TuskConstants{
    public static final int kTuskPivotMotorId = 39;
    public static final int kTuskRollerMotorId = 38;


    public static final double kGroundPosition = -13.5;
    public static final double kL3Position = -9;
    public static final double kL4Position = -8.75;
    public static final double kProcessorPosition = -9.1;
    public static final double kHomePosition = 0;
    public static final double kInterruptedPosition = 0;

    public static final double kOutPosition = -13.3;
    public static final double kInPosition = 0.1;

    public static final double kRollerP = 0.075;
    public static final double kRollerI = 0;
    public static final double kRollerD = 0;

    public static final double kPivotP = 0.035;
    public static final double kPivotI = 0.00008;
    public static final double kPivotD = 0.0;
  }


  
  
}
