// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.sensors.PhotonVisionCameras;
import frc.robot.subsystems.utils.KalmanLocalization;
//import frc.robot.subsystems.utils.KalmanLocalization;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;


public class SwerveDrive extends SubsystemBase {
  // Create MAXSwerveModules
  private MAXSwerveModule m_frontLeft;
  private MAXSwerveModule m_frontRight;
  private MAXSwerveModule m_rearLeft;
  private MAXSwerveModule m_rearRight;

  private boolean isNoKalmanFilterBackUpMode;
  
  StructArrayPublisher<SwerveModuleState> publisher;
  private boolean alignFastMode = true;
  
  // The gyro sensor
  public Gyro m_gyro;
  
  KalmanLocalization kalmanLocalization;
  
  private boolean slowMode = false;
  private boolean isTestMode = false;
  private RobotConfig config;
  public Field2d fieldVisualization;
  public PhotonVisionCameras m_cameras;

  /** Creates a new DriveSubsystem. */
  public SwerveDrive(PhotonVisionCameras cameras) {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    
    // Initialize module objects
    m_frontLeft = new MAXSwerveModule(
      PortConstants.kFrontLeftDrivingCanId,
      PortConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);
      
    m_frontRight = new MAXSwerveModule(
      PortConstants.kFrontRightDrivingCanId,
      PortConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);
      
    m_rearLeft = new MAXSwerveModule(
      PortConstants.kRearLeftDrivingCanId,
      PortConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftChassisAngularOffset);
      
    m_rearRight = new MAXSwerveModule(
      PortConstants.kRearRightDrivingCanId,
      PortConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightChassisAngularOffset);
    
    // Initialize publisher
    publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    
    // Initialize gyro
    m_gyro = new Gyro();
    
    // Initialize KalmanLocalization
    kalmanLocalization = new KalmanLocalization(new Pose2d(
      new Translation2d(0, 0),
      new Rotation2d(0)
    ));
    
    this.isTestMode = false;
    fieldVisualization = new Field2d();
    m_cameras = cameras;
    
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Blue;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    isNoKalmanFilterBackUpMode = false;
  }

  public SwerveDrive(PhotonVisionCameras cameras, Gyro gyro, MAXSwerveModule frontLeft, MAXSwerveModule frontRight, MAXSwerveModule rearLeft, MAXSwerveModule rearRight, RobotConfig config, boolean isTestMode){
    m_gyro = gyro;
    m_frontLeft = frontLeft;
    m_frontRight = frontRight;
    m_rearLeft = rearLeft;
    m_rearRight = rearRight;
    this.config = config;
    this.isTestMode = isTestMode;
    fieldVisualization = new Field2d();
    m_cameras = cameras;

    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            (Pose2d p) -> {}, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(10, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(10, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Blue;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    // m_odometry.update(
    //     m_gyro.getProcessedRot2dYaw(),
    //     new SwerveModulePosition[] {
    //         m_frontLeft.getPosition(),
    //         m_frontRight.getPosition(),
    //         m_rearLeft.getPosition(),
    //         m_rearRight.getPosition()
    //     });
    ArrayList<Translation2d> wheel_pos = new ArrayList<Translation2d>();
    wheel_pos.add(DriveConstants.frontLeftPos);
    wheel_pos.add(DriveConstants.frontRightPos);
    wheel_pos.add(DriveConstants.rearLeftPos);
    wheel_pos.add(DriveConstants.rearRightPos);

    ArrayList<Translation2d> wheel_vel = new ArrayList<Translation2d>();
    wheel_vel.add(m_frontLeft.getVelocityVector());
    wheel_vel.add(m_frontRight.getVelocityVector());
    wheel_vel.add(m_rearLeft.getVelocityVector());
    wheel_vel.add(m_rearRight.getVelocityVector());

    double gyro_rate = m_gyro.getZRate()*Math.PI/180;
    Pose2d reef_camera_pose = null;
    double reef_camera_area = 0;
    double reef_camera_distance_to_tag = 0;
    boolean reef_has_target = false;

    if (m_cameras == null && !RobotBase.isSimulation()){
      System.out.println("Null!");
    }
    else if (m_cameras == null){
      reef_has_target = false;
    }
    else{
      reef_has_target = m_cameras.reefCameraHasTarget();
    }

    try{
      reef_camera_pose = m_cameras.getPoseRelativeReefCamera();
    } catch (Exception e){
      reef_camera_pose = kalmanLocalization.getPoseMeters();
    }

    
    try{
      reef_camera_area = m_cameras.getReefCameraArea();
    } catch (Exception e){
      reef_camera_area = -1;

    }
    try{
      SmartDashboard.putNumber("Camera area", reef_camera_area);
      SmartDashboard.putBoolean("Has target", m_cameras.reefCameraHasTarget());
    }
    catch (Exception e){
      SmartDashboard.putNumber("Camera area", -1);
      SmartDashboard.putBoolean("Has target", false);
    }


    Pose2d human_camera_pose = null;
    double human_camera_area = 0;
    boolean human_has_target = false;
    if (m_cameras == null){
      human_has_target = false;
    }
    else{
      human_has_target = m_cameras.humanPlayerCameraHasTarget();
    }

    try{
      human_camera_pose = m_cameras.getPoseRelativeHumanPlayerCamera();
    } catch (Exception e){
      human_camera_pose = kalmanLocalization.getPoseMeters();
    }

    try{
      human_camera_area = m_cameras.getHumanPlayerCameraArea();
    } catch (Exception e){
      human_camera_area = -1;
    }

    try{
      SmartDashboard.putNumber("Human Camera area", human_camera_area);
      SmartDashboard.putBoolean("Human Has target", m_cameras.humanPlayerCameraHasTarget());
    }
    catch (Exception e){
      SmartDashboard.putNumber("Human Camera area", -1);
      SmartDashboard.putBoolean("Human Has target", false);
    }

    

    kalmanLocalization.update(
      wheel_vel,
      wheel_pos,
      reef_camera_area >= 0.27 ? reef_camera_pose : null,
      human_camera_area >= 0.27 ? human_camera_pose : null,
      gyro_rate,
      reef_camera_area,
      human_camera_area,
      reef_has_target,
      human_has_target
    );

    SmartDashboard.putNumber("Gyro rate", gyro_rate);
    SmartDashboard.putNumber("pose_x", kalmanLocalization.getX());
    SmartDashboard.putNumber("pose_y", kalmanLocalization.getY());
    SmartDashboard.putNumber("pose_th", kalmanLocalization.getTheta());
    SmartDashboard.putNumber("pose_vx", kalmanLocalization.getXVel());
    SmartDashboard.putNumber("pose_vy", kalmanLocalization.getYVel());
    SmartDashboard.putNumber("pose_vth", kalmanLocalization.getThetaVel());
    SmartDashboard.putNumber("bias", kalmanLocalization.getGyroBias());

    fieldVisualization.setRobotPose(kalmanLocalization.getPoseMeters());

    publisher.set(new SwerveModuleState[]{
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    });

    SmartDashboard.putData(fieldVisualization);
    SmartDashboard.putNumber("X Pose",getPose().getX());
    SmartDashboard.putNumber("Y Vaue", getPose().getY());
    SmartDashboard.putNumber("Front Left Module Encoder Val", m_frontLeft.m_drivingEncoder.getPosition());
    SmartDashboard.putBoolean("Fast Align Mode", alignFastMode);
    try{
      SmartDashboard.putNumber("Fiducial ID Detected", m_cameras.getBestReefCameraFiducialId());
    } catch (Exception e){
      SmartDashboard.putNumber("Fiducial ID Detected", -1);
    }
    

    SmartDashboard.putNumber("Front Left Module", this.m_frontLeft.getVelocityVector().getNorm());
    SmartDashboard.putNumber("Front Right Module", this.m_frontRight.getVelocityVector().getNorm());
    SmartDashboard.putNumber("Rear Left Module", this.m_rearLeft.getVelocityVector().getNorm());
    SmartDashboard.putNumber("Rear Right Module", this.m_rearRight.getVelocityVector().getNorm());
    SmartDashboard.putNumber("Gyro Value", this.m_gyro.getProcessedRot2dYaw().getDegrees());

    SmartDashboard.putNumber("Swerve Front Left Module", m_frontLeft.m_drivingSpark.getOutputCurrent());
    SmartDashboard.putBoolean("BACKUP MODE", isNoKalmanFilterBackUpMode);
    
  }


  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return kalmanLocalization.getPoseMeters();
  }

  public KalmanLocalization getOdometry() {
    return kalmanLocalization;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    drive(xSpeed, ySpeed, rot, fieldRelative, false, false);
  }

  public void driveOnlyGyro(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    drive(xSpeed, ySpeed, rot, fieldRelative, false, true);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean isTestMode, boolean isOnlyGyro) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, (isOnlyGyro ? m_gyro.getProcessedRot2dYaw() : new Rotation2d(kalmanLocalization.getTheta())))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }
  
  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    //return m_gyro.getProcessedRot2dYaw().getDegrees();
    return kalmanLocalization.getTheta();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    //return m_gyro.getZRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    return kalmanLocalization.getThetaVel();
  }
  
  public void resetGyro(){
    m_gyro.reset();
  }

  public void toggleSlowMode(){
    slowMode = !slowMode;
  }

  public double getProcessedHeading(){
    return m_gyro.getProcessedRot2dYaw().getDegrees();
  }

  public boolean getSlowMode(){
    return slowMode;
  }

  public Rotation2d getHeadingObject(){
    return m_gyro.getProcessedRot2dYaw();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
      }
    );
  }

  public void resetPose(){
  }

  public void resetPose(Pose2d pose){
  }

  public void setAlignFastMode(boolean mode){
    alignFastMode = mode;
  }

  public boolean getAlignFastMode(){
    return alignFastMode;
  }

  public Gyro getGyro(){
    return m_gyro;
  }

  public void setNoKalmanFilterBackUpMode(boolean mode){
    if (isNoKalmanFilterBackUpMode == false && mode == true){
      m_gyro.setGyroYawOffset(m_gyro.getProcessedRot2dYaw().getDegrees());
    }
    isNoKalmanFilterBackUpMode = mode;
  }

  public boolean getNoKalmanFilterBackUpMode(){
    return isNoKalmanFilterBackUpMode;
  }

  public void testButton(){
    System.out.println("Hello World");
  }

  
}