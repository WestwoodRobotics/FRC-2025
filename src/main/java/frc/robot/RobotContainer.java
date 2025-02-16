


package frc.robot;
import java.sql.Driver;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.ParseException;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.PortConstants;
import frc.robot.commands.ODCommandFactory;
import frc.robot.commands.elevator.elevatorHoldCommand;
import frc.robot.commands.elevator.elevatorSetPosition;
import frc.robot.commands.elevator.elevatorSetPositionWithCurrentLimit;
import frc.robot.commands.elevator.elevatorSetPositionWithLimitSwitch;
import frc.robot.commands.outtake.OuttakeBeamBreakCommand;
import frc.robot.commands.outtake.OuttakeBeamBreakTimeCommand;
import frc.robot.commands.outtake.OuttakeCurrentTimeCommand;
import frc.robot.commands.outtake.OuttakePIDCommand;
import frc.robot.commands.outtake.OuttakePIDCurrentTimeCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.utils.elevator.elevatorPositions;
import frc.robot.subsystems.utils.swerve.ReefAlignSide;
import frc.robot.sensors.PhotonVisionCameras;
import frc.robot.commands.swerve.*;
import frc.robot.commands.swerve.GoToNearestScoringPoseCommand;




/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  protected final SwerveDrive m_robotDrive;
  protected final Elevator m_elevator = new Elevator(PortConstants.kElevatorMotor1Port, PortConstants.kElevatorMotor2Port);
  protected final Intake m_intake = new Intake();
  protected final Outtake m_outtake = new Outtake();
  protected PhotonVisionCameras m_cameras;
  protected AprilTagFieldLayout m_layout;
  
  //private final Intake m_intake = new Intake();
  //private final preRoller m_preRoller = new preRoller();
  //protected final Shooter m_shooter = new Shooter(false);
  //protected final Axe m_axe = new Axe();
  private final SendableChooser<Command> autoChooser;


  // LED for indicating robot state, not implemented in hardware.

  // The driver's controller
  XboxController m_driverController = new XboxController(PortConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(PortConstants.kOperatorControllerPort);

  private final JoystickButton DriverAButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  private final JoystickButton DriverBButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  private final JoystickButton DriverXButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  private final JoystickButton DriverYButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  private final JoystickButton DriverGyroButton = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
  private final JoystickButton OperatorStartButton = new JoystickButton(m_operatorController, XboxController.Button.kStart.value);


  private final POVButton DriverDPadUp = new POVButton(m_driverController, 0);
  private final POVButton DriverDPadRight = new POVButton(m_driverController, 90);
  private final POVButton DriverDPadDown = new POVButton(m_driverController, 180);
  private final POVButton DriverDPadLeft = new POVButton(m_driverController, 270);

  private final JoystickButton DriverRightBumper = new JoystickButton(m_driverController,
      XboxController.Button.kRightBumper.value);
  private final JoystickButton DriverLeftBumper = new JoystickButton(m_driverController,
      XboxController.Button.kLeftBumper.value);
      
  private final Trigger driverLeftTrigger = new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5);
  private final Trigger driverRightTrigger = new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.5);

  private final JoystickButton OperatorAButton = new JoystickButton(m_operatorController,
      XboxController.Button.kA.value);
  private final JoystickButton OperatorBButton = new JoystickButton(m_operatorController,
      XboxController.Button.kB.value);
  private final JoystickButton OperatorXButton = new JoystickButton(m_operatorController,
      XboxController.Button.kX.value);
  private final JoystickButton OperatorYButton = new JoystickButton(m_operatorController,
      XboxController.Button.kY.value);

  private final POVButton OperatorDPadUp = new POVButton(m_operatorController, 0);
  private final POVButton OperatorDPadRight = new POVButton(m_operatorController, 90);
  private final POVButton OperatorDPadDown = new POVButton(m_operatorController, 180);
  private final POVButton OperatorDPadLeft = new POVButton(m_operatorController, 270);

  private Trigger operatorRightYTrigger = new Trigger(() -> Math.abs(m_operatorController.getRightY()) > 0.10);

  private final Trigger operatorLeftTrigger = new Trigger(() -> m_operatorController.getLeftTriggerAxis() > 0.5);
  private final Trigger operatorRightTrigger = new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.5);

  private final JoystickButton OperatorRightBumper = new JoystickButton(m_operatorController,
      XboxController.Button.kRightBumper.value);
  private final JoystickButton OperatorLeftBumper = new JoystickButton(m_operatorController,
      XboxController.Button.kLeftBumper.value);

  private SendableChooser<Command> m_chooser = new SendableChooser<>();
  //protected ODCommandFactory ODCommandFactory = new ODCommandFactory(m_intake, m_preRoller, m_shooter);

    
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    ODCommandFactory ODCommandFactory = new ODCommandFactory(m_intake, m_outtake, m_elevator);
    // Configure the button bindings
    // NamedCommands.registerCommand("SpinSensePreRoller", ODCommandFactory.intakeSenseCommand());
    // NamedCommands.registerCommand("Intake", ODCommandFactory.intakeSenseCommand());
    // NamedCommands.registerCommand("StopIntake", ODCommandFactory.stopIntakeSenseCommand());
    // NamedCommands.registerCommand("RevUpAndShoot", ODCommandFactory.revUpAndShootCommandAuton(0.90, 4000, 3000));
    // NamedCommands.registerCommand("StopShooter", ODCommandFactory.stopShooterCommand());
    // NamedCommands.registerCommand("releasePreRollerCommand", ODCommandFactory.fireNote());
    // NamedCommands.registerCommand("stopAllCommand", ODCommandFactory.stopAllCommand());
    // NamedCommands.registerCommand("resetPosition", new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(new Translation2d(1.31, m_robotDrive.getPose().getY()), new Rotation2d(Math.toRadians(0))))));
    // NamedCommands.registerCommand("checkAutoAndShoot", ODCommandFactory.checkAutoAndShoot());
    // NamedCommands.registerCommand("resetGyro", new InstantCommand(() -> m_robotDrive.resetGyro()));

    // //Auto Commands
    // NamedCommands.registerCommand("LLSeekAndRotateOnly", new SeekAndTrackRotOnly(m_robotDrive, "limelight"));
    // NamedCommands.registerCommand("LLAlignAndRange", new AlignAndRangeAprilTag(m_robotDrive, "limelight"));
    // NamedCommands.registerCommand("LLAlignHorizontally", new AprilTagFollowGeneral(m_robotDrive, "limelight"));


     
    



    try {
        m_layout = new AprilTagFieldLayout(
            "/home/lvuser/deploy/2025-reefscape.json"
        );  
        m_cameras = new PhotonVisionCameras(m_layout);
        
    }
    catch(IOException exc) {
        System.out.println("Failed to load field layout!");
        m_cameras = null;
    }
    m_robotDrive = new SwerveDrive(m_cameras);

    
    DriverStation.silenceJoystickConnectionWarning(true);



    NamedCommands.registerCommand("GoToScorePoseLeft", new GoToNearestScoringPoseCommand(m_robotDrive, m_layout, ReefAlignSide.LEFT));
    NamedCommands.registerCommand("GoToScorePoseRight", new GoToNearestScoringPoseCommand(m_robotDrive, m_layout, ReefAlignSide.RIGHT));
    NamedCommands.registerCommand("GoToElevatorL4", new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.L4));
    NamedCommands.registerCommand("GoToElevatorL3", new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.L3));
    NamedCommands.registerCommand("GoToElevatorL2", new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.L2));
    NamedCommands.registerCommand("GoToElevatorHome", new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.HOME));
    NamedCommands.registerCommand("Intake", ODCommandFactory.IntakeToOuttakeBeamBreakCommand());
    NamedCommands.registerCommand("ScoreCoral", ODCommandFactory.scoreCoral());

    
    // Configure default commands 
    // m_robotDrive.setDefaultCommand(new driveCommand(m_robotDrive, m_driverController));
    
    m_robotDrive.setDefaultCommand(new driveCommand(m_robotDrive, m_driverController));
    //m_elevator.setDefaultCommand(new elevatorSetPosition(m_elevator, m_elevator.getElevatorPosition()));
    





    autoChooser = AutoBuilder.buildAutoChooser();

    //if in auto set the default command of the shooter subsystem to be the shooterPIDCommand

    

    configureButtonBindings();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }

  /*
   * Use this method to define your button->command mappings. Buttons can be
   * created by 
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  


    private void configureButtonBindings() {
        /*
        * DRIVER BUTTON MAPPINGS
        */

        DriverGyroButton.whileTrue(new InstantCommand(
            () -> m_robotDrive.resetGyro(),
            m_robotDrive));

        //driverLeftTrigger.onTrue(ODCommandFactory.intakeSenseCommand());
        //driverLeftTrigger.onFalse(ODCommandFactory.stopIntakeSenseCommand());

        // driverRightTrigger.onTrue(ODCommandFactory.revUpShooter());
        // driverRightTrigger.onFalse(ODCommandFactory.stopShooterCommand());

        // DriverRightBumper.onTrue(ODCommandFactory.revUpAndShootCommand(0.75, 4000));
        // DriverRightBumper.onFalse(ODCommandFactory.stopShooterCommand());


        // DriverDPadDown.onTrue(new InstantCommand(() -> m_shooter.setShooterPower(-0.85), m_shooter));

        // // DriverBButton.onTrue(new InstantCommand(() -> m_preRoller.setPreRollerPower(1), m_preRoller));
        // // DriverBButton.onFalse(new InstantCommand(() -> m_preRoller.setPreRollerPower(0), m_preRoller));
        // DriverBButton.onTrue(ODCommandFactory.intakeSenseCommand());
        // DriverBButton.onFalse(ODCommandFactory.stopPreRollerCommand().alongWith(ODCommandFactory.stopIntakeCommand()));

        



        DriverDPadUp
        .onTrue(new InstantCommand(()-> m_elevator.setElevatorSpeed(-0.25), m_elevator))
        .onFalse(new elevatorHoldCommand(m_elevator));

        DriverDPadDown
        .onTrue(new InstantCommand(()-> m_elevator.setElevatorSpeed(0.25), m_elevator))
        .onFalse(new elevatorHoldCommand(m_elevator));

        DriverLeftBumper.whileTrue(new GoToNearestScoringPoseCommand(m_robotDrive, m_layout, ReefAlignSide.LEFT));

        DriverRightBumper.whileTrue(new GoToNearestScoringPoseCommand(m_robotDrive, m_layout, ReefAlignSide.RIGHT));

        




        driverLeftTrigger
        .onTrue(new InstantCommand(()-> m_intake.setIntakePower(0.4), m_intake)//right trigger
        .andThen(new OuttakeBeamBreakCommand(m_outtake, -0.3)))
        .onFalse(new InstantCommand(()-> m_intake.stopIntake(), m_intake)
        .andThen(new InstantCommand(() -> m_outtake.setOuttakeSpeed(0), m_outtake)));
       
        
        driverRightTrigger
        .onTrue(new InstantCommand(()-> m_intake.setIntakePower(0.4), m_intake) //right bumper
        .andThen(new InstantCommand(()-> m_outtake.setOuttakeSpeed(-0.3))))
        .onFalse(new InstantCommand(()-> m_intake.stopIntake(), m_intake)
        .andThen(new InstantCommand(() -> m_outtake.setOuttakeSpeed(0), m_outtake)));

        OperatorLeftBumper
        .onTrue(new InstantCommand(()-> m_intake.setIntakePower(-0.4), m_intake) //left trigger
        .andThen(new InstantCommand(()-> m_outtake.setOuttakeSpeed( 0.3))))
        .onFalse(new InstantCommand(()-> m_intake.stopIntake(), m_intake)
        .andThen(new InstantCommand(() -> m_outtake.setOuttakeSpeed(0), m_outtake)));

        
        operatorLeftTrigger
        .onTrue(new InstantCommand(()-> m_intake.setIntakePower(-0.4), m_intake) //left bumper
        .andThen(new InstantCommand(()-> m_outtake.setOuttakeSpeed(0.3))))
        .onFalse(new InstantCommand(()-> m_intake.stopIntake(), m_intake)
        .andThen(new InstantCommand(() -> m_outtake.setOuttakeSpeed(0), m_outtake)));

        // DriverAButton.onTrue(new elevatorSetPositionWithCurrentLimit(m_elevator, elevatorPositions.HOME, 50, 30, 4));
        // DriverBButton.onTrue(new elevatorSetPositionWithCurrentLimit(m_elevator, elevatorPositions.L1, 50, 30, 4));
        // DriverXButton.onTrue(new elevatorSetPositionWithCurrentLimit(m_elevator, elevatorPositions.L2, 50, 30, 4));
        // DriverYButton.onTrue(new elevatorSetPositionWithCurrentLimit(m_elevator, elevatorPositions.L3, 50, 30, 4));

        DriverAButton.onTrue(new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.HOME));
        DriverBButton.onTrue(new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.L4));
        DriverYButton.onTrue(new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.L3));
        DriverXButton.onTrue(new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.L2));

        //DriverRightBumper.onTrue(new InstantCommand(() -> m_outtake.setOuttakeSpeed(-0.3)));


        /*
         * OPERATOR BUTTON MAPPINGS
         */
        
        // OperatorDPadLeft.whileTrue(new GoToFieldPose(m_robotDrive, 11.71, 4.02+0.165, 0));
        // OperatorDPadRight.whileTrue(new GoToFieldPose(m_robotDrive, 11.71, 4.02-0.165, 0));

        OperatorAButton
        .onTrue(new InstantCommand(()-> m_elevator.setElevatorEncoderOffset(m_elevator.getElevatorPosition()), m_elevator))
        .onFalse(new elevatorHoldCommand(m_elevator));
        
        

        //DriverBButton.onTrue(new InstantCommand(() -> m_elevator.stopElevator()));

        
    }

    // AutoBuilder.resetOdom();
    // AutoBuilder.pathfindToPose(null, null);

    // DriverAButton.onTrue(new InstantCommand(() -> m_intake.setIntakePower(0.5), m_intake));
    // DriverAButton.onFalse(new InstantCommand(() -> m_intake.setIntakePower(0), m_intake));
    // DriverYButton.onTrue(new InstantCommand(() -> m_intake.setIntakePower(-0.5
    // ), m_intake));
    // DriverYButton.onFalse(new InstantCommand(() -> m_intake.setIntakePower(0), m_intake));

    //DriverLeftBumper.onTrue(new InstantCommand(() -> m_shooter.setShooterPower(-0.15), m_shooter).alongWith(new InstantCommand(() -> m_preRoller.setPreRollerPower(-1), m_preRoller)));
    //DriverLeftBumper.onFalse(new InstantCommand(() -> m_shooter.setShooterPower(0), m_shooter).alongWith(new InstantCommand(() -> m_preRoller.setPreRollerPower(0), m_preRoller)));
    //DriverDPadLeft.onTrue(new InstantCommand(()-> m_robotDrive.toggleYuMode()));
    // DriverDPadLeft.onTrue(new InstantCommand(() -> m_axe.resetEncoder()));

    // //axe
    // DriverRightBumper.onTrue(new AxePIDCommand(m_axe, AxeConstants.kAxeUpPosition));
    // DriverLeftBumper.onTrue(new AxePIDCommand(m_axe, AxeConstants.kAxeDownPosition));

    /*

     * OPERATOR BUTTON MAPPING
     */


//------------------------------------------- autonom555555ous modes -------------------------------------------
    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

    public Command getAutonomousCommand() {
      //return autoChooser.getSelected();
      //return NamedCommands.getCommand("LLAlignHorizontally");
      //this.m_robotDrive.gyroSubsystem.setGyroYawOffset(m_robotDrive.gyroSubsystem.getGyroHeadingFromPathPlannerAuto(autoChooser.getSelected().getName())+90);
    //   while (0 == 0){
    //     System.out.println("Yaw Offset: " + m_robotDrive.gyroSubsystem.getProcessedRot2dYaw().getDegrees());
    //   }
      //this.m_robotDrive.m_gyro.setGyroYawOffset(m_robotDrive.m_gyro.getGyroHeadingFromPathPlannerAuto(autoChooser.getSelected().getName()));

        return autoChooser.getSelected();
        //return new GoToNearestScoringPoseCommand(m_robotDrive, ReefAlignSide.LEFT);
    }
}
