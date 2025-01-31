package frc.robot;
import java.sql.Driver;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.ParseException;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.Constants.AxeConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.commands.shooter.shooterPIDCommand;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.preRoller;

import frc.robot.subsystems.elevator.elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.commands.swerve.*;




/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  protected final SwerveDrive m_robotDrive = new SwerveDrive();
  protected final elevator m_elevator = new elevator(PortConstants.kElevatorMotor1Port);
  protected final Intake m_intake = new Intake();
  
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
    // Configure the button bindings
    configureButtonBindings();
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
     

    DriverStation.silenceJoystickConnectionWarning(true);
    
    // Configure default commands 
    // m_robotDrive.setDefaultCommand(new driveCommand(m_robotDrive, m_driverController));
    
    m_robotDrive.setDefaultCommand(new driveCommand(m_robotDrive, m_driverController));

    autoChooser = AutoBuilder.buildAutoChooser();
    


    //if in auto set the default command of the shooter subsystem to be the shooterPIDCommand



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
    driverLeftTrigger.onTrue(new InstantCommand(() -> m_robotDrive.toggleSlowMode()));
    driverLeftTrigger.onFalse(new InstantCommand(() -> m_robotDrive.toggleSlowMode()));

    

    DriverRightBumper.onTrue(new InstantCommand(()-> m_intake.setIntakePower(0.5)));
    DriverRightBumper.onFalse(new InstantCommand(()-> m_intake.setIntakePower(0)));

    DriverDPadDown.onTrue(new InstantCommand(()-> m_elevator.setElevatorSpeed(0.5)));
    DriverDPadDown.onFalse(new InstantCommand(()-> m_elevator.setElevatorSpeed(0)));

    DriverDPadUp.onTrue(new InstantCommand(()-> m_elevator.setElevatorSpeed(-0.5)));
    DriverDPadUp.onFalse(new InstantCommand(()-> m_elevator.setElevatorSpeed(0)));

    DriverLeftBumper.whileTrue(new InstantCommand(()-> m_intake.setIntakePower(-0.5)));
    DriverLeftBumper.onFalse(new InstantCommand(()-> m_intake.setIntakePower(0)));

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
  }

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
      this.m_robotDrive.m_gyro.setGyroYawOffset(m_robotDrive.m_gyro.getGyroHeadingFromPathPlannerAuto(autoChooser.getSelected().getName()));

      return autoChooser.getSelected();
    }
}
