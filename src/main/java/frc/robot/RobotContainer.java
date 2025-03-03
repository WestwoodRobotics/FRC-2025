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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.PortConstants;
import frc.robot.commands.ConditionalTuskBasedIntakeOuttakeCommand;
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
import frc.robot.commands.outtake.OuttakeUntilBeamRestored;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.tusks.Tusks;
import frc.robot.subsystems.utils.elevator.elevatorPositions;
import frc.robot.subsystems.utils.swerve.ReefAlignSide;
import frc.robot.subsystems.utils.tusks.tuskPositions;
import frc.robot.sensors.PhotonVisionCameras;
import frc.robot.sensors.DIO.LEDController;
import frc.robot.commands.swerve.*;
import frc.robot.commands.tusks.tuskSetPositionCommand;
import frc.robot.commands.tusks.tuskHoldPositionCommand;




/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    protected SwerveDrive m_robotDrive;
    protected Elevator m_elevator;
    protected Intake m_intake;
    protected Outtake m_outtake;
    protected Tusks m_tusks;
    protected PhotonVisionCameras m_cameras;
    protected AprilTagFieldLayout m_layout;
    protected LEDController ledController;
    

    private  SendableChooser<Command> autoChooser;


    // LED for indicating robot state, not implemented in hardware.

    // The driver's controller
    XboxController m_driverController;
    XboxController m_operatorController;

    private  JoystickButton DriverAButton;
    private  JoystickButton DriverBButton;
    private  JoystickButton DriverXButton;
    private  JoystickButton DriverYButton;
    private JoystickButton  DriverStartButton;


    private  POVButton DriverDPadUp;
    private  POVButton DriverDPadDown;

    private  JoystickButton DriverRightBumper;
    private  JoystickButton DriverLeftBumper;
    private  Trigger driverLeftTrigger;
    private  Trigger driverRightTrigger;

    private  JoystickButton OperatorAButton;
    private  JoystickButton OperatorBButton; 
    private  JoystickButton OperatorXButton;
    private  JoystickButton OperatorYButton;

    private  POVButton OperatorDPadUp;
    private  POVButton OperatorDPadRight;
    private  POVButton OperatorDPadDown;
    private  POVButton OperatorDPadLeft;

    private  Trigger operatorLeftTrigger;
    private  Trigger operatorRightTrigger;

    private  JoystickButton OperatorRightBumper;
    private JoystickButton OperatorLeftBumper;

    ODCommandFactory ODCommandFactory;


    
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
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

    // Initialize controllers first
    m_driverController = new XboxController(PortConstants.kDriverControllerPort);
    m_operatorController = new XboxController(PortConstants.kOperatorControllerPort);



    // Initialize subsystems
    m_robotDrive = new SwerveDrive(m_cameras);
    m_elevator =  new Elevator(PortConstants.kElevatorMotor1Port, PortConstants.kElevatorMotor2Port);
    m_intake = new Intake();
    m_tusks = new Tusks();
    m_outtake = new Outtake();
    ledController = new LEDController(m_cameras);
    ODCommandFactory = new ODCommandFactory(m_intake, m_outtake, m_elevator, m_tusks, ledController);

    // Configure default commands 
    m_robotDrive.setDefaultCommand(new driveCommand(m_robotDrive, m_driverController));

    // Register named commands
    NamedCommands.registerCommand("GoToScorePoseLeft", new GoToNearestScoringPoseCommand(m_robotDrive, m_layout, ReefAlignSide.LEFT, m_robotDrive.getAlignFastMode()));
    NamedCommands.registerCommand("GoToScorePoseRight", new GoToNearestScoringPoseCommand(m_robotDrive, m_layout, ReefAlignSide.RIGHT, m_robotDrive.getAlignFastMode()));
    NamedCommands.registerCommand("GoToElevatorL4",((new OuttakeBeamBreakCommand(m_outtake, ledController, -0.2, true).alongWith(new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.L4)))).raceWith(new WaitCommand(1.3)));
    NamedCommands.registerCommand("GoToElevatorL3", new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.L3));
    NamedCommands.registerCommand("GoToElevatorL2", new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.L2));
    
    NamedCommands.registerCommand("GoToElevatorHome", new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.HOME));
    NamedCommands.registerCommand("Intake", ODCommandFactory.IntakeToOuttakeBeamBreakCommand());
    NamedCommands.registerCommand("ScoreCoral", ODCommandFactory.scoreCoral());
    NamedCommands.registerCommand("StopIntakeAndOuttake", ODCommandFactory.stopIntake());

    // Build autonomous chooser
    autoChooser = AutoBuilder.buildAutoChooser();
  
    // Initialize controller buttons
    DriverAButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    DriverBButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
    DriverXButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    DriverYButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    
    DriverDPadUp = new POVButton(m_driverController, 0);
    DriverDPadDown = new POVButton(m_driverController, 180);
  
    DriverRightBumper = new JoystickButton(m_driverController,
        XboxController.Button.kRightBumper.value);
    DriverLeftBumper = new JoystickButton(m_driverController,
        XboxController.Button.kLeftBumper.value);
        
    driverLeftTrigger = new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5);
    driverRightTrigger = new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.5);

    DriverStartButton = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
  
    OperatorAButton = new JoystickButton(m_operatorController,
        XboxController.Button.kA.value);
    OperatorBButton = new JoystickButton(m_operatorController,
        XboxController.Button.kB.value);
    OperatorXButton = new JoystickButton(m_operatorController,
        XboxController.Button.kX.value);
    OperatorYButton = new JoystickButton(m_operatorController,
        XboxController.Button.kY.value);
    
    OperatorDPadUp = new POVButton(m_operatorController, 0);
    OperatorDPadRight = new POVButton(m_operatorController, 90);
    OperatorDPadDown = new POVButton(m_operatorController, 180);
    OperatorDPadLeft = new POVButton(m_operatorController, 270);
    
    operatorLeftTrigger = new Trigger(() -> m_operatorController.getLeftTriggerAxis() > 0.5);
    operatorRightTrigger = new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.5);
    
    OperatorRightBumper = new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value);
    OperatorLeftBumper = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value);

    configureButtonBindings();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    DriverStation.silenceJoystickConnectionWarning(true);

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

        

        DriverDPadUp
        .onTrue(new InstantCommand(()-> m_elevator.setElevatorSpeed(-0.25), m_elevator))
        .onFalse(new elevatorHoldCommand(m_elevator));

        DriverDPadDown
        .onTrue(new InstantCommand(()-> m_elevator.setElevatorSpeed(0.25), m_elevator))
        .onFalse(new elevatorHoldCommand(m_elevator));


        DriverLeftBumper.whileTrue(new GoToNearestScoringPoseCommand(m_robotDrive, m_layout, ReefAlignSide.LEFT, m_robotDrive.getAlignFastMode()));

        DriverRightBumper.whileTrue(new GoToNearestScoringPoseCommand(m_robotDrive, m_layout, ReefAlignSide.RIGHT, m_robotDrive.getAlignFastMode()));

        //intake
        driverLeftTrigger
        .onTrue(
            new InstantCommand(() -> m_intake.setBothPowers(0.25, 0.4), m_intake)
            .andThen(new OuttakeBeamBreakCommand(m_outtake, ledController, 1, -0.4)
            ))
        .onFalse(ODCommandFactory.stopIntake()); 

        //score
        driverRightTrigger
        .onTrue(new InstantCommand(()-> m_intake.setIntakePower(0.4), m_intake)
        .andThen(new InstantCommand(()-> m_outtake.setOuttakeSpeed(-0.45),m_outtake)).alongWith(new InstantCommand(() -> m_tusks.setRollerPower(-0.45), m_tusks)))
        .onFalse(new InstantCommand(()-> m_intake.stopIntake(), m_intake)
        .andThen(new InstantCommand(() -> m_outtake.setOuttakeSpeed(0), m_outtake)).alongWith(new InstantCommand(()-> m_tusks.setRollerPower(0), m_tusks)));
        

        OperatorLeftBumper
        .onTrue(new tuskSetPositionCommand(m_tusks, tuskPositions.OUT)
        .andThen(new tuskHoldPositionCommand(m_tusks))
        .andThen(new ParallelCommandGroup(
            new InstantCommand(() -> m_outtake.setOuttakeSpeed(0.3), m_outtake),
            new InstantCommand(() -> m_tusks.setRollerPower(0.3), m_tusks)
        )));
        
        operatorLeftTrigger
        .onTrue(new InstantCommand(()-> m_intake.setIntakePower(-1), m_intake) //right bumper
        .andThen(new InstantCommand(()-> m_outtake.setOuttakeSpeed(0.3))).alongWith(new InstantCommand(() -> m_tusks.setRollerPower(0.3))))
        .onFalse(new InstantCommand(()-> m_intake.stopIntake(), m_intake)
        .andThen(new InstantCommand(() -> m_outtake.setOuttakeSpeed(0), m_outtake)).alongWith(new InstantCommand(()-> m_tusks.setRollerPower(0), m_tusks)));

        operatorRightTrigger
        .onTrue(new InstantCommand(()-> m_intake.setIntakePower(0.4), m_intake) //right bumper
        .andThen(new InstantCommand(()-> m_outtake.setOuttakeSpeed(-0.3))).alongWith(new InstantCommand(() -> m_tusks.setRollerPower(-0.3))))
        .onFalse(new InstantCommand(()-> m_intake.stopIntake(), m_intake)
        .andThen(new InstantCommand(() -> m_outtake.setOuttakeSpeed(0), m_outtake)).alongWith(new InstantCommand(()-> m_tusks.setRollerPower(0), m_tusks)));

        OperatorRightBumper
        .onTrue(new tuskSetPositionCommand(m_tusks, tuskPositions.IN));

        
        
        


        DriverAButton.onTrue(new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.HOME));
        DriverBButton.onTrue(new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.L4).alongWith(new OuttakeBeamBreakCommand(m_outtake, ledController, -0.2, true)));
        DriverYButton.onTrue(new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.L3).alongWith(new OuttakeBeamBreakCommand(m_outtake, ledController, -0.2, true)));
        DriverXButton.onTrue(new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.L2).alongWith(new OuttakeBeamBreakCommand(m_outtake, ledController, -0.2, true)));

        DriverStartButton.onTrue(new InstantCommand(() -> m_robotDrive.setAlignFastMode(!m_robotDrive.getAlignFastMode()), m_robotDrive));


        /*
         * OPERATOR BUTTON MAPPINGS
         */

        
        // OperatorDPadLeft.whileTrue(new GoToFieldPose(m_robotDrive, 11.71, 4.02+0.165, 0));
        // OperatorDPadRight.whileTrue(new GoToFieldPose(m_robotDrive, 11.71, 4.02-0.165, 0));

        OperatorDPadUp.onTrue(new InstantCommand(() -> m_elevator.setElevatorSpeed(-0.25), m_elevator)).onFalse(new elevatorHoldCommand(m_elevator));
        OperatorDPadDown.onTrue(new InstantCommand(() -> m_elevator.setElevatorSpeed(0.25), m_elevator)).onFalse(new elevatorHoldCommand(m_elevator));
        OperatorDPadLeft.onTrue(new InstantCommand(() -> m_tusks.setPivotPower(0.2), m_tusks)).onFalse(new tuskHoldPositionCommand(m_tusks));
        OperatorDPadRight.onTrue(new InstantCommand(() -> m_tusks.setPivotPower(-0.2), m_tusks)).onFalse(new tuskHoldPositionCommand(m_tusks)); 

        OperatorXButton.onTrue(new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.HOME));
        //OperatorBButton.onTrue(new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.L4).alongWith(new OuttakeUntilBeamRestored(m_outtake, -0.2)));
        OperatorBButton.whileTrue(new GoToNearestScoringPoseCommand(m_robotDrive, m_layout, ReefAlignSide.CENTER, m_robotDrive.getAlignFastMode()));
        OperatorYButton.onTrue(new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.L3).alongWith(new OuttakeUntilBeamRestored(m_outtake, -0.2)));
        OperatorAButton.onTrue(new elevatorSetPositionWithLimitSwitch(m_elevator, elevatorPositions.L2).alongWith(new OuttakeUntilBeamRestored(m_outtake, -0.2)));


        
        

        //DriverBButton.onTrue(new InstantCommand(() -> m_elevator.stopElevator()));

        
    }

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
        return autoChooser.getSelected();
    }
}
