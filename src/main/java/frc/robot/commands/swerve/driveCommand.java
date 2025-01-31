package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;

/**
 * The driveCommand class is responsible for controlling the swerve drive subsystem using an Xbox controller.
 * It handles joystick inputs to drive the robot in various directions and speeds, including a slow mode.
 */
public class driveCommand extends Command {

  private final SwerveDrive m_swerveDrive;
  private XboxController controller;
  private boolean slowMode;
  private boolean isYuMode;
  private PIDController rotationPIDController;
  private double targetHeading;
  private boolean isRotInput;
  private Timer timer;

  /**
   * Constructs a new driveCommand.
   * 
   * @param swerveDrive The swerve drive subsystem used by this command.
   * @param controller The Xbox controller used to control the swerve drive.
   */
  public driveCommand(SwerveDrive swerveDrive, XboxController controller) {
    m_swerveDrive = swerveDrive;
    this.controller = controller;
    timer = new Timer();
    
    addRequirements(swerveDrive);

  }

  /**
   * Initializes the command by setting the slow mode to false.
   */
  @Override
  public void initialize() {
    slowMode = false;
    isRotInput = true;
    rotationPIDController = new PIDController(Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD);
    rotationPIDController.setTolerance(5);
    targetHeading = m_swerveDrive.getProcessedHeading();
    rotationPIDController.setSetpoint(targetHeading);
    isYuMode = false;
  }

  /**
   * Executes the command by reading joystick inputs and driving the swerve drive subsystem.
   * It handles slow mode and Yu mode toggling based on controller inputs.
   */
  @Override
  public void execute() {
    //System.out.println("Worked Here");
    slowMode = m_swerveDrive.getSlowMode();
    double leftX, leftY, rightX;
    //isYuMode = m_swerveDrive.isYuMode();
    if (controller.getBackButtonPressed()) {
      slowMode = !slowMode;
    }
    if (isYuMode == false) {
      leftX = -MathUtil.applyDeadband(controller.getLeftX(), ControllerConstants.kDriveDeadband);
      leftY = -MathUtil.applyDeadband(controller.getLeftY(), ControllerConstants.kDriveDeadband);
      rightX = -MathUtil.applyDeadband(controller.getRightX(), ControllerConstants.kDriveDeadband);
    } else {
      leftX = -MathUtil.applyDeadband(controller.getRightX(), ControllerConstants.kDriveDeadband);
      leftY = -MathUtil.applyDeadband(controller.getRightY(), ControllerConstants.kDriveDeadband);
      rightX = -MathUtil.applyDeadband(controller.getLeftX(), ControllerConstants.kDriveDeadband);
    }

    if (slowMode) {
      leftX *= Constants.DriveConstants.slowModeMultiplier;
      leftY *= Constants.DriveConstants.slowModeMultiplier;
      rightX *= Constants.DriveConstants.slowModeMultiplier;
    }

    if (Math.abs(rightX) > 0) {
      isRotInput = false;
      timer.reset();
      timer.stop();
    } else {
      if (!isRotInput) {
        timer.start();
        if (timer.hasElapsed(0.3)) {
          targetHeading = m_swerveDrive.getProcessedHeading();
          rotationPIDController.setSetpoint(targetHeading);
          isRotInput = true;
          timer.reset();
          timer.stop();
        }
      }
      if (isRotInput) {
        rightX = rotationPIDController.calculate(m_swerveDrive.getProcessedHeading());
      }
    }

    m_swerveDrive.drive(leftY, leftX, rightX, true                                                                                    );
    //controller.setRumble(RumbleType.kBothRumble, leftY > 0 ? 1 : 0);
  }

  /**
   * Ends the command. This method is called once when the command ends or is interrupted.
   * 
   * @param interrupted Whether the command was interrupted.
   */
  @Override
  public void end(boolean interrupted) {
    //controller.setRumble(RumbleType.kBothRumble, 0);
  }

  /**
   * Toggles the Yu mode.
   */
  public void toggleYuMode() {
    isYuMode = !isYuMode;
  }

  /**
   * Returns whether the command has finished.
   * 
   * @return Always returns false, as this command never finishes on its own.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
