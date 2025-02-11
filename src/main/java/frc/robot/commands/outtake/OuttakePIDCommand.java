package frc.robot.commands.outtake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.Outtake;

public class OuttakePIDCommand extends Command{

    private final Outtake outtake;
  private final PIDController outtakePIDController;
  private final double targetRpm;
  

  public OuttakePIDCommand(Outtake outtake, double targetRpm) {
    this.outtake = outtake;
    this.outtakePIDController = outtake.getOuttakePIDController();
    this.targetRpm = targetRpm;
    addRequirements(outtake);
  }

  @Override
  public void initialize() {
    outtakePIDController.setSetpoint(targetRpm);
  }

  @Override
  public void execute() {
    outtake.setOuttakeSpeed(outtakePIDController.calculate(targetRpm));
  }

  @Override
  public void end(boolean interrupted) {
    outtake.setOuttakeSpeed(0);
  }
    
}
