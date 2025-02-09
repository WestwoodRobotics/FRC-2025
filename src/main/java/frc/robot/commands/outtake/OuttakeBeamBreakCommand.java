package frc.robot.commands.outtake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.Outtake;

public class OuttakeBeamBreakCommand extends Command {
  
  private enum CoralState {
    WAITING_FOR_FRONT_CORAL,            // Haven't seen the coral yet; spin forward
    WAITING_FOR_NO_CORAL_AFTER_FRONT, // Coral’s front has passed beam, but not the rest
    WAITING_FOR_BACK_CORAL,             // Coral’s back not yet detected; spin reverse
    DONE                          // We’ve seen the back of the coral; stop
  }

  private final Outtake outtake;

  private double power;
  private CoralState state;

  public OuttakeBeamBreakCommand(Outtake outtake, double power) {
    this.outtake = outtake;
    this.power = power;
    addRequirements(outtake);
  }

  @Override
  public void initialize() {
    state = CoralState.WAITING_FOR_FRONT_CORAL;

  }

  @Override
  public void execute() {
    switch (state) {
      case WAITING_FOR_FRONT_CORAL:
        // Spin forward until the beam breaks (we detect coral)
        outtake.setOuttakeSpeed(power);
        if (outtake.isCoralDetected()) {
          // The front of the coral just passed
          state = CoralState.WAITING_FOR_NO_CORAL_AFTER_FRONT;
        }
        break;

      case WAITING_FOR_NO_CORAL_AFTER_FRONT:
        // Still spin forward until coral has fully passed -> beam unbroken
        outtake.setOuttakeSpeed(power);
        if (outtake.isCoralNotDetected()) {
          // Entire coral just passed the sensor; reverse direction
          this.power = -this.power;
          state = CoralState.WAITING_FOR_BACK_CORAL;
        }
        break;

      case WAITING_FOR_BACK_CORAL:
        // Spin in reverse until the beam breaks again (back of coral)
        outtake.setOuttakeSpeed(power);
        if (outtake.isCoralDetected()) {
          // We just detected the back of the coral
          outtake.setOuttakeSpeed(0);
          state = CoralState.DONE;
        }
        break;

      case DONE:
        // No more action needed
        outtake.setOuttakeSpeed(0);
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    outtake.setOuttakeSpeed(0);
  }
}