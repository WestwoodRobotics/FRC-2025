package Archives.commands.outtake;

import java.util.LinkedList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.Outtake;

public class OuttakeCurrentTimeCommand extends Command {

  private enum CoralStates{
    WAITING_FOR_CORAL,
    FOUND_CORAL,
    FINISHED
  }
  

  private final Outtake outtake;

  private double power;

  private LinkedList<Double> recentCurrents = new LinkedList<Double>();

  private Timer timer = new Timer();

  private CoralStates current_state;

  private double currentDetectionThreshold;

  private double timeOffset;

  private double first_twenty;

  private double running_avg;

  public OuttakeCurrentTimeCommand(Outtake outtake, double power, double currentDetectionThreshold, double timeOffset) {
    this.outtake = outtake;
    this.power = power;
    this.currentDetectionThreshold = currentDetectionThreshold;
    this.timeOffset = timeOffset;
    addRequirements(outtake);
  }

  @Override
  public void initialize() {
    first_twenty = 0;
    running_avg = 0;
    recentCurrents.clear();
    timer.reset();

    recentCurrents.add(outtake.getOuttakeCurrent());
    current_state = CoralStates.WAITING_FOR_CORAL;
  }

  // Language: Java
  @Override
  public void execute() {
      switch (current_state) {
          case WAITING_FOR_CORAL:
              outtake.setOuttakeSpeed(power);
              if (first_twenty < 30) {
                  // Warming up: increment counter and ignore current sample.
                  first_twenty++;
              } else {
                  // After warming up, add current readings.
                  recentCurrents.add(outtake.getOuttakeCurrent());
                  if (recentCurrents.size() > 5) {
                      recentCurrents.removeFirst();
                  }
                  if (recentCurrents.size() == 5) {
                      double sum = 0;
                      for (Double current : recentCurrents) {
                          sum += current;
                      }
                      running_avg = sum / recentCurrents.size();
                      if (running_avg > currentDetectionThreshold) {
                          current_state = CoralStates.FOUND_CORAL;
                          timer.reset();
                          timer.start();
                      }
                  }
              }
              break;
          case FOUND_CORAL:
              if (timer.hasElapsed(timeOffset)) {
                  outtake.setOuttakeSpeed(0);
                  current_state = CoralStates.FINISHED;
              }
              break;
          case FINISHED:
              outtake.setOuttakeSpeed(0);
              break;
      }
      SmartDashboard.putNumber("Avg Current", running_avg);
  }

  @Override
  public boolean isFinished() {
    return current_state == CoralStates.FINISHED;
  }

  @Override
  public void end(boolean interrupted) {
    outtake.setOuttakeSpeed(0);


  }
}