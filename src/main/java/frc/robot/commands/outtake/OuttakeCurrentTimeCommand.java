package frc.robot.commands.outtake;

import java.util.LinkedList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
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


  public OuttakeCurrentTimeCommand(Outtake outtake, double power, double currentDetectionThreshold, double timeOffset) {
    this.outtake = outtake;
    this.power = power;
    this.currentDetectionThreshold = currentDetectionThreshold;
    this.timeOffset = timeOffset;
    addRequirements(outtake);
  }

  @Override
  public void initialize() {
    recentCurrents.add(outtake.getOuttakeCurrent());
    current_state = CoralStates.WAITING_FOR_CORAL;

  }

  @Override
  public void execute() {

    switch (current_state){
        case WAITING_FOR_CORAL:
            outtake.setOuttakeSpeed(power);
            recentCurrents.add(outtake.getOuttakeCurrent());
            if(recentCurrents.size() > 10){
                recentCurrents.removeFirst();
            }
            if(recentCurrents.size() == 10){
                double sum = 0;
                for(Double current : recentCurrents){
                    sum += current;
                }
                double average = sum / recentCurrents.size();
                if(average > currentDetectionThreshold){
                    current_state = CoralStates.FOUND_CORAL;
                    timer.start();
                }
            }
            break;
        case FOUND_CORAL:
            if(timer.hasElapsed(0.5)){
                outtake.setOuttakeSpeed(0);
            }
            break;
        case FINISHED:
            outtake.setOuttakeSpeed(0);
            break;
    }

    
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