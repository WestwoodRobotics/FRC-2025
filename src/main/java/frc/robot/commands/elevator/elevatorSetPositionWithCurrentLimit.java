package frc.robot.commands.elevator;

import java.util.LinkedList;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.utils.elevator.elevatorPositions;

public class elevatorSetPositionWithCurrentLimit extends Command {
    private double targetPosition;
    private Elevator elevator;
    private LinkedList<Double> recentCurrents = new LinkedList<Double>();
    private int samplesProcessed = 0;
    private double currentThreshold;
    private int warmupSamples;
    private int windowSize;
    private double runningAvg = 0;

    public elevatorSetPositionWithCurrentLimit(Elevator elevator, elevatorPositions position, 
                                             double currentThreshold, int warmupSamples, int windowSize) {
        this.targetPosition = position.getPosition();
        this.elevator = elevator;
        this.currentThreshold = currentThreshold;
        this.warmupSamples = warmupSamples;
        this.windowSize = windowSize;
        addRequirements(elevator);
    }

    public elevatorSetPositionWithCurrentLimit(Elevator elevator, double position,
                                             double currentThreshold, int warmupSamples, int windowSize) {
        this.targetPosition = position;
        this.elevator = elevator;
        this.currentThreshold = currentThreshold;
        this.warmupSamples = warmupSamples;
        this.windowSize = windowSize;
        addRequirements(elevator);
    }

    public elevatorSetPositionWithCurrentLimit(Elevator elevator, elevatorPositions position) {
        this(elevator, position, 
             ElevatorConstants.kCurrentDetectionThreshold,
             ElevatorConstants.kCurrentWarmupSamples,
             ElevatorConstants.kCurrentWindowSize);
    }

    public elevatorSetPositionWithCurrentLimit(Elevator elevator, double position) {
        this(elevator, position,
             ElevatorConstants.kCurrentDetectionThreshold,
             ElevatorConstants.kCurrentWarmupSamples,
             ElevatorConstants.kCurrentWindowSize);
    }

    @Override
    public void initialize() {
        elevator.getPIDController().setSetpoint(targetPosition);
        recentCurrents.clear();
        samplesProcessed = 0;
        runningAvg = 0;
    }

    @Override
    public void execute() {
        samplesProcessed++;
        double current = elevator.getElevatorCurrent();

        if (samplesProcessed > warmupSamples) {
            recentCurrents.add(current);
            if (recentCurrents.size() > windowSize) {
                recentCurrents.removeFirst();
            }

            if (recentCurrents.size() == windowSize) {
                double sum = 0;
                for (Double value : recentCurrents) {
                    sum += value;
                }
                runningAvg = sum / windowSize;
            }
        }

        elevator.setElevatorSpeed(elevator.getPIDController().calculate(elevator.getElevatorPosition()));
        SmartDashboard.putNumber("Elevator Current Avg", runningAvg);
    }
    

    @Override
    public void end(boolean interrupted) {
        elevator.setElevatorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return samplesProcessed > warmupSamples && 
               recentCurrents.size() == windowSize && 
               runningAvg > currentThreshold;
    }
}