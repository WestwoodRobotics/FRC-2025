package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.utils.elevator.elevatorPositions;

public class elevatorSetPosition extends Command{

    private double targetPosition;
    private Elevator elevator;

    public elevatorSetPosition(Elevator elevator, elevatorPositions position) {
        this.targetPosition = position.getPosition();
        this.elevator = elevator;        
        addRequirements(elevator);
    }

    public elevatorSetPosition(Elevator elevator, double position){
        this.targetPosition = position;
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.getPIDController().setSetpoint(targetPosition);
    }

    @Override
    public void execute() {
        elevator.setElevatorSpeed(elevator.getPIDController().calculate(elevator.getElevatorPosition()));
    }

    @Override
    public void end(boolean isInterrupted) {
        elevator.setElevatorSpeed(0);

    }





    
}
