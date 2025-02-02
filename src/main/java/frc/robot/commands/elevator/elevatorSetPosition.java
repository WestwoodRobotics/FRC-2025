package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.elevator;
import frc.robot.subsystems.utils.elevator.elevatorPositions;

public class elevatorSetPosition extends Command{

    private double targetPosition;
    private elevator elevator;

    public elevatorSetPosition(elevator elevator, elevatorPositions position) {
        this.targetPosition = position.getPosition();
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
        elevator.setElevatorPosition(targetPosition);
    }

    @Override
    public void end(boolean isInterrupted) {
    }





    
}
