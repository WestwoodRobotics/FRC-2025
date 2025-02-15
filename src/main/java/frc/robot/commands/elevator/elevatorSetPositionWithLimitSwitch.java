package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.elevator;
import frc.robot.subsystems.utils.elevator.elevatorPositions;
import frc.robot.Constants;

public class elevatorSetPositionWithLimitSwitch extends Command {
    private final double targetPosition;
    private final elevator elevator;
    private final double tolerance = 0.5;
    private boolean finished = false;
    
    public elevatorSetPositionWithLimitSwitch(elevator elevator, elevatorPositions position) {
        this.targetPosition = position.getPosition();
        this.elevator = elevator;
        addRequirements(elevator);
    }
    
    public elevatorSetPositionWithLimitSwitch(elevator elevator, double position) {
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
        
        // if (elevator.getElevatorTopLimitSwitch().getStatus()) {
        //     elevator.setElevatorSpeed(0);
        //     elevator.setElevatorEncoderPosition(elevatorPositions.L4.getPosition());
        // }

        if (elevator.getElevatorBottomLimitSwitch().isTriggered()) {
            elevator.setElevatorSpeed(0);
            elevator.setElevatorEncoderPosition(elevatorPositions.HOME.getPosition());
        }
        
        double output = elevator.getPIDController().calculate(elevator.getElevatorPosition(), targetPosition);
        elevator.setElevatorSpeed(output);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        elevator.setElevatorSpeed(0);
    }
}

