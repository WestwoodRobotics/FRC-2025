package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.utils.elevator.elevatorPositions;
import frc.robot.Constants;

public class elevatorSetPositionWithLimitSwitch extends Command {
    private final double targetPosition;
    private final Elevator elevator;
    private final double tolerance = 0.5;
    private boolean finished = false;
    private Timer timer;
    
    public elevatorSetPositionWithLimitSwitch(Elevator elevator, elevatorPositions position) {
        this.targetPosition = position.getPosition();
        timer = new Timer();
        this.elevator = elevator;
        addRequirements(elevator);
    }
    
    public elevatorSetPositionWithLimitSwitch(Elevator elevator, double position) {
        this.targetPosition = position;
        this.elevator = elevator;
        addRequirements(elevator);
    }
    
    @Override
    public void initialize() {
        elevator.setElevatorPosition(targetPosition);
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute() {
        
        // if (elevator.getElevatorTopLimitSwitch().getStatus()) {
        //     elevator.setElevatorSpeed(0);
        //     elevator.setElevatorEncoderPosition(elevatorPositions.L4.getPosition());
        // }

        if (elevator.getElevatorBottomLimitSwitch().isTriggered()) {
            // elevator.stopElevator();
            elevator.setElevatorEncoderPosition(elevatorPositions.HOME.getPosition());
        }
    }
    
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }
    
    @Override
    public void end(boolean interrupted) {
        // elevator.setElevatorSpeed(0);
    }
}

