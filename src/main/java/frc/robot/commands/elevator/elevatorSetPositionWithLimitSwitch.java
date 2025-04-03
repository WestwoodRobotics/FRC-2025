package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.utils.elevator.elevatorPositions;
import frc.robot.Constants;

public class elevatorSetPositionWithLimitSwitch extends Command {
    private final double targetPosition;
    private  elevatorPositions targetPoseEnum;
    private final Elevator elevator;
    private static final double thresh = 0.5;
    private final double tolerance = 0.5;
    private boolean finished = false;
    private Timer timer;
    private double counter;
    
    public elevatorSetPositionWithLimitSwitch(Elevator elevator, elevatorPositions position) {
        this.targetPoseEnum = position;
        this.targetPosition = position.getPosition();
        timer = new Timer();
        this.elevator = elevator;
        addRequirements(elevator);
        this.counter = 0;
    }
    
    public elevatorSetPositionWithLimitSwitch(Elevator elevator, double position) {
        this.targetPosition = position;
        
        this.elevator = elevator;
        addRequirements(elevator);
        this.counter = 0;
    }
    
    @Override
    public void initialize() {
        elevator.setElevatorPosition(targetPosition);
        timer.reset();
        timer.start();
        counter = 0;
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
        if (Math.abs(elevator.getElevatorPosition() - this.targetPosition) < thresh) {
            counter++;
        }
    }
    
    @Override
    public boolean isFinished() {
        return counter >= 10;
    }
    
    @Override
    public void end(boolean interrupted) {
        // elevator.setElevatorSpeed(0);
        if (interrupted){
            elevator.setElevatorPositionEnum(elevatorPositions.INTERRUPTED);
        }   
        else{
            elevator.setElevatorPositionEnum(targetPoseEnum);
        }

    }
}

