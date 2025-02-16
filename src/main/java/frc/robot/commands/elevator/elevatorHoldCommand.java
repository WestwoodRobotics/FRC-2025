package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;


public class elevatorHoldCommand extends Command {
    private final Elevator elevator;
    private double holdPosition;

    public elevatorHoldCommand(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        holdPosition = elevator.getElevatorPosition();
        elevator.getPIDController().setSetpoint(holdPosition);
    }

    @Override
    public void execute() {
        double output = elevator.getPIDController().calculate(elevator.getElevatorPosition());
        elevator.setElevatorSpeed(output);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setElevatorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}