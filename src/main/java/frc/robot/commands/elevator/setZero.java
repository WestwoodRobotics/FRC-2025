package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj.Timer;

public class setZero extends Command {
    private final Elevator elevator;
    private Timer timer;
    private double prevPos;

    public elevatorHoldCommand(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
        timer = new Timer();
        prevPos = elevator.getElevatorPosition();
    }

    @Override
    public void initialize() {
        elevator.setElevatorSpeed(0.05);
        timer.start();
    }

    @Override
    public void execute() {
        elevator.setElevatorSpeed(0.05);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setElevatorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        if (timer.get() > 0.5) {
            if (Math.abs(elevator.getElevatorPosition() - prevPos) < 0.01) {
                elevator.setElevatorEncoderPosition(0);
                return true;
            }}
        prevPos = elevator.getElevatorPosition();
        timer.reset();
        return false;
    }
}