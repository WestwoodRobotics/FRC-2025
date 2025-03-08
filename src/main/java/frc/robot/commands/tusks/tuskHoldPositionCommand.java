package frc.robot.commands.tusks;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TuskConstants;
import frc.robot.subsystems.tusks.Tusks;

public class tuskHoldPositionCommand extends Command {
    private final Tusks tusks;
    private final PIDController pidController;
    private double holdPosition;

    public tuskHoldPositionCommand(Tusks tusks) {
        
        this.tusks = tusks;

        // Instantiate a new PIDController based on the tusk pivot PID constants
        this.pidController = new PIDController(TuskConstants.kPivotP, TuskConstants.kPivotI, TuskConstants.kPivotD);
        pidController.setTolerance(0.05);
        addRequirements(tusks);
        
    }

    @Override
    public void initialize() {
        holdPosition = tusks.getPivotPosition();
        pidController.reset();
        pidController.setSetpoint(holdPosition);
        tusks.setHoldPoseMode(false);
    }

    @Override
    public void execute() {
        double output = pidController.calculate(tusks.getPivotPosition());
        tusks.setPivotPower(output);
    }

    @Override
    public boolean isFinished() {
        tusks.setHoldPoseMode(true);
        return false; // continue holding indefinitely
    }

    @Override
    public void end(boolean interrupted) {
        tusks.stopPivot();
    }
}
