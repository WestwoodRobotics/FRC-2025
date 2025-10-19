package frc.robot.commands.tusks;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tusks.Tusks;

public class tusksHoldCommand extends Command {
    private final Tusks tusks;
    private double holdPosition;

    public tusksHoldCommand(Tusks tusks) {
        this.tusks = tusks;
        addRequirements(tusks);
    }

    @Override
    public void initialize() {
        holdPosition = tusks.getPivotPosition();
        tusks.getPivotPIDController().setSetpoint(holdPosition);
    }

    @Override
    public void execute() {
        double output = tusks.getPivotPIDController().calculate(tusks.getPivotPosition());
        tusks.setPivotPower(output);
    }

    @Override
    public void end(boolean interrupted) {
        tusks.setPivotPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
