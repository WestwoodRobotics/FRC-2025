package frc.robot.commands.tusks;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tusks.Tusks;
import frc.robot.subsystems.utils.tusks.tuskPositions;

public class tuskSetPositionCommand extends Command{

    private PIDController tuskPIDController;
    private tuskPositions position;
    private Tusks tusks;

    public tuskSetPositionCommand(Tusks tusks, tuskPositions position){
        this.tuskPIDController = tusks.getPIDController();
        this.tusks = tusks;

        this.position = position;
        addRequirements(tusks);
    }

    @Override
    public void initialize(){
        tuskPIDController.setSetpoint(position.getPosition());
    }

    @Override
    public void execute(){
        tusks.setPivotPower(tuskPIDController.calculate(tusks.getPivotPosition()));
    }

    @Override
    public boolean isFinished(){
        return tuskPIDController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted){
        tusks.stopPivot();
    }

    

    

    
}
