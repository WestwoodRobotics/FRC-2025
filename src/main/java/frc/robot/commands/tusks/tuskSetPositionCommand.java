package frc.robot.commands.tusks;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tusks.Tusks;
import frc.robot.subsystems.utils.tusks.tuskPositions;

public class tuskSetPositionCommand extends Command{

    private PIDController tuskPIDController;
    private tuskPositions target_position;
    private tuskPositions current_position;
    private Tusks tusks;
    

    public tuskSetPositionCommand(Tusks tusks, tuskPositions target_position){
        this.tusks = tusks;
        this.target_position = target_position;
        addRequirements(tusks);
    }

    @Override
    public void initialize(){

        this.tuskPIDController = tusks.getPIDController();
        this.tuskPIDController.setTolerance(0.3);
        tusks.setTargetPosition(target_position.getPosition());
    }

    @Override
    public boolean isFinished(){
        return tuskPIDController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted){
        tusks.stopPivot();
        if (!interrupted){
            tusks.setCurrentState(target_position);
        } else {
            tusks.setCurrentState(tuskPositions.INTERRUPTED);
        }
    }

    

    

    
}
