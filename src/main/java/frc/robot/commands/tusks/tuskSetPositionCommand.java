package frc.robot.commands.tusks;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tusks.Tusks;
import frc.robot.subsystems.utils.tusks.tuskPositions;

public class tuskSetPositionCommand extends Command{

    private final double targetPosition;
    private  tuskPositions targetPoseEnum;
    private final Tusks tusks;
    private static final double thresh = 0.2;
    private final double tolerance = 0.5;
    private boolean finished = false;
    private Timer timer;
    private double counter;

    

    public tuskSetPositionCommand(Tusks tusks, tuskPositions target_position){
        this.targetPoseEnum = target_position;
        this.targetPosition = target_position.getPosition();
        timer = new Timer();
        this.tusks = tusks;
        addRequirements(tusks);
        this.counter = 0;
    }

    public tuskSetPositionCommand(Tusks tusks, double target_position) {
        this.targetPosition = target_position;
        
        this.tusks = tusks;
        addRequirements(tusks);
        this.counter = 0;
    }

    @Override
    public void initialize(){

        tusks.setPivotPosition(targetPosition);
        timer.reset();
        timer.start();
        counter = 0;
    }

    @Override
    public void execute(){
        if (Math.abs(tusks.getPivotPosition() - this.targetPosition) < thresh){
            counter++;
        }
    }

    @Override
    public boolean isFinished(){
        return counter >= 10;
    }

    @Override
    public void end(boolean interrupted){
        if (interrupted){
            tusks.setPivotPostionEnum(tuskPositions.INTERRUPTED);
        }
        else {
            tusks.setPivotPostionEnum(targetPoseEnum);
        }
        
    }

    

    

    
}
