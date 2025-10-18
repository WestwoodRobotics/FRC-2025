package frc.robot.commands.tusks;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tusks.Tusks;
import frc.robot.subsystems.utils.tusks.tuskPositions;

public class tuskSetZero extends Command{

    private tuskPositions current_position;
    private Tusks tusks;
    private Timer timer;
    private double prevPos;
    

    public tuskSetZero(Tusks tusks){
        this.tusks = tusks;
        addRequirements(tusks);
        timer = new Timer();
        prevPos = tusks.getPivotPosition();
    }

    @Override
    public void initialize(){

        tusks.setPivotPower(0.05);
        timer.start();

    }

    
    @Override
    public void execute() {
        tusks.setPivotPower(0.05);
    }


    @Override
    public void end(boolean interrupted){
        tusks.setPivotPower(0);
    }

    @Override
    public boolean isFinished(){
        if (timer.get() > 0.5) {
            if (Math.abs(tusks.getPivotPosition() - prevPos) < 0.01) {
                tusks.setTuskEncoderPosition(0);
                return true;
            }}
        prevPos = tusks.getPivotPosition();
        timer.reset();
        return false;
    }



    

    

    
}
