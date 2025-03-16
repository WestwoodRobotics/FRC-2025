package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outtake.Outtake;

public class IntakeOuttakeUntilBeamBroken extends Command{

    private Outtake outtake;
    private double target_power; //-0.2


    public IntakeOuttakeUntilBeamBroken(Outtake outtake, double power){
        this.outtake = outtake;
        this.target_power = power;
        addRequirements(outtake);
    }

    @Override
    public void execute(){
        if (!outtake.isCoralDetected()){
            outtake.setOuttakeSpeed(target_power);
        }
    }

    @Override
    public boolean isFinished(){
        return outtake.isCoralDetected();

    }

    @Override
    public void end(boolean interrupted){
        outtake.setOuttakeSpeed(0);
    }
    
}
