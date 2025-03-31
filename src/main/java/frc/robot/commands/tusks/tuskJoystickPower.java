package frc.robot.commands.tusks;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tusks.Tusks;

public class tuskJoystickPower extends Command{

    private Tusks tusks;
    private XboxController operatorController;
    
    public tuskJoystickPower(Tusks tusks, XboxController operatorController){
        this.tusks = tusks;
        this.operatorController = operatorController;
    }   
    //left out
    //right in
    @Override
    public void execute (){
        tusks.setPivotPower(operatorController.getLeftY());
    }

    public void end(boolean interrupted){
        tusks.lockPosition();
    }
    
}
