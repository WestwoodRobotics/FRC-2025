package frc.robot.sensors.DIO;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase {

    public Spark blinkin;

    
    public Blinkin(){
       blinkin = new Spark(2);
    }

    public void setPower(double power){


        blinkin.setVoltage(null);

    }

    @Override
    public void periodic(){
        this.setPower(-0.81);
    }
    
}
