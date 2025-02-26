package frc.robot.sensors.DIO;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.PhotonVisionCameras;
import frc.robot.subsystems.intake.Intake;

public class LEDController extends SubsystemBase {
    public Spark blinkin;
    private PhotonVisionCameras cameras;
    
    public LEDController(PhotonVisionCameras cameras){
       blinkin = new Spark(2);
       this.cameras = cameras;
    }
    
    @Override
    public void periodic(){
        if (this.cameras != null ){
            if(this.cameras.reefCameraHasTarget()) {
                this.blinkin.set(0.73);
            }
            else {
                this.blinkin.set(-0.81);
            }
        }
    }
}
