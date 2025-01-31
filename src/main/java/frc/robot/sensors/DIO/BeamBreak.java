package frc.robot.sensors.DIO;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The BeamBreak class represents a digital beam break sensor subsystem.
 * It uses a DigitalInput to detect whether the beam is broken or not.
 * 
 * This class extends GenericDigitalPinObject, providing methods to check the beam break status.
 */
public class BeamBreak extends GenericDigitalPinObject{

    private double beamBreakChannel; // Channel number for the beam break sensor

    /**
     * Constructs a BeamBreak object with a specified channel.
     * 
     * @param beamBreakChannel The digital input channel the beam break sensor is connected to.
     */
    public BeamBreak(int beamBreakChannel){
       super(beamBreakChannel);
       this.beamBreakChannel = beamBreakChannel;
    }

    public boolean isBroken(){
        return getStatus();
    }
    /**
     * Periodically updates the SmartDashboard with the status of the beam break sensor.
     * This method is called automatically to update sensor status on the dashboard.
     */
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Beam Break " + beamBreakChannel, this.getStatus());
    }
}