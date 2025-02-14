package frc.robot.sensors.DIO;

public class LimitSwitch extends GenericDigitalPinObject {
    /**
     * Constructs a LimitSwitch object with a specified channel.
     * 
     * @param channel The digital input channel the limit switch is connected to.
     */
    public LimitSwitch(int channel) {
        super(channel);
        
    }

    /**
     * Returns the status of the limit switch.
     * 
     * @return True if the switch is triggered, false if it is not.
     */
    public boolean isTriggered() {
        return getStatus();
    }
    
}
