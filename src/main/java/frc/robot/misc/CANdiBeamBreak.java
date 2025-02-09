package frc.robot.misc;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;


public class CANdiBeamBreak {
    
    private CANdi CANdi;
    
    public CANdiBeamBreak(double CANdiID) {
        CANdi = new CANdi(0);
    }
    
    public int getBeamBreakStatus() {

        return CANdi.getS1State(true).getValue().value;
        // 0 == Floating -- (Assumptions from here on out) == (No Beam Break)
        // 1 == Low -- (Assumptions from here on out)) == (Beam Broken)
        // 2 == High -- (Assumptions from here on out) == (Beam Intact)
    }

    public boolean doesBeamBreakExist() {
        return getBeamBreakStatus() == 0;
    }

    public boolean isBeamBroken() {
        return getBeamBreakStatus() == 1;
    }

    public boolean isBeamIntact() {
        return getBeamBreakStatus() == 2;
    }

    




}
