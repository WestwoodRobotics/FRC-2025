package frc.robot.subsystems.utils.tusks;

import frc.robot.Constants.TuskConstants;

public enum tuskPositions {

    GROUND(TuskConstants.kGroundPosition),
    L3(TuskConstants.kL3Position),
    L4(TuskConstants.kL4Position),
    PROCESSOR(TuskConstants.kProcessorPosition),
    
    HOME(TuskConstants.kHomePosition),
    INTERRUPTED(TuskConstants.kInterruptedPosition);

    

    private final double position;

    tuskPositions(double position){
        this.position = position;
    }

    public double getPosition(){
        return position;
    }
    
}
