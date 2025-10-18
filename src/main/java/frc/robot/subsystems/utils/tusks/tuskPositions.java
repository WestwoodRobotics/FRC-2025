package frc.robot.subsystems.utils.tusks;

import frc.robot.Constants.TuskConstants;

public enum tuskPositions {

    OUT(TuskConstants.kOutPosition),
    IN(TuskConstants.kInPosition);
    

    private final double position;

    tuskPositions(double position){
        this.position = position;
    }

    public double getPosition(){
        return position;
    }
    
}
