package frc.robot.subsystems.utils.elevator;

import frc.robot.Configs.Elevator;
import frc.robot.Constants.ElevatorConstants;

public enum elevatorPositions {

    HOME(ElevatorConstants.kElevatorHomePosition),
    L2(ElevatorConstants.kElevatorL2Position),
    L3(ElevatorConstants.kElevatorL3Position),
    L35(ElevatorConstants.kElevatorL35Position),
    L4(ElevatorConstants.kElevatorL4Position);
    


    private final double position;

    elevatorPositions(double position) {
        this.position = position;
    }

    public double  getPosition() {
        return position;
    }   
}
