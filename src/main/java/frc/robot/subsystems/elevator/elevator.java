package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevator extends SubsystemBase {
    private SparkFlex elevatorMotor1;
    
    

    public elevator(int elevatorMotor1Port) {
        this.elevatorMotor1 = new SparkFlex(elevatorMotor1Port, MotorType.kBrushless);
        
    }

    public void setElevatorSpeed(double speed) {
        elevatorMotor1.set(speed);
        
    }

    public void stopElevator() {
        elevatorMotor1.set(0);
    }
}
