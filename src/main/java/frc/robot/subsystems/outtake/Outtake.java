package frc.robot.subsystems.outtake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class Outtake extends SubsystemBase {
    private SparkMax outtakeMotor;
    
    public Outtake() {
        outtakeMotor = new SparkMax(PortConstants.outtakeMotorPort, MotorType.kBrushless);
    }

    public void setOuttakeSpeed(double speed) {
        outtakeMotor.set(speed);
    }

    public void stopOuttake() {
    }
    
}
