package frc.robot.subsystems.outtake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {
    private SparkMax outtakeMotor;
    
    public Outtake() {
        outtakeMotor = new SparkMax(0, MotorType.kBrushless);
    }

    public void setOuttakeSpeed(double speed) {
    }

    public void stopOuttake() {
    }
    
}
