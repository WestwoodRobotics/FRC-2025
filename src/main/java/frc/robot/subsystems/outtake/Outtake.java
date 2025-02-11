package frc.robot.subsystems.outtake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.misc.CANdiBeamBreak;

import com.ctre.phoenix6.hardware.CANdi;

public class Outtake extends SubsystemBase {
    private SparkMax outtakeMotor;
    private CANdiBeamBreak coralDetector;
    private PIDController outtakePIDController;
    
    public Outtake() {
        
        outtakeMotor = new SparkMax(PortConstants.outtakeMotorPort, MotorType.kBrushless);
        outtakePIDController = new PIDController(OuttakeConstants.kP, OuttakeConstants.kI, OuttakeConstants.kD);
        coralDetector = new CANdiBeamBreak(PortConstants.kCANdiPort);   

    }

    public void setOuttakeSpeed(double speed) {
        outtakeMotor.set(speed);
    }

    public double getOuttakeRPM() {
        return outtakeMotor.getEncoder().getVelocity();
    }

    public PIDController getOuttakePIDController() {
        return outtakePIDController;
    }

    public void stopOuttake() {
        outtakeMotor.set(0);
    }

    public boolean isCoralDetected(){
        return coralDetector.isBeamBroken();
    }

    public boolean isCoralNotDetected(){
        return coralDetector.isBeamIntact();
    }

    public boolean doesCoralDetectorExist(){
        return coralDetector.doesBeamBreakExist();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Outtake RPM", outtakeMotor.getEncoder().getVelocity());
    }
    
}
