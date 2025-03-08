package frc.robot.subsystems.outtake;

import com.revrobotics.spark.SparkFlex;
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
    private SparkFlex outtakeMotor;
    private CANdiBeamBreak coralDetector;
    private PIDController outtakePIDController;
    
    public Outtake() {

        outtakeMotor = new SparkFlex(PortConstants.outtakeMotorPort, MotorType.kBrushless);
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
        return coralDetector.getBeamBreakStatus();
    }

    public boolean isCoralNotDetected(){
        return !coralDetector.getBeamBreakStatus();
    }

    public double getOuttakeCurrent(){
        return outtakeMotor.getOutputCurrent();
    }

    @Override
    public void periodic(){
        //System.out.println( outtakeMotor.getEncoder().getVelocity()); //-1700
        SmartDashboard.putNumber("Outtake Current", outtakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Outtake RPM", outtakeMotor.getEncoder().getVelocity());
    }


    
}
