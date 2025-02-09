package frc.robot.subsystems.intake;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.UtilityConstants;
import frc.robot.subsystems.utils.SparkModels;

import com.revrobotics.spark.SparkMax;



import CustomLibs.QualityOfLife.NeoSparkMax;


import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.*;




/**
 * The Intake class represents the intake subsystem of the robot.
 * It controls the intake motor and provides methods to set the motor power,
 * stop the motor, and get the motor's RPM.
 */
public class Intake extends SubsystemBase {
    private SparkMax intakeMotorSide;
    private SparkMax intakeMotorTop;
    private PIDController sidePIDController;
    private PIDController topPIDController;

    /**
     * Constructs a new Intake subsystem.
     * Initializes the intake motor and PID controller.
     */
    public Intake() {
        intakeMotorSide = new SparkMax(PortConstants.kIntakeMotorSidePort, MotorType.kBrushless); 
        intakeMotorTop = new SparkMax(PortConstants.kIntakeMotorTopPort, MotorType.kBrushless); 
        this.sidePIDController = new PIDController(IntakeConstants.kSideP, 
                                               IntakeConstants.kSideI, 
                                               IntakeConstants.kSideD);
        this.topPIDController = new PIDController(IntakeConstants.kTopP, 
                                               IntakeConstants.kTopI, 
                                               IntakeConstants.kTopD);
    }

    /**
     * Sets the power of the intake motor.
     * 
     * @param power The power to set for the intake motor.
     */
    public void setIntakePower(double power) {
        intakeMotorSide.set(power);
        intakeMotorTop.set(power/2);
        // System.out.println(power);
    }

    /**
     * Stops the intake motor.
     */
    public void stopIntake(){
        intakeMotorSide.set(0);
        intakeMotorTop.set(0);
    }

    /**
     * Gets the raw RPM of the intake motor.
     * 
     * @return The raw RPM of the intake motor.
     */
    public double getRawMotorRPM(){
        return intakeMotorSide.getEncoder().getVelocity();
    }

    /**
     * Gets the RPM of the intake motor, converted using a conversion factor.
     * 
     * @return The converted RPM of the intake motor.
     */
    public double getRPM(){
        return getRawMotorRPM() * IntakeConstants.kRPMConversionFactor;
    }

    /**
     * Gets the PID controller for the side intake motor.
     * 
     * @return The PID controller for the side intake motor.
     */
    public PIDController getSidePIDController(){
        return sidePIDController;
    }

    /**
     * Gets the PID controller for the top intake motor.
     * 
     * @return The PID controller for the top intake motor.
     */
    public PIDController getTopPIDController(){
        return topPIDController;
    }

    /**
     * Sets the power of the side intake motor.
     * 
     * @param power The power to set for the side intake motor.
     */
    public void setSideIntakePower(double power) {
        intakeMotorSide.set(power);
    }

    /**
     * Sets the power of the top intake motor.
     * 
     * @param power The power to set for the top intake motor.
     */
    public void setTopIntakePower(double power) {
        intakeMotorTop.set(power);
    }

    /**
     * Gets the raw RPM of the side intake motor.
     * 
     * @return The raw RPM of the side intake motor.
     */
    public double getSideMotorRPM() {
        return intakeMotorSide.getEncoder().getVelocity();
    }

    /**
     * Gets the raw RPM of the top intake motor.
     * 
     * @return The raw RPM of the top intake motor.
     */
    public double getTopMotorRPM() {
        return intakeMotorTop.getEncoder().getVelocity();
    }
    /**
     * Periodically updates the SmartDashboard with the intake motor's RPM and current.
     * This method is called automatically to update sensor status on the dashboard.
     */
    @Override
    public void periodic() {
        if (UtilityConstants.debugMode){
            SmartDashboard.putNumber("Intake RPM", getRPM());
            SmartDashboard.putNumber("Intake Current", intakeMotorSide.getOutputCurrent());
        }
    }

}