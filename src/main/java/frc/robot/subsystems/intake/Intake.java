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





/**
 * The Intake class represents the intake subsystem of the robot.
 * It controls the intake motor and provides methods to set the motor power,
 * stop the motor, and get the motor's RPM.
 */
public class Intake extends SubsystemBase {
    private SparkBase intakeMotor;
    private PIDController PIDController;

    /**
     * Constructs a new Intake subsystem.
     * Initializes the intake motor and PID controller.
     */
    public Intake() {
        intakeMotor = new NeoSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless); 
        this.PIDController = new PIDController(IntakeConstants.kP, 
                                               IntakeConstants.kI, 
                                               IntakeConstants.kD);
    }

    /**
     * Sets the power of the intake motor.
     * 
     * @param power The power to set for the intake motor.
     */
    public void setIntakePower(double power) {
        intakeMotor.set(power);
    }

    /**
     * Stops the intake motor.
     */
    public void stopIntake(){
        intakeMotor.set(0);
    }

    /**
     * Gets the raw RPM of the intake motor.
     * 
     * @return The raw RPM of the intake motor.
     */
    public double getRawMotorRPM(){
        return intakeMotor.getEncoder().getVelocity();
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
     * Gets the PID controller for the intake motor.
     * 
     * @return The PID controller for the intake motor.
     */
    public PIDController getPIDController(){
        return PIDController;
    }

    /**
     * Periodically updates the SmartDashboard with the intake motor's RPM and current.
     * This method is called automatically to update sensor status on the dashboard.
     */
    @Override
    public void periodic() {
        if (UtilityConstants.debugMode){
            SmartDashboard.putNumber("Intake RPM", getRPM());
            SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
        }
    }

}