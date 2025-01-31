package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import CustomLibs.QualityOfLife.NeoSparkMax;


import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.UtilityConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The preRoller class represents the preRoller subsystem of the robot.
 * It controls the preRoller motor and provides methods to set the motor power,
 * stop the motor, and get the motor's RPM and current.
 */
public class preRoller extends SubsystemBase {
    // Define your member variables here
    private NeoSparkMax preRollerMotor;

    private PIDController PIDController;
    private double currentCurrentOffset = 0;

    private boolean printed = false;
    private double lastCurrent;

    private boolean isHoldingNote;

    // Constructor
    /**
     * Constructs a new preRoller subsystem.
     * Initializes the preRoller motor and PID controller.
     */
    public preRoller() {
        preRollerMotor = new NeoSparkMax(ShooterConstants.kPreRollerPort, MotorType.kBrushless);
        this.PIDController = new PIDController(ShooterConstants.kPreRollerP, 
                                               ShooterConstants.kPreRollerI, 
                                               ShooterConstants.kPreRollerD);
    }

    // Define your methods here

    /**
     * Sets the power of the preRoller motor.
     * 
     * @param power The power to set for the preRoller motor.
     */
    public void setPreRollerPower(double power) {
        preRollerMotor.set(power);
    }

    /**
     * Stops the preRoller motor.
     */
    public void stopPreRoller(){
        preRollerMotor.set(0);
    }

    /**
     * Gets the raw RPM of the preRoller motor.
     * 
     * @return The raw RPM of the preRoller motor.
     */
    public double getRawMotorRPM(){
        return preRollerMotor.getEncoder().getVelocity();
    }

    // public double getRPM(){
    //     return getRawMotorRPM() * ShooterConstants.kPreRollerRPMConversionFactor;
    // }

    /**
     * Gets the PID controller for the preRoller motor.
     * 
     * @return The PID controller for the preRoller motor.
     */
    public PIDController getPIDController(){
        return PIDController;
    }

    //A method that will return the current output current of the motor
    /**
     * Gets the output current of the preRoller motor.
     * 
     * @return The output current of the preRoller motor.
     */
    public double getOutputCurrent(){
        return preRollerMotor.getOutputCurrent() - currentCurrentOffset;
    }
    

    /**
     * Periodically updates the SmartDashboard with the preRoller motor's RPM and current.
     * This method is called automatically to update sensor status on the dashboard.
     */
    @Override
    public void periodic() {
        if (UtilityConstants.debugMode){
            // if(!printed){
            //     System.out.println("PreRoller Current: " + getOutputCurrent());
            //     lastCurrent = this.getOutputCurrent();
            //     printed = true;
            // }
            // else if(Math.abs(lastCurrent - preRollerMotor.getOutputCurrent()) > 10){
            //     System.out.println("PreRoller Current: " + getOutputCurrent());
            //     lastCurrent = this.getOutputCurrent();
            // }

            //System.out.println("Motor RPM: " + this.getRawMotorRPM());

            SmartDashboard.putNumber("PreRoller Current", getOutputCurrent());
            SmartDashboard.putNumber("PreRoller RPM", getRawMotorRPM());
        }
    }

    /**
     * Resets the output current offset of the preRoller motor.
     */
    public void resetOutputCurrent(){
        currentCurrentOffset = getOutputCurrent();
    }

    /**
     * Sets whether the preRoller is holding a note.
     * 
     * @param isHoldingNote True if the preRoller is holding a note, false otherwise.
     */
    public void setHoldingNote(boolean isHoldingNote){
         this.isHoldingNote = isHoldingNote;
    }

    /**
     * Gets whether the preRoller is holding a note.
     * 
     * @return True if the preRoller is holding a note, false otherwise.
     */
    public boolean getHoldingNote(){
        return isHoldingNote;
    }
}