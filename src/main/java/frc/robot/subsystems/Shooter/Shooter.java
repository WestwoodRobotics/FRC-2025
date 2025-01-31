package frc.robot.subsystems.Shooter;




import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import CustomLibs.QualityOfLife.NeoSparkFlex;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.UtilityConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Shooter class represents the shooter subsystem of the robot.
 * It controls the top and bottom rollers of the shooter and provides methods to set the motor power,
 * stop the motors, and get the motor's RPM.
 */
public class Shooter extends SubsystemBase {

    private NeoSparkFlex topRoller;
    private NeoSparkFlex bottomRoller;

    private boolean isPIDControl;

    private PIDController TopRollerPIDController;
    private PIDController BottomRollerPIDController;

    private double TopRollerRPMSetpoint;

    private double BottomRollerRPMSetpoint;

    /**
     * Constructs a new Shooter subsystem.
     * Initializes the top and bottom rollers and PID controllers.
     * 
     * @param isPIDControl Whether to use PID control for the shooter.
     */
    public Shooter(boolean isPIDControl) {
        topRoller = new NeoSparkFlex(ShooterConstants.kTopRollerPort, MotorType.kBrushless);
        bottomRoller = new NeoSparkFlex(ShooterConstants.kBottomRollerPort, MotorType.kBrushless);

        this.isPIDControl = isPIDControl;
        topRoller.setIdleMode(IdleMode.kBrake);
        bottomRoller.setIdleMode(IdleMode.kBrake);

        // Apply the configuration to the rollers
        topRoller.configure(topRoller.getCurrentWorkingConfig());
        bottomRoller.configure(bottomRoller.getCurrentWorkingConfig());

        this.TopRollerPIDController = new PIDController(
            ShooterConstants.kTopRollerP,
            ShooterConstants.kTopRollerI,
            ShooterConstants.kTopRollerD
        );

        this.BottomRollerPIDController = new PIDController(
            ShooterConstants.kBottomRollerP,
            ShooterConstants.kBottomRollerI,
            ShooterConstants.kBottomRollerD
        );

        
    }

    /**
     * Sets the power of the top roller motor.
     * 
     * @param power The power to set for the top roller motor.
     */
    public void setTopRollerPower(double power) {
        topRoller.set(-power);
    }

    /**
     * Sets the power of the bottom roller motor.
     * 
     * @param power The power to set for the bottom roller motor.
     */
    public void setBottomRollerPower(double power) {
        bottomRoller.set(-power);
    }

    /**
     * Sets the power of both the top and bottom roller motors.
     * 
     * @param power The power to set for both the top and bottom roller motors.
     */
    public void setShooterPower(double power) {
        setTopRollerPower(power);
        setBottomRollerPower(power);
    }

    public void setShooterPower(double upperPower, double lowerPower) {
        setTopRollerPower(-upperPower);
        setBottomRollerPower(-lowerPower);
    }

    /**
     * Gets the raw RPM of the top roller motor.
     * 
     * @return The raw RPM of the top roller motor.
     */
    public double getTopRollerMotorRawRPM() {
        return topRoller.getEncoder().getVelocity();
    }

    /**
     * Gets the RPM of the top roller motor, converted using a conversion factor.
     * 
     * @return The converted RPM of the top roller motor.
     */
    public double getTopRollerRPM() {
        return getTopRollerMotorRawRPM() * ShooterConstants.kTopRollerRPMConversionFactor;
    }

    /**
     * Gets the raw RPM of the bottom roller motor.
     * 
     * @return The raw RPM of the bottom roller motor.
     */
    public double getBottomRollerMotorRawRPM() {
        return bottomRoller.getEncoder().getVelocity();
    }

    /**
     * Gets the RPM of the bottom roller motor, converted using a conversion factor.
     * 
     * @return The converted RPM of the bottom roller motor.
     */
    public double getBottomRollerRPM() {
        return getBottomRollerMotorRawRPM() * ShooterConstants.kBottomRollerRPMConversionFactor;
    }

    /**
     * Gets the PID controller for the top roller motor.
     * 
     * @return The PID controller for the top roller motor.
     */
    public PIDController getTopRollerPIDController() {
        return TopRollerPIDController;
    }

    /**
     * Gets the PID controller for the bottom roller motor.
     * 
     * @return The PID controller for the bottom roller motor.
     */
    public PIDController getBottomRollerPIDController() {
        return BottomRollerPIDController;
    }

    /**
     * Sets the RPM setpoint for the top roller motor.
     * 
     * @param setpoint The RPM setpoint for the top roller motor.
     */
    public void setTopRollerRPMSetpoint(double setpoint) {
        TopRollerRPMSetpoint = setpoint;
    }

    /**
     * Sets the RPM setpoint for the bottom roller motor.
     * 
     * @param setpoint The RPM setpoint for the bottom roller motor.
     */
    public void setBottomRollerRPMSetpoint(double setpoint) {
        BottomRollerRPMSetpoint = setpoint;
    }

    /**
     * Gets the RPM setpoint for the top roller motor.
     * 
     * @return The RPM setpoint for the top roller motor.
     */
    public double getTopRollerRPMSetpoint() {
        return TopRollerRPMSetpoint;
    }

    /**
     * Gets the RPM setpoint for the bottom roller motor.
     * 
     * @return The RPM setpoint for the bottom roller motor.
     */
    public double getBottomRollerRPMSetpoint() {
        return BottomRollerRPMSetpoint;
    }

    /**
     * Returns whether PID control is enabled for the shooter.
     * 
     * @return True if PID control is enabled, false otherwise.
     */
    public boolean isPIDControl() {
        return isPIDControl;
    }

    /**
     * Stops both the top and bottom roller motors.
     */
    public void stopShooter() {
        setTopRollerPower(0);
        setBottomRollerPower(0);
    }

    /**
     * Periodically updates the SmartDashboard with the shooter motor's RPM and current.
     * This method is called automatically to update sensor status on the dashboard.
     */
    @Override
    public void periodic() {
        if (UtilityConstants.debugMode) {
            SmartDashboard.putNumber("Top Roller RPM", getTopRollerMotorRawRPM());
            SmartDashboard.putNumber("Bottom Roller RPM", getBottomRollerMotorRawRPM());
            SmartDashboard.putNumber("Top Roller Current", topRoller.getOutputCurrent());
            SmartDashboard.putNumber("Bottom Roller Current", bottomRoller.getOutputCurrent());
        }
    }
}