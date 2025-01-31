package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

/**
 * The MotorControlGroup class represents a group of motor controllers.
 * It provides methods to control multiple motors simultaneously, including setting power,
 * stopping motors, and accessing encoder values.
 */
public class MotorControlGroup extends SubsystemBase {
    private List<SparkBase> motors;

    /**
     * Constructs a new MotorControlGroup with the specified motors.
     * 
     * @param motors The motors to be controlled by this group.
     */
    public MotorControlGroup(SparkBase... motors) {
        this.motors = new ArrayList<>();
        for (SparkBase motor : motors) {
            this.motors.add(motor);
        }
    }

    /**
     * Sets the power for all motors in the group.
     * 
     * @param power The power to set for the motors.
     */
    public void set(double power) {
        for (SparkBase motor : motors) {
            motor.set(power);
        }
    }

    /**
     * Sets the power for a specific motor in the group.
     * 
     * @param power The power to set for the motor.
     * @param motorIndex The index of the motor to set the power for.
     */
    public void set(double power, int motorIndex) {
        if (motorIndex >= 0 && motorIndex < motors.size()) {
            motors.get(motorIndex).set(power);
        }
    }

    /**
     * Stops all motors in the group.
     */
    public void stop() {
        for (SparkBase motor : motors) {
            motor.set(0);
        }
    }

    /**
     * Stops a specific motor in the group.
     * 
     * @param motorIndex The index of the motor to stop.
     */
    public void stop(int motorIndex) {
        if (motorIndex >= 0 && motorIndex < motors.size()) {
            motors.get(motorIndex).set(0);
        }
    }

    /**
     * Gets the encoder position of a specific motor in the group.
     * 
     * @param motorIndex The index of the motor to get the encoder position for.
     * @return The encoder position of the motor.
     */
    public double getEncoderPosition(int motorIndex) {
        if (motorIndex >= 0 && motorIndex < motors.size()) {
            return motors.get(motorIndex).getEncoder().getPosition();
        }
        return 0;
    }

    /**
     * Gets the encoder velocity of a specific motor in the group.
     * 
     * @param motorIndex The index of the motor to get the encoder velocity for.
     * @return The encoder velocity of the motor.
     */
    public double getEncoderVelocity(int motorIndex) {
        if (motorIndex >= 0 && motorIndex < motors.size()) {
            return motors.get(motorIndex).getEncoder().getVelocity();
        }
        return 0;
    }

    /**
     * Sets the PID coefficients for all motors in the group.
     * 
     * @param p The proportional coefficient.
     * @param i The integral coefficient.
     * @param d The derivative coefficient.
     */
    public void setPID(double p, double i, double d) {
        for (SparkBase motor : motors) {
            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop.pid(p, i, d);
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    /**
     * Sets the PID coefficients for a specific motor in the group.
     * 
     * @param p The proportional coefficient.
     * @param i The integral coefficient.
     * @param d The derivative coefficient.
     * @param motorIndex The index of the motor to set the PID coefficients for.
     */
    public void setPID(double p, double i, double d, int motorIndex) {
        if (motorIndex >= 0 && motorIndex < motors.size()) {
            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop.pid(p, i, d);
            motors.get(motorIndex).configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    /**
     * Gets the temperature of a specific motor in the group.
     * 
     * @param motorIndex The index of the motor to get the temperature for.
     * @return The temperature of the motor.
     */
    public double getTemperature(int motorIndex) {
        if (motorIndex >= 0 && motorIndex < motors.size()) {
            return motors.get(motorIndex).getMotorTemperature();
        }
        return 0;
    }

    /**
     * Gets the average temperature of all motors in the group.
     * 
     * @return The average temperature of all motors.
     */
    public double getAverageTemperature() {
        double totalTemp = 0;
        for (SparkBase motor : motors) {
            totalTemp += motor.getMotorTemperature();
        }
        return totalTemp / motors.size();
    }

    /**
     * Sets the idle mode for all motors in the group.
     * 
     * @param mode The idle mode to set for the motors.
     */
    public void setIdleMode(IdleMode mode) {
        for (SparkBase motor : motors) {
            SparkMaxConfig config = new SparkMaxConfig();
            config.idleMode(mode);
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    /**
     * Sets the idle mode for a specific motor in the group.
     * 
     * @param mode The idle mode to set for the motor.
     * @param motorIndex The index of the motor to set the idle mode for.
     */
    public void setIdleMode(IdleMode mode, int motorIndex) {
        if (motorIndex >= 0 && motorIndex < motors.size()) {
            SparkMaxConfig config = new SparkMaxConfig();
            config.idleMode(mode);
            motors.get(motorIndex).configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    /**
     * Burns the flash memory for all motors in the group.
     */
    public void burnFlash() {
        for (SparkBase motor : motors) {
            // In REVLib 2025, burnFlash() is handled by configure() with PersistMode.kPersistParameters
            SparkMaxConfig config = new SparkMaxConfig();
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    /**
     * Burns the flash memory for a specific motor in the group.
     * 
     * @param motorIndex The index of the motor to burn the flash memory for.
     */
    public void burnFlash(int motorIndex) {
        if (motorIndex >= 0 && motorIndex < motors.size()) {
            // In REVLib 2025, burnFlash() is handled by configure() with PersistMode.kPersistParameters
            SparkMaxConfig config = new SparkMaxConfig();
            motors.get(motorIndex).configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }
}