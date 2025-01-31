package CustomLibs.QualityOfLife;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkFlexConfigAccessor;

public class NeoSparkFlex extends SparkFlex {
    
    private SparkFlexConfig currentActiveConfig;
    private SparkFlexConfig currentWorkingConfig;

    public NeoSparkFlex(int deviceID, MotorType type) {
        super(deviceID, type);
        getStartingConfig();
        this.configure(currentWorkingConfig);
    }

    private SparkBaseConfig getStartingConfig() {
        SparkFlexConfig config = new SparkFlexConfig();

        // Top-level motor config
        config.inverted(configAccessor.getInverted());
        config.idleMode(configAccessor.getIdleMode());
        // Current limits
        config.smartCurrentLimit(configAccessor.getSmartCurrentLimit(), configAccessor.getSmartCurrentFreeLimit(), configAccessor.getSmartCurrentRPMLimit());
        //config.secondaryCurrentLimit(configAccessor.getSecondaryCurrentLimit(), configAccessor.getSecondaryCurrentLimitChopCycles());
        // Ramp rates
        config.openLoopRampRate(configAccessor.getOpenLoopRampRate());
        config.closedLoopRampRate(configAccessor.getClosedLoopRampRate());

        // Voltage Compensation
        config.voltageCompensation(configAccessor.getVoltageCompensation());

        //
        // Built-in (primary) encoder config
        //
        // config.encoder.inverted(configAccessor.encoder.getInverted());
        // config.encoder.positionConversionFactor(configAccessor.encoder.getPositionConversionFactor());
        // config.encoder.velocityConversionFactor(configAccessor.encoder.getVelocityConversionFactor());
        // If your API supports it, you could also fetch zero offset or other parameters here.
        // config.encoder.zeroOffset(...);

        //
        // External encoder (SparkFlex supports an external encoder)
        // If you use an external encoder, populate those fields:
        //
        config.externalEncoder.inverted(configAccessor.externalEncoder.getInverted());
        config.externalEncoder.positionConversionFactor(configAccessor.externalEncoder.getPositionConversionFactor());
        config.externalEncoder.velocityConversionFactor(configAccessor.externalEncoder.getVelocityConversionFactor());
        // etc., if your code or use case needs them

        //
        // Analog sensor config (if relevant)
        //
        config.analogSensor.inverted(configAccessor.analogSensor.getInverted());
        config.analogSensor.positionConversionFactor(configAccessor.analogSensor.getPositionConversionFactor());
        config.analogSensor.velocityConversionFactor(configAccessor.analogSensor.getVelocityConversionFactor());

        //
        // Absolute encoder config (if relevant)
        //
        config.absoluteEncoder.inverted(configAccessor.absoluteEncoder.getInverted());
        config.absoluteEncoder.positionConversionFactor(configAccessor.absoluteEncoder.getPositionConversionFactor());
        config.absoluteEncoder.velocityConversionFactor(configAccessor.absoluteEncoder.getVelocityConversionFactor());
        config.absoluteEncoder.zeroOffset(configAccessor.absoluteEncoder.getZeroOffset());
        config.absoluteEncoder.averageDepth(configAccessor.absoluteEncoder.getAverageDepth());
        config.absoluteEncoder.startPulseUs(configAccessor.absoluteEncoder.getStartPulseUs());
        config.absoluteEncoder.endPulseUs(configAccessor.absoluteEncoder.getEndPulseUs());

        //
        // Limit switch config
        //
        config.limitSwitch.forwardLimitSwitchEnabled(configAccessor.limitSwitch.getForwardLimitSwitchEnabled());
        config.limitSwitch.forwardLimitSwitchType(configAccessor.limitSwitch.getForwardSwitchType());
        config.limitSwitch.reverseLimitSwitchEnabled(configAccessor.limitSwitch.getReverseLimitSwitchEnabled());
        config.limitSwitch.reverseLimitSwitchType(configAccessor.limitSwitch.getReverseSwitchType());

        //
        // Software limit (soft limit) config
        //
        config.softLimit.forwardSoftLimitEnabled(configAccessor.softLimit.getForwardSoftLimitEnabled());
        config.softLimit.forwardSoftLimit(configAccessor.softLimit.getForwardSoftLimit());
        config.softLimit.reverseSoftLimitEnabled(configAccessor.softLimit.getReverseSoftLimitEnabled());
        config.softLimit.reverseSoftLimit(configAccessor.softLimit.getReverseSoftLimit());

        //
        // Closed-loop (PIDF) config
        //
        config.closedLoop.pidf(configAccessor.closedLoop.getP(), configAccessor.closedLoop.getI(),
                configAccessor.closedLoop.getD(), configAccessor.closedLoop.getFF());

        config.closedLoop.dFilter(configAccessor.closedLoop.getDFilter());
        config.closedLoop.iZone(configAccessor.closedLoop.getIZone());
        config.closedLoop.iMaxAccum(configAccessor.closedLoop.getMaxIAccumulation());
        config.closedLoop.minOutput(configAccessor.closedLoop.getMinOutput());
        config.closedLoop.maxOutput(configAccessor.closedLoop.getMaxOutput());
        config.closedLoop.positionWrappingEnabled(configAccessor.closedLoop.getPositionWrappingEnabled());
        config.closedLoop.positionWrappingMinInput(configAccessor.closedLoop.getPositionWrappingMinInput());
        config.closedLoop.positionWrappingMaxInput(configAccessor.closedLoop.getPositionWrappingMaxInput());
        config.closedLoop.feedbackSensor(configAccessor.closedLoop.getFeedbackSensor());

        //
        // If desired, you can also fill in the "signals" periods/enable flags from the SignalsConfigAccessor
        // For example:
        //
        // config.signals.busVoltagePeriodMs(configAccessor.signals.getBusVoltagePeriodMs());
        // config.signals.busVoltageAlwaysOn(configAccessor.signals.getBusVoltageAlwaysOn());
        // ... etc. for each status frame you care about ...

        // Finally, return the fully populated config
        currentActiveConfig = config;
        currentWorkingConfig = config;
        return config;
    }

    public SparkFlexConfig getCurrentActiveConfig() {
        return currentActiveConfig;
    }

    public SparkFlexConfig getCurrentWorkingConfig() {
        return currentWorkingConfig;
    }


    public void configure(SparkFlexConfig config) {
        configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void configure(){
        if (!(currentWorkingConfig.equals(currentActiveConfig))) {
            configure(currentWorkingConfig);
        }
    }

    // Other methods (e.g. setPIDF, if you want to combine config usage with direct calls)
    public void setPIDF(double p, double i, double d, double f) {
        currentWorkingConfig.closedLoop.pidf(p, i, d, f);
    }

    public void setPID(double p, double i, double d) {
        currentWorkingConfig.closedLoop.pidf(p, i, d, configAccessor.closedLoop.getFF());
    }


    public void setOutputRange(double min, double max) {
        currentWorkingConfig.closedLoop.minOutput(min);
        currentWorkingConfig.closedLoop.maxOutput(max);
    }

    public void setIdleMode(IdleMode mode) {
        currentWorkingConfig.idleMode(mode);
    }

    public void setSmartCurrentLimit(int limit) {
        currentWorkingConfig.smartCurrentLimit(limit);
    }

    public void setPositionConversionFactor(double factor) {
        currentWorkingConfig.encoder.positionConversionFactor(factor);
    }

    public void setVelocityConversionFactor(double factor) {
        currentWorkingConfig.encoder.velocityConversionFactor(factor);
    }

    public void setInverted(boolean inverted) {
        currentWorkingConfig.inverted(inverted);
    }

    public void setPositionWrappingEnabled(boolean enabled) {
        currentWorkingConfig.closedLoop.positionWrappingEnabled(enabled);
    }

    public void setPositionWrappingInputRange(double min, double max) {
        currentWorkingConfig.closedLoop.positionWrappingMinInput(min);
        currentWorkingConfig.closedLoop.positionWrappingMaxInput(max);
    }

    public void setMAXMotionMaxAcceleration(double maxAcceleration) {
        currentWorkingConfig.closedLoop.maxMotion.maxAcceleration(maxAcceleration);
    }


    public void setMAXMotionMaxVelocity(double maxVelocity) {
        currentWorkingConfig.closedLoop.maxMotion.maxVelocity(maxVelocity);
    }

    public void allowedClosedLoopError(double allowedError) {
        currentWorkingConfig.closedLoop.maxMotion.allowedClosedLoopError(allowedError);
    }

    public void positionMode(MAXMotionPositionMode mode) {
        currentWorkingConfig.closedLoop.maxMotion.positionMode(mode);
    }

    


    
}