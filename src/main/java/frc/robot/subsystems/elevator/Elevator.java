package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.utils.elevator.elevatorPositions;
import frc.robot.sensors.DIO.LimitSwitch;

public class Elevator extends SubsystemBase {


    private SparkFlex elevatorMotor1;
    private SparkFlex elevatorMotor2;
    private PIDController elevatorPIDController;
    private double elevatorEncoderOffset = 0;
    //private LimitSwitch elevatorTopLimitSwitch = new LimitSwitch(0);
    private LimitSwitch elevatorBottomLimitSwitch = new LimitSwitch(0);
    private TrapezoidProfile profile;
    private double elevatorPosSetpoint;
    private double elevatorPower;
    private Timer elevatorProfileTimer;
    private boolean elevatorManual;
    private State startState;
    private State currentState;
    

    public Elevator(int elevatorMotor1Port, int elevatorMotor2Port) {
        this.elevatorMotor1 = new SparkFlex(elevatorMotor1Port, MotorType.kBrushless);
        this.elevatorMotor2 = new SparkFlex(elevatorMotor2Port, MotorType.kBrushless);
        elevatorMotor1.configure(Configs.Elevator.elevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotor2.configure(Configs.Elevator.elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorPIDController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(50, 50));
        elevatorPosSetpoint = 0;
        elevatorPower = 0;
        elevatorProfileTimer = new Timer();
        elevatorManual = true;
        startState = new State(0, 0);
        currentState = new State(0, 0);
    }

    public void setElevatorSpeed(double speed) {
        elevatorPower = speed;
        elevatorManual = true;
    }

    public void setElevatorPosition(double position) {
        if(elevatorManual) {
            startState = new State(elevatorMotor1.getEncoder().getPosition(), elevatorMotor1.getEncoder().getVelocity()/60);
        }
        else {
            startState = new State(currentState.position, currentState.velocity);
        }
        elevatorPosSetpoint = position;
        elevatorManual = false;
        elevatorProfileTimer.restart();
    }

    public void setElevatorPosition(elevatorPositions position) {
        setElevatorPosition(position.getPosition());
    }

    public void stopElevator() {
        elevatorPower = 0;
        elevatorManual = true;
    }
    @Override
    public void periodic(){
        //System.out.println(elevatorMotor1.getEncoder().getPosition());
        //System.out.println(elevatorMotor1.getOutputCurrent());
        // SmartDashboard.putBoolean("Detected Limit Switch", elevatorBottomLimitSwitch.isTriggered());
        // SmartDashboard.putNumber("Elevator 1 Encoder", this.getElevatorPosition());
        if(elevatorManual) {
            //System.out.println("Manual");
            elevatorMotor1.set(elevatorPower);
        }
        else {
            currentState = profile.calculate(
                elevatorProfileTimer.get(),
                startState,
                new State(elevatorPosSetpoint, 0)
            );
            SmartDashboard.putNumber("Elevator current", getElevatorPosition());
            SmartDashboard.putNumber("Elevator calc", currentState.position);
            SmartDashboard.putNumber("Elevator vel", elevatorMotor1.getEncoder().getVelocity()/60);
            SmartDashboard.putNumber("Elevator vel calc", currentState.velocity);
            elevatorMotor1.set(elevatorPIDController.calculate(getElevatorPosition(), currentState.position));
        }
        SmartDashboard.putNumber("Elevator (ID 50) Encoder Value", elevatorMotor1.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator (ID 34) Encoder Value", elevatorMotor1.getEncoder().getPosition());


    }

    public PIDController getPIDController() {
        return elevatorPIDController;
    }

    public double getElevatorPosition() {
        return elevatorMotor1.getEncoder().getPosition() + elevatorEncoderOffset;
        
    }

    public double getElevatorCurrent() {
        return elevatorMotor1.getOutputCurrent();
    }

    public double getElevatorSecondCurrent() {
        return elevatorMotor2.getOutputCurrent();
    }

    public void setElevatorEncoderOffset(double offset) {
        elevatorEncoderOffset = offset;
    }

    public void setElevatorEncoderPosition(double position) {
        elevatorEncoderOffset = position - elevatorMotor1.getEncoder().getPosition();
    }

    // public LimitSwitch getElevatorTopLimitSwitch() {
    //     return elevatorTopLimitSwitch;
    // }
    
    public LimitSwitch getElevatorBottomLimitSwitch() {
        return elevatorBottomLimitSwitch;
    }
    
}
