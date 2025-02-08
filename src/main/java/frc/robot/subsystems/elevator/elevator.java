package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.utils.elevator.elevatorPositions;

public class elevator extends SubsystemBase {


    private SparkFlex elevatorMotor1;
    private SparkFlex elevatorMotor2;
    private PIDController elevatorPIDController;
    
    

    public elevator(int elevatorMotor1Port, int elevatorMotor2Port) {
        this.elevatorMotor1 = new SparkFlex(elevatorMotor1Port, MotorType.kBrushless);
        this.elevatorMotor2 = new SparkFlex(elevatorMotor2Port, MotorType.kBrushless);
        elevatorMotor1.configure(Configs.Elevator.elevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotor2.configure(Configs.Elevator.elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorPIDController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    }

    public void setElevatorSpeed(double speed) {
        elevatorMotor1.set(speed);   
    }

    public void setElevatorPosition(double position) {
        elevatorMotor1.getClosedLoopController().setReference(position, ControlType.kPosition);
    }

    public void setElevatorPosition(elevatorPositions position) {
        elevatorMotor1.getClosedLoopController().setReference(position.getPosition(), ControlType.kPosition);
    }

    public void stopElevator() {
        elevatorMotor1.set(0);
    }

    public void periodic(){
        // System.out.println(elevatorMotor1.getEncoder().getPosition());
    }

    public PIDController getPIDController() {
        return elevatorPIDController;
    }

    public double getElevatorPosition() {
        return elevatorMotor1.getEncoder().getPosition();
    }


}
