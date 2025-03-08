package frc.robot.subsystems.tusks;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TuskConstants;
import frc.robot.subsystems.utils.tusks.tuskPositions;

public class Tusks extends SubsystemBase{

    private SparkFlex tuskRollerMotor;
    private SparkMax tuskPivotMotor;
    
    private PIDController tuskPivotPIDController;
    private PIDController tuskPivotSubsystemPIDController;
    private tuskPositions currentState;
    private boolean isHoldPose;
    private boolean isHoldPoseUpdated;
    private double holdPose;

    public Tusks(){
        tuskRollerMotor = new SparkFlex(TuskConstants.kTuskRollerMotorId, MotorType.kBrushless);
        tuskPivotMotor = new SparkMax(TuskConstants.kTuskPivotMotorId, MotorType.kBrushless);
        tuskPivotPIDController = new PIDController(TuskConstants.kPivotP, TuskConstants.kPivotI, TuskConstants.kPivotD);
        tuskPivotSubsystemPIDController = new PIDController(TuskConstants.kPivotP, TuskConstants.kPivotI, TuskConstants.kPivotD);
        isHoldPose = true;
        isHoldPoseUpdated = false;
        holdPose = 0;
        

        currentState = tuskPositions.IN;

    }

    public void setRollerPower(double power){
        tuskRollerMotor.set(power);
    }

    public void setPivotPower(double power){
        isHoldPose = false;
        isHoldPoseUpdated = false;
        tuskPivotMotor.set(power);
    }

    public void stopRoller(){
        tuskRollerMotor.set(0);
    }

    public void stopPivot(){
        tuskPivotMotor.set(0);
        lockPosition();
    }

    public void stopAll(){
        stopRoller();
        stopPivot();
        lockPosition();
    }

    public PIDController getPIDController(){
        return tuskPivotPIDController;
    }

    public double getPivotPosition(){
        return tuskPivotMotor.getEncoder().getPosition();
    }
    
    public void setTargetPosition(double position) {
        isHoldPose = true;
        isHoldPoseUpdated = true;
        tuskPivotPIDController.setSetpoint(position);
    }



    @Override
    public void periodic(){
        //System.out.println("Pivot Position: " + tuskPivotMotor.getEncoder().getPosition());
        if (currentState == tuskPositions.IN){
            SmartDashboard.putString("Tusk Position", "IN");
        } else if (currentState == tuskPositions.OUT){
            SmartDashboard.putString("Tusk Position", "OUT");
        }
        if (!isHoldPoseUpdated){
            holdPose = tuskPivotMotor.getEncoder().getPosition();
            tuskPivotPIDController.setSetpoint(holdPose);
            isHoldPoseUpdated = true;
        }

        if (isHoldPose && isHoldPoseUpdated){
            tuskPivotMotor.set(tuskPivotPIDController.calculate(tuskPivotMotor.getEncoder().getPosition()));
            
        }

        SmartDashboard.putBoolean("isHoldPose", isHoldPoseUpdated);
        SmartDashboard.putBoolean("isHoldPoseUpdated", isHoldPoseUpdated);
        SmartDashboard.putNumber("holdPose", holdPose);
        SmartDashboard.putNumber("Tusk Pivot Encoder", tuskPivotMotor.getEncoder().getPosition());


    }

    public tuskPositions getCurrentState(){
        return currentState;
    }

    public void setCurrentState(tuskPositions state){
        currentState = state;
    }

    public boolean getHoldPoseMode(){
        return isHoldPose;
    }

    public void lockPosition() {
        isHoldPoseUpdated = false;
        isHoldPose = true;
    }

    public void toggleHoldPoseMode(){
        isHoldPose = !isHoldPose;
    }

    public void setHoldPoseMode(boolean holdPose){
        isHoldPose = holdPose;
    }


    
}
