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
    private PIDController tuskRollerPIDController;
    private PIDController tuskPivotSubsystemPIDController;
    private tuskPositions currentState;
    private boolean isHoldPose;
    private boolean isHoldPoseUpdated;
    private double rollerHoldPose;
    private double targetPower;

    private boolean isRollerHold;
    private boolean isRollerHoldUpdated;

    private double holdPose;

    public Tusks(){
        tuskRollerMotor = new SparkFlex(TuskConstants.kTuskRollerMotorId, MotorType.kBrushless);
        tuskPivotMotor = new SparkMax(TuskConstants.kTuskPivotMotorId, MotorType.kBrushless);
        tuskPivotPIDController = new PIDController(TuskConstants.kPivotP, TuskConstants.kPivotI, TuskConstants.kPivotD);
        tuskRollerPIDController = new PIDController(TuskConstants.kRollerP, TuskConstants.kRollerI, TuskConstants.kRollerD);
        tuskPivotSubsystemPIDController = new PIDController(TuskConstants.kPivotP, TuskConstants.kPivotI, TuskConstants.kPivotD);
        isHoldPose = true;
        isHoldPoseUpdated = false;
        isRollerHold = true;
        isRollerHoldUpdated = false;

        holdPose = 0;
        

        currentState = tuskPositions.HOME;

    }

    public void setRollerPower(double power){
        
        isRollerHold = false;
        isRollerHoldUpdated = false;
        tuskRollerMotor.set(power);


    }

    public void setPivotPower(double power){
        isHoldPose = false;
        isHoldPoseUpdated = false;
        tuskPivotMotor.set(power);
        currentState = tuskPositions.INTERRUPTED;
    }

    public void stopRoller(){
        tuskRollerMotor.set(0);
        lockRollerPosition();
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
        /*    GROUND(TuskConstants.kGroundPosition),
    L3(TuskConstants.kL3Position),
    L4(TuskConstants.kL4Position),
    PROCESSOR(TuskConstants.kProcessorPosition),
    NET(TuskConstants.kNetPosition),
    HOME(TuskConstants.kHomePosition),
    INTERRUPTED(TuskConstants.kInterruptedPosition); */
        if (currentState == tuskPositions.HOME){
            SmartDashboard.putString("Tusk Position", "HOME");
        } else if (currentState == tuskPositions.L3){
            SmartDashboard.putString("Tusk Position", "L3");
        } else if (currentState == tuskPositions.L4){
            SmartDashboard.putString("Tusk Position", "L4");
        } else if (currentState == tuskPositions.PROCESSOR){
            SmartDashboard.putString("Tusk Position", "PROCESSOR");
        } else if (currentState == tuskPositions.NET){
            SmartDashboard.putString("Tusk Position", "NET");
        } else if (currentState == tuskPositions.GROUND){
            SmartDashboard.putString("Tusk Position", "GROUND");
        } else if (currentState == tuskPositions.INTERRUPTED){
            SmartDashboard.putString("Tusk Position", "INTERRUPTED");
        }

        if (!isHoldPoseUpdated){
            holdPose = tuskPivotMotor.getEncoder().getPosition();
            tuskPivotPIDController.setSetpoint(holdPose);
            isHoldPoseUpdated = true;
        }

        if(!isRollerHoldUpdated){
            rollerHoldPose = tuskRollerMotor.getEncoder().getPosition();
            tuskRollerPIDController.setSetpoint(rollerHoldPose);
            isRollerHoldUpdated = true;
        }

        if (isHoldPose && isHoldPoseUpdated){
            tuskPivotMotor.set(tuskPivotPIDController.calculate(tuskPivotMotor.getEncoder().getPosition()));
        }

        if (isRollerHold && isRollerHoldUpdated){
            tuskRollerMotor.set(tuskRollerPIDController.calculate(tuskRollerMotor.getEncoder().getPosition()));
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

    public void lockRollerPosition() {
        isRollerHoldUpdated = false;
        isRollerHold = true;
    }

    public void toggleHoldPoseMode(){
        isHoldPose = !isHoldPose;
    }

    public void setHoldPoseMode(boolean holdPose){
        isHoldPose = holdPose;
    }

    public void setRollerHoldPoseMode(boolean holdPose){
        isRollerHold = holdPose;
    }

    public void setRollerHoldPose(double position){
        isRollerHold = true;
        isRollerHoldUpdated = true;
        tuskRollerMotor.getEncoder().setPosition(position);
    }

    public void resetTusksPivot(){
        tuskPivotMotor.getEncoder().setPosition(0);
    }


    
}
