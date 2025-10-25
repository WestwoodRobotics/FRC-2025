package frc.robot.subsystems.tusks;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import frc.robot.Constants.TuskConstants;
import frc.robot.subsystems.utils.elevator.elevatorPositions;
import frc.robot.subsystems.utils.tusks.tuskPositions;

public class Tusks extends SubsystemBase{

    private SparkFlex tuskRollerMotor;
    private SparkMax tuskPivotMotor;

    private TrapezoidProfile profile;
    private Timer pivotProfileTimer;
    
    private PIDController tuskPivotPIDController;
    private PIDController tuskRollerPIDController;
    private PIDController tuskPivotSubsystemPIDController;

    private double pivotEncoderOffset = 0;

    private State startState;
    private State currentState;
    private tuskPositions currentPosition;

    private double tuskPosSetPoint;
    private double tuskPivotPower;
    private boolean tuskManual;

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
        isHoldPose = true;
        isHoldPoseUpdated = false;
        isRollerHold = true;
        isRollerHoldUpdated = false;

        holdPose = 0;
        
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(75, 75));
        pivotProfileTimer = new Timer();

        tuskPosSetPoint = 0;
        tuskPivotPower = 0;
        tuskManual = true;
        currentPosition = tuskPositions.HOME;

        currentState = new State(tuskPositions.HOME.getPosition(), 0);
        startState = new State(tuskPositions.HOME.getPosition(), 0);

    }

    public void setRollerPower(double power){
        
        isRollerHold = false;
        isRollerHoldUpdated = false;
        tuskRollerMotor.set(power);


    }

    public void setPivotPower(double power){
        // isHoldPose = false;
        // isHoldPoseUpdated = false;
        // tuskPivotMotor.set(power);
        // currentState = tuskPositions.INTERRUPTED;
        tuskPivotPower = power;
        tuskManual = true;
        currentPosition = tuskPositions.INTERRUPTED;
    }

    public void setPivotPosition(double position){
        if(tuskManual){
            startState = new State(tuskPivotMotor.getEncoder().getPosition(), tuskPivotMotor.getEncoder().getVelocity()/60);
        } else {
            startState = new State(currentState.position, currentState.velocity);
        }
        tuskPosSetPoint = position;
        tuskManual = false;
        pivotProfileTimer.restart();
    }

    public void setPivotPosition(tuskPositions position) {
        setPivotPosition(position.getPosition());
    }

    public void stopRoller(){
        tuskRollerMotor.set(0);
        lockRollerPosition();
    }

    public void stopPivot(){
        tuskPivotPower = 0;
        tuskManual = true;
        //lockPosition();
    }

    // public void stopAll(){
    //     stopRoller();
    //     stopPivot();
    //     lockPosition();
    // }
    


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

        if(tuskManual){
            tuskPivotMotor.set(tuskPivotPower);
        } else {
            currentState = profile.calculate(
                pivotProfileTimer.get(),
                startState,
                new State(tuskPosSetPoint, 0)
            );
            SmartDashboard.putNumber("Tusk Pivot current", getPivotPosition());
            SmartDashboard.putNumber("Tusk Pivot calc", currentState.position);
            SmartDashboard.putNumber("Tusk Pivot vel", tuskPivotMotor.getEncoder().getVelocity() / 60);
            SmartDashboard.putNumber("Tusk Pivot vel calc", currentState.velocity);
            tuskPivotMotor.set(tuskPivotPIDController.calculate(tuskPivotMotor.getEncoder().getPosition(), currentState.position));
        }

        // if (currentPosition == tuskPositions.HOME){
        //     SmartDashboard.putString("Tusk Position", "HOME");
        // } else if (currentPosition == tuskPositions.L3){
        //     SmartDashboard.putString("Tusk Position", "L3");
        // } else if (currentPosition == tuskPositions.L4){
        //     SmartDashboard.putString("Tusk Position", "L4");
        // } else if (currentPosition == tuskPositions.PROCESSOR){
        //     SmartDashboard.putString("Tusk Position", "PROCESSOR");
        // } else if (currentPosition == tuskPositions.GROUND){
        //     SmartDashboard.putString("Tusk Position", "GROUND");
        // } else if (currentPosition == tuskPositions.INTERRUPTED){
        //     SmartDashboard.putString("Tusk Position", "INTERRUPTED");
        // }

        // if (!isHoldPoseUpdated){
        //     holdPose = tuskPivotMotor.getEncoder().getPosition();
        //     tuskPivotPIDController.setSetpoint(holdPose);
        //     isHoldPoseUpdated = true;
        // }

        if(!isRollerHoldUpdated){
            rollerHoldPose = tuskRollerMotor.getEncoder().getPosition();
            tuskRollerPIDController.setSetpoint(rollerHoldPose+0.15);
            isRollerHoldUpdated = true;
        }

        // if (isHoldPose && isHoldPoseUpdated){
        //     tuskPivotMotor.set(tuskPivotPIDController.calculate(tuskPivotMotor.getEncoder().getPosition()));
        // }

        if (isRollerHold && isRollerHoldUpdated){
            //tuskRollerMotor.set(tuskRollerPIDController.calculate(tuskRollerMotor.getEncoder().getPosition()));

            tuskRollerMotor.set(tuskRollerPIDController.calculate(tuskRollerMotor.getEncoder().getPosition()));
        }



        // SmartDashboard.putBoolean("isHoldPose", isHoldPoseUpdated);
        // SmartDashboard.putBoolean("isHoldPoseUpdated", isHoldPoseUpdated);
        // SmartDashboard.putNumber("holdPose", holdPose);
        SmartDashboard.putNumber("Tusk Pivot Encoder", tuskPivotMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Tusk Roller Encoder",tuskRollerMotor.getEncoder().getPosition());
        


    }

    public PIDController getPivotPIDController(){
        return tuskPivotPIDController;
    }

    public double getPivotPosition(){
        return tuskPivotMotor.getEncoder().getPosition() + pivotEncoderOffset;
    }

    public double getPivotCurrent(){
        return tuskPivotMotor.getOutputCurrent();
    }

    public void setPivotEncoderOffset(double offset){
        pivotEncoderOffset = offset;
    }

    public void setPivotEncoderPosition(double position){
        pivotEncoderOffset = position - tuskPivotMotor.getEncoder().getPosition();
    }

    public void setPivotPostionEnum(tuskPositions position){
        currentPosition = position;
    }

    public tuskPositions getPivotPositionEnum(){
        return currentPosition;
    }

    // public tuskPositions getCurrentState(){
    //     return currentState;
    // }

    // public void setCurrentState(tuskPositions state){
    //     currentState = state;
    // }

    // public boolean getHoldPoseMode(){
    //     return isHoldPose;
    // }

    // public void lockPosition() {
    //     isHoldPoseUpdated = false;
    //     isHoldPose = true;
    // }

    public void lockRollerPosition() {
        isRollerHoldUpdated = false;
        isRollerHold = true;
    }

    // public void toggleHoldPoseMode(){
    //     isHoldPose = !isHoldPose;
    // }

    // public void setHoldPoseMode(boolean holdPose){
    //     isHoldPose = holdPose;
    // }

    public void setRollerHoldPoseMode(boolean holdPose){
        isRollerHold = holdPose;
    }

    public void setRollerHoldPose(double position){
        isRollerHold = true;
        isRollerHoldUpdated = true;
        tuskRollerMotor.getEncoder().setPosition(position);
    }

    // public void resetTusksPivot(){
    //     tuskPivotMotor.getEncoder().setPosition(0);
    // }


    
}
