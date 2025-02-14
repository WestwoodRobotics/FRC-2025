package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

public class GoToFieldPose extends Command{

    private SwerveDrive swerve;

    private double targetX;
    private double targetY;
    private double targetAngle;

    private double currentX;
    private double currentY;
    private double currentAngle;

    private double xTolerance = 0.03;
    private double yTolerance = 0.03;
    private double angleTolerance = 0.01;

    private PIDController xController;
    private PIDController yController;
    private PIDController angleController;
    //11.8 
    public GoToFieldPose(SwerveDrive swerve, double targetX, double targetY, double targetAngle){
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetAngle = targetAngle;
        this.swerve = swerve;

        this.xTolerance = 0.02;
        this.yTolerance = 0.02;
        this.angleTolerance = 0.02;

        xController = new PIDController(1.5, 0, 0);
        yController = new PIDController(1.5, 0, 0);
        angleController = new PIDController(1.5, 0, 0);

        addRequirements(swerve);
    }

    public GoToFieldPose(SwerveDrive swerve, double targetX, double targetY, double targetAngle, double xTolerance, double yTolerance, double angleTolerance){
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetAngle = targetAngle;
        this.swerve = swerve;

        this.xTolerance = xTolerance;
        this.yTolerance = yTolerance;
        this.angleTolerance = angleTolerance;





        xController = new PIDController(1.5, 0, 0);
        yController = new PIDController(1.5, 0, 0);
        angleController = new PIDController(1.5, 0, 0);

        addRequirements(swerve);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        currentX = swerve.getPose().getX();
        currentY = swerve.getPose().getY();
        currentAngle = swerve.getPose().getRotation().getRadians();

        double xOutput = xController.calculate(currentX, targetX);
        double yOutput = yController.calculate(currentY, targetY);
        double angleOutput = angleController.calculate(currentAngle, targetAngle);
        xOutput = Math.min(1, Math.max(-1, xOutput));
        yOutput = Math.min(1, Math.max(-1, yOutput));
        angleOutput = Math.min(1, Math.max(-1, angleOutput));
        swerve.drive(xOutput, yOutput, angleOutput, true);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(currentX - targetX) < xTolerance && Math.abs(currentY - targetY) < yTolerance && Math.abs(currentAngle - targetAngle) < angleTolerance;
    }

    @Override
    public void end(boolean interrupted){
        swerve.drive(0, 0, 0, true);
    }

    




    
}
