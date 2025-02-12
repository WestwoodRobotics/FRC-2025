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

    private double xTolerance = 0.1;
    private double yTolerance = 0.1;
    private double angleTolerance = 0.1;

    private PIDController xController;
    private PIDController yController;
    private PIDController angleController;

    public GoToFieldPose(SwerveDrive swerve, double targetX, double targetY, double targetAngle){
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetAngle = targetAngle;
        this.swerve = swerve;

        this.xTolerance = 0.1;
        this.yTolerance = 0.1;
        this.angleTolerance = 0.1;

        xController = new PIDController(0.1, 0, 0);
        yController = new PIDController(0.1, 0, 0);
        angleController = new PIDController(0.1, 0, 0);

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





        xController = new PIDController(0.1, 0, 0);
        yController = new PIDController(0.1, 0, 0);
        angleController = new PIDController(0.1, 0, 0);

        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        currentX = swerve.getPose().getX();
        currentY = swerve.getPose().getY();
        currentAngle = swerve.getPose().getRotation().getDegrees();
    }

    @Override
    public void execute(){
        double xOutput = xController.calculate(currentX, targetX);
        double yOutput = yController.calculate(currentY, targetY);
        double angleOutput = angleController.calculate(currentAngle, targetAngle);

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
