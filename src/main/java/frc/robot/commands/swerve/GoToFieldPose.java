package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.PhotonVisionCameras;
import frc.robot.subsystems.swerve.SwerveDrive;

public class GoToFieldPose extends Command{

    private enum ReefPose{
        BLUE_18(new Pose2d(new Translation2d(3.7, 4.0), new Rotation2d(0.0))),
        BLUE_19(new Pose2d(new Translation2d(4.0, 4.75), new Rotation2d(-60.0))),
        BLUE_20(new Pose2d(new Translation2d(4.9, 4.75), new Rotation2d(-120.0))),
        BLUE_21(new Pose2d(new Translation2d(5.3, 4.0), new Rotation2d(-180.0))),
        BLUE_22(new Pose2d(new Translation2d(4.9, 3.3), new Rotation2d(120.0))),
        BLUE_17(new Pose2d(new Translation2d(4.1, 3.3), new Rotation2d(60.0))),
        RED_10(new Pose2d(new Translation2d(12.2, 4.19), new Rotation2d(0.0))),
        RED_9(new Pose2d(new Translation2d(12.6, 4.8), new Rotation2d(-60.0))),
        RED_8(new Pose2d(new Translation2d(13.5, 4.8), new Rotation2d(-120.0))),
        RED_7(new Pose2d(new Translation2d(13.9, 4.1), new Rotation2d(-180.0))),
        RED_6(new Pose2d(new Translation2d(13.5, 3.4), new Rotation2d(120.0))),
        RED_11(new Pose2d(new Translation2d(12.6, 3.4), new Rotation2d(60.0)));

        private Pose2d pose2d;
        
        ReefPose(Pose2d pose2d) {
            this.pose2d = pose2d;
        }

        public Pose2d getPose2d() {
            return pose2d;
        }

        
    }

    /*
     * BLUE REEF CLOCKWISE STARTING AT 18 
     * 18:
     * x: 3.7
     * y: 4.0
     * angle: 0
     * 
     * 19:
     * x: 4.0
     * y: 4.75
     * angle: -60
     * 
     * 20:
     * x: 4.9
     * y: 4.75
     * angle: -120
     * 
     * 21:
     * x: 5.3
     * y: 4.0
     * angle: -180
     * 
     * 22:
     * x: 4.9
     * y: 3.3
     * angle: 120
     * 
     * 17:
     * x: 4.1
     * y: 3.3
     * angle: 60
     * 
     */

    /*
     * RED REEF CLOCKWISE STARTING AT 10
     * 10:
     * x: 12.2
     * y: 4.19
     * angle: 0
     * 
     * 9:
     * x: 12.6
     * y: 4.8
     * angle: -60
     * 
     * 8:
     * x:13.5
     * y: 4.8
     * angle: -120
     * 
     * 7:
     * x: 13.9
     * y: 4.1
     * angle: -180
     * 
     * 6:
     * x: 13.5
     * y: 3.4
     * angle: 120
     * 
     * 11:
     * x: 12.6
     * y: 3.4
     * angle: 60
     *
     */


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

    private boolean isFinished;

    private PhotonVisionCameras m_cameras;
    //11.8 
    public GoToFieldPose(SwerveDrive swerve, double targetX, double targetY, double targetAngle){
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetAngle = targetAngle;
        this.swerve = swerve;
        this.m_cameras = swerve.m_cameras;

        this.xTolerance = 0.01;
        this.yTolerance = 0.01;
        this.angleTolerance = 0.01;

        xController = new PIDController(2, 0.0, 0.005);
        xController.setIntegratorRange(-0.2, 0.2);
        yController = new PIDController(2, 0.0, 0.005);
        yController.setIntegratorRange(-0.2, 0.2);
        angleController = new PIDController(2, 0.0, 0.005);
        angleController.setIntegratorRange(-0.2, 0.2);





        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        if (!swerve.m_cameras.hasTarget()){
            isFinished = true;
        }
    }

    @Override
    public void execute(){
        currentX = swerve.getPose().getX();
        currentY = swerve.getPose().getY();
        currentAngle = swerve.getPose().getRotation().getRadians();

        double xOutput = xController.calculate(currentX, targetX);
        double yOutput = yController.calculate(currentY, targetY);
        double angleOutput = angleController.calculate(currentAngle, targetAngle);
        xOutput = Math.min(0.3, Math.max(-0.3, xOutput));
        yOutput = Math.min(0.3, Math.max(-0.3, yOutput));
        angleOutput = Math.min(0.3, Math.max(-0.3, angleOutput));
        swerve.drive(xOutput, yOutput, angleOutput, true);
    }

    @Override
    public boolean isFinished(){
        return (Math.abs(currentX - targetX) < xTolerance && Math.abs(currentY - targetY) < yTolerance && Math.abs(currentAngle - targetAngle) < angleTolerance) || isFinished;
    }

    @Override
    public void end(boolean interrupted){
        swerve.drive(0, 0, 0, true);
    }

    




    
}