package frc.robot.commands.swerve;

import java.util.HashMap;
import java.util.Optional;

import javax.swing.text.html.HTML.Tag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.PhotonVisionCameras;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.utils.swerve.ReefAlignSide;

public class GoToNearestScoringPoseCommand extends Command{

    private SwerveDrive swerve;

    private double targetX;
    private double targetY;
    private double targetAngle;

    private double currentX;
    private double currentY;
    private double currentAngle;

    private final double xTolerance = 0.03;
    private final double yTolerance = 0.03;
    private final double angleTolerance = 0.01;

    private PIDController xController;
    private PIDController yController;
    private PIDController angleController;

    private boolean finished;


    private int visibleFiducialID = 0;
    private AprilTagFieldLayout layout;

    // private HashMap<Integer, ReefLeftPoses> reefLeftPoseToFiducialID = new HashMap<Integer, ReefLeftPoses>();
    // private HashMap<Integer, ReefRightPoses> reefRightPoseToFiducialID = new HashMap<Integer, ReefRightPoses>();
    private ReefAlignSide side;

    private final Transform2d left_transform = new Transform2d(
        new Translation2d(0.42, -0.165),
        new Rotation2d(Math.PI)
    );
    private final Transform2d right_transform = new Transform2d(
        new Translation2d(0.42, 0.165),
        new Rotation2d(Math.PI)
    );
    //11.8 
    public GoToNearestScoringPoseCommand(SwerveDrive swerve, AprilTagFieldLayout layout, ReefAlignSide side){ 

        this.swerve = swerve;
        this.layout = layout;
        this.side = side;
        this.finished = false;

        xController = new PIDController(1.5, 0.0, 0.00);
        xController.setIntegratorRange(-0.2, 0.2);
        yController = new PIDController(1.5, 0.0, 0.00);
        yController.setIntegratorRange(-0.2, 0.2);
        angleController = new PIDController(1
        , 0.0, 0.00);
        angleController.setIntegratorRange(-0.2, 0.2);

        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        finished = false;
        try{
            swerve.m_cameras.hasTarget();
            swerve.m_cameras.getFiducialId();
        }
        catch (Exception e){
            System.out.println("Crashed!");
            finished = true;
            return;
        }

        if (!swerve.m_cameras.hasTarget()){
            System.out.println("No target!");
            finished = true;
            return;
        }

        visibleFiducialID = swerve.m_cameras.getFiducialId();
        Optional<Pose3d> maybeTagPose = layout.getTagPose(visibleFiducialID);
        if(maybeTagPose.isEmpty()) {
            System.out.println("No pose!");
            finished = true;
            return;
        }

        Pose2d tagPose = maybeTagPose.get().toPose2d();

        Pose2d targetPose = tagPose.transformBy((side.equals(ReefAlignSide.LEFT)) ? left_transform : right_transform);
        

        // if (side == ReefAlignSide.LEFT){
        //     for (int i = 0; i < ReefLeftPoses.values().length; i++){
        //         reefLeftPoseToFiducialID.put(ReefID.values()[i].getId(), ReefLeftPoses.values()[i]);
        //     }
        // }
        // else{
        //     for (int i = 0; i < ReefRightPoses.values().length; i++){
        //         reefRightPoseToFiducialID.put(ReefID.values()[i].getId(), ReefRightPoses.values()[i]);
        //     }
        // }

        // targetPose = (this.side == ReefAlignSide.LEFT) 
        //                 ? reefLeftPoseToFiducialID.get(visibleFiducialID).getPose2d() 
        //                 : reefRightPoseToFiducialID.get(visibleFiducialID).getPose2d();

        targetX = targetPose.getTranslation().getX();
        targetY = targetPose.getTranslation().getY();
        targetAngle = targetPose.getRotation().getRadians();





        // for (ReefPose reefPose : ReefPose.values()){
        //     for (ReefID reefID : ReefID.values()){
        //         reefPoseToFiducialID.put(reefPose, reefID);
        //     }
        // }




    }

    @Override
    public void execute(){
        currentX = swerve.getPose().getX();
        currentY = swerve.getPose().getY();
        currentAngle = swerve.getPose().getRotation().getRadians();

        double xOutput = xController.calculate(currentX, targetX);
        double yOutput = yController.calculate(currentY, targetY);

        double speedBound = Math.min(0.4, 0.5*Math.log(
            0.5*(Math.pow(currentX-targetX, 2) +
            Math.pow(currentY - targetY, 2)) + 1)+0.1
        );
        
        double angleOutput = angleController.calculate(new Rotation2d(currentAngle).minus(new Rotation2d(targetAngle)).getRadians(), 0);
        xOutput = Math.min(speedBound, Math.max(-speedBound, xOutput));
        yOutput = Math.min(speedBound, Math.max(-speedBound, yOutput));
        angleOutput = Math.min(0.3, Math.max(-0.3, angleOutput));
        swerve.drive(xOutput, yOutput, angleOutput, true);
        SmartDashboard.putNumber("Target X", targetX);
        SmartDashboard.putNumber("Target Y", targetY);
        SmartDashboard.putNumber("Target Theta", new Rotation2d(currentAngle).minus(new Rotation2d(targetAngle)).getRadians());
        SmartDashboard.putNumber("X Move", xOutput);
        SmartDashboard.putNumber("Y Move", yOutput);
        SmartDashboard.putNumber("Theta Move", angleOutput);
        // System.out.println("TargetX: " + targetX + " | " + "TargetY: " + targetY + " | " + "TargetAngle: " + targetAngle);
    }

    @Override
    public boolean isFinished(){
        return (
            Math.abs(currentX - targetX) < xTolerance &&
            Math.abs(currentY - targetY) < yTolerance &&
            Math.abs(currentAngle - targetAngle) < angleTolerance
        ) || finished;
    }


    @Override
    public void end(boolean interrupted){
        System.out.println("Done!");
        swerve.drive(0, 0, 0, true);
    }

    




    
}
