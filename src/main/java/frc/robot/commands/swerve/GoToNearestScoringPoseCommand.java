package frc.robot.commands.swerve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import javax.swing.text.html.HTML.Tag;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
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

    private Pose2d targetPose;

    private final double xTolerance = 0.01;
    private final double yTolerance = 0.03;
    private final double angleTolerance = 0.01;

    private Timer profileTimer;

    private PIDController xController;
    private PIDController yController;
    private PIDController angleController;
    private Trajectory trajectory;

    private boolean finished;
    private int terminateFinish;


    private int visibleFiducialID = 0;
    private AprilTagFieldLayout layout;
    private double topSpeed;

    // private HashMap<Integer, ReefLeftPoses> reefLeftPoseToFiducialID = new HashMap<Integer, ReefLeftPoses>();
    // private HashMap<Integer, ReefRightPoses> reefRightPoseToFiducialID = new HashMap<Integer, ReefRightPoses>();
    private ReefAlignSide side;

    private static final double LEFT_DIST = -0.165;
    private static final double RIGHT_DIST = 0.16;

    private final Transform2d center_far_left_transform = new Transform2d(
        new Translation2d(0.75, LEFT_DIST),
        new Rotation2d(Math.PI)
    );

    private final Transform2d center_far_right_transform = new Transform2d(
        new Translation2d(0.75, RIGHT_DIST
        ),
        new Rotation2d(Math.PI)
    );

    private final Transform2d left_transform = new Transform2d(
        new Translation2d(0.5, LEFT_DIST),
        new Rotation2d(Math.PI)
    );
    private final Transform2d right_transform = new Transform2d(
        new Translation2d(0.5, RIGHT_DIST),
        new Rotation2d(Math.PI)
    );
    private final Transform2d true_center_transform = new Transform2d(

        new Translation2d(0.5, 0),
        new Rotation2d(Math.PI)
    );

    private final Transform2d algae_reef_pickup_transform_initital = new Transform2d(
        new Translation2d(0.73, RIGHT_DIST),
        new Rotation2d(Math.PI)
    );

    private final Transform2d algae_reef_pickup_transform_final = new Transform2d(
        new Translation2d(0.68, RIGHT_DIST-0.1),
        new Rotation2d(Math.PI)
    );

    public GoToNearestScoringPoseCommand(SwerveDrive swerve, AprilTagFieldLayout layout, ReefAlignSide side, double top_speed){ 
        this.swerve = swerve;
        this.layout = layout;
        this.side = side;
        this.finished = false;

        xController = new PIDController(1.2, 0, 0.01);
        xController.setIntegratorRange(-0.2, 0.2);
        yController = new PIDController(1.2, 0, 0.01);
        yController.setIntegratorRange(-0.2, 0.2);
        angleController = new PIDController(1.2, 0.0, 0.0003);
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setIntegratorRange(-0.2, 0.2);
        profileTimer = new Timer();
        trajectory = new Trajectory();
        terminateFinish = 0;
        targetPose = new Pose2d();

        this.topSpeed = top_speed;

        addRequirements(swerve);
    }

    public GoToNearestScoringPoseCommand(SwerveDrive swerve, AprilTagFieldLayout layout, ReefAlignSide side) {
        this(swerve, layout, side, 1.5);
    }

    public Trajectory generateTrajectory(Pose2d tagPose, Pose2d targetPose) {
        double x_vel = swerve.getOdometry().getXVel();
        double y_vel = swerve.getOdometry().getYVel();
        double x_pos = swerve.getOdometry().getX();
        double y_pos = swerve.getOdometry().getY();
        
        Pose2d first_waypoint;
        ArrayList<Translation2d> waypointList = new ArrayList<Translation2d>();
        if (side == ReefAlignSide.LEFT){
            first_waypoint = tagPose.transformBy(center_far_left_transform);
        }
        else if (side == ReefAlignSide.RIGHT){
            first_waypoint = tagPose.transformBy(center_far_right_transform);
        }
        else if (side == ReefAlignSide.CENTER){
            first_waypoint = tagPose.transformBy(true_center_transform);
        }
        else if (side == ReefAlignSide.ALGAE_SCORE){
            first_waypoint = tagPose.transformBy(algae_reef_pickup_transform_initital);
        }
        else {
            first_waypoint = tagPose.transformBy(true_center_transform);
        }
        waypointList.add(first_waypoint.getTranslation());

        Rotation2d start_rotation;
        if(Math.pow(x_vel, 2) + Math.pow(y_vel, 2) < 0.01) {
            start_rotation = new Rotation2d(
                Math.atan2(
                    first_waypoint.getY() - y_pos,
                    first_waypoint.getX() - x_pos
                )
            );
        }
        else {
            start_rotation = new Rotation2d(
                Math.atan2(
                    y_vel,
                    x_vel
                )
            );
        }

        Pose2d startPose = new Pose2d(
            x_pos+x_vel*0.1,
            y_pos+y_vel*0.1,
            start_rotation
        );

        double accelerationLimit = 2;
        boolean is_reversed = Math.abs(start_rotation.minus(tagPose.getRotation()).getRadians()) < Math.PI/2;
        double max_vel = 2;
        if(is_reversed) {
            max_vel = 1;
        }

        return TrajectoryGenerator.generateTrajectory(
            startPose,
            waypointList,
            targetPose,
            new TrajectoryConfig(max_vel, accelerationLimit).setStartVelocity(
                Math.sqrt(x_vel*x_vel+y_vel*y_vel)*(is_reversed ? -1: 1)
            ).setEndVelocity(0).setReversed(is_reversed)
        );
    }

    @Override
    public void initialize(){
        finished = false;
        this.xController.reset();
        this.yController.reset();
        this.angleController.reset();

        visibleFiducialID = this.closestAprilTag(swerve.getPose().getX(), swerve.getPose().getY());

        Optional<Pose3d> maybeTagPose = layout.getTagPose(visibleFiducialID);
        
        if(maybeTagPose.isEmpty()) {
            System.out.println("No pose!");
            finished = true;
            return;
        }
        Pose2d tagPose = maybeTagPose.get().toPose2d();
        
        //Pose2d targetPose = tagPose.transformBy((side.equals(ReefAlignSide.LEFT)) ? left_transform : right_transform);
        targetPose = new Pose2d();
        
        if (side.equals(ReefAlignSide.LEFT)){
            targetPose = tagPose.transformBy(left_transform);
        }
        else if (side.equals(ReefAlignSide.RIGHT)){
            targetPose = tagPose.transformBy(right_transform);
        }
        else if (side.equals(ReefAlignSide.CENTER)){
            targetPose = tagPose.transformBy(true_center_transform);
        }
        else if (side.equals(ReefAlignSide.ALGAE_SCORE)){
            targetPose = tagPose.transformBy(algae_reef_pickup_transform_final);
        }
        trajectory = generateTrajectory(tagPose, targetPose);
        targetAngle = targetPose.getRotation().getRadians();
        profileTimer.restart();

        terminateFinish = 0;
        
        

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

        if(trajectory.getStates().size() > 0) {
            Pose2d currPose = trajectory.sample(profileTimer.get()).poseMeters;
            double currXTarget = currPose.getX();
            double currYTarget = currPose.getY();
            double xOutput = xController.calculate(currentX, currXTarget);
            double yOutput = yController.calculate(currentY, currYTarget);

            double xSpeedBound = 4;
            double ySpeedBound = 4;
            
            double angleOutput = angleController.calculate(currentAngle, targetAngle);
            xOutput = Math.min(xSpeedBound, Math.max(-xSpeedBound, xOutput));
            yOutput = Math.min(ySpeedBound, Math.max(-ySpeedBound, yOutput));
            angleOutput = Math.min(0.3, Math.max(-0.3, angleOutput));
            swerve.drive(xOutput, yOutput, angleOutput, true);
            SmartDashboard.putNumber("Target X", targetX);
            SmartDashboard.putNumber("Target Y", targetY);
            // SmartDashboard.putNumber("Target Theta", new Rotation2d(currentAngle).minus(new Rotation2d(targetAngle)).getRadians());
            SmartDashboard.putNumber("Curr Target X", currXTarget);
            SmartDashboard.putNumber("Curr Target Y", currYTarget);

            SmartDashboard.putNumber("Theta Move", angleOutput);

            if(
                Math.abs(swerve.getOdometry().getXVel()) < 0.02 &&
                Math.abs(swerve.getOdometry().getYVel()) < 0.02 &&
                Math.abs(swerve.getOdometry().getX() - targetPose.getX()) < 0.03 &&
                Math.abs(swerve.getOdometry().getY() - targetPose.getY()) < 0.03
            ) {
                terminateFinish++;
            }
        }
        else {
            finished = true;
        }
        // System.out.println("TargetX: " + targetX + " | " + "TargetY: " + targetY + " | " + "TargetAngle: " + targetAngle);
    }

    @Override
    public boolean isFinished(){
        return finished || terminateFinish > 2;
    }


    @Override
    public void end(boolean interrupted){
        System.out.println("Done!");
        swerve.drive(0, 0, 0, true);
    }

    public int closestAprilTag(double xPose, double yPose){
        List<AprilTag> tagList = layout.getTags();
        int closestAprilTagId = -1;
        double closestDistance = 3;
        for (AprilTag aprilTag : tagList){
            double distance = Math.sqrt(Math.pow(Math.abs(aprilTag.pose.getX() - xPose),2) + Math.pow(Math.abs(aprilTag.pose.getY() - yPose),2));
            if (distance < closestDistance){
                closestDistance = distance;
                closestAprilTagId = aprilTag.ID;
            }
        }
        return closestAprilTagId;

    }

    




    
}
