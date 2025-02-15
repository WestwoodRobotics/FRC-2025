package frc.robot.commands.swerve;

import java.util.HashMap;

import javax.swing.text.html.HTML.Tag;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.PhotonVisionCameras;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.utils.swerve.ReefAlignSide;

public class GoToFieldPose extends Command{

    private enum ReefLeftPoses{
        BLUE_18(new Pose2d(new Translation2d(3.14029, 4.19090), new Rotation2d(0.0))),
        BLUE_19(new Pose2d(new Translation2d(3.95804, 5.27682), new Rotation2d(-60.0))),
        BLUE_20(new Pose2d(new Translation2d(5.30619, 5.11182), new Rotation2d(-120.0))),
        BLUE_21(new Pose2d(new Translation2d(5.83836, 3.86090), new Rotation2d(-180.0))),
        BLUE_22(new Pose2d(new Translation2d(5.02060, 2.77498), new Rotation2d(120.0))),
        BLUE_17(new Pose2d(new Translation2d(3.67246, 2.93998), new Rotation2d(60.0))),
        RED_10(new Pose2d(new Translation2d(11.709996, 4.1909), new Rotation2d(0.0))),
        RED_9(new Pose2d(new Translation2d(12.52750, 5.27682), new Rotation2d(-60.0))),
        RED_8(new Pose2d(new Translation2d(13.87590, 5.11182), new Rotation2d(-120.0))),
        RED_7(new Pose2d(new Translation2d(14.40781, 3.86090), new Rotation2d(-180.0))),
        RED_6(new Pose2d(new Translation2d(13.59031, 2.93998), new Rotation2d(120.0))),
        RED_11(new Pose2d(new Translation2d(12.24191, 2.93998), new Rotation2d(60.0)));

        private Pose2d pose2d;
        
        ReefLeftPoses(Pose2d pose2d) {
            this.pose2d = pose2d;
        }

        public Pose2d getPose2d() {
            return pose2d;
        }

    }


    private enum ReefRightPoses{
        BLUE_18(new Pose2d(new Translation2d(3.14029, 3.86090), new Rotation2d(0.0))),
        BLUE_19(new Pose2d(new Translation2d(3.67246, 5.11182), new Rotation2d(-60.0))),
        BLUE_20(new Pose2d(new Translation2d(5.02060, 5.27682), new Rotation2d(-120.0))),
        BLUE_21(new Pose2d(new Translation2d(5.83836, 4.19090), new Rotation2d(-180.0))),
        BLUE_22(new Pose2d(new Translation2d(5.30619, 2.93998), new Rotation2d(120.0))),
        BLUE_17(new Pose2d(new Translation2d(3.95804, 2.77498), new Rotation2d(60.0))),
        RED_10(new Pose2d(new Translation2d(11.709996, 3.8609), new Rotation2d(0.0))),
        RED_9(new Pose2d(new Translation2d(12.24191, 5.11182), new Rotation2d(-60.0))),
        RED_8(new Pose2d(new Translation2d(13.59031, 5.27682), new Rotation2d(-120.0))),
        RED_7(new Pose2d(new Translation2d(14.40781, 4.19090), new Rotation2d(-180.0))),
        RED_6(new Pose2d(new Translation2d(13.87590, 2.77498), new Rotation2d(120.0))),
        RED_11(new Pose2d(new Translation2d(12.52750, 2.77498), new Rotation2d(60.0)));

        private Pose2d pose2d;
        
        ReefRightPoses(Pose2d pose2d) {
            this.pose2d = pose2d;
        }

        public Pose2d getPose2d() {
            return pose2d;
        }
    }

    private enum ReefID{
        BLUE_18(18),
        BLUE_19(19),
        BLUE_20(20),
        BLUE_21(21),
        BLUE_22(22),
        BLUE_17(17),
        RED_10(10),
        RED_9(9),
        RED_8(8),
        RED_7(7),
        RED_6(6),
        RED_11(11);


        private int id;

        ReefID(int id){
            this.id = id;
        }

        public int getId(){
            return id;
        }
    }


    /*
     * Reef	ID	    theta (deg)	Tag Position (x, y)	    Left Target (x, y)	    Right Target (x, y)
        RED	10	    0	        (12.227306, 4.0259)	    (11.709996, 4.1909)	    (11.709996, 3.8609)
        RED	9	    -60	        (12.643358, 4.745482)	(12.52750, 5.27682)	    (12.24191, 5.11182)
        RED	8	    -120	    (13.474446, 4.745482)	(13.87590, 5.11182)	    (13.59031, 5.27682)
        RED	7	    -180	    (13.890498, 4.0259)	    (14.40781, 3.86090)	    (14.40781, 4.19090)
        RED	6	    120	        (13.474446, 3.306318)	(13.59031, 2.77498)     (13.87590, 2.93998)
        RED	11	    60	        (12.643358, 3.306318)	(12.24191, 2.93998)	    (12.52750, 2.77498)
        BLUE 18	    0	        (3.6576, 4.0259)	    (3.14029, 4.19090)	    (3.14029, 3.86090)
        BLUE 19	    -60	        (4.073906, 4.745482)	(3.95804, 5.27682)	    (3.67246, 5.11182)
        BLUE 20	    -120	    (4.90474, 4.745482)	    (5.30619, 5.11182)	    (5.02060, 5.27682)
        BLUE 21	    -180	    (5.321046, 4.0259)	    (5.83836, 3.86090)  	(5.83836, 4.19090)
        BLUE 22	    120	        (4.90474, 3.306318)	    (5.02060, 2.77498)	    (5.30619, 2.93998)
        BLUE 17	    60	        (4.073906, 3.306318)	(3.67246, 2.93998)	    (3.95804, 2.77498)
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


    private int visibleFiducialID = 0;

    private Pose2d targetPose;

    private HashMap<Integer, ReefLeftPoses> reefLeftPoseToFiducialID = new HashMap<Integer, ReefLeftPoses>();
    private HashMap<Integer, ReefRightPoses> reefRightPoseToFiducialID = new HashMap<Integer, ReefRightPoses>();
    private ReefAlignSide side;
    //11.8 
    public GoToFieldPose(SwerveDrive swerve, ReefAlignSide side){ 

        this.swerve = swerve;


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
        try{
            swerve.m_cameras.hasTarget();
            swerve.m_cameras.getFiducialId();
        }
        catch (Exception e){
            isFinished = true;
            return;
        }

        if (!swerve.m_cameras.hasTarget()){
            isFinished = true;
            return;
        }

        visibleFiducialID = swerve.m_cameras.getFiducialId();

        if (visibleFiducialID == -1){
            isFinished = true;
            return;
        }

        if (side == ReefAlignSide.LEFT){
            for (int i = 0; i < ReefLeftPoses.values().length; i++){
                reefLeftPoseToFiducialID.put(ReefID.values()[i].getId(), ReefLeftPoses.values()[i]);
            }
        }
        else{
            for (int i = 0; i < ReefRightPoses.values().length; i++){
                reefRightPoseToFiducialID.put(ReefID.values()[i].getId(), ReefRightPoses.values()[i]);
            }
        }

        targetPose = (this.side == ReefAlignSide.LEFT) 
                        ? reefLeftPoseToFiducialID.get(visibleFiducialID).getPose2d() 
                        : reefRightPoseToFiducialID.get(visibleFiducialID).getPose2d();

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
