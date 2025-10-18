package frc.robot.commands.swerve;

import java.lang.annotation.Target;

import javax.sound.midi.Track;

import org.opencv.video.TrackerGOTURN;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.utils.LimelightHelpers;

enum TrackingState {
    SEARCHING,
    LOCKED_ON,
    TARGET_LOST
}

public class AprilTagFollowTarget extends Command {
    private final SwerveDrive swerve;
    private final String limelightName;
    private final PIDController xPID;
    private final PIDController zPID;
    private final PIDController rotPID;
    private final double SEARCH_ROTATION_SPEED = 0.2; // Adjust this for search speed
    private TrackingState state;
    private final double lostTimeOut = 1;
    private Timer timer;
    private double targetID;
    private double xOffset;
    private double zOffset;
    private double rotOffset;
    // private double x_p;
    // private double z_p;
    // private double rot_p;
    private boolean resetOdometryToEstimate;

    public AprilTagFollowTarget(SwerveDrive swerve, String limelightName, double targetID, double xOffset, double zOffset, double rotOffset, boolean resetOdometryToEstimate) {
        this(swerve, limelightName, targetID, xOffset, zOffset, rotOffset, 0.3, 0.2, 0.2, resetOdometryToEstimate);
    }

    public AprilTagFollowTarget(SwerveDrive swerve, String limelightName, double targetID, double xOffset, double zOffset, double rotOffset, double x_p, double z_p, double rot_p, boolean resetOdometryToEstimate) {
        this.swerve = swerve;
        this.limelightName = limelightName;
        this.state = TrackingState.SEARCHING;
        this.timer = new Timer();
        // this.x_p = x_p;
        // this.z_p = z_p;
        // this.rot_p = rot_p;
        // Tune PID values for your robot
        xPID = new PIDController(x_p, 0.0, 0.00);
        xPID.setTolerance(0.05); // 1 degree tolerance

        zPID = new PIDController(z_p, 0.0, 0.00);
        zPID.setTolerance(0.05); // 1 degree tolerance
        
        rotPID = new PIDController(rot_p, 0.0, 0.00);
        rotPID.setTolerance(0.05); // 1 degree tolerance

        this.targetID = targetID;
        this.xOffset = xOffset;
        this.zOffset = zOffset;
        this.rotOffset = rotOffset;
        this.resetOdometryToEstimate = resetOdometryToEstimate;

        
        addRequirements(swerve);

    }

    @Override
    public void initialize() {
        xPID.reset();
        zPID.reset();
        rotPID.reset();
        timer.reset();
        
        this.state = TrackingState.SEARCHING;
    }

    @Override
    public void execute() {
        switch(this.state) {
            case SEARCHING:
                swerve.drive(0, 0, -0.1, true);
                if (LimelightHelpers.getTV(limelightName) && LimelightHelpers.getT2DArray(limelightName)[9] == targetID) {
                    this.state = TrackingState.LOCKED_ON;
                }
                
            break;
            case TARGET_LOST:
                if (timer.hasElapsed(lostTimeOut)){
                    this.state = TrackingState.SEARCHING;
                } 
                
                else if (LimelightHelpers.getTV(limelightName) && LimelightHelpers.getT2DArray(limelightName)[9] == targetID) {
                    this.state = TrackingState.LOCKED_ON;
                }
                swerve.drive(0,0,0,true);

            break;
            case LOCKED_ON:
                if (!LimelightHelpers.getTV(limelightName) && LimelightHelpers.getT2DArray(limelightName)[9] == targetID) {
                    this.state = TrackingState.TARGET_LOST;
                    this.timer.reset();
                    this.timer.start();
                }

                // Get horizontal offset from target
                Pose3d targetPose = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName);
                double t_z = targetPose.getZ();
                double t_x = targetPose.getX();
                double t_rot = targetPose.getRotation().getY();

                // Calculate rotation to track target
                double x_output = xPID.calculate(t_x, xOffset);//Because limelight is offcenter of the robot
                x_output = x_output > 0.2 ? 0.2 : x_output;
                x_output = x_output < -0.2 ? -0.2 : x_output;
                
                double z_output = zPID.calculate(t_z, zOffset); //Desired distance (one dimensionally) from april tag
                z_output = z_output > 0.2 ? 0.2 : z_output;
                z_output = z_output < -0.2 ? -0.2 : z_output;

                double rot_output = rotPID.calculate(t_rot, rotOffset);
                rot_output = rot_output > 0.4 ? 0.4 : rot_output;
                rot_output = rot_output < -0.4 ? -0.4 : rot_output;


                swerve.drive(-z_output, x_output, 
                rot_output, true);
                System.out.println("X: " + t_x + " Z: " + t_z + "Rot: " + t_rot);
                
            break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, true);
        if (resetOdometryToEstimate){
            swerve.resetOdometry(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName).pose);
        }
    }

    @Override
    public boolean isFinished() {
        // Command never finishes on its own - must be interrupted
        return false;
    }
}