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

enum TrackingState1 {
    SEARCHING,
    LOCKED_ON,
    TARGET_LOST
}

public class AprilTagFollowGeneral extends Command {
    private final SwerveDrive swerve;
    private final String limelightName;
    private final PIDController xPID;
    private final PIDController zPID;
    private final PIDController rotPID;
    private final double SEARCH_ROTATION_SPEED = 0.2; // Adjust this for search speed
    private TrackingState1 state;
    private final double lostTimeOut = 1;
    private Timer timer;
    public AprilTagFollowGeneral(SwerveDrive swerve, String limelightName) {
        this.swerve = swerve;
        this.limelightName = limelightName;
        this.state = TrackingState1.SEARCHING;
        this.timer = new Timer();
        
        // Tune PID values for your robot
        xPID = new PIDController(0.3, 0.0, 0.00);
        xPID.setTolerance(0.05); // 1 degree tolerance

        zPID = new PIDController(0.2, 0.0, 0.00);
        zPID.setTolerance(0.05); // 1 degree tolerance
        
        rotPID = new PIDController(0.2, 0.0, 0.00);
        rotPID.setTolerance(0.05); // 1 degree tolerance
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        xPID.reset();
        zPID.reset();
        rotPID.reset();
        timer.reset();
        
        this.state = TrackingState1.SEARCHING;
    }

    @Override
    public void execute() {
        switch(this.state) {
            case SEARCHING:
                swerve.drive(0, 0, -0.1, true);
                if (LimelightHelpers.getTV(limelightName)) {
                    this.state = TrackingState1.LOCKED_ON;

                }
                
            break;
            case TARGET_LOST:
                if (timer.hasElapsed(lostTimeOut)){
                    this.state = TrackingState1.SEARCHING;
                } 
                
                else if (LimelightHelpers.getTV(limelightName)) {
                    this.state = TrackingState1.LOCKED_ON;
                }
                swerve.drive(0,0,0,true);

            break;
            case LOCKED_ON:
                if (!LimelightHelpers.getTV(limelightName)) {
                    this.state = TrackingState1.TARGET_LOST;
                    this.timer.reset();
                    this.timer.start();
                }
                    // Get horizontal offset from target
                Pose3d targetPose = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName);
                double t_z = targetPose.getZ();
                double t_x = targetPose.getX();
                double t_rot = targetPose.getRotation().getY();


                
                // Calculate rotation to track target
                double x_output = xPID.calculate(t_x, 0.25);//Because limelight is offcenter of the robot
                x_output = x_output > 0.2 ? 0.2 : x_output;
                x_output = x_output < -0.2 ? -0.2 : x_output;
                
                double z_output = zPID.calculate(t_z, 1.2); //Desired distance (one dimensionally) from april tag
                z_output = z_output > 0.2 ? 0.2 : z_output;
                z_output = z_output < -0.2 ? -0.2 : z_output;

                double rot_output = rotPID.calculate(t_rot, 0.0);
                rot_output = rot_output > 0.4 ? 0.4 : rot_output;
                rot_output = rot_output < -0.4 ? -0.4 : rot_output;

                // Apply rotation only - no translation
                swerve.drive(-z_output, x_output, 
                rot_output, true);
                System.out.println("X: " + t_x + " Z: " + t_z + "Rot: " + t_rot);
                
            break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        // Command never finishes on its own - must be interrupted
        return false;
    }
}