package frc.robot.sensors;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class PhotonVisionCameras extends SubsystemBase {
    PhotonCamera reefCamera;
    PhotonCamera humanPlayerCamera;
    PhotonPipelineResult reef_camera_result;
    PhotonPipelineResult human_camera_result;

    AprilTagFieldLayout layout;
    Transform3d reefCameraToRobot;
    Transform3d humanCameraToRobot;
    double[] x_poses;
    double[] y_poses;
    double[] theta_poses;
    int counter;

    public PhotonVisionCameras(AprilTagFieldLayout layout) {
        // TODO: gracefully handle camera not being found
        reefCamera = new PhotonCamera("reef_camera");
        humanPlayerCamera = new PhotonCamera("human_camera");
        //TODO: put this in robot constants
        reefCameraToRobot = CameraConstants.kReefCameraToRobotTransform;
        humanCameraToRobot = CameraConstants.kHumanPlayerCameraToRobotTransform;

        reef_camera_result = null;
        this.layout = layout;
        counter = 0;
        x_poses = new double[300];
        y_poses = new double[300];
        theta_poses = new double[300];
    }

    public void periodic() {
        // This method will be called once per scheduler run
        reef_camera_result = reefCamera.getLatestResult();
        human_camera_result = humanPlayerCamera.getLatestResult();

        // SmartDashboard.putNumber("test_area", getReefCameraArea());
        // SmartDashboard.putNumber("test_x_var", variance(x_poses));
        // SmartDashboard.putNumber("test_y_var", variance(y_poses));
        // SmartDashboard.putNumber("test_theta_var", variance(theta_poses));
    }

    public double variance(double[] test) {
        double mean_sum = 0;
        for(int i = 0; i < test.length; i++) {
            mean_sum += test[i];
        }
        double mean = mean_sum/(double)(test.length);

        double var_sum = 0;
        for(int i = 0; i < test.length; i++) {
            var_sum += Math.pow(test[i] - mean, 2);
        }
        return var_sum/(double)(test.length - 1);
    }

    public boolean reefCameraHasTarget() {
        if(reef_camera_result == null) {
            return false;
        }
        return reef_camera_result.hasTargets();
    }

    public boolean humanPlayerCameraHasTarget() {
        if(human_camera_result == null) {
            return false;
        }
        return human_camera_result.hasTargets();
    }

    public double getReefCameraArea() {
        if(!reefCameraHasTarget() || reef_camera_result == null) {
            return 0;
        }
        return reef_camera_result.getBestTarget().getArea();
    }

    public double getHumanPlayerCameraArea() {
        if(!humanPlayerCameraHasTarget() || human_camera_result == null) {
            return 0;
        }
        return human_camera_result.getBestTarget().getArea();
    }

    public Pose2d getPoseRelativeReefCamera() {
        if(!reefCameraHasTarget() || reef_camera_result == null) {
            return null;
        }
        PhotonTrackedTarget target = reef_camera_result.getBestTarget();
        if(layout.getTagPose(target.getFiducialId()).isPresent()) {
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(),
                layout.getTagPose(target.getFiducialId()).get(), 
                reefCameraToRobot
            );
            Transform3d test_pose = target.getBestCameraToTarget();
            x_poses[counter] = test_pose.getX();
            y_poses[counter] = test_pose.getY();
            theta_poses[counter] = test_pose.getRotation().getZ();
            counter = (counter + 1) % 300;
            return robotPose.toPose2d();
        }
        return null;
    }

    public Pose2d getPoseRelativeHumanPlayerCamera() {
        if(!humanPlayerCameraHasTarget() || human_camera_result == null) {
            return null;
        }
        PhotonTrackedTarget target = human_camera_result.getBestTarget();
        if(layout.getTagPose(target.getFiducialId()).isPresent()) {
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(),
                layout.getTagPose(target.getFiducialId()).get(), 
                humanCameraToRobot
            );
            return robotPose.toPose2d();
        }
        return null;
    }

    public int getBestReefCameraFiducialId() {
        if(!reefCameraHasTarget() || reef_camera_result == null) {
            return -1;
        }
        return reef_camera_result.getBestTarget().getFiducialId();
    }

    public int getBestHumanPlayerCameraFiducialId() {
        if(!humanPlayerCameraHasTarget() || human_camera_result == null) {
            return -1;
        }
        return human_camera_result.getBestTarget().getFiducialId();
    }
}
