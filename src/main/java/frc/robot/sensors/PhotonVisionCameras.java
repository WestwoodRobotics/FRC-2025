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

    public PhotonVisionCameras(AprilTagFieldLayout layout) {
        // TODO: gracefully handle camera not being found
        reefCamera = new PhotonCamera("reef_camera");
        humanPlayerCamera = new PhotonCamera("human_camera");
        //TODO: put this in robot constants
        reefCameraToRobot = CameraConstants.kReefCameraToRobotTransform;
        humanCameraToRobot = CameraConstants.kHumanPlayerCameraToRobotTransform;

        reef_camera_result = null;
        this.layout = layout;
    }

    public void periodic() {
        // This method will be called once per scheduler run
        reefCamera.getLatestResult();
        humanPlayerCamera.getLatestResult();
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
