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

public class PhotonVisionCameras extends SubsystemBase {
    PhotonCamera camera;
    PhotonPipelineResult camera_result;
    AprilTagFieldLayout layout;
    Transform3d cameraToRobot;

    public PhotonVisionCameras(AprilTagFieldLayout layout) {
        // TODO: gracefully handle camera not being found
        camera = new PhotonCamera("reef_camera");

        //TODO: put this in robot constants
        cameraToRobot = new Transform3d(
            new Translation3d(0.24765, 0, 0.305),
            new Rotation3d(0, 0, 0)
        );
        camera_result = null;
        this.layout = layout;
    }

    public void periodic() {
        // This method will be called once per scheduler run
        camera_result = camera.getLatestResult();
    }

    public boolean hasTarget() {
        if(camera_result == null) {
            return false;
        }
        return camera_result.hasTargets();
    }

    public double getArea() {
        if(!hasTarget() || camera_result == null) {
            return 0;
        }
        return camera_result.getBestTarget().getArea();
    }

    public Pose2d getPose() {
        if(!hasTarget() || camera_result == null) {
            return null;
        }
        PhotonTrackedTarget target = camera_result.getBestTarget();
        if(layout.getTagPose(target.getFiducialId()).isPresent()) {
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(),
                layout.getTagPose(target.getFiducialId()).get(), 
                cameraToRobot
            );
            return robotPose.toPose2d();
        }
        return null;
    }
}
