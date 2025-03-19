package frc.robot.subsystems.utils;

import org.ejml.simple.SimpleMatrix;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;

public class KalmanLocalization {

    public Matrix<N7, N1> state;
    public Matrix<N7, N7> cov;

    public DriveTrainKalmanFilter filter;

    public double curr_time;

    public Pose2d startingPose;

    public Pose2d currentPose;

    public Twist2d deltaTwist;

    public double dx;
    public double dy;
    public double dtheta;

    public KalmanLocalization(Pose2d startingPose) {
        state = new Matrix<N7, N1>(new SimpleMatrix(7, 1, true,
                startingPose.getX(),
                startingPose.getY(),
                startingPose.getRotation().getRadians(),
                0,
                0,
                0,
                0
        ));
        cov = new Matrix<N7, N7>(new SimpleMatrix(7, 7, true,
                0.1, 0, 0, 0, 0, 0, 0,
                0, 0.1, 0, 0, 0, 0, 0,
                0, 0, 0.1, 0, 0, 0, 0,
                0, 0, 0, 0.0001, 0, 0, 0,
                0, 0, 0, 0, 0.0001, 0, 0,
                0, 0, 0, 0, 0, 0.0001, 0,
                0, 0, 0, 0, 0, 0, 100
        ));
        filter = new DriveTrainKalmanFilter(state, cov);
        curr_time = Timer.getFPGATimestamp();
    }

    private Matrix<N7, N7> getUpdateMatrix(double dt) {
        return new Matrix<N7, N7>(new SimpleMatrix(7, 7, true,
                1, 0, 0, dt, 0, 0, 0,
                0, 1, 0, 0, dt, 0, 0,
                0, 0, 1, 0, 0, dt, 0,
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 1
        ));
    }

    private Matrix<N7, N8> getControlMatrix(ArrayList<Translation2d> wheel_pos, double theta, double dt) {
        SimpleMatrix swerve_inv_kinematics = new SimpleMatrix(2*wheel_pos.size(), 3);
        for(int i = 0; i < wheel_pos.size(); i++) {
            swerve_inv_kinematics.setRow(2*i, 0, 1, 0, -wheel_pos.get(i).getY());
            swerve_inv_kinematics.setRow(2*i+1, 0, 0, 1, wheel_pos.get(i).getX());
        }
        SimpleMatrix swerve_forward_kinematics = swerve_inv_kinematics.pseudoInverse();
        SimpleMatrix rotation_matrix = new SimpleMatrix(3, 3, true,
                Math.cos(theta), -Math.sin(theta), 0,
                Math.sin(theta), Math.cos(theta), 0,
                0, 0, 1
        );
        SimpleMatrix world_forward_kinematics = (swerve_forward_kinematics.transpose().mult(rotation_matrix.transpose())).transpose();
        SimpleMatrix out_matrix = new SimpleMatrix(7, 8);
        out_matrix.insertIntoThis(3, 0, world_forward_kinematics);
        return new Matrix<N7, N8>(out_matrix);
    }

    private Matrix<N7, N7> getSensorMatrix(double dt, boolean reefCameraHasTarget, boolean humanCameraHasTarget) {
        double reef_flag_target = reefCameraHasTarget ? 1.0 : 0.0;
        double human_flag_target = humanCameraHasTarget ? 1.0 : 0.0;
        return new Matrix<N7, N7>(new SimpleMatrix(7, 7, true,
                0,             0,             0,             0, 0, 1, 1,
                reef_flag_target, 0,          0,             0, 0, 0, 0,
                0,             reef_flag_target,0,            0, 0, 0, 0,
                0,             0,             reef_flag_target,0, 0, 0, 0,
                human_flag_target,0,          0,             0, 0, 0, 0,
                0,             human_flag_target,0,          0, 0, 0, 0,
                0,             0,             human_flag_target,0,0, 0, 0
        ));
    }

    private Matrix<N7, N7> getSensorCovariance(double dt, double reefCameraTargetArea, boolean reefHasTarget, double humanCameraTargetArea, boolean humanHasTarget) {
        final double ANG_RAND_WALK_RAD_PER_SEC = 9.89019909e-5;
        final double AREA_CART_VAR_FACTOR = 0.0005;
        final double AREA_ANG_VAR_FACTOR = 0.0005;
        double reef_cart_var = 10;
        double reef_ang_var = 10;
        if(reefHasTarget && reefCameraTargetArea > 0) {
            reef_cart_var = AREA_CART_VAR_FACTOR/reefCameraTargetArea;
            reef_ang_var = AREA_ANG_VAR_FACTOR/reefCameraTargetArea;
        }
        double human_cart_var = 10;
        double human_ang_var = 10;
        if(humanHasTarget && humanCameraTargetArea > 0) {
            human_cart_var = AREA_CART_VAR_FACTOR/humanCameraTargetArea;
            human_ang_var = AREA_ANG_VAR_FACTOR/humanCameraTargetArea;
        }
        return new Matrix<N7, N7>(new SimpleMatrix(7, 7, true,
                ANG_RAND_WALK_RAD_PER_SEC*dt, 0, 0, 0, 0, 0, 0,
                0, reef_cart_var, 0, 0, 0, 0, 0,
                0, 0, reef_cart_var, 0, 0, 0, 0,
                0, 0, 0, reef_ang_var, 0, 0, 0,
                0, 0, 0, 0, human_cart_var, 0, 0,
                0, 0, 0, 0, 0, human_cart_var, 0,
                0, 0, 0, 0, 0, 0, human_ang_var
        ));
    }

    private Matrix<N7, N7> getProcessCovariance(Pose2d velocity, double dt) {
        final double CONSTANT_UNCERTAINTY = 1e-7;
        final double DIRECTIONAL_UNCERTAINTY = 1e-5;
        final double SPEED_UNCERTAINTY = 1e-4;
        final double ROTATION_UNCERTAINTY = 1e-3;
        double speed = Math.sqrt(Math.pow(velocity.getX(),2)+Math.pow(velocity.getY(),2));
        double x_uncertainty = CONSTANT_UNCERTAINTY+Math.abs(velocity.getX())*DIRECTIONAL_UNCERTAINTY+speed*SPEED_UNCERTAINTY;
        double y_uncertainty = CONSTANT_UNCERTAINTY+Math.abs(velocity.getY())*DIRECTIONAL_UNCERTAINTY+speed*SPEED_UNCERTAINTY;
        double t_uncertainty = CONSTANT_UNCERTAINTY+Math.abs(velocity.getRotation().getRadians())*ROTATION_UNCERTAINTY+speed*SPEED_UNCERTAINTY;
        final double BIAS_STABILITY_RAD_PER_SEC = 1e-8;
        double bias_stability_var = BIAS_STABILITY_RAD_PER_SEC;
        return new Matrix<N7, N7>(new SimpleMatrix(new double[][]{
                {0,0,0,0,0,0,0},
                {0,x_uncertainty,0,0,0,0,0},
                {0,0,y_uncertainty,0,0,0,0},
                {0,0,0,t_uncertainty,0,0,0},
                {0,0,0,0,0,0,0},
                {0,0,0,0,0,0,0},
                {0,0,0,0,0,0,bias_stability_var*dt},
        }));
    }

    public void update(ArrayList<Translation2d> wheel_vel, ArrayList<Translation2d> wheel_pos, Pose2d reef_camera_robot_pose, Pose2d human_camera_robot_pose, double gyro_dtheta, double reef_camera_area, double human_camera_area, boolean reef_camera_has_target, boolean human_camera_has_target, double visionDelay) {
        Pose2d robot_frame_vel = new Pose2d(
                new Translation2d(
                        Math.cos(getTheta())*getX()-Math.sin(getTheta())*getY(),
                        Math.sin(getTheta())*getX()+Math.cos(getTheta())*getY()
                ),
                new Rotation2d(getTheta())
        );

        double next_time = Timer.getFPGATimestamp();
        double dt = next_time - curr_time;
        curr_time = next_time;

        SimpleMatrix control_matrix = new SimpleMatrix(2*wheel_pos.size(), 1);

        for(int i = 0; i < wheel_pos.size(); i++) {
            control_matrix.set(2*i,0, wheel_vel.get(i).getX());
            control_matrix.set(2*i+1,0, wheel_vel.get(i).getY());
        }
        filter.aPrioriUpdate(new Matrix<>(control_matrix), getProcessCovariance(robot_frame_vel, dt), getUpdateMatrix(dt), getControlMatrix(wheel_pos, getTheta(), dt));

        double reef_x = 0, reef_y = 0, reef_theta = 0;

        if(reef_camera_has_target && reef_camera_robot_pose != null) {
            double vx = this.getXVel();
            double vy = this.getYVel();
            double vtheta = this.getThetaVel(); 
            reef_x = reef_camera_robot_pose.getX() + vx*visionDelay; //I'm assuming vel stays constant during vision delay 
            reef_y = reef_camera_robot_pose.getY() + vy*visionDelay; 
            reef_theta = reef_camera_robot_pose.getRotation().getRadians() + vtheta*visionDelay;
        }

        double human_x = 0, human_y = 0, human_theta = 0;

        if(human_camera_has_target && human_camera_robot_pose != null) {
            double vx = state.get(3,0);
            double vy = state.get(4,0);
            double vtheta = state.get(5,0);
            human_x = human_camera_robot_pose.getX() + vx*visionDelay;
            human_y = human_camera_robot_pose.getY() + vy*visionDelay;
            human_theta = human_camera_robot_pose.getRotation().getRadians() + vtheta*visionDelay;
        }
        Matrix<N7, N1> sensor_input = new Matrix<>(new SimpleMatrix(7,1,true,
                gyro_dtheta,
                reef_x,
                reef_y,
                reef_theta,
                human_x,
                human_y,
                human_theta
        ));
        filter.aPosteriorUpdate(sensor_input, getSensorCovariance(dt, reef_camera_area, reef_camera_has_target, human_camera_area, human_camera_has_target), getSensorMatrix(dt, reef_camera_has_target, human_camera_has_target), N7.instance);
        state = filter.getState();
        cov = filter.getCovariance();
    }

    public double getX() {
        return state.get(0,0);
    }

    public double getY() {
        return state.get(1,0);
    }
    public double getTheta(){
        return state.get(2, 0);
    }
    public double getXVel(){
        return state.get(3, 0);
    }
    public double getYVel(){
        return state.get(4, 0);
    }
    public double getThetaVel(){
        return state.get(5, 0);
    }
    public double getGyroBias(){
        return state.get(6, 0);
    }

    public Pose2d getPoseMeters(){
        return new Pose2d(new Translation2d(this.getX(), this.getY()), new Rotation2d(this.getTheta()));
    }

    
        
}


