package frc.robot.subsystems.utils;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
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

    public KalmanFilter<N7, N8, N7> filter;

    public double curr_time;

    public Pose2d startingPose;

    public Pose2d currentPose;

    public Twist2d deltaTwist;

    public double dx;
    public double dy;
    public double dtheta;


    public KalmanLocalization(
        Pose2d startingPose
    ) {
        state = new Matrix<N7, N1>(
            new SimpleMatrix(7, 1, true,
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
        filter = new KalmanFilter<N7, N8, N7>(state, cov);

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
    private Matrix<N7, N8> getControlMatrix(
        ArrayList<Translation2d> wheel_pos,
        double theta,
        double dt
    ) {
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

    private Matrix<N7, N7> getSensorMatrix(double dt, boolean reefCameraHasTarget, boolean humanPlayerCameraHasTarget) {
        double reef_flag_target = 0.0;
        if(reefCameraHasTarget) {
            reef_flag_target = 1.0;
        }

        double human_flag_target = 0.0;
        if(humanPlayerCameraHasTarget) {
            human_flag_target = 1.0;
        }


        // return new Matrix<N4, N7>(new SimpleMatrix(4, 7, true,
        //     0, 0, 0, 0, 0, 1, 1,
        //             reef_flag_target, 0, 0, 0, 0, 0, 0,
        //             0, reef_flag_target, 0, 0, 0, 0, 0,
        //             0, 0, reef_flag_target, 0, 0, 0, 0
        // ));

        return new Matrix<N7, N7> (new SimpleMatrix(7,7,true,
        0, 0, 0, 0, 0, 1, 1,
            reef_flag_target, 0, 0, 0, 0, 0, 0,
            0, reef_flag_target, 0, 0, 0, 0, 0,
            0, 0, reef_flag_target, 0, 0, 0, 0,
            human_flag_target, 0, 0, 0, 0, 0, 0,
            0, human_flag_target, 0, 0, 0, 0, 0,
            0, 0, human_flag_target, 0, 0, 0, 0
            ));
    }

    private Matrix<N7, N7> getSensorCovariance(double dt, double camera_area, boolean has_target) {
        final double ANG_RAND_WALK_RAD_PER_SEC = 0.1;

        final double AREA_CART_VAR_FACTOR = 0.01;
        final double AREA_ANG_VAR_FACTOR = 0.01;

        double cart_var = 10;
        double ang_var = 10;
        if(has_target && camera_area > 0) {
            cart_var = AREA_CART_VAR_FACTOR/camera_area;
            ang_var = AREA_ANG_VAR_FACTOR/camera_area;
        }

        return new Matrix<N7, N7>(new SimpleMatrix(7, 7, true,
            Math.pow(ANG_RAND_WALK_RAD_PER_SEC*dt, 2), 0, 0, 0, 0, 0 , 0,
            0, cart_var, 0, 0, 0, 0, 0,
            0, 0, cart_var, 0, 0, 0, 0,
            0, 0, 0, ang_var, 0, 0, 0,
            0, 0, 0, 0, cart_var, 0, 0,
            0, 0, 0, 0, 0, cart_var, 0,
            0, 0, 0, 0, 0, 0, ang_var
        ));
    }


    private Matrix<N7, N7> getProcessCovariance(
        Pose2d velocity,
        double dt
    ) {
        final double CONSTANT_UNCERTAINTY = 0.000001;
        final double DIRECTIONAL_UNCERTAINTY = 0.01;
        final double SPEED_UNCERTAINTY = 0.01;
        final double ROTATION_UNCERTAINTY = 0.01;
        double speed = Math.sqrt(velocity.getX()*velocity.getX() + velocity.getY()*velocity.getY());
        double x_uncertainty = CONSTANT_UNCERTAINTY + Math.abs(velocity.getX())*DIRECTIONAL_UNCERTAINTY + speed*SPEED_UNCERTAINTY;
        double y_uncertainty = CONSTANT_UNCERTAINTY + Math.abs(velocity.getY())*DIRECTIONAL_UNCERTAINTY + speed*SPEED_UNCERTAINTY;
        double t_uncertainty = CONSTANT_UNCERTAINTY + Math.abs(velocity.getRotation().getRadians())*ROTATION_UNCERTAINTY + speed*SPEED_UNCERTAINTY;


        final double BIAS_STABILITY_RAD_PER_SEC = 0.00003878509;
        double bias_stability_var = BIAS_STABILITY_RAD_PER_SEC;
        return new Matrix<N7, N7>(new SimpleMatrix(
            new double[][]{
                {0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, x_uncertainty, 0, 0, 0},
                {0, 0, 0, 0, y_uncertainty, 0, 0},
                {0, 0, 0, 0, 0, t_uncertainty, 0},
                {0, 0, 0, 0, 0, 0, bias_stability_var},
            }
        ));
    }

    private double posMod(double x, double y) {
        double mod = x % y;
        if (mod < 0)
        {
            mod += y;
        }
        return mod;
    }

    public void update(
        ArrayList<Translation2d> wheel_vel,
        ArrayList<Translation2d> wheel_pos,
        Pose2d reef_camera_robot_pose,
        Pose2d human_camera_robot_pose,
        double gyro_dtheta,
        double reef_camera_area,
        double human_camera_area,
        boolean reef_camera_has_target,
        boolean human_camera_has_target
    ) {
        Pose2d robot_frame_vel = new Pose2d(
            new Translation2d(
                Math.cos(getTheta())*getX() - Math.sin(getTheta())*getY(),
                Math.sin(getTheta())*getX() + Math.cos(getTheta())*getY()
            ),
            new Rotation2d(getThetaVel())
        );

        double next_time = Timer.getFPGATimestamp();
        double dt = next_time - curr_time;
        curr_time = next_time;

        SimpleMatrix control_matrix = new SimpleMatrix(2*wheel_pos.size(), 1);
        for(int i = 0; i < wheel_pos.size(); i++) {
            control_matrix.set(2*i, 0, wheel_vel.get(i).getX());
            control_matrix.set(2*i + 1, 0, wheel_vel.get(i).getY());
        }

        filter.aPrioriUpdate(
            new Matrix<N8, N1>(
                control_matrix
            ),
            getProcessCovariance(robot_frame_vel, dt),
            getUpdateMatrix(dt),
            getControlMatrix(wheel_pos, getTheta(), dt)
        );
        double reef_camera_x = 0;
        double reef_camera_y = 0;
        double reef_camera_theta = 0;

        double human_camera_x = 0;
        double human_camera_y = 0;
        double human_camera_theta = 0;

        if(reef_camera_has_target && reef_camera_robot_pose != null) {
            double zeroed_camera = reef_camera_robot_pose.getRotation().getRadians() + Math.PI;
            double zeroed_pose = state.get(2, 0) + Math.PI;
            double angle_offset = Math.floor(zeroed_pose/(2*Math.PI))*2*Math.PI;
            System.out.println("Current angle: " + zeroed_pose);
            System.out.println("Angle offset: " + angle_offset);
            System.out.println("Camera angle: " + zeroed_camera);
            reef_camera_x = reef_camera_robot_pose.getX();
            reef_camera_y = reef_camera_robot_pose.getY();
            System.out.println("Camera mod: " + posMod(zeroed_camera, 2*Math.PI));
            reef_camera_theta = posMod(zeroed_camera, 2*Math.PI)+angle_offset - Math.PI;
            System.out.println("Camera corrected: " + reef_camera_theta);
        }

        if (human_camera_has_target && human_camera_robot_pose != null) {
            double zeroed_camera = human_camera_robot_pose.getRotation().getRadians() + Math.PI;
            double zeroed_pose = state.get(2, 0) + Math.PI;
            double angle_offset = Math.floor(zeroed_pose/(2*Math.PI))*2*Math.PI;
            System.out.println("Current angle: " + zeroed_pose);
            System.out.println("Angle offset: " + angle_offset);
            System.out.println("Camera angle: " + zeroed_camera);
            human_camera_x = human_camera_robot_pose.getX();
            human_camera_y = human_camera_robot_pose.getY();
            System.out.println("Camera mod: " + posMod(zeroed_camera, 2*Math.PI));
            human_camera_theta = posMod(zeroed_camera, 2*Math.PI)+angle_offset - Math.PI;
            System.out.println("Camera corrected: " + human_camera_theta);
        }
        
        Matrix <N7, N1> sensor_input = new Matrix<N7, N1>(
            new SimpleMatrix(7, 1, true,
                gyro_dtheta,
                reef_camera_x,
                reef_camera_y,
                reef_camera_theta,
                human_camera_x,
                human_camera_y,
                human_camera_theta

            )
        );
        filter.aPosteriorUpdate(
            sensor_input,
            getSensorCovariance(dt, reef_camera_area, reef_camera_has_target && reef_camera_robot_pose != null),
            getSensorMatrix(dt, reef_camera_has_target && reef_camera_robot_pose != null, human_camera_has_target && human_camera_robot_pose != null),
            N7.instance
        );
        state = filter.getState();
        cov = filter.getCovariance();
    }

    public double getX(){
        return state.get(0, 0);
    }
    public double getY(){
        return state.get(1, 0);
    }
    public double getTheta(){
        return posMod(state.get(2, 0) + Math.PI, 2*Math.PI) - Math.PI;
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


