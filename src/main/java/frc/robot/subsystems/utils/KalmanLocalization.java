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

    public KalmanFilter<N7, N8, N1> filter;

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
        filter = new KalmanFilter<N7, N8, N1>(state, cov);

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

    private Matrix<N1, N7> getSensorMatrix(double dt) {
        return new Matrix<N1, N7>(new SimpleMatrix(1, 7, true,
            0, 0, 0, 0, 0, 1, 1
        ));
    }

    private Matrix<N1, N1> getSensorCovariance(double dt) {
        final double ANG_RAND_WALK_RAD_PER_SEC = 0.1;

        return new Matrix<N1, N1>(new SimpleMatrix(1, 1, true,
            Math.pow(ANG_RAND_WALK_RAD_PER_SEC*dt, 2)
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

    public void update(
        ArrayList<Translation2d> wheel_vel,
        ArrayList<Translation2d> wheel_pos,
        double gyro_dtheta
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
        Matrix <N1, N1> sensor_input = new Matrix<N1, N1>(
            new SimpleMatrix(1, 1, true, gyro_dtheta)
        );
        filter.aPosteriorUpdate(
            sensor_input,
            getSensorCovariance(dt),
            getSensorMatrix(dt),
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


