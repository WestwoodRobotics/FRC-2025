package frc.robot.subsystems.utils;

import java.nio.channels.Pipe;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrainKalmanFilter extends KalmanFilter<N7, N8, N7>{
    public DriveTrainKalmanFilter(Matrix<N7, N1> initial_state, Matrix<N7, N7> initial_covariance) {
        super(initial_state, initial_covariance);
    }

    @Override
    public Matrix<N7, N1> innovation(
        Matrix<N7, N1> sensor_input,
        Matrix<N7, N7> sensor_matrix,
        Matrix<N7, N1> state
    ) {
        Matrix<N7, N1> calc_sensor = sensor_matrix.times(state);
        SmartDashboard.putNumber("sensor theta", sensor_input.get(3, 0));
        SmartDashboard.putNumber("calc theta", calc_sensor.get(3, 0));
        sensor_input.set(3, 0, posMod(sensor_input.get(3, 0) + Math.PI, 2*Math.PI)-Math.PI);
        sensor_input.set(6, 0, posMod(sensor_input.get(6, 0) + Math.PI, 2*Math.PI)-Math.PI);
        calc_sensor.set(3, 0, posMod(calc_sensor.get(3, 0) + Math.PI, 2*Math.PI)-Math.PI);
        calc_sensor.set(6, 0, posMod(calc_sensor.get(6, 0) + Math.PI, 2*Math.PI)-Math.PI);
        SmartDashboard.putNumber("new sensor theta", sensor_input.get(3, 0));
        SmartDashboard.putNumber("new calc theta", calc_sensor.get(3, 0));
        SmartDashboard.putNumber("sensor diff", angDiff(sensor_input.get(3, 0), calc_sensor.get(3, 0)));
        Matrix<N7, N1> ret_val = new Matrix<N7, N1>(
            new SimpleMatrix(7, 1, true,
            sensor_input.get(0, 0) - calc_sensor.get(0, 0),
            sensor_input.get(1, 0) - calc_sensor.get(1, 0),
            sensor_input.get(2, 0) - calc_sensor.get(2, 0),
            angDiff(sensor_input.get(3, 0), calc_sensor.get(3, 0)),
            sensor_input.get(4, 0) - calc_sensor.get(4, 0),
            sensor_input.get(5, 0) - calc_sensor.get(5, 0),
            angDiff(sensor_input.get(6, 0), calc_sensor.get(6, 0))
        ));
        return ret_val;
    }

    @Override
    public void aPrioriUpdate(Matrix<N8, N1> control_input, Matrix<N7, N7> process_cov, Matrix<N7, N7> update_matrix,
            Matrix<N7, N8> control_matrix) {
        super.aPrioriUpdate(control_input, process_cov, update_matrix, control_matrix);
        this.state.set(2, 0, posMod(this.state.get(2, 0) + Math.PI, 2*Math.PI)-Math.PI);
    }

    @Override
    public void aPosteriorUpdate(Matrix<N7, N1> sensor_input, Matrix<N7, N7> sensor_cov, Matrix<N7, N7> sensor_matrix,
            Nat<N7> n_dim) {
        super.aPosteriorUpdate(sensor_input, sensor_cov, sensor_matrix, n_dim);
        // System.out.println("Original theta: " + this.state.get(2, 0));
        // System.out.println("New theta: " + (posMod(this.state.get(2, 0) + Math.PI, 2*Math.PI)-Math.PI));
        this.state.set(2, 0, posMod(this.state.get(2, 0) + Math.PI, 2*Math.PI)-Math.PI);
    }

    private static double angDiff(double a, double b) {
        double x = posMod(a-b, 2*Math.PI);
        double y = posMod(b-a, 2*Math.PI);
        return -(x < y ? -x : y);
    }

    private static double posMod(double x, double y) {
        double mod = x % y;
        if (mod < 0) {
            mod += y;
        }
        return mod;
    }
}
