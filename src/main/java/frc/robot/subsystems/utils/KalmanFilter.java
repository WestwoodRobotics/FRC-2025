package frc.robot.subsystems.utils;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Nat;

public class KalmanFilter<N extends Num, C extends Num, S extends Num> {
    private Matrix<N, N1> state;
    private Matrix<N, N> state_covariance;
    public KalmanFilter(Matrix<N, N1> initial_state, Matrix<N, N> initial_covariance) {
        state = initial_state;
        state_covariance = initial_covariance;
    }

    public void aPrioriUpdate(
        Matrix<C, N1> control_input,
        Matrix<N, N> process_cov,
        Matrix<N, N> update_matrix,
        Matrix<N, C> control_matrix
    ) {
        Matrix<N, N1> a_priori_state = update_matrix.times(state).plus(
            control_matrix.times(control_input)
        );
        Matrix<N, N> a_priori_cov = update_matrix.times(state_covariance).times(update_matrix.transpose()).plus(process_cov);
        state = a_priori_state;
        state_covariance = a_priori_cov;
    }

    public void aPosteriorUpdate(
        Matrix<S, N1> sensor_input,
        Matrix<S, S> sensor_cov,
        Matrix <S, N> sensor_matrix,
        Nat<N> n_dim
    ) {
        Matrix<S, N1> pre_fit_res = sensor_input.minus(sensor_matrix.times(state));
        Matrix<S, S> pre_fit_cov = sensor_matrix.times(state_covariance).times(sensor_matrix.transpose()).plus(sensor_cov);

        Matrix<N,S> kalman_gain = state_covariance.times(sensor_matrix.transpose().times(pre_fit_cov.inv()));
        Matrix<N, N1> a_post_state = state.plus(kalman_gain.times(pre_fit_res));
        Matrix<N, N> a_post_cov = (Matrix.eye(n_dim).minus(kalman_gain.times(sensor_matrix))).times(state_covariance);

        state = a_post_state;
        state_covariance = a_post_cov;
    }

    public Matrix<N, N1> getState(){
        return state;
    }

    public Matrix<N, N> getCovariance() {
        return state_covariance;
    }
}
