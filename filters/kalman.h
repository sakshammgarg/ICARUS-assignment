#ifndef KALMAN_H
#define KALMAN_H

/*
 * KalmanFilter – single-state (scalar) Kalman filter
 *
 * x : current state estimate
 * P : estimate uncertainty (error covariance)
 * Q : process  noise covariance  (how much the true state drifts per step)
 * R : measurement noise covariance (how noisy the sensor is)
 *
 * All fields are floats; no dynamic allocation is used anywhere.
 */
typedef struct {
    float x;   /* State estimate          */
    float P;   /* State uncertainty       */
    float Q;   /* Process noise           */
    float R;   /* Measurement noise       */
} KalmanFilter;

/*
 * kalman_init – initialise the filter with known priors.
 *
 * initial_state       : best guess of the true value at t=0
 * initial_uncertainty : confidence in that guess  (larger = less confident)
 * process_noise       : Q – expected variance introduced each time step
 * measurement_noise   : R – sensor noise variance
 */
void  kalman_init(KalmanFilter *kf,
                  float initial_state,
                  float initial_uncertainty,
                  float process_noise,
                  float measurement_noise);

/*
 * kalman_update – incorporate one measurement and return the new estimate.
 *
 * Time complexity : O(1)
 * Returns the previous state estimate if kf is NULL or if the innovation
 * covariance is zero (degenerate case where P and R are both 0).
 */
float kalman_update(KalmanFilter *kf, float measurement);

#endif /* KALMAN_H */
