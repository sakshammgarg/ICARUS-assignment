#include "kalman.h"
#include <stddef.h>  /* NULL */

/* -----------------------------------------------------------------------
 * kalman_init
 * Populate the filter struct with caller-supplied priors.
 * Negative noise values are clamped to 0 to keep the filter stable.
 * ----------------------------------------------------------------------- */
void kalman_init(KalmanFilter *kf,
                 float initial_state,
                 float initial_uncertainty,
                 float process_noise,
                 float measurement_noise)
{
    if (kf == NULL) {
        return;
    }

    kf->x = initial_state;

    /* Uncertainty must be non-negative */
    kf->P = (initial_uncertainty >= 0.0f) ? initial_uncertainty : 0.0f;

    /* Noise covariances must be non-negative */
    kf->Q = (process_noise      >= 0.0f) ? process_noise      : 0.0f;
    kf->R = (measurement_noise  >= 0.0f) ? measurement_noise  : 0.0f;
}

/* -----------------------------------------------------------------------
 * kalman_update
 *
 * Scalar Kalman equations (O(1) – four arithmetic ops, no loops):
 *
 *   PREDICT
 *     x_pred = x          (constant-velocity model, no control input)
 *     P_pred = P + Q      (uncertainty grows with process noise)
 *
 *   UPDATE
 *     S = P_pred + R      (innovation covariance)
 *     K = P_pred / S      (Kalman gain: 0 = trust model, 1 = trust sensor)
 *     x = x_pred + K * (measurement - x_pred)
 *     P = (1 - K) * P_pred
 *
 * Edge cases handled:
 *   • NULL pointer          → return 0.0f
 *   • S == 0  (P==0, R==0)  → skip update, return current estimate
 *     (both the model and sensor claim perfect certainty; trusting the
 *      existing state is the only numerically safe choice)
 * ----------------------------------------------------------------------- */
float kalman_update(KalmanFilter *kf, float measurement)
{
    float p_pred;   /* Predicted uncertainty   */
    float s;        /* Innovation covariance   */
    float k;        /* Kalman gain             */

    if (kf == NULL) {
        return 0.0f;
    }

    /* --- Predict -------------------------------------------------------- */
    p_pred = kf->P + kf->Q;

    /* --- Innovation covariance ----------------------------------------- */
    s = p_pred + kf->R;

    /* Degenerate guard: if S ≈ 0 the gain is undefined.  Both the model
     * and measurement claim zero variance, so the existing estimate is
     * already "perfect" – return it unchanged. */
    if (s <= 0.0f) {
        return kf->x;
    }

    /* --- Kalman gain ----------------------------------------------------- */
    k = p_pred / s;

    /* --- Update state and uncertainty ------------------------------------ */
    kf->x = kf->x + k * (measurement - kf->x);
    kf->P = (1.0f - k) * p_pred;

    return kf->x;
}
