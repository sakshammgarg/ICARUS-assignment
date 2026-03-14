/*
 * test_filters.c
 *
 * Self-contained test suite for MovingAverageFilter and KalmanFilter.
 * No dynamic allocation.  Prints PASS/FAIL per test and a final summary.
 */

#include <stdio.h>
#include <math.h>      /* fabsf */
#include "../filters/moving_average.h"
#include "../filters/kalman.h"

/* ── helpers ──────────────────────────────────────────────────────────── */

#define EPSILON 1e-4f

static int total = 0;
static int passed = 0;

static void check(const char *name, int condition)
{
    total++;
    if (condition) {
        passed++;
        printf("  [PASS] %s\n", name);
    } else {
        printf("  [FAIL] %s\n", name);
    }
}

static int nearly_equal(float a, float b)
{
    return fabsf(a - b) < EPSILON;
}

/* ══════════════════════════════════════════════════════════════════════
 * MOVING AVERAGE TESTS
 * ══════════════════════════════════════════════════════════════════════ */

static void test_ma_init(void)
{
    MovingAverageFilter f;
    uint8_t i;

    printf("\n[Moving Average] Initialisation\n");

    moving_average_init(&f);

    check("sum is 0 after init",   nearly_equal(f.sum,   0.0f));
    check("index is 0 after init", f.index == 0);
    check("count is 0 after init", f.count == 0);

    for (i = 0; i < WINDOW_SIZE; i++) {
        check("buffer slot zeroed", nearly_equal(f.buffer[i], 0.0f));
    }
}

static void test_ma_single_sample(void)
{
    MovingAverageFilter f;
    float result;

    printf("\n[Moving Average] Single sample\n");

    moving_average_init(&f);
    result = moving_average_update(&f, 10.0f);

    check("average of one sample equals that sample",
          nearly_equal(result, 10.0f));
    check("count is 1 after one update", f.count == 1);
}

static void test_ma_fill_phase(void)
{
    MovingAverageFilter f;
    float result;
    uint8_t i;

    printf("\n[Moving Average] Fill phase (before window is full)\n");

    moving_average_init(&f);

    /* Push WINDOW_SIZE values all equal to 4.0 */
    for (i = 0; i < WINDOW_SIZE; i++) {
        result = moving_average_update(&f, 4.0f);
    }

    check("average of identical samples equals that value",
          nearly_equal(result, 4.0f));
    check("count capped at WINDOW_SIZE", f.count == WINDOW_SIZE);

    /* One more sample should not increment count past WINDOW_SIZE */
    moving_average_update(&f, 4.0f);
    check("count stays at WINDOW_SIZE after overflow",
          f.count == WINDOW_SIZE);
}

static void test_ma_sliding_window(void)
{
    MovingAverageFilter f;
    float result;
    uint8_t i;

    printf("\n[Moving Average] Sliding window eviction\n");

    moving_average_init(&f);

    /* Fill with 2.0 */
    for (i = 0; i < WINDOW_SIZE; i++) {
        moving_average_update(&f, 2.0f);
    }

    /* Push WINDOW_SIZE samples of 4.0 – old 2.0 values must be evicted */
    for (i = 0; i < WINDOW_SIZE; i++) {
        result = moving_average_update(&f, 4.0f);
    }

    check("average converges to new value after full window replacement",
          nearly_equal(result, 4.0f));
}

static void test_ma_alternating(void)
{
    MovingAverageFilter f;
    float result;
    uint8_t i;

    printf("\n[Moving Average] Alternating values smoothing\n");

    moving_average_init(&f);

    /* Fill with alternating 0 and 8 → mean should be 4 once window full */
    for (i = 0; i < WINDOW_SIZE; i++) {
        result = moving_average_update(&f, (i % 2 == 0) ? 0.0f : 8.0f);
    }

    check("alternating 0/8 over even window averages to 4.0",
          nearly_equal(result, 4.0f));
}

static void test_ma_negative_values(void)
{
    MovingAverageFilter f;
    float result;
    uint8_t i;

    printf("\n[Moving Average] Negative values\n");

    moving_average_init(&f);

    for (i = 0; i < WINDOW_SIZE; i++) {
        result = moving_average_update(&f, -6.0f);
    }

    check("average of negative samples is correct",
          nearly_equal(result, -6.0f));
}

static void test_ma_null_safety(void)
{
    float result;

    printf("\n[Moving Average] NULL pointer safety\n");

    moving_average_init(NULL);                      /* must not crash */
    result = moving_average_update(NULL, 5.0f);     /* must not crash */

    check("update on NULL returns 0.0f", nearly_equal(result, 0.0f));
}

/* ══════════════════════════════════════════════════════════════════════
 * KALMAN FILTER TESTS
 * ══════════════════════════════════════════════════════════════════════ */

static void test_kf_init(void)
{
    KalmanFilter kf;

    printf("\n[Kalman] Initialisation\n");

    kalman_init(&kf, 5.0f, 1.0f, 0.1f, 0.5f);

    check("initial state stored",       nearly_equal(kf.x, 5.0f));
    check("initial uncertainty stored", nearly_equal(kf.P, 1.0f));
    check("process noise stored",       nearly_equal(kf.Q, 0.1f));
    check("measurement noise stored",   nearly_equal(kf.R, 0.5f));
}

static void test_kf_init_negative_clamping(void)
{
    KalmanFilter kf;

    printf("\n[Kalman] Negative noise clamped to 0\n");

    kalman_init(&kf, 0.0f, -1.0f, -0.5f, -2.0f);

    check("negative P clamped to 0", nearly_equal(kf.P, 0.0f));
    check("negative Q clamped to 0", nearly_equal(kf.Q, 0.0f));
    check("negative R clamped to 0", nearly_equal(kf.R, 0.0f));
}

static void test_kf_exact_measurement(void)
{
    KalmanFilter kf;
    float result;

    printf("\n[Kalman] High confidence measurement dominates\n");

    /* R very small → almost fully trust measurement */
    kalman_init(&kf, 0.0f, 1.0f, 0.0f, 0.0001f);
    result = kalman_update(&kf, 10.0f);

    check("estimate close to measurement when R≈0",
          fabsf(result - 10.0f) < 0.01f);
}

static void test_kf_high_measurement_noise(void)
{
    KalmanFilter kf;
    float result;

    printf("\n[Kalman] Low confidence measurement – model dominates\n");

    /* R very large → almost fully trust model */
    kalman_init(&kf, 5.0f, 0.0001f, 0.0f, 10000.0f);
    result = kalman_update(&kf, 100.0f);

    check("estimate stays close to prior when R is huge",
          fabsf(result - 5.0f) < 1.0f);
}

static void test_kf_convergence(void)
{
    KalmanFilter kf;
    float result = 0.0f;
    int i;
    /* True signal is 20.0; we feed it many times with moderate noise. */
    float true_val = 20.0f;

    printf("\n[Kalman] Convergence over many samples\n");

    kalman_init(&kf, 0.0f, 100.0f, 0.01f, 1.0f);

    for (i = 0; i < 200; i++) {
        result = kalman_update(&kf, true_val);
    }

    check("estimate converges to true value within 0.1",
          fabsf(result - true_val) < 0.1f);
}

static void test_kf_degenerate_zero_noise(void)
{
    KalmanFilter kf;
    float result;

    printf("\n[Kalman] Degenerate case P=0, R=0\n");

    /* Both P and R are 0 → innovation covariance S = 0. */
    kalman_init(&kf, 7.0f, 0.0f, 0.0f, 0.0f);
    result = kalman_update(&kf, 99.0f);

    check("returns existing estimate when S==0",
          nearly_equal(result, 7.0f));
}

static void test_kf_null_safety(void)
{
    float result;

    printf("\n[Kalman] NULL pointer safety\n");

    kalman_init(NULL, 0.0f, 0.0f, 0.0f, 0.0f);   /* must not crash */
    result = kalman_update(NULL, 5.0f);            /* must not crash */

    check("update on NULL returns 0.0f", nearly_equal(result, 0.0f));
}

static void test_kf_uncertainty_decreases(void)
{
    KalmanFilter kf;
    float p_before, p_after;

    printf("\n[Kalman] Uncertainty decreases after update\n");

    kalman_init(&kf, 0.0f, 10.0f, 0.0f, 1.0f);
    p_before = kf.P;
    kalman_update(&kf, 5.0f);
    p_after = kf.P;

    check("P decreases after incorporating a measurement",
          p_after < p_before);
}

/* ══════════════════════════════════════════════════════════════════════
 * MAIN
 * ══════════════════════════════════════════════════════════════════════ */

int main(void)
{
    printf("===========================================\n");
    printf("  ICARUS Sensor Filter Test Suite\n");
    printf("===========================================\n");

    /* Moving Average */
    test_ma_init();
    test_ma_single_sample();
    test_ma_fill_phase();
    test_ma_sliding_window();
    test_ma_alternating();
    test_ma_negative_values();
    test_ma_null_safety();

    /* Kalman */
    test_kf_init();
    test_kf_init_negative_clamping();
    test_kf_exact_measurement();
    test_kf_high_measurement_noise();
    test_kf_convergence();
    test_kf_degenerate_zero_noise();
    test_kf_null_safety();
    test_kf_uncertainty_decreases();

    printf("\n===========================================\n");
    printf("  Results: %d / %d tests passed\n", passed, total);
    printf("===========================================\n");

    return (passed == total) ? 0 : 1;
}
