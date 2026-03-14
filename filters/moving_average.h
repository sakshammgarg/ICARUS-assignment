#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include <stdint.h>

#define WINDOW_SIZE 8

/*
 * MovingAverageFilter
 *
 * buffer : circular buffer holding the last WINDOW_SIZE samples
 * index  : write-head pointing to the oldest slot (next to overwrite)
 * sum    : running sum – updated in O(1) each call
 * count  : number of samples seen so far, capped at WINDOW_SIZE;
 *          used to compute the correct divisor during the fill phase
 *          so the filter is unbiased before the window is full
 */
typedef struct {
    float   buffer[WINDOW_SIZE];
    uint8_t index;
    float   sum;
    uint8_t count;
} MovingAverageFilter;

/*
 * moving_average_init – zero-initialise the filter.
 * Must be called before the first moving_average_update.
 */
void  moving_average_init(MovingAverageFilter *filter);

/*
 * moving_average_update – push one sample and return the current average.
 * Time complexity : O(1)
 * Returns 0.0f if filter is NULL.
 */
float moving_average_update(MovingAverageFilter *filter, float sample);

#endif /* MOVING_AVERAGE_H */
