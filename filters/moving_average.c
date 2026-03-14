#include "moving_average.h"
#include <stddef.h>  /* NULL */

/* -----------------------------------------------------------------------
 * moving_average_init
 * Zero-fill the buffer, reset the write index, running sum, and sample
 * counter.  Must be called once before any moving_average_update call.
 * ----------------------------------------------------------------------- */
void moving_average_init(MovingAverageFilter *filter)
{
    uint8_t i;

    if (filter == NULL) {
        return;
    }

    for (i = 0; i < WINDOW_SIZE; i++) {
        filter->buffer[i] = 0.0f;
    }
    filter->index = 0;
    filter->sum   = 0.0f;
    filter->count = 0;
}

/* -----------------------------------------------------------------------
 * moving_average_update
 *
 * Algorithm (O(1) per sample):
 *   1. Subtract the oldest slot from the running sum.
 *   2. Overwrite that slot with the new sample.
 *   3. Add the new sample to the running sum.
 *   4. Advance the circular index with wrap-around.
 *   5. Divide by the actual number of samples seen (fill phase) or
 *      by WINDOW_SIZE once the buffer is full.
 *
 * No loops, no re-summation → strictly O(1).
 * ----------------------------------------------------------------------- */
float moving_average_update(MovingAverageFilter *filter, float sample)
{
    if (filter == NULL) {
        return 0.0f;
    }

    /* Step 1 – remove contribution of the slot about to be overwritten */
    filter->sum -= filter->buffer[filter->index];

    /* Step 2 – write new sample into the circular buffer */
    filter->buffer[filter->index] = sample;

    /* Step 3 – add new sample to running sum */
    filter->sum += sample;

    /* Step 4 – advance write head (modular arithmetic, no branch needed) */
    filter->index = (uint8_t)((filter->index + 1u) % WINDOW_SIZE);

    /* Step 5 – track fill level so early outputs are unbiased */
    if (filter->count < WINDOW_SIZE) {
        filter->count++;
    }

    /* Guard against divide-by-zero (only possible if WINDOW_SIZE == 0,
     * which the preprocessor should catch, but we defend anyway). */
    if (filter->count == 0u) {
        return 0.0f;
    }

    return filter->sum / (float)filter->count;
}
