/* dsp_filters.h : 0.5–5 Hz gait-band filter ------------------------------*/
#ifndef DSP_FILTERS_H
#define DSP_FILTERS_H

#include "arm_math.h"


/* Initialises the filter instances (call once, e.g. from main()). */
void dsp_filters_init(void);

/**
 * Processes one 3-axis acceleration sample.
 *
 * @param ax_g,ay_g,az_g  Acceleration in g units (float).
 * @param hp_out          High-pass ( >0.5 Hz ) magnitude  [optional, may be NULL]
 * @param bp_out          Band-pass (0.5–5 Hz) magnitude   [optional, may be NULL]
 *
 * All outputs are in g units.
 */
void dsp_filters_process(float32_t ax_g, 
                         float32_t ay_g, 
                         float32_t az_g,
                         float32_t *hp_out,
                         float32_t *bp_out);

#endif /* DSP_FILTERS_H */

