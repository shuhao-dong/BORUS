#ifndef GAIT_ANALYSIS_H
#define GAIT_ANALYSIS_H

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"

struct gait_metrics {
    float stride_freq_hz;
    float step_regularity;
    float stride_regularity;
    float step_symmetry;
    uint32_t total_steps;
};

void gait_init(void);
bool gait_feed_sample(float bp_sample_g, struct gait_metrics *out); 

#endif