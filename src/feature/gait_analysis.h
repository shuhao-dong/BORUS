#ifndef GAIT_ANALYSIS_H
#define GAIT_ANALYSIS_H

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"

/* One walking-bout’s quality measures */
struct gait_metrics {
    bool is_walking;            /* flag to indicate if walking now   */
    float cadence_spm;          /* steps / minute                    */
    float dom_freq_hz;          /* dominant PSD frequency            */
    float range_g;              /* max-min SVM magnitude (g)         */
    float rms_g;                /* RMS magnitude (g)                 */
    float dom_amp;              /* amplitude @ dominant freq         */
    float dom_width_hz;         /* width @ ½ amplitude               */
    float step_regularity;      /* AC peak 1                         */
    float stride_regularity;    /* AC peak 2                         */
    float step_time_var_pct;    /* coefficient of variation (%)      */
    uint32_t total_steps;       /* total steps since power up        */
    float skewness;             /* skewness of the 6-second window   */
    float kurtosis;             /* kurtosis of the 6-second window   */
    float std_diff1;            /* SD of the first derivative        */
    float std_diff2;            /* SD of the second derivative       */
};

void gait_init(void);
bool gait_feed_sample(float bp_sample_g, struct gait_metrics *m); 

#endif