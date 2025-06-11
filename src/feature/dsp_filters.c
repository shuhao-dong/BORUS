/* dsp_filters.c : 0.5–5 Hz gait-band filter -----------------------------*/
#include "dsp_filters.h"
#include <math.h>

/* ---------- filter coefficients (Butterworth, fs = 100 Hz) ------------- */
/* Design done in Python / SciPy:
   sos = butter(2, 0.5, 'hp', fs=100, output='sos')
   sos = butter(2, 5.0, 'lp', fs=100, output='sos')
   Denominator signs inverted for CMSIS-DSP (a1, a2 positive).            */

/* 2-pole high-pass 0.5 Hz */
static const float32_t hp_coeffs[5] = {
    0.97803048f, -1.95606096f, 0.97803048f, 1.95557824f, -0.95654368f
};
static float32_t hp_state[4];

static arm_biquad_casd_df1_inst_f32 hp_inst; /* 1 stage */

/* 2-pole low-pass  5 Hz  */
static const float32_t lp_coeffs[5] = {
    0.02008337f, 0.04016673f, 0.02008337f, 1.56101808f, -0.64135154f
};
static float32_t lp_state[4];

static arm_biquad_casd_df1_inst_f32 lp_inst; /* 1 stage */

/* --------------------------- public API -------------------------------- */
void dsp_filters_init(void)
{
    arm_biquad_cascade_df1_init_f32(&hp_inst, 1, (float32_t *)hp_coeffs, hp_state);
    arm_biquad_cascade_df1_init_f32(&lp_inst, 1, (float32_t *)lp_coeffs, lp_state);
}

void dsp_filters_process(float32_t ax_g, float32_t ay_g, float32_t az_g,
                            float32_t *hp_out, float32_t *bp_out)
{
    /* 1) orientation-independent magnitude a */
    float32_t mag; 
    arm_sqrt_f32(ax_g * ax_g + ay_g * ay_g + az_g * az_g, &mag);
    
    /* 2) high-pass > 0.5 Hz */
    float32_t hp = mag;
    arm_biquad_cascade_df1_f32(&hp_inst, &hp, &hp, 1);

    /* 3) low-pass < 5 Hz */
    float32_t bp = hp;
    arm_biquad_cascade_df1_f32(&lp_inst, &bp, &bp, 1); 

    if (hp_out)
    {
        *hp_out = hp;
    }

    if (bp_out)
    {
        *bp_out = bp; 
    }
}