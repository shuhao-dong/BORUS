#include "dsp_filters.h"
#include <arm_math.h>     /* CMSIS-DSP */
#include <math.h>         /* for sqrtf / optional */

#define NUM_SECTIONS_HP   3      /* order 5 → 3 biquads */
#define NUM_SECTIONS_LP   3

/* ---------- Butterworth coefficients (generated with SciPy) ----------- */
/* Each row: { b0, b1, b2, +a1, +a2 } – CMSIS sign convention             */
/* High-pass 0.5 Hz ------------------------------------------------------*/
static const float32_t hp_coeffs[NUM_SECTIONS_HP][5] = {
    { 0.95043584,      -0.95043584,     0.0f,            0.96906742,    -0.0f           },
    { 1.0f,            -2.0f,           1.0f,            1.94947342,    -0.95043584     },
    { 1.0f,            -2.0f,           1.0f,            1.97979631,    -0.9807737      },
};

/* Low-pass 15 Hz ---------------------------------------------------------*/
static const float32_t lp_coeffs[NUM_SECTIONS_LP][5] = {
    { 0.0069332,       0.01386639,      0.0069332,       0.3249197,     -0.0f           },
    { 1.0f,            2.0f,            1.0f,            0.71052552,    -0.20881821     },
    { 1.0f,            1.0f,            0.0f,            0.9404564,     -0.6            },
};

/* ------------------------------- state -------------------------------- */
static float32_t hp_state[4 * NUM_SECTIONS_HP];
static float32_t lp_state[4 * NUM_SECTIONS_LP];

static arm_biquad_casd_df1_inst_f32 hp_inst;
static arm_biquad_casd_df1_inst_f32 lp_inst;

/* ============================  API  =================================== */
void dsp_filters_init(void)
{
    /* Initialise cascades (one call per filter) */
    arm_biquad_cascade_df1_init_f32(&hp_inst,
                                    NUM_SECTIONS_HP,
                                    (float32_t *)hp_coeffs,
                                    hp_state);

    arm_biquad_cascade_df1_init_f32(&lp_inst,
                                    NUM_SECTIONS_LP,
                                    (float32_t *)lp_coeffs,
                                    lp_state);
}

/* Process ONE 3-axis sample  (must be called at 100 Hz) */
void dsp_filters_process(float32_t ax_g, float32_t ay_g, float32_t az_g,
                         float32_t *hp_out, float32_t *bp_out)
{
    /* 1) orientation-independent magnitude: ‖a‖ */
    float32_t mag;
    arm_sqrt_f32(ax_g * ax_g + ay_g * ay_g + az_g * az_g, &mag);
    /* √ can be swapped to sqrtf() if you prefer */

    /* 2) high-pass >0.5 Hz */
    float32_t hp = mag;
    arm_biquad_cascade_df1_f32(&hp_inst, &hp, &hp, 1);

    /* 3) low-pass  <5 Hz to make the 0.5–5 Hz band */
    float32_t bp = hp;
    arm_biquad_cascade_df1_f32(&lp_inst, &bp, &bp, 1);

    /* 4) optional outputs */
    if (hp_out) *hp_out = hp;
    if (bp_out) *bp_out = bp;
}