#include "gait_analysis.h"
#include <arm_math.h>
#include <string.h>
#include <math.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(GAIT, LOG_LEVEL_DBG); 

/* ---------------- parameters ----------------------------------------- */
#define FS_HZ   100
#define WIN_SECONDS 6
#define HOP_SECONDS 1
#define WIN_SAMPLES (FS_HZ * WIN_SECONDS) // 600
#define HOP_SAMPLES (FS_HZ * HOP_SECONDS) // 100

#define MOV_STD_THRESH  0.1f
#define REG_THRESH  0.4f

/* RING BUFFER: stores raw band-pass floats as bytes                  */
#define BYTES_PER_SAMPLE    sizeof(float) // 4
#define RB_CAPACITY_BYTES   (WIN_SAMPLES * BYTES_PER_SAMPLE + BYTES_PER_SAMPLE)

RING_BUF_DECLARE(gait_rb, RB_CAPACITY_BYTES);

/* Workspace for each 6-s FFT window */
#define NFFT 1024
static float win_buf[WIN_SAMPLES];
static float fft_in[NFFT];
static float fft_out[NFFT];
static arm_rfft_fast_instance_f32 fft_inst; 

static uint32_t g_total_steps = 0;
static float g_total_steps_f = 0.0f; 

/* step-time stats for the *current* walking bout (Welford) */
static bool     bout_active = false;
static uint32_t bout_n      = 0;
static float    bout_mean   = 0.0f;
static float    bout_M2     = 0.0f;

/* ------------------ helpers --------------------------- */
static float autocorr_lag(const float *x, uint32_t N, uint32_t k)
{
    float num = 0, den = 0;
    for (uint32_t i = 0; i < N - k; i++)
    {
        num += x[i] * x[i + k]; 
    }
    arm_power_f32(x, N, &den);
    return num / den; 
}

static float width_halfamp(const float *mag, uint32_t p, uint32_t len)
{
    const float h = mag[p] * 0.5f;
    uint32_t l = p, r = p;
    while (l > 0 && mag[l] > h) 
    {
        l --;
    }
    while (r < len - 1 && mag[r] > h) 
    {
        r ++;
    }
    return (r - l) * (FS_HZ / (float)NFFT); 
}

/* ---------------------------- API --------------------------------- */
void gait_init(void)
{
    ring_buf_reset(&gait_rb);
    arm_rfft_fast_init_f32(&fft_inst, NFFT);
    g_total_steps = 0; 
    g_total_steps_f = 0.0f; 
    bout_active     = false;
    bout_n          = 0;
    bout_mean       = 0.0f;
    bout_M2         = 0.0f;
}

bool gait_feed_sample(float bp_sample_g, struct gait_metrics *m)
{
    /* 1. Push sample (as bytes) */
    ring_buf_put(&gait_rb, (uint8_t *)&bp_sample_g, BYTES_PER_SAMPLE); 

    /* 2. Check if have 6s window full */
    if (ring_buf_size_get(&gait_rb) < WIN_SAMPLES * BYTES_PER_SAMPLE)
    {
        return false;
    }

    /* 3. Copy 600 samples into win_buf without removing them */
    size_t copied = 0;
    size_t need = WIN_SAMPLES * BYTES_PER_SAMPLE;
    while (copied < need)
    {
        uint8_t *blk;
        /* ask for “as much as is still missing” */
        size_t blk_len = ring_buf_get_claim(&gait_rb, &blk, need - copied);
        if (!blk_len)
        { /* should never happen, but be safe */
            LOG_ERR("ring_buf_get_claim returned 0");
            return false;
        }
        memcpy((uint8_t *)win_buf + copied, blk, blk_len);
        ring_buf_get_finish(&gait_rb, 0); /* keep data (we are just peeking) */
        copied += blk_len;
    }

    /* 4. Movement gate - std dev > 0.10 g */ 
    float mean, var;
    arm_mean_f32(win_buf, WIN_SAMPLES, &mean); 
    arm_var_f32(win_buf, WIN_SAMPLES, &var);
    float std = sqrtf(var);
    LOG_WRN("Standard deviation is: %.2f", (double)std); 
    if (std < MOV_STD_THRESH)
    {   
        LOG_WRN("No significant movement detected"); 
        goto slide; 
    }

    /* 5. FFT gate - dominant frequency 0.5-3 Hz */
    memcpy(fft_in, win_buf, WIN_SAMPLES * sizeof(float)); 
    memset(fft_in + WIN_SAMPLES, 0, (NFFT - WIN_SAMPLES) * sizeof(float));
    arm_rfft_fast_f32(&fft_inst, fft_in, fft_out, 0); 

    /* magnitude spectrum */
    for (uint32_t i = 0; i < NFFT; i += 2)
    {
        fft_out[i >> 1] = hypotf(fft_out[i], fft_out[i + 1]);
    }

    uint32_t half = NFFT / 2;   /* Nyquist */
    float peak_val;
    uint32_t peak_idx;
    arm_max_f32(fft_out + 1, half - 1, &peak_val, &peak_idx); 

    float dom_freq = (peak_idx * FS_HZ) / (float)NFFT;

    LOG_WRN("Found stride frequency: %.2f", (double)dom_freq); 
    if (dom_freq < 0.5f || dom_freq > 3.0f)
    {   
        LOG_WRN("Frequency not in range");
        goto slide;
    }

    /* 6. Regularity gate - step auto-correlation peak > 0.15 */
    float step_reg = autocorr_lag(win_buf, WIN_SAMPLES, (uint32_t)(FS_HZ / (dom_freq * 2)));
    LOG_WRN("Found step regularity: %.2f", (double)fabsf(step_reg)); 

    if (fabsf(step_reg) < REG_THRESH)
    {   
        LOG_WRN("Step regularity not in range"); 
        goto slide; 
    }

    float stride_reg = autocorr_lag(win_buf, WIN_SAMPLES, (uint32_t)(FS_HZ / dom_freq));

    /* ---------- bout metrics -------------------------------------- */

    /* cadence & step count from dominant frequency */
    static float f_lp = 0;
    float step_freq = dom_freq * 2.0f; 

    if (f_lp == 0.0f)
    {
        f_lp = step_freq;
    } else
    {
        f_lp = 0.8f * f_lp + 0.2f * step_freq;
    }
    
    float cadence_spm = f_lp * 60.0f; 
    float step_time = 1.0f / f_lp;

    if (!bout_active)
    {
        /* new bout starts */
        bout_active = true;
        bout_n = 0;
        bout_mean = 0.0f;
        bout_M2 = 0.0f;
    }
    bout_n++;
    float delta = step_time - bout_mean;
    bout_mean += delta / bout_n;
    bout_M2 += delta * (step_time - bout_mean);

    /* coefficient of variation so far */
    float step_cv = 0.0f;
    if (bout_n > 1)
    {
        float sd = sqrtf(bout_M2 / (bout_n - 1));
        step_cv = 100.0f * sd / bout_mean;
    } 
    
    /* amplitude, width, range, RMS */
    float dom_width = width_halfamp(fft_out, peak_idx, half);

    float min_g, max_g;
    uint32_t idx_not_use;
    arm_min_f32(win_buf, WIN_SAMPLES, &min_g, &idx_not_use);
    arm_max_f32(win_buf, WIN_SAMPLES, &max_g, &idx_not_use);
    float range_g = max_g - min_g;

    float rms_g;
    arm_rms_f32(win_buf, WIN_SAMPLES, &rms_g);

    g_total_steps_f += f_lp * HOP_SECONDS;
    g_total_steps = (uint32_t)roundf(g_total_steps_f); 
    
    if (m)
    {
        m->cadence_spm = cadence_spm;
        m->dom_freq_hz = dom_freq;
        m->range_g = range_g;
        m->rms_g = rms_g;
        m->dom_amp = peak_val; 
        m->dom_width_hz = dom_width;
        m->step_regularity = step_reg;
        m->stride_regularity = stride_reg;
        m->step_time_var_pct = step_cv;
        m->total_steps = g_total_steps;
    }
    
slide:
    /* 5. Drop the oldest 100 samples (1-second hop) */
    uint8_t dummy[HOP_SAMPLES * BYTES_PER_SAMPLE];
    ring_buf_get(&gait_rb, dummy, sizeof(dummy));

    return m;
}