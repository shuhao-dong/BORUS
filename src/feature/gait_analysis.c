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

#define MOV_STD_THRESH  0.2f
#define REG_THRESH  0.5f

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

/* Stats that need to persist */
static uint32_t total_steps;

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

static uint32_t peak_detect(const float *x, uint32_t N, float height, uint32_t min_dist)
{
    uint32_t count = 0, last = 0;
    for (uint32_t i = 1; i < N - 1; i++)
    {
        if (x[i] > height && x[i] > x[i - 1] && x[i] >= x[i + 1])
        {
            if (i - last >= min_dist) 
            {
                count ++; 
                last = i;
            }
        }
    }
    return count; 
}

/* ---------------------------- API --------------------------------- */
void gait_init(void)
{
    ring_buf_reset(&gait_rb);
    arm_rfft_fast_init_f32(&fft_inst, NFFT);
    total_steps = 0;
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

    // Safety check if we ever reach to InF
    for (size_t i = 0; i < WIN_SAMPLES; i++)
    {
        if (!isfinite(win_buf[i]))
        {
            LOG_ERR("NaN/Inf at i=%zu, raw=0x%08X", i,
                    (unsigned int)*(uint32_t *)&win_buf[i]);
            goto slide; /* discard the window */
        }
    }

    /* 4. Process window */
    // Calculate mean, variance, standard deviation 
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

    // Calculate FFT
    memcpy(fft_in, win_buf, WIN_SAMPLES * sizeof(float)); 
    memset(fft_in + WIN_SAMPLES, 0, (NFFT - WIN_SAMPLES) * sizeof(float));
    arm_rfft_fast_f32(&fft_inst, fft_in, fft_out, 0); 

    for (uint32_t i = 0; i < NFFT; i += 2)
    {
        fft_out[i >> 1] = hypotf(fft_out[i], fft_out[i + 1]);
    }

    // Find the dominant (max) frequency in this window
    float max_val;
    uint32_t max_idx;
    uint32_t half = NFFT / 2;
    arm_max_f32(fft_out + 1, half - 1, &max_val, &max_idx); 
    float stride_freq = (max_idx * FS_HZ) / (float)NFFT;
    LOG_WRN("Found stride frequency: %.2f", (double)stride_freq); 
    // If frequency is not between 0.5 and 3, this is not a walking bout
    if (stride_freq < 0.5f || stride_freq > 3.0f)
    {   
        LOG_WRN("Frequency not in range");
        goto slide;
    }

    // Calculate step regularity and stride regularity 
    float step_reg = autocorr_lag(win_buf, WIN_SAMPLES, (uint32_t)(FS_HZ / (stride_freq * 2)));
    float stride_reg = autocorr_lag(win_buf, WIN_SAMPLES, (uint32_t)(FS_HZ / stride_freq));
    // If step regularity is below the threshold, this is not a walking bout
    LOG_WRN("Found step regularity: %.2f", (double)fabsf(step_reg)); 
    if (fabsf(step_reg) < REG_THRESH)
    {   
        LOG_WRN("Step regularity not in range"); 
        goto slide; 
    }

    uint32_t min_dist = FS_HZ / 3;
    uint32_t pos = peak_detect(win_buf, WIN_SAMPLES, 0.15f, min_dist);
    uint32_t neg = peak_detect(win_buf, WIN_SAMPLES, -0.15f, min_dist); 
    uint32_t win_steps = pos + neg; 
    total_steps += win_steps;
    
    m->stride_freq_hz = stride_freq;
    m->step_regularity = fabsf(step_reg);
    m->stride_regularity = stride_reg;
    m->step_symmetry = step_reg / stride_freq; 
    m->total_steps = total_steps; 
    
slide:
    /* 5. Drop the oldest 100 samples (1-second hop) */
    uint8_t dummy[HOP_SAMPLES * BYTES_PER_SAMPLE];
    ring_buf_get(&gait_rb, dummy, sizeof(dummy));

    return true;
}