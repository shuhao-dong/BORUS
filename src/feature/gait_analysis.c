#include "gait_analysis.h"
#include <arm_math.h>
#include <string.h>
#include <math.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(GAIT, LOG_LEVEL_INF);

/* Window parameter */
#define FS_HZ 100
#define WIN_SECONDS 6
#define HOP_SECONDS 1
#define WIN_SAMPLES (FS_HZ * WIN_SECONDS) // 600
#define HOP_SAMPLES (FS_HZ * HOP_SECONDS) // 100

/* Movement threshold */
#define MOV_STD_THRESH 0.1f
#define REG_THRESH 0.15f

/* Debounce: allow short gaps before ending a walking bout (seconds) */
#define BOUT_END_DEBOUNCE 2

/* RING BUFFER: stores raw band-pass floats as bytes */
#define BYTES_PER_SAMPLE sizeof(float) // 4
#define RB_CAPACITY_BYTES (WIN_SAMPLES * BYTES_PER_SAMPLE + BYTES_PER_SAMPLE)

/* Create ring buffer */
RING_BUF_DECLARE(gait_rb, RB_CAPACITY_BYTES);

/* Workspace for each 6-s FFT window */
#define NFFT 1024
static float win_buf[WIN_SAMPLES];
static float fft_in[NFFT];
static float fft_out[NFFT];
static arm_rfft_fast_instance_f32 fft_inst;

/* Hold total steps integer and total steps float */
static uint32_t g_total_steps = 0;
static float g_total_steps_f = 0.0f;
static uint32_t g_total_steps_weighted = 0;
static float g_total_steps_f_weighted = 0.0f; 

/* Step-time stats for the *current* walking bout (Welford) */
static bool bout_active = false;
static uint32_t bout_n = 0;
static float bout_mean = 0.0f;
static float bout_M2 = 0.0f;

/* Count of consecutive non-walking windows */
static uint32_t gap_count = 0;

/* Hold derivative values */
static float d1_buf[WIN_SAMPLES - 1];
static float d2_buf[WIN_SAMPLES - 2];

/* ------------------ Helpers --------------------------- */

/**
 * @brief Count the number of events (peaks or valleys) in a window of data.
 *
 * @param x Input array of floats.
 * @param N Number of elements in the array.
 * @param height Minimum height for an event to be counted.
 * @param min_dist Minimum distance between consecutive events.
 *
 * @return uint32_t Number of events detected in the window.
 */
static uint32_t count_events_in_window(const float *x, uint32_t N, float height, uint32_t min_dist)
{
    uint32_t count = 0;
    uint32_t last_event_idx = -min_dist;

    for (uint32_t i = 1; i < N - 1; i++) {
        // Look for a positive peak OR a negative valley
        if ((x[i] > height && x[i] > x[i - 1] && x[i] >= x[i + 1]) ||
            (x[i] < -height && x[i] < x[i - 1] && x[i] <= x[i + 1])) {
            
            if (i - last_event_idx >= min_dist) {
                count++;
                last_event_idx = i;
            }
        }
    }
    return count;
}

/**
 * @brief Calculate skewness and kurtosis of a float array.
 *
 * @param x Input array of floats.
 * @param N Number of elements in the array.
 * @param skew Pointer to store the skewness result.
 * @param kurt Pointer to store the kurtosis result.
 *
 */
static void stats_skew_kurt(const float *x, uint32_t N,
                            float *skew, float *kurt)
{
    float sum = 0.0f, sum2 = 0.0f, sum3 = 0.0f, sum4 = 0.0f;
    for (uint32_t i = 0; i < N; i++)
    {
        sum += x[i];
        sum2 += x[i] * x[i];
    }
    float mean = sum / N;
    float var = (sum2 / N) - mean * mean;
    float sd = sqrtf(var);

    for (uint32_t i = 0; i < N; i++)
    {
        float d = (x[i] - mean) / sd;
        float d2 = d * d;
        sum3 += d * d2;  /* (x-μ)^3 / σ^3 */
        sum4 += d2 * d2; /* (x-μ)^4 / σ^4 */
    }
    *skew = sum3 / N;
    *kurt = sum4 / N - 3.0f; /* excess kurtosis (Pearson) */
}

/**
 * @brief Calculate the first-order difference of a float array.
 *
 * @param in Input array of floats.
 * @param out Output array to store the differences.
 * @param N Number of elements in the input array.
 */
static void diff_f32(const float *in, float *out, uint32_t N)
{
    for (uint32_t i = 0; i < N - 1; i++)
    {
        out[i] = in[i + 1] - in[i];
    }
}

/**
 * @brief Calculate the autocorrelation at lag k for a float array.
 *
 * @param x Input array of floats.
 * @param N Number of elements in the array.
 * @param k Lag value for autocorrelation.
 *
 * @return float Autocorrelation value at lag k.
 */
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

/**
 * @brief Calculate the width of the dominant frequency peak at half its amplitude.
 *
 * @param mag Magnitude spectrum array.
 * @param p Index of the peak in the magnitude spectrum.
 * @param len Length of the magnitude spectrum array.
 *
 * @return float Width of the peak at half its amplitude in Hz.
 */
static float width_halfamp(const float *mag, uint32_t p, uint32_t len)
{
    const float h = mag[p] * 0.5f;
    uint32_t l = p, r = p;
    while (l > 0 && mag[l] > h)
    {
        l--;
    }
    while (r < len - 1 && mag[r] > h)
    {
        r++;
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
    g_total_steps_weighted = 0;
    g_total_steps_f_weighted = 0.0f; 
    bout_active = false;
    bout_n = 0;
    bout_mean = 0.0f;
    bout_M2 = 0.0f;
    gap_count = 0;
}

bool gait_analyse(float bp_sample_g, struct gait_metrics *m)
{
    bool is_walking = true;

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

    if (std < MOV_STD_THRESH)
    {
        LOG_WRN("No significant movement detected");
        is_walking = false;
        goto slide;
    }

    /* 5. FFT gate - dominant frequency (step frequency since using magnitude, negative part flipped up to positive) 0.5-3 Hz */
    memcpy(fft_in, win_buf, WIN_SAMPLES * sizeof(float));
    memset(fft_in + WIN_SAMPLES, 0, (NFFT - WIN_SAMPLES) * sizeof(float));
    arm_rfft_fast_f32(&fft_inst, fft_in, fft_out, 0);

    // magnitude spectrum (complex to real)
    for (uint32_t i = 0; i < NFFT; i += 2)
    {
        fft_out[i >> 1] = hypotf(fft_out[i], fft_out[i + 1]);
    }

    uint32_t half = NFFT / 2; /* Nyquist */
    float peak_val;
    uint32_t peak_idx;

    arm_max_f32(fft_out + 1, half - 1, &peak_val, &peak_idx);
    float step_freq = (peak_idx * FS_HZ) / (float)NFFT;

    if (step_freq < 0.5f || step_freq > 3.0f)
    {
        LOG_WRN("Frequency not in range");
        is_walking = false;
        goto slide;
    }

    /* 6. Regularity gate - stride regularity > @ref REG_THRESH */
    float step_reg = autocorr_lag(win_buf, WIN_SAMPLES, (uint32_t)(FS_HZ / step_freq));
    float stride_reg = autocorr_lag(win_buf, WIN_SAMPLES, (uint32_t)(FS_HZ / (step_freq / 2)));

    if (fabsf(stride_reg) < REG_THRESH)
    {
        LOG_WRN("Step regularity not in range");
        is_walking = false;
        goto slide;
    }

    /* -------------------- Passed all gates, calculate features below -------------------- */

    /* 7.1. Skewness, kurtosis, standard deviation of 1st and 2nd derivatives */
    float skew, kurt;
    stats_skew_kurt(win_buf, WIN_SAMPLES, &skew, &kurt);

    diff_f32(win_buf, d1_buf, WIN_SAMPLES);
    diff_f32(d1_buf, d2_buf, WIN_SAMPLES - 1);

    float var_d1, var_d2;
    arm_var_f32(d1_buf, WIN_SAMPLES - 1, &var_d1);
    arm_var_f32(d2_buf, WIN_SAMPLES - 2, &var_d2);

    float std_d1 = sqrtf(var_d1);
    float std_d2 = sqrtf(var_d2);

    /* 7.2. Ryhthm: cadence & step frequency */
    float cadence_spm = step_freq * 60.0f;

    /* 7.3. Magnitude: range and RMS */
    float min_g, max_g, rms_g;
    uint32_t idx_not_use;
    arm_min_f32(win_buf, WIN_SAMPLES, &min_g, &idx_not_use);
    arm_max_f32(win_buf, WIN_SAMPLES, &max_g, &idx_not_use);

    float range_g = max_g - min_g;
    arm_rms_f32(win_buf, WIN_SAMPLES, &rms_g);

    /* 7.4. Regularity / Consistency:
        amplitude @step frequency,
        width @step frequency,
        step regularity,
        stride regularity,
        step time variability*/
    float dom_width = width_halfamp(fft_out, peak_idx, half);
    float step_time = 1.0f / step_freq;
    
    if (!bout_active)
    {
        bout_active = true;
        bout_n = 0;
        bout_mean = 0.0f;
        bout_M2 = 0.0f;
    }
    bout_n++;
    float delta = step_time - bout_mean;
    bout_mean += delta / bout_n;
    bout_M2 += delta * (step_time - bout_mean);

    float step_cv = 0.0f;
    if (bout_n > 1)
    {
        float sd = sqrtf(bout_M2 / (bout_n - 1));
        step_cv = 100.0f * sd / bout_mean;
    }

    /* 7.5. Quantity: total steps */
    
    // Estimation 1: From FFT ryhthm, good for steady cadence
    float fft_steps_in_hop = step_freq * HOP_SECONDS;

    // Estimation 2: From direct peak and valley finding
    const uint32_t start_offset = WIN_SAMPLES - HOP_SAMPLES;
    const uint32_t min_dist = (uint32_t)(FS_HZ / 6.0f);
    const float peak_height = 0.15f * std; 

    uint32_t total_events_in_hop = count_events_in_window(win_buf + start_offset, HOP_SAMPLES, peak_height, min_dist);
    float counted_steps_in_hop = total_events_in_hop / 2.0f;

    const float REG_LOW_CONF = REG_THRESH;
    const float REG_HIGH_CONF = 0.9f; 
    const float WIDTH_LOW_CONF = 0.15f;
    const float WIDTH_HIGH_CONF = 0.9f;

    // Weight from Stride Regularity (scales 0-1)
    float w_reg = (stride_reg - REG_LOW_CONF) / (REG_HIGH_CONF - REG_LOW_CONF);
    w_reg = fmaxf(0.0f, fminf(1.0f, w_reg)); // Clamp between 0.0 and 1.0

    // Weight from FFT Peak Width (inverted scale, 0-1)
    float w_width = (WIDTH_HIGH_CONF - dom_width) / (WIDTH_HIGH_CONF - WIDTH_LOW_CONF);
    w_width = fmaxf(0.0f, fminf(1.0f, w_width)); // Clamp between 0.0 and 1.0

    float inv_reg = 1.0f - w_reg;
    float inv_width = 1.0f - w_width;
    float w = 1.0f - sqrtf(inv_reg * inv_width);

    // When w=1 (high confidence), we trust the stable FFT estimate.
    // When w=0 (low confidence), we trust the precise time-domain count.
    float fused_steps = (w * fft_steps_in_hop) + ((1.0f - w) * counted_steps_in_hop);

    g_total_steps_f_weighted += fused_steps;
    g_total_steps_weighted = (uint32_t)ceilf(g_total_steps_f_weighted); 

    g_total_steps_f += step_freq * HOP_SECONDS;
    g_total_steps = (uint32_t)roundf(g_total_steps_f);

    LOG_WRN("Regularity: %.2f, Width: %.2f, S= %d, Fusion: FFT=%.1f, Count=%.1f -> w_reg=%.2f, w_width=%.2f -> w=%.2f -> Fused=%d",
            (double)step_reg, (double)dom_width, g_total_steps,
            (double)fft_steps_in_hop, (double)counted_steps_in_hop,
            (double)w_reg, (double)w_width, (double)w, g_total_steps_weighted);

    if (m)
    {
        m->is_walking = is_walking;
        m->cadence_spm = cadence_spm;
        m->dom_freq_hz = step_freq;
        m->range_g = range_g;
        m->rms_g = rms_g;
        m->dom_amp = peak_val;
        m->dom_width_hz = dom_width;
        m->step_regularity = fabsf(step_reg);
        m->stride_regularity = stride_reg;
        m->step_time_var_pct = step_cv;
        m->total_steps = g_total_steps;
        m->total_steps_weighted = g_total_steps_weighted; 
        m->kurtosis = kurt;
        m->skewness = skew;
        m->std_diff1 = std_d1;
        m->std_diff2 = std_d2;
    }

slide:
    /* 8. Drop the oldest 100 samples (1-second hop) */
    ring_buf_get(&gait_rb, NULL, HOP_SAMPLES * BYTES_PER_SAMPLE);

    /* Debounce short gaps between walking bouts */
    if (!is_walking) {
        if (gap_count < BOUT_END_DEBOUNCE) {
            is_walking = true;
            gap_count++;
        } else {
            if (bout_active) {
                bout_active = false;
                bout_n = 0;
                bout_mean = 0.0f;
                bout_M2 = 0.0f;
            }
        }
    } else {
        gap_count = 0;
    }

    if (m) {
        m->is_walking = is_walking;
    }

    // Only return when walking
    return m;
}