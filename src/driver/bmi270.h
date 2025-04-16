/**
 * Copyright (c) 2021 Bosch Sensortec GmbH
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 *  @brief Driver implementation for BMP390
 *  This is the driver for TORUS bmp390 implementation using zephyr i2c API 
 * 
 *  @file bmp390.h
 *  @date 2024-11-22
 *  @version v1.0
 * 
 *  @author Shuhao Dong <shuhao.dong@bristol.ac.uk>
 * 
 *  @cite https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/sensor/bosch/bmi270/bmi270.h
 */

#ifndef BMI270_H_
#define BMI270_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>

typedef struct {
    const struct spi_dt_spec *spec;
} BMI270_Context;

extern const uint8_t bmi270_config_file[]; 

/* Register map for BMI270 */
#define BMI270_REG_CHIP_ID         0x00
#define BMI270_REG_ERROR           0x02
#define BMI270_REG_STATUS          0x03
#define BMI270_REG_AUX_X_LSB       0x04
#define BMI270_REG_ACC_X_LSB       0x0C
#define BMI270_REG_GYR_X_LSB       0x12
#define BMI270_REG_SENSORTIME_0    0x18
#define BMI270_REG_EVENT           0x1B
#define BMI270_REG_INT_STATUS_0    0x1C
#define BMI270_REG_SC_OUT_0        0x1E
#define BMI270_REG_WR_GEST_ACT     0x20
#define BMI270_REG_INTERNAL_STATUS 0x21
#define BMI270_REG_TEMPERATURE_0   0x22
#define BMI270_REG_FIFO_LENGTH_0   0x24
#define BMI270_REG_FIFO_DATA       0x26
#define BMI270_REG_FEAT_PAGE       0x2F
#define BMI270_REG_FEATURES_0      0x30
#define BMI270_REG_ACC_CONF        0x40
#define BMI270_REG_ACC_RANGE       0x41
#define BMI270_REG_GYR_CONF        0x42
#define BMI270_REG_GYR_RANGE       0x43
#define BMI270_REG_AUX_CONF        0x44
#define BMI270_REG_FIFO_DOWNS      0x45
#define BMI270_REG_FIFO_WTM_0      0x46
#define BMI270_REG_FIFO_CONFIG_0   0x48
#define BMI270_REG_SATURATION      0x4A
#define BMI270_REG_AUX_DEV_ID      0x4B
#define BMI270_REG_AUX_IF_CONF     0x4C
#define BMI270_REG_AUX_RD_ADDR     0x4D
#define BMI270_REG_AUX_WR_ADDR     0x4E
#define BMI270_REG_AUX_WR_DATA     0x4F
#define BMI270_REG_ERR_REG_MSK     0x52
#define BMI270_REG_INT1_IO_CTRL    0x53
#define BMI270_REG_INT2_IO_CTRL    0x54
#define BMI270_REG_INT_LATCH       0x55
#define BMI270_REG_INT1_MAP_FEAT   0x56
#define BMI270_REG_INT2_MAP_FEAT   0x57
#define BMI270_REG_INT_MAP_DATA    0x58
#define BMI270_REG_INIT_CTRL       0x59
#define BMI270_REG_INIT_ADDR_0     0x5B
#define BMI270_REG_INIT_DATA       0x5E
#define BMI270_REG_INTERNAL_ERROR  0x5F
#define BMI270_REG_AUX_IF_TRIM     0x68
#define BMI270_REG_GYR_CRT_CONF    0x69
#define BMI270_REG_NVM_CONF        0x6A
#define BMI270_REG_IF_CONF         0x6B
#define BMI270_REG_DRV             0x6C
#define BMI270_REG_ACC_SELF_TEST   0x6D
#define BMI270_REG_GYR_SELF_TEST   0x6E
#define BMI270_REG_NV_CONF         0x70
#define BMI270_REG_OFFSET_0        0x71
#define BMI270_REG_PWR_CONF        0x7C
#define BMI270_REG_PWR_CTRL        0x7D
#define BMI270_REG_CMD             0x7E

#define CHIP_ID_BMI270      0x24

// Power mode configuration @BMI270_REG_PWR_CONF 0x7C
#define BMI270_ADV_POWER_SAVE_EN            UINT8_C(1 << 0) // Advanced power save mode enabled
#define BMI270_ADV_POWER_SAVE_DIS           UINT8_C(0 << 0) // Advanced power save mode disabled
#define BMI270_FIFO_SELF_WAKE_UP_OFF        UINT8_C(0 << 1) // FIFO read disabled in low power mode
#define BMI270_FIFO_SELF_WAKE_UP_ON         UINT8_C(1 << 1) // FIFO read enabled in low power mode
#define BMI270_FAST_POWER_UP_EN             UINT8_C(1 << 2) // Fast power up enabled
#define BMI270_FAST_POWER_UP_DIS            UINT8_C(0 << 2) // Fast power up disabled

// Power mode control @BMI270_REG_PWR_CTRL 0x7D
#define BMI270_AUX_EN_ON                    UINT8_C(1 << 0) // Enable the auxiliary sensor
#define BMI270_AUX_EN_OFF                   UINT8_C(0 << 0) // Disable the auxiliary sensor
#define BMI270_GYR_EN_ON                    UINT8_C(1 << 1) // Enable the gyroscope
#define BMI270_GYR_EN_OFF                   UINT8_C(0 << 1) // Disable the gyroscope
#define BMI270_ACC_EN_ON                    UINT8_C(1 << 2) // Enable the accelerometer
#define BMI270_ACC_EN_OFF                   UINT8_C(0 << 2) // Disable the accelerometer
#define BMI270_TEMP_EN_ON                   UINT8_C(1 << 3) // Enable the temperature sensor
#define BMI270_TEMP_EN_OFF                  UINT8_C(0 << 3) // Disable the temperature sensor 

/* -------------------------------------- Acc Configurations ------------------------------------------------- */

typedef enum _acc_odr {
    BMI270_ACC_ODR_0_78 = 0x01,     // ODR = 0.78Hz
    BMI270_ACC_ODR_1_5,      // ODR = 1.5Hz
    BMI270_ACC_ODR_3_1,      // ODR = 3.1Hz
    BMI270_ACC_ODR_6_25,     // ODR = 6.25Hz
    BMI270_ACC_ODR_12_5,     // ODR = 12.5Hz
    BMI270_ACC_ODR_25,       // ODR = 25Hz
    BMI270_ACC_ODR_50,       // ODR = 50Hz
    BMI270_ACC_ODR_100,             // ODR = 100Hz
    BMI270_ACC_ODR_200,             // ODR = 200Hz
    BMI270_ACC_ODR_400,             // ODR = 400Hz
    BMI270_ACC_ODR_800,             // ODR = 800Hz
    BMI270_ACC_ODR_1600,            // ODR = 1600Hz 
} BMI270_AccODR; 

typedef enum _acc_bwp {
    BMI270_ACC_BWP_OSR4_AVG1,
    BMI270_ACC_BWP_OSR2_AVG2,
    BMI270_ACC_BWP_NORM_AVG4,
    BMI270_ACC_BWP_CIC_AVG8,
    BMI270_ACC_BWP_RES_AVG16,
    BMI270_ACC_BWP_RES_AVG32,
    BMI270_ACC_BWP_RES_AVG64,
    BMI270_ACC_BWP_RES_AVG128,
} BMI270_AccBwp;

typedef enum _acc_range {
    BMI270_ACC_RANGE_2G,
    BMI270_ACC_RANGE_4G,
    BMI270_ACC_RANGE_8G,
    BMI270_ACC_RANGE_16G,
} BMI270_AccRange; 

typedef struct {
    BMI270_AccODR acc_odr;      // Acc ODR
    BMI270_AccBwp acc_bwp;      // Acc oversample and averaging
    BMI270_AccRange acc_range;  // Acc measurement range
    bool low_power_enable;      // 0: Enable low power mode, 1: Enable performance mode
} bmi270_acc_config_t; 

/* --------------------------------------- Gyr Configuration ------------------------------------------------- */

typedef enum _gyr_odr {
    BMI270_GYR_ODR_25 = 0x06,
    BMI270_GYR_ODR_50,
    BMI270_GYR_ODR_100,
    BMI270_GYR_ODR_200,
    BMI270_GYR_ODR_400,
    BMI270_GYR_ODR_800,
    BMI270_GYR_ODR_1600,
    BMI270_GYR_ODR_3200,
} BMI270_GyrODR;

typedef enum _gyr_bwp {
    BMI270_GYR_BWP_OSR4,
    BMI270_GYR_BWP_OSR2,
    BMI270_GYR_BWP_NORM,
} BMI270_GyrBwp;

typedef enum _gyr_range {
    BMI270_GYR_RANGE_2000,
    BMI270_GYR_RANGE_1000,
    BMI270_GYR_RANGE_500,
    BMI270_GYR_RANGE_250,
    BMI270_GYR_RANGE_125,
} BMI270_GyrRange;

typedef enum _ois_range {
    BMI270_OIS_RANGE_250,
    BMI270_OIS_RANGE_2000,
} BMI270_OisRange; 

typedef struct {
    BMI270_GyrODR gyr_odr;  // ODR for gyro
    BMI270_GyrBwp gyr_bwp;  // Oversample for gyro
    BMI270_GyrRange gyr_range;  // Range of gry: applies to filtered FIFO data and DATA register
    bool low_power_enable;  // 0: Enable low power mode, 1: Enable performance mode
} bmi270_gyr_config_t;

typedef struct {
    bool gyr_noise_perf;    // Enable high performance (1) or normal (0)
    bool gyr_filter_perf;   // Enable high performance (1) or normal (0)
    bool high_perf_en;          // Enable high performance mode (1) or low power mode (0)
} bmi270_pwr_config_t; 

/* ----------------------------------------- FIFO Configurations -------------------------------------------------- */
// Downsample rate for gyro data in FIFO 
typedef enum _gyr_fifo_downs {
    BMI270_GYR_FIFO_DOWNS_1,
    BMI270_GYR_FIFO_DOWNS_2,    // Downsample 2^1
    BMI270_GYR_FIFO_DOWNS_4,
    BMI270_GYR_FIFO_DOWNS_8,
    BMI270_GYR_FIFO_DOWNS_16,
    BMI270_GYR_FIFO_DOWNS_32,
    BMI270_GYR_FIFO_DOWNS_64,
    BMI270_GYR_FIFO_DOWNS_128,
} BMI270_GyrFifoDowns;

// Downsample rate for acc data in FIFO 
typedef enum _acc_fifo_downs {
    BMI270_ACC_FIFO_DOWNS_1,
    BMI270_ACC_FIFO_DOWNS_2,
    BMI270_ACC_FIFO_DOWNS_4,
    BMI270_ACC_FIFO_DOWNS_8,
    BMI270_ACC_FIFO_DOWNS_16,
    BMI270_ACC_FIFO_DOWNS_32,
    BMI270_ACC_FIFO_DOWNS_64,
    BMI270_ACC_FIFO_DOWNS_128, 
} BMI270_AccFifoDowns; 

// FIFO configuration structure 
typedef struct {
    bool gyr_filter;                        // Enable filtered gyr data in FIFO
    bool acc_filter;                        // Enable filtered acc data in FIFO 
    BMI270_GyrFifoDowns gyr_fifo_downs;     // Set downsample for gyro data in FIFO
    BMI270_AccFifoDowns acc_fifo_downs;     // Set downsample for acc data in FIFO
    bool fifo_stop_on_full;                 // Stop writing to FIFO when FIFO is full
    bool ret_sensor_time;                   // Return sensor time frame in FIFO
    bool fifo_header_en;                    // Header is stored (if disabled, ODR for all sensors must be the same)
    bool fifo_aux_en;                       // Store auxiliary sensor data in FIFO
    bool fifo_acc_en;                       // Store acc data in FIFO
    bool fifo_gyr_en;                       // Store gyr data in FIFO 
} bmi270_fifo_config_t;

/* --------------------------------------------------------------------------------------------------- */

typedef enum _cmd {
    BMI270_CMD_G_TRIGGER = 0x02,    // Trigger special gyro operations
    BMI270_CMD_USR_GAIN = 0x03,     // Applies new gyro gain value
    BMI270_CMD_NVM_PROG = 0xa0,     // Writes the NVM backed registers into NVM
    BMI270_CMD_FIFO_FLUSH = 0xb0,   // Clears FIFO content
    BMI270_CMD_SOFTRESET = 0xb6,    // Triggers a reset, back to default values
} BMI270_cmd; 

/* ------------------------------------------ Power Mode ------------------------------------------------ */

typedef enum _power_mode {
    BMI270_NORMAL_MODE,
    BMI270_LOW_POWER_MODE,
    BMI270_SUSPEND_MODE,
    BMI270_PERFORMANCE_MODE, 
} BMI270_PowerMode; 

/* ------------------------------------------- Sensor Data Output ---------------------------------------- */

typedef struct {
    struct sensor_value x;
    struct sensor_value y;
    struct sensor_value z;
} BMI270_DataValue;

typedef struct {
    BMI270_DataValue accel;
    BMI270_DataValue gyro;
} BMI270_IMU_Value;

extern bmi270_pwr_config_t bmi270_normal_mode;
extern bmi270_pwr_config_t bmi270_performance_mode;
extern bmi270_pwr_config_t bmi270_low_power_mode; 

/* --------------------------------------- Interrupt ----------------------------------------------------- */
typedef enum {
    FFULL_INT = 1 << 0,      // FIFO full interrupt
    FWM_INT = 1 << 1,        // FIFO watermark interrupt
    DRDY_INT = 1 << 2,       // Data ready interrupt
    ERR_INT = 1 << 3,        // Error interrupt 
} BMI270_Int_Data;

typedef enum {
    SIG_MOTION_OUT = 1 << 0,         // Significant motion output
    STEP_COUNTER_OUT = 1 << 1,       // Step-counter watermark or Step-detector output
    ACTIVITY_OUT = 1 << 2,           // Step activity output
    WRIST_WEAR_WAKEUP_OUT = 1 << 3,  // Wrist wear wakeup output
    WRIST_GESTURE_OUT = 1 << 4,      // Wrist gesture output
    NO_MOTION_OUT = 1 << 5,          // No motion detection output
    ANY_MOTION_OUT = 1 << 6,         // Any motion detection output 
} BMI270_Int_Feature;

/* Options for interrupt pin configurations:
 * lvl: output level 
 * odb: output behaviour 
 * output_en: enable output 
 * input_en: enable input */
typedef struct {
    bool lvl;                           // 0 for active low, 1 for active high
    bool odb;                           // 0 for push-pull, 1 for open drain
    bool output_en;                     // 0 for output disable, 1 for output enable
    bool input_en;                      // 0 for input disable, 1 for input enable
    bool int_latch;                     // 0 for non latched, 1 for permanent 
    union {
        BMI270_Int_Feature feature_int; // Advanced feature interrupt
        BMI270_Int_Data data_int;       // Data type interrupt 
    } interrupt_type;     
    bool use_data;                   // Select type of interrupt to use: 0 for feature int, 1 for data int
} bmi270_interrupt_config_t;

typedef struct {
    uint8_t duration;       // Duration when threshold condition is met. Range is 0 to 163 sec.
    bool select_x;          // Select this feature on x axis
    bool select_y;          // Select this feature on y axis
    bool select_z;          // Select this feature on z axis 
    uint16_t threshold;     // Slop threshold value for no-motion detection. Range from 0 to 1000 mg
    bool enable;            // Enable/disable the feature 
} bmi270_no_motion_config_t; 


/* --------------------------------------- Prototyes ----------------------------------------------------- */

bool bmi270_init(BMI270_Context *ctx, const struct spi_dt_spec *spec); 
int bmi270_conf_fifo(BMI270_Context *ctx, uint16_t watermark, bmi270_fifo_config_t *config); 
int bmi270_conf_acc(BMI270_Context *ctx, bmi270_acc_config_t *config);
int bmi270_conf_gyr(BMI270_Context *ctx, bmi270_gyr_config_t *config);
void bmi270_set_mode(BMI270_Context *ctx, BMI270_PowerMode mode, bool acc_enable, bool gyr_enable); 
int bmi270_read_imu(BMI270_Context *ctx, BMI270_IMU_Value *value, uint8_t acc_range, uint8_t gyr_range); 
int bmi270_conf_interrupt(BMI270_Context *ctx, bmi270_interrupt_config_t *config, uint8_t pin); 
int bmi270_conf_no_motion(BMI270_Context *ctx, bmi270_no_motion_config_t *config); 
int bmi270_write_command(BMI270_Context *ctx, BMI270_cmd *cmd);
int bmi270_read_int_status(BMI270_Context *ctx);
uint8_t bmi270_read_fifo_length(BMI270_Context *ctx); 
void bmi270_read_no_motion_config(BMI270_Context *ctx); 

#endif