/** 
 *  @brief Driver implementation for BMP390
 *  This is the driver for TORUS bmp390 implementation using zephyr i2c API 
 * 
 *  @file bmp390.h
 *  @date 2024-11-04
 *  @version v1.0
 */

#ifndef BMP390_H_
#define BMP390_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

/* Register map for BMP390 */
#define BMP390_I2C_ADDR       0x77  // Default I2C address when SDO is grounded
#define CHIP_ID_REG           0x00  // Chip identification code
#define REV_ID_REG            0x01  // Mask revision of the ASIC
#define ERR_REG               0x02  // Sensor error conditions
#define STATUS_REG            0x03  // Sensor status flag: temperature and pressure ready
#define PRE_DATA0_REG         0x04  // XLSB for pressure value
#define PRE_DATA1_REG         0x05  // LSB for pressure value
#define PRE_DATA2_REG         0x06  // MSB for pressure value 
#define TEMP_DATA3_REG        0x07  // XLSB for temperature value
#define TEMP_DATA4_REG        0x08  // LSB for temperature value
#define TEMP_DATA5_REG        0x09  // MSB for temperature value 
#define SENSOR_TIME_0_REG     0x0C  // XLSB for sensor time
#define SENSOR_TIME_1_REG     0x0D  // LSB for sensor time
#define SENSOR_TIME_2_REG     0x0E  // MSB for sensor time
#define EVENT_REG             0x10  // Sensor status flat: power up and serial interface transaction
#define INT_STATUS_REG        0x11  // Interrupt status
#define FIFO_LENGTH_0_REG     0x12  // LSB for FIFO byte counter: current fill level of FIFO buffer (9bit only)
#define FIFO_LENGTH_1_REG     0x13  // MSB for FIFO byte counter (9 bit only)
#define FIFO_DATA_REG         0x14  // FIFO read data (output)
#define FIFO_WATERMARK_1_REG  0x15  // LSB for FIFO watermark setting (fill all 8 bits for this one)
#define FIFO_WATERMARK_2_REG  0x16  // MSB for FIFO watermakr setting (fill the LSBit only)
#define FIFO_CONFIG_1_REG     0x17  // Configure FIFO
#define FIFO_CONFIG_2_REG     0x18  // Configure FIFO
#define INT_CTRL_REG          0x19  // Interrupt configuration 
#define IF_CONF_REG           0x1A  // Serial interface control
#define PWR_CTRL_REG          0x1B  // Measurement mode
#define OSR_REG               0x1C  // Oversampling 
#define ODR_REG               0x1D  // Output data rate
#define IIR_CONFIG_REG        0x1F  // IIR filter coefficient 
#define CMD_REG               0x7E  // Command register 
#define NVM_PAR_T1            0x31  // NVM start address for pressure and temperature compensation

#define CHIP_ID               0x60  // Default chip ID
#define FIFO_DATA_LENGTH      512

/**
 * @brief Macros to configure FIFO settings
 */
// For CONFIG_1_REG
#define FIFO_CONF_ENABLE        UINT8_C(1<<0)   // Enable (1) or disable (0) the FIFO
#define FIFO_CONF_STOP_ON_FULL  UINT8_C(1<<1)   // Stop writing FIFO when full 
#define FIFO_CONF_TIME_EN       UINT8_C(1<<2)   // Return sensortime frame after the last valid data frame
#define FIFO_CONF_PRESS_EN      UINT8_C(1<<3)   // Store pressure data in FIFO
#define FIFO_CONF_TEMP_EN       UINT8_C(1<<4)   // Store temperature data in FIFO
// For CONFIG_2_REG
#define FIFO_CONF_SUBSAMPLE     UINT8_C(1<<2)   // Downsample factor 2^fifo_subsamp
#define FIFO_CONF_DATA_SELECT   UINT8_C(1<<3)   // Use filtered data

/**
 * @brief Macros to configure interrupt settings
 */
#define INT_CONF_OD         UINT8_C(1<<0)   // open drain (1) or push pull (0)
#define INT_CONF_LEVEL      UINT8_C(1<<1)   // Level of INT pin: 1 active high
#define INT_CONF_LATCH      UINT8_C(1<<2)   // Latching of interrupts: 1 enable
#define INT_CONF_FWTM_EN    UINT8_C(1<<3)   // Enable FIFO watermark reached interrupt
#define INT_CONF_FFULL_EN   UINT8_C(1<<4)   // Enable FIFO full interrupt
#define INT_CONF_INT_DS     UINT8_C(1<<5)   // Drive strength: 1 high
#define INT_CONF_DRDY_EN    UINT8_C(1<<6)   // Enable pressure/temperature data ready interrupt

/**
 * @brief Macros to indicate interrupt status
 */
#define BMP390_FWM_BIT      1<<0
#define BMP390_FFULL_BIT    1<<1
#define BMP390_DRDY_BIT     1<<3

/**
 * @brief Macros to control serial interface
 */
#define IF_CONF_SPI_3       UINT8_C(1<<0)   // Enable 3-wire SPI
#define IF_CONF_I2C_WDT_EN  UINT8_C(1<<1)   // Enable I2C watchdog timer
#define IF_CONF_I2C_WDT_SEL UINT8_C(1<<2)   // Select timer period for I2C watchdog: 1 timuout 40ms 0 timeout 1.25ms

/**
 * @brief Measurement mode options for PWR_CTRL_REG
 */
typedef enum _sensor_enable {
    BMP390_SENSOR_DISABLE,       // Disable 
    BMP390_PRESSURE_ENABLE,      // Enable pressure
    BMP390_TEMPERATURE_ENABLE,   // Enable temperature
    BMP390_ENABLE_BOTH,          // Enable both
} BMP390_SensorEnable;

typedef enum _mode {
    BMP390_MODE_SLEEP = 0x00,           // Sleep mode
    BMP390_MODE_FORCE = 0x10,           // Forced mode
    BMP390_MODE_NORMAL = 0x30,          // Normal mode
} BMP390_MeasurementMode;

/**
 * @brief Oversampling options for OSR_REG
 */
typedef enum _osr {
    BMP390_OSR_NO,
    BMP390_OSR_X2,
    BMP390_OSR_X4,
    BMP390_OSR_X8,
    BMP390_OSR_X16,
    BMP390_OSR_X32,
} BMP390_OSR;

/**
 * @brief ODR options for temperature and pressure 
 */
typedef enum _odr {
    BMP390_ODR_200,
    BMP390_ODR_100,
    BMP390_ODR_50,
    BMP390_ODR_25,
    BMP390_ODR_12_5,
    BMP390_ODR_6_25,
    BMP390_ODR_3_1,
    BMP390_ODR_1_5,
} BMP390_ODR; 

/**
 * @brief IIR filter options 
 */
typedef enum _iir_filter {
    BMP390_IIR_COEF_0,
    BMP390_IIR_COEF_1,
    BMP390_IIR_COEF_3,
    BMP390_IIR_COEF_7,
    BMP390_IIR_COEF_15,
    BMP390_IIR_COEF_31,
    BMP390_IIR_COEF_63,
    BMP390_IIR_COEF_127,
} BMP390_IIR; 

/**
 * @brief Macros for command control 
 */
#define CMD_FIFO_FLUSH      0xB0    // Clears all data in FIFO
#define CMD_SOFTRESET       0xB6    // Trigger a reset

/**
 * @brief BMP390 Configurations
 */
typedef struct {
    BMP390_SensorEnable sensor_enable;
    BMP390_MeasurementMode mode;
    BMP390_OSR osr_pressure;
    BMP390_OSR osr_temperature; 
    BMP390_ODR odr;
    BMP390_IIR iir;
} BMP390_Configurations; 

/**
 * @brief Compensation coefficients
 */
typedef struct {
    float par_t1;
    float par_t2;
    float par_t3;
    float par_p1;
    float par_p2;
    float par_p3;
    float par_p4;
    float par_p5;
    float par_p6;
    float par_p7;
    float par_p8;
    float par_p9;
    float par_p10;
    float par_p11; 
    float t_lin;
} BMP390_calib_data;

typedef struct {
    BMP390_calib_data calib_data;
    const struct i2c_dt_spec *spec;
} BMP390_Context; 

int bmp390_get_reading_count(void);
float bmp390_get_temperature_from_fifo(int index);
float bmp390_get_pressure_from_fifo(int index);

bool bmp390_init(BMP390_Context *ctx, const struct i2c_dt_spec *spec);
bool bmp390_configure(BMP390_Context *ctx, BMP390_Configurations *bmp390_config);
int bmp390_read_sensor(BMP390_Context *ctx, float *temperature, float *pressure);
bool bmp390_set_interrupt(BMP390_Context *ctx, uint8_t config);
uint8_t bmp390_get_interrupt(BMP390_Context *ctx);
void bmp390_get_configuration(BMP390_Context *ctx); 
bool bmp390_set_FIFO(BMP390_Context *ctx, uint8_t fifo_config_1, uint8_t fifo_config_2); 
void bmp390_get_fifo(BMP390_Context *ctx);
uint16_t bmp390_read_fifo_length(BMP390_Context *ctx); 
int bmp390_read_fifo_data(BMP390_Context *ctx, uint16_t fifo_size, float *temp_buffer, float *pres_buffer, int buffer_size);
void bmp390_flush_fifo(BMP390_Context *ctx);


#endif