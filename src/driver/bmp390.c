#include "bmp390.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(BMP390, LOG_LEVEL_DBG); 

/**
 * @brief Helper function to get compensation coefficient
 * 
 * Retrive compensation coefficient for calculation of physical temperature and pressure
 * 
 * @param ctx Pointer to a bmp390 context 
 * 
 * @return Compensation coefficient structure
 */
static BMP390_calib_data get_compensation_coefficient(BMP390_Context *ctx){
    int ret; 

    BMP390_calib_data calib_data;
    uint8_t compensation[21] = {0};

    ret = i2c_burst_read_dt(ctx->spec, NVM_PAR_T1, compensation, sizeof(compensation));
    if(ret){
        LOG_ERR("Failed to read compensation coefficients\n");
    }

    calib_data.par_t1 = (float)((uint16_t)(compensation[1] << 8 | compensation[0])) / (float)pow(2.0, -8.0); 
    calib_data.par_t2 = (float)((uint16_t)(compensation[3] << 8 | compensation[2])) / (float)pow(2.0, 30);
    calib_data.par_t3 = (float)((int8_t)(compensation[4])) / (float)pow(2, 48);
    calib_data.par_p1 = (float)((int16_t)(compensation[6] << 8 | compensation[5]) - (float)pow(2,14)) / (float)pow(2, 20);
    calib_data.par_p2 = (float)((int16_t)(compensation[8] << 8 | compensation[7]) - (float)pow(2,14)) / (float)pow(2,29); 
    calib_data.par_p3 = (float)((int8_t)(compensation[9])) / (float)pow(2,32);
    calib_data.par_p4 = (float)((int8_t)(compensation[10])) / (float)pow(2,37);
    calib_data.par_p5 = (float)((uint16_t)(compensation[12] << 8 | compensation[11])) / (float)pow(2,-3); 
    calib_data.par_p6 = (float)((uint16_t)(compensation[14] << 8 | compensation[13])) / (float)pow(2,6);
    calib_data.par_p7 = (float)((int8_t)(compensation[15])) / (float)pow(2,8);
    calib_data.par_p8 = (float)((int8_t)(compensation[16])) / (float)pow(2,15);
    calib_data.par_p9 = (float)((int16_t)(compensation[18] << 8 | compensation[17])) / (float)pow(2,48);
    calib_data.par_p10 = (float)((int8_t)(compensation[19])) / (float)pow(2,48);
    calib_data.par_p11 = (float)((int8_t)(compensation[20])) / (float)pow(2,65); 

    return calib_data; 
}

/**
 * @brief Initialise BMP390 sensor
 * 
 * Initialise and softrest sensor
 * 
 * @param ctx Pointer to a bmp390 context 
 * @param spec Pointer to a I2C device specification
 * 
 * @return True if success false otherwise
 */
bool bmp390_init(BMP390_Context *ctx, const struct i2c_dt_spec *spec){
    int ret = 0;

    ctx->spec = spec; 

    /* Softrest sensor */
    uint8_t softrest_cmd[2] = {CMD_REG, CMD_SOFTRESET};
    ret = i2c_write_dt(ctx->spec, softrest_cmd, 2);
    if(ret){
        LOG_ERR("Failed to reset sensor");
        return false;
    }

    k_msleep(1); 

    /* Flush FIFO */
    uint8_t flush_cmd[2] = {CMD_REG, CMD_FIFO_FLUSH};

    ret = i2c_write_dt(ctx->spec, flush_cmd, sizeof(flush_cmd));
    if(ret){
        LOG_ERR("Failed to flush FIFO buffer");
        return false;
    }
    LOG_INF("FIFO buffer cleared");

    /* Read the chip identification ID to make sure this is the correct sensor */
    uint8_t chip_id;
    uint8_t chip_id_reg = CHIP_ID_REG;
    ret = i2c_write_read_dt(ctx->spec, &chip_id_reg, 1, &chip_id, 1); 
    if(ret != 0){
        LOG_ERR("Failed to read/write I2C (%x) at register %x\n", spec->addr, chip_id_reg);
        return false;
    }

    /* Check the chip id */
    if (chip_id != CHIP_ID){
        LOG_ERR("Chip ID mismatch\n");
        return false; 
    }
    LOG_INF("Chip ID verified as 0x%x", chip_id); 

    /* Get compensation data */
    ctx->calib_data = get_compensation_coefficient(ctx);

    return true; 
}

/**
 * @brief Configure BMP390 sensor
 * 
 * Configure sensor power mode, ODR, OSR and IIR
 * 
 * @param ctx Pointer to a bmp390 context 
 * @param bmp390_config Pointer to a configuration structure
 * 
 * @return True if succeed false otherwise
 */
bool bmp390_configure(BMP390_Context *ctx, BMP390_Configurations *bmp390_config){
    int ret;

    /* Configure power mode */
    uint8_t power_mode[2] = {PWR_CTRL_REG, (bmp390_config->mode | bmp390_config->sensor_enable)}; 
    ret = i2c_write_dt(ctx->spec, power_mode, sizeof(power_mode)); 
    if (ret){
        LOG_ERR("Failed to configure sensor mode (err: %d)", ret);
        return false; 
    }     
 
    /* Configure OSR oversampling for temperature and pressure */
    uint8_t osr[2] = {OSR_REG, (bmp390_config->osr_temperature << 3) | bmp390_config->osr_pressure};
    ret = i2c_write_dt(ctx->spec, osr, sizeof(osr));
    if (ret){
        LOG_ERR("Failed to configure oversampling (err: %d)", ret);
        return false;
    }

    /* Configure ODR */
    uint8_t odr[2] = {ODR_REG, bmp390_config->odr}; 
    ret = i2c_write_dt(ctx->spec, odr, sizeof(odr)); 
    if (ret){
        LOG_ERR("Failed to configure ODR (err: %d)", ret);
        return false; 
    }

    /* Configure IIR filter */
    uint8_t iir[2] = {IIR_CONFIG_REG, (bmp390_config->iir << 1)};
    ret = i2c_write_dt(ctx->spec, iir, sizeof(iir));
    if (ret){
        LOG_ERR("Failed to configure IIR (err: %d)", ret); 
        return false; 
    }
    return true;
}

/**
 * @brief Helper function to calculate compensated temperature
 * 
 * @param uncomp_temp Raw sensor temperature value
 * @param calib_data Pointer to sensor compensation coefficients
 * 
 * @return Physical temperature in degree
 */
static float BMP390_compensate_temperature(uint32_t uncomp_temp, BMP390_calib_data *calib_data){
    float partial_data1;
    float partial_data2;

    partial_data1 = (float)(uncomp_temp - calib_data->par_t1);
    partial_data2 = (float)(partial_data1 * calib_data->par_t2);

    calib_data->t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data->par_t3;

    return calib_data->t_lin;
}

/**
 * @brief Helper function to calculate compensated pressure
 * 
 * @param uncomp_temp Raw sensor pressure value
 * @param calib_data Pointer to sensor compensation coefficients
 * 
 * @return Physical pressure in hPa
 */
static float BMP390_compensate_pressure(uint32_t uncomp_press, BMP390_calib_data *calib_data){
    float comp_press;

    float partial_data1, partial_data2, partial_data3, partial_data4, partial_out1, partial_out2;

    partial_data1 = calib_data->par_p6 * calib_data->t_lin;
    partial_data2 = calib_data->par_p7 * (calib_data->t_lin * calib_data->t_lin);
    partial_data3 = calib_data->par_p8 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
    partial_out1 = calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = calib_data->par_p2 * calib_data->t_lin;
    partial_data2 = calib_data->par_p3 * (calib_data->t_lin * calib_data->t_lin);
    partial_data3 = calib_data->par_p4 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
    partial_out2 = (float)uncomp_press * (calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)uncomp_press * (float)uncomp_press;
    partial_data2 = calib_data->par_p9 + calib_data->par_p10 * calib_data->t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib_data->par_p11;
    comp_press = partial_out1 + partial_out2 + partial_data4;

    return comp_press / 100.0f;
}

/**
 * @brief Read sensor data
 * 
 * Read and compensate raw sensor values to get physical values
 * 
 * @param ctx Pointer to a bmp390 context 
 * @param temperature Pointer to save temperature
 * @param pressure Pointer to save pressure
 * 
 * @retval 0 If success
 */
int bmp390_read_sensor(BMP390_Context *ctx, float *temperature, float *pressure){
    
    if (temperature == NULL || pressure == NULL){
        LOG_ERR("Null pointer passed to function");
        return -1; 
    }

    *temperature = 0.0f;
    *pressure = 0.0f; 

    int ret;
    
    /* Read raw values from the sensor */
    uint8_t data[6] = {0};
    // Burst read
    ret = i2c_burst_read_dt(ctx->spec, PRE_DATA0_REG, data, 6); 
    
    if (ret){
        LOG_ERR("Failed to read raw sensor values\n");
        return ret;
    }

    uint32_t raw_pressure = (data[2] << 16 | data[1] << 8 | data[0]);
    uint32_t raw_temperature = (data[5] << 16 | data[4] << 8 | data[3]);    

    /* Calculate compensated temperature */
    *temperature = BMP390_compensate_temperature(raw_temperature, &ctx->calib_data);
    *pressure = BMP390_compensate_pressure(raw_pressure, &ctx->calib_data); 

    return 0;    
}

/**
 * @brief Configure interrupt
 * 
 * Set sensor interrupt options
 * 
 * @param ctx Pointer to a bmp390 context 
 * @param config Interrupt configuration
 * 
 * @return True if success false otherwise
 */
bool bmp390_set_interrupt(BMP390_Context *ctx, uint8_t config){
    int ret;
    
    uint8_t int_config[2] = {INT_CTRL_REG, config};

    ret = i2c_write_dt(ctx->spec, int_config, sizeof(int_config));

    if (ret){
        LOG_ERR("Failed to configure interrupt");
        return false;
    }
    
    LOG_INF("Interrupt -- Write 0x%.2x to register 0x%.2x", config, INT_CTRL_REG); 
    return true; 
}

/**
 * @brief Get current interrupt status
 * 
 * Get current interrupt configurations
 * 
 * @param ctx Pointer to a bmp390 context 
 * 
 * @return Interrupt configuration
 */
uint8_t bmp390_get_interrupt(BMP390_Context *ctx){
    int ret;

    uint8_t int_status;
    uint8_t int_status_reg = INT_STATUS_REG;
    ret = i2c_write_read_dt(ctx->spec, &int_status_reg, 1, &int_status, 1); 
    if (ret){
        LOG_ERR("Failed to read interrupt status\n");
    }

    LOG_INF("Current interrupt status: 0x%.2x", int_status); 
    return int_status; 
}

/**
 * @brief Get current sensor mode
 * 
 * @param ctx Pointer to a bmp390 context 
 * 
 */
void bmp390_get_configuration(BMP390_Context *ctx){
    int ret;

    /* Get current mode */
    uint8_t mode_value;
    uint8_t mode_reg = PWR_CTRL_REG;
    ret = i2c_write_read_dt(ctx->spec, &mode_reg, 1, &mode_value, 1);
    if (ret){
        LOG_ERR("Failed to write/read register %.2x (err: %d)", mode_reg, ret); 
    }
    LOG_INF("Power mode register value: 0x%.2x", mode_value);

    /* Get current OSR */
    uint8_t osr_reg = OSR_REG;
    uint8_t osr_value;
    ret = i2c_write_read_dt(ctx->spec, &osr_reg, 1, &osr_value, 1);
    if (ret){
        LOG_ERR("Failed to write/read register %.2x (err: %d)", osr_reg, ret); 
    }
    LOG_INF("OSR register value: 0x%.2x", osr_value);

    /* Get current ODR */
    uint8_t odr_reg = ODR_REG;
    uint8_t odr_value;
    ret = i2c_write_read_dt(ctx->spec, &odr_reg, 1, &odr_value, 1);
    if (ret){
        LOG_ERR("Failed to write/read register %.2x (err: %d)", odr_reg, ret); 
    }
    LOG_INF("ODR register value: 0x%.2x", odr_value);

    /* Get current IIR */
    uint8_t iir_reg = IIR_CONFIG_REG;
    uint8_t iir_value;
    ret = i2c_write_read_dt(ctx->spec, &iir_reg, 1, &iir_value, 1);
    if (ret){
        LOG_ERR("Failed to write/read register %.2x (err: %d)", iir_reg, ret); 
    }
    LOG_INF("IIR register value: 0x%.2x", iir_value);
}

/**
 * @brief Configure FIFO
 * 
 *  Set FIFO configurations
 * 
 * @param ctx Pointer to a bmp390 context 
 * @param fifo_config_1 Value to write to FIFO_CONFIG_1 register
 * @param fifo_config_2 Value to write to FIFO_CONFIG_2 register
 * 
 * @return True if succeed. False otherwise
 */
bool bmp390_set_FIFO(BMP390_Context *ctx, uint8_t fifo_config_1, uint8_t fifo_config_2){
    int ret;

    uint8_t fifo_config_value[2] = {fifo_config_1, fifo_config_2};

    ret = i2c_burst_write_dt(ctx->spec, FIFO_CONFIG_1_REG, fifo_config_value, sizeof(fifo_config_value));

    if(ret){
        LOG_ERR("Failed to configure FIFO registers");
        return false;
    }

    LOG_INF("Writing FIFO configuration:");
    LOG_INF("FIFO Config 1 (0x17): 0x%.2x", fifo_config_value[0]);
    LOG_INF("FIFO Config 2 (0x18): 0x%.2x", fifo_config_value[1]);

    return true; 
}

/**
 * @brief Get current FIFO configuration
 * 
 * @param ctx Pointer to a bmp390 context 
 * 
 */
void bmp390_get_fifo(BMP390_Context *ctx){
    int ret;

    uint8_t fifo_config1_reg = FIFO_CONFIG_1_REG;
    uint8_t fifo_config1_value;
    ret = i2c_write_read_dt(ctx->spec, &fifo_config1_reg, 1, &fifo_config1_value, 1);

    if(ret){
        LOG_ERR("Failed to ger FIFO config 1");
    }

    LOG_INF("FIFO Config 1: 0x%.2x", fifo_config1_value); 
}

/**
 * @brief Get number of bytes available to read in FIFO
 * 
 * @param ctx Pointer to a bmp390 context 
 * 
 * @return Number of bytes
 */
uint16_t bmp390_read_fifo_length(BMP390_Context *ctx){
    int ret;

    /* How many bytes are available currently in FIFO */
    uint8_t fifo_length[2] = {0};
    ret = i2c_burst_read_dt(ctx->spec, FIFO_LENGTH_0_REG, fifo_length, sizeof(fifo_length));
    if(ret){
        LOG_ERR("Faild to get FIFO length");
    }
    
    uint16_t fifo_size = (fifo_length[1] << 8 | fifo_length[0]);

    return fifo_size; 
}

static float temperature_readings[80];
static float pressure_readings[80];
static int reading_count = 0;

/**
 * @brief Helper function to parse FIFO frame
 * 
 * @param ctx Pointer to a bmp390 context 
 * @param fifo_data Pointer to FIFO frames
 * @param fifo_size Number of bytes to read from FIFO
 * 
 */
static int parse_fifo_data(BMP390_Context *ctx, uint8_t *fifo_data, uint16_t fifo_size, 
    float *temp_buffer, float *pres_buffer, int buffer_size){

    int i = 0;
    reading_count = 0;

    while (i < (fifo_size - 1) && reading_count < buffer_size){
        uint8_t header = fifo_data[i++];
        uint32_t raw_temperature = 0;
        uint32_t raw_pressure = 0;

        if(header & 0x10){
            raw_temperature = (fifo_data[i+2] << 16) | (fifo_data[i+1] << 8) | fifo_data[i];  
            i += 3;
        }

        if(header & 0x04){
            raw_pressure = (fifo_data[i+2] << 16) | (fifo_data[i+1] << 8) | fifo_data[i];
            i += 3;
        }

        if (raw_temperature || raw_pressure){
            float compensated_temperature = BMP390_compensate_temperature(raw_temperature, &ctx->calib_data);
            float compensated_pressure = BMP390_compensate_pressure(raw_pressure, &ctx->calib_data); 

            temp_buffer[reading_count] = compensated_temperature;
            pres_buffer[reading_count] = compensated_pressure; 
            reading_count ++;
        }
    }
    return reading_count; 
}

/**
 * @brief Read FIFO frame
 * 
 * @param fifo_size Number of bytes to read in FIFO
 * 
 * @param ctx Pointer to a bmp390 context 
 * 
 */
int bmp390_read_fifo_data(BMP390_Context *ctx, uint16_t fifo_size, float *temp_buffer, float *pres_buffer, int buffer_size){
    int ret;
    uint8_t fifo_data_reg = FIFO_DATA_REG; 
    uint8_t *fifo_data = k_malloc(fifo_size); 

    if (!fifo_data){
        LOG_ERR("Failed to allocate memory for FIFO data");
        return -ENOMEM; 
    }

    ret = i2c_burst_read_dt(ctx->spec, fifo_data_reg, fifo_data, fifo_size);
    if(ret){
        LOG_ERR("Failed to read FIFO buffer");
        k_free(fifo_data);
        return ret; 
    }

    int readings = parse_fifo_data(ctx, fifo_data, fifo_size, temp_buffer, pres_buffer, buffer_size);
    k_free(fifo_data);
    return readings; 
}

/**
 * @brief Number of samples included in retrived FIFO frames
 * 
 * @return Number of samples
 */
int bmp390_get_reading_count(){
    return reading_count;
}

/**
 * @brief Calculate compensated temperature in FIFO frame
 * 
 * @param index Index the sample in FIFO
 * 
 * @return Temperature in degree
 */
float bmp390_get_temperature_from_fifo(int index){
    if(index >= 0 && index < reading_count){
        return temperature_readings[index];
    }else{
        return -1;
    }
}

/**
 * @brief Calculate compensated pressure in FIFO frame
 * 
 * @param index Index the sample in FIFO
 * 
 * @return pressure in degree
 */
float bmp390_get_pressure_from_fifo(int index){
    if(index >= 0 && index < reading_count){
        return pressure_readings[index];
    }else{
        return -1; 
    }
}

/**
 * @brief Flush FIFO
 * 
 * @param ctx Pointer to a bmp390 context 
 * 
 */
void bmp390_flush_fifo(BMP390_Context *ctx){
    int ret;

    uint8_t flush_cmd[2] = {CMD_REG, CMD_FIFO_FLUSH};

    ret = i2c_write_dt(ctx->spec, flush_cmd, sizeof(flush_cmd));
    if(ret){
        LOG_ERR("Failed to flush FIFO buffer");
    }
    LOG_INF("FIFO buffer cleared");
}