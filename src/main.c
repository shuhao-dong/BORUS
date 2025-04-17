/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * This is the BORUS firmware development based on Thingy53. It demonstrates
 * how to collect sensor data via SPI and I2C, send data over BLE, log data
 * to the external flash using littlefs and a context awareness state change.
 *
 * Author: Shuhao Dong <shuhao.dong@bristol.ac.uk>
 * Version: v0.1.0
 *
 * Date: 16/04/2025
 * 
 * TBD: BLE encryption, BLE Extended adv 
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include "driver/bmi270.h"
#include <math.h>
#include "battery.h"
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>
#include <pm_config.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <errno.h>

LOG_MODULE_REGISTER(THINGY, LOG_LEVEL_INF);

/* -------------------- Thread Configurations -------------------- */

// Stack sizes
#define BMI270_HANDLER_STACKSIZE 		1024
#define BMP390_HANDLER_STACKSIZE 		1024
#define BLE_LOGGER_THREAD_STACKSIZE 	4096
#define SCANNER_THREAD_STACKSIZE 		1024

// Priorities (Lower number = higher priority)
#define BMI270_HANDLER_PRIORITY 		5	// Highest sensor priority due to higher sample rate
#define BMP390_HANDLER_PRIORITY 		6	// Medium sensor priority due to lower sample rate
#define BLE_THREAD_PRIORITY 			7	// Lower priority tasks for BLE and logging
#define SCANNER_THREAD_PRIORITY 		7 	// Lower priority tasks for scan AP heartbeat

// Thread Stacks
K_THREAD_STACK_DEFINE(bmi270_handler_stack_area, BMI270_HANDLER_STACKSIZE);
K_THREAD_STACK_DEFINE(bmp390_handler_stack_area, BMP390_HANDLER_STACKSIZE);
K_THREAD_STACK_DEFINE(ble_logger_stack_area, BLE_LOGGER_THREAD_STACKSIZE);
K_THREAD_STACK_DEFINE(scanner_stack_area, SCANNER_THREAD_STACKSIZE);

// Thread Control Blocks
static struct k_thread bmi270_handler_thread_data;
static struct k_thread bmp390_handler_thread_data;
static struct k_thread ble_logger_thread_data;
static struct k_thread scanner_thread_data;

/* -------------------- State Machine -------------------- */

// Define the state structure
typedef enum
{
	STATE_INIT,					// Initial state before normal operation
	STATE_HOME_ADVERTISING, 	// At home, advertise sensor data
	STATE_AWAY_LOGGING,			// Away, log sensor data
	STATE_CHARGING				// USB connected
} device_state_t;

// Define system workqueue for state transition 
static struct k_work heartbeat_timeout_work;	// Work item for heartbeat timeout -> Away state
static struct k_work usb_connect_work;			// Work item for USB connect -> CHARGING state
static struct k_work usb_disconnect_work;		// Work item for USB disconnect -> HOME state (or chosen default)
static struct k_work scan_found_ap_work;		// Work item for Scan Found AP -> HOME state

// --- Use atomic type for state changes between threads/ISRs
static atomic_t current_state = ATOMIC_INIT(STATE_INIT);

/* -------------------- Message Queue for Sensor Data -------------------- */

// Define message types
typedef enum
{
	SENSOR_MSG_TYPE_IMU,
	SENSOR_MSG_TYPE_ENVIRONMENT,
	SENSOR_MSG_TYPE_BATTERY,
} sensor_msg_type_t;

// Define payload structures: imu, environment, battery
typedef struct
{
	int16_t imu_data[6];
	uint32_t timestamp;
} imu_payload_t;

// Environmental sensor data structure
typedef struct
{
	uint16_t temperature;
	uint32_t pressure;
	uint32_t timestamp;
} environment_payload_t;

// Battery voltage data structure 
typedef struct
{
	uint8_t battery;
	uint32_t timestamp;
} battery_payload_t;

// Unified message structure for message queue
typedef struct
{
	sensor_msg_type_t type;
	union
	{
		imu_payload_t imu;
		environment_payload_t env;
		battery_payload_t batt;
	} payload;
} sensor_message_t;

// Define the message queue
K_MSGQ_DEFINE(sensor_message_queue, sizeof(sensor_message_t), 32, 4);

/* -------------------- Semaphores for Interrupts -------------------- */

K_SEM_DEFINE(bmi270_isr_sem, 0, 20);
// K_SEM_DEFINE(bmp390_isr_sem, 0, 1);

/* -------------------- Timers for hearbeat check -------------------- */

// Timers for Periodic Tasks
static struct k_timer heartbeat_timeout_timer; // For away detection
static struct k_timer battery_timer;		   // For periodic battery reading

/* -------------------- Configuration Constants -------------------- */

#define BMP390_READ_INTERVAL 		1000 / 20				// BMP390 sample rate = 20 Hz
#define BATTERY_READ_INTERVAL 		K_MINUTES(30)			// Battery voltage read interval = 30 minutes
#define HEARTBEAT_TIMEOUT 			K_SECONDS(90) 			// If no heartbeat for 90s, assume away, enter AWAY state
#define BLE_ADV_INTERVAL_MIN 		32						// This is the minimum allowed adv interval = 32 * 0.625 = 20 ms
#define BLE_ADV_INTERVAL_MAX 		33						// Added a small delay to avoid alias = 33 * 0.625 = 20.625 ms
#define SENSOR_DATA_PACKET_SIZE 	20 						// Size calculated from prepare_packet
#define SCAN_INTERVAL 				K_MINUTES(1)			// This is how often to perform a scan: every 1 minute
#define SCAN_WINDOW 				K_SECONDS(5)			// This is how long to scan during that interval: 5 seconds
#define TARGET_AP_ADDR 				"2C:CF:67:89:E0:5D"		// TORUS_1
#define PRESSURE_BASE_PA 			90000			   		// Base offset in Pascals
#define TEMPERATURE_LOW_LIMIT		30		  				// -30 degree as the lowest temperature of interest
#define TEMPERATURE_HIGH_LIMIT		40		   				// +40 degree as the highest temperature of interest

/* -------------------- File system and MSC -------------------- */

#define LOG_FILE_PATH "/lfs1/imu_log.bin"
#define LFS_MOUNT_POINT "/lfs1"

USBD_DEFINE_MSC_LUN(NAND, "Zephyr", "BORUS", "0.00");	// Make sure name "NAND" matches the Kconfig option @ref CONFIG_MASS_STORAGE_DISK_NAME

/* -------------------- BLE Legacy Configurations -------------------- */

// Define BLE packet structure - Not used in real BLE packet
typedef struct
{
	int16_t temperature;
	uint32_t pressure;
	int16_t imu_data[6];
	uint32_t timestamp;
	uint8_t battery;
} ble_packet_t;

// BLE advertisement parameters
static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	BT_LE_ADV_OPT_USE_IDENTITY, // Use identity MAC for advertisement
	BLE_ADV_INTERVAL_MIN,		// Min advertisement interval (min * 0.625)
	BLE_ADV_INTERVAL_MAX,		// Max advertisement interval (max * 0.625), add short delay to avoid aliasing
	NULL						// not directed, pass NULL
);

// Buffer for dynamic manufacturer data in advertisement
static uint8_t manuf_data_buffer[SENSOR_DATA_PACKET_SIZE];

// Advertising data structure (hold device name and adv type)
#define DEVICE_NAME 		CONFIG_BT_DEVICE_NAME		// Use the name defined in Kconfig
#define DEVICE_NAME_LEN 	(sizeof(DEVICE_NAME) - 1)

struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

// Scan response data (hold sensor data)
struct bt_data sd[] = {
	BT_DATA(BT_DATA_ENCRYPTED_AD_DATA, manuf_data_buffer, sizeof(manuf_data_buffer))
};

// BLE scan parameters
static const struct bt_le_scan_param scan_param = {
	.type = BT_HCI_LE_SCAN_PASSIVE,
	.options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
	.interval = BT_GAP_SCAN_FAST_INTERVAL,
	.window = BT_GAP_SCAN_FAST_WINDOW,
};

// Flag set by scan_cb to confirm at home
static volatile bool heartbeat_received_this_cycle = false;

/* -------------------- IMU Configurations -------------------- */

#define IMU_ACC_ODR_HIGH BMI270_ACC_ODR_50
#define IMU_ACC_ODR_LOW BMI270_ACC_ODR_25
#define IMU_GYR_ODR_HIGH BMI270_GYR_ODR_50
#define IMU_GYR_ODR_LOW BMI270_GYR_ODR_25

bmi270_acc_config_t bmi270_acc_config_high = {
	.acc_bwp = BMI270_ACC_BWP_NORM_AVG4,
	.acc_odr = BMI270_ACC_ODR_50,
	.acc_range = BMI270_ACC_RANGE_8G,
	.low_power_enable = 1,
};
bmi270_acc_config_t bmi270_acc_config_low = {
	.acc_bwp = BMI270_ACC_BWP_NORM_AVG4,
	.acc_odr = BMI270_ACC_ODR_25,
	.acc_range = BMI270_ACC_RANGE_4G,
	.low_power_enable = 1,
};
bmi270_gyr_config_t bmi270_gyr_config_high = {
	.gyr_bwp = BMI270_GYR_BWP_NORM,
	.gyr_odr = BMI270_GYR_ODR_50,
	.gyr_range = BMI270_GYR_RANGE_500,
	.low_power_enable = 0,
};
bmi270_gyr_config_t bmi270_gyr_config_low = {
	.gyr_bwp = BMI270_GYR_BWP_NORM,
	.gyr_odr = BMI270_GYR_ODR_25,
	.gyr_range = BMI270_GYR_RANGE_250,
	.low_power_enable = 0,
};

/* -------------------- GPIO Configuration -------------------- */

static const struct gpio_dt_spec leds[3] = {
	GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios), // Red LED
	GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios), // Green LED
	GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios), // Blue LED
};

// BMI270 Interrupt Pin
static const struct gpio_dt_spec bmi270_interrupts = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(bmi270), irq_gpios, {0});
static struct gpio_callback bmi270_interrupts_cb_data;

// BMP390 Interrupt Pin

/* -------------------- Sensor Device Handles -------------------- */

// BMI270
BMI270_Context bmi270_ctx;
#define SPIOP SPI_WORD_SET(8) | SPI_TRANSFER_MSB
static const struct spi_dt_spec bmi270_spi = SPI_DT_SPEC_GET(DT_NODELABEL(bmi270), SPIOP, 0);

// BMP390 (currently BME688 but can replace later)
const struct device *const dev = DEVICE_DT_GET_ONE(bosch_bme680);

/* -------------------- Discharge curve -------------------- This needs to be changed!! */

static const struct battery_level_point levels[] = {
	{10000, 3950},
	{625, 3550},
	{0, 3100},
};

/* -------------------- LittleFS Mount Configuration -------------------- */

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);
static struct fs_mount_t lfs_mount_p = {
	.type = FS_LITTLEFS,
	.fs_data = &storage,
	.storage_dev = (void *)PM_LITTLEFS_STORAGE_ID,
	.mnt_point = LFS_MOUNT_POINT,
};

/* -------------------- State Management Functions -------------------- */

/**
 * @brief Set IMU to with different sample frequency
 * @param high_rate Use high sample rate if set true
 */
static void set_imu_rate(bool high_rate)
{
	LOG_INF("Setting IMU rate to %s", high_rate ? "HIGH" : "LOW");

	bmi270_acc_config_t *acc_cfg = high_rate ? &bmi270_acc_config_high : &bmi270_acc_config_low;
	bmi270_gyr_config_t *gyr_cfg = high_rate ? &bmi270_gyr_config_high : &bmi270_gyr_config_low;

	bmi270_conf_acc(&bmi270_ctx, acc_cfg);
	bmi270_conf_gyr(&bmi270_ctx, gyr_cfg);
}

/**
 * @brief Start BLE advertising when enter HOME state
 */
static void start_advertising(void)
{
	int ret;

	ret = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (ret && ret != -EALREADY)
	{
		LOG_ERR("Failed to start advertisement: %d", ret);
	}
	else
	{
		LOG_INF("Advertising started/updated");
	}
}

/**
 * @brief Stop BLE advertising when enter AWAY state
 */
static void stop_advertising(void)
{
	int ret = bt_le_adv_stop();

	if (ret == 0 || ret == -EALREADY)
	{
		k_msleep(100);
		LOG_INF("Advertising stopped");
	}
	else
	{
		LOG_ERR("Failed to stop advertisement: %d", ret);
	}
}

/**
 * @brief Central function to manage state transitions and associated actions
 * @param new_state The target state to enter
 */
static void enter_state(device_state_t new_state)
{
	// Check our current state
	device_state_t old_state = atomic_set(&current_state, new_state);

	// If new = old, do nothing
	if (old_state == new_state)
	{
		LOG_INF("Already in state: %d", new_state);
		return;
	}

	LOG_INF("STATE TRANSITION: %d -> %d", old_state, new_state);

	// Actions on EXITING the old state
	switch (old_state)
	{
	case STATE_HOME_ADVERTISING:
		k_timer_stop(&heartbeat_timeout_timer); // stops checking for loss of AP
		break;
	case STATE_AWAY_LOGGING:
		break;
	case STATE_CHARGING:
		// Actions for stopping charging are handled by USB disconnect callback
		break;
	default: // STATE_INIT or others
		break;
	}

	// Actions on ENTERING the new state
	switch (new_state)
	{
	case STATE_HOME_ADVERTISING:
		LOG_INF("Entering Home Adv Mode");
		set_imu_rate(true);													   // Set high IMU rate and performance mode
		start_advertising();												   // Start BLE advertisement
		k_timer_start(&heartbeat_timeout_timer, HEARTBEAT_TIMEOUT, K_NO_WAIT); // Start timer to track in-home
		break;
	case STATE_AWAY_LOGGING:
		LOG_INF("Entering Away Log Mode");
		set_imu_rate(false); // Set low IMU rate and low-power mode
		stop_advertising();	 // Stop advertising
		// Heartbeat timer should be stopped already from leaving HOME
		break;
	case STATE_CHARGING:
		// Ensure other activities are stopped
		LOG_INF("Entering Charging Mode");
		stop_advertising();
		k_timer_stop(&heartbeat_timeout_timer);
		break;
	case STATE_INIT:
		stop_advertising();
		break;
	}
}

/**
 * @brief Callback function for heartbeat timeout workqueue. This
 * checks if we received heartbeat from APs to confirm we are still at home
 */
static void heartbeat_timeout_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	LOG_INF("Workqueue: Heartbeat timeout expired, entering AWAY");

	if (atomic_get(&current_state) == STATE_HOME_ADVERTISING)
	{
		enter_state(STATE_AWAY_LOGGING);
	}
	else
	{
		LOG_WRN("Workqueue: Heartbeat timeout work ran, but state was not HOME: %d", (int)atomic_get(&current_state));
	}
}

/**
 * @brief Callback function for USB connection workqueue. This checks if our
 * device is plugged in to the docking station
 */
static void usb_connect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	LOG_INF("Workqueue: USB connected, entering CHARGING");
	k_msleep(500);
	if (atomic_get(&current_state) != STATE_CHARGING)
	{
		enter_state(STATE_CHARGING);
	}
}

/**
 * @brief Callback function for USB disconnection workqueue. This checks if our
 * device is unplugged from the docking station
 */
static void usb_disconnect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	LOG_INF("Workqueue: USB disconnected, entering HOME");

	if (atomic_get(&current_state) != STATE_HOME_ADVERTISING)
	{
		enter_state(STATE_HOME_ADVERTISING);
	}
}

/**
 * @brief Callback function for scanner workqueue. This handles state transition to
 * HOME state. Needed to offload since BLE blocks.
 */
static void scan_found_ap_work_handler(struct k_work *k_work)
{
	ARG_UNUSED(k_work);
	LOG_INF("Workqueue: Scan found AP, entering HOME");

	if (atomic_get(&current_state) != STATE_HOME_ADVERTISING)
	{
		enter_state(STATE_HOME_ADVERTISING);
	}
	else
	{
		LOG_INF("Workqueue: In HOME, restart timer");
		k_timer_start(&heartbeat_timeout_timer, HEARTBEAT_TIMEOUT, K_NO_WAIT);
	}
}

/**
 * @brief ISR for bmi270 interrupt event, just give semaphore
 */
void bmi270_int1_interrupt_triggered(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_sem_give(&bmi270_isr_sem);
}

/**
 * @brief Callback function when battery timer expires. Offloaded to a workqueue.
 */
void battery_timer_expiry(struct k_timer *timer_id)
{
	int batt_mV = battery_sample();
	if (batt_mV >= 0)
	{ // Check for valid reading
		unsigned int bat_pptt = battery_level_pptt(batt_mV, levels);
		sensor_message_t msg = {.type = SENSOR_MSG_TYPE_BATTERY};
		msg.payload.batt.battery = (uint8_t)(bat_pptt / 100);
		msg.payload.batt.timestamp = k_uptime_get_32();

		if (k_msgq_put(&sensor_message_queue, &msg, K_NO_WAIT) != 0)
		{
			LOG_WRN("Battery queue full");
		}
		LOG_DBG("Battery level: %u%% (%d mV)", msg.payload.batt.battery, batt_mV);
	}
	else
	{
		LOG_ERR("Failed to read battery voltage: %d", batt_mV);
	}
}

/**
 * @brief Offload to workqueue when heartbeat timer expires
 */
void heartbeat_timeout_expiry(struct k_timer *timer_id)
{
	// This timer fires only if it wasn't stopped by receiving a heartbeat
	LOG_DBG("Heartbeat timer expired, submitting work");
	k_work_submit(&heartbeat_timeout_work);
}

/**
 * @brief BLE scanner callback function to receive heartbeat information. Maybe receive more information
 * from APs, TBD.
 */
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type, struct net_buf_simple *buf)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	static const char target_addr[] = TARGET_AP_ADDR;

	/* Convert the address to a string -- We only want the address part, not the type part */
	bt_addr_to_str(&addr->a, addr_str, sizeof(addr_str));

	if (strcmp(addr_str, target_addr) == 0)
	{
		LOG_INF("Heartbeat received from AP (%s)", addr_str);
		heartbeat_received_this_cycle = true; // Set flag for scanner thread

		k_work_submit(&scan_found_ap_work);

		for (int i = 0; i < buf->len; i++)
		{
			LOG_INF("0x%02X", buf->data[i]);
		}
	}
}

/**
 * @brief Callbacks when USB is connected to detect USB state
 */
static void usb_dc_status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	switch (status)
	{
	case USB_DC_CONNECTED:
		LOG_INF("USB Connected - Entering Charging State");
		k_work_submit(&usb_connect_work);
		break;
	case USB_DC_DISCONNECTED:
		LOG_INF("USB Disconnected");
		// Assume Home initially, scanner will correct if Away
		LOG_INF("Exiting Charging State - Default to Home");
		k_work_submit(&usb_disconnect_work);
		break;
	case USB_DC_CONFIGURED:
		LOG_INF("USB Configured by Host");
		if (atomic_get(&current_state) != STATE_CHARGING)
		{
			k_work_submit(&usb_connect_work);
		}
		LOG_INF("USB Ready for potential data offload commands");
		break;
	default:
		break;
	}
}

/* -------------------- Data Preparation -------------------- */

/**
 * @brief Unpad message queue to save space for BLE packet
 *
 * @param data Pointer to the received message queue
 * @param buffer Pointer to the actual BLE packet
 *
 */
void prepare_packet(const ble_packet_t *data, uint8_t *buffer, size_t buffer_size)
{
	if (buffer_size < SENSOR_DATA_PACKET_SIZE)
	{
		LOG_ERR("Buffer too small (%d < %d)", buffer_size, SENSOR_DATA_PACKET_SIZE);
		return;
	}

	int offset = 0;

	// Temperature: Assume we only need -30 to 40 degree with 1 degree C precision
	int temp_C = round((float)data->temperature / 100.0f);
	temp_C = CLAMP(temp_C, -TEMPERATURE_LOW_LIMIT, TEMPERATURE_HIGH_LIMIT); 
	uint8_t encoded_temp = (uint8_t)(temp_C + TEMPERATURE_LOW_LIMIT); 
	buffer[offset++] = encoded_temp;	// 1 byte

	// Pressure (Convert to uint16_t offset Pascals)
	uint32_t pressure_pa_x10 = data->pressure;
	uint32_t pressure_pa = pressure_pa_x10 / 10;
	uint16_t pressure_offset = 0;
	if (pressure_pa >= PRESSURE_BASE_PA)
	{
		uint32_t temp_offset = pressure_pa - PRESSURE_BASE_PA;
		// Clamp to uint16_t max if pressure exceeds expected range + base
		pressure_offset = (temp_offset > UINT16_MAX) ? UINT16_MAX : (uint16_t)temp_offset;
	} // else offset remains 0 (for pressure below base)
	sys_put_le16(pressure_offset, &buffer[offset]);
	offset += 2;	// 2 bytes

	/* IMU data */
	for (int i = 0; i < 6; i++)
	{
		sys_put_le16(data->imu_data[i], &buffer[offset]);
		offset += 2;
	} // 12 bytes

	/* Timestamp */
	sys_put_le32(data->timestamp, &buffer[offset]);
	offset += 4;	// 4 bytes

	/* Battery percentage */
	buffer[offset++] = data->battery; // 1 byte

	// Check total size
	if (offset != SENSOR_DATA_PACKET_SIZE)
	{
		LOG_WRN("BLE packet size mimatch: expected %d, got %d", SENSOR_DATA_PACKET_SIZE, offset);
	}
}

/* ----------------------------------------------------------------------------- */

/* BMI270 handler thread */
static void bmi270_handler_func(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	LOG_INF("BMI270 handler thread started");

	while (1)
	{
		// Wait indefinitely for the BMI270 interrupt signal
		if (k_sem_take(&bmi270_isr_sem, K_FOREVER) == 0)
		{
			// semaphore taken, read data
			BMI270_IMU_Value raw_value;

			// Determine which config to use based on current state
			device_state_t state = atomic_get(&current_state);
			uint8_t acc_range = (state == STATE_HOME_ADVERTISING) ? bmi270_acc_config_high.acc_range : bmi270_acc_config_low.acc_range;
			uint8_t gyr_range = (state == STATE_HOME_ADVERTISING) ? bmi270_gyr_config_high.gyr_range : bmi270_gyr_config_low.gyr_range;

			bool read_ok = bmi270_read_imu(&bmi270_ctx, &raw_value, acc_range, gyr_range);

			if (read_ok == 0)
			{
				sensor_message_t msg = {.type = SENSOR_MSG_TYPE_IMU};
				msg.payload.imu.imu_data[0] = (int16_t)(sensor_value_to_double(&raw_value.accel.x) * 100);
				msg.payload.imu.imu_data[1] = (int16_t)(sensor_value_to_double(&raw_value.accel.y) * 100);
				msg.payload.imu.imu_data[2] = (int16_t)(sensor_value_to_double(&raw_value.accel.z) * 100);
				msg.payload.imu.imu_data[3] = (int16_t)(sensor_value_to_double(&raw_value.gyro.x) * 100);
				msg.payload.imu.imu_data[4] = (int16_t)(sensor_value_to_double(&raw_value.gyro.y) * 100);
				msg.payload.imu.imu_data[5] = (int16_t)(sensor_value_to_double(&raw_value.gyro.z) * 100);
				msg.payload.imu.timestamp = k_uptime_get_32();

				if (k_msgq_put(&sensor_message_queue, &msg, K_NO_WAIT) != 0)
				{
					LOG_WRN("BMI270 queue full");
				}

				// LOG_DBG("Timestamp: %u, Accel Z: %.2f",
				// 	msg.payload.imu.timestamp, (double)(msg.payload.imu.imu_data[2] / 100));
			}
			else
			{
				LOG_ERR("Failed to read from BMI270 in hanlder thread: %d", read_ok);
			}
		}
	}
}

/* BMP390 hanlder thread: simulated by BME680 pulling  */
static void bmp390_handler_func(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	LOG_INF("BMP390 hanlder thread started (polling at %d ms interval)", BMP390_READ_INTERVAL);

	while (1)
	{
		// --- 1Hz BME680 Reading ---
		if (!dev || !device_is_ready(dev))
		{
			LOG_ERR("BME680 device not ready in handler thread");
			k_sleep(K_SECONDS(5));
			continue;
		}

		struct sensor_value temp, press;
		int ret = sensor_sample_fetch(dev);
		if (ret < 0)
		{
			LOG_ERR("Failed to fetch from BME688: %d", ret);
			k_sleep(K_MSEC(500));
			continue;
		}

		if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) == 0 && sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press) == 0)
		{
			sensor_message_t msg = {.type = SENSOR_MSG_TYPE_ENVIRONMENT};
			msg.payload.env.temperature = (int16_t)(sensor_value_to_double(&temp) * 100);
			msg.payload.env.pressure = (uint32_t)(sensor_value_to_double(&press) * 100 * 10);
			msg.payload.env.timestamp = k_uptime_get_32();

			if (k_msgq_put(&sensor_message_queue, &msg, K_NO_WAIT) != 0)
			{
				LOG_WRN("BMP390 queue full");
			}

			LOG_DBG("Timestamp: %u, Temperature: %.2f",
					msg.payload.env.timestamp, (double)(msg.payload.env.temperature / 100));
		}
		else
		{
			LOG_ERR("Failed to read BMP390 (simulated): %d", ret);
		}

		k_msleep(BMP390_READ_INTERVAL);
	}
}

/**
 * @brief Consumer thread to send data via BLE advertisement or log data to
 * the external flash using littlefs
 */
static void ble_logger_func(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	LOG_INF("BLE Logger thread started");

	sensor_message_t received_msg;						// Received from message queue
	ble_packet_t current_sensor_state = {0};			// Initialise
	current_sensor_state.timestamp = k_uptime_get_32(); // Set initial timestamp
	bool state_needs_update = false;					// Flag to reduce ADV updates

	struct fs_file_t log_file;			  // File object for littlefs
	bool file_is_open = false;			  // Track if log file is currently open
	int log_write_count = 0;			  // Counter for periodic sync
	static bool log_storage_full = false; // Flag to indicate storage is full

	fs_file_t_init(&log_file); // Initialise file object structure

	while (1)
	{
		/* Get each message from the message queue */
		int ret = k_msgq_get(&sensor_message_queue, &received_msg, K_FOREVER);
		if (ret == 0)
		{
			state_needs_update = true; // Received new data

			// Update the local copy of the full sensor state
			switch (received_msg.type)
			{
			case SENSOR_MSG_TYPE_IMU:
				memcpy(current_sensor_state.imu_data, received_msg.payload.imu.imu_data, sizeof(current_sensor_state.imu_data));
				current_sensor_state.timestamp = received_msg.payload.imu.timestamp;
				break;
			case SENSOR_MSG_TYPE_ENVIRONMENT:
				current_sensor_state.temperature = received_msg.payload.env.temperature;
				current_sensor_state.pressure = received_msg.payload.env.pressure;
				// Use newest timestamp if relevant
				if (received_msg.payload.env.timestamp > current_sensor_state.timestamp)
				{
					current_sensor_state.timestamp = received_msg.payload.env.timestamp;
				}
				break;
			case SENSOR_MSG_TYPE_BATTERY:
				current_sensor_state.battery = received_msg.payload.batt.battery;
				if (received_msg.payload.batt.timestamp > current_sensor_state.timestamp)
				{
					current_sensor_state.timestamp = received_msg.payload.batt.timestamp;
				}
				break;
			default:
				state_needs_update = false; // No relevant update
				break;
			}
		}
		else
		{
			LOG_ERR("Failed to get message from queue: %d", ret);
			state_needs_update = false;
			k_sleep(K_SECONDS(1));
			continue;
		}

		// Check the current device state
		device_state_t active_state = atomic_get(&current_state);

		// Perform actions based on state only if new data arrived
		if (state_needs_update)
		{
			if (active_state == STATE_HOME_ADVERTISING)
			{
				if (file_is_open)
				{
					LOG_INF("Home mode: Closing log file.");
					ret = fs_sync(&log_file); // Ensure data is flushed
					if (ret < 0)
					{
						LOG_ERR("Failed to sync log file before closing: %d", ret);
					}
					ret = fs_close(&log_file);
					if (ret < 0)
					{
						LOG_ERR("Failed to close log file: %d", ret);
					}
					file_is_open = false;
					log_write_count = 0; // Reset sync counter
				}

				// Update BLE advertisement
				prepare_packet(&current_sensor_state, manuf_data_buffer, sizeof(manuf_data_buffer));

				// Update the advertising data (non-blocking)
				ret = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
				if (ret && ret != -EALREADY)
				{
					LOG_WRN("Failed to update ADV data: %d", ret);
				}

				state_needs_update = false;
			}
			else if (active_state == STATE_AWAY_LOGGING)
			{
				// Ensure BLE update flag is false if we are only logging
				state_needs_update = false;

				// Only log IMU data in away mode
				if (received_msg.type == SENSOR_MSG_TYPE_IMU)
				{
					// Open file if not already open and storage is not full
					if (!file_is_open && !log_storage_full)
					{
						LOG_INF("Away mode: Opening log file %s", LOG_FILE_PATH);

						int ret = fs_open(&log_file, LOG_FILE_PATH, FS_O_APPEND | FS_O_CREATE | FS_O_WRITE);

						if (ret < 0)
						{
							LOG_ERR("Failed to open log file %s: %d", LOG_FILE_PATH, ret);
							k_sleep(K_SECONDS(5));
							continue;
						}
						else
						{
							LOG_INF("Log file opened successfully");
							file_is_open = true;
							log_write_count = 0; // Reset sync counter on open
						}
					}

					// Prepare binary log record only if file is open
					if (file_is_open && !log_storage_full)
					{
						// Prepare binary log record
						struct __attribute__((packed))
						{
							uint32_t timestamp;
							int16_t imu[6];
						} log_record;

						log_record.timestamp = received_msg.payload.imu.timestamp;
						memcpy(log_record.imu, received_msg.payload.imu.imu_data, sizeof(log_record.imu));

						// Write the record
						ssize_t written = fs_write(&log_file, &log_record, sizeof(log_record));
						if (written < 0)
						{
							LOG_ERR("Failed to write to log file: %d", (int)written);
							fs_close(&log_file);
							file_is_open = false;
						}
						else if (written < sizeof(log_record))
						{
							LOG_WRN("Partial write to log file: %d / %u bytes", (int)written, sizeof(log_record));
							fs_close(&log_file);
							file_is_open = false;
						}
						else
						{
							log_write_count++;

							// Sync periodically to flush cache to flash (e.g., every 1000 records)
							// Adjust based on data rate and acceptable data loss on power failure
							LOG_DBG("Logged IMU data (%d bytes), write count: %d",
									(int)written, log_write_count);

							if (log_write_count >= 1000)
							{
								ret = fs_sync(&log_file);
								if (ret < 0)
								{
									LOG_WRN("Log file sync failed: %d", ret);
								}
								else
								{
									LOG_DBG("Log file synced");
								}
								log_write_count = 0;
							}
						}
					}
					else if (log_storage_full)
					{
						// Optional: Log periodically that storage is full
						static uint32_t last_full_log = 0;
						if (k_uptime_get_32() - last_full_log > 30000)
						{
							LOG_WRN("Storage is full. No longer logging data.");
							last_full_log = k_uptime_get_32();
						}
					}
				}
			}
			else
			{
				// Close file if open (transitioned from AWAY)
				if (file_is_open)
				{
					LOG_INF("State %d: Closing log file.", active_state);
					ret = fs_sync(&log_file); // Sync before closing
					if (ret < 0)
					{
						LOG_ERR("Failed to sync log file before closing: %d", ret);
					}
					ret = fs_close(&log_file);
					if (ret < 0)
					{
						LOG_ERR("Failed to close log file: %d", ret);
					}
					file_is_open = false;
					log_write_count = 0;
				}
			}
			// Reset update flag after processing
			state_needs_update = false;
		}
		// Loop continues, blocking on k_msgq_get
	}
}

/* Scanner Thread: Periodically scans for AP heartbeat, triggers state changes */
static void scanner_func(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	LOG_INF("Scanner thread started");

	while (1)
	{
		// Only perform scan logic if not charging
		if (atomic_get(&current_state) != STATE_CHARGING)
		{
			heartbeat_received_this_cycle = false; // Reset flag before scan
			LOG_INF("Starting heartbeat scan ...");

			// Start scanning (scan_cb will be called for received packets)
			int ret = bt_le_scan_start(&scan_param, scan_cb);
			if (ret != 0)
			{
				LOG_ERR("Failed to start scanning: %d", ret);
				// Don't start timeout timer if scan failed
			}
			else
			{
				// Scan runs for SCAN_WINDOW duration
				k_sleep(SCAN_WINDOW);

				// Stop scanning
				ret = bt_le_scan_stop();
				if (ret && ret != -EALREADY)
				{
					LOG_ERR("Failed to stop scan: %d", ret);
				}
				LOG_INF("Scan window finished");
				// Check the flag set by scan_cb
				// If we are AWAY and received a heartbeat, scan_cb already triggered state change
				// If we are HOME and did NOT receive a heartbeat, the heartbeat_timeout_timer will fire
				// If we are HOME and DID receive a heartbeat, scan_cb restarted the timer
				// No explicit state change needed here based *only* on the flag after scan stops.
				// The timer expiry / scan_cb handle the transitions.
			}
		}
		else
		{
			LOG_INF("Skipping scan during CHARGING state");
		}

		// Wait for the next scan interval regardless of state (unless very low power needed in charge)
		k_sleep(SCAN_INTERVAL);
	}
}

int main(void)
{
	int ret;

	device_state_t initial_state = STATE_HOME_ADVERTISING;

	LOG_INF("===== Thingy Application Starting =====");

	// --- Initialise Core Peripherals ---
	// LEDs, Button GPIOs and Interrupts
	for (int i = 0; i < 3; i++)
	{
		if (!gpio_is_ready_dt(&leds[i]))
		{
			LOG_ERR("LED%d on pin %d is not ready\n", i, leds[i].pin);
			return -1;
		}
		else
		{
			ret = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
			if (ret)
			{
				LOG_ERR("Failed to configure LED%d\n", i);
				return -1;
			}
		}
	}

	LOG_INF("Step 1: GPIOs configured");

	// --- Initialise Sensors ---

	// BMI270 (IMU)
	bool bmi270_ret;
	if (!gpio_is_ready_dt(&bmi270_interrupts))
	{
		LOG_ERR("Interrupt GPIO is not ready on pin %d", bmi270_interrupts.pin);
		return -1;
	}
	ret = gpio_pin_configure_dt(&bmi270_interrupts, GPIO_INPUT);
	if (ret)
	{
		LOG_ERR("Failed to configure BMI270 interrupt as input");
		return -1;
	}
	gpio_pin_interrupt_configure_dt(&bmi270_interrupts, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0)
	{
		LOG_ERR("Error %d: failed to configure interrupt on %s pin %d",
				ret, bmi270_interrupts.port->name, bmi270_interrupts.pin);
		return -1;
	}
	gpio_init_callback(&bmi270_interrupts_cb_data, bmi270_int1_interrupt_triggered, BIT(bmi270_interrupts.pin));
	gpio_add_callback_dt(&bmi270_interrupts, &bmi270_interrupts_cb_data);

	LOG_INF("Step 2.1: BMI270 interrupts configured");

	/* Check SPI bus */
	if (!spi_is_ready_dt(&bmi270_spi))
	{
		gpio_pin_toggle_dt(&leds[0]);
		return -1;
	}
	/* Configurations of BMI270 */
	bmi270_ret = bmi270_init(&bmi270_ctx, &bmi270_spi);
	if (!bmi270_ret)
	{
		gpio_pin_toggle_dt(&leds[0]);
		LOG_ERR("Failed to initialise BMI270 sensor");
		return -1;
	}

	bmi270_fifo_config_t bmi270_fifo_default_config = {
		.acc_fifo_downs = BMI270_ACC_FIFO_DOWNS_1,
		.acc_filter = 1,
		.fifo_acc_en = 1,
		.fifo_aux_en = 0,
		.fifo_gyr_en = 1,
		.fifo_header_en = 1,
		.fifo_stop_on_full = 0,
		.gyr_fifo_downs = BMI270_GYR_FIFO_DOWNS_1,
		.gyr_filter = 1,
		.ret_sensor_time = 0};

	ret = bmi270_conf_fifo(&bmi270_ctx, 4096, &bmi270_fifo_default_config);

	bmi270_set_mode(&bmi270_ctx, BMI270_NORMAL_MODE, 1, 1);

	bmi270_conf_acc(&bmi270_ctx, &bmi270_acc_config_high);
	bmi270_conf_gyr(&bmi270_ctx, &bmi270_gyr_config_high);

	bmi270_interrupt_config_t int1_config = {
		.input_en = 0,
		.lvl = 1,
		.odb = 0,
		.output_en = 1,
		.interrupt_type.data_int = DRDY_INT,
		.use_data = 1,
		.int_latch = 0,
	};
	bmi270_conf_interrupt(&bmi270_ctx, &int1_config, 1);

	bmi270_interrupt_config_t int1_feature_config = {
		.input_en = 0,
		.lvl = 1,
		.odb = 0,
		.output_en = 1,
		.interrupt_type.feature_int = NO_MOTION_OUT,
		.use_data = 0,
		.int_latch = 0,
	};
	bmi270_conf_interrupt(&bmi270_ctx, &int1_feature_config, 1);

	bmi270_no_motion_config_t no_motion_param = {
		.duration = 3,
		.enable = 1,
		.select_x = 1,
		.select_y = 1,
		.select_z = 1,
		.threshold = 0x90,
	};
	bmi270_conf_no_motion(&bmi270_ctx, &no_motion_param);

	LOG_INF("Step 2.2: BMI270 sensor feature configured");

	// BME688 (Environmental)
	if (!device_is_ready(dev))
	{
		LOG_ERR("BME688: device not ready");
		return -1;
	}

	LOG_INF("Step 3: BME688 sensor ready");

	// Battery voltage monitoring
	ret = battery_measure_enable(true);
	if (ret != 0)
	{
		LOG_ERR("Failed to initialise battery measurement: %d", ret);
		return -1;
	}

	LOG_INF("Step 4: Enable battery voltage measurement");

	struct fs_mount_t *mp = &lfs_mount_p;

	ret = fs_mount(mp);
	if (ret < 0)
	{
		LOG_WRN("LittleFS mount failed (%d). Attempting format...", ret);
	}
	else
	{
		LOG_INF("LittleFS mounted on %s", mp->mnt_point);
	}
	// Log FS stats if mount succeeded
	if (ret == 0)
	{
		struct fs_statvfs stats;
		if (fs_statvfs(mp->mnt_point, &stats) == 0)
		{
			LOG_INF("%s: bsize = %lu ; frsize = %lu ;"
					" blocks = %lu ; bfree = %lu\n",
					mp->mnt_point,
					stats.f_bsize, stats.f_frsize,
					stats.f_blocks, stats.f_bfree);
		}
	}

	LOG_INF("Step 5: File system mounted");

	// --- Initialise Communication ---
	// BLE
	ret = bt_enable(NULL);
	if (ret)
	{
		LOG_ERR("Failed to enable BLE stack (err: %d)\n", ret);
		return -1;
	}

	LOG_INF("Step 6: Bluetooth enabled");

	// USB Device Subsystem
	ret = usb_enable(usb_dc_status_cb); // Register callback
	if (ret)
	{
		LOG_ERR("Failed to enable USB: %d", ret);
		initial_state = STATE_HOME_ADVERTISING; // If USB fails, we are not charging
	}
	else
	{
		LOG_INF("Step 7: USB callback registered");
	}

	// --- Initialise Timers ---
	k_timer_init(&heartbeat_timeout_timer, heartbeat_timeout_expiry, NULL);
	k_timer_init(&battery_timer, battery_timer_expiry, NULL);

	// Start the periodic battery timer
	k_timer_start(&battery_timer, BATTERY_READ_INTERVAL, BATTERY_READ_INTERVAL);

	// Initialise workqueue items
	k_work_init(&heartbeat_timeout_work, heartbeat_timeout_work_handler);
	k_work_init(&usb_connect_work, usb_connect_work_handler);
	k_work_init(&usb_disconnect_work, usb_disconnect_work_handler);
	k_work_init(&scan_found_ap_work, scan_found_ap_work_handler);

	LOG_INF("Step 8: Timers and WQ initialized and Battery timer started");

	/* LED to indicate successful initialisation sequence */
	for (int i = 0; i < 3; i++)
	{
		gpio_pin_set_dt(&leds[i], 1);
		k_msleep(250);
		gpio_pin_set_dt(&leds[i], 0);
	}

	// --- Create Threads ---
	k_thread_create(&bmi270_handler_thread_data, bmi270_handler_stack_area,
					K_THREAD_STACK_SIZEOF(bmi270_handler_stack_area), bmi270_handler_func, NULL, NULL, NULL,
					BMI270_HANDLER_PRIORITY, 0, K_MSEC(1000));

	k_thread_create(&bmp390_handler_thread_data, bmp390_handler_stack_area,
					K_THREAD_STACK_SIZEOF(bmp390_handler_stack_area), bmp390_handler_func, NULL, NULL, NULL,
					BMP390_HANDLER_PRIORITY, 0, K_MSEC(1000));

	k_thread_create(&ble_logger_thread_data, ble_logger_stack_area,
					K_THREAD_STACK_SIZEOF(ble_logger_stack_area), ble_logger_func, NULL, NULL, NULL,
					BLE_THREAD_PRIORITY, 0, K_MSEC(1000));

	k_thread_create(&scanner_thread_data, scanner_stack_area,
					K_THREAD_STACK_SIZEOF(scanner_stack_area), scanner_func, NULL, NULL, NULL,
					SCANNER_THREAD_PRIORITY, 0, K_MSEC(1000));

	// --- Initial State ---
	// Start assuming HOME, scanner/USB callback will correct quickly if needed.
	LOG_INF("Setting *initial state* determined state to: %d", initial_state);
	enter_state(initial_state);

	LOG_INF("Entering main loop (idle)");

	while (1)
	{
		// Main thread can sleep or handle very low priority tasks
		k_sleep(K_MINUTES(5));
	}

	return 0;
}
