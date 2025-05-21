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
#include "driver/battery.h"
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
#include <psa/crypto.h>
#include <psa/crypto_values.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/sys/util.h>
#include <zephyr/random/random.h>

LOG_MODULE_REGISTER(THINGY, LOG_LEVEL_DBG);

/* -------------------- Watchdog Timer --------------------*/

#define WDT_TIMEOUT_MS 			4000	// Watchdog timeout
#define WDT_FEED_INTERVAL_MS 	1000	// Watchdog feed interval

static const struct device *wdt_dev = DEVICE_DT_GET_ONE(nordic_nrf_wdt); // Get the WDT device
static int wdt_channel_id = -1;											 // Initialize the WDT channel ID
static struct k_timer wdt_feed_timer;									 // Timer for feeding the watchdog

/* -------------------- BLE Packet Encryption -------------------- */

#define BORUS_SETTINGS_PATH	"borus/state" // Save nonce in NVM, allow reboot
#define NONCE_SAVE_INTERVAL	5 * 60 * 1000 // Save nonce every 5 minutes

static psa_key_id_t g_aes_key_id = PSA_KEY_ID_NULL; // Initialize the AES key ID
static uint64_t nonce_counter = 0;					// Unique nonce for each BLE message

/* -------------------- Thread Configurations -------------------- */

// Stack sizes
#define BMI270_HANDLER_STACKSIZE 	1024
#define BMP390_HANDLER_STACKSIZE 	1024
#define BLE_LOGGER_THREAD_STACKSIZE 4096
#define SCANNER_THREAD_STACKSIZE 	1024

// Priorities (Lower number = higher priority)
#define BMI270_HANDLER_PRIORITY 5 	// Highest sensor priority due to higher sample rate
#define BMP390_HANDLER_PRIORITY 6 	// Medium sensor priority due to lower sample rate
#define BLE_THREAD_PRIORITY 	7	// Lower priority tasks for BLE and logging
#define SCANNER_THREAD_PRIORITY 7 	// Lower priority tasks for scan AP heartbeat

// Thread Stacks
K_THREAD_STACK_DEFINE(bmi270_handler_stack_area, BMI270_HANDLER_STACKSIZE);
K_THREAD_STACK_DEFINE(bmp390_handler_stack_area, BMP390_HANDLER_STACKSIZE);
K_THREAD_STACK_DEFINE(ble_logger_stack_area, BLE_LOGGER_THREAD_STACKSIZE);
K_THREAD_STACK_DEFINE(scanner_stack_area, SCANNER_THREAD_STACKSIZE);

// Thread Control Blocks
static struct k_thread bmi270_handler_thread_data;
static struct k_thread bmp390_handler_thread_data;
static struct k_thread ble_logger_thread_data;

/* -------------------- State Machine -------------------- */

// Define the state structure
typedef enum
{
	STATE_INIT,				// Initial state before normal operation
	STATE_HOME_ADVERTISING, // At home, advertise sensor data
	STATE_AWAY_LOGGING,		// Away, log sensor data
	STATE_CHARGING			// USB connected
} device_state_t;

static struct k_work battery_timeout_work; 	// Work item for battery timeout -> Periodic voltage reading
static struct k_work usb_connect_work;	   	// Work item for USB connect -> CHARGING state
static struct k_work usb_disconnect_work;  	// Work item for USB disconnect -> HOME state (or chosen default)
static struct k_work scan_found_ap_work;   	// Work item for Scan Found AP -> HOME state
static struct k_work scan_open_work;		
static struct k_work scan_close_work;
static struct k_work sync_check_work;
static struct k_work_delayable sync_adv_stop_work;
static struct k_work_delayable sync_scan_trigger_work;	

// Use atomic type for state changes between threads/ISRs
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

/* -------------------- Timers -------------------- */

static struct k_timer battery_timer;	// For periodic battery reading
static struct k_timer sync_check_timer; // For scan open
static struct k_timer scan_close_timer; // For scan close

/* -------------------- Configuration Constants -------------------- */

#define BMP390_READ_INTERVAL 					1000 			// Read environment at 1 Hz
#define BATTERY_READ_INTERVAL 					K_MINUTES(15)	// Every 15 minute read one battery voltage
#define SENSOR_ADV_PAYLOAD_TYPE 				0x00			// Custom packet type 0x00: Sensor data
#define SYNC_REQ_ADV_PAYLOAD_TYPE 				0x01			// Custom packet type 0x01: Time sync data
#define MAX_IMU_SAMPLES_IN_PACKET 				14				// (251 - 9)/16: 251 bytes max - 9 bytes (env, nonce, etc.), divided by IMU (16 bytes)
#define ADV_PAYLOAD_UPDATE_INTERVAL_MS 			140				// If sample at 100Hz (10ms) * 14 samples = 140 ms 
#define AGGREGATED_SENSOR_DATA_PLAINTEXT_SIZE	229				// 1+2+1+1+16*14: see prepare packet
#define AGGREGATED_ENC_ADV_PAYLOAD_LEN 			(1 + NONCE_LEN + AGGREGATED_SENSOR_DATA_PLAINTEXT_SIZE)	// Custom type + Nonce + Payload
#define NONCE_LEN 								8				// Size of the encryption nonce
#define SYNC_REQ_PAYLOAD_LEN 					(1 + 2)			// Custom type + Time
#define PRESSURE_BASE_HPA_X10 					8500 			// Base offset in hPa x 10
#define TEMPERATURE_LOW_LIMIT 					30   			// -30 degree as the lowest temperature of interest
#define TEMPERATURE_HIGH_LIMIT 					40  			// +40 degree as the highest temperature of interest

/* -------------------- File system and MSC -------------------- */

#define LOG_FILE_PATH "/lfs1/imu_log.bin" 				// File path for the log file
#define LFS_MOUNT_POINT "/lfs1"			  				// Mount point for the file system

USBD_DEFINE_MSC_LUN(NAND, "Zephyr", "BORUS", "0.00");	// Define the USB MSC LUN

/* -------------------- BLE Configurations -------------------- */

#define SYNC_CHECK_INTERVAL_BASE_MS 	1 * 30 * 1000	// Period to perform an at-home check 
#define SYNC_REQ_ADV_BURST_DURATION 	K_MSEC(250) 	// Duration to send type 0x01 advertisment
#define X_ANNOUNCED_S 					3				// Tells the scanner I am going to scan in 3 seconds
#define MISSES_BEFORE_AWAY 				3				// If miss 3 HB check from the RPi, consider AWAY
#define AWAY_BACKOFF_MAX_INTERVAL_MS	2 * 60 * 1000	// Linearly increase the scan interval while AWAY
#define EARLY_MARGIN_MS 				500				// Guard time for scanning, plus the other half is the total scanning time
#define LATE_MARGIN_MS 					500				// Guard time for scanning
#define SYNC_SCAN_WINDOW_MS 			(EARLY_MARGIN_MS + LATE_MARGIN_MS)	// Total scanning time
#define EXT_ADV_INTERVAL_MIN			BT_GAP_ADV_FAST_INT_MIN_2			// Min advertise interval for sensor extended advertisement 
#define EXT_ADV_INTERVAL_MAX			192 			// Max advertise interval for sensor extended advertisement: 120ms / 0.625 = 192
#define SYNC_ADV_INTERVAL_MIN			80				// Min advertise interval for time sync request advertisement: 50ms / 0.625 = 80
#define SYNC_ADV_INTERVAL_MAX			84				// Max advertise interval for time sync request advertisement

// Define BLE packet structure - Not used in real BLE packet
typedef struct
{
	int16_t temperature;
	uint32_t pressure;
	int16_t imu_data[6];
	uint32_t timestamp;
	uint8_t battery;
} ble_packet_t;

// Buffer for dynamic manufacturer data in advertisement
static uint8_t manuf_plain_aggregated[AGGREGATED_SENSOR_DATA_PLAINTEXT_SIZE];
static uint8_t manuf_payload_sensor_aggregated[AGGREGATED_ENC_ADV_PAYLOAD_LEN];
static uint8_t manuf_payload_sync_req[SYNC_REQ_PAYLOAD_LEN];

/**
 * If enable BT_LE_ADV_OPT_SCANNABLE, then pass sd as scan response data
 * 	- Benifit: Scanner will see the name and flag of adv event, we are scannable undirected adv
 * 	- Lose: Double radio on time so consume more power
 *
 * If not enable BT_LE_ADV_OPT_SCANNABLE, then pass null to scan response data
 * 	- Save power as no scan response data is provided, we are non-connectable undirected adv
 * 	- Lose device name and data flag
 */

// Legacy BLE advertisement parameters
static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	BT_LE_ADV_OPT_USE_IDENTITY, 	// Use identity MAC for advertisement
	SYNC_ADV_INTERVAL_MIN,			// Min advertisement interval (min * 0.625)
	SYNC_ADV_INTERVAL_MAX,			// Max advertisement interval (max * 0.625), add short delay to avoid aliasing
	NULL							// not directed, pass NULL
);

// Extended BLE advertisement parameters
static struct bt_le_adv_param ext_adv_params_sensor = {
	.id = BT_ID_DEFAULT,
	.sid = 0,
	.secondary_max_skip = 0,
	.options = (BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_IDENTITY),
	.interval_min = EXT_ADV_INTERVAL_MIN,
	.interval_max = EXT_ADV_INTERVAL_MAX,
	.peer = NULL,
};

static struct bt_le_ext_adv *g_adv_sensor_ext_handle;	// Extended advertisement handler

// Type 0x00: Main adv packet (hold encrypted sensor data)
struct bt_data ad_sensor_agg[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_payload_sensor_aggregated, sizeof(manuf_payload_sensor_aggregated))};

// Type 0x01: Back home anouncement (hold next scan time)
struct bt_data ad_sync_req[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_payload_sync_req, sizeof(manuf_payload_sync_req))};

// BLE scan parameters
static const struct bt_le_scan_param scan_param = {
	.type = BT_HCI_LE_SCAN_PASSIVE,
	.options = BT_LE_SCAN_OPT_FILTER_DUPLICATE | BT_LE_SCAN_OPT_FILTER_ACCEPT_LIST,
	.interval = BT_GAP_SCAN_FAST_INTERVAL,
	.window = BT_GAP_SCAN_FAST_WINDOW,
};

// Define the list of target AP addresses
static const char *target_ap_addrs[] = {
	"2C:CF:67:89:E0:5D",	// Public address of the built-in RPi controller 
	"C0:54:52:53:00:00",	// Random static address of the nrf53840dk
};

static atomic_t adv_running_flag = ATOMIC_INIT(0);						 	// Flag to indicate if any advertisement is running
static atomic_t current_adv_type = ATOMIC_INIT(SENSOR_ADV_PAYLOAD_TYPE); 	// Flag to indicate current custom advertisement type
static volatile bool scan_active = false;									// Flag to indicate if scanning 

/* -------------------- IMU Configurations -------------------- */

#define IMU_ACC_ODR_HIGH BMI270_ACC_ODR_50
#define IMU_ACC_ODR_LOW BMI270_ACC_ODR_25
#define IMU_GYR_ODR_HIGH BMI270_GYR_ODR_50
#define IMU_GYR_ODR_LOW BMI270_GYR_ODR_25

bmi270_acc_config_t bmi270_acc_config_high = {
	.acc_bwp = BMI270_ACC_BWP_NORM_AVG4,
	.acc_odr = BMI270_ACC_ODR_100,
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
	.gyr_odr = BMI270_GYR_ODR_100,
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

#define LOG_SYNC_THRESHOLD 15000 // Record to log before fs_sync, this tolerates 5-min data missing

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);
static struct fs_mount_t lfs_mount_p = {
	.type = FS_LITTLEFS,
	.fs_data = &storage,
	.storage_dev = (void *)PM_LITTLEFS_STORAGE_ID,
	.mnt_point = LFS_MOUNT_POINT,
};

/* -------------------- Home Detection -------------------- */

static uint8_t missed_sync_responses = 0;
static uint32_t sync_check_interval_ms = SYNC_CHECK_INTERVAL_BASE_MS;

/**
 * @brief Get the current time in milliseconds.
 */
static inline uint64_t ms_now(void)
{
	return k_uptime_get();
}

/* -------------------- State Management Functions -------------------- */

/**
 * @brief Create an extended advertisement set for the sensor
 */
static int create_sensor_ext_adv_set(void)
{
	int err;

	err = bt_le_ext_adv_create(&ext_adv_params_sensor, NULL, &g_adv_sensor_ext_handle);
	if (err)
	{
		LOG_ERR("Failed to create sensor extended adv set: %d", err);
		return err;
	}
	LOG_DBG("Sensor extended adv. set created");
	return 0;
}

/**
 * @brief Set the IMU sampling rate and configuration.
 *
 * This function configures the IMU to use either a high or low sampling rate
 * based on the provided parameter. It adjusts both accelerometer and gyroscope
 * settings accordingly.
 *
 * @param high_rate Set to true for high sampling rate, false for low sampling rate.
 */
static void set_imu_rate(bool high_rate)
{
	LOG_DBG("Setting IMU rate to %s", high_rate ? "HIGH" : "LOW");

	bmi270_acc_config_t *acc_cfg = high_rate ? &bmi270_acc_config_high : &bmi270_acc_config_low;
	bmi270_gyr_config_t *gyr_cfg = high_rate ? &bmi270_gyr_config_high : &bmi270_gyr_config_low;

	bmi270_conf_acc(&bmi270_ctx, acc_cfg);
	bmi270_conf_gyr(&bmi270_ctx, gyr_cfg);
}

/**
 * @brief Setup the BLE scan filter to accept only specific target APs.
 */
static int setup_scan_filter(void)
{
	int ret;
	bt_addr_le_t addr_le;

	// Clearn any previous filter entries
	bt_le_filter_accept_list_clear();

	LOG_DBG("Populating BLE scan filter accept list:");

	// Convert string address to bt_addr_le_t.
	// We assume the target APs use public addresses ("public").
	// If they use Random Static, use "random" or the correct type string.
	ret = bt_addr_le_from_str(target_ap_addrs[0], "public", &addr_le);
	if (ret)
	{
		LOG_ERR("Invalid address string '%s': %d", target_ap_addrs[0], ret);
		// Decide how to handle: return error, skip, etc.
		return ret;
	}

	// Add the parsed address to the controller's filter accept list
	ret = bt_le_filter_accept_list_add(&addr_le);
	if (ret && ret != -EALREADY)
	{
		// Ignore if already added
		LOG_ERR("Failed to add '%s' to accept list: %d", target_ap_addrs[0], ret);
		// Decide how to handle: return error, skip, etc.
		// If one fails, the list might be partially populated.
	}

	ret = bt_addr_le_from_str(target_ap_addrs[1], "random", &addr_le);
	ret = bt_le_filter_accept_list_add(&addr_le);

	return 0;
}

/**
 * @brief Stop BLE advertising when enter AWAY state
 *
 * This function stops BLE advertising. It will not stop if already stopped.
 */
static void stop_all_advertising(void)
{
	int ret_legacy = 0;
	int ret_ext = 0;
	bool stopped_something = false;

	// Try to stop legacy advertising (which is now only for sync requests)
	if (atomic_get(&current_adv_type) == SYNC_REQ_ADV_PAYLOAD_TYPE && atomic_get(&adv_running_flag))
	{
		ret_legacy = bt_le_adv_stop(); // Stop legacy
		if (ret_legacy == 0)
		{
			LOG_DBG("Legacy advertising (sync_req) stopped");
			stopped_something = true;
		}
		else if (ret_legacy == -EALREADY)
		{
			LOG_DBG("Legacy advertising (sync_req) already stopped");
			stopped_something = true; // Effectively stopped from our perspective
		}
		else if (ret_legacy == -ECONNRESET)
		{
			LOG_WRN("Failed to stop legacy adv (sync_req), radio was reset: %d", ret_legacy);
		}
		else
		{
			LOG_ERR("Failed to stop legacy adv (sync_req): %d", ret_legacy);
		}
	}

	// Try to stop extended sensor advertising
	if (g_adv_sensor_ext_handle && atomic_get(&current_adv_type) == SENSOR_ADV_PAYLOAD_TYPE && atomic_get(&adv_running_flag))
	{
		ret_ext = bt_le_ext_adv_stop(g_adv_sensor_ext_handle);
		if (ret_ext == 0)
		{
			LOG_DBG("Extended sensor advertising stopped");
			stopped_something = true;
		}
		else if (ret_ext == -EALREADY)
		{
			LOG_DBG("Extended sensor advertising already stopped");
			stopped_something = true;
		}
		else if (ret_ext == -ECONNRESET)
		{
			LOG_WRN("Failed to stop extended adv (sensor), radio was reset: %d", ret_ext);
		}
		else if (ret_ext != -EINVAL)
		{ // -EINVAL if handle is invalid
			LOG_ERR("Failed to stop extended sensor adv: %d", ret_ext);
		}
	}

	// If we explicitly stopped something or if adv_running_flag was already 0
	if (stopped_something || !atomic_get(&adv_running_flag))
	{
		atomic_set(&adv_running_flag, 0);
		atomic_set(&current_adv_type, 2);
	}
	else if (!stopped_something && atomic_get(&adv_running_flag))
	{
		// This case means adv_running_flag was true, but we didn't match any current_adv_type
		// to stop it. This might indicate a state inconsistency. Reset flags.
		LOG_WRN("ADV was running but no matching type to stop. Resetting flags.");
		atomic_set(&adv_running_flag, 0);
		atomic_set(&current_adv_type, 2);
	}
}

/**
 * @brief Start BLE advertising when enter HOME state
 *
 * This function starts BLE advertising with the specified parameters and data.
 * It will not start if already running.
 */
static void start_sensor_advertising_ext(void)
{
	int ret;

	// Ensure legacy (sync_req) advertising is stopped
	if (atomic_get(&adv_running_flag)) { // Only try to stop if something is flagged as running
        LOG_DBG("start_sensor_advertising_ext: Stopping existing advertising first.");
        stop_all_advertising();
        k_msleep(50); // Allow controller time to process stop
    }

	if (!g_adv_sensor_ext_handle)
	{
		LOG_ERR("Sensor extended adv handle is NULL. Cannot start.");
		return;
	}

	ret = bt_le_ext_adv_set_data(g_adv_sensor_ext_handle, ad_sensor_agg, ARRAY_SIZE(ad_sensor_agg), NULL, 0);
	if (ret)
	{
		LOG_ERR("Failed to set sensor extended advertising data (err %d)", ret);
		return;
	}

	LOG_DBG("Starting Sensor Data extended advertising (Custom Type 0x%02X)", SENSOR_ADV_PAYLOAD_TYPE);
	// SENSOR_ADV_PAYLOAD_TYPE is the first byte *inside* manuf_payload_sensor
	ret = bt_le_ext_adv_start(g_adv_sensor_ext_handle, BT_LE_EXT_ADV_START_DEFAULT);

	if (ret && ret != -EALREADY)
	{
		LOG_ERR("Failed to start sensor extended advertisement: %d", ret);
		atomic_set(&adv_running_flag, 0);
		atomic_set(&current_adv_type, 2);
	}
	else
	{
		atomic_set(&adv_running_flag, 1);
		atomic_set(&current_adv_type, SENSOR_ADV_PAYLOAD_TYPE); // Mark as extended sensor type
		if (ret == -EALREADY)
			LOG_DBG("Sensor Ext Adv already started/updated.");
		else
			LOG_DBG("Sensor Ext Adv started/updated.");
	}
}

/**
 * @brief Start away advertisement burst.
 *
 * This function starts a BLE advertisement burst for the AWAY state.
 * It will not start if already running.
 */
static void start_sync_request_advertising(void)
{
	int ret;

	// Ensure other type is stopped first
	// Ensure extended (sensor) advertising is stopped
	if (atomic_get(&adv_running_flag)) { // Only try to stop if something is flagged as running
        LOG_DBG("start_sync_request_advertising: Stopping existing advertising first.");
        stop_all_advertising();
        // Add a small delay to allow the controller to process the stop command fully.
        // This is often necessary when rapidly switching advertising types/sets.
        k_msleep(20); // Try 20-100ms, tune as needed.
    }

	LOG_DBG("Starting AWAY Scan Announcement advertising burst (Type 0x01)");

	ret = bt_le_adv_start(adv_param, ad_sync_req, ARRAY_SIZE(ad_sync_req), NULL, 0);
	if (ret && ret != -EALREADY)
	{
		LOG_ERR("Failed to start AWAY Scan Announce advertisement: %d", ret);
		atomic_set(&adv_running_flag, 0);
		atomic_set(&current_adv_type, 2);
	}
	else if (ret == -EALREADY)
	{
		LOG_WRN("AWAY ADV already started");
	}
	else
	{
		LOG_DBG("AWAY Scan Announce advertising burst started");
		atomic_set(&adv_running_flag, 1);
		atomic_set(&current_adv_type, SYNC_REQ_ADV_PAYLOAD_TYPE);
	}
}

/**
 * @brief Stop the sync request advertisement burst.
 *
 * This function stops the BLE advertisement burst for the AWAY state.
 */
static void sync_adv_stop_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (atomic_get(&adv_running_flag) && atomic_get(&current_adv_type) == SYNC_REQ_ADV_PAYLOAD_TYPE)
	{
		stop_all_advertising();

		// If we were in HOME state before sync adv, restart sensor ext adv
		if (atomic_get(&current_state) == STATE_HOME_ADVERTISING)
		{
			LOG_DBG("Sync ADV burst finished, restarting sensor EXT adv.");
			start_sensor_advertising_ext();
		}
		else
		{
			LOG_DBG("Sync ADV burst finished, not in HOME state, no sensor adv restart.");
		}
	}
	else
	{
		LOG_WRN("Sync adv stop work ran, but adv not running or not sync type (flag: %d, type: %d)",
				(int)atomic_get(&adv_running_flag), (int)atomic_get(&current_adv_type));
	}
}

/**
 * @brief Trigger the scan open work item.
 *
 * This function is called when the sync scan trigger work item is executed.
 * It checks the current state and submits the scan open work if in the correct state.
 *
 * @param work Pointer to the work item.
 */
static void sync_scan_trigger_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	device_state_t state = atomic_get(&current_state);

	if (state == STATE_AWAY_LOGGING || STATE_HOME_ADVERTISING)
	{
		LOG_DBG("Sync Scan Triggers: Submitting scan_open_work");
		k_work_submit(&scan_open_work);
	}
	else
	{
		LOG_WRN("Sync Scan Trigger ran but state is %d", state);
	}
}

/**
 * @brief Manage state transitions and associated actions.
 *
 * This function handles entering and exiting states, performing the necessary
 * actions for each state transition.
 *
 * @param new_state The target state to enter.
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
		LOG_DBG("Exiting HOME: Stopping sensor ADV timer and radio.");
		stop_all_advertising();
		k_timer_stop(&sync_check_timer);
		k_timer_stop(&scan_close_timer);
		if (scan_active)
		{
			bt_le_scan_stop();
			scan_active = false;
		}
		break;
	case STATE_AWAY_LOGGING:
		LOG_DBG("Exiting AWAY: Stopping AWAY ADV timer, work, and radio.");

		k_timer_stop(&sync_check_timer);
		(void)k_work_cancel_delayable(&sync_adv_stop_work);
		(void)k_work_cancel_delayable(&sync_scan_trigger_work);
		stop_all_advertising();
		k_timer_stop(&scan_close_timer);
		if (scan_active)
		{
			bt_le_scan_stop();
			scan_active = false;
		}
		break;
	case STATE_CHARGING:
		LOG_DBG("Exiting CHARGING.");
		break;
	default:
		break;
	}

	// Actions on ENTERING the new state
	switch (new_state)
	{
	case STATE_HOME_ADVERTISING:
		LOG_INF("Entering Home Adv Mode");
		set_imu_rate(true); // Set high IMU rate and performance mode
		missed_sync_responses = 0;
		sync_check_interval_ms = SYNC_CHECK_INTERVAL_BASE_MS;
		start_sensor_advertising_ext();																	  // Start BLE advertisement
		k_timer_start(&sync_check_timer, K_MSEC(sync_check_interval_ms), K_MSEC(sync_check_interval_ms)); // Start timer to track in-home
		break;
	case STATE_AWAY_LOGGING:
		LOG_INF("Entering Away Log Mode");
		set_imu_rate(false);	// Set low IMU rate and low-power mode
		stop_all_advertising(); // Stop advertising
		uint32_t jitter = sys_rand32_get() % (SYNC_CHECK_INTERVAL_BASE_MS / 4);
		k_timer_start(&sync_check_timer, K_MSEC(jitter), K_MSEC(sync_check_interval_ms));
		break;
	case STATE_CHARGING:
		// Ensure other activities are stopped
		LOG_INF("Entering Charging Mode");
		stop_all_advertising();
		k_timer_stop(&sync_check_timer);
		(void)k_work_cancel_delayable(&sync_adv_stop_work);
		(void)k_work_cancel_delayable(&sync_scan_trigger_work);
		if (scan_active)
		{
			LOG_DBG("Stopped active scan on entering CHARGING state");
			bt_le_scan_stop();
			scan_active = false;
		}
		k_timer_stop(&scan_close_timer);
		break;
	case STATE_INIT:
		LOG_INF("Entering INIT state.");
		stop_all_advertising();
		if (scan_active)
		{
			bt_le_scan_stop();
			scan_active = false;
		}
		k_timer_stop(&scan_close_timer);
		k_timer_stop(&sync_check_timer);
		break;
	}
}

/**
 * @brief Callback function for BLE scanning.
 *
 * This function is called when a BLE advertisement is received. It checks if
 * the advertisement is from a target AP and submits the scan found AP work
 * to the workqueue.
 *
 * @param addr Pointer to the Bluetooth address of the advertiser.
 * @param rssi Received signal strength indicator.
 * @param adv_type Advertisement type.
 * @param buf Pointer to the advertisement data buffer.
 */
/* ------------------------------------------------------------------ */
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
					struct net_buf_simple *buf)
{
	if (atomic_get(&current_state) == STATE_CHARGING)
	{
		/* ignore all packets while charging */
		return;
	}

	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str)); // Still useful for logging
	LOG_INF("Heartbeat from %s, RSSI=%d", addr_str, rssi);

	// Stop scan and close timer
	if (scan_active)
	{
		bt_le_scan_stop();
		scan_active = false;
	}
	k_timer_stop(&scan_close_timer);
	(void)k_work_cancel_delayable(&sync_scan_trigger_work); // Cancel pending scan if we got a response early

	// Reset miss counter
	missed_sync_responses = 0;
	sync_check_interval_ms = SYNC_CHECK_INTERVAL_BASE_MS;

	// Handle state
	device_state_t s = atomic_get(&current_state);

	if (s == STATE_AWAY_LOGGING)
	{
		LOG_DBG("Sync Response Received: Transitioning to HOME state.");
		enter_state(STATE_HOME_ADVERTISING);
	}
	else if (s == STATE_HOME_ADVERTISING)
	{
		LOG_DBG("Sync Response Received: Confirmed HOME state.");
		start_sensor_advertising_ext();
	}
}

/**
 * @brief Queue the initial battery level to the message queue.
 *
 * This function reads the initial battery voltage and queues it to the
 * message queue for processing.
 */
static void queue_initial_battery_level(void)
{
	int batt_mV = battery_sample();
	if (batt_mV < 0)
	{
		LOG_ERR("Initial battery read failed: %d", batt_mV);
		return;
	}

	unsigned int batt_pptt = battery_level_pptt(batt_mV, levels);

	sensor_message_t msg = {.type = SENSOR_MSG_TYPE_BATTERY};
	msg.payload.batt.battery = (uint8_t)(batt_pptt / 100);
	msg.payload.batt.timestamp = k_uptime_get_32();

	if (k_msgq_put(&sensor_message_queue, &msg, K_NO_WAIT) != 0)
	{
		LOG_WRN("Initial battery queue full - value dropped");
	}
	else
	{
		LOG_INF("Step 4.2: Queued initial battery level: %u%% (%d mV)", msg.payload.batt.battery, batt_mV);
	}
}

/**
 * @brief Handle the away advertisement work item.
 *
 * This function is called when the away advertisement work item is triggered.
 * It prepares the advertisement payload and starts the AWAY advertisement burst.
 *
 * @param work Pointer to the work structure.
 */
static void sync_check_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	device_state_t state = atomic_get(&current_state);
	// Only proceed if we are actually in AWAY state
	if (state != STATE_AWAY_LOGGING && state != STATE_HOME_ADVERTISING)
	{
		LOG_WRN("Sync check work ran in unexpected state %d", state);
		return;
	}

	// Calculate the time until next scan window opens
	uint16_t time_to_announce_s = X_ANNOUNCED_S;
	LOG_DBG("Preparing Type 0x01 ADV. Announcing scan reference time: %u seconds", time_to_announce_s);

	// Prepare payload: [ Type | Time LSB | MSB ]
	manuf_payload_sync_req[0] = SYNC_REQ_ADV_PAYLOAD_TYPE;
	sys_put_le16(time_to_announce_s, &manuf_payload_sync_req[1]);

	start_sync_request_advertising();

	if (atomic_get(&adv_running_flag) && atomic_get(&current_adv_type) == SYNC_REQ_ADV_PAYLOAD_TYPE)
	{
		k_work_schedule(&sync_adv_stop_work, SYNC_REQ_ADV_BURST_DURATION);

		int32_t scan_trigger_delay_ms = (time_to_announce_s * 1000) - EARLY_MARGIN_MS;
		if (scan_trigger_delay_ms < 0)
		{
			scan_trigger_delay_ms = 0;
		}
		LOG_DBG("Scheduling scan trigger work in %d ms.", scan_trigger_delay_ms);

		int ret = k_work_schedule(&sync_scan_trigger_work, K_MSEC(scan_trigger_delay_ms));
		if (ret < 0)
		{
			LOG_ERR("Failed to schedule away scan trigger work (%d)", ret);
			stop_all_advertising();

			if (state == STATE_HOME_ADVERTISING)
			{
				start_sensor_advertising_ext();
			}
		}
	}
	else
	{
		LOG_ERR("Sync Req ADV failed to start, scan not scheduled");
	}
}

/**
 * @brief Handle the scan open work item.
 *
 * This function is called when the scan open work item is triggered. It starts
 * the BLE scan and stops advertising if necessary.
 *
 * @param w Pointer to the work structure.
 */
static void scan_open_work_handler(struct k_work *w)
{
	ARG_UNUSED(w);

	// Stop any advertising before scanning
	if (atomic_get(&adv_running_flag))
	{
		stop_all_advertising();
	}

	LOG_DBG("Scan Open Work: Starting scan (State: %d)", (int)atomic_get(&current_state));
	int err = bt_le_scan_start(&scan_param, scan_cb);
	if (err && err != -EALREADY)
	{
		LOG_ERR("bt_le_scan_start failed (%d)", err);
		atomic_set(&adv_running_flag, 0);
		return;
	}
	scan_active = true;

	k_timer_start(&scan_close_timer, K_MSEC(SYNC_SCAN_WINDOW_MS), K_NO_WAIT);
	LOG_DBG("Scan Open: Scan started (%d ms window).", SYNC_SCAN_WINDOW_MS);
}

/**
 * @brief Handle the scan close work item.
 *
 * This function is called when the scan close work item is triggered. It stops
 * the BLE scan and restarts advertising if necessary.
 *
 * @param w Pointer to the work structure.
 */
static void scan_close_work_handler(struct k_work *w)
{
	ARG_UNUSED(w);

	if (scan_active)
	{
		bt_le_scan_stop();
		scan_active = false;
	}
	LOG_DBG("Scan Close Work: Scan window finished (AP Response MISSING).");

	device_state_t state = atomic_get(&current_state);

	if (state == STATE_HOME_ADVERTISING)
	{
		missed_sync_responses++;
		LOG_WRN("Sync Response Missed (%u/%u) while HOME", missed_sync_responses, MISSES_BEFORE_AWAY);

		if (missed_sync_responses >= MISSES_BEFORE_AWAY)
		{
			LOG_WRN("Missed %u consecutive Sync Responses, entering AWAY", MISSES_BEFORE_AWAY);
			enter_state(STATE_AWAY_LOGGING);
		}
		else
		{
			if (!atomic_get(&adv_running_flag))
			{
				LOG_DBG("Scan Close (HOME miss): Restarting sensor advertising");
				start_sensor_advertising_ext();
			}
		}
	}
	else if (state == STATE_AWAY_LOGGING)
	{
		LOG_WRN("Sync Response MISSED while AWAY");
		sync_check_interval_ms = MIN(sync_check_interval_ms * 2, AWAY_BACKOFF_MAX_INTERVAL_MS);
		sync_check_interval_ms += sys_rand32_get() * (SYNC_CHECK_INTERVAL_BASE_MS / 2);
		sync_check_interval_ms = MIN(sync_check_interval_ms, AWAY_BACKOFF_MAX_INTERVAL_MS);

		LOG_INF("AWAY Backoff: Next sync check attempt in %u ms", sync_check_interval_ms);
		k_timer_start(&sync_check_timer, K_MSEC(sync_check_interval_ms), K_MSEC(sync_check_interval_ms));

		if (atomic_get(&adv_running_flag))
		{
			stop_all_advertising();
		}
	}
}

/**
 * @brief Handle battery timeout events.
 *
 * This function is called when the battery timer expires. It reads the battery
 * voltage, calculates the battery percentage, and queues the data for processing.
 *
 * @param work Pointer to the work structure.
 */
static void battery_timeout_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

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
 * @brief Handle USB connection events.
 *
 * This function is called when the USB is connected. It transitions the device
 * to the CHARGING state.
 *
 * @param work Pointer to the work structure.
 */
static void usb_connect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	LOG_DBG("Workqueue: USB connected, entering CHARGING");
	k_msleep(500);
	if (atomic_get(&current_state) != STATE_CHARGING)
	{
		enter_state(STATE_CHARGING);
	}
}

/**
 * @brief Handle USB disconnection events.
 *
 * This function is called when the USB is disconnected. It transitions the device
 * to the HOME state.
 *
 * @param work Pointer to the work structure.
 */
static void usb_disconnect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	LOG_DBG("Workqueue: USB disconnected, entering HOME");

	if (atomic_get(&current_state) == STATE_CHARGING)
	{
		LOG_DBG("Workqueue: USB Disconnected, entering AWAY state to initiate sync check");
		enter_state(STATE_HOME_ADVERTISING);
	}
	else
	{
		LOG_WRN("USB disconnect event when not in CHARGING state (%d). Forcing AWAY state.", (int)atomic_get(&current_state));
		enter_state(STATE_HOME_ADVERTISING);
	}
}

/**
 * @brief Handle AP scan results and transition to HOME state.
 *
 * This function is called when a target AP is found during scanning. It transitions
 * the device to the HOME state and restarts the heartbeat timer.
 *
 * @param k_work Pointer to the work structure.
 */
static void scan_found_ap_work_handler(struct k_work *k_work)
{
	ARG_UNUSED(k_work);

	if (atomic_get(&current_state) == STATE_CHARGING)
	{
		LOG_DBG("Workqueue: Scan found AP, but in CHARGING state");
		return;
	}

	LOG_DBG("Workqueue: Scan found AP, entering HOME");

	if (atomic_get(&current_state) != STATE_HOME_ADVERTISING)
	{
		LOG_INF("Workqueue: Scan found AP -> Entering HOME state.");
		enter_state(STATE_HOME_ADVERTISING);
	}
	else
	{
		LOG_DBG("Workqueue: Scan found AP while already HOME.");
	}
}

/**
 * @brief Check if the sync check timer has expired.
 *
 * This function is called when the sync check timer expires. It submits the
 * sync check work to the workqueue if in the correct state.
 *
 * @param timer_id Pointer to the timer structure.
 */
static void sync_check_timer_expiry(struct k_timer *timer_id)
{
	ARG_UNUSED(timer_id);
	device_state_t state = atomic_get(&current_state);
	if (state == STATE_AWAY_LOGGING || state == STATE_HOME_ADVERTISING)
	{
		k_work_submit(&sync_check_work);
	}
}

/**
 * @brief Handle the scan close timer expiry event.
 *
 * This function is called when the scan close timer expires. It submits the
 * scan close work to the workqueue.
 *
 * @param t Pointer to the timer structure.
 */
static void scan_close_timer_expiry(struct k_timer *t)
{
	ARG_UNUSED(t);
	k_work_submit(&scan_close_work);
}

/**
 * @brief ISR for BMI270 interrupt events.
 *
 * This function is triggered when the BMI270 interrupt pin is activated. It
 * signals the BMI270 handler thread using a semaphore.
 *
 * @param dev Pointer to the device structure.
 * @param cb Pointer to the GPIO callback structure.
 * @param pins Pin mask for the interrupt.
 */
void bmi270_int1_interrupt_triggered(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_sem_give(&bmi270_isr_sem);
}

/**
 * @brief Handle battery timer expiry events.
 *
 * This function is called when the battery timer expires. It submits the battery
 * timeout work to the workqueue.
 *
 * @param timer_id Pointer to the timer structure.
 */
void battery_timer_expiry(struct k_timer *timer_id)
{
	LOG_DBG("Heartbeat timer expired, submitting work");
	k_work_submit(&battery_timeout_work);
}

/**
 * @brief Feed the watchdog timer to prevent timeout.
 *
 * This function is called periodically to feed the watchdog timer and prevent
 * it from timing out.
 *
 * @param timer_id Pointer to the timer structure.
 */
static void watchdog_feed(struct k_timer *timer_id)
{
	const struct device *dev = (const struct device *)k_timer_user_data_get(timer_id);
	wdt_feed(dev, wdt_channel_id);
}

/**
 * @brief Callback function for USB device status changes.
 *
 * This function is called when the USB device status changes. It handles
 * connection and disconnection events, as well as configuration events.
 *
 * @param status The new USB device status.
 * @param param Pointer to additional parameters (not used).
 */
static void usb_dc_status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	switch (status)
	{
	case USB_DC_CONNECTED:
		LOG_DBG("USB Connected - Entering Charging State");
		k_work_submit(&usb_connect_work);
		break;
	case USB_DC_DISCONNECTED:
		LOG_DBG("USB Disconnected");
		k_work_submit(&usb_disconnect_work);
		break;
	case USB_DC_CONFIGURED:
		LOG_INF("USB Configured by Host, ready for reading");
		if (atomic_get(&current_state) != STATE_CHARGING)
		{
			k_work_submit(&usb_connect_work);
		}
		break;
	default:
		break;
	}
}

/* -------------------- Data Preparation -------------------- */

/**
 * @brief Prepare the aggregated packet for transmission.
 * 
 * This function prepares the aggregated packet by encoding the sensor data
 * 
 * @param data Pointer to the BLE packet data.
 * @param imu_batch Pointer to the IMU payload data.
 * @param num_imu_samples Number of IMU samples in the batch.
 * @param buffer Pointer to the buffer where the packet will be stored.
 * @param buffer_size Size of the buffer.
 */
void prepare_aggregated_packet(const ble_packet_t *data,
							   const imu_payload_t *imu_batch,
							   uint8_t num_imu_samples,
							   uint8_t *buffer, size_t buffer_size)
{
	if (buffer_size < AGGREGATED_SENSOR_DATA_PLAINTEXT_SIZE)
	{
		LOG_ERR("Buffer too small (%d < %d)", buffer_size, AGGREGATED_SENSOR_DATA_PLAINTEXT_SIZE);
		return;
	}

	if (num_imu_samples > MAX_IMU_SAMPLES_IN_PACKET)
	{
		LOG_WRN("num_imu_samples (%u) > MAX_IMU_SAMPLES_IN_PACKET (%d). Clamping.",
				num_imu_samples, MAX_IMU_SAMPLES_IN_PACKET);
		num_imu_samples = MAX_IMU_SAMPLES_IN_PACKET;
	}

	int offset = 0;

	// Temperature: Assume we only need -30 to 40 degree with 1 degree C precision
	int temp_C = round((float)data->temperature / 100.0f);
	temp_C = CLAMP(temp_C, -TEMPERATURE_LOW_LIMIT, TEMPERATURE_HIGH_LIMIT);
	uint8_t encoded_temp = (uint8_t)(temp_C + TEMPERATURE_LOW_LIMIT);
	buffer[offset++] = encoded_temp; // 1 byte

	// Pressure (Convert to uint16_t offset Pascals)
	uint16_t pressure_x10hpa = (uint16_t)data->pressure;
	uint16_t pressure_offset = 0;
	if (pressure_x10hpa >= PRESSURE_BASE_HPA_X10)
	{
		pressure_offset = pressure_x10hpa - PRESSURE_BASE_HPA_X10;
	}
	sys_put_le16(pressure_offset, &buffer[offset]);
	offset += 2; // 2 bytes

	/* Battery percentage */
	buffer[offset++] = data->battery; // 1 byte

	// Number of IMU samples in this batch
	buffer[offset++] = num_imu_samples; // 1 byte

	// IMU data batch (each imu_payload_t is 16 bytes: 6*int16_t + uint32_t timestamp)
	for (uint8_t i = 0; i < num_imu_samples; i++)
	{
		memcpy(&buffer[offset], imu_batch[i].imu_data, sizeof(imu_batch[i].imu_data));
		offset += sizeof(imu_batch[i].imu_data); // 12 bytes
		sys_put_le32(imu_batch[i].timestamp, &buffer[offset]);
		offset += sizeof(imu_batch[i].timestamp); // 4 bytes
	}

	// Fill remaining space if num_imu_samples < MAX_IMU_SAMPLES_IN_PACKET to maintain fixed size for encryption
	int remaining_imu_slots = MAX_IMU_SAMPLES_IN_PACKET - num_imu_samples;
	if (remaining_imu_slots > 0)
	{
		memset(&buffer[offset], 0, remaining_imu_slots * sizeof(imu_payload_t));
		offset += remaining_imu_slots * sizeof(imu_payload_t);
	}

	// Check total size
	if (offset != AGGREGATED_SENSOR_DATA_PLAINTEXT_SIZE)
	{
		LOG_WRN("Aggregated packet size mismatch: expected %d, got %d. Num_samples: %u",
				AGGREGATED_SENSOR_DATA_PLAINTEXT_SIZE, offset, num_imu_samples);
	}
}

/* ----------------------------------------------------------------------------- */

/**
 * @brief BMI270 handler thread function.
 *
 * This thread handles BMI270 interrupt events, reads IMU data, and queues
 * the data for further processing.
 *
 * @param unused1 Unused parameter.
 * @param unused2 Unused parameter.
 * @param unused3 Unused parameter.
 */
static void bmi270_handler_func(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	LOG_DBG("BMI270 handler thread started");

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

/**
 * @brief BMP390 handler thread function.
 *
 * This thread periodically reads environmental data from the BMP390 sensor
 * and queues the data for further processing.
 *
 * @param unused1 Unused parameter.
 * @param unused2 Unused parameter.
 * @param unused3 Unused parameter.
 */
static void bmp390_handler_func(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	LOG_DBG("BMP390 hanlder thread started (polling at %d ms interval)", BMP390_READ_INTERVAL);

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
			msg.payload.env.temperature = (uint16_t)(sensor_value_to_double(&temp) * 100);
			msg.payload.env.pressure = (uint32_t)(sensor_value_to_double(&press) * 100); // kPa x 100 = hPa x 10 gives 0.1 hPa resolution
			msg.payload.env.timestamp = k_uptime_get_32();

			if (k_msgq_put(&sensor_message_queue, &msg, K_NO_WAIT) != 0)
			{
				LOG_WRN("BMP390 queue full");
			}

			// LOG_DBG("Timestamp: %u, Temperature: %.2f, Pressure: %.1f",
			// 		msg.payload.env.timestamp,
			// 		(double)(msg.payload.env.temperature / 100),
			// 		(double)(msg.payload.env.pressure / 10.0));
		}
		else
		{
			LOG_ERR("Failed to read BMP390 (simulated): %d", ret);
		}

		k_msleep(BMP390_READ_INTERVAL);
	}
}

/**
 * @brief Settings handler for loading the nonce counter.
 *
 * This function is called by the settings subsystem to load the nonce counter
 * from persistent storage.
 *
 * @param name Name of the setting.
 * @param len Length of the setting value.
 * @param read_cb Callback function to read the setting value.
 * @param cb_arg Argument for the read callback.
 * @return 0 on success, negative error code on failure.
 */
static int settings_handle_nonce_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	LOG_DBG("Settings handler entered for key: %s (len: %d)", name ? name : "NULL", len);

	const char *next;
	int ret;

	if (settings_name_steq(name, "nonce_ctr", &next) && !next)
	{
		if (len != sizeof(nonce_counter))
		{
			LOG_ERR("Invalid size for nonce_ctr setting (%d != %d)", len, sizeof(nonce_counter));
			return -EINVAL;
		}

		ret = read_cb(cb_arg, &nonce_counter, sizeof(nonce_counter));
		if (ret > 0)
		{
			LOG_DBG("Loaded nonce_counter from NVS: %llu", nonce_counter);
			return 0;
		}
		LOG_ERR("Failed to read nonce_ctr setting: %d", ret);
		return ret;
	}
	return -ENOENT;
}

// Register the settings handler for the specific subtree
SETTINGS_STATIC_HANDLER_DEFINE(
	borus_state,
	BORUS_SETTINGS_PATH,
	NULL,
	settings_handle_nonce_set,
	NULL,
	NULL);

/**
 * @brief Encrypt sensor data using AES-CTR mode.
 *
 * This function encrypts a 20-byte sensor data block using AES-CTR mode and
 * appends an 8-byte nonce to the output buffer.
 *
 * @param plain Pointer to the plaintext data.
 * @param out Pointer to the output buffer.
 * @return 0 on success, negative error code on failure.
 */
static int encrypt_aggregated_sensor_block(const uint8_t *plain, size_t plain_len, uint8_t *out, size_t out_len)
{
	if (out_len < (1 + NONCE_LEN + plain_len))
	{
		LOG_ERR("Output buffer too small for encryption. Needed %zu, got %zu",
				(1 + NONCE_LEN + plain_len), out_len);
		return -ENOMEM;
	}

	memset(out, 0, out_len);

	if (g_aes_key_id == PSA_KEY_ID_NULL)
	{
		LOG_ERR("AES key handle is not valid");
		return -EPERM;
	}
	psa_status_t st;
	int ret = 0;

	// Build the 8 byte Nonce
	uint8_t nonce[NONCE_LEN];

	// Bytes 0-1: Company ID (nordic 0x0059) in little-endian
	static uint16_t cid = sys_cpu_to_le16(0x0059);
	memcpy(nonce, &cid, sizeof(cid));

	// Bytes 2-7: 48-bit packet counter, big-endian
	nonce_counter++;
	sys_put_be48(nonce_counter, &nonce[2]);

	uint8_t iv_buffer_for_api[PSA_BLOCK_CIPHER_BLOCK_LENGTH(PSA_KEY_TYPE_AES)]; // Should be 16
	memset(iv_buffer_for_api, 0, sizeof(iv_buffer_for_api));					// Zero pad
	memcpy(iv_buffer_for_api, nonce, NONCE_LEN);								// Copy the 8-byte nonce

	psa_cipher_operation_t op = PSA_CIPHER_OPERATION_INIT;
	size_t olen = 0;		   // For output length from PSA functions
	size_t ciphertext_len = 0; // Accumulate total ciphertext written

	st = psa_cipher_encrypt_setup(&op, g_aes_key_id, PSA_ALG_CTR);
	if (st != PSA_SUCCESS)
	{
		LOG_ERR("Failed to setup cipher encryption: %d", st);
		ret = -EFAULT; // Map PSA error
		goto cleanup;  // No operation to abort yet
	}

	// --- Use the 16-byte padded buffer for the API call ---
	st = psa_cipher_set_iv(&op, iv_buffer_for_api, sizeof(iv_buffer_for_api));
	if (st != PSA_SUCCESS)
	{
		LOG_ERR("Failed to set cipher IV: %d", st);
		ret = -EFAULT;
		goto cleanup_op; // Abort the operation
	}

	out[0] = SENSOR_ADV_PAYLOAD_TYPE;

	// --- Copy the actual 8-byte nonce to the output buffer ---
	memcpy(&out[1], nonce, NONCE_LEN);

	// Encrypt data, placing ciphertext *after* the nonce space in 'out'
	st = psa_cipher_update(&op, plain, plain_len,
						   &out[1 + NONCE_LEN],		// Output buffer starts after nonce + type
						   plain_len, // Max capacity for ciphertext
						   &olen);
	if (st != PSA_SUCCESS)
	{
		LOG_ERR("Failed to update cipher: %d", st);
		ret = -EFAULT;
		goto cleanup_op;
	}
	ciphertext_len += olen;

	// Finish the operation (usually produces no more output for CTR)
	if (ciphertext_len < plain_len)
	{
		st = psa_cipher_finish(&op,
							   out + 1 + NONCE_LEN + ciphertext_len,	 // Where to write if any
							   plain_len - ciphertext_len, // Remaining capacity
							   &olen);

		if (st != PSA_SUCCESS)
		{
			LOG_ERR("Failed to finish cipher op: %d", st);
			ret = -EFAULT;
			goto cleanup_op;
		}

		ciphertext_len += olen;
	}
	// Now 'out' contains [ 1 byte type | 8-byte nonce | 20-byte ciphertext ]

	// --- Verification ---
	if (ciphertext_len != plain_len)
	{
		LOG_ERR("Ciphertext length mismatch: expected %d, got %u",
				plain_len, (unsigned int)ciphertext_len);
		ret = -EIO; // Unexpected data size error
		// Nonce is copied, but ciphertext is incomplete/wrong. Return error.
		goto cleanup_op; // Ensure abort is called
	}

// Cleanup labels
cleanup_op:
	// Abort the operation if it was successfully set up
	psa_cipher_abort(&op); // Best effort cleanup
cleanup:
	// --- Removed key destruction ---
	return ret; // 0 on success, negative error code on failure
}

/**
 * @brief BLE logger thread function.
 *
 * This thread processes sensor data messages from the queue and either updates
 * BLE advertisements or logs the data to external flash storage.
 *
 * @param unused1 Unused parameter.
 * @param unused2 Unused parameter.
 * @param unused3 Unused parameter.
 */
static void ble_logger_func(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	LOG_DBG("BLE Logger thread started");

	int ret;
	sensor_message_t received_msg;						// Received from message queue
	ble_packet_t current_sensor_state = {0};			// Initialise
	current_sensor_state.timestamp = k_uptime_get_32(); // Set initial timestamp

	static imu_payload_t imu_batch_buffer[MAX_IMU_SAMPLES_IN_PACKET];
	static uint8_t imu_batch_count = 0;
	static uint32_t last_adv_payload_update_time = 0;

	uint32_t save_timer = k_uptime_get_32(); // Timer for saving nonce

	struct fs_file_t log_file;			  // File object for littlefs
	bool file_is_open = false;			  // Track if log file is currently open
	int log_write_count = 0;			  // Counter for periodic sync
	static bool log_storage_full = false; // Flag to indicate storage is full

	fs_file_t_init(&log_file); // Initialise file object structure

	while (1)
	{
		bool new_data_for_adv = false;

		/* Get each message from the message queue */
		ret = k_msgq_get(&sensor_message_queue, &received_msg, K_FOREVER);
		if (ret == 0)
		{
			// Update the local copy of the full sensor state
			switch (received_msg.type)
			{
			case SENSOR_MSG_TYPE_IMU:
				if (imu_batch_count < MAX_IMU_SAMPLES_IN_PACKET)
				{
					imu_batch_buffer[imu_batch_count++] = received_msg.payload.imu;
				}
				else
				{
					LOG_WRN("IMU batch buffer full, dropping new IMU samples");
				}
				break;
			case SENSOR_MSG_TYPE_ENVIRONMENT:
				current_sensor_state.temperature = received_msg.payload.env.temperature;
				current_sensor_state.pressure = received_msg.payload.env.pressure;
				// Use newest timestamp if relevant
				if (received_msg.payload.env.timestamp > current_sensor_state.timestamp)
				{
					current_sensor_state.timestamp = received_msg.payload.env.timestamp;
				}
				new_data_for_adv = true;
				break;
			case SENSOR_MSG_TYPE_BATTERY:
				current_sensor_state.battery = received_msg.payload.batt.battery;
				if (received_msg.payload.batt.timestamp > current_sensor_state.timestamp)
				{
					current_sensor_state.timestamp = received_msg.payload.batt.timestamp;
				}
				new_data_for_adv = true;
				break;
			default:
				break;
			}
		}
		else
		{
			LOG_ERR("Failed to get message from queue: %d", ret);
			k_sleep(K_SECONDS(1));
			continue;
		}

		// Check the current device state
		device_state_t active_state = atomic_get(&current_state);
		uint32_t current_time = k_uptime_get_32();

		// Perform actions based on state only if new data arrived
		if (active_state == STATE_HOME_ADVERTISING)
		{
			if (file_is_open)
			{
				LOG_DBG("Home mode: Closing log file.");
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

			bool send_adv = false;
			if (imu_batch_count > 0 && (imu_batch_count == MAX_IMU_SAMPLES_IN_PACKET || current_time - last_adv_payload_update_time >= ADV_PAYLOAD_UPDATE_INTERVAL_MS))
			{
				send_adv = true;
			}

			if (send_adv)
			{
				if (imu_batch_count > 0)
				{
					current_sensor_state.timestamp = imu_batch_buffer[imu_batch_count - 1].timestamp;
				}
				else
				{
					current_sensor_state.timestamp = current_time;
				}

				prepare_aggregated_packet(&current_sensor_state, imu_batch_buffer, imu_batch_count, manuf_plain_aggregated, sizeof(manuf_plain_aggregated));

				ret = encrypt_aggregated_sensor_block(manuf_plain_aggregated, AGGREGATED_SENSOR_DATA_PLAINTEXT_SIZE,
													  manuf_payload_sensor_aggregated, sizeof(manuf_payload_sensor_aggregated));

				if (ret != 0)
				{
					LOG_ERR("Encrypt failed - ADV not updated: %d", ret);
				}
				else
				{
					if (atomic_get(&adv_running_flag) && atomic_get(&current_adv_type) == SENSOR_ADV_PAYLOAD_TYPE)
					{
						ret = bt_le_ext_adv_set_data(g_adv_sensor_ext_handle, ad_sensor_agg, ARRAY_SIZE(ad_sensor_agg), NULL, 0);
						if (ret && ret != -EALREADY)
						{ // -EALREADY is fine, means it was already trying to send
							LOG_WRN("Failed to update aggregated ADV data: %d", ret);
						}
						else
						{
							LOG_DBG("Updated ADV with %u IMU samples.", imu_batch_count);
						}
					}
				}
				imu_batch_count = 0;
				last_adv_payload_update_time = current_time;
			}

			if (current_time - save_timer > NONCE_SAVE_INTERVAL)
			{
				ret = settings_save_one(BORUS_SETTINGS_PATH "/nonce_ctr", (const void *)&nonce_counter, sizeof(nonce_counter));
				if (ret == 0)
				{
					LOG_DBG("Saved nonce_counter to NVS: %llu", nonce_counter);
				}
				else
				{
					LOG_ERR("Failed to save nonce_counter to NVS: %d", ret);
				}
				save_timer = k_uptime_get_32();
			}
		}
		else if (active_state == STATE_AWAY_LOGGING)
		{
			if (imu_batch_count > 0)
			{
				imu_batch_count = 0;
				last_adv_payload_update_time = current_time;
			}

			// Only log IMU data in away mode
			if (ret == 0 && received_msg.type == SENSOR_MSG_TYPE_IMU)
			{
				// Open file if not already open and storage is not full
				if (!file_is_open && !log_storage_full)
				{
					LOG_DBG("Away mode: Opening log file %s", LOG_FILE_PATH);

					int ret = fs_open(&log_file, LOG_FILE_PATH, FS_O_APPEND | FS_O_CREATE | FS_O_WRITE);

					if (ret < 0)
					{
						LOG_ERR("Failed to open log file %s: %d", LOG_FILE_PATH, ret);
						k_sleep(K_SECONDS(5));
					}
					else
					{
						LOG_DBG("Log file opened successfully");
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
						if (written == -ENOSPC)
						{
							LOG_WRN("Log storage full");
							log_storage_full = true; // Set flag
						}
						fs_close(&log_file); // Close on error
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
						// LOG_DBG("Logged IMU data (%d bytes), write count: %d",
						// 		(int)written, log_write_count);

						if (log_write_count >= LOG_SYNC_THRESHOLD)
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
					if (current_time - last_full_log > NONCE_SAVE_INTERVAL)
					{
						LOG_WRN("Storage is full. No longer logging data.");
						last_full_log = current_time;
					}
				}
			}
		}
		else
		{
			// Close file if open (transitioned from AWAY)
			if (file_is_open)
			{
				LOG_DBG("State %d: Closing log file.", active_state);
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
			if (imu_batch_count > 0)
			{
				imu_batch_count = 0;
				last_adv_payload_update_time = current_time;
			}
		}
	}
	// Loop continues, blocking on k_msgq_get
}

/**
 * @brief Main entry point of the application.
 *
 * This function initializes peripherals, sensors, BLE, and threads, and
 * enters the main loop.
 *
 * @return 0 on success, negative error code on failure.
 */
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
	LOG_INF("Step 4.1: Enable battery voltage measurement");

	// Push initial battery pct to queue
	queue_initial_battery_level();

	struct fs_mount_t *mp = &lfs_mount_p;

	ret = fs_mount(mp);
	if (ret < 0)
	{
		LOG_WRN("LittleFS mount failed (%d). Attempting format...", ret);
	}
	else
	{
		LOG_DBG("LittleFS mounted on %s", mp->mnt_point);
	}
	// Log FS stats if mount succeeded
	if (ret == 0)
	{
		struct fs_statvfs stats;
		if (fs_statvfs(mp->mnt_point, &stats) == 0)
		{
			LOG_DBG("%s: bsize = %lu ; frsize = %lu ;"
					" blocks = %lu ; bfree = %lu",
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

	ret = create_sensor_ext_adv_set();
	if (ret)
	{
		LOG_ERR("Failed to create sensor extended adv set: %d", ret);
		return -1;
	}

	ret = setup_scan_filter();
	if (ret)
	{
		LOG_ERR("Failed to configure scan filter accept list: %d", ret);
		return -1;
	}

	ret = psa_crypto_init();
	if (ret != PSA_SUCCESS)
	{
		LOG_ERR("Failed to initialise crypto: %d", ret);
		return -1;
	}

	// --- Import the AES Key ONCE ---
	static const uint8_t aes_key[16] = {
		0x9F, 0x7B, 0x25, 0xA0, 0x68, 0x52, 0x33, 0x1C,
		0x10, 0x42, 0x5E, 0x71, 0x99, 0x84, 0xC7, 0xDD};

	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
	psa_set_key_lifetime(&attr, PSA_KEY_LIFETIME_VOLATILE); // Or persistent if needed
	psa_set_key_algorithm(&attr, PSA_ALG_CTR);
	psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attr, 128);

	// Store the key handle in the global variable
	ret = psa_import_key(&attr, aes_key, sizeof(aes_key), &g_aes_key_id);
	if (ret != PSA_SUCCESS)
	{
		LOG_ERR("Failed to import AES key: %d", ret);
		// Handle error - perhaps cannot continue securely
		return -1;
	}
	psa_reset_key_attributes(&attr); // Good practice
	LOG_INF("Step 7: AES Key Imported Successfully (ID: %u)", (unsigned int)g_aes_key_id);

	ret = settings_subsys_init();
	if (ret != 0)
	{
		LOG_ERR("Failed to initialise settings subsystem: %d", ret);
		return -1;
	}
	else
	{
		LOG_DBG("Setting subsystem initialised");
	}

	ret = settings_load_subtree("borus/state");
	if (ret == 0)
	{
		LOG_DBG("Settings loaded successfully");
	}
	else if (ret == -ENOENT)
	{
		LOG_DBG("No 'bours/state' settings found in NVS. Using default = 0");
	}
	else
	{
		LOG_ERR("Failed to load settings for 'borus/state': %d", ret);
	}

	LOG_INF("Step 8: Settings loaded, nonce = %llu", nonce_counter);

	// USB Device Subsystem
	ret = usb_enable(usb_dc_status_cb); // Register callback
	if (ret)
	{
		LOG_ERR("Failed to enable USB: %d", ret);
		initial_state = STATE_HOME_ADVERTISING; // If USB fails, we are not charging
	}
	else
	{
		LOG_INF("Step 9: USB callback registered");
	}

	// --- Initialise Timers ---
	k_timer_init(&battery_timer, (k_timer_expiry_t)battery_timer_expiry, NULL);
	k_timer_init(&sync_check_timer, sync_check_timer_expiry, NULL);
	k_timer_init(&scan_close_timer, scan_close_timer_expiry, NULL);

	// Initialise workqueue items
	k_work_init(&battery_timeout_work, battery_timeout_work_handler);
	k_work_init(&usb_connect_work, usb_connect_work_handler);
	k_work_init(&usb_disconnect_work, usb_disconnect_work_handler);
	k_work_init(&scan_found_ap_work, scan_found_ap_work_handler);
	k_work_init(&scan_open_work, scan_open_work_handler);
	k_work_init(&scan_close_work, scan_close_work_handler);
	k_work_init(&sync_check_work, sync_check_work_handler);					// Init new work
	k_work_init_delayable(&sync_adv_stop_work, sync_adv_stop_work_handler); // Init new delayable work
	k_work_init_delayable(&sync_scan_trigger_work, sync_scan_trigger_handler);

	k_timer_start(&battery_timer, BATTERY_READ_INTERVAL, BATTERY_READ_INTERVAL);

	LOG_INF("Step 10: Timers and WQ initialized and Battery timer started");

	/* Watchdog initialisation */
	if (!device_is_ready(wdt_dev))
	{
		LOG_ERR("WDT device not ready");
		return -1;
	}

	struct wdt_timeout_cfg wdt_cfg = {
		.window = {
			.min = 0,
			.max = WDT_TIMEOUT_MS,
		},
		.callback = NULL, // Reset immediately, no callback
		.flags = WDT_FLAG_RESET_SOC,
	};

	wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_cfg);
	if (wdt_channel_id < 0)
	{
		LOG_ERR("Failed to install WDT timeout configuration: %d", wdt_channel_id);
		return -1;
	}

	ret = wdt_setup(wdt_dev, 0);
	if (ret)
	{
		LOG_ERR("Failed to start WDT: %d", ret);
		return -1;
	}

	LOG_INF("Step 11: Watchdog started (timeout %d ms)", WDT_TIMEOUT_MS);

	k_timer_init(&wdt_feed_timer, (k_timer_expiry_t)watchdog_feed, NULL);
	k_timer_user_data_set(&wdt_feed_timer, (void *)wdt_dev);
	k_timer_start(&wdt_feed_timer, K_MSEC(WDT_FEED_INTERVAL_MS), K_MSEC(WDT_FEED_INTERVAL_MS));

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

	// --- Initial State ---
	// Start assuming HOME, scanner/USB callback will correct quickly if needed.
	LOG_DBG("Setting *initial state* determined state to: %d", initial_state);
	enter_state(initial_state);

	LOG_INF("Entering main loop (idle)");

	while (1)
	{
		// Main thread can sleep or handle very low priority tasks
		k_sleep(K_MINUTES(5));
	}

	return 0;
}
