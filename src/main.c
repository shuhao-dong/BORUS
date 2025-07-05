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
 * Version: v0.3.0
 *
 * Date: 16/04/2025
 *
 * New feature introduced since v0.1.0
 * 
 * 	- Added data encryption for BLE sensor data pakcet (type 0x00) with self-built Nonce
 * 	- Added time sync request packet (type 0x01) to achieve at-home check
 *  - Implemented extended advertisement for sensor data packet to allow high sample rate
 *  - Added zephyr SMF to manage state transition of the wearable
 * 
 * This is only for torus53 board and can be build with Nordic Thingy53
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
#include <zephyr/bluetooth/addr.h>
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
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/smf.h>
#include "feature/dsp_filters.h"
#include "feature/gait_analysis.h"
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/pm/device.h>
#include "ram_pwrdn.h"
#include <zephyr/storage/disk_access.h>
#include <zephyr/sys/poweroff.h>

LOG_MODULE_REGISTER(TORUS53, LOG_LEVEL_DBG);

/* -------------------- Setting subsystem for Encryption and Watchdog -------------------- */

#define BORUS_SETTINGS_SUBTREE	"borus/state"
#define BORUS_SETTINGS_KEY_WDT	"wdt_cnt"					// Path to save watchdog counter
#define BORUS_SETTINGS_KEY_NONCE "nonce_ctr"				// Path to save encryption nonce
#define BORUS_SETTINGS_PATH_WDT     BORUS_SETTINGS_SUBTREE "/" BORUS_SETTINGS_KEY_WDT
#define BORUS_SETTINGS_PATH_NONCE   BORUS_SETTINGS_SUBTREE "/" BORUS_SETTINGS_KEY_NONCE

#define NONCE_SAVE_INTERVAL	5 * 60 * 1000 					// Save nonce every 5 minutes
#define WDT_REBOOT_NUMBER_THRESHOLD	3						// Number of watchdog reboot before indicating error
#define WDT_TIMEOUT_MS 			10000						// Watchdog timeout
#define WDT_FEED_INTERVAL_MS 	1000						// Watchdog feed interval

static psa_key_id_t g_aes_key_id = PSA_KEY_ID_NULL; 		// Initialize the AES key ID
static uint64_t nonce_counter = 0;							// Unique nonce for each BLE message

static const struct device *wdt_dev = 
						DEVICE_DT_GET_ONE(nordic_nrf_wdt); 	// Get the WDT device
static int wdt_channel_id = -1;								// Initialize the WDT channel ID
static struct k_timer wdt_feed_timer;						// Timer for feeding the watchdog

static uint8_t wdt_reboot_count;							// Hold the number of reboot count

/**
 * @brief Set the watchdog reboot count and nonce counter
 * 
 * @param name The name of the setting to set
 * @param len The length of the value
 * @param read_cb Callback to read the value
 * 
 * @return 0 on success, negative error code on failure
 */
static int borus_state_set(const char *name, size_t len,
                           settings_read_cb read_cb, void *cb_arg)
{	
	ssize_t rc;

    if (strcmp(name, BORUS_SETTINGS_KEY_WDT) == 0 &&
        len == sizeof(wdt_reboot_count)) {
		rc = read_cb(cb_arg, &wdt_reboot_count, sizeof(wdt_reboot_count));
        return (rc < 0) ? (int)rc : 0; 
    }

    if (strcmp(name, BORUS_SETTINGS_KEY_NONCE) == 0 &&
        len == sizeof(nonce_counter)) {
		rc = read_cb(cb_arg, &nonce_counter, sizeof(nonce_counter));
        return (rc < 0) ? (int)rc : 0;
    }

    return -ENOENT;
}

/**
 * @brief Get the current watchdog reboot count and nonce counter
 * 
 * @param name The name of the setting to get
 * @param buf The buffer to store the value
 * @param buf_len The length of the buffer
 * 
 * @return The length of the value copied to the buffer, or an error code
 */
static int borus_state_get(const char *name, char *buf, int buf_len)
{
    if (strcmp(name, BORUS_SETTINGS_KEY_WDT) == 0) {
        if (buf_len < sizeof(wdt_reboot_count))
            return -ENOMEM;
        memcpy(buf, &wdt_reboot_count, sizeof(wdt_reboot_count));
        return sizeof(wdt_reboot_count);
    }

    if (strcmp(name, BORUS_SETTINGS_KEY_NONCE) == 0) {
        if (buf_len < sizeof(nonce_counter))
            return -ENOMEM;
        memcpy(buf, &nonce_counter, sizeof(nonce_counter));
        return sizeof(nonce_counter);
    }

    return -ENOENT;
}

/* Register settings */
SETTINGS_STATIC_HANDLER_DEFINE(borus_state, BORUS_SETTINGS_SUBTREE, borus_state_get, borus_state_set, NULL, NULL);

/* -------------------- Thread Configurations -------------------- */

/* Stack sizes */ 
#define BMI270_HANDLER_STACKSIZE 		1024
#define BMP390_HANDLER_STACKSIZE 		1024
#define BLE_LOGGER_THREAD_STACKSIZE		1536

/* Priorities (Lower number = higher priority) */ 
#define BMI270_HANDLER_PRIORITY 		5 	// Highest sensor priority due to higher sample rate
#define BMP390_HANDLER_PRIORITY 		6 	// Medium sensor priority due to lower sample rate
#define BLE_THREAD_PRIORITY 			7	// Lower priority tasks for BLE and logging

/* Thread Stacks */ 
K_THREAD_STACK_DEFINE(bmi270_handler_stack_area, BMI270_HANDLER_STACKSIZE);
K_THREAD_STACK_DEFINE(bmp390_handler_stack_area, BMP390_HANDLER_STACKSIZE);
K_THREAD_STACK_DEFINE(ble_logger_stack_area, BLE_LOGGER_THREAD_STACKSIZE);

/* Thread Control Blocks */ 
static struct k_thread bmi270_handler_thread_data;
static struct k_thread bmp390_handler_thread_data;
static struct k_thread ble_logger_thread_data;

/* -------------------- State Machine -------------------- */

static inline const struct smf_state * smf_current_state_get(struct smf_ctx *ctx)
{
    return ctx->current;          // the SMF core stores the pointer here
}

/* Define the state structure */ 
typedef enum
{
	STATE_INIT,				// Initial state before normal operation
	STATE_HOME_ADVERTISING, // At home, advertise sensor data
	STATE_AWAY_LOGGING,		// Away, log sensor data
	STATE_CHARGING,			// USB connected
	STATE_FAULT,			// System fault 
} device_state_t;

/* Define application-specific flags */ 
enum app_flags {
	FLAG_ADV_RUNNING,
	FLAG_SCAN_ACTIVE,
};

/* Define application SMF context */
struct app_ctx {
	struct smf_ctx smf;					// SMF context
	atomic_t flags;						// Flag for adv and scan state
	uint8_t missed_sync_responses;		// Number of missed heartbeat check
	uint32_t sync_check_interval_ms;	// Heartbeat check interval
	atomic_t current_adv_type;			// Type 0: sensor, Type 1: sync
};
static struct app_ctx app;

/* Forward declarations of functions used in SMF */
static void stop_all_advertising(struct app_ctx *ctx);
static void start_sensor_advertising_ext(struct app_ctx *ctx);
static void set_imu_rate(bool high_rate);

/* Forward declarations of state functions and objects */ 
static void state_init_entry(void *o);
static void state_home_entry(void *o);
static void state_home_exit(void *o);
static void state_away_entry(void *o);
static void state_away_exit(void *o);
static void state_charging_entry(void *o);
static void state_charging_exit(void *o);
static void state_fault_entry(void *o);

/* Define the state table */ 
static const struct smf_state states[] = {
    [STATE_INIT]           	 = SMF_CREATE_STATE(state_init_entry, NULL, NULL, NULL, NULL),
    [STATE_HOME_ADVERTISING] = SMF_CREATE_STATE(state_home_entry, NULL, state_home_exit, NULL, NULL),
    [STATE_AWAY_LOGGING]   	 = SMF_CREATE_STATE(state_away_entry, NULL, state_away_exit, NULL, NULL),
    [STATE_CHARGING]       	 = SMF_CREATE_STATE(state_charging_entry, NULL, state_charging_exit, NULL, NULL),
	[STATE_FAULT]			 = SMF_CREATE_STATE(state_fault_entry, NULL, NULL, NULL, NULL), 
};

/* Track if ble and usb stack is enabled */
static volatile bool ble_enabled = false;
static volatile bool usb_enabled = false; 

/* Workqueue declarations */
static struct k_work battery_timeout_work; 				// Work item for battery timeout -> Periodic voltage reading
static struct k_work usb_connect_work;	   				// Work item for USB connect -> CHARGING state
static struct k_work usb_disconnect_work;  				// Work item for USB disconnect -> HOME state (or chosen default)
static struct k_work scan_found_ap_work;   				// Work item for Scan Found AP -> HOME state
static struct k_work scan_open_work;					// Work item for starting to scan	
static struct k_work scan_close_work;					// Work item for stoppint to scan
static struct k_work sync_check_work;					// Work to perform heartbeat check 
static struct k_work_delayable sync_adv_stop_work;		// Work to stop type 0x01 packet
static struct k_work_delayable sync_scan_trigger_work;	// Work to start heartbeat check scann

/* -------------------- Message Queue for Sensor Data -------------------- */

/* Define message types */ 
typedef enum
{
	SENSOR_MSG_TYPE_IMU,
	SENSOR_MSG_TYPE_ENVIRONMENT,
	SENSOR_MSG_TYPE_MONITOR,
} sensor_msg_type_t;

/* Define payload structures: imu, environment, battery */ 
typedef struct
{
	int16_t imu_data[6];	// IMU data: x, y, z for acc and gyr
	uint32_t timestamp;		// SoC timestamp
} imu_payload_t;

/* Environmental sensor data structure */ 
typedef struct
{
	uint16_t temperature;	// Environmental temperature in degree C 
	uint32_t pressure;		// Environmental pressure in hPa
	uint32_t timestamp;		// SoC timestamp
} environment_payload_t;

/* Battery voltage data structure */ 
typedef struct
{
	uint8_t battery;		// Battery voltage in mV
	uint32_t timestamp;		// SoC timestamp
	int8_t soc_temp; 		// SoC temperature in degree C
	uint8_t npm_status;		// PMIC error status 
} monitor_payload_t;

/* Unified message structure for message queue */ 
typedef struct
{
	sensor_msg_type_t type;
	union
	{
		imu_payload_t imu;
		environment_payload_t env;
		monitor_payload_t monitor;
	} payload;
} sensor_message_t;

/*  Define the message queue */
K_MSGQ_DEFINE(sensor_message_queue, sizeof(sensor_message_t), 16, 4);

/* -------------------- Semaphores for Interrupts -------------------- */

K_SEM_DEFINE(bmi270_isr_sem, 0, 20);
// K_SEM_DEFINE(bmp390_isr_sem, 0, 1);

/* -------------------- Timers -------------------- */

static struct k_timer battery_timer;	// For periodic battery reading
static struct k_timer sync_check_timer; // For scan open
static struct k_timer scan_close_timer; // For scan close

/* -------------------- Configuration Constants -------------------- */

#define BMP390_READ_INTERVAL 					60000 			// Read environment at 1 Hz
#define BATTERY_READ_INTERVAL 					K_MINUTES(15)	// Every 15 minute read one battery voltage
#define SENSOR_ADV_PAYLOAD_TYPE 				0x00			// Custom packet type 0x00: Sensor data
#define SYNC_REQ_ADV_PAYLOAD_TYPE 				0x01			// Custom packet type 0x01: Time sync data
#define ADV_TYPE_NONE							2				// No advertising active
#define MAX_IMU_SAMPLES_IN_PACKET 				14				// (251 - 9)/16: 251 bytes max - 9 bytes (env, nonce, etc.), divided by IMU (16 bytes)
#define ADV_PAYLOAD_UPDATE_INTERVAL_MS 			140				// If sample at 100Hz (10ms) * 14 samples = 140 ms 
#define AGGREGATED_SENSOR_DATA_PLAINTEXT_SIZE	231				// 1+2+1+1+1+1+16*14: see prepare packet
#define NONCE_LEN 								8				// Size of the encryption nonce
#define AGGREGATED_ENC_ADV_PAYLOAD_LEN 			(1 + NONCE_LEN + AGGREGATED_SENSOR_DATA_PLAINTEXT_SIZE)	// Custom type + Nonce + Payload
#define SYNC_REQ_PAYLOAD_LEN 					(1 + 2)			// Custom type + Time
#define PRESSURE_BASE_HPA_X10 					8500 			// Base offset in hPa x 10
#define TEMPERATURE_LOW_LIMIT 					30   			// -30 degree as the lowest temperature of interest
#define TEMPERATURE_HIGH_LIMIT 					40  			// +40 degree as the highest temperature of interest
#define BATTERY_MV_TO_8BIT_SCALE_FACTOR 		20

/* -------------------- File system and MSC -------------------- */

const struct device *const qspi_dev = DEVICE_DT_GET(DT_INST(0, nordic_qspi_nor)); 

#define LOG_FILE_PATH "/lfs1/imu_log.bin" 					// File path for the log file
#define LFS_MOUNT_POINT "/lfs1"			  					// Mount point for the file system

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
USBD_DEVICE_DEFINE(my_usb, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), 0x2fe3, 0x0008);

USBD_DESC_LANG_DEFINE(my_lang);
USBD_DESC_MANUFACTURER_DEFINE(my_mfr, "TORUS53");
USBD_DESC_PRODUCT_DEFINE(my_prod, "BORUS");
USBD_DESC_SERIAL_NUMBER_DEFINE(my_sn);

USBD_DESC_CONFIG_DEFINE(cfg_desc, "MSC Configuration");
static const uint8_t attributes = USB_SCD_SELF_POWERED;
USBD_CONFIGURATION_DEFINE(my_cfg, attributes, 125, &cfg_desc);
#endif // CONFIG_USB_DEVICE_STACK

USBD_DEFINE_MSC_LUN(NOR, "NOR", "Zephyr", "BORUS", "0.00");	// Define the USB MSC LUN

struct fs_file_t log_file; // File object for littlefs
bool file_is_open = false; // Track if log file is currently open
int log_write_count = 0;   // Counter for periodic sync
static bool log_storage_full = false; // Flag to indicate storage is full

/* -------------------- BLE Configurations -------------------- */

#define SYNC_CHECK_INTERVAL_BASE_MS 	1 * 60 * 1000	// Period to perform an at-home check 
#define SYNC_REQ_ADV_BURST_DURATION 	K_MSEC(250) 	// Duration to send type 0x01 advertisment
#define X_ANNOUNCED_S 					3				// Tells the scanner I am going to scan in 3 seconds
#define MISSES_BEFORE_AWAY 				1				// If miss 3 HB check from the RPi, consider AWAY
#define AWAY_BACKOFF_MAX_INTERVAL_MS	2 * 60 * 1000	// Linearly increase the scan interval while AWAY
#define EARLY_MARGIN_MS 				500				// Guard time for scanning, plus the other half is the total scanning time
#define LATE_MARGIN_MS 					500				// Guard time for scanning
#define SYNC_SCAN_WINDOW_MS 			(EARLY_MARGIN_MS + LATE_MARGIN_MS)	// Total scanning time
#define EXT_ADV_INTERVAL_MIN			BT_GAP_ADV_FAST_INT_MIN_2			// Min advertise interval for sensor extended advertisement 
#define EXT_ADV_INTERVAL_MAX			192 			// Max advertise interval for sensor extended advertisement: 120ms / 0.625 = 192
#define SYNC_ADV_INTERVAL_MIN			160				// Min advertise interval for time sync request advertisement: 100ms / 0.625 = 160
#define SYNC_ADV_INTERVAL_MAX			164				// Max advertise interval for time sync request advertisement

/* Define BLE packet structure - Not used in real BLE packet */ 
typedef struct
{
	int16_t temperature;
	uint32_t pressure;
	int16_t imu_data[6];
	uint32_t timestamp;
	uint8_t battery;
	int8_t soctemp; 
	uint8_t npm_status; 
} ble_packet_t;

/* Buffer for dynamic manufacturer data in advertisement */ 
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

/* Legacy BLE advertisement parameters */ 
static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	BT_LE_ADV_OPT_USE_IDENTITY, 	// Use identity MAC for advertisement to allow self-configured address
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
static const char *target_ap_addrs[] __attribute__((used)) = {
	"C0:54:52:53:00:00",	// Random static address of the RPi's nrf controller
	"C0:54:52:53:00:01",	// Random static address of the RPi's nrf controller
	"C0:54:52:53:00:02",	// Random static address of the RPi's nrf controller
}; 

// Define wearable address
#define FACTORY_DATA_ADDR	PM_FACTORY_DATA_ADDRESS

static char ble_addr_str[18];

bool factory_data_get_ble_addr(char *out)
{
	memcpy(out, (void *)FACTORY_DATA_ADDR, 17);
	out[17] = '\0'; 
	return true;
}

/**
 * @brief Set a custom static address for the wearable device.
 * 
 * @param addr_string The static address in string format.
 * @return int 0 on success, negative error code on failure.
 */
static int set_custom_static_addr(const char *addr_string)
{
	bt_addr_le_t addr;

	/* Turn text into bytes */
	bt_addr_from_str(addr_string, &addr.a); 

	/* Mark it as random static address */
	addr.type = BT_ADDR_LE_RANDOM;
	BT_ADDR_SET_STATIC(&addr.a);

	/* Create a new identity that uses this address */
	int id = bt_id_create(&addr, NULL);

	return (id < 0) ? id : 0; 
}

/* -------------------- IMU Configurations -------------------- */

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

// 1. LEDs 
static const struct gpio_dt_spec leds[3] = {
	GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios), // Red LED
	GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios), // Green LED
	GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios), // Blue LED
};

// 2. NPM1100 
static const struct gpio_dt_spec npm_status[2] = {
	GPIO_DT_SPEC_GET(DT_ALIAS(npm1100chg), gpios),	// PMIC CHG status pin
	GPIO_DT_SPEC_GET(DT_ALIAS(npm1100err), gpios), 	// PMIC ERR status pin 
};

static volatile bool charging = false; 
static volatile int usb_status = -1; 
static void charging_status_handler(struct k_work *w); 
K_WORK_DELAYABLE_DEFINE(charging_status_work, charging_status_handler);

static void charging_status_handler(struct k_work *w)
{
	gpio_pin_toggle_dt(&leds[2]);
	if (charging)
	{
		k_work_schedule(&charging_status_work, K_MSEC(1000)); 
	}
}

// NPM1100 CHG Interrupt pin
static struct gpio_callback npm1100_chg_interrupts_cb_data; 

// BMI270 Interrupt Pin
static const struct gpio_dt_spec bmi270_interrupts = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(bmi270), irq_gpios, {0});
static struct gpio_callback bmi270_interrupts_cb_data;

/* -------------------- Sensor Device Handles -------------------- */

// BMI270
BMI270_Context bmi270_ctx;
#define SPIOP SPI_WORD_SET(8) | SPI_TRANSFER_MSB
static const struct spi_dt_spec bmi270_spi = SPI_DT_SPEC_GET(DT_NODELABEL(bmi270), SPIOP, 0);

// BMP390 (currently BME688 but can replace later)
const struct device *const bme688_dev = DEVICE_DT_GET_ONE(bosch_bme680);

/* -------------------- LittleFS Mount Configuration -------------------- */

#define LOG_SYNC_THRESHOLD 7500 // Record to log before fs_sync, this tolerates 5-min data missing

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);
static struct fs_mount_t lfs_mount_p = {
	.type = FS_LITTLEFS,
	.fs_data = &storage,
	.storage_dev = (void *)PM_LITTLEFS_STORAGE_ID,
	.mnt_point = LFS_MOUNT_POINT,
};

/* -------------------- SoC Temperature Measurement -------------------- */

int temperature_measure(void)
{
	int err = 0;
	struct net_buf *buf, *rsp = NULL;
	struct bt_hci_rp_vs_read_chip_temp *cmd_params;
	struct bt_hci_rp_vs_read_chip_temp *rsp_params;

	buf = bt_hci_cmd_create(BT_HCI_OP_VS_READ_CHIP_TEMP, sizeof(*cmd_params));
	if (!buf) {
		LOG_ERR("Could not allocate command buffer");
		return -ENOMEM;
	}

	cmd_params = net_buf_add(buf, sizeof(*cmd_params));

	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_READ_CHIP_TEMP, buf, &rsp);
	if (err) {
		LOG_ERR("bt_hci_cmd_send_sync failed (err: %d)",err);
		return err;
	}

	rsp_params = (void *) rsp->data;
	net_buf_unref(rsp);

	return rsp_params->temps;
}

/* -------------------- State Management Functions -------------------- */

static void stop_all_timers_and_scans(struct app_ctx *ctx)
{
    stop_all_advertising(ctx); // This function will now use the context
    k_timer_stop(&sync_check_timer);
    k_timer_stop(&scan_close_timer);
    (void)k_work_cancel_delayable(&sync_adv_stop_work);
    (void)k_work_cancel_delayable(&sync_scan_trigger_work);
    if (atomic_test_and_clear_bit(&ctx->flags, FLAG_SCAN_ACTIVE)) {
        bt_le_scan_stop();
    }
}

static void state_fault_entry(void *o)
{
	struct app_ctx *ctx = o;

	LOG_ERR("FAULT: watchdog rebooted %u times - device halted", wdt_reboot_count);

	if(usb_enabled) usb_disable();
	if(ble_enabled) bt_disable(); 

	/* 1. Indicate error state by red LED */
	gpio_pin_set_dt(&leds[0], 1);
	k_sleep(K_SECONDS(3));
	gpio_pin_set_dt(&leds[0], 0); 

	/* 2. stop everything dangerous or power-hungry */
	stop_all_timers_and_scans(ctx); 

	/* 3. stop feeding the watchdog so the LED stays on */
	k_timer_stop(&wdt_feed_timer);
	
	sys_poweroff(); 
}

static void state_init_entry(void *o)
{
    struct app_ctx *ctx = o; // Cast the generic object pointer to our context type
    LOG_INF("SMF: Entering STATE_INIT");
    stop_all_timers_and_scans(ctx);
}

static void state_home_entry(void *o)
{
    struct app_ctx *ctx = o;
    LOG_INF("SMF: Entering STATE_HOME_ADVERTISING");
    set_imu_rate(true);
    ctx->missed_sync_responses = 0;
    ctx->sync_check_interval_ms = SYNC_CHECK_INTERVAL_BASE_MS;
    start_sensor_advertising_ext(ctx);
    k_timer_start(&sync_check_timer, K_MSEC(ctx->sync_check_interval_ms), K_MSEC(ctx->sync_check_interval_ms));
}

static void state_home_exit(void *o)
{
    struct app_ctx *ctx = o;
    LOG_INF("SMF: Exiting STATE_HOME_ADVERTISING");
    stop_all_timers_and_scans(ctx);
}

static void state_away_entry(void *o)
{
    struct app_ctx *ctx = o;
    LOG_INF("SMF: Entering STATE_AWAY_LOGGING");
    set_imu_rate(false);
    stop_all_advertising(ctx);
    uint32_t jitter = sys_rand32_get() % (SYNC_CHECK_INTERVAL_BASE_MS / 4);
    k_timer_start(&sync_check_timer, K_MSEC(jitter), K_MSEC(ctx->sync_check_interval_ms));
}

static void state_away_exit(void *o)
{
    struct app_ctx *ctx = o;
    LOG_INF("SMF: Exiting STATE_AWAY_LOGGING");
    stop_all_timers_and_scans(ctx);
}

static void state_charging_entry(void *o)
{
    struct app_ctx *ctx = o;
    LOG_INF("SMF: Entering STATE_CHARGING");
    stop_all_timers_and_scans(ctx);
}

static void state_charging_exit(void *o)
{
    ARG_UNUSED(o);
    LOG_INF("SMF: Exiting STATE_CHARGING");
}

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

	for (size_t i = 0; i < ARRAY_SIZE(target_ap_addrs); i++)
	{
		ret = bt_addr_le_from_str(target_ap_addrs[i], "random", &addr_le);
		if (ret)
		{
			LOG_ERR("Bad address %s: %d", target_ap_addrs[i], ret);
			return ret;
		}

		ret = bt_le_filter_accept_list_add(&addr_le);
		if (ret)
		{
			LOG_ERR("Failed to add address %s to accept list: %d", target_ap_addrs[i], ret);
			return ret;
		}
	}
	return 0;
}

/**
 * @brief Stop BLE advertising when enter AWAY state
 *
 * This function stops BLE advertising. It will not stop if already stopped.
 */
static void stop_all_advertising(struct app_ctx *ctx)
{
	if (!atomic_test_and_clear_bit(&ctx->flags, FLAG_ADV_RUNNING))
	{
		return;
	}
	
	if (atomic_get(&ctx->current_adv_type) == SYNC_REQ_ADV_PAYLOAD_TYPE)
	{
		bt_le_adv_stop();
	} 
	else if (atomic_get(&ctx->current_adv_type) == SENSOR_ADV_PAYLOAD_TYPE && g_adv_sensor_ext_handle)
	{
		bt_le_ext_adv_stop(g_adv_sensor_ext_handle);
	}
	atomic_set(&ctx->current_adv_type, ADV_TYPE_NONE);
}

/**
 * @brief Start BLE advertising when enter HOME state
 *
 * This function starts BLE advertising with the specified parameters and data.
 * It will not start if already running.
 */
static void start_sensor_advertising_ext(struct app_ctx *ctx)
{
	int ret;

	// Ensure legacy (sync_req) advertising is stopped
	if (atomic_test_bit(&ctx->flags, FLAG_ADV_RUNNING)) { // Only try to stop if something is flagged as running
        LOG_DBG("start_sensor_advertising_ext: Stopping existing advertising first.");
        stop_all_advertising(ctx);
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

	// SENSOR_ADV_PAYLOAD_TYPE is the first byte *inside* manuf_payload_sensor
	ret = bt_le_ext_adv_start(g_adv_sensor_ext_handle, BT_LE_EXT_ADV_START_DEFAULT);

	if (ret && ret != -EALREADY)
	{
		LOG_ERR("Failed to start sensor extended advertisement: %d", ret);
	}
	else
	{
		atomic_set_bit(&ctx->flags, FLAG_ADV_RUNNING); 
		atomic_set(&ctx->current_adv_type, SENSOR_ADV_PAYLOAD_TYPE); 
	}
}

/**
 * @brief Start away advertisement burst.
 *
 * This function starts a BLE advertisement burst for the AWAY state.
 * It will not start if already running.
 */
static void start_sync_request_advertising(struct app_ctx *ctx)
{
	int ret;

	// Ensure other type is stopped first
	if (atomic_test_bit(&ctx->flags, FLAG_ADV_RUNNING)) { 
        stop_all_advertising(ctx);
        k_msleep(20); 
    }

	ret = bt_le_adv_start(adv_param, ad_sync_req, ARRAY_SIZE(ad_sync_req), NULL, 0);
	if (ret && ret != -EALREADY)
	{
		LOG_ERR("Failed to start AWAY Scan Announce advertisement: %d", ret);
	}
	else if (ret == -EALREADY)
	{
		LOG_WRN("AWAY ADV already started");
	}
	else
	{
		atomic_set_bit(&ctx->flags, FLAG_ADV_RUNNING); 
		atomic_set(&ctx->current_adv_type, SYNC_REQ_ADV_PAYLOAD_TYPE);
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
	struct app_ctx *ctx = &app;
	if (atomic_test_bit(&ctx->flags, FLAG_ADV_RUNNING) && atomic_get(&ctx->current_adv_type) == SYNC_REQ_ADV_PAYLOAD_TYPE)
	{
		stop_all_advertising(ctx);
		if (smf_current_state_get(SMF_CTX(ctx)) == &states[STATE_HOME_ADVERTISING])
		{
			start_sensor_advertising_ext(ctx);
		}
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
	const struct smf_state *current = smf_current_state_get(SMF_CTX(&app));
	if (current == &states[STATE_AWAY_LOGGING] || current == &states[STATE_HOME_ADVERTISING])
	{
		k_work_submit(&scan_open_work);
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
	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str)); // Still useful for logging
	LOG_INF("Heartbeat from %s, RSSI=%d", addr_str, rssi);

	if (smf_current_state_get(SMF_CTX(&app)) == &states[STATE_CHARGING])
	{
		/* ignore all packets while charging */
		return;
	}
	k_work_submit(&scan_found_ap_work); 
}

/**
 * @brief Queue the initial battery level to the message queue.
 *
 * This function reads the initial battery voltage and queues it to the
 * message queue for processing.
 */
static void queue_initial_monitoring_info(void)
{
	int batt_mV = battery_sample();
	if (batt_mV < 0)
	{
		LOG_ERR("Initial battery read failed: %d", batt_mV);
		return;
	}

	int soc_temp = temperature_measure();
	uint8_t npm_err_status = gpio_pin_get_dt(&npm_status[1]); // Read NPM1100 ERR status pin
	uint8_t batt_8bit = (uint8_t)(batt_mV / BATTERY_MV_TO_8BIT_SCALE_FACTOR); 

	sensor_message_t msg = {.type = SENSOR_MSG_TYPE_MONITOR};
	msg.payload.monitor.battery = batt_8bit;
	msg.payload.monitor.timestamp = k_uptime_get_32();
	msg.payload.monitor.soc_temp = soc_temp; 
	msg.payload.monitor.npm_status = npm_err_status; // Store NPM1100 ERR status

	if (k_msgq_put(&sensor_message_queue, &msg, K_NO_WAIT) != 0)
	{
		LOG_WRN("Initial battery queue full - value dropped");
	}
	else
	{
		LOG_INF("Queued initial monitor info, battery @ %d mV, SoC @ %d degreeC, npm1100 @ %d", 
			batt_mV, soc_temp, npm_err_status);
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
	struct app_ctx *ctx = &app;
	const struct smf_state *current = smf_current_state_get(SMF_CTX(ctx));

	if (current != &states[STATE_AWAY_LOGGING] && current != &states[STATE_HOME_ADVERTISING])
	{
		return;
	}

	manuf_payload_sync_req[0] = SYNC_REQ_ADV_PAYLOAD_TYPE;
	sys_put_le16(X_ANNOUNCED_S, &manuf_payload_sync_req[1]);

	start_sync_request_advertising(ctx);

	if (atomic_test_bit(&ctx->flags, FLAG_ADV_RUNNING) && atomic_get(&ctx->current_adv_type) == SYNC_REQ_ADV_PAYLOAD_TYPE)
	{
		k_work_schedule(&sync_adv_stop_work, SYNC_REQ_ADV_BURST_DURATION);
		int32_t scan_trigger_delay_ms = (X_ANNOUNCED_S * 1000) - EARLY_MARGIN_MS;
		if (scan_trigger_delay_ms < 0)
		{
			scan_trigger_delay_ms = 0;
		}
		k_work_schedule(&sync_scan_trigger_work, K_MSEC(scan_trigger_delay_ms));
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
	struct app_ctx *ctx = &app;
	stop_all_advertising(ctx);

	int err = bt_le_scan_start(&scan_param, scan_cb);
	if (err && err != -EALREADY)
	{
		LOG_ERR("bt_le_scan_start failed (%d)", err);
		return;
	}
	atomic_set_bit(&ctx->flags, FLAG_SCAN_ACTIVE);
	k_timer_start(&scan_close_timer, K_MSEC(SYNC_SCAN_WINDOW_MS), K_NO_WAIT);
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
	struct app_ctx *ctx = &app;

	if (atomic_test_and_clear_bit(&ctx->flags, FLAG_SCAN_ACTIVE))
	{
		bt_le_scan_stop();
	}
	LOG_DBG("Scan window finished (AP Response MISSING).");

	const struct smf_state *current = smf_current_state_get(SMF_CTX(ctx));

	if (current == &states[STATE_HOME_ADVERTISING])
	{
		ctx->missed_sync_responses++;
		if (ctx->missed_sync_responses >= MISSES_BEFORE_AWAY)
		{
			smf_set_state(SMF_CTX(ctx), &states[STATE_AWAY_LOGGING]);
		}
		else
		{
			start_sensor_advertising_ext(ctx);
		}
	}
	else if (current == &states[STATE_AWAY_LOGGING])
	{
		ctx->sync_check_interval_ms = MIN(ctx->sync_check_interval_ms * 2, AWAY_BACKOFF_MAX_INTERVAL_MS);
		ctx->sync_check_interval_ms += sys_rand32_get() % (SYNC_CHECK_INTERVAL_BASE_MS / 2);
		ctx->sync_check_interval_ms = MIN(ctx->sync_check_interval_ms, AWAY_BACKOFF_MAX_INTERVAL_MS);
		k_timer_start(&sync_check_timer, K_MSEC(ctx->sync_check_interval_ms), K_MSEC(ctx->sync_check_interval_ms));
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
	int soc_temp = temperature_measure(); 
	uint8_t npm_err_status = gpio_pin_get_dt(&npm_status[1]); // Read NPM1100 ERR status pin

	if (batt_mV >= 0)
	{ 	// Check for valid reading
		uint8_t batt_8bit = (uint8_t)(batt_mV / BATTERY_MV_TO_8BIT_SCALE_FACTOR); 
		sensor_message_t msg = {.type = SENSOR_MSG_TYPE_MONITOR};
		msg.payload.monitor.battery = batt_8bit;
		msg.payload.monitor.timestamp = k_uptime_get_32();
		msg.payload.monitor.soc_temp = soc_temp; 
		msg.payload.monitor.npm_status = npm_err_status; // Store NPM1100 ERR status

		if (k_msgq_put(&sensor_message_queue, &msg, K_NO_WAIT) != 0)
		{
			LOG_WRN("Battery queue full");
		}
		LOG_INF("Battery level: %d mV", batt_mV);
	}
	else
	{
		LOG_ERR("Failed to read battery voltage: %d", batt_mV);
	}
}

/**
 * @brief Delete previous extracted binary file
 * 
 * This function will attempt to delete the binary file specified by @ref LOG_FILE_PATH if exists
 *  
 */
static void delete_old_log(void)
{
	struct fs_dirent st;
	int ret = fs_stat(LOG_FILE_PATH, &st);

	if (!ret && st.type == FS_DIR_ENTRY_FILE)
	{
		LOG_INF("Deleting old log file, size=%u", (unsigned)st.size);

		ret = fs_unlink(LOG_FILE_PATH);
		if (ret)
		{
			LOG_ERR("Failed to erase imu_log.bin: %d", ret);
		}
		else
		{
			fs_sync(&log_file); 
		}
	}
}

/**
 * @brief Clean up littlefs file system
 * 
 * This function will attempt to sync, close the file, and finally unmount the file system
 */
static void stop_logging_and_unmount(void)
{
    if (file_is_open) {
        fs_sync(&log_file);
        fs_close(&log_file);
        file_is_open = false;
        log_write_count = 0;
    }
    fs_unmount(&lfs_mount_p);        /* detach LittleFS pages from cache */
}

/**
 * @brief Resume littlefs file system
 * 
 * This function will attempt to delete any existing binary file and resume file system
 */
static void remount_and_resume_logging(void)
{
    fs_mount(&lfs_mount_p);          /* make LittleFS visible again     */
	delete_old_log(); 
    log_storage_full = false;        /* optional: re-probe free space    */
}

/**
 * @brief Handle USB connection events.
 *
 * This function is called when the USB is connected. It transitions the device
 * to the CHARGING state. It will unmount the file system for host copy operation.
 *
 * @param work Pointer to the work structure.
 */
static void usb_connect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	stop_logging_and_unmount();
	smf_set_state(SMF_CTX(&app), &states[STATE_CHARGING]); 
}

/**
 * @brief Handle USB disconnection events.
 *
 * This function is called when the USB is disconnected. It transitions the device
 * to the HOME state. It will delete existing file and resume file system. 
 *
 * @param work Pointer to the work structure.
 */
static void usb_disconnect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	remount_and_resume_logging();
	smf_set_state(SMF_CTX(&app), &states[STATE_HOME_ADVERTISING]); 
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
	struct app_ctx *ctx = &app;

	if (smf_current_state_get(SMF_CTX(ctx)) == &states[STATE_CHARGING])
	{
		return;
	}

	/* 1.  Cancel the scan-close timer and stop the scan */
	if (atomic_test_and_clear_bit(&ctx->flags, FLAG_SCAN_ACTIVE))
	{
		bt_le_scan_stop();
	}
	k_timer_stop(&scan_close_timer); /* <- new */
	ctx->missed_sync_responses = 0;	 /* <- new */

	if (smf_current_state_get(SMF_CTX(ctx)) != &states[STATE_HOME_ADVERTISING])
	{
		smf_set_state(SMF_CTX(ctx), &states[STATE_HOME_ADVERTISING]);
	}
	else
	{
		/* already home – just restart the sensor advertiser if needed */
		start_sensor_advertising_ext(ctx);
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
	const struct smf_state *current = smf_current_state_get(SMF_CTX(&app));
	if (current == &states[STATE_AWAY_LOGGING] || current == &states[STATE_HOME_ADVERTISING])
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
 * @brief ISR for NPM1100 CHG interrupt events.
 * 
 * This function is triggered when the NPM1100 CHG interrupt pin is activated.
 * 
 * @param dev Pointer to the device structure.
 * @param cb Pointer to the GPIO callback structure.
 * @param pins Pin mask for the interrupt.
 */
static void npm1100_chg_interrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    /* gpio_pin_get_dt() returns LOGICAL level (already inverted) */
    charging = gpio_pin_get_dt(&npm_status[0]);           		/* 1 = charging, 0 = not charging */

    if (charging) {
        gpio_pin_set_dt(&leds[1], 0);             				/* green off */
        k_work_schedule(&charging_status_work,K_NO_WAIT);       /* start blinking immediately */
    } else {
        k_work_cancel_delayable(&charging_status_work);       	/* stop blink, turn blue off */
        gpio_pin_set_dt(&leds[2], 0);
        /* cable still in? then show solid green, will be turned off in the USB disconnect cb */
        bool vbus_present = (usb_status != USB_DC_CONNECTED); 	/* or POWER_USB_EVT */
        gpio_pin_set_dt(&leds[1], vbus_present);
    }
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

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
/**
 * @brief Callback function for USB device status changes.
 *
 * This function is called when the USB device status changes. It handles
 * connection and disconnection events, as well as configuration events.
 *
 * @param status The new USB device status.
 * @param param Pointer to additional parameters (not used).
 */
static void usbd_app_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *const msg)
{	
	struct app_ctx *state_ctx = &app;
	switch (msg->type)
	{
	case USBD_MSG_VBUS_READY:
		k_work_submit(&usb_connect_work);
		usb_status = USB_DC_CONNECTED; 
		break;
	case USBD_MSG_VBUS_REMOVED:
		k_work_submit(&usb_disconnect_work);
		usb_status = USB_DC_DISCONNECTED; 
		gpio_pin_set_dt(&leds[1], 0);
		break;
	case USBD_MSG_CONFIGURATION:
		if (smf_current_state_get(SMF_CTX(state_ctx)) != &states[STATE_CHARGING])
		{
			k_work_submit(&usb_connect_work);
		}
		usb_status = USB_DC_CONFIGURED;
		break;
	default:
		usb_status = -1; 
		break;
	}
}

/**
 * @brief Enable USB device support and register descriptors.
 * 
 * This function initializes the USB device support, adds string descriptors,
 * 
 * @return 0 on success, negative error code on failure.
 */
static int usb_enable_now(void)
{
	int err;

	struct usbd_desc_node *strs[] = {
		&my_lang, &my_mfr, &my_prod, &my_sn,
	};

	// add string descriptors
	for (size_t i = 0; i < ARRAY_SIZE(strs); ++i)
	{
		err = usbd_add_descriptor(&my_usb, strs[i]);
		if (err)
		{
			LOG_ERR("add_descriptor [%d] failed (%d)", i, err);
			return err;
		}
	}

	// add one FS configuration
	err = usbd_add_configuration(&my_usb, USBD_SPEED_FS, &my_cfg);
	if (err)
	{
		LOG_ERR("add_configuration failed (%d)", err);
		return err;
	}

	// register every class instance we have (here: MSC)
	err = usbd_register_all_classes(&my_usb, USBD_SPEED_FS, 1); 
	if (err)
	{
		LOG_ERR("register_all_classes failed (%d)", err);
		return err;
	}

	err = usbd_msg_register_cb(&my_usb, usbd_app_msg_cb);
	if (err)
	{
		LOG_ERR("Failed to register msg cb: %d", err);
		return err; 
	}
	
	err = usbd_init(&my_usb);
	if (err)
	{
		LOG_ERR("Failed to initialize device support");
		return err;
	}

	err = usbd_enable(&my_usb);
	if (err)
	{
		LOG_ERR("Failed to enable usb device: %d", err);
		return err; 
	}

	return 0; 
}
#else
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
		k_work_submit(&usb_connect_work);
		usb_status = USB_DC_CONNECTED; 
		break;
	case USB_DC_DISCONNECTED:
		k_work_submit(&usb_disconnect_work);
		usb_status = USB_DC_DISCONNECTED; 
		gpio_pin_set_dt(&leds[1], 0);
		break;
	case USB_DC_CONFIGURED:
		if (smf_current_state_get(SMF_CTX(&app)) != &states[STATE_CHARGING])
		{
			k_work_submit(&usb_connect_work);
		}
		usb_status = USB_DC_CONFIGURED; 
		break;
	default:
		usb_status = -1; 
		break;
	}
}
#endif // CONFIG_USB_DEVICE_STACK

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

	// SoC temperature 
	buffer[offset++] = data->soctemp; // 1 byte

	// NPM1100 ERR status
	buffer[offset++] = data->npm_status; // 1 byte

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

	while (1)
	{
		// Wait indefinitely for the BMI270 interrupt signal
		if (k_sem_take(&bmi270_isr_sem, K_FOREVER) == 0)
		{
			// semaphore taken, read data
			BMI270_IMU_Value raw_value;

			// Determine which config to use based on current state
			const struct smf_state *current = smf_current_state_get(SMF_CTX(&app)); 
			
			uint8_t acc_range = (current == &states[STATE_HOME_ADVERTISING]) ? bmi270_acc_config_high.acc_range : bmi270_acc_config_low.acc_range;
			uint8_t gyr_range = (current == &states[STATE_HOME_ADVERTISING]) ? bmi270_gyr_config_high.gyr_range : bmi270_gyr_config_low.gyr_range;

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
				
				// DSP process try 
				float32_t ax = msg.payload.imu.imu_data[0] / 100.0f;
				float32_t ay = msg.payload.imu.imu_data[1] / 100.0f;
				float32_t az = msg.payload.imu.imu_data[2] / 100.0f;

				// Only perform gait analysis when we are in HOME or AWAY
				if (current != &states[STATE_CHARGING])
				{
					// Gait analysis
					float32_t bp_out;
					dsp_filters_process(ax, ay, az, NULL, &bp_out);

					struct gait_metrics gm;
					gait_analyse(bp_out, &gm);
				}
				
				if (k_msgq_put(&sensor_message_queue, &msg, K_NO_WAIT) != 0)
				{
					LOG_WRN("BMI270 queue full");
				}
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

	while (1)
	{
		// --- 1Hz BME680 Reading ---
		if (!bme688_dev || !device_is_ready(bme688_dev))
		{
			LOG_ERR("BME680 device not ready in handler thread");
			k_sleep(K_SECONDS(5));
			continue;
		}

		struct sensor_value temp, press;
		int ret = sensor_sample_fetch(bme688_dev);
		if (ret < 0)
		{
			LOG_ERR("Failed to fetch from BME688: %d", ret);
			k_sleep(K_MSEC(500));
			continue;
		}

		if (sensor_channel_get(bme688_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) == 0 && sensor_channel_get(bme688_dev, SENSOR_CHAN_PRESS, &press) == 0)
		{
			sensor_message_t msg = {.type = SENSOR_MSG_TYPE_ENVIRONMENT};
			msg.payload.env.temperature = (uint16_t)(sensor_value_to_double(&temp) * 100);
			msg.payload.env.pressure = (uint32_t)(sensor_value_to_double(&press) * 100); // kPa x 100 = hPa x 10 gives 0.1 hPa resolution
			msg.payload.env.timestamp = k_uptime_get_32();

			if (k_msgq_put(&sensor_message_queue, &msg, K_NO_WAIT) != 0)
			{
				LOG_WRN("BMP390 queue full");
			}
		}
		else
		{
			LOG_ERR("Failed to read BMP390 (simulated): %d", ret);
		}

		k_msleep(BMP390_READ_INTERVAL);
	}
}



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

	struct app_ctx *ctx = &app; 

	int ret;
	sensor_message_t received_msg;						// Received from message queue
	ble_packet_t current_sensor_state = {0};			// Initialise
	current_sensor_state.timestamp = k_uptime_get_32(); // Set initial timestamp

	static imu_payload_t imu_batch_buffer[MAX_IMU_SAMPLES_IN_PACKET];
	static uint8_t imu_batch_count = 0;
	static uint32_t last_adv_payload_update_time = 0;

	uint32_t save_timer = k_uptime_get_32(); // Timer for saving nonce

	fs_file_t_init(&log_file); // Initialise file object structure

	while (1)
	{
		/* Get each message from the message queue */
		ret = k_msgq_get(&sensor_message_queue, &received_msg, K_MSEC(200));
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
		
				if (received_msg.payload.env.timestamp > current_sensor_state.timestamp)
				{
					current_sensor_state.timestamp = received_msg.payload.env.timestamp;
				}
				break;
			case SENSOR_MSG_TYPE_MONITOR:
				current_sensor_state.battery = received_msg.payload.monitor.battery;
				current_sensor_state.soctemp = received_msg.payload.monitor.soc_temp; 
				current_sensor_state.npm_status = received_msg.payload.monitor.npm_status; 

				if (received_msg.payload.monitor.timestamp > current_sensor_state.timestamp)
				{
					current_sensor_state.timestamp = received_msg.payload.monitor.timestamp;
				}
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
		const struct smf_state *active_state = smf_current_state_get(SMF_CTX(ctx));
		uint32_t current_time = k_uptime_get_32();

		// Perform actions based on state only if new data arrived
		if (active_state == &states[STATE_HOME_ADVERTISING])
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
					if (atomic_test_bit(&ctx->flags, FLAG_ADV_RUNNING) && atomic_get(&ctx->current_adv_type) == SENSOR_ADV_PAYLOAD_TYPE)
					{
						ret = bt_le_ext_adv_set_data(g_adv_sensor_ext_handle, ad_sensor_agg, ARRAY_SIZE(ad_sensor_agg), NULL, 0);
						if (ret && ret != -EALREADY)
						{ // -EALREADY is fine, means it was already trying to send
							LOG_WRN("Failed to update aggregated ADV data: %d", ret);
						}
						else
						{
							// LOG_DBG("Updated ADV with %u IMU samples.", imu_batch_count);
						}
					}
				}
				imu_batch_count = 0;
				last_adv_payload_update_time = current_time;
			}

			if (current_time - save_timer > NONCE_SAVE_INTERVAL)
			{
				ret = settings_save_one(BORUS_SETTINGS_PATH_NONCE, (const void *)&nonce_counter, sizeof(nonce_counter));
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
		else if (active_state == &states[STATE_AWAY_LOGGING])
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
				LOG_DBG("Closing log file.");
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
 * @brief Initialize the LEDs GPIO.
 * 
 * This function initializes the GPIO pins for the LEDs defined in the device tree.
 * 
 * @return 0 on success, negative error code on failure.
 */
static int init_leds(void)
{
	for (int i = 0; i < ARRAY_SIZE(leds); i++)
	{
		if (!gpio_is_ready_dt(&leds[i]))
		{
			LOG_ERR("LED%d on pin %d is not ready\n", i, leds[i].pin);
			return -ENODEV; 
		}
		else
		{
			int ret = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
			if (ret)
			{
				LOG_ERR("Failed to configure LED%d\n", i);
				return ret;
			}
		}
	}
	LOG_INF("Step 1: LEDs initialised"); 
	return 0; 
}

/**
 * @brief Initialize the NPM1100 PMIC status pins.
 * 
 * This function initializes the GPIO pins for the NPM1100 PMIC status defined in the device tree.
 * It also sets up an interrupt callback for the PMIC status pin.
 * 
 * @return 0 on success, negative error code on failure.
 */
static int init_npm_pins(void)
{
	for (int i = 0; i < ARRAY_SIZE(npm_status); i++)
	{
		if (!gpio_is_ready_dt(&npm_status[i]))
		{
			LOG_ERR("NPM1100 status pin not ready");
			return -ENODEV;
		}
		else
		{
			int ret = gpio_pin_configure_dt(&npm_status[i], GPIO_INPUT); 
			if (ret)
			{
				LOG_ERR("Failed to configure NPM1100 status pin as INPUT: %d", ret);
				return ret;
			}
		}
	}

	// NPM1100 Interrupt
	gpio_pin_interrupt_configure_dt(&npm_status[0], GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&npm1100_chg_interrupts_cb_data, npm1100_chg_interrupt, BIT(npm_status[0].pin));
	gpio_add_callback_dt(&npm_status[0], &npm1100_chg_interrupts_cb_data); 

	LOG_INF("Step 2: PMIC status pin initialised");
	return 0; 
}

/**
 * @brief Initialize the sensor IRQ for BMI270.
 * 
 * This function initializes the GPIO pin for the BMI270 interrupt and sets up an interrupt callback.
 * 
 * @return 0 on success, negative error code on failure.
 */
static int init_sensor_irq(void)
{
	// BMI270 (IMU)
	if (!gpio_is_ready_dt(&bmi270_interrupts))
	{
		LOG_ERR("Interrupt GPIO is not ready on pin %d", bmi270_interrupts.pin);
		return -ENODEV;
	}
	int ret = gpio_pin_configure_dt(&bmi270_interrupts, GPIO_INPUT);
	if (ret)
	{
		LOG_ERR("Failed to configure BMI270 interrupt as input: %d", ret);
		return ret;
	}
	// BMI270 Interrupts
	gpio_pin_interrupt_configure_dt(&bmi270_interrupts, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&bmi270_interrupts_cb_data, bmi270_int1_interrupt_triggered, BIT(bmi270_interrupts.pin));
	gpio_add_callback_dt(&bmi270_interrupts, &bmi270_interrupts_cb_data);

	LOG_INF("Step 3: Sensor's IRQ configured"); 
	return 0; 
}

/**
 * @brief Initialize the SPI bus for BMI270.
 * 
 * This function checks if the SPI bus is ready and initializes the BMI270 sensor.
 * It also configures the FIFO and sets the sensor mode and configurations.
 * 
 * @return 0 on success, negative error code on failure.
 */
static int bmi270_init_full(void)
{	
	/* Check SPI bus */
	if (!spi_is_ready_dt(&bmi270_spi))
	{	
		LOG_ERR("SPI bus for BMI270 is not ready"); 
		return -ENODEV;
	}

	/* Configurations of BMI270 */
	bool bmi270_ret = bmi270_init(&bmi270_ctx, &bmi270_spi);
	if (!bmi270_ret)
	{
		LOG_ERR("Failed to initialise BMI270 sensor");
		return -ENODEV;
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

	bmi270_conf_fifo(&bmi270_ctx, 4096, &bmi270_fifo_default_config);
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

	LOG_INF("Step 7: BMI270 sensor initialised"); 
	return 0; 
}

/**
 * @brief Initialize the BME688 environmental sensor.
 * 
 * This function checks if the BME688 device is ready and initializes it.
 * 
 * @return 0 on success, negative error code on failure.
 */
static int bmp390_init_full(void)
{
	// BME688 (Environmental)
	if (!device_is_ready(bme688_dev))
	{
		LOG_ERR("BME688: device not ready");
		return -ENODEV;
	}

	LOG_INF("Step 8: Environmental sensor initialised"); 
	return 0; 
}

/**
 * @brief Initialize the battery measurement.
 * 
 * This function enables the battery measurement and logs the status.
 * 
 * @return 0 on success, negative error code on failure.
 */
static int battery_measure_init(void)
{
	int ret = battery_measure_enable(true);
	if (ret != 0)
	{
		LOG_ERR("Failed to initialise battery measurement: %d", ret);
		return ret;
	}

	LOG_INF("Step 9: Battery monitoring enabled"); 
	return 0;
}

/**
 * @brief Start the watchdog timer.
 * 
 * This function initializes and starts the watchdog timer with a specified timeout.
 * It also installs a timeout configuration and feeds the watchdog to prevent reset.
 * 
 * @return 0 on success, negative error code on failure.
 */
static int watchdog_start(void)
{
	if (!device_is_ready(wdt_dev))
	{
		LOG_ERR("WDT device not ready");
		return -ENODEV;
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

	int ret = wdt_setup(wdt_dev, 0);
	if (ret)
	{
		LOG_ERR("Failed to start WDT: %d", ret);
		return ret;
	}
	wdt_feed(wdt_dev, wdt_channel_id); 

	LOG_INF("Step 5: Watchdog settup and fed");
	return 0; 
}

/**
 * @brief Initialize kernel timers and workqueue.
 * 
 * This function initializes various kernel timers and work items used in the application.
 */
void init_kernel_timers_and_work(void)
{
	k_timer_init(&battery_timer, (k_timer_expiry_t)battery_timer_expiry, NULL);
	k_timer_init(&sync_check_timer, sync_check_timer_expiry, NULL);
	k_timer_init(&scan_close_timer, scan_close_timer_expiry, NULL);
	k_timer_init(&wdt_feed_timer, (k_timer_expiry_t)watchdog_feed, NULL);

	k_work_init(&usb_connect_work, usb_connect_work_handler);
	k_work_init(&usb_disconnect_work, usb_disconnect_work_handler);
	k_work_init(&battery_timeout_work, battery_timeout_work_handler);
	k_work_init(&scan_found_ap_work, scan_found_ap_work_handler);
	k_work_init(&scan_open_work, scan_open_work_handler);
	k_work_init(&scan_close_work, scan_close_work_handler);
	k_work_init(&sync_check_work, sync_check_work_handler);					// Init new work
	k_work_init_delayable(&sync_adv_stop_work, sync_adv_stop_work_handler); // Init new delayable work
	k_work_init_delayable(&sync_scan_trigger_work, sync_scan_trigger_handler);

	k_timer_user_data_set(&wdt_feed_timer, (void *)wdt_dev);
	k_timer_start(&wdt_feed_timer, K_MSEC(WDT_FEED_INTERVAL_MS), K_MSEC(WDT_FEED_INTERVAL_MS));
	k_timer_start(&battery_timer, BATTERY_READ_INTERVAL, BATTERY_READ_INTERVAL);

	LOG_INF("Step 6: Workqueue and timers initialised"); 
}

/**
 * @brief Initialize the state machine.
 * 
 * This function initializes the state machine with the initial state and clears all flags.
 */
void init_state_machine(void)
{
	atomic_set(&app.flags, 0); 				// Clear all flags
    atomic_set(&app.current_adv_type, 2); 	// '2' for idle/none
    app.missed_sync_responses = 0;
    app.sync_check_interval_ms = SYNC_CHECK_INTERVAL_BASE_MS;

    smf_set_initial(SMF_CTX(&app), &states[STATE_INIT]);	// Set the initial state of the SMF

	LOG_INF("Step 11: State machine initialised"); 
}

/**
 * @brief Initialize the NVM and settings subsystem.
 * 
 * This function initializes the disk access, mounts the file system, initializes
 * the settings subsystem, and handles reset causes and watchdog reboot counts.
 * 
 * @return 0 on success, negative error code on failure.
 */
static int init_nvm_and_settings(void)
{
	int ret; 

	ret = disk_access_ioctl("NOR", DISK_IOCTL_CTRL_INIT, NULL);
	if (ret)
	{
		LOG_ERR("Failed to init disk: %d", ret);
		return ret;
	}
	
	struct fs_mount_t *mp = &lfs_mount_p;
	ret = fs_mount(mp);
	if (ret == 0)
	{
		struct fs_statvfs stats;
		fs_statvfs(mp->mnt_point, &stats);
		LOG_INF("File system mounted, bfree = %lu", stats.f_bfree);
	}
	else
	{
		LOG_ERR("Failed to mount file system: %d", ret);
		return ret; 
	}

	ret = settings_subsys_init();
	if (ret)
	{
		LOG_ERR("Failed to initialise settings subsystem: %d", ret);
		return ret;
	}

	ret = settings_load_subtree(BORUS_SETTINGS_SUBTREE);
	if (ret && ret != -ENOENT)
	{
		LOG_ERR("Failed to load settings for 'borus/state': %d", ret);
		return ret;
	}

	uint32_t rst_cause = 0;
	hwinfo_get_reset_cause(&rst_cause);
	hwinfo_clear_reset_cause();

	if (rst_cause & RESET_WATCHDOG)
	{
		wdt_reboot_count ++;
		settings_save_one(BORUS_SETTINGS_PATH_WDT, &wdt_reboot_count, sizeof(wdt_reboot_count));
	} 
	else if (wdt_reboot_count)
	{
		wdt_reboot_count = 0;
		settings_save_one(BORUS_SETTINGS_PATH_WDT, &wdt_reboot_count, sizeof(wdt_reboot_count));
	}

	if (!(rst_cause & RESET_WATCHDOG) && wdt_reboot_count)
	{
		wdt_reboot_count = 0;
		settings_save_one(BORUS_SETTINGS_PATH_WDT,
						  &wdt_reboot_count,
						  sizeof(wdt_reboot_count));
	}

	LOG_INF("Step 4: Settings loaded, nonce = %llu, watchdog boot counter = %u", nonce_counter, wdt_reboot_count);

	return 0; 
}

/**
 * @brief Initialize the BLE stack and set up advertising.
 * 
 * This function initializes the BLE stack, sets a custom static address,
 * creates an extended advertising set for sensor data, and configures the scan filter accept list.
 * 
 * @return 0 on success, negative error code on failure.
 */
static int init_ble_full(void)
{	
	const char *fallback = "EE:55:55:55:00:00"; 
	char addr_buf[18];

	bool ok = factory_data_get_ble_addr(addr_buf);
	if (ok && strlen(addr_buf) == 17)
	{
		strcpy(ble_addr_str, addr_buf);
	}
	else
	{
		strcpy(ble_addr_str, fallback); 
	}
	
	int	ret = set_custom_static_addr(ble_addr_str); 
	if (ret)
	{
		LOG_ERR("ID create failed: %d", ret); 
		return ret;
	}

	ret = bt_enable(NULL);
	if (ret)
	{
		LOG_ERR("Failed to enable BLE stack: %d", ret);
		return ret;
	}
	ble_enabled = true; 

	ret = create_sensor_ext_adv_set();
	if (ret)
	{
		LOG_ERR("Failed to create sensor extended adv set: %d", ret);
		return ret;
	}

	ret = setup_scan_filter();
	if (ret)
	{
		LOG_ERR("Failed to configure scan filter accept list: %d", ret);
		return ret;
	}

	LOG_INF("Step 10: BLE enabled"); 
	return 0; 
}

/**
 * @brief Initialize the crypto subsystem and import the AES key.
 * 
 * This function initializes the PSA crypto subsystem, imports a static AES key,
 * and sets up the key attributes for encryption and decryption.
 * 
 * @return 0 on success, negative error code on failure.
 */
static int init_crypto(void)
{
	int ret = psa_crypto_init();
	if (ret != PSA_SUCCESS)
	{
		LOG_ERR("Failed to initialise crypto: %d", ret);
		return ret;
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
		return ret;
	}
	psa_reset_key_attributes(&attr); // Good practice

	LOG_INF("Step 12: AES Key Imported Successfully (ID: %u)", (unsigned int)g_aes_key_id);
	return 0; 
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

	/* Initialise the SMF first with STATUS_INIT */
	init_state_machine();

	ret = init_leds(); 			/* LEDs + error indication */
	if (ret) goto fail; 
	ret = init_npm_pins(); 		/* PMIC status + interrupt */
	if (ret) goto fail; 
	ret = init_sensor_irq(); 		/* BMI270 INT pin          */
	if (ret) goto fail; 

	/* ─── Phase-1: Subsystem Loading ───────────────────────── */
	ret = init_nvm_and_settings();
	if (ret) goto fail; 

	device_state_t initial_state = (wdt_reboot_count >= WDT_REBOOT_NUMBER_THRESHOLD) ? STATE_FAULT : STATE_HOME_ADVERTISING;

	if (initial_state != STATE_FAULT)
	{	 
		ret = watchdog_start();
		if (ret) goto fail; 
		init_kernel_timers_and_work();
		ret = bmi270_init_full();
		if (ret) goto fail; 
		ret = bmp390_init_full();
		if (ret) goto fail; 
		ret = battery_measure_init();
		if (ret) goto fail; 
		ret = init_crypto();
		if (ret) goto fail; 
		ret = init_ble_full();
		if (ret) goto fail; 
		queue_initial_monitoring_info();
		smf_set_state(SMF_CTX(&app), &states[initial_state]);

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
		ret = usb_enable_now();
#else
		ret = usb_enable(usb_dc_status_cb);
#endif
		if (ret) goto fail; 
		usb_enabled = true; 
		dsp_filters_init();
		gait_init();

		for (int i = 0; i < 3; i++)
		{
			gpio_pin_set_dt(&leds[i], 1);
			k_msleep(250);
			gpio_pin_set_dt(&leds[i], 0);
		}

		k_tid_t tid;
		tid = k_thread_create(&bmi270_handler_thread_data, bmi270_handler_stack_area,
							  K_THREAD_STACK_SIZEOF(bmi270_handler_stack_area), bmi270_handler_func, NULL, NULL, NULL,
							  BMI270_HANDLER_PRIORITY, 0, K_MSEC(1000));
		k_thread_name_set(tid, "bmi270");
		tid = k_thread_create(&bmp390_handler_thread_data, bmp390_handler_stack_area,
							  K_THREAD_STACK_SIZEOF(bmp390_handler_stack_area), bmp390_handler_func, NULL, NULL, NULL,
							  BMP390_HANDLER_PRIORITY, 0, K_MSEC(1000));
		k_thread_name_set(tid, "bmp390");
		tid = k_thread_create(&ble_logger_thread_data, ble_logger_stack_area,
							  K_THREAD_STACK_SIZEOF(ble_logger_stack_area), ble_logger_func, NULL, NULL, NULL,
							  BLE_THREAD_PRIORITY, 0, K_MSEC(1000));
		k_thread_name_set(tid, "ble_log");
	}
	else
	{
		smf_set_state(SMF_CTX(&app), &states[initial_state]);
	}

	power_down_unused_ram(); 

	while (1)
	{	
		k_sleep(K_MINUTES(5));
	}

fail:
	smf_set_state(SMF_CTX(&app), &states[STATE_FAULT]); 

	return 0; 
}