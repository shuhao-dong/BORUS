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

LOG_MODULE_REGISTER(THINGY, LOG_LEVEL_DBG);

/* -------------------- Watchdog Timer --------------------*/

#define WDT_TIMEOUT_MS 4000		  // Watchdog timeout
#define WDT_FEED_INTERVAL_MS 1000 // Watchdog feed interval

static const struct device *wdt_dev = DEVICE_DT_GET_ONE(nordic_nrf_wdt); // Get the WDT device
static int wdt_channel_id = -1;											 // Initialize the WDT channel ID
static struct k_timer wdt_feed_timer;									 // Timer for feeding the watchdog

/* -------------------- BLE Packet Encryption -------------------- */

#define BORUS_SETTINGS_PATH "borus/state" // Save nonce in NVM, allow reboot
#define NONCE_SAVE_INTERVAL 5 * 60 * 1000 // Save nonce every 5 minutes

static psa_key_id_t g_aes_key_id = PSA_KEY_ID_NULL; // Initialize the AES key ID
static uint64_t nonce_counter = 0;					// Unique nonce for each BLE message

/* -------------------- Thread Configurations -------------------- */

// Stack sizes
#define BMI270_HANDLER_STACKSIZE 1024
#define BMP390_HANDLER_STACKSIZE 1024
#define BLE_LOGGER_THREAD_STACKSIZE 4096
#define SCANNER_THREAD_STACKSIZE 1024

// Priorities (Lower number = higher priority)
#define BMI270_HANDLER_PRIORITY 5 // Highest sensor priority due to higher sample rate
#define BMP390_HANDLER_PRIORITY 6 // Medium sensor priority due to lower sample rate
#define BLE_THREAD_PRIORITY 7	  // Lower priority tasks for BLE and logging
#define SCANNER_THREAD_PRIORITY 7 // Lower priority tasks for scan AP heartbeat

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
	STATE_INIT, // Initial state before normal operation
	STATE_FIND_PERIOD,
	STATE_HOME_ADVERTISING, // At home, advertise sensor data
	STATE_AWAY_LOGGING,		// Away, log sensor data
	STATE_CHARGING			// USB connected
} device_state_t;


static struct k_work heartbeat_timeout_work; // Work item for heartbeat timeout -> Away state
static struct k_work battery_timeout_work;	// Work item for battery timeout -> Periodic voltage reading
static struct k_work usb_connect_work;	// Work item for USB connect -> CHARGING state
static struct k_work usb_disconnect_work;	// Work item for USB disconnect -> HOME state (or chosen default)
static struct k_work scan_found_ap_work;	// Work item for Scan Found AP -> HOME state
static struct k_work scan_open_work;
static struct k_work scan_close_work;

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
static struct k_timer scan_open_timer;
static struct k_timer scan_close_timer;

/* -------------------- Configuration Constants -------------------- */

#define BMP390_READ_INTERVAL 1000								  // Read environment at 1 Hz
#define BATTERY_READ_INTERVAL K_MINUTES(15)						  // Every 15 minute read one battery voltage
#define HEARTBEAT_TIMEOUT K_SECONDS(90)							  // If no heartbeat for 90s, assume away
#define BLE_ADV_INTERVAL_MIN 32									  // BLE advertise interval 32*0.625 ms
#define BLE_ADV_INTERVAL_MAX 33									  // BLE advertise interval
#define SENSOR_DATA_PACKET_SIZE 20								  // Size of the sensor data
#define NONCE_LEN 8												  // Size of the encryption nonce
#define ENC_ADV_PAYLOAD_LEN (NONCE_LEN + SENSOR_DATA_PACKET_SIZE) // Size of the packet data
#define SCAN_INTERVAL K_MINUTES(1)								  // BLE scan interval, start scan every 1 minute
#define SCAN_WINDOW K_SECONDS(5)								  // BLE scan window, when started, scan for 5 seconds
#define PRESSURE_BASE_HPA_X10 8500								  // Base offset in hPa x 10
#define TEMPERATURE_LOW_LIMIT 30								  // -30 degree as the lowest temperature of interest
#define TEMPERATURE_HIGH_LIMIT 40								  // +40 degree as the highest temperature of interest

/* -------------------- File system and MSC -------------------- */

#define LOG_FILE_PATH "/lfs1/imu_log.bin" // File path for the log file
#define LFS_MOUNT_POINT "/lfs1"			  // Mount point for the file system

USBD_DEFINE_MSC_LUN(NAND, "Zephyr", "BORUS", "0.00"); // Define the USB MSC LUN

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

// Buffer for dynamic manufacturer data in advertisement
static uint8_t manuf_plain[SENSOR_DATA_PACKET_SIZE];
static uint8_t manuf_encrypted[ENC_ADV_PAYLOAD_LEN];

// Advertising data structure (hold device name and adv type)
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME // Use the name defined in Kconfig
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/**
 * If enable BT_LE_ADV_OPT_SCANNABLE, then pass sd as scan response data
 * 	- Benifit: Scanner will see the name and flag of adv event, we are scannable undirected adv
 * 	- Lose: Double radio on time so consume more power
 *
 * If not enable BT_LE_ADV_OPT_SCANNABLE, then pass null to scan response data
 * 	- Save power as no scan response data is provided, we are non-connectable undirected adv
 * 	- Lose device name and data flag
 */

// BLE advertisement parameters
static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	BT_LE_ADV_OPT_USE_IDENTITY, // Use identity MAC for advertisement
	BLE_ADV_INTERVAL_MIN,		// Min advertisement interval (min * 0.625)
	BLE_ADV_INTERVAL_MAX,		// Max advertisement interval (max * 0.625), add short delay to avoid aliasing
	NULL						// not directed, pass NULL
);

// Scan response data (hold device name and flag)
struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)};

// Main adv packet (hold sensor data)
struct bt_data ad[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_encrypted, sizeof(manuf_encrypted))};

// BLE scan parameters
static const struct bt_le_scan_param scan_param = {
	.type = BT_HCI_LE_SCAN_PASSIVE,
	.options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
	.interval = BT_GAP_SCAN_FAST_INTERVAL,
	.window = BT_GAP_SCAN_FAST_WINDOW,
};

// Define the list of target AP addresses
static const char *target_ap_addrs[] = {
	"2C:CF:67:89:E0:5D",
	"AA:BB:CC:DD:EE:FF",
};
static const size_t num_target_aps = ARRAY_SIZE(target_ap_addrs);
static volatile bool adv_running = false; // Flag indicating adv or not
static volatile bool scan_active = false;

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

#define LOG_SYNC_THRESHOLD 15000 // Record to log before fs_sync, this tolerates 5-min data missing

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);
static struct fs_mount_t lfs_mount_p = {
	.type = FS_LITTLEFS,
	.fs_data = &storage,
	.storage_dev = (void *)PM_LITTLEFS_STORAGE_ID,
	.mnt_point = LFS_MOUNT_POINT,
};

#define DEFAULT_HB_MS 60000
#define EARLY_MARGIN_MS 2000
#define LATE_MARGIN_MS 2000
#define MISSES_BEFORE_FALLBACK 3

static uint32_t hb_period_ms = DEFAULT_HB_MS;
static uint8_t hb_misses = 0;
static K_MUTEX_DEFINE(hb_lock);

static struct k_timer scan_open_timer;
static struct k_timer scan_close_timer;

static uint64_t next_deadline_ms = 0; /* absolute time (ms)   */
static bool period_learned = false; 

/**
 * @brief Get the current time in milliseconds.
 */
static inline uint64_t ms_now(void)	  /* shorthand            */
{
	return k_uptime_get();
}

/**
 * @brief Schedule the scan open timer based on the next deadline.
 *
 * This function calculates the time until the next deadline and sets the
 * scan open timer accordingly. If no period has been learned, it does nothing.
 */
static void schedule_open_from_deadline(void)
{
    if (!period_learned) {            /* no beacon yet → do nothing   */
        return;
	}

    int32_t delta = (int32_t)(next_deadline_ms - EARLY_MARGIN_MS - ms_now());
    if (delta < 0) {
        delta = 0;
    }
    k_timer_start(&scan_open_timer, K_MSEC(delta), K_NO_WAIT);
}

/* -------------------- State Management Functions -------------------- */

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
 * @brief Start BLE advertising when enter HOME state
 *
 * This function starts BLE advertising with the specified parameters and data.
 * It will not start if already running.
 */
static void start_advertising(void)
{
	int ret;

	if (adv_running)
	{
		return;
	}

	ret = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0);

	if (ret && ret != -EALREADY)
	{
		LOG_ERR("Failed to start advertisement: %d", ret);
	}
	else
	{
		adv_running = true;
		LOG_DBG("Advertising started/updated");
	}
}

/**
 * @brief Stop BLE advertising when enter AWAY state
 *
 * This function stops BLE advertising. It will not stop if already stopped.
 */
static void stop_advertising(void)
{
	if (!adv_running)
	{
		return;
	}

	int ret = bt_le_adv_stop();

	if (ret == 0 || ret == -EALREADY)
	{
		adv_running = false;
		LOG_DBG("Advertising stopped");
	}
	else
	{
		LOG_ERR("Failed to stop advertisement: %d", ret);
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
	case STATE_FIND_PERIOD:
		LOG_INF("Scanning to learn AP period ... ");
		k_work_submit(&scan_open_work);
		break;
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
		k_timer_start(&scan_open_timer, K_MSEC(hb_period_ms - EARLY_MARGIN_MS), K_NO_WAIT);
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
 * @brief Parse the manufacturer data from the advertisement packet to get time for scan.
 *
 * This function checks if the manufacturer data contains a specific CID and
 * extracts the period in milliseconds from it.
 *
 * @param d Pointer to the Bluetooth data structure.
 * @param user_data Pointer to user data (in this case, a pointer to store the period).
 *
 * @return true to continue parsing, false to stop parsing.
 */
static bool parse_mfr_period(struct bt_data *d, void *user_data)
{
	uint16_t *period_ms = user_data;

	if (d->type == BT_DATA_MANUFACTURER_DATA &&
		d->data_len >= 4 && 				/* 2 B CID + 2 B time  */
		sys_get_le16(d->data) == 0x0059)	/* Nordic CID          */
	{ 
		uint16_t t_s = sys_get_le16(d->data + 2); 	/*   Time_L + Time_H   */
		if (t_s == 0)
		{						 	/* ignore “0 s” keep-alive bursts */
			return true; 			/* keep parsing                      */
		}

		*period_ms = t_s * 1000U; 	/* hand back to caller */
		return false;			  	/* stop any further AD */
	}
	return true; 					/* keep iterating      */
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
	bt_addr_to_str(&addr->a, addr_str, sizeof(addr_str));

	/* 1. Match the target address */
	bool known_ap = false;
	for (size_t i = 0; i < num_target_aps; i++)
	{
		if (strcmp(addr_str, target_ap_addrs[i]) == 0)
		{
			known_ap = true;
			break;
		}
	}
	if (!known_ap)
	{
		return;
	}

	/* 2. Pull the period out of the Manufacturer field */
	uint16_t new_period_ms = 0;
	bt_data_parse(buf, parse_mfr_period, &new_period_ms);
	if (new_period_ms == 0)
	{ /* malformed or keep-alive  */
		return;
	}

	LOG_DBG("packet @%llu, δ = %lld ms",
		(unsigned long long)ms_now(),                     /* arrival      */
		(long long)( (int64_t)ms_now() -
					 (int64_t)next_deadline_ms) );

	LOG_INF("Heartbeat from %s  period=%u ms  RSSI=%d",
			addr_str, new_period_ms, rssi);

	/* 3. Update shared timing info */
	k_mutex_lock(&hb_lock, K_FOREVER);
	period_learned = true; 
	hb_period_ms = new_period_ms;
	hb_misses = 0;
	next_deadline_ms = ms_now() + hb_period_ms;
	k_mutex_unlock(&hb_lock);

	/* 4. State-dependent handling */

	device_state_t s = atomic_get(&current_state);

	if (s == STATE_FIND_PERIOD)
	{
		/* boot-time negotiation finished */
		bt_le_scan_stop(); /* stop continuous scan */
		scan_active = false;

		schedule_open_from_deadline(); /* first timed window */
		start_advertising();		   /* start outgoing ADV */
		k_timer_start(&heartbeat_timeout_timer,
					  HEARTBEAT_TIMEOUT, K_NO_WAIT);
		enter_state(STATE_HOME_ADVERTISING);
		return;
	}

	/* normal HOME operation: reschedule next window */
	schedule_open_from_deadline();

	/* close current scan immediately */
	bt_le_scan_stop();
	scan_active = false;

	/* refresh watchdog / ensure we stay HOME */
	k_work_submit(&scan_found_ap_work);
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
 * @brief Handle the scan open work item.
 *
 * This function is called when the scan open work item is triggered. It starts
 * the BLE scan and stops advertising if necessary.
 *
 * @param w Pointer to the work structure.
 */
static void scan_open_work_handler(struct k_work *w)
{
    /* Stop legacy ADV while scanning */
    if (adv_running) {
        bt_le_adv_stop();
        adv_running = false;
    }

    int err = bt_le_scan_start(&scan_param, scan_cb);
    if (err && err != -EALREADY) {
        LOG_ERR("bt_le_scan_start failed (%d)", err);
        return;
    }
    scan_active = true;

	LOG_DBG("open @%llu  close @%llu  (deadline %llu)",
		(unsigned long long)(next_deadline_ms - EARLY_MARGIN_MS),
		(unsigned long long)(next_deadline_ms + LATE_MARGIN_MS),
		(unsigned long long) next_deadline_ms);

    /* --------------------------------------------------------------
     *  BEFORE we know the period (i.e. in STATE_FIND_PERIOD)
     *  we want one *continuous* scan – no close timer.
     * -------------------------------------------------------------- */
    if (atomic_get(&current_state) == STATE_FIND_PERIOD) {
        return;                     /* keep listening indefinitely   */
    }

    /* Normal operation → close after EARLY+LATE ms */
    k_timer_start(&scan_close_timer,
                  K_MSEC(EARLY_MARGIN_MS + LATE_MARGIN_MS),
                  K_NO_WAIT);
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
    if (scan_active) {
        bt_le_scan_stop();
        scan_active = false;
    }

    /* Count misses / enter fallback **only after first heartbeat** */
    if (period_learned) {
        if (++hb_misses >= MISSES_BEFORE_FALLBACK) {
            LOG_WRN("Missed %u heartbeats → fallback to 60 s",
                    MISSES_BEFORE_FALLBACK);
            hb_period_ms     = DEFAULT_HB_MS;
            hb_misses        = 0;
            next_deadline_ms = ms_now() + hb_period_ms;
        }
    }

    /* Restart advertising if we are HOME */
    if (!adv_running &&
        atomic_get(&current_state) == STATE_HOME_ADVERTISING) {
        start_advertising();
    }

    /* Schedule the *next* window; will no-op until period_learned */
    schedule_open_from_deadline();
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
 * @brief Handle heartbeat timeout events.
 *
 * This function is called when the heartbeat timer expires. It transitions the
 * device to the AWAY state if the current state is HOME.
 *
 * @param work Pointer to the work structure.
 */
static void heartbeat_timeout_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	LOG_DBG("Workqueue: Heartbeat timeout expired, entering AWAY");

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

	if (atomic_get(&current_state) != STATE_HOME_ADVERTISING)
	{
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
		enter_state(STATE_HOME_ADVERTISING);
	}
	else
	{
		LOG_DBG("Workqueue: In HOME, restart timer");
		k_timer_start(&heartbeat_timeout_timer, HEARTBEAT_TIMEOUT, K_NO_WAIT);
	}
}

/**
 * @brief Handle the scan open timer expiry event.
 *
 * This function is called when the scan open timer expires. It checks if the
 * period has been learned and schedules the scan open work accordingly.
 *
 * @param t Pointer to the timer structure.
 */
static void scan_open_timer_expiry(struct k_timer *t)
{	
	if (!period_learned) {            /* still in continuous scan     */
        return;                       /* wait for first heartbeat     */
    }

	int32_t late = (int32_t)(ms_now() - (next_deadline_ms - EARLY_MARGIN_MS));

    if (late > LATE_MARGIN_MS) {
        /* Window would open too late → count as miss, try next time */
        if (++hb_misses >= MISSES_BEFORE_FALLBACK) {
            LOG_WRN("Missed %u heartbeats → fallback to default period",
                    MISSES_BEFORE_FALLBACK);
            hb_period_ms     = DEFAULT_HB_MS;
            hb_misses        = 0;
            next_deadline_ms = ms_now() + hb_period_ms;
        }
        schedule_open_from_deadline();
        return;
    }

    /* OK, still in band → proceed with real radio scan */
    k_work_submit(&scan_open_work);
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
	k_work_submit(&scan_close_work);
}

/**
 * @brief Initialize the timed scan functionality.
 *
 * This function initializes the work items and timers used for timed scanning
 * and state transitions.
 */
static void timed_scan_init(void)
{
	/* initialise work items */
	k_work_init(&scan_open_work, scan_open_work_handler);
	k_work_init(&scan_close_work, scan_close_work_handler);

	/* initialise timers */
	k_timer_init(&scan_open_timer, scan_open_timer_expiry, NULL);
	k_timer_init(&scan_close_timer, scan_close_timer_expiry, NULL);
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
 * @brief Handle heartbeat timer expiry events.
 *
 * This function is called when the heartbeat timer expires. It submits the
 * heartbeat timeout work to the workqueue.
 *
 * @param timer_id Pointer to the timer structure.
 */
void heartbeat_timeout_expiry(struct k_timer *timer_id)
{
	LOG_DBG("Heartbeat timer expired, submitting work");
	k_work_submit(&heartbeat_timeout_work);
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
		LOG_DBG("USB Disconnected - Entering Home state");
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
 * @brief Prepare a BLE packet from sensor data.
 *
 * This function encodes sensor data into a BLE packet format, ensuring that
 * the data fits within the specified buffer size.
 *
 * @param data Pointer to the sensor data structure.
 * @param buffer Pointer to the output buffer.
 * @param buffer_size Size of the output buffer.
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

	/* IMU data */
	memcpy(&buffer[offset], data->imu_data, sizeof(data->imu_data));
	offset += sizeof(data->imu_data); // 12 bytes

	/* Timestamp */
	sys_put_le32(data->timestamp, &buffer[offset]);
	offset += 4; // 4 bytes

	/* Battery percentage */
	buffer[offset++] = data->battery; // 1 byte

	// Check total size
	if (offset != SENSOR_DATA_PACKET_SIZE)
	{
		LOG_WRN("BLE packet size mimatch: expected %d, got %d", SENSOR_DATA_PACKET_SIZE, offset);
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
static int encrypt_sensor_block(const uint8_t *plain, uint8_t *out)
{
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

	// --- Copy the actual 8-byte nonce to the output buffer ---
	memcpy(out, nonce, NONCE_LEN);

	// Encrypt data, placing ciphertext *after* the nonce space in 'out'
	st = psa_cipher_update(&op, plain, SENSOR_DATA_PACKET_SIZE,
						   out + NONCE_LEN,			// Output buffer starts after nonce
						   SENSOR_DATA_PACKET_SIZE, // Max capacity for ciphertext
						   &olen);
	if (st != PSA_SUCCESS)
	{
		LOG_ERR("Failed to update cipher: %d", st);
		ret = -EFAULT;
		goto cleanup_op;
	}
	ciphertext_len += olen;

	// Finish the operation (usually produces no more output for CTR)
	st = psa_cipher_finish(&op,
						   out + NONCE_LEN + ciphertext_len,		 // Where to write if any
						   SENSOR_DATA_PACKET_SIZE - ciphertext_len, // Remaining capacity
						   &olen);
	if (st != PSA_SUCCESS)
	{
		LOG_ERR("Failed to finish cipher op: %d", st);
		ret = -EFAULT;
		goto cleanup_op;
	}
	ciphertext_len += olen;

	// Now 'out' contains [ 8-byte nonce | 20-byte ciphertext ]

	// --- Verification ---
	if (ciphertext_len != SENSOR_DATA_PACKET_SIZE)
	{
		LOG_ERR("Ciphertext length mismatch: expected %d, got %u",
				SENSOR_DATA_PACKET_SIZE, (unsigned int)ciphertext_len);
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
	bool state_needs_update = false;					// Flag to reduce ADV updates

	uint32_t save_timer = k_uptime_get_32(); // Timer for saving nonce

	struct fs_file_t log_file;			  // File object for littlefs
	bool file_is_open = false;			  // Track if log file is currently open
	int log_write_count = 0;			  // Counter for periodic sync
	static bool log_storage_full = false; // Flag to indicate storage is full

	fs_file_t_init(&log_file); // Initialise file object structure

	while (1)
	{
		/* Get each message from the message queue */
		ret = k_msgq_get(&sensor_message_queue, &received_msg, K_FOREVER);
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

				// Build the 20-byte plain text
				prepare_packet(&current_sensor_state, manuf_plain, sizeof(manuf_plain));

				// Encrypt to 31-byte buffer
				ret = encrypt_sensor_block(manuf_plain, manuf_encrypted);
				if (ret != 0)
				{
					LOG_ERR("Encrypt failed - ADV not updated: %d", ret);
					state_needs_update = false;
					k_sleep(K_SECONDS(5));
					continue;
				}

				if (adv_running)
				{
					// Update the advertising data (non-blocking)
					ret = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
					if (ret && ret != -EALREADY)
					{
						LOG_WRN("Failed to update ADV data: %d", ret);
					}
				}

				if (k_uptime_get_32() - save_timer > NONCE_SAVE_INTERVAL)
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
						LOG_DBG("Away mode: Opening log file %s", LOG_FILE_PATH);

						int ret = fs_open(&log_file, LOG_FILE_PATH, FS_O_APPEND | FS_O_CREATE | FS_O_WRITE);

						if (ret < 0)
						{
							LOG_ERR("Failed to open log file %s: %d", LOG_FILE_PATH, ret);
							k_sleep(K_SECONDS(5));
							continue;
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
							LOG_DBG("Logged IMU data (%d bytes), write count: %d",
									(int)written, log_write_count);

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
						if (k_uptime_get_32() - last_full_log > NONCE_SAVE_INTERVAL)
						{
							LOG_WRN("Storage is full. No longer logging data.");
							last_full_log = k_uptime_get_32();
						}
						continue;
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
			}
			// Reset update flag after processing
			state_needs_update = false;
		}
		// Loop continues, blocking on k_msgq_get
	}
}

static inline bool adaptive_has_lock(void)
{
	bool learned;
	k_mutex_lock(&hb_lock, K_FOREVER);
	learned = (hb_period_ms != DEFAULT_HB_MS);
	k_mutex_unlock(&hb_lock);
	return learned; /* true ⇒ we already know the period */
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

	device_state_t initial_state = STATE_FIND_PERIOD;

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

	timed_scan_init();

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
	k_timer_init(&heartbeat_timeout_timer, (k_timer_expiry_t)heartbeat_timeout_expiry, NULL);
	k_timer_init(&battery_timer, (k_timer_expiry_t)battery_timer_expiry, NULL);
	k_timer_start(&battery_timer, BATTERY_READ_INTERVAL, BATTERY_READ_INTERVAL);

	// Initialise workqueue items
	k_work_init(&battery_timeout_work, battery_timeout_work_handler);
	k_work_init(&heartbeat_timeout_work, heartbeat_timeout_work_handler);
	k_work_init(&usb_connect_work, usb_connect_work_handler);
	k_work_init(&usb_disconnect_work, usb_disconnect_work_handler);
	k_work_init(&scan_found_ap_work, scan_found_ap_work_handler);

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
