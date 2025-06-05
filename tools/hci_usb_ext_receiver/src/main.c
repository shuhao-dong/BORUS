/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/gpio.h>

#define LED1_NODE DT_ALIAS(led1_green)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
#include <sample_usbd.h>

static int enable_usb_device_next(void)
{
	struct usbd_context *sample_usbd = sample_usbd_init_device(NULL);

	if (sample_usbd == NULL) {
		printk("Failed to initialize USB device");
		return -ENODEV;
	}

	return usbd_enable(sample_usbd);
}
#endif /* CONFIG_USB_DEVICE_STACK_NEXT */

int main(void)
{
	int ret;

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	ret = enable_usb_device_next();
#else
	ret = usb_enable(NULL);
#endif

	if (ret != 0) {
		printk("Failed to enable USB");
		return 0;
	}

	if (!gpio_is_ready_dt(&led)) {
		return 0; 
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0; 
	}

	for (int i = 0; i < 5; i++)
	{
		gpio_pin_toggle_dt(&led);
		k_msleep(500); 
	}
	
	printk("Bluetooth over USB sample\n");
	return 0;
}
