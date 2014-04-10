/*
 *  HID driver for Amazon.com game controllers
 *  Copyright 2012-2013 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>

#include "hid-ids.h"

static const struct hid_device_id amazon_devices[] = {
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_AMAZON,
	  USB_DEVICE_ID_AMAZON_GAMEPAD_BLUETOOTH)},
	{ }
};
MODULE_DEVICE_TABLE(hid, amazon_devices);

static int amazon_input_register(struct hid_device *hdev,
	struct hid_input *hidinput)
{
	input_set_capability(hidinput->input, EV_LED, LED_NUML);
	input_set_capability(hidinput->input, EV_LED, LED_CAPSL);
	input_set_capability(hidinput->input, EV_LED, LED_SCROLLL);
	input_set_capability(hidinput->input, EV_LED, LED_COMPOSE);
	return 0;
}

static struct hid_driver amazon_driver = {
	.name = "amazon-gamepad",
	.id_table = amazon_devices,
	.input_register = amazon_input_register,
};

static int __init amazon_init(void)
{
	return hid_register_driver(&amazon_driver);
}

static void __exit amazon_exit(void)
{
	hid_unregister_driver(&amazon_driver);
}

module_init(amazon_init);
module_exit(amazon_exit);
MODULE_LICENSE("Proprietary");
