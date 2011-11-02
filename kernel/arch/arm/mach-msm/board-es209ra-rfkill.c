/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

static struct rfkill *bt_rfk;
static const char bt_name[] = "bt-power(rachel)";

static int bluetooth_set_power(void *data, bool blocked)
{
	int(*power_control)(int enable);

	power_control = data;
	return (power_control)(!blocked);
}

static struct rfkill_ops es209ra_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int es209ra_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool default_state = true;  /* off */

	bluetooth_set_power(pdev->dev.platform_data, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
				&es209ra_rfkill_ops, pdev->dev.platform_data);
	if (!bt_rfk) {
		rc = -ENOMEM;
		goto err_rfkill_alloc;
	}

	rfkill_set_states(bt_rfk, default_state, false);

	/* userspace cannot take exclusive control */

	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	platform_set_drvdata(pdev, bt_rfk);
	return 0;

err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill_alloc:
	return rc;
}

static int es209ra_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);
	return 0;
}

static struct platform_driver es209ra_rfkill_driver = {
	.probe = es209ra_rfkill_probe,
	.remove = es209ra_rfkill_remove,
	.driver = {
		.name = "bt_power",
		.owner = THIS_MODULE,
	},
};

static int __init es209ra_rfkill_init(void)
{
	return platform_driver_register(&es209ra_rfkill_driver);
}

static void __exit es209ra_rfkill_exit(void)
{
	platform_driver_unregister(&es209ra_rfkill_driver);
}

module_init(es209ra_rfkill_init);
module_exit(es209ra_rfkill_exit);
MODULE_DESCRIPTION("es209ra rfkill");
MODULE_AUTHOR("Goro Tsukiyama <goro.tsukiyama@gmail.com>");
MODULE_LICENSE("GPL");
