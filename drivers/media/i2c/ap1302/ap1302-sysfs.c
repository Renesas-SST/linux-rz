/**
 * @file ap1302-sysfs.c
 * @copyright Copyright (c) 2022 IMD Technologies. All rights reserved.
 * @author: Paul Thomson
 */

#include <linux/device.h>
#include <linux/kernel.h>

#include "ap1302.h"

static ssize_t gpio_in_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct ap1302_device *ap1302;
	u32 val;

	ap1302 = dev_get_drvdata(dev);

	ap1302_read(ap1302, AP1302_ADV_GPIO_DI, &val);

	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", val);
}

static DEVICE_ATTR_RO(gpio_in);

static ssize_t face_detection_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct ap1302_device *ap1302;
	u32 val;
	bool enabled;

	ap1302 = dev_get_drvdata(dev);

	ap1302_read(ap1302, AP1302_PREVIEW_ENABLE, &val);

	enabled = (val & 0x0110) != 0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", enabled);
}

static ssize_t face_detection_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct ap1302_device *ap1302;
	int status, ret;
	u32 val;

	ret = kstrtoint(buf, 0, &status);
	if (ret < 0)
		return ret;

	ap1302 = dev_get_drvdata(dev);
	ap1302_read(ap1302, AP1302_PREVIEW_ENABLE, &val);

	if (status)
		val |= 0x0110;
	else
		val &= ~0x0110;

	ap1302_write(ap1302, AP1302_PREVIEW_ENABLE, val, NULL);

	return len;
}

static DEVICE_ATTR_RW(face_detection);

int create_sysfs_attributes(struct ap1302_device *ap1302)
{
	dev_set_drvdata(ap1302->dev, ap1302);
	device_create_file(ap1302->dev, &dev_attr_gpio_in);
	device_create_file(ap1302->dev, &dev_attr_face_detection);

	return 0;
}

void remove_sysfs_attributes(struct ap1302_device *ap1302)
{
	device_remove_file(ap1302->dev, &dev_attr_gpio_in);
	device_remove_file(ap1302->dev, &dev_attr_face_detection);
}
