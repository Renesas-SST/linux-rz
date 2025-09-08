// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the AP1302 external camera ISP from ON Semiconductor
 *
 * Copyright (C) 2022, IMD Technologies Ltd
 *
 */

#include <linux/module.h>
#include <linux/spi/spi.h>

#include "ap1302.h"

#define AP1302_SPI_DRV_NAME "ap1302-spi"

static const struct of_device_id ap1302_spi_ids[] = {
	{ .compatible = "onnn,ap1302" },
	{ }
};

MODULE_DEVICE_TABLE(of, ap1302_spi_ids);

int ap1302_spi_init(struct ap1302_device *ap1302)
{
	int ret = 0;

	ap1302_write(ap1302, AP1302_ADV_SYS_RST_EN_0, 0x0000003D, &ret);
	ap1302_write(ap1302, AP1302_ADV_SYS_CLK_EN_0, 0x0000001C, &ret);
	ap1302_write(ap1302, AP1302_ADV_SYS_DIV_EN, 0x0000039D, &ret);
	ap1302_write(ap1302, AP1302_ADV_SYS_STBY_GPIO_OSEL, 0x03333001, &ret);
	ap1302_write(ap1302, AP1302_ADV_SYS_STBY_GPIO_IN_MASK, 0x19919111, &ret);

	return ret;
}

static int __ap1302_spi_write(struct spi_device *spi_dev, u32 reg, u32 val)
{
	unsigned int size = AP1302_REG_SIZE(reg);
	u16 addr = AP1302_REG_ADDR(reg);
	int ret;

	u8 tx_data[8];

	tx_data[0] = (addr >> 8) & 0xFF;
	tx_data[1] = (addr & 0xFF);
	tx_data[2] = 0x80;

	switch (size) {
	case 2:
		tx_data[3] = (val >> 8) & 0xFF;
		tx_data[4] = (val & 0xFF);
		break;
	case 4:
		tx_data[3] = (val >> 24) & 0xFF;
		tx_data[4] = (val >> 16) & 0xFF;
		tx_data[5] = (val >> 8) & 0xFF;
		tx_data[6] = (val & 0xFF);
		break;
	default:
		return -EINVAL;
	}

	ret = spi_write(spi_dev, tx_data, size + 3);

	if (ret) {
		dev_err(&spi_dev->dev, "%s: register 0x%04x %s failed: %d\n",
			__func__, addr, "write", ret);
	}

	return ret;
}

int ap1302_spi_write(struct ap1302_device *ap1302, u32 reg, u32 val, int *err)
{
	struct spi_device *spi_dev = ap1302->spi_dev;
	int ret;

	u32 page = AP1302_REG_PAGE(reg);

	if (!spi_dev)
		return -ENODEV;

	if (err && *err)
		return *err;

	if (page) {
		if (ap1302->reg_page != page) {
			ret = __ap1302_spi_write(spi_dev, AP1302_ADVANCED_BASE,
						 page);
			if (ret < 0)
				goto done;

			ap1302->reg_page = page;
		}

		reg &= ~AP1302_REG_PAGE_MASK;
		reg += AP1302_REG_ADV_START;
	}

	ret = __ap1302_spi_write(spi_dev, reg, val);

done:
	if (err && ret)
		*err = ret;

	return ret;
}

int ap1302_spi_write_block(struct ap1302_device *ap1302, u32 addr, const u8 *data, u32 len)
{
	struct spi_device *spi_dev = ap1302->spi_dev;
	int ret;

	struct spi_transfer xfers[2];
	u8 tx_data[3];

	if (!spi_dev)
		return -ENODEV;

	tx_data[0] = (addr >> 8) & 0xFF;
	tx_data[1] = addr & 0xFF;
	tx_data[2] = 0x80;

	memset(xfers, 0, sizeof(xfers));

	xfers[0].tx_buf = tx_data;
	xfers[0].len = 3;

	xfers[1].tx_buf = data;
	xfers[1].len = len;

	ret = spi_sync_transfer(spi_dev, xfers, 2);

	if (ret < 0) {
		dev_err(&spi_dev->dev, "%s: block write failed %d", __func__, ret);
	}

	return ret;
}

int ap1302_spi_read(struct ap1302_device *ap1302, u32 reg, u32 *val)
{
	struct spi_device *spi_dev = ap1302->spi_dev;
	int ret, i;

	int size = AP1302_REG_SIZE(reg);
	u32 page = AP1302_REG_PAGE(reg);
	u16 addr;

	u8 tx_data[3], rx_data[4];

	if (!spi_dev)
		return -ENODEV;

	if (page) {
		if (ap1302->reg_page != page) {
			ret = __ap1302_spi_write(spi_dev, AP1302_ADVANCED_BASE,
						 page);
			if (ret < 0)
				return ret;

			ap1302->reg_page = page;
		}

		reg &= ~AP1302_REG_PAGE_MASK;
		reg += AP1302_REG_ADV_START;
	}

	addr = AP1302_REG_ADDR(reg);

	tx_data[0] = (addr >> 8) & 0xFF;
	tx_data[1] = (addr & 0xFF);
	tx_data[2] = 0;

	memset(rx_data, 0, sizeof(rx_data));

	ret = spi_write_then_read(spi_dev, tx_data, 3, rx_data, size);
	if (ret < 0) {
		dev_err(&spi_dev->dev, "%s spi_sync failed %d\n", __func__, ret);
	} else {

		*val = 0;

		for (i = 0; i < size; i++) {
			*val <<= 8;
			*val |= rx_data[i];
		}
	}

	return ret;
}

static int ap1302_spi_probe(struct spi_device *spi)
{
	int ret;
	struct ap1302_device *ap1302 = container_of(spi->dev.driver,
						    struct ap1302_device,
						    spidrv.driver);

	spi->bits_per_word = 8;
	spi->max_speed_hz = 3000000;

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "spi_setup() failed");
		return ret;
	}

	mutex_lock(&ap1302->lock);
	ap1302->spi_dev = spi;
	mutex_unlock(&ap1302->lock);

	dev_info(&spi->dev, "driver probed successfully");
	return 0;
}

static void ap1302_spi_remove(struct spi_device *spi)
{
	return;
}

int ap1302_register_spi_driver(struct ap1302_device *ap1302)
{
	struct spi_driver *spidrv = &ap1302->spidrv;

	spidrv->remove = ap1302_spi_remove;
	spidrv->probe = ap1302_spi_probe;
	spidrv->driver.name = AP1302_SPI_DRV_NAME;
	spidrv->driver.of_match_table = ap1302_spi_ids;

	return spi_register_driver(spidrv);
}

void ap1302_unregister_spi_driver(struct ap1302_device *ap1302)
{
	spi_unregister_driver(&ap1302->spidrv);
}
