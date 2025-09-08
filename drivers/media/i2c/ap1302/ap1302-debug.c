// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the AP1302 external camera ISP from ON Semiconductor
 *
 * Copyright (C) 2021, Witekio, Inc.
 * Copyright (C) 2021, Xilinx, Inc.
 * Copyright (C) 2021, Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 * Copyright (C) 2021, IMD Technologies Ltd
 *
 */

#include "ap1302.h"

#include <linux/debugfs.h>
#include <linux/delay.h>

/**
 * @brief Wait for the AP1302 DMA engine to go idle
 * @param ap1302 AP1302 device
 * @return 0, -ETIMEDOUT or -EINVAL
 */
static int ap1302_dma_wait_idle(struct ap1302_device *ap1302)
{
	unsigned int i;
	u32 ctrl;
	int ret;

	for (i = 50; i > 0; i--) {
		ret = ap1302_read(ap1302, AP1302_DMA_CTRL, &ctrl);
		if (ret < 0)
			return ret;

		if ((ctrl & AP1302_DMA_CTRL_MODE_MASK) ==
		    AP1302_DMA_CTRL_MODE_IDLE)
			break;

		usleep_range(1000, 1500);
	}

	if (!i) {
		dev_err(ap1302->dev, "DMA timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int ap1302_sipm_read(struct ap1302_device *ap1302, unsigned int port,
			    u32 reg, u32 *val)
{
	unsigned int size = AP1302_REG_SIZE(reg);
	u32 src;
	int ret;

	if (size > 2)
		return -EINVAL;

	ret = ap1302_dma_wait_idle(ap1302);
	if (ret < 0)
		return ret;

	ap1302_write(ap1302, AP1302_DMA_SIZE, size, &ret);
	src = AP1302_DMA_SIP_SIPM(port)
	    | (size == 2 ? AP1302_DMA_SIP_DATA_16_BIT : 0)
	    | AP1302_DMA_SIP_ADDR_16_BIT
	    | AP1302_DMA_SIP_ID(ap1302->sensor_info->i2c_addr)
	    | AP1302_DMA_SIP_REG(AP1302_REG_ADDR(reg));
	ap1302_write(ap1302, AP1302_DMA_SRC, src, &ret);

	/*
	 * Use the AP1302_DMA_DST register as both the destination address, and
	 * the scratch pad to store the read value.
	 */
	ap1302_write(ap1302, AP1302_DMA_DST, AP1302_REG_ADDR(AP1302_DMA_DST),
		     &ret);

	ap1302_write(ap1302, AP1302_DMA_CTRL,
		     AP1302_DMA_CTRL_SCH_NORMAL |
		     AP1302_DMA_CTRL_DST_REG |
		     AP1302_DMA_CTRL_SRC_SIP |
		     AP1302_DMA_CTRL_MODE_COPY, &ret);
	if (ret < 0)
		return ret;

	ret = ap1302_dma_wait_idle(ap1302);
	if (ret < 0)
		return ret;

	ret = ap1302_read(ap1302, AP1302_DMA_DST, val);
	if (ret < 0)
		return ret;

	/*
	 * The value is stored in big-endian at the DMA_DST address. The regmap
	 * uses big-endian, so 8-bit values are stored in bits 31:24 and 16-bit
	 * values in bits 23:16.
	 */
	*val >>= 32 - size * 8;

	return 0;
}

static int ap1302_sipm_write(struct ap1302_device *ap1302, unsigned int port,
			     u32 reg, u32 val)
{
	unsigned int size = AP1302_REG_SIZE(reg);
	u32 dst;
	int ret;

	if (size > 2)
		return -EINVAL;

	ret = ap1302_dma_wait_idle(ap1302);
	if (ret < 0)
		return ret;

	ap1302_write(ap1302, AP1302_DMA_SIZE, size, &ret);

	/*
	 * Use the AP1302_DMA_SRC register as both the source address, and the
	 * scratch pad to store the write value.
	 *
	 * As the AP1302 uses big endian, to store the value at address DMA_SRC
	 * it must be written in the high order bits of the registers. However,
	 * 8-bit values seem to be incorrectly handled by the AP1302, which
	 * expects them to be stored at DMA_SRC + 1 instead of DMA_SRC. The
	 * value is thus unconditionally shifted by 16 bits, unlike for DMA
	 * reads.
	 */
	ap1302_write(ap1302, AP1302_DMA_SRC,
		     (val << 16) | AP1302_REG_ADDR(AP1302_DMA_SRC), &ret);
	if (ret < 0)
		return ret;

	dst = AP1302_DMA_SIP_SIPM(port)
	    | (size == 2 ? AP1302_DMA_SIP_DATA_16_BIT : 0)
	    | AP1302_DMA_SIP_ADDR_16_BIT
	    | AP1302_DMA_SIP_ID(ap1302->sensor_info->i2c_addr)
	    | AP1302_DMA_SIP_REG(AP1302_REG_ADDR(reg));
	ap1302_write(ap1302, AP1302_DMA_DST, dst, &ret);

	ap1302_write(ap1302, AP1302_DMA_CTRL,
		     AP1302_DMA_CTRL_SCH_NORMAL |
		     AP1302_DMA_CTRL_DST_SIP |
		     AP1302_DMA_CTRL_SRC_REG |
		     AP1302_DMA_CTRL_MODE_COPY, &ret);
	if (ret < 0)
		return ret;

	ret = ap1302_dma_wait_idle(ap1302);
	if (ret < 0)
		return ret;

	return 0;
}

/* -----------------------------------------------------------------------------
 * Debugfs
 */

static int ap1302_sipm_addr_get(void *arg, u64 *val)
{
	struct ap1302_device *ap1302 = arg;

	mutex_lock(&ap1302->debugfs.lock);
	*val = ap1302->debugfs.sipm_addr;
	mutex_unlock(&ap1302->debugfs.lock);

	return 0;
}

static int ap1302_sipm_addr_set(void *arg, u64 val)
{
	struct ap1302_device *ap1302 = arg;

	if (val & ~0x8700ffff) {
		return -EINVAL;
	}

	switch ((val >> 24) & 7) {
	case 1:
	case 2:
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&ap1302->debugfs.lock);
	ap1302->debugfs.sipm_addr = val;
	mutex_unlock(&ap1302->debugfs.lock);

	return 0;
}

static int ap1302_sipm_data_get(void *arg, u64 *val)
{
	struct ap1302_device *ap1302 = arg;
	u32 value;
	u32 addr;
	int ret;

	mutex_lock(&ap1302->debugfs.lock);

	addr = ap1302->debugfs.sipm_addr;
	if (!addr) {
		ret = -EINVAL;
		goto unlock;
	}

	ret = ap1302_sipm_read(ap1302, addr >> 30, addr & ~BIT(31),
			       &value);
	if (!ret)
		*val = value;

unlock:
	mutex_unlock(&ap1302->debugfs.lock);

	return ret;
}

static int ap1302_sipm_data_set(void *arg, u64 val)
{
	struct ap1302_device *ap1302 = arg;
	u32 addr;
	int ret;

	mutex_lock(&ap1302->debugfs.lock);

	addr = ap1302->debugfs.sipm_addr;
	if (!addr) {
		ret = -EINVAL;
		goto unlock;
	}

	ret = ap1302_sipm_write(ap1302, addr >> 30, addr & ~BIT(31),
				val);

unlock:
	mutex_unlock(&ap1302->debugfs.lock);

	return ret;
}

static int ap1302_isp_addr_get(void *arg, u64 *val)
{
	struct ap1302_device *ap1302 = arg;

	mutex_lock(&ap1302->debugfs.lock);
	*val = ap1302->debugfs.isp_addr;
	mutex_unlock(&ap1302->debugfs.lock);

	return 0;
}

static int ap1302_isp_addr_set(void *arg, u64 val)
{
	struct ap1302_device *ap1302 = arg;

	mutex_lock(&ap1302->debugfs.lock);
	ap1302->debugfs.isp_addr = val;
	mutex_unlock(&ap1302->debugfs.lock);

	return 0;
}

static int ap1302_isp_data_get(void *arg, u64 *val)
{
	struct ap1302_device *ap1302 = arg;
	u32 value;
	u32 addr;
	int ret;

	mutex_lock(&ap1302->debugfs.lock);

	addr = ap1302->debugfs.isp_addr;

	if (0 == AP1302_REG_SIZE(addr)) {
		addr =  AP1302_REG_16BIT(addr);
	}

	ret = ap1302_read(ap1302, addr, &value);
	if (!ret)
		*val = value;

	mutex_unlock(&ap1302->debugfs.lock);

	return ret;
}

static int ap1302_isp_data_set(void *arg, u64 val)
{
	struct ap1302_device *ap1302 = arg;
	u32 addr;
	int ret = 0;

	mutex_lock(&ap1302->debugfs.lock);

	addr = ap1302->debugfs.isp_addr;
	if (!addr) {
		ret = -EINVAL;
		goto unlock;
	}

	if (0 == AP1302_REG_SIZE(addr)) {
		addr =  AP1302_REG_16BIT(addr);
	}

	ret = ap1302_write(ap1302, addr, val, NULL);

unlock:
	mutex_unlock(&ap1302->debugfs.lock);

	return ret;
}

static int ap1302_mipi_tclk_post_get(void *arg, u64 *val)
{
	struct ap1302_device *ap1302 = arg;
	u32 value;
	int ret;

	mutex_lock(&ap1302->debugfs.lock);

	ret = ap1302_read(ap1302, AP1302_ADV_HINF_MIPI_T3, &value);
	if (!ret)
		*val = value & AP1302_TCLK_POST_MASK;

	mutex_unlock(&ap1302->debugfs.lock);

	return ret;
}

static int ap1302_mipi_tclk_post_set(void *arg, u64 val)
{
	struct ap1302_device *ap1302 = arg;
	u32 reg, reg_val;
	int ret;

	mutex_lock(&ap1302->debugfs.lock);
	ret = ap1302_read(ap1302, AP1302_ADV_HINF_MIPI_T3, &reg);
	if (ret < 0)
		goto unlock;

	reg_val = reg & ~(AP1302_TCLK_POST_MASK);
	reg_val = reg_val | val;
	ret = ap1302_write(ap1302, AP1302_ADV_HINF_MIPI_T3, reg_val, NULL);

unlock:
	mutex_unlock(&ap1302->debugfs.lock);

	return ret;
}

static int ap1302_mipi_tclk_pre_get(void *arg, u64 *val)
{
	struct ap1302_device *ap1302 = arg;
	u32 value;
	int ret;

	mutex_lock(&ap1302->debugfs.lock);

	ret = ap1302_read(ap1302, AP1302_ADV_HINF_MIPI_T3, &value);
	if (!ret)
		*val = (value & AP1302_TCLK_PRE_MASK) >> AP1302_TCLK_PRE_SHIFT;

	mutex_unlock(&ap1302->debugfs.lock);

	return ret;
}

static int ap1302_mipi_tclk_pre_set(void *arg, u64 val)
{
	struct ap1302_device *ap1302 = arg;
	u32 reg, reg_val;
	int ret;

	mutex_lock(&ap1302->debugfs.lock);
	ret = ap1302_read(ap1302, AP1302_ADV_HINF_MIPI_T3, &reg);
	if (ret < 0)
		goto unlock;

	reg_val = reg & ~(AP1302_TCLK_PRE_MASK);
	reg_val = reg_val | val << AP1302_TCLK_PRE_SHIFT;
	ret = ap1302_write(ap1302, AP1302_ADV_HINF_MIPI_T3, reg_val, NULL);

unlock:
	mutex_unlock(&ap1302->debugfs.lock);

	return ret;
}

/*
 * The sipm_addr and sipm_data attributes expose access to the sensor I2C bus.
 *
 * To read or write a register, sipm_addr has to first be written with the
 * register address. The address is a 32-bit integer formatted as follows.
 *
 * I000 0SSS 0000 0000 RRRR RRRR RRRR RRRR
 *
 * I: SIPM index (0 or 1)
 * S: Size (1: 8-bit, 2: 16-bit)
 * R: Register address (16-bit)
 *
 * The sipm_data attribute can then be read to read the register value, or
 * written to write it.
 */

DEFINE_DEBUGFS_ATTRIBUTE(ap1302_sipm_addr_fops, ap1302_sipm_addr_get,
			 ap1302_sipm_addr_set, "0x%08llx\n");
DEFINE_DEBUGFS_ATTRIBUTE(ap1302_sipm_data_fops, ap1302_sipm_data_get,
			 ap1302_sipm_data_set, "0x%08llx\n");

/*
 * The isp_addr and isp_data attributes allow for direct access to the ISP
 * registers.
 */

DEFINE_DEBUGFS_ATTRIBUTE(ap1302_isp_addr_fops, ap1302_isp_addr_get,
			 ap1302_isp_addr_set, "0x%08llx\n");
DEFINE_DEBUGFS_ATTRIBUTE(ap1302_isp_data_fops, ap1302_isp_data_get,
			 ap1302_isp_data_set, "0x%08llx\n");

/* The debugfs is to read and write mipi clk parameters tclk_post values */
DEFINE_DEBUGFS_ATTRIBUTE(ap1302_mipi_tclk_post_fops, ap1302_mipi_tclk_post_get,
			 ap1302_mipi_tclk_post_set, "0x%08llx\n");

/* The debugfs is to read and write mipi clk parameters and tclk_pre values */
DEFINE_DEBUGFS_ATTRIBUTE(ap1302_mipi_tclk_pre_fops, ap1302_mipi_tclk_pre_get,
			 ap1302_mipi_tclk_pre_set, "0x%08llx\n");

void ap1302_debugfs_init(struct ap1302_device *ap1302)
{
	struct dentry *dir;
	char name[16];

	mutex_init(&ap1302->debugfs.lock);

	snprintf(name, sizeof(name), "ap1302.%s", dev_name(ap1302->dev));

	dir = debugfs_create_dir(name, NULL);
	if (IS_ERR(dir))
		return;

	ap1302->debugfs.dir = dir;

	debugfs_create_file_unsafe("sipm_addr", 0600, ap1302->debugfs.dir,
				   ap1302, &ap1302_sipm_addr_fops);
	debugfs_create_file_unsafe("sipm_data", 0600, ap1302->debugfs.dir,
				   ap1302, &ap1302_sipm_data_fops);

	debugfs_create_file_unsafe("isp_addr", 0600, ap1302->debugfs.dir,
				   ap1302, &ap1302_isp_addr_fops);
	debugfs_create_file_unsafe("isp_data", 0600, ap1302->debugfs.dir,
				   ap1302, &ap1302_isp_data_fops);

	debugfs_create_file_unsafe("mipi_tclk_post", 0600, ap1302->debugfs.dir,
				   ap1302, &ap1302_mipi_tclk_post_fops);
	debugfs_create_file_unsafe("mipi_tclk_pre", 0600, ap1302->debugfs.dir,
				   ap1302, &ap1302_mipi_tclk_pre_fops);
}

void ap1302_debugfs_cleanup(struct ap1302_device *ap1302)
{
	debugfs_remove_recursive(ap1302->debugfs.dir);
	mutex_destroy(&ap1302->debugfs.lock);
}
