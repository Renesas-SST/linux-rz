// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the AP1302 external camera ISP from ON Semiconductor
 *
 * Copyright (C) 2021, Witekio, Inc.
 * Copyright (C) 2021, Xilinx, Inc.
 * Copyright (C) 2021, Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 * Copyright (C) 2021-2022, IMD Technologies Ltd
 *
 */

#include "ap1302.h"

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/ktime.h>

static int ap1302_set_mipi_t3_clk(struct ap1302_device *ap1302)
{
	unsigned int mipi_t3, t_clk_post, t_clk_pre;
	int ret;

	/* Set the Tclk_post and Tclk_pre values */
	ret = ap1302_read(ap1302, AP1302_ADV_HINF_MIPI_T3, &mipi_t3);
	if (ret)
		return ret;

	/* Read Tclk post default setting and increment by 2 */
	t_clk_post = ((mipi_t3 & AP1302_TCLK_POST_MASK)
					>> AP1302_TCLK_POST_SHIFT) + 0x5;
	/* Read Tclk pre default setting and increment by 1 */
	t_clk_pre = ((mipi_t3 & AP1302_TCLK_PRE_MASK)
					>> AP1302_TCLK_PRE_SHIFT) + 0x1;

	mipi_t3 = ((mipi_t3 & ~(AP1302_TCLK_POST_MASK))
					& ~(AP1302_TCLK_PRE_MASK));
	mipi_t3 = (mipi_t3 | (t_clk_pre << AP1302_TCLK_PRE_SHIFT)
					| t_clk_post);

	/* Write MIPI_T3 register with updated Tclk_post and Tclk_pre values */
	return ap1302_write(ap1302, AP1302_ADV_HINF_MIPI_T3, mipi_t3, NULL);
}

int ap1302_request_firmware(struct ap1302_device *ap1302)
{
	static const char * const suffixes[] = {
		"",
		"_single",
		"_dual",
	};

	unsigned int num_sensors;
	unsigned int i;
	char name[64];
	int ret;

	for (i = 0, num_sensors = 0; i < ARRAY_SIZE(ap1302->sensors); ++i) {
		if (ap1302->sensors[i].present)
			num_sensors++;
	}

	ret = snprintf(name, sizeof(name), "ap1302_%s%s_fw.bin",
		       ap1302->sensor_info->name, suffixes[num_sensors]);
	if (ret >= sizeof(name)) {
		dev_err(ap1302->dev, "Firmware name too long\n");
		return -EINVAL;
	}

	dev_dbg(ap1302->dev, "Requesting firmware %s\n", name);

	ret = request_firmware(&ap1302->fw, name, ap1302->dev);
	if (ret) {
		dev_err(ap1302->dev, "Failed to request firmware: %d\n", ret);
		return ret;
	}

	return 0;
}

/*
 * ap1302_write_fw_window() - Write a piece of firmware to the AP1302
 * @win_pos: Firmware load window current position
 * @buf: Firmware data buffer
 * @len: Firmware data length
 *
 * The firmware is loaded through a window in the registers space. Writes are
 * sequential starting at address 0x8000, and must wrap around when reaching
 * 0x9fff. This function write the firmware data stored in @buf to the AP1302,
 * keeping track of the window position in the @win_pos argument.
 */
static int ap1302_write_fw_window(struct ap1302_device *ap1302, const u8 *buf,
				  u32 len, unsigned int *win_pos)
{
	while (len > 0) {
		unsigned int write_addr;
		unsigned int write_size;
		int ret;

		/*
		 * Write at most len bytes, from the current position to the
		 * end of the window.
		 */
		write_addr = *win_pos + AP1302_FW_WINDOW_OFFSET;
		write_size = min(len, AP1302_FW_WINDOW_SIZE - *win_pos);

		dev_dbg(ap1302->dev, "%u bytes remaining", len);

		if (ap1302->spi_dev) {
			ret = ap1302_spi_write_block(ap1302, write_addr, buf,
							write_size);
		} else {
			ret = regmap_raw_write(ap1302->regmap16, write_addr,
						buf, write_size);
		}

		if (ret) {
			dev_err(ap1302->dev, "ret = %d", ret);
			return ret;
		}

		buf += write_size;
		len -= write_size;

		*win_pos += write_size;
		if (*win_pos >= AP1302_FW_WINDOW_SIZE)
			*win_pos = 0;

		usleep_range(10000, 11000);
	}

	return 0;
}

int ap1302_load_firmware(struct ap1302_device *ap1302)
{
	unsigned int win_pos = 0;
	int ret;
	u32 bootdata_stage = 0;
	u32 checksum = 0;
	u64 start_time, stop_time, elapsed_time;
	int retries;

	dev_info(ap1302->dev, "Loading firmware");

	start_time = ktime_get_ns();

	ret = ap1302_write(ap1302, AP1302_SIPS_SLEW_CTRL, 0x14, NULL);
	if (ret)
		return ret;

	ret = ap1302_write(ap1302, AP1302_SYSTEM_FREQ_IN,
			   ap1302->system_freq_in, NULL);
	if (ret)
		return ret;

	ret = ap1302_write(ap1302, AP1302_HINF_MIPI_FREQ_TGT,
			   ap1302->hinf_mipi_freq_tgt, NULL);
	if (ret)
		return ret;

	ret = ap1302_write_fw_window(ap1302, ap1302->fw->data, ap1302->fw->size,
	                             &win_pos);
	if (ret)
		return ret;

	stop_time = ktime_get_ns();

	elapsed_time = (stop_time - start_time) / 1000000;

	dev_info(ap1302->dev, "Finished loading firmware; took %llu ms",
			elapsed_time);

	/*
	 * Write 0xffff to the BOOTDATA_STAGE register to indicate to the
	 * AP1302 that the whole bootdata content has been loaded.
	 */
	ret = ap1302_write(ap1302, AP1302_BOOTDATA_STAGE, 0xffff, NULL);
	if (ret)
		return ret;

	/* Wait for the AP1302_BOOTDATA_STAGE register to read 0xFFFF */
	retries = 500;
	bootdata_stage = 0;
	while (retries && (0xFFFF != bootdata_stage)) {
		ret = ap1302_read(ap1302, AP1302_BOOTDATA_STAGE,
		                  &bootdata_stage);
		if (ret)
			return ret;

		usleep_range(10000, 11000);
		retries--;
	}

	checksum = 0;
	while (retries && (0xFFFF != checksum)) {
		ret = ap1302_read(ap1302, AP1302_BOOTDATA_CHECKSUM, &checksum);
		if (ret)
			return ret;

		usleep_range(10000, 11000);
		retries--;
	}

	dev_info(ap1302->dev, "CRC matches expected value");

	if (0 == retries)
		return -EBUSY;

	/* The AP1302 starts outputting frames right after boot, stop it. */
	ret = ap1302_stall(ap1302, true);
	if (ret)
		return ret;

	/* Adjust MIPI TCLK timings */
	return ap1302_set_mipi_t3_clk(ap1302);
}
