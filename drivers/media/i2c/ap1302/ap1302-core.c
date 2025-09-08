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

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>

#include <media/media-entity.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>

#define MAX_FW_LOAD_RETRIES 5

/**
 * @note YUYV8_1X16 is the only fully supported format, and is used for video
 * streaming and full size snapshots; the other formats are included to satisfy
 * the caps negotiation process. The ISI should be configured to override the
 * source format to ensure YUYV8_1X16 is always selected - conversion to RGB
 * and BGR will be performed by the ISI.
 */
static const struct ap1302_format_info supported_video_formats[] = {
	{
		.code = MEDIA_BUS_FMT_YUYV8_1X16,
		.out_fmt = AP1302_PREVIEW_OUT_FMT_FT_YUV_JFIF
			 | AP1302_PREVIEW_OUT_FMT_FST_YUV_422,
	}, {
		.code = MEDIA_BUS_FMT_RGB888_1X24,
		.out_fmt = AP1302_PREVIEW_OUT_FMT_FT_RGB
			 | AP1302_PREVIEW_OUT_FMT_FST_RGB_888
	}, {
		.code = MEDIA_BUS_FMT_BGR888_1X24,
		.out_fmt = AP1302_PREVIEW_OUT_FMT_FT_RGB
			 | AP1302_PREVIEW_OUT_FMT_FST_RGB_888,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.out_fmt = AP1302_PREVIEW_OUT_FMT_FT_RAW8
			 | AP1302_PREVIEW_OUT_FMT_FST_RAW_SENSOR, 
	},
};

/* Module parameters */

static bool disable_power_down;
module_param(disable_power_down, bool, 0644);
MODULE_PARM_DESC(disable_power_down, "Disable the power-down functionality");

static bool retry_firmware;
module_param(retry_firmware, bool, 0644);
MODULE_PARM_DESC(retry_firmware, "If set, only a single AP1302 ISP is present");

static unsigned int system_freq_in_hz = 24000000;
module_param(system_freq_in_hz, uint, 0644);
MODULE_PARM_DESC(system_freq_in_hz, "Ref clock for AP1302 in Hz.");

static unsigned int hinf_mipi_freq_tgt_hz = 1104000000;
module_param(hinf_mipi_freq_tgt_hz, uint, 0644);
MODULE_PARM_DESC(hinf_mipi_freq_tgt_hz, "MIPI clock for AP1302 in Hz.");

/*
 *
 * Sensor Info
 *
 */

static const struct ap1302_sensor_info ap1302_sensor_info[] = {
	{
		.model = "onnn,ar1335",
		.name = "ar1335",
		.i2c_addr = 0x36,
		.resolution = { 4208, 3120 },
		.format = MEDIA_BUS_FMT_SGRBG10_1X10,
	},
};

static const struct ap1302_mode ap1302_other_modes[] = {
		{
		.resolution = { 3840, 2160 },
		.min_frame_interval = { 1, 20 }
	},	{
		.resolution = { 4096, 3072 },
		.min_frame_interval = { 1, 20 }
	},	{
		.resolution = { 1920, 1080 },
		.min_frame_interval = { 1, 30 }
	},	{
		.resolution = { 1440, 1080 },
		.min_frame_interval = { 1, 30 }
	},	{
		.resolution = { 1280, 720 },
		.min_frame_interval = { 1, 60 }
	},
};

static const struct v4l2_fract ap1302_frame_intervals[] = {
	{ 1, 5 },
	{ 1, 8 },
	{ 1, 10 },
	{ 1, 12 },
	{ 1, 15 },
	{ 1, 20 },
	{ 1, 25 },
	{ 1, 30 },
	{ 1, 60 },
};

static const struct ap1302_sensor_mode ap1302_sensor_modes[] = {
	{
		.mode = 4,
		.resolution = { 2104, 780 },
		.min_frame_interval = { 1, 120 }
	}, {
		.mode = 2,
		.resolution = { 2104, 1560 },
		.min_frame_interval = { 1, 60 }
	}, {
		.mode = 0,
		.resolution = { 4208, 3120 },
		.min_frame_interval = { 1, 12 }
	},
};

/*
 *
 * Register Access
 *
 */

static const struct regmap_config ap1302_reg16_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 2,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_config ap1302_reg32_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.reg_stride = 4,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.cache_type = REGCACHE_NONE,
};

static int __ap1302_write(struct ap1302_device *ap1302, u32 reg, u32 val)
{
	unsigned int size = AP1302_REG_SIZE(reg);
	u16 addr = AP1302_REG_ADDR(reg);
	int ret;

	switch (size) {
	case 2:
		ret = regmap_write(ap1302->regmap16, addr, val);
		break;
	case 4:
		ret = regmap_write(ap1302->regmap32, addr, val);
		break;
	default:
		return -EINVAL;
	}

	if (ret) {
		dev_err(ap1302->dev, "%s: register 0x%04x %s failed: %d\n",
			__func__, addr, "write", ret);
		return ret;
	}

	return 0;
}

int ap1302_write(struct ap1302_device *ap1302, u32 reg, u32 val, int *err)
{
	u32 page = AP1302_REG_PAGE(reg);
	int ret;

	if (err && *err)
		return *err;

	if (page) {
		if (ap1302->reg_page != page) {
			ret = __ap1302_write(ap1302, AP1302_ADVANCED_BASE,
					     page);
			if (ret < 0)
				goto done;

			ap1302->reg_page = page;
		}

		reg &= ~AP1302_REG_PAGE_MASK;
		reg += AP1302_REG_ADV_START;
	}

	ret = __ap1302_write(ap1302, reg, val);

done:
	if (err && ret)
		*err = ret;

	return ret;
}

static int __ap1302_read(struct ap1302_device *ap1302, u32 reg, u32 *val)
{
	unsigned int size = AP1302_REG_SIZE(reg);
	u16 addr = AP1302_REG_ADDR(reg);
	int ret;

	switch (size) {
	case 2:
		ret = regmap_read(ap1302->regmap16, addr, val);
		break;
	case 4:
		ret = regmap_read(ap1302->regmap32, addr, val);
		break;
	default:
		return -EINVAL;
	}

	if (ret) {
		dev_err(ap1302->dev, "%s: register 0x%04x %s failed: %d\n",
			__func__, addr, "read", ret);
		return ret;
	}

	dev_dbg(ap1302->dev, "%s: R0x%04x = 0x%0*x\n", __func__,
		addr, size * 2, *val);

	return 0;
}

int ap1302_read(struct ap1302_device *ap1302, u32 reg, u32 *val)
{
	u32 page = AP1302_REG_PAGE(reg);
	int ret;

	if (page) {
		if (ap1302->reg_page != page) {
			ret = __ap1302_write(ap1302, AP1302_ADVANCED_BASE,
					     page);
			if (ret < 0)
				return ret;

			ap1302->reg_page = page;
		}

		reg &= ~AP1302_REG_PAGE_MASK;
		reg += AP1302_REG_ADV_START;
	}

	return __ap1302_read(ap1302, reg, val);
}

/*
 *
 * Power Handling
 *
 */

static int ap1302_power_on(struct ap1302_device *ap1302)
{
	int ret;

	usleep_range(2000, 3000);

	/* 0. RESET was asserted when getting the GPIO. */

	/* 1. Assert STANDBY. */
	if (ap1302->standby_gpio) {
		gpiod_set_value_cansleep(ap1302->standby_gpio, 1);
		usleep_range(500, 1000);
	}

	/* 2. Power up the regulators. To be implemented. */
	if (ap1302->power_gpio) {
		gpiod_set_value_cansleep(ap1302->power_gpio, 1);
	}

	/* 3. De-assert STANDBY. */
	if (ap1302->standby_gpio) {
		gpiod_set_value_cansleep(ap1302->standby_gpio, 0);
		usleep_range(500, 1000);
	}

	/* 4. Turn the clock on. */
	ret = clk_prepare_enable(ap1302->clock);

	if (ret < 0) {
		dev_err(ap1302->dev, "Failed to enable clock: %d\n", ret);
		return ret;
	}

	/* 5. De-assert RESET. */
	if (ap1302->reset_gpio) {
		gpiod_set_value_cansleep(ap1302->reset_gpio, 0);
	}

	/*
	 * 6. Wait for the AP1302 to initialize. The datasheet doesn't specify
	 * how long this takes.
	 */
	usleep_range(10000, 11000);

	return 0;
}

static void ap1302_power_off(struct ap1302_device *ap1302)
{
	if (disable_power_down) {
		dev_warn(ap1302->dev, "Power control disabled");
	} else {
		/* 1. Assert RESET. */
		if (ap1302->reset_gpio) {
			gpiod_set_value_cansleep(ap1302->reset_gpio, 1);
		}

		/* 2. Turn the clock off. */
		clk_disable_unprepare(ap1302->clock);

		/* 3. Assert STANDBY. */
		if (ap1302->standby_gpio) {
			gpiod_set_value_cansleep(ap1302->standby_gpio, 1);
			usleep_range(500, 1000);
		}

		/* 4. Power down the regulators. To be implemented. */
		if (ap1302->power_gpio) {
			gpiod_set_value_cansleep(ap1302->power_gpio, 0);
		}

		/* 5. De-assert STANDBY. */
		if (ap1302->standby_gpio) {
			usleep_range(500, 1000);
			gpiod_set_value_cansleep(ap1302->standby_gpio, 0);
		}
	}
}

/*
 *
 * Hardware Configuration
 *
 */

static int ap1302_dump_console(struct ap1302_device *ap1302)
{
	u8 *buffer;
	u8 *endp;
	u8 *p;
	int ret;

	buffer = kmalloc(AP1302_CON_BUF_SIZE + 1, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	ret = regmap_raw_read(ap1302->regmap16, AP1302_CON_BUF(0), buffer,
			      AP1302_CON_BUF_SIZE);
	if (ret < 0) {
		dev_err(ap1302->dev, "Failed to read console buffer: %d\n",
			ret);
		goto done;
	}

	print_hex_dump(KERN_INFO, "console ", DUMP_PREFIX_OFFSET, 16, 1, buffer,
		       AP1302_CON_BUF_SIZE, true);

	buffer[AP1302_CON_BUF_SIZE] = '\0';

	for (p = buffer; p < buffer + AP1302_CON_BUF_SIZE && *p; p = endp + 1) {
		endp = strchrnul(p, '\n');
		*endp = '\0';

		pr_info("console %s\n", p);
	}

	ret = 0;

done:
	kfree(buffer);
	return ret;
}

static int ap1302_configure(struct ap1302_device *ap1302)
{
	const struct ap1302_format *format = &ap1302->formats[AP1302_PAD_SOURCE];
	unsigned int data_lanes = ap1302->bus_cfg.bus.mipi_csi2.num_data_lanes;
	int ret = 0;

	ap1302_write(ap1302, AP1302_SYS_START,
		     AP1302_SYS_START_STALL_MODE_DISABLED, &ret);

	ap1302_write(ap1302, AP1302_PREVIEW_HINF_CTRL,
		     AP1302_PREVIEW_HINF_CTRL_SPOOF |
		     AP1302_PREVIEW_HINF_CTRL_MIPI_LANES(data_lanes), &ret);

	ap1302_write(ap1302, AP1302_PREVIEW_WIDTH, format->format.width, &ret);
	ap1302_write(ap1302, AP1302_PREVIEW_HEIGHT, format->format.height, &ret);
	ap1302_write(ap1302, AP1302_PREVIEW_OUT_FMT, format->info->out_fmt, &ret);
	if (ret < 0)
		return ret;

	__v4l2_ctrl_handler_setup(&ap1302->ctrls);

	return 0;
}

int ap1302_stall(struct ap1302_device *ap1302, bool stall)
{
	int i, ret = 0;
	u32 sys_start = 0;

	dev_dbg(ap1302->dev, "stream %s", stall ? "stopped" : "started");

	if (stall) {
		ap1302_write(ap1302, AP1302_SYS_START,
			     AP1302_SYS_START_STALL_EN |
			     AP1302_SYS_START_STALL_MODE_DISABLED, &ret);
		if (ret < 0)
			return ret;

		for (i = 0; i < 500; i++) {

			ap1302_read(ap1302, AP1302_SYS_START, &sys_start);

			if (sys_start & AP1302_SYS_START_STALL_STATUS) {
				dev_dbg(ap1302->dev, "%s i = %d %x %lx",
					__func__, i, sys_start,
					sys_start & AP1302_SYS_START_STALL_STATUS);
				break;
			}

			usleep_range(1000, 1500);
		}

		if (ret < 0)
			return ret;

		ap1302->streaming = false;
		return 0;
	} else {
		ap1302->streaming = true;
		ret = ap1302_write(ap1302, AP1302_SYS_START,
				   AP1302_SYS_START_STALL_STATUS |
				   AP1302_SYS_START_STALL_EN |
				   AP1302_SYS_START_STALL_MODE_DISABLED, NULL);

		for (i = 0; i < 500; i++) {

			ap1302_read(ap1302, AP1302_SYS_START, &sys_start);

			if (0 == (sys_start & AP1302_SYS_START_STALL_STATUS)) {
				dev_dbg(ap1302->dev, "%s i = %d %x %lx",
					__func__, i, sys_start,
					sys_start & AP1302_SYS_START_STALL_STATUS);
				break;
			}

			usleep_range(1000, 1500);
		}

		return ret;
	}
}

/*
 *
 * V4L2 Subdev Operations
 *
 */

static struct v4l2_mbus_framefmt *
ap1302_get_pad_format(struct ap1302_device *ap1302,
		      struct v4l2_subdev_state *cfg,
		      unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_format(cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ap1302->formats[pad].format;
	default:
		return NULL;
	}
}

static int ap1302_init_state(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *cfg)
{
	u32 which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	struct ap1302_device *ap1302 = to_ap1302(sd);
	const struct ap1302_sensor_info *info = ap1302->sensor_info;
	unsigned int pad;

	for (pad = 0; pad < ARRAY_SIZE(ap1302->formats); ++pad) {
		struct v4l2_mbus_framefmt *format =
			ap1302_get_pad_format(ap1302, cfg, pad, which);

		format->width = info->resolution.width;
		format->height = info->resolution.height;

		/*
		 * The source pad combines images side by side in multi-sensor
		 * setup.
		 */
		if (pad == AP1302_PAD_SOURCE) {
			format->code = ap1302->formats[pad].info->code;
		} else {
			format->code = info->format;
		}

		format->field = V4L2_FIELD_NONE;
		format->colorspace = V4L2_COLORSPACE_SRGB;
	}

	return 0;
}

static int ap1302_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);

	dev_dbg(sd->dev, "ap1302_enum_mbus_code: index = %x\n", code->index);

	if (code->pad != AP1302_PAD_SOURCE) {
		/*
		 * On the sink pads, only the format produced by the sensor is
		 * supported.
		 */
		if (code->index)
			return -EINVAL;

		code->code = ap1302->sensor_info->format;
	} else {
		/* On the source pad, multiple formats are supported. */
		if (code->index >= ARRAY_SIZE(supported_video_formats))
			return -EINVAL;

		code->code = supported_video_formats[code->index].code;
	}

	return 0;
}

static int ap1302_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	unsigned int i;

	dev_dbg(sd->dev, "ap1302_enum_frame_size: index = %u", fse->index);
	dev_dbg(sd->dev, "ap1302_enum_frame_size: fse->code = %x", fse->code);

	if (fse->pad != AP1302_PAD_SOURCE) {

		/*
		 * On the sink pads, only the size produced by the sensor is
		 * supported.
		 */
		if (fse->code != ap1302->sensor_info->format)
			return -EINVAL;

		fse->min_width = ap1302->sensor_info->resolution.width;
		fse->min_height = ap1302->sensor_info->resolution.height;
		fse->max_width = ap1302->sensor_info->resolution.width;
		fse->max_height = ap1302->sensor_info->resolution.height;
	} else {

		for (i = 0; i < ARRAY_SIZE(supported_video_formats); i++) {
			if (supported_video_formats[i].code == fse->code) {
				dev_dbg(sd->dev, "ap1302_enum_frame_size: supported format = %u", i);
				break;
			}
		}

		if (i >= ARRAY_SIZE(supported_video_formats)) {
			dev_dbg(sd->dev, "ap1302_enum_frame_size: unsupported format");
			return -EINVAL;
		}

		if (fse->index < ARRAY_SIZE(ap1302_other_modes)) {
			fse->min_width = fse->max_width = ap1302_other_modes[fse->index].resolution.width;
			fse->min_height = fse->max_height = ap1302_other_modes[fse->index].resolution.height;
		} else {
			dev_dbg(sd->dev, "ap1302_enum_frame_size: unsupported frame size");
			return -EINVAL;
		}
	}

	return 0;
}

static int ap1302_enum_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_state *cfg,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	unsigned int i;

	dev_dbg(sd->dev, "ap1302_enum_frame_interval: index = %u\n", fie->index);

	for (i = 0; i < ARRAY_SIZE(supported_video_formats); i++) {
		if (supported_video_formats[i].code == fie->code) {
			dev_dbg(sd->dev, "ap1302_enum_frame_interval: supported format = %u\n", i);
			break;
		}
	}

	if (i >= ARRAY_SIZE(supported_video_formats)) {
		dev_dbg(sd->dev, "ap1302_enum_frame_interval: *** unsupported format ***\n");
		return -EINVAL;
	}

	if (fie->index < ARRAY_SIZE(ap1302_frame_intervals)) {
		for (i = 0; i < ARRAY_SIZE(ap1302_other_modes); i++) {
			if ((fie->width == ap1302_other_modes[i].resolution.width) &&
			    (fie->height == ap1302_other_modes[i].resolution.height)) {
				if (ap1302_frame_intervals[fie->index].denominator <= ap1302_other_modes[i].min_frame_interval.denominator) {
					fie->interval = ap1302_frame_intervals[fie->index];
					break;
				} else {
					return -EINVAL;
				}

			} else {
				continue;
			}
		}

	} else {
		return -EINVAL;
	}

	return 0;
}

static int ap1302_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	const struct v4l2_mbus_framefmt *format;

	format = ap1302_get_pad_format(ap1302, cfg, fmt->pad, fmt->which);

	mutex_lock(&ap1302->lock);
	fmt->format = *format;
	mutex_unlock(&ap1302->lock);

	return 0;
}

static int ap1302_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	const struct ap1302_format_info *info = NULL;
	struct v4l2_mbus_framefmt *format;
	unsigned int i;
	bool valid_frame_size = false;

	/* Formats on the sink pads can't be changed. */
	if (fmt->pad != AP1302_PAD_SOURCE)
		return ap1302_get_fmt(sd, cfg, fmt);

	/* Access the v4l2_mbus_framefmt structure for the source pad */
	format = ap1302_get_pad_format(ap1302, cfg, fmt->pad, fmt->which);

	/* Validate the media bus code */
	for (i = 0; i < ARRAY_SIZE(supported_video_formats); i++) {
		if (supported_video_formats[i].code == fmt->format.code) {
			info = &supported_video_formats[i];
			break;
		}
	}

	if (!info) {
		dev_err(sd->dev, "%s: invalid format\n", __func__);
		return -EINVAL;
	}

	/* Validate the frame size against the media bus code */
	for (i = 0; i < ARRAY_SIZE(ap1302_other_modes); i++) {
		if ((fmt->format.width == ap1302_other_modes[i].resolution.width) &&
		    (fmt->format.height == ap1302_other_modes[i].resolution.height)) {
			valid_frame_size = true;
			break;
		}
	}

	if (!valid_frame_size) {
		dev_err(sd->dev, "%s: invalid frame size\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&ap1302->lock);

	/* Update the AP1302 configuration with the new resolution and code */
	format->width = fmt->format.width;
	format->height = fmt->format.height;
	format->code = info->code;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		ap1302->formats[fmt->pad].info = info;

	/* Set the sensor mode, based on the resolution */
	for (i = 0; i < ARRAY_SIZE(ap1302_sensor_modes); i++) {
		if ((format->width <= ap1302_sensor_modes[i].resolution.width) &&
		    (format->height <= ap1302_sensor_modes[i].resolution.height)) {
			dev_dbg(sd->dev, "%s: setting sensor to mode %u\n", __func__, ap1302_sensor_modes[i].mode);
			ap1302_write(ap1302, AP1302_PREVIEW_SENSOR_MODE, ap1302_sensor_modes[i].mode, NULL);
			break;
		}
	}

	mutex_unlock(&ap1302->lock);

	fmt->format = *format;

	return 0;
}

static int ap1302_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	const struct ap1302_size *resolution = &ap1302->sensor_info->resolution;

	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = resolution->width;
		sel->r.height = resolution->height;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int ap1302_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);

	fi->interval = ap1302->frame_interval;

	return 0;
}

static int ap1302_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_interval *fi)
{
	u32 numerator = fi->interval.numerator;
	u32 denominator = fi->interval.denominator;
	u32 val;
	u32 et;

	// TODO: restrict frame interval based on current sensor mode

	struct ap1302_device *ap1302 = to_ap1302(sd);

	dev_dbg(sd->dev, "%s: %d/%d\n", __func__, numerator, denominator);

	ap1302->frame_interval = fi->interval;

	val = denominator << 8;
	val /= numerator;
	ap1302_write(ap1302, AP1302_PREVIEW_MAX_FPS, val, NULL);

	et = numerator * 1000000;
	et /= denominator;

	ap1302_write(ap1302, AP1302_PREVIEW_AE_UPPER_ET, et, NULL);
	ap1302_write(ap1302, AP1302_PREVIEW_AE_MAX_ET, et, NULL);

	return 0;
}

static int ap1302_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct ap1302_device *ap1302 = to_ap1302(sd);

	dev_dbg(sd->dev, "%s: stream", enable ? "enable" : "disable");

	mutex_lock(&ap1302->lock);

	if (enable == ap1302->streaming)
		goto done;

	if (enable) {
		ret = ap1302_configure(ap1302);
		if (ret < 0)
			goto done;

		ret = ap1302_stall(ap1302, false);
	} else {
		ret = ap1302_stall(ap1302, true);
	}

done:
	mutex_unlock(&ap1302->lock);

	if (ret < 0)
		dev_err(ap1302->dev, "Failed to %s stream: %d\n",
			enable ? "start" : "stop", ret);

	return ret;
}

static const char * const ap1302_warnings[] = {
	"HINF_BANDWIDTH",
	"FLICKER_DETECTION",
	"FACED_NE",
	"SMILED_NE",
	"HINF_OVERRUN",
	NULL,
	"FRAME_TOO_SMALL",
	"MISSING_PHASES",
	"SPOOF_UNDERRUN",
	"JPEG_NOLAST",
	"NO_IN_FREQ_SPEC",
	"SINF0",
	"SINF1",
	"CAPTURE0",
	"CAPTURE1",
	"ISR_UNHANDLED",
	"INTERLEAVE_SPOOF",
	"INTERLEAVE_BUF",
	"COORD_OUT_OF_RANGE",
	"ICP_CLOCKING",
	"SENSOR_CLOCKING",
	"SENSOR_NO_IHDR",
	"DIVIDE_BY_ZERO",
	"INT0_UNDERRUN",
	"INT1_UNDERRUN",
	"SCRATCHPAD_TOO_BIG",
	"OTP_RECORD_READ",
	"NO_LSC_IN_OTP",
	"GPIO_INT_LOST",
	"NO_PDAF_DATA",
	"FAR_PDAF_ACCESS_SKIP",
	"PDAF_ERROR",
	"ATM_TVI_BOUNDS",
	"SIPM_0_RTY",
	"SIPM_1_TRY",
	"SIPM_0_NO_ACK",
	"SIPM_1_NO_ACK",
	"SMILE_DIS",
	"DVS_DIS",
	"TEST_DIS",
	"SENSOR_LV2LV",
	"SENSOR_FV2FV",
	"FRAME_LOST",
};

static const char * const ap1302_lane_states[] = {
	"stop_s",
	"hs_req_s",
	"lp_req_s",
	"hs_s",
	"lp_s",
	"esc_req_s",
	"turn_req_s",
	"esc_s",
	"esc_0",
	"esc_1",
	"turn_s",
	"turn_mark",
	"error_s",
};

static void ap1302_log_lane_state(struct ap1302_sensor *sensor,
				  unsigned int index)
{
	static const char * const lp_states[] = {
		"00", "10", "01", "11",
	};

	unsigned int counts[4][ARRAY_SIZE(ap1302_lane_states)];
	unsigned int samples = 0;
	unsigned int lane;
	unsigned int i;
	u32 first[4] = { 0, };
	u32 last[4] = { 0, };
	int ret;

	memset(counts, 0, sizeof(counts));

	for (i = 0; i < 1000; ++i) {
		u32 values[4];

		/*
		 * Read the state of all lanes and skip read errors and invalid
		 * values.
		 */
		for (lane = 0; lane < 4; ++lane) {
			ret = ap1302_read(sensor->ap1302,
					  AP1302_ADV_SINF_MIPI_INTERNAL_p_LANE_n_STAT(index, lane),
					  &values[lane]);
			if (ret < 0)
				break;

			if (AP1302_LANE_STATE(values[lane]) >=
			    ARRAY_SIZE(ap1302_lane_states)) {
				ret = -EINVAL;
				break;
			}
		}

		if (ret < 0)
			continue;

		/* Accumulate the samples and save the first and last states. */
		for (lane = 0; lane < 4; ++lane)
			counts[lane][AP1302_LANE_STATE(values[lane])]++;

		if (!samples)
			memcpy(first, values, sizeof(first));
		memcpy(last, values, sizeof(last));

		samples++;
	}

	if (!samples)
		return;

	/*
	 * Print the LP state from the first sample, the error state from the
	 * last sample, and the states accumulators for each lane.
	 */
	for (lane = 0; lane < 4; ++lane) {
		u32 state = last[lane];
		char error_msg[25] = "";

		if (state & (AP1302_LANE_ERR | AP1302_LANE_ABORT)) {
			unsigned int err = AP1302_LANE_ERR_STATE(state);
			const char *err_state = NULL;

			err_state = err < ARRAY_SIZE(ap1302_lane_states)
				  ? ap1302_lane_states[err] : "INVALID";

			snprintf(error_msg, sizeof(error_msg), "ERR (%s%s) %s LP%s",
				 state & AP1302_LANE_ERR ? "E" : "",
				 state & AP1302_LANE_ABORT ? "A" : "",
				 err_state,
				 lp_states[AP1302_LANE_ERR_LP_VAL(state)]);
		}

		dev_info(sensor->ap1302->dev, "SINF%u L%u state: LP%s %s",
			 index, lane, lp_states[AP1302_LANE_LP_VAL(first[lane])],
			 error_msg);

		for (i = 0; i < ARRAY_SIZE(ap1302_lane_states); ++i) {
			if (counts[lane][i])
				pr_cont(" %s:%u",
				       ap1302_lane_states[i],
				       counts[lane][i]);
		}
		pr_cont("\n");
	}

	/* Reset the error flags. */
	for (lane = 0; lane < 4; ++lane)
		ap1302_write(sensor->ap1302,
			     AP1302_ADV_SINF_MIPI_INTERNAL_p_LANE_n_STAT(index, lane),
			     AP1302_LANE_ERR | AP1302_LANE_ABORT, NULL);
}

static int ap1302_log_status(struct v4l2_subdev *sd)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	u16 frame_count_icp;
	u16 frame_count_brac;
	u16 frame_count_hinf;
	u32 warning[4];
	u32 error[3];
	unsigned int i;
	u32 value;
	int ret;

	/* Dump the console buffer. */
	ret = ap1302_dump_console(ap1302);
	if (ret < 0)
		return ret;

	/* Print errors. */
	ret = ap1302_read(ap1302, AP1302_ERROR, &error[0]);
	if (ret < 0)
		return ret;

	ret = ap1302_read(ap1302, AP1302_ERR_FILE, &error[1]);
	if (ret < 0)
		return ret;

	ret = ap1302_read(ap1302, AP1302_ERR_LINE, &error[2]);
	if (ret < 0)
		return ret;

	dev_info(ap1302->dev, "ERROR: 0x%04x (file 0x%08x:%u)\n",
		 error[0], error[1], error[2]);

	ret = ap1302_read(ap1302, AP1302_SIPM_ERR_0, &error[0]);
	if (ret < 0)
		return ret;

	ret = ap1302_read(ap1302, AP1302_SIPM_ERR_1, &error[1]);
	if (ret < 0)
		return ret;

	dev_info(ap1302->dev, "SIPM_ERR [0] 0x%04x [1] 0x%04x\n",
		 error[0], error[1]);

	/* Print warnings. */
	for (i = 0; i < ARRAY_SIZE(warning); ++i) {
		ret = ap1302_read(ap1302, AP1302_WARNING(i), &warning[i]);
		if (ret < 0)
			return ret;
	}

	dev_info(ap1302->dev,
		 "WARNING [0] 0x%04x [1] 0x%04x [2] 0x%04x [3] 0x%04x\n",
		 warning[0], warning[1], warning[2], warning[3]);

	for (i = 0; i < ARRAY_SIZE(ap1302_warnings); ++i) {
		if ((warning[i / 16] & BIT(i % 16)) &&
		    ap1302_warnings[i])
			dev_info(ap1302->dev, "- WARN_%s\n",
				 ap1302_warnings[i]);
	}

	/* Print the frame counter. */
	ret = ap1302_read(ap1302, AP1302_FRAME_CNT, &value);
	if (ret < 0)
		return ret;

	frame_count_hinf = value >> 8;
	frame_count_brac = value & 0xff;

	ret = ap1302_read(ap1302, AP1302_ADV_CAPTURE_A_FV_CNT, &value);
	if (ret < 0)
		return ret;

	frame_count_icp = value & 0xffff;

	dev_info(ap1302->dev, "Frame counters: ICP %u, HINF %u, BRAC %u\n",
		 frame_count_icp, frame_count_hinf, frame_count_brac);

	/* Sample the lane state. */
	for (i = 0; i < ARRAY_SIZE(ap1302->sensors); ++i) {
		struct ap1302_sensor *sensor = &ap1302->sensors[i];

		if (!sensor->ap1302)
			continue;

		ap1302_log_lane_state(sensor, i);
	}

	return 0;
}

static int ap1302_subdev_registered(struct v4l2_subdev *sd)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	unsigned int i;
	int ret;

	dev_dbg(ap1302->dev, "%s", __func__);

	for (i = 0; i < ARRAY_SIZE(ap1302->sensors); ++i) {
		struct ap1302_sensor *sensor = &ap1302->sensors[i];

		if (!sensor->present)
			continue;

		dev_dbg(ap1302->dev, "registering sensor %u\n", i);

		ret = v4l2_device_register_subdev(sd->v4l2_dev, &sensor->sd);
		if (ret)
			return ret;

		ret = media_create_pad_link(&sensor->sd.entity,
					    AR1335_PAD_SOURCE,
					    &sd->entity, AP1302_PAD_SINK_0 + i,
					    MEDIA_LNK_FL_IMMUTABLE |
					    MEDIA_LNK_FL_ENABLED);
		if (ret)
			return ret;
	}

	return 0;
}

static void ap1302_subdev_unregistered(struct v4l2_subdev *sd)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	int i;

	dev_dbg(ap1302->dev, "%s", __func__);

	for (i = 0; i < ARRAY_SIZE(ap1302->sensors); ++i) {
		struct ap1302_sensor *sensor = &ap1302->sensors[i];

		if (!sensor->present)
			continue;

		dev_info(ap1302->dev, "unregistering sensor %u\n", i);

		v4l2_device_unregister_subdev(&sensor->sd);
	}
}

static int ap1302_link_setup(struct media_entity *entity,
		  const struct media_pad *local,
		  const struct media_pad *remote, u32 flags)
{
	return 0;
}

/**
 *
 */
static int ap1302_s_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static const struct media_entity_operations ap1302_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.link_setup = ap1302_link_setup
};

static const struct v4l2_subdev_pad_ops ap1302_pad_ops = {
	.enum_mbus_code = ap1302_enum_mbus_code,
	.enum_frame_size = ap1302_enum_frame_size,
	.enum_frame_interval = ap1302_enum_frame_interval,
	.get_fmt = ap1302_get_fmt,
	.set_fmt = ap1302_set_fmt,
	.get_selection = ap1302_get_selection,
	.set_selection = ap1302_get_selection,
};

static const struct v4l2_subdev_video_ops ap1302_video_ops = {
	.s_stream = ap1302_s_stream,
};

static const struct v4l2_subdev_core_ops ap1302_core_ops = {
	.log_status = ap1302_log_status,
	.s_power = ap1302_s_power
};

static const struct v4l2_subdev_ops ap1302_subdev_ops = {
	.core = &ap1302_core_ops,
	.video = &ap1302_video_ops,
	.pad = &ap1302_pad_ops,
};

static const struct v4l2_subdev_internal_ops ap1302_subdev_internal_ops = {
	.init_state = ap1302_init_state,
	.registered = ap1302_subdev_registered,
	.unregistered = ap1302_subdev_unregistered,
};

/*
 *
 * Sensor
 *
 */

static int ap1302_sensor_enum_mbus_code(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *cfg,
					struct v4l2_subdev_mbus_code_enum *code)
{
	struct ap1302_sensor *sensor = to_ap1302_sensor(sd);
	const struct ap1302_sensor_info *info = sensor->ap1302->sensor_info;

	if (code->index)
		return -EINVAL;

	code->code = info->format;
	return 0;
}

static int ap1302_sensor_enum_frame_size(struct v4l2_subdev *sd,
					 struct v4l2_subdev_state *cfg,
					 struct v4l2_subdev_frame_size_enum *fse)
{
	struct ap1302_sensor *sensor = to_ap1302_sensor(sd);
	const struct ap1302_sensor_info *info = sensor->ap1302->sensor_info;

	if (fse->index)
		return -EINVAL;

	if (fse->code != info->format)
		return -EINVAL;

	fse->min_width = info->resolution.width;
	fse->min_height = info->resolution.height;
	fse->max_width = info->resolution.width;
	fse->max_height = info->resolution.height;

	return 0;
}

static int ap1302_sensor_get_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct ap1302_sensor *sensor = to_ap1302_sensor(sd);
	const struct ap1302_sensor_info *info = sensor->ap1302->sensor_info;

	memset(&fmt->format, 0, sizeof(fmt->format));

	fmt->format.width = info->resolution.width;
	fmt->format.height = info->resolution.height;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.code = info->format;
	fmt->format.colorspace = V4L2_COLORSPACE_SRGB;

	return 0;
}

static const struct v4l2_subdev_pad_ops ap1302_sensor_pad_ops = {
	.enum_mbus_code = ap1302_sensor_enum_mbus_code,
	.enum_frame_size = ap1302_sensor_enum_frame_size,
	.get_fmt = ap1302_sensor_get_fmt,
	.set_fmt = ap1302_sensor_get_fmt,
	.get_frame_interval = ap1302_g_frame_interval,
	.set_frame_interval = ap1302_s_frame_interval,
};

static const struct v4l2_subdev_ops ap1302_sensor_subdev_ops = {
	.pad = &ap1302_sensor_pad_ops,
};

static int ap1302_sensor_parse_of(struct ap1302_device *ap1302,
				  struct device_node *node)
{
	struct ap1302_sensor *sensor;
	u32 reg;
	int ret;

	/* Retrieve the sensor index from the reg property. */
	ret = of_property_read_u32(node, "reg", &reg);
	if (ret < 0) {
		dev_warn(ap1302->dev,
			 "'reg' property missing in sensor node\n");
		return -EINVAL;
	}

	if (reg >= ARRAY_SIZE(ap1302->sensors)) {
		dev_warn(ap1302->dev, "Out-of-bounds 'reg' value %u\n",
			 reg);
		return -EINVAL;
	}

	sensor = &ap1302->sensors[reg];
	if (sensor->ap1302) {
		dev_warn(ap1302->dev, "Duplicate entry for sensor %u\n", reg);
		return -EINVAL;
	}

	sensor->ap1302 = ap1302;
	sensor->of_node = of_node_get(node);

	return 0;
}

static int ap1302_sensor_init(struct ap1302_sensor *sensor, unsigned int index)
{
	struct ap1302_device *ap1302 = sensor->ap1302;
	struct v4l2_subdev *sd = &sensor->sd;
	int ret;

	sensor->index = index;
	sensor->present = true;

	v4l2_subdev_init(sd, &ap1302_sensor_subdev_ops);

	snprintf(sd->name, sizeof(sd->name), "%s %u",
		 ap1302->sensor_info->name, index);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, 1, &sensor->pad);
	if (ret < 0) {
		dev_err(ap1302->dev,
			"failed to initialize media entity for sensor %u: %d\n",
			index, ret);
		goto error;
	}

	return 0;

error:
	return ret;
}

static void ap1302_sensor_cleanup(struct ap1302_sensor *sensor)
{
	media_entity_cleanup(&sensor->sd.entity);

	of_node_put(sensor->of_node);
}

/*
 *
 * Hardware Initialisation
 *
 */

static int ap1302_detect_chip(struct ap1302_device *ap1302)
{
	unsigned int version;
	unsigned int revision;
	int ret;

	ret = ap1302_spi_read(ap1302, AP1302_CHIP_VERSION, &version);

	if (ret >= 0) {
		ret = ap1302_spi_read(ap1302, AP1302_CHIP_REV, &revision);
		if (ret)
			return ret;
	} else {
		ret = ap1302_read(ap1302, AP1302_CHIP_VERSION, &version);
		if (ret)
			return ret;

		ret = ap1302_read(ap1302, AP1302_CHIP_REV, &revision);
		if (ret)
			return ret;
	}

	if (version != AP1302_CHIP_ID) {
		dev_err(ap1302->dev,
			"Invalid chip version, expected 0x%04x, got 0x%04x\n",
			AP1302_CHIP_ID, version);
		return -EINVAL;
	}

	dev_info(ap1302->dev, "AP1302 revision %u.%u.%u detected\n",
		 (revision & 0xf000) >> 12, (revision & 0x0f00) >> 8,
		 revision & 0x00ff);

	return 0;
}

static int ap1302_hw_init(struct ap1302_device *ap1302)
{
	unsigned int retries, number_of_retries;
	int ret;

	/* Request and validate the firmware. */
	ret = ap1302_request_firmware(ap1302);
	if (ret)
		return ret;

	number_of_retries = MAX_FW_LOAD_RETRIES;

	/*
	 * If more than one ISP, power them both on and only try loading the
	 * firmware once.
	 */
	if (!retry_firmware) {
		number_of_retries = 1;

		ret = ap1302_power_on(ap1302);
		if (ret < 0)
			goto error_power;
	}

	/*
	 * Load the firmware, retrying in case of CRC errors.
	 */
	for (retries = 0; retries < number_of_retries; ++retries) {

		if (retry_firmware) {
			ret = ap1302_power_on(ap1302);
			if (ret < 0)
				goto error_firmware;
		}

		ret = ap1302_detect_chip(ap1302);
		if (ret)
			goto error_power;

		ret = ap1302_load_firmware(ap1302);
		if (!ret)
			break;

		if (ret != -EAGAIN)
			goto error_power;

		if (retry_firmware) {
			ap1302_power_off(ap1302);
		}
	}

	if (retries == number_of_retries) {
		dev_err(ap1302->dev,
			"Firmware load retries exceeded, aborting\n");
		ret = -ETIMEDOUT;
		goto error_firmware;
	}

	return 0;

error_power:
	ap1302_power_off(ap1302);
error_firmware:
	release_firmware(ap1302->fw);

	return ret;
}

static void ap1302_hw_cleanup(struct ap1302_device *ap1302)
{
	ap1302_power_off(ap1302);
}

/*
 *
 * Probe & Remove
 *
 */

static int ap1302_config_v4l2(struct ap1302_device *ap1302)
{
	struct v4l2_subdev *sd;
	unsigned int i;
	int ret;

	sd = &ap1302->sd;
	sd->dev = ap1302->dev;
	v4l2_i2c_subdev_init(sd, ap1302->client, &ap1302_subdev_ops);

	strscpy(sd->name, DRIVER_NAME, sizeof(sd->name));
	strlcat(sd->name, ".", sizeof(sd->name));
	strlcat(sd->name, dev_name(ap1302->dev), sizeof(sd->name));
	dev_dbg(ap1302->dev, "name %s\n", sd->name);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->internal_ops = &ap1302_subdev_internal_ops;
	sd->entity.function = MEDIA_ENT_F_PROC_VIDEO_ISP;
	sd->entity.ops = &ap1302_media_ops;

	for (i = 0; i < ARRAY_SIZE(ap1302->pads); ++i)
		ap1302->pads[i].flags = i == AP1302_PAD_SOURCE
				      ? MEDIA_PAD_FL_SOURCE : MEDIA_PAD_FL_SINK;

	ret = media_entity_pads_init(&sd->entity, ARRAY_SIZE(ap1302->pads),
				     ap1302->pads);
	if (ret < 0) {
		dev_err(ap1302->dev, "media_entity_init failed %d\n", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(ap1302->formats); ++i)
		ap1302->formats[i].info = &supported_video_formats[0];

	ret = ap1302_init_state(sd, NULL);
	if (ret < 0)
		goto error_media;

	ret = ap1302_ctrls_init(ap1302);
	if (ret < 0)
		goto error_media;

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(ap1302->dev, "v4l2_async_register_subdev failed %d\n", ret);
		goto error_ctrls;
	}

	return 0;

error_ctrls:
	ap1302_ctrls_cleanup(ap1302);
error_media:
	media_entity_cleanup(&sd->entity);
	return ret;
}

static int ap1302_parse_of(struct ap1302_device *ap1302)
{
	struct device_node *sensors;
	struct device_node *node;
	struct fwnode_handle *ep;
	unsigned int num_sensors = 0;
	const char *model;
	unsigned int i;
	int ret;

	/* Clock */
	ap1302->clock = devm_clk_get(ap1302->dev, "csi_mclk");


	if (IS_ERR(ap1302->clock)) {
		dev_err(ap1302->dev, "Failed to get clock: %ld\n",
			PTR_ERR(ap1302->clock));
		return PTR_ERR(ap1302->clock);
	}

	/* GPIOs */

	ap1302->power_gpio = devm_gpiod_get_optional(ap1302->dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(ap1302->power_gpio)) {
		dev_err(ap1302->dev, "Can't get power GPIO: %ld\n",
			PTR_ERR(ap1302->power_gpio));
		return PTR_ERR(ap1302->power_gpio);
	}

	ap1302->reset_gpio = devm_gpiod_get_optional(ap1302->dev, "reset",
					    GPIOD_OUT_HIGH);
	if (IS_ERR(ap1302->reset_gpio)) {
		dev_err(ap1302->dev, "Can't get reset GPIO: %ld\n",
			PTR_ERR(ap1302->reset_gpio));
		return PTR_ERR(ap1302->reset_gpio);
	}

	ap1302->standby_gpio = devm_gpiod_get_optional(ap1302->dev, "standby",
						       GPIOD_OUT_LOW);
	if (IS_ERR(ap1302->standby_gpio)) {
		dev_err(ap1302->dev, "Can't get standby GPIO: %ld\n",
			PTR_ERR(ap1302->standby_gpio));
		return PTR_ERR(ap1302->standby_gpio);
	}

	/* Bus configuration */
	ep = fwnode_graph_get_next_endpoint(dev_fwnode(ap1302->dev), NULL);
	if (!ep)
		return -EINVAL;

	ap1302->bus_cfg.bus_type = V4L2_MBUS_CSI2_DPHY;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &ap1302->bus_cfg);
	if (ret < 0) {
		dev_err(ap1302->dev, "Failed to parse bus configuration\n");
		return ret;
	}

	/* Sensors */
	sensors = of_get_child_by_name(ap1302->dev->of_node, "sensors");
	if (!sensors) {
		dev_err(ap1302->dev, "'sensors' child node not found\n");
		return -EINVAL;
	}

	ret = of_property_read_string(sensors, "onnn,model", &model);
	if (ret < 0) {
		dev_warn(ap1302->dev, "Missing sensor model attribute\n");
		ret = -EINVAL;
		goto done;
	}

	for (i = 0; i < ARRAY_SIZE(ap1302_sensor_info); ++i) {
		const struct ap1302_sensor_info *info =
			&ap1302_sensor_info[i];

		if (!strcmp(info->model, model)) {
			ap1302->sensor_info = info;
			break;
		}
	}

	if (!ap1302->sensor_info) {
		dev_warn(ap1302->dev, "Unsupported sensor model %s\n", model);
		ret = -EINVAL;
		goto done;
	}

	for_each_child_of_node(sensors, node) {
		if (of_node_name_eq(node, "sensor")) {
			if (!ap1302_sensor_parse_of(ap1302, node))
				num_sensors++;
		}
	}

	if (!num_sensors) {
		dev_err(ap1302->dev, "No sensor found\n");
		ret = -EINVAL;
		goto done;
	}

done:
	of_node_put(sensors);
	return ret;
}

static void ap1302_cleanup(struct ap1302_device *ap1302)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ap1302->sensors); ++i) {
		struct ap1302_sensor *sensor = &ap1302->sensors[i];

		if (!sensor->ap1302)
			continue;

		ap1302_sensor_cleanup(sensor);
	}

	v4l2_fwnode_endpoint_free(&ap1302->bus_cfg);

	mutex_destroy(&ap1302->lock);

	devm_kfree(ap1302->dev, ap1302);
}

static int ap1302_probe(struct i2c_client *client)
{
	struct ap1302_device *ap1302;
	unsigned int i;
	int ret;
	u32 val;
	u64 freq;

	ap1302 = devm_kzalloc(&client->dev, sizeof(*ap1302), GFP_KERNEL);
	if (!ap1302)
		return -ENOMEM;

	ap1302->dev = &client->dev;
	ap1302->client = client;

	if (retry_firmware) {
		dev_info(ap1302->dev, "Firmware retries enabled");
	} else {
		dev_info(ap1302->dev, "Firmware retries disabled");
	}

	freq = (u64)hinf_mipi_freq_tgt_hz;
	ap1302->hinf_mipi_freq_tgt = (u32)((freq * 0x10000UL)/1000000UL);

	freq = (u64)system_freq_in_hz;
	ap1302->system_freq_in = (u32)((freq * 0x10000UL)/1000000UL);

	mutex_init(&ap1302->lock);

	ap1302->regmap16 = devm_regmap_init_i2c(client, &ap1302_reg16_config);
	if (IS_ERR(ap1302->regmap16)) {
		dev_err(ap1302->dev, "regmap16 init failed: %ld\n",
			PTR_ERR(ap1302->regmap16));
		ret = -ENODEV;
		goto error;
	}

	ap1302->regmap32 = devm_regmap_init_i2c(client, &ap1302_reg32_config);
	if (IS_ERR(ap1302->regmap32)) {
		dev_err(ap1302->dev, "regmap32 init failed: %ld\n",
			PTR_ERR(ap1302->regmap32));
		ret = -ENODEV;
		goto error;
	}

	ret = ap1302_parse_of(ap1302);
	if (ret < 0)
		goto error;

	for (i = 0; i < ARRAY_SIZE(ap1302->sensors); ++i) {
		struct ap1302_sensor *sensor = &ap1302->sensors[i];

		if (!sensor->ap1302)
			continue;

		ret = ap1302_sensor_init(sensor, i);
		if (ret < 0)
			goto error;
	}

	ret = ap1302_hw_init(ap1302);
	if (ret)
		goto error;

	ap1302_debugfs_init(ap1302);

	ret = ap1302_config_v4l2(ap1302);
	if (ret)
		goto error_hw_cleanup;

	/* Record the current frame interval */

	ap1302_read(ap1302, AP1302_PREVIEW_MAX_FPS, &val);

	if ((val & 0xFF) == 0) {
		ap1302->frame_interval.numerator = 1;
		ap1302->frame_interval.denominator = (val >> 8);
	} else {
		dev_warn(ap1302->dev, "Fractional frame detected");
	}

	create_sysfs_attributes(ap1302);

	dev_info(ap1302->dev, "Sensor ready");

	return 0;

error_hw_cleanup:
	ap1302_hw_cleanup(ap1302);
error:
	ap1302_cleanup(ap1302);
	return ret;
}

static void ap1302_remove(struct i2c_client *client)
{
	struct ap1302_device *ap1302 = i2c_get_clientdata(client);

	remove_sysfs_attributes(ap1302);

	ap1302_debugfs_cleanup(ap1302);

	ap1302_hw_cleanup(ap1302);

	release_firmware(ap1302->fw);

	v4l2_async_unregister_subdev(&ap1302->sd);
	media_entity_cleanup(&ap1302->sd.entity);

	ap1302_ctrls_cleanup(ap1302);

	ap1302_cleanup(ap1302);
}

static const struct of_device_id ap1302_of_id_table[] = {
	{ .compatible = "onnn,ap1302" },
	{ }
};
MODULE_DEVICE_TABLE(of, ap1302_of_id_table);

static struct i2c_driver ap1302_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table	= ap1302_of_id_table,
	},
	.probe		= ap1302_probe,
	.remove		= ap1302_remove,
};

module_i2c_driver(ap1302_i2c_driver);

MODULE_AUTHOR("Florian Rebaudo <frebaudo@witekio.com>");
MODULE_AUTHOR("Laurent Pinchart <laurent.pinchart@ideasonboard.com>");
MODULE_AUTHOR("Anil Kumar M <anil.mamidala@xilinx.com>");
MODULE_AUTHOR("Paul Thomson <pault@imd-tec.com>");

MODULE_DESCRIPTION("ON Semiconductor AP1302 ISP driver");
MODULE_LICENSE("GPL");
