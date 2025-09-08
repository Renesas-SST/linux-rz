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

#define AP1302_BITS_PER_PIXEL 16

static u16 ap1302_wb_values[] = {
	AP1302_AWB_CTRL_MODE_OFF,	/* V4L2_WHITE_BALANCE_MANUAL */
	AP1302_AWB_CTRL_MODE_AUTO,	/* V4L2_WHITE_BALANCE_AUTO */
	AP1302_AWB_CTRL_MODE_A,		/* V4L2_WHITE_BALANCE_INCANDESCENT */
	AP1302_AWB_CTRL_MODE_D50,	/* V4L2_WHITE_BALANCE_FLUORESCENT */
	AP1302_AWB_CTRL_MODE_D65,	/* V4L2_WHITE_BALANCE_FLUORESCENT_H */
	AP1302_AWB_CTRL_MODE_HORIZON,	/* V4L2_WHITE_BALANCE_HORIZON */
	AP1302_AWB_CTRL_MODE_D65,	/* V4L2_WHITE_BALANCE_DAYLIGHT */
	AP1302_AWB_CTRL_MODE_AUTO,	/* V4L2_WHITE_BALANCE_FLASH */
	AP1302_AWB_CTRL_MODE_D75,	/* V4L2_WHITE_BALANCE_CLOUDY */
	AP1302_AWB_CTRL_MODE_D75,	/* V4L2_WHITE_BALANCE_SHADE */
};

static int ap1302_set_wb_mode(struct ap1302_device *ap1302, s32 mode)
{
	u32 val;
	int ret;

	ret = ap1302_read(ap1302, AP1302_AWB_CTRL, &val);
	if (ret)
		return ret;

	val &= ~AP1302_AWB_CTRL_MODE_MASK;
	val |= ap1302_wb_values[mode];

	if (mode == V4L2_WHITE_BALANCE_FLASH)
		val |= AP1302_AWB_CTRL_FLASH;
	else
		val &= ~AP1302_AWB_CTRL_FLASH;

	return ap1302_write(ap1302, AP1302_AWB_CTRL, val, NULL);
}

static int ap1302_set_exposure(struct ap1302_device *ap1302, s32 mode)
{
	u32 val;
	int ret;

	ret = ap1302_read(ap1302, AP1302_AE_CTRL, &val);
	if (ret)
		return ret;

	val &= ~AP1302_AE_CTRL_MODE_MASK;
	val |= mode;

	return ap1302_write(ap1302, AP1302_AE_CTRL, val, NULL);
}

static int ap1302_set_exp_met(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_AE_MET, val, NULL);
}

static int ap1302_set_gain(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_AE_MANUAL_GAIN, val, NULL);
}

static int ap1302_set_contrast(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_CONTRAST, val, NULL);
}

static int ap1302_set_brightness(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_BRIGHTNESS, val, NULL);
}

static int ap1302_set_saturation(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_SATURATION, val, NULL);
}

static int ap1302_set_gamma(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_GAMMA, val, NULL);
}

static int ap1302_set_zoom(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_DZ_TGT_FCT, val, NULL);
}

static u16 ap1302_sfx_values[] = {
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_NONE */
	AP1302_SFX_MODE_SFX_BW,		/* V4L2_COLORFX_BW */
	AP1302_SFX_MODE_SFX_SEPIA1,	/* V4L2_COLORFX_SEPIA */
	AP1302_SFX_MODE_SFX_NEGATIVE,	/* V4L2_COLORFX_NEGATIVE */
	AP1302_SFX_MODE_SFX_EMBOSS,	/* V4L2_COLORFX_EMBOSS */
	AP1302_SFX_MODE_SFX_SKETCH,	/* V4L2_COLORFX_SKETCH */
	AP1302_SFX_MODE_SFX_BLUISH,	/* V4L2_COLORFX_SKY_BLUE */
	AP1302_SFX_MODE_SFX_GREENISH,	/* V4L2_COLORFX_GRASS_GREEN */
	AP1302_SFX_MODE_SFX_REDISH,	/* V4L2_COLORFX_SKIN_WHITEN */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_VIVID */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_AQUA */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_ART_FREEZE */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_SILHOUETTE */
	AP1302_SFX_MODE_SFX_SOLARIZE, /* V4L2_COLORFX_SOLARIZATION */
	AP1302_SFX_MODE_SFX_ANTIQUE, /* V4L2_COLORFX_ANTIQUE */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_SET_CBCR */
};

static int ap1302_set_special_effect(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_SFX_MODE, ap1302_sfx_values[val],
			    NULL);
}

static u16 ap1302_scene_mode_values[] = {
	AP1302_SCENE_CTRL_MODE_NORMAL,		/* V4L2_SCENE_MODE_NONE */
	AP1302_SCENE_CTRL_MODE_BACKLIGHT,	/* V4L2_SCENE_MODE_BACKLIGHT */
	AP1302_SCENE_CTRL_MODE_BEACH,		/* V4L2_SCENE_MODE_BEACH_SNOW */
	AP1302_SCENE_CTRL_MODE_TWILIGHT,	/* V4L2_SCENE_MODE_CANDLE_LIGHT */
	AP1302_SCENE_CTRL_MODE_NORMAL,		/* V4L2_SCENE_MODE_DAWN_DUSK */
	AP1302_SCENE_CTRL_MODE_NORMAL,		/* V4L2_SCENE_MODE_FALL_COLORS */
	AP1302_SCENE_CTRL_MODE_FIREWORKS,	/* V4L2_SCENE_MODE_FIREWORKS */
	AP1302_SCENE_CTRL_MODE_LANDSCAPE,	/* V4L2_SCENE_MODE_LANDSCAPE */
	AP1302_SCENE_CTRL_MODE_NIGHT,		/* V4L2_SCENE_MODE_NIGHT */
	AP1302_SCENE_CTRL_MODE_PARTY,		/* V4L2_SCENE_MODE_PARTY_INDOOR */
	AP1302_SCENE_CTRL_MODE_PORTRAIT,	/* V4L2_SCENE_MODE_PORTRAIT */
	AP1302_SCENE_CTRL_MODE_SPORT,		/* V4L2_SCENE_MODE_SPORTS */
	AP1302_SCENE_CTRL_MODE_SUNSET,		/* V4L2_SCENE_MODE_SUNSET */
	AP1302_SCENE_CTRL_MODE_DOCUMENT,	/* V4L2_SCENE_MODE_TEXT */
};

static int ap1302_set_scene_mode(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_SCENE_CTRL,
			    ap1302_scene_mode_values[val], NULL);
}

static const u16 ap1302_flicker_values[] = {
	AP1302_FLICK_CTRL_MODE_DISABLED,
	AP1302_FLICK_CTRL_FREQ(50) | AP1302_FLICK_CTRL_MODE_MANUAL,
	AP1302_FLICK_CTRL_FREQ(60) | AP1302_FLICK_CTRL_MODE_MANUAL,
	AP1302_FLICK_CTRL_MODE_AUTO,
};

static int ap1302_set_test_pattern(struct ap1302_device *ap1302, s32 val)
{
	int ret;
	u32 reg = 0;
	u32 read_reg;
	u32 const default_tp_mode = 2;

	/* At the time of writing the ap1302 firmware set AP1302_SENSOR_SELECT
	 * to 0x0211. ie
	 * 	TP_MODE 2
	 * 	SENSOR (1) (primary sensor)
	 * 	SINF (1) MIPI
	 * 	PATTERN_ON (0)
	 * Minimise write backs by setting TP_MODE to default_tp_mode if the
	 * test pattern is off.
	 * 
	 * Preserve SENSOR as best we can. However, at the time of writing,
	 * switching an enabled test patterm off will not restore the original
	 * setting and SENSOR is set to (1) primary sensor.
	 */
	ret = ap1302_read(ap1302, AP1302_SENSOR_SELECT, &reg);
	
	if (ret) {
		return ret;
	}

	read_reg = reg;

	if (val == 0) {
		/* TP is OFF */
		reg = reg & (~AP1302_SENSOR_SELECT_PATTERN_ON);

		/* SENSOR needs to be something other than zero (TP) */
		if (0 == (reg & AP1302_SENSOR_SELECT_SENSOR_MASK)) {
			/* This cannot be zero. Set to primary sensor. */

			/* TODO:
			 * This does not support a secondary sensor. Switching
			 * on TP and then switching it OFF only works for 
			 * primary sensor.
			 */
			reg |= AP1302_SENSOR_SELECT_SENSOR(0);
		}

		/* Set the TP_MODE to default_tp_mode.
		 * No TP is produced as AP1302_SENSOR_SELECT_PATTERN_ON is not
		 * set. Using the default may save us an unnessasery write back.
		 */

		/* Clear TP_MODE */
		reg &= (~(AP1302_SENSOR_TP_MODE_MASK));

		/* Set TP_MODE to the default (default_tp_mode) */
		reg |= AP1302_SENSOR_SELECT_TP_MODE(default_tp_mode);
	} else {
		/* TP on */

		/* Set bit AP1302_SENSOR_SELECT_PATTERN_ON */
		reg |= AP1302_SENSOR_SELECT_PATTERN_ON;

		/* Clear SENSOR field to 0 (TP) */
		reg &= (~AP1302_SENSOR_SELECT_SENSOR_MASK);

		/* Clear TP_MODE field */
		reg &= (~(AP1302_SENSOR_TP_MODE_MASK));

		/* Set TP_MODE field to the requested TP */
		reg |= AP1302_SENSOR_SELECT_TP_MODE(val);
	}

	/* Only write back if different to read value. */
	if (reg != read_reg) {
		ret = ap1302_write(ap1302, AP1302_SENSOR_SELECT, reg, NULL);
		return ret;
	} else {
		return 0;
	}
}

static int ap1302_set_flicker_freq(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_FLICK_CTRL,
			    ap1302_flicker_values[val], NULL);
}

static int ap1302_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ap1302_device *ap1302 =
		container_of(ctrl->handler, struct ap1302_device, ctrls);

	switch (ctrl->id) {
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		return ap1302_set_wb_mode(ap1302, ctrl->val);

	case V4L2_CID_EXPOSURE:
		return ap1302_set_exposure(ap1302, ctrl->val);

	case V4L2_CID_EXPOSURE_METERING:
		return ap1302_set_exp_met(ap1302, ctrl->val);

	case V4L2_CID_GAIN:
		return ap1302_set_gain(ap1302, ctrl->val);

	case V4L2_CID_GAMMA:
		return ap1302_set_gamma(ap1302, ctrl->val);

	case V4L2_CID_CONTRAST:
		return ap1302_set_contrast(ap1302, ctrl->val);

	case V4L2_CID_BRIGHTNESS:
		return ap1302_set_brightness(ap1302, ctrl->val);

	case V4L2_CID_SATURATION:
		return ap1302_set_saturation(ap1302, ctrl->val);

	case V4L2_CID_ZOOM_ABSOLUTE:
		return ap1302_set_zoom(ap1302, ctrl->val);

	case V4L2_CID_COLORFX:
		return ap1302_set_special_effect(ap1302, ctrl->val);

	case V4L2_CID_SCENE_MODE:
		return ap1302_set_scene_mode(ap1302, ctrl->val);

	case V4L2_CID_POWER_LINE_FREQUENCY:
		return ap1302_set_flicker_freq(ap1302, ctrl->val);

	case V4L2_CID_TEST_PATTERN:
		return ap1302_set_test_pattern(ap1302, ctrl->val);

	default:
		pr_info("ap1302: s_ctrl invalid\n");
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops ap1302_ctrl_ops = {
	.s_ctrl = ap1302_s_ctrl,
};

static const struct v4l2_ctrl_config ap1302_ctrls[] = {
	{
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
		.min = 0,
		.max = 9,
		.def = 1,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_GAMMA,
		.name = "Gamma",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -32768,
		.max = 32768,
		.step = 1,
		.def = 0,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_CONTRAST,
		.name = "Contrast",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -32768,
		.max = 0x5999,
		.step = 1,
		.def = 0,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_BRIGHTNESS,
		.name = "Brightness",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -4096,
		.max = 4096,
		.step = 1,
		.def = 0,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_SATURATION,
		.name = "Saturation",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0x4000,
		.step = 1,
		.def = 0x1000,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_EXPOSURE,
		.name = "Exposure",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xC,
		.step = 1,
		.def = 0xC,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_EXPOSURE_METERING,
		.name = "Exposure Metering",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0x3,
		.step = 1,
		.def = 0x1,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFFFF,
		.step = 1,
		.def = 0x100,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_ZOOM_ABSOLUTE,
		.min = 256,
		.max = 4096,
		.step = 1,
		.def = 256,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_COLORFX,
		.min = 0,
		.max = 15,
		.def = 0,
		.menu_skip_mask =
			BIT(15) | BIT(12) | BIT(11) | BIT(10) | BIT(9),
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_SCENE_MODE,
		.min = 0,
		.max = 13,
		.def = 0,
		.menu_skip_mask = BIT(5) | BIT(4),
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_POWER_LINE_FREQUENCY,
		.min = 0,
		.max = 3,
		.def = 3,
	},
};

static const char *const test_pattern_menu[] = {
	"Normal Operation",
	"flat color",
	"pseudo random (noise)",
	"100% color bars",
	"fade to grey bars",
	"PRBS1",
	"PRBS2",
	"PRBS3",
	"PRBS4",
	"vertical stripes",
	"vertical ramp",
	"walking 1's (10bit)",
	"walking 1's (8bit)",
	"black and white"
};

static u32 ap1302_calculate_pixel_rate(struct ap1302_device *ap1302) 
{
	u64 num_data_lanes = ap1302->bus_cfg.bus.mipi_csi2.num_data_lanes;
	u64 mipi_data_rate_hz;
	u64 pixel_rate;

	/*
	 * We do a back calculation to take the AP1302 MIPI data rate 
	 * and convert it to a Pixel rate that can be used by other drivers.
	 * The MIPI Data rate is in Hz (ie Bits per second)
	 */
	mipi_data_rate_hz = (ap1302->hinf_mipi_freq_tgt * 1000000UL);
	do_div(mipi_data_rate_hz, 0x10000UL);

	pixel_rate = mipi_data_rate_hz * num_data_lanes / AP1302_BITS_PER_PIXEL;  
	return (u32)pixel_rate;
}

int ap1302_ctrls_init(struct ap1302_device *ap1302)
{
	unsigned int i;
	u32 pixel_rate;
	int ret;

	ret = v4l2_ctrl_handler_init(&ap1302->ctrls, ARRAY_SIZE(ap1302_ctrls));
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(ap1302_ctrls); i++)
		v4l2_ctrl_new_custom(&ap1302->ctrls, &ap1302_ctrls[i], NULL);


	v4l2_ctrl_new_std_menu_items(&ap1302->ctrls, &ap1302_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(test_pattern_menu) - 1,
				     0, 0, test_pattern_menu);

	pixel_rate = ap1302_calculate_pixel_rate(ap1302);
	v4l2_ctrl_new_std(&ap1302->ctrls, &ap1302_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  0, pixel_rate, 1, pixel_rate);

	if (ap1302->ctrls.error) {
		ret = ap1302->ctrls.error;
		v4l2_ctrl_handler_free(&ap1302->ctrls);
		return ret;
	}

	/* Use same lock for controls as for everything else. */
	ap1302->ctrls.lock = &ap1302->lock;
	ap1302->sd.ctrl_handler = &ap1302->ctrls;

	return 0;
}

void ap1302_ctrls_cleanup(struct ap1302_device *ap1302)
{
	v4l2_ctrl_handler_free(&ap1302->ctrls);
}
