// SPDX-License-Identifier: GPL-2.0
/*
 * AR1335 driver
 *
 * Copyright (C) 2024 Advanced Micro Devices, Inc.
 *
 * Contacts: Anil Kumar Mamidala
 *           Vishnu Vardhan Ravuri
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/videodev2.h>
#include <media/v4l2-device.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#define AR1335_NAME "ar1335"
#define AR1335_MAX_RATIO_MISMATCH 10
#define EXPOSURE_MAX 0xC4E
#define FRAME_LENGTH_LINE_MAX 0x0C4E
#define LINE_LENGTH_PCK_MAX 4656
/* External clock (extclk) frequencies */
#define AR1335_EXTCLK_MIN		(6 * 1000 * 1000)
#define AR1335_EXTCLK_MAX		(48 * 1000 * 1000)
/* PLL and PLL2 */
#define AR1335_PLL_MIN			(320 * 1000 * 1000)
#define AR1335_PLL_MAX			(1200 * 1000 * 1000)
#define MAX_FRAME_RATE 60
#define MIN_FRAME_RATE 30
#define AR1335_DEF_FRAME_RATE 30
#define REG_FRAME_RATE 0x0340

/* Effective pixel sample rate on the pixel array. */
#define AR1335_PIXEL_CLOCK_RATE		(220 * 1000 * 1000)
#define AR1335_PIXEL_CLOCK_MIN		(168 * 1000 * 1000)
#define AR1335_PIXEL_CLOCK_MAX		(414 * 1000 * 1000)

#define AR1335_MIN_X_ADDR_START		8u
#define AR1335_MIN_Y_ADDR_START		8u
#define AR1335_MAX_X_ADDR_END		4231u
#define AR1335_MAX_Y_ADDR_END		3143u

#define AR1335_WIDTH_MIN		0u
#define AR1335_WIDTH_MAX		4239u
#define AR1335_HEIGHT_MIN		0u
#define AR1335_HEIGHT_MAX		3151u

#define AR1335_WIDTH_BLANKING_MIN	240u
#define AR1335_HEIGHT_BLANKING_MIN	142u /* must be even */
#define AR1335_TOTAL_HEIGHT_MAX		65535u /* max_frame_length_lines */
#define AR1335_TOTAL_WIDTH_MAX		65532u /* max_line_length_pck */

#define AR1335_ANA_GAIN_MIN		0x00
#define AR1335_ANA_GAIN_MAX		0x3f
#define AR1335_ANA_GAIN_STEP		0x01
#define AR1335_ANA_GAIN_DEFAULT		0x00

/* AR1335 registers */
#define AR1335_REG_VT_PIX_CLK_DIV		0x0300
#define AR1335_REG_FRAME_LENGTH_LINES		0x0340

#define AR1335_REG_CHIP_ID			0x0000
#define AR1335_REG_COARSE_INTEGRATION_TIME	0x3012
#define AR1335_REG_ROW_SPEED			0x3016
#define AR1335_REG_EXTRA_DELAY			0x3018
#define AR1335_REG_RESET			0x301A
#define   AR1335_REG_RESET_DEFAULTS		  0x0238
#define   AR1335_REG_RESET_GROUP_PARAM_HOLD	  0x8000
#define   AR1335_REG_RESET_STREAM		  BIT(2)
#define   AR1335_REG_RESET_RESTART		  BIT(1)
#define   AR1335_REG_RESET_INIT			  BIT(0)

#define AR1335_REG_ANA_GAIN_CODE_GLOBAL		0x3028

#define AR1335_REG_GREEN1_GAIN			0x3056
#define AR1335_REG_BLUE_GAIN			0x3058
#define AR1335_REG_RED_GAIN			0x305A
#define AR1335_REG_GREEN2_GAIN			0x305C
#define AR1335_REG_GLOBAL_GAIN			0x305E

#define AR1335_REG_HISPI_TEST_MODE		0x3066
#define AR1335_REG_HISPI_TEST_MODE_LP11		  0x0004

#define AR1335_REG_TEST_PATTERN_MODE		0x3070

#define AR1335_REG_SERIAL_FORMAT		0x31AE
#define AR1335_REG_SERIAL_FORMAT_MIPI		  0x0200

#define AR1335_REG_HISPI_CONTROL_STATUS		0x31C6
#define AR1335_REG_HISPI_CONTROL_STATUS_FRAMER_TEST_MODE_ENABLE 0x80

#define be		cpu_to_be16

static const char * const ar1335_supply_names[] = {
	"vdd_io",	/* I/O (1.8V) supply */
	"vdd",		/* Core, PLL and MIPI (1.2V) supply */
	"vaa",		/* Analog (2.7V) supply */
};
struct ar1335_reg {
	u16 addr;
	u16 val;
};

struct ar1335_res_struct {
	u16 width;
	u16 height;
	u16 out_fmt;
	u16 fps;
	struct ar1335_reg *ar1335_mode;
};

struct ar1335_context_res {
	s32 res_num;
	s32 cur_res;
	struct ar1335_res_struct *res_table;
};
static const s64 ar1335_link_frequencies[] = {
	184000000,
};

struct ar1335_ctrls {
	struct v4l2_ctrl_handler handler;
	struct {
		struct v4l2_ctrl *gain;
		struct v4l2_ctrl *red_balance;
		struct v4l2_ctrl *blue_balance;
	};
	struct {
		struct v4l2_ctrl *hblank;
		struct v4l2_ctrl *vblank;
	};
	struct v4l2_ctrl *pixrate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *test_pattern;
};

struct ar1335_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct clk *extclk;
	u32 extclk_freq;
	struct v4l2_subdev subdev;
	struct v4l2_ctrl_handler ctrl_handler;

	struct regulator *supplies[ARRAY_SIZE(ar1335_supply_names)];
	struct gpio_desc *reset_gpio;

	/* lock to protect all members below */
	struct mutex lock;
	struct ar1335_res_struct *res_table;
	s32 cur_res;
	struct v4l2_fract frame_rate;
	struct v4l2_mbus_framefmt fmt;
	struct ar1335_ctrls ctrls;
	unsigned int lane_count;
	struct {
		u16 pre;
		u16 mult;
		u16 pre2;
		u16 mult2;
		u16 vt_pix;
	} pll;
};

static inline struct ar1335_dev *to_ar1335_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar1335_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ar1335_dev,
			     ctrls.handler)->sd;
}

static u32 div64_round(u64 v, u32 d)
{
	return div_u64(v + (d >> 1), d);
}

static u32 div64_round_up(u64 v, u32 d)
{
	return div_u64(v + d - 1, d);
}

static int ar1335_code_to_bpp(struct ar1335_dev *sensor)
{
	switch (sensor->fmt.code) {
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return 10;
	}

	return -EINVAL;
}


/* Data must be BE16, the first value is the register address */
static int ar1335_write_regs(struct ar1335_dev *sensor, const __be16 *data,
			     unsigned int count)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg;
	int ret;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = (u8 *)data;
	msg.len = count * sizeof(*data);
	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret < 0) {
		v4l2_err(&sensor->sd, "%s: I2C write error\n", __func__);
		return ret;
	}
	return 0;
}

static int ar1335_write_reg(struct ar1335_dev *sensor, u16 reg, u16 val)
{
	__be16 buf[2] = {be(reg), be(val)};

	return ar1335_write_regs(sensor, buf, 2);
}

static int ar1335_set_geometry(struct ar1335_dev *sensor)
{
	/* Center the image in the visible output window. */
	u16 x = clamp((AR1335_WIDTH_MAX - sensor->fmt.width) / 2,
		       AR1335_MIN_X_ADDR_START, AR1335_MAX_X_ADDR_END);
	u16 y = clamp(((AR1335_HEIGHT_MAX - sensor->fmt.height) / 2) & ~1,
		       AR1335_MIN_Y_ADDR_START, AR1335_MAX_Y_ADDR_END);

	/* All dimensions are unsigned 12-bit integers */
	__be16 regs[] = {
		be(AR1335_REG_FRAME_LENGTH_LINES),
		be(sensor->fmt.height + sensor->ctrls.vblank->val),
		be(sensor->fmt.width + sensor->ctrls.hblank->val),
		be(x),
		be(y),
		be(x + sensor->fmt.width - 1),
		be(y + sensor->fmt.height - 1),
		be(sensor->fmt.width),
		be(sensor->fmt.height)
	};
	return ar1335_write_regs(sensor, regs, ARRAY_SIZE(regs));
}
static int ar1335_set_gains(struct ar1335_dev *sensor)
{
	int green = sensor->ctrls.gain->val;
	int red = max(green + sensor->ctrls.red_balance->val, 0);
	int blue = max(green + sensor->ctrls.blue_balance->val, 0);
	unsigned int gain = min(red, min(green, blue));
	unsigned int analog = min(gain, 64u); /* range is 0 - 127 */
	__be16 regs[5];

	red   = min(red   - analog + 64, 511u);
	green = min(green - analog + 64, 511u);
	blue  = min(blue  - analog + 64, 511u);
	regs[0] = be(AR1335_REG_GREEN1_GAIN);
	regs[1] = be(green << 7 | analog);
	regs[2] = be(blue  << 7 | analog);
	regs[3] = be(red   << 7 | analog);
	regs[4] = be(green << 7 | analog);
	return ar1335_write_regs(sensor, regs, ARRAY_SIZE(regs));
}

static u32 calc_pll(struct ar1335_dev *sensor, u32 freq, u16 *pre_ptr, u16 *mult_ptr)
{
	u16 pre = 1, mult = 1, new_pre;
	u32 pll = AR1335_PLL_MAX + 1;
//dev_info(&sensor->i2c_client->dev, "Sensor is running at %u Hz input clock\n", sensor->extclk_freq);
	for (new_pre = 1; new_pre < 64; new_pre++) {
		u32 new_pll;
		u32 new_mult = div64_round_up((u64)freq * new_pre,
					      sensor->extclk_freq);

		if (new_mult < 32)
			continue; /* Minimum value */
		if (new_mult > 254)
			break; /* Maximum, larger pre won't work either */
		if (sensor->extclk_freq * (u64)new_mult < AR1335_PLL_MIN *
		    new_pre)
			continue;
		if (sensor->extclk_freq * (u64)new_mult > AR1335_PLL_MAX *
		    new_pre)
			break; /* Larger pre won't work either */
		new_pll = div64_round_up(sensor->extclk_freq * (u64)new_mult,
					 new_pre);
		if (new_pll < pll) {
			pll = new_pll;
			pre = new_pre;
			mult = new_mult;
		}
	}
	pll = div64_round(sensor->extclk_freq * (u64)mult, pre);
	*pre_ptr = pre;
	*mult_ptr = mult;
	return pll;
}

static void ar1335_calc_pll(struct ar1335_dev *sensor)
{
	unsigned int pixel_clock;
	u16 pre, mult;
	u32 vco;
	int bpp;
	pixel_clock = AR1335_PIXEL_CLOCK_RATE * 2 / sensor->lane_count;
	bpp = ar1335_code_to_bpp(sensor);
	sensor->pll.vt_pix = bpp / 2;
	vco = pixel_clock * sensor->pll.vt_pix;

	calc_pll(sensor, vco, &pre, &mult);

	sensor->pll.pre = sensor->pll.pre2 = pre;
	sensor->pll.mult = sensor->pll.mult2 = mult;
}

static int ar1335_pll_config(struct ar1335_dev *sensor)
{
	__be16 pll_regs[] = {
		be(AR1335_REG_VT_PIX_CLK_DIV),
		/* 0x300 */ be(sensor->pll.vt_pix), /* vt_pix_clk_div = bpp / 2 */
		/* 0x302 */ be(1), /* vt_sys_clk_div */
		/* 0x304 */ be((sensor->pll.pre2 << 8) | sensor->pll.pre),
		/* 0x306 */ be((sensor->pll.mult2 << 8) | sensor->pll.mult),
		/* 0x308 */ be(sensor->pll.vt_pix * 2), /* op_pix_clk_div = 2 * vt_pix_clk_div */
		/* 0x30A */ be(1)  /* op_sys_clk_div */
	};
	ar1335_calc_pll(sensor);
	return ar1335_write_regs(sensor, pll_regs, ARRAY_SIZE(pll_regs));
}

static int ar1335_set_stream(struct ar1335_dev *sensor, bool on)
{
	int ret;
	if (on) {
		/* Stop streaming for just a moment */
		ret = ar1335_write_reg(sensor, AR1335_REG_RESET,
				       AR1335_REG_RESET_DEFAULTS);
		if (ret)
			return ret;

		ret = ar1335_set_geometry(sensor);
		if (ret)
			return ret;

		ret = ar1335_pll_config(sensor);
		if (ret)
			goto err;

		ret =  __v4l2_ctrl_handler_setup(&sensor->ctrls.handler);
		if (ret)
			goto err;

		/* Exit LP-11 mode on clock and data lanes */
		ret = ar1335_write_reg(sensor, AR1335_REG_HISPI_CONTROL_STATUS,
				       0);
		if (ret)
			goto err;

		/* Start streaming */
		ret = ar1335_write_reg(sensor, AR1335_REG_RESET,
				       AR1335_REG_RESET_DEFAULTS |
				       AR1335_REG_RESET_STREAM);
		if (ret)
			goto err;

		return 0;

err:
		return ret;

	} else {
		/*
		 * Reset gain, the sensor may produce all white pixels without
		 * this
		 */
		ret = ar1335_write_reg(sensor, AR1335_REG_GLOBAL_GAIN, 0x2000);
		if (ret)
			return ret;

		/* Stop streaming */
		ret = ar1335_write_reg(sensor, AR1335_REG_RESET,
				       AR1335_REG_RESET_DEFAULTS);
		if (ret)
			return ret;

		//pm_runtime_put(&sensor->i2c_client->dev);
		return 0;
	}
}

static struct ar1335_res_struct ar1335_res_table[] = {
	{
		.width = 1920,
		.height = 1080,
	},
	{
		.width = 3840,
		.height = 2160,
	}
};


static int ar1335_match_resolution(struct v4l2_mbus_framefmt *fmt)
{
	s32 w0, h0, mismatch, distance;
	s32 w1 = fmt->width;
	s32 h1 = fmt->height;
	s32 min_distance = INT_MAX;
	s32 i, idx = -1;

	if (w1 == 0 || h1 == 0)
		return -1;

	for (i = 0; i < ARRAY_SIZE(ar1335_res_table); i++) {
		w0 = ar1335_res_table[i].width;
		h0 = ar1335_res_table[i].height;
		if (w0 < w1 || h0 < h1)
			continue;
		mismatch = abs(w0 * h1 - w1 * h0) * 8192 / w1 / h0;

		if (mismatch > 8192 * AR1335_MAX_RATIO_MISMATCH / 100)
			continue;
		distance = (w0 * h1 + w1 * h0) * 8192 / w1 / h1;
		if (distance < min_distance) {
			min_distance = distance;
			idx = i;
			break;
		}
	}
	return idx;
}

static s32 ar1335_try_mbus_fmt_locked(struct v4l2_subdev *sd,
				      struct v4l2_mbus_framefmt *fmt)
{
	s32 res_num, idx = -1;

	res_num = ARRAY_SIZE(ar1335_res_table);

	if (fmt->width <= ar1335_res_table[res_num - 1].width &&
	    fmt->height <= ar1335_res_table[res_num - 1].height)
		idx = ar1335_match_resolution(fmt);
	if (idx == -1)
		idx = res_num - 1;

	fmt->width = ar1335_res_table[idx].width;
	fmt->height = ar1335_res_table[idx].height;
	return idx;
}

static void ar1335_adj_fmt(struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = clamp(ALIGN(fmt->width, 4), AR1335_WIDTH_MIN,
			   AR1335_WIDTH_MAX);
	fmt->height = clamp(ALIGN(fmt->height, 4), AR1335_HEIGHT_MIN,
			    AR1335_HEIGHT_MAX);
	fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int ar1335_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct ar1335_dev *sensor = to_ar1335_dev(sd);
	if (format->pad)
		return -EINVAL;

	mutex_lock(&sensor->lock);
	fmt->width = sensor->fmt.width;
	fmt->height = sensor->fmt.height;
	fmt->code = sensor->fmt.code;
	fmt->field = sensor->fmt.field;
	mutex_unlock(&sensor->lock);
	return 0;
}

static int ar1335_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar1335_dev *sensor = to_ar1335_dev(sd);
	struct v4l2_mbus_framefmt *fmt = &format->format;
	int max_vblank, max_hblank;
	s32 idx, ret = 0;
	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		ar1335_try_mbus_fmt_locked(sd, fmt);
		#ifdef V4L2_SUBDEV_PAD_CONFIG_HAS_TRY_FMT
		if (sd_state->pads)
			sd_state->pads->try_fmt = *fmt;
		#endif

		mutex_unlock(&sensor->lock);

		return 0;
	}
	idx = ar1335_try_mbus_fmt_locked(sd, &format->format);
	sensor->cur_res = idx;
	ar1335_calc_pll(sensor);
	sensor->fmt.width = format->format.width;
	sensor->fmt.height = format->format.height;
	sensor->fmt.field = V4L2_FIELD_NONE;
	if (format->format.code == MEDIA_BUS_FMT_SRGGB10_1X10 ||
	    format->format.code == MEDIA_BUS_FMT_SRGGB8_1X8) {
		sensor->fmt.code = format->format.code;
	} else {
		dev_err(&client->dev, "%s %d format->format.code %d\n", __func__, __LINE__,
			format->format.code);
		return -EINVAL;
	}

	/*
	 * Update the exposure and blankings limits. Blankings are also reset
	 * to the minimum.
	 */
	 // Calculate vblank and hblank values
    	int vblank = FRAME_LENGTH_LINE_MAX - fmt->height;
    	int hblank = LINE_LENGTH_PCK_MAX - fmt->width;
	max_hblank = AR1335_TOTAL_WIDTH_MAX - sensor->fmt.width;
	ret = __v4l2_ctrl_modify_range(sensor->ctrls.hblank,
				       sensor->ctrls.hblank->minimum,
				       max_hblank, sensor->ctrls.hblank->step,
				       hblank);
	if (ret)
		goto unlock;

	ret = __v4l2_ctrl_s_ctrl(sensor->ctrls.hblank,hblank);
	if (ret)
		goto unlock;

	max_vblank = AR1335_TOTAL_HEIGHT_MAX - sensor->fmt.height;
	ret = __v4l2_ctrl_modify_range(sensor->ctrls.vblank,
				       sensor->ctrls.vblank->minimum,
				       max_vblank, sensor->ctrls.vblank->step,
				       vblank);
	if (ret)
		goto unlock;

	ret = __v4l2_ctrl_s_ctrl(sensor->ctrls.vblank,
				 vblank);
	if (ret)
		goto unlock;
	ret = __v4l2_ctrl_modify_range(sensor->ctrls.exposure,
				       sensor->ctrls.exposure->minimum,
				       EXPOSURE_MAX,
				       sensor->ctrls.exposure->step,
				       sensor->ctrls.exposure->default_value);
unlock:
	mutex_unlock(&sensor->lock);

	return ret;
}

static u16 ar1335_test_pattern_values[] = {
        0x0, // Normal pixel mode
        0x1, // Solid color
        0x2, // 100% color bar
        0x3, // fade to gray color
        0x100, // walking 1 (10bit)
        0x101, // walking 1 (8bit)
};

static int ar1335_test_pattern(struct v4l2_subdev *sd, s32 val)
{
        struct ar1335_dev *sensor = to_ar1335_dev(sd);
        return ar1335_write_reg(sensor, AR1335_REG_TEST_PATTERN_MODE,
                                    ar1335_test_pattern_values[val]);
}


static int ar1335_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct ar1335_dev *sensor = to_ar1335_dev(sd);
	int exp_max;
	int ret;

	/* v4l2_ctrl_lock() locks our own mutex */

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		exp_max = sensor->fmt.height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(sensor->ctrls.exposure,
					 sensor->ctrls.exposure->minimum,
					 exp_max, sensor->ctrls.exposure->step,
					 sensor->ctrls.exposure->default_value);
		break;
	}
	switch (ctrl->id) {
	case V4L2_CID_HBLANK:
	case V4L2_CID_VBLANK:
		ret = ar1335_set_geometry(sensor);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ar1335_write_reg(sensor, AR1335_REG_ANA_GAIN_CODE_GLOBAL,
				       ctrl->val);
		break;
	case V4L2_CID_GAIN:
	case V4L2_CID_RED_BALANCE:
	case V4L2_CID_BLUE_BALANCE:
		ret = ar1335_set_gains(sensor);
		break;
	case V4L2_CID_EXPOSURE:
		ret = ar1335_write_reg(sensor,
				       AR1335_REG_COARSE_INTEGRATION_TIME,
				       ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		 ret = ar1335_test_pattern(&sensor->sd,ctrl->val);
		break;
	default:
		dev_err(&sensor->i2c_client->dev,
			"Unsupported control %x\n", ctrl->id);
		ret = -EINVAL;
		break;
	}
	return ret;
}

static const struct v4l2_ctrl_ops ar1335_ctrl_ops = {
	.s_ctrl = ar1335_s_ctrl,
};

static const char * const test_pattern_menu[] = {
	"Normal pixel operation",
	"Solid color",
	"100% Color Bar",
	"Fade-to-Gray Color Bars",
	"Walking 1s (10-bit)",
	"Walking 1s (8-bit)",
};

static int ar1335_init_controls(struct ar1335_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &ar1335_ctrl_ops;
	struct ar1335_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	int max_vblank, max_hblank;
	struct v4l2_ctrl *link_freq;
	int ret;

	v4l2_ctrl_handler_init(hdl, 32);

	/* We can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;

	/* Analog gain */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN,
			  AR1335_ANA_GAIN_MIN, AR1335_ANA_GAIN_MAX,
			  AR1335_ANA_GAIN_STEP, AR1335_ANA_GAIN_DEFAULT);

	/* Manual gain */
	ctrls->gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_GAIN, 0, 511, 1, 40);
	ctrls->red_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_RED_BALANCE,
					       -512, 511, 1, 0);
	ctrls->blue_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BLUE_BALANCE,
						-512, 511, 1, 0);
	v4l2_ctrl_cluster(3, &ctrls->gain);

	/* Initialize blanking limits using the default 2592x1944 format. */
	max_hblank = AR1335_TOTAL_WIDTH_MAX - AR1335_WIDTH_MAX;
	ctrls->hblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HBLANK,
					  AR1335_WIDTH_BLANKING_MIN,
					  max_hblank, 1,
					  AR1335_WIDTH_BLANKING_MIN);

	max_vblank = AR1335_TOTAL_HEIGHT_MAX - AR1335_HEIGHT_MAX;
	ctrls->vblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK,
					  AR1335_HEIGHT_BLANKING_MIN,
					  max_vblank, 2,
					  AR1335_HEIGHT_BLANKING_MIN);
	v4l2_ctrl_cluster(2, &ctrls->hblank);

	/* Read-only */
	ctrls->pixrate = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_PIXEL_RATE,
					   AR1335_PIXEL_CLOCK_MIN,
					   AR1335_PIXEL_CLOCK_MAX, 1,
					   AR1335_PIXEL_CLOCK_RATE);
	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE, 0,
					    EXPOSURE_MAX, 1, 0xC2E);

	link_freq = v4l2_ctrl_new_int_menu(hdl, ops, V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(ar1335_link_frequencies) - 1,
					0, ar1335_link_frequencies);
	if (link_freq)
		link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrls->test_pattern = v4l2_ctrl_new_std_menu_items(hdl, ops,
					V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(test_pattern_menu) - 1,
					0, 0, test_pattern_menu);

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	sensor->sd.ctrl_handler = hdl;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

#define REGS_ENTRY(a)	{(a), ARRAY_SIZE(a)}
#define REGS(...)	REGS_ENTRY(((const __be16[]){__VA_ARGS__}))

static const struct initial_reg {
	const __be16 *data; /* data[0] is register address */
	unsigned int count;
} initial_regs[] = {
	REGS(be(0x301A),be(0x0210)),               
	REGS(be(0x3EB6),be(0x004D)), 
	REGS(be(0x3EBC),be(0xAA06)),
	REGS(be(0x3EC0),be(0x1E02)),
	REGS(be(0x3EC2),be(0x7700)),
	REGS(be(0x3EC4),be(0x1C08)),
	REGS(be(0x3EC6),be(0xEA44)),
	REGS(be(0x3EC8),be(0x0F0F)),
	REGS(be(0x3ECA),be(0x0F4A)),
	REGS(be(0x3ECC),be(0x0706)),
	REGS(be(0x3ECE),be(0x443B)),
	REGS(be(0x3ED0),be(0x12F0)),
	REGS(be(0x3ED2),be(0x0039)),
	REGS(be(0x3ED4),be(0x862F)),
	REGS(be(0x3ED6),be(0x4080)),
	REGS(be(0x3ED8),be(0x0523)),
	REGS(be(0x3EDA),be(0xF896)),
	REGS(be(0x3EDC),be(0x508C)),
	REGS(be(0x3EDE),be(0x5005)),
	REGS(be(0x316A),be(0x8200)),
	REGS(be(0x316E),be(0x8200)),
	REGS(be(0x316C),be(0x8200)),
	REGS(be(0x3EF0),be(0x414D)),
	REGS(be(0x3EF2),be(0x0101)),
	REGS(be(0x3EF6),be(0x0307)),
	REGS(be(0x3EFA),be(0x0F0F)),
	REGS(be(0x3EFC),be(0x0F0F)),
	REGS(be(0x3EFE),be(0x0F0F)),
	REGS(be(0x3172), be(0x0206)), /* txlo clk divider options */
	REGS(be(0x3040), be(0x4041)),
	REGS(be(0x317A), be(0x416E)),
	REGS(be(0x3F3C), be(0x0003)),
	REGS(be(0x0400), be(0x0000)),
	REGS(be(0x0404), be(0x0010)),
	REGS(be(0x31B0),
	     be(0x0086),  /* 31B0: frame_preamble - FIXME check WRT lanes# */
	     be(0x0057), /* 31B2: line_preamble - FIXME check WRT lanes# */
	     be(0x2412),
	     be(0x142A),
	     be(0x2413),
	     be(0x1C70),
	     be(0x068B)),
	/* don't use continuous clock mode while shut down */
	//REGS(be(0x31BC), be(0x068B)),
	REGS(be(0x0112), be(0x0A0A)), /* 10-bit/10-bit mode */
	REGS(be(0x3D00),be(0x0446)),
	REGS(be(0x3D02),be(0x4C66)),
	REGS(be(0x3D04),be(0xFFFF)),
	REGS(be(0x3D06),be(0xFFFF)),
	REGS(be(0x3D08),be(0x5E40)),
	REGS(be(0x3D0A),be(0x1146)),
	REGS(be(0x3D0C),be(0x5D41)),
	REGS(be(0x3D0E),be(0x1088)),
	REGS(be(0x3D10),be(0x8342)),
	REGS(be(0x3D12),be(0x00C0)),
	REGS(be(0x3D14),be(0x5580)),
	REGS(be(0x3D16),be(0x5B83)),
	REGS(be(0x3D18),be(0x6084)),
	REGS(be(0x3D1A),be(0x5A8D)),
	REGS(be(0x3D1C),be(0x00C0)),
	REGS(be(0x3D1E),be(0x8342)),
	REGS(be(0x3D20),be(0x925A)),
	REGS(be(0x3D22),be(0x8664)),
	REGS(be(0x3D24),be(0x1030)),
	REGS(be(0x3D26),be(0x801C)),
	REGS(be(0x3D28),be(0x00A0)),
	REGS(be(0x3D2A),be(0x56B0)),
	REGS(be(0x3D2C),be(0x5788)),
	REGS(be(0x3D2E),be(0x5150)),
	REGS(be(0x3D30),be(0x824D)),
	REGS(be(0x3D32),be(0x8D58)),
	REGS(be(0x3D34),be(0x58D2)),
	REGS(be(0x3D36),be(0x438A)),
	REGS(be(0x3D38),be(0x4592)),
	REGS(be(0x3D3A),be(0x458A)),
	REGS(be(0x3D3C),be(0x4389)),
	REGS(be(0x3D3E),be(0x51FF)),
	REGS(be(0x3D40),be(0x8451)),
	REGS(be(0x3D42),be(0x8410)),
	REGS(be(0x3D44),be(0x0C88)),
	REGS(be(0x3D46),be(0x5959)),
	REGS(be(0x3D48),be(0x8A5F)),
	REGS(be(0x3D4A),be(0xDA42)),
	REGS(be(0x3D4C),be(0x9361)),
	REGS(be(0x3D4E),be(0X8262)),
	REGS(be(0x3D50),be(0x8342)),
	REGS(be(0x3D52),be(0x8010)),
	REGS(be(0x3D54),be(0xC041)),
	REGS(be(0x3D56),be(0x64FF)),
	REGS(be(0x3D58),be(0xFFB7)),
	REGS(be(0x3D5A),be(0x4081)),
	REGS(be(0x3D5C),be(0x4080)),
	REGS(be(0x3D5E),be(0x4180)),
	REGS(be(0x3D60),be(0x4280)),
	REGS(be(0x3D62),be(0x438D)),
	REGS(be(0x3D64),be(0x44BA)),
	REGS(be(0x3D66),be(0x4488)),
	REGS(be(0x3D68),be(0x4380)),
	REGS(be(0x3D6A),be(0x4241)),
	REGS(be(0x3D6C),be(0x8140)),
	REGS(be(0x3D6E),be(0x8240)),
	REGS(be(0x3D70),be(0x8041)),
	REGS(be(0x3D72),be(0x8042)),
	REGS(be(0x3D74),be(0x8043)),
	REGS(be(0x3D76),be(0x8D44)),
	REGS(be(0x3D78),be(0xBA44)),
	REGS(be(0x3D7A),be(0x875E)),
	REGS(be(0x3D7C),be(0x4354)),
	REGS(be(0x3D7E),be(0x4241)),
	REGS(be(0x3D80),be(0x8140)),
	REGS(be(0x3D82),be(0x8120)),
	REGS(be(0x3D84),be(0x2881)),
	REGS(be(0x3D86),be(0x6026)),
	REGS(be(0x3D88),be(0x8055)),
	REGS(be(0x3D8A),be(0x8070)),
	REGS(be(0x3D8C),be(0x8040)),
	REGS(be(0x3D8E),be(0x4C81)),
	REGS(be(0x3D90),be(0x45C3)),
	REGS(be(0x3D92),be(0x4581)),
	REGS(be(0x3D94),be(0x4C40)),
	REGS(be(0x3D96),be(0x8070)),
	REGS(be(0x3D98),be(0x8040)),
	REGS(be(0x3D9A),be(0x4C85)),
	REGS(be(0x3D9C),be(0x6CA8)),
	REGS(be(0x3D9E),be(0x6C8C)),
	REGS(be(0x3DA0),be(0x000E)),
	REGS(be(0x3DA2),be(0xBE44)),
	REGS(be(0x3DA4),be(0x8844)),
	REGS(be(0x3DA6),be(0xBC78)),
	REGS(be(0x3DA8),be(0x0900)),
	REGS(be(0x3DAA),be(0x8904)),
	REGS(be(0x3DAC),be(0x8080)),
	REGS(be(0x3DAE),be(0x0240)),
	REGS(be(0x3DB0),be(0x8609)),
	REGS(be(0x3DB2),be(0x008E)),
	REGS(be(0x3DB4),be(0x0900)),
	REGS(be(0x3DB6),be(0x8002)),
	REGS(be(0x3DB8),be(0x4080)),
	REGS(be(0x3DBA),be(0x0480)),
	REGS(be(0x3DBC),be(0x887C)),
	REGS(be(0x3DBE),be(0xAA86)),
	REGS(be(0x3DC0),be(0x0900)),
	REGS(be(0x3DC2),be(0x877A)),
	REGS(be(0x3DC4),be(0x000E)),
	REGS(be(0x3DC6),be(0xC379)),
	REGS(be(0x3DC8),be(0x4C40)),
	REGS(be(0x3DCA),be(0xBF70)),
	REGS(be(0x3DCC),be(0x5E40)),
	REGS(be(0x3DCE),be(0x114E)),
	REGS(be(0x3DD0),be(0x5D41)),
	REGS(be(0x3DD2),be(0x5383)),
	REGS(be(0x3DD4),be(0x4200)),
	REGS(be(0x3DD6),be(0xC055)),
	REGS(be(0x3DD8),be(0xA400)),
	REGS(be(0x3DDA),be(0xC083)),
	REGS(be(0x3DDC),be(0x4288)),
	REGS(be(0x3DDE),be(0x6083)),
	REGS(be(0x3DE0),be(0x5B80)),
	REGS(be(0x3DE2),be(0x5A64)),
	REGS(be(0x3DE4),be(0x1030)),
	REGS(be(0x3DE6),be(0x801C)),
	REGS(be(0x3DE8),be(0x00A5)),
	REGS(be(0x3DEA),be(0x5697)),
	REGS(be(0x3DEC),be(0x57A5)),
	REGS(be(0x3DEE),be(0x5180)),
	REGS(be(0x3DF0),be(0x505A)),
	REGS(be(0x3DF2),be(0x814D)),
	REGS(be(0x3DF4),be(0x8358)),
	REGS(be(0x3DF6),be(0x8058)),
	REGS(be(0x3DF8),be(0xA943)),
	REGS(be(0x3DFA),be(0x8345)),
	REGS(be(0x3DFC),be(0xB045)),
	REGS(be(0x3DFE),be(0x8343)),
	REGS(be(0x3E00),be(0xA351)),
	REGS(be(0x3E02),be(0xE251)),
	REGS(be(0x3E04),be(0x8C59)),
	REGS(be(0x3E06),be(0x8059)),
	REGS(be(0x3E08),be(0x8A5F)),
	REGS(be(0x3E0A),be(0xEC7C)),
	REGS(be(0x3E0C),be(0xCC84)),
	REGS(be(0x3E0E),be(0x6182)),
	REGS(be(0x3E10),be(0x6283)),
	REGS(be(0x3E12),be(0x4283)),
	REGS(be(0x3E14),be(0x10CC)),
	REGS(be(0x3E16),be(0x6496)),
	REGS(be(0x3E18),be(0x4281)),
	REGS(be(0x3E1A),be(0x41BB)),
	REGS(be(0x3E1C),be(0x4082)),
	REGS(be(0x3E1E),be(0x407E)),
	REGS(be(0x3E20),be(0xCC41)),
	REGS(be(0x3E22),be(0x8042)),
	REGS(be(0x3E24),be(0x8043)),
	REGS(be(0x3E26),be(0x8300)),
	REGS(be(0x3E28),be(0xC088)),
	REGS(be(0x3E2A),be(0x44BA)),
	REGS(be(0x3E2C),be(0x4488)),
	REGS(be(0x3E2E),be(0x00C8)),
	REGS(be(0x3E30),be(0x8042)),
	REGS(be(0x3E32),be(0x4181)),
	REGS(be(0x3E34),be(0x4082)),
	REGS(be(0x3E36),be(0x4080)),
	REGS(be(0x3E38),be(0x4180)),
	REGS(be(0x3E3A),be(0x4280)),
	REGS(be(0x3E3C),be(0x4383)),
	REGS(be(0x3E3E),be(0x00C0)),
	REGS(be(0x3E40),be(0x8844)),
	REGS(be(0x3E42),be(0xBA44)),
	REGS(be(0x3E44),be(0x8800)),
	REGS(be(0x3E46),be(0xC880)),
	REGS(be(0x3E48),be(0x4241)),
	REGS(be(0x3E4A),be(0x8240)),
	REGS(be(0x3E4C),be(0x8140)),
	REGS(be(0x3E4E),be(0x8041)),
	REGS(be(0x3E50),be(0x8042)),
	REGS(be(0x3E52),be(0x8043)),
	REGS(be(0x3E54),be(0x8300)),
	REGS(be(0x3E56),be(0xC088)),
	REGS(be(0x3E58),be(0x44BA)),
	REGS(be(0x3E5A),be(0x4488)),
	REGS(be(0x3E5C),be(0x00C8)),
	REGS(be(0x3E5E),be(0x8042)),
	REGS(be(0x3E60),be(0x4181)),
	REGS(be(0x3E62),be(0x4082)),
	REGS(be(0x3E64),be(0x4080)),
	REGS(be(0x3E66),be(0x4180)),
	REGS(be(0x3E68),be(0x4280)),
	REGS(be(0x3E6A),be(0x4383)),
	REGS(be(0x3E6C),be(0x00C0)),
	REGS(be(0x3E6E),be(0x8844)),
	REGS(be(0x3E70),be(0xBA44)),
	REGS(be(0x3E72),be(0x8800)),
	REGS(be(0x3E74),be(0xC880)),
	REGS(be(0x3E76),be(0x4241)),
	REGS(be(0x3E78),be(0x8140)),
	REGS(be(0x3E7A),be(0x9F5E)),
	REGS(be(0x3E7C),be(0x8A54)),
	REGS(be(0x3E7E),be(0x8620)),
	REGS(be(0x3E80),be(0x2881)),
	REGS(be(0x3E82),be(0x6026)),
	REGS(be(0x3E84),be(0x8055)),
	REGS(be(0x3E86),be(0x8070)),
	REGS(be(0x3E88),be(0x0000)),
	REGS(be(0x3E8A),be(0x0000)),
	REGS(be(0x3E8C),be(0x0000)),
	REGS(be(0x3E8E),be(0x0000)),
	REGS(be(0x3E90),be(0x0000)),
	REGS(be(0x3E92),be(0x0000)),
	REGS(be(0x3E94),be(0x0000)),
	REGS(be(0x3E96),be(0x0000)),
	REGS(be(0x3E98),be(0x0000)),
	REGS(be(0x3E9A),be(0x0000)),
	REGS(be(0x3E9C),be(0x0000)),
	REGS(be(0x3E9E),be(0x0000)),
	REGS(be(0x3EA0),be(0x0000)),
	REGS(be(0x3EA2),be(0x0000)),
	REGS(be(0x3EA4),be(0x0000)),
	REGS(be(0x3EA6),be(0x0000)),
	REGS(be(0x3EA8),be(0x0000)),
	REGS(be(0x3EAA),be(0x0000)),
	REGS(be(0x3EAC),be(0x0000)),
	REGS(be(0x3EAE),be(0x0000)),
	REGS(be(0x3EB0),be(0x0000)),
	REGS(be(0x3EB2),be(0x0000)),
	REGS(be(0x3EB4),be(0x0000)),
};

static int ar1335_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ar1335_dev *sensor = to_ar1335_dev(sd);
	int i;

	clk_disable_unprepare(sensor->extclk);

	if (sensor->reset_gpio)
		gpiod_set_value(sensor->reset_gpio, 1); /* assert RESET signal */

	for (i = ARRAY_SIZE(ar1335_supply_names) - 1; i >= 0; i--) {
		if (sensor->supplies[i])
			regulator_disable(sensor->supplies[i]);
	}
	return 0;
}

static int ar1335_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ar1335_dev *sensor = to_ar1335_dev(sd);
	unsigned int cnt;
	int ret;
		gpiod_set_value(sensor->reset_gpio, 0);
		mdelay(1);
		gpiod_set_value(sensor->reset_gpio, 1);
		mdelay(1);

	for (cnt = 0; cnt < ARRAY_SIZE(initial_regs); cnt++) {
		ret = ar1335_write_regs(sensor, initial_regs[cnt].data,
					initial_regs[cnt].count);
		if (ret)
			goto off;
	}

	ret = ar1335_write_reg(sensor, AR1335_REG_SERIAL_FORMAT,
			       AR1335_REG_SERIAL_FORMAT_MIPI |
			       sensor->lane_count);
	if (ret)
		goto off;

	/* set MIPI test mode - disabled for now */
	ret = ar1335_write_reg(sensor, AR1335_REG_HISPI_TEST_MODE,
			       ((0x40 << sensor->lane_count) - 0x40) |
			       AR1335_REG_HISPI_TEST_MODE_LP11);
	if (ret)
		goto off;

	ret = ar1335_write_reg(sensor, AR1335_REG_ROW_SPEED, 0x110 |
			       4 / sensor->lane_count);
	if (ret)
		goto off;

	return 0;
off:
	ar1335_power_off(dev);
	return ret;
}

static int ar1335_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ar1335_dev *sensor = to_ar1335_dev(sd);

	if (code->index)
		return -EINVAL;

	code->code = sensor->fmt.code;
	return 0;
}

static int ar1335_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index)
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGRBG10_1X10)
		return -EINVAL;

	fse->min_width = AR1335_WIDTH_MIN;
	fse->max_width = AR1335_WIDTH_MAX;
	fse->min_height = AR1335_HEIGHT_MIN;
	fse->max_height = AR1335_HEIGHT_MAX;

	return 0;
}

static int ar1335_pre_streamon(struct v4l2_subdev *sd, u32 flags)
{
	struct ar1335_dev *sensor = to_ar1335_dev(sd);
	int ret;

	if (!(flags & V4L2_SUBDEV_PRE_STREAMON_FL_MANUAL_LP))
		return -EACCES;
	/* Set LP-11 on clock and data lanes */
	ret = ar1335_write_reg(sensor, AR1335_REG_HISPI_CONTROL_STATUS,
			AR1335_REG_HISPI_CONTROL_STATUS_FRAMER_TEST_MODE_ENABLE);
	if (ret)
		goto err;

	/* Start streaming LP-11 */
	ret = ar1335_write_reg(sensor, AR1335_REG_RESET,
			       AR1335_REG_RESET_DEFAULTS |
			       AR1335_REG_RESET_STREAM);
	if (ret)
		goto err;
	return 0;

err:
	return ret;
}

static int ar1335_post_streamoff(struct v4l2_subdev *sd)
{
	return 0;
}

static int ar1335_set_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_frame_interval *ival)
{
	struct ar1335_dev *sensor = to_ar1335_dev(sd);
	struct v4l2_fract *tpf = &ival->interval;

	if (tpf->numerator == 0 || tpf->denominator == 0 ||
		(tpf->denominator > tpf->numerator * MAX_FRAME_RATE)) {
			/* Reset to max frame rate */
			tpf->numerator = 1;
			tpf->denominator = MAX_FRAME_RATE;
	}

	sensor->frame_rate.numerator = tpf->numerator;

	if (tpf->numerator == 30) {
		ar1335_write_reg(sensor, 0x340, 0xC4E);
		ar1335_write_reg(sensor, 0x202, 0xC4E);
		sensor->frame_rate.denominator = tpf->denominator;
	} else if (tpf->numerator == 60) {
		ar1335_write_reg(sensor, 0x340, 0x626);
		ar1335_write_reg(sensor, 0x202, 0x5E8);
		sensor->frame_rate.denominator = tpf->denominator;
	} else {
		ar1335_write_reg(sensor, 0x340, 0xC4E);
		ar1335_write_reg(sensor, 0x202, 0xC4E);
		sensor->frame_rate.denominator = MIN_FRAME_RATE;
	}

	return 0;
}

static int ar1335_get_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_frame_interval *interval)
{
	struct ar1335_dev *sensor = to_ar1335_dev(sd);

	mutex_lock(&sensor->lock);
	interval->interval.denominator = sensor->frame_rate.denominator;
	interval->interval.numerator = sensor->frame_rate.numerator;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int ar1335_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar1335_dev *sensor = to_ar1335_dev(sd);
	int ret;

	mutex_lock(&sensor->lock);
	ret = ar1335_set_stream(sensor, enable);
	mutex_unlock(&sensor->lock);

	return ret;
}

static const struct v4l2_subdev_core_ops ar1335_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
};

static const struct v4l2_subdev_video_ops ar1335_video_ops = {
	.s_stream = ar1335_s_stream,
	.pre_streamon = ar1335_pre_streamon,
	.post_streamoff = ar1335_post_streamoff,
};

static const struct v4l2_subdev_pad_ops ar1335_pad_ops = {
	.enum_mbus_code = ar1335_enum_mbus_code,
	.enum_frame_size = ar1335_enum_frame_size,
	.set_frame_interval = ar1335_set_frame_interval,
	.get_frame_interval = ar1335_get_frame_interval,
	.get_fmt = ar1335_get_fmt,
	.set_fmt = ar1335_set_fmt,
};

static const struct v4l2_subdev_ops ar1335_subdev_ops = {
	.core = &ar1335_core_ops,
	.video = &ar1335_video_ops,
	.pad = &ar1335_pad_ops,
};

static int ar1335_probe(struct i2c_client *client)
{
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct ar1335_dev *sensor;
	unsigned int cnt;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;
	sensor->fmt.width = AR1335_WIDTH_MAX;
	sensor->fmt.height = AR1335_HEIGHT_MAX;
	sensor->frame_rate.numerator = 1;
	sensor->frame_rate.denominator = AR1335_DEF_FRAME_RATE;
	endpoint = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 0, 0,
						   FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "could not parse endpoint\n");
		return ret;
	}

	if (ep.bus_type != V4L2_MBUS_CSI2_DPHY) {
		dev_err(dev, "invalid bus type, must be MIPI CSI2\n");
		return -EINVAL;
	}

	sensor->lane_count = ep.bus.mipi_csi2.num_data_lanes;
	switch (sensor->lane_count) {
	case 1:
	case 2:
	case 4:
		break;
	default:
		dev_err(dev, "invalid number of MIPI data lanes\n");
		return -EINVAL;
	}
	/* Get master clock (extclk) */
	sensor->extclk = devm_clk_get(dev, "extclk");
	if (IS_ERR(sensor->extclk)) {
		dev_err(dev, "failed to get extclk\n");
		return PTR_ERR(sensor->extclk);
	}

	sensor->extclk_freq = clk_get_rate(sensor->extclk);

	if (sensor->extclk_freq < AR1335_EXTCLK_MIN ||
	    sensor->extclk_freq > AR1335_EXTCLK_MAX) {
		dev_err(dev, "extclk frequency out of range: %u Hz\n",
			sensor->extclk_freq);
		return -EINVAL;
	}
	dev_info(&client->dev, "Sensor is running at %u Hz input clock\n", sensor->extclk_freq); 

	/* Request optional reset pin (usually active low) and assert it */
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	v4l2_i2c_subdev_init(&sensor->sd, client, &ar1335_subdev_ops);

	sensor->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

	for (cnt = 0; cnt < ARRAY_SIZE(ar1335_supply_names); cnt++) {
		struct regulator *supply = devm_regulator_get(dev,
						ar1335_supply_names[cnt]);

		if (IS_ERR(supply)) {
			dev_info(dev, "no %s regulator found: %li\n",
				 ar1335_supply_names[cnt], PTR_ERR(supply));
			return PTR_ERR(supply);
		}
		sensor->supplies[cnt] = supply;
	}

	mutex_init(&sensor->lock);

	ret = ar1335_init_controls(sensor);
	if (ret)
		goto entity_cleanup;

	ar1335_adj_fmt(&sensor->fmt);

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret)
		goto free_ctrls;
	ret = ar1335_power_on(&client->dev);
	if (ret)
		goto disable;
	dev_info(&client->dev, "AR1335 probe completed successfully\n");
	return 0;

disable:
	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
	mutex_destroy(&sensor->lock);
	return ret;
}

static void ar1335_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar1335_dev *sensor = to_ar1335_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
	mutex_destroy(&sensor->lock);
}

static const struct of_device_id ar1335_id[] = {
	{.compatible = AR1335_NAME },
	{}
};
MODULE_DEVICE_TABLE(of, ar1335_id);

static struct i2c_driver ar1335_driver = {
	.driver = {
		.name  = AR1335_NAME,
		.of_match_table = ar1335_id,
	},
	.probe = ar1335_probe,
	.remove = ar1335_remove,
};

module_i2c_driver(ar1335_driver);

MODULE_AUTHOR("Anil Kumar Mamidala, Vishnu Vardhan Ravuri");
MODULE_DESCRIPTION("V4L driver for camera sensor AR1335");
MODULE_LICENSE("GPL v2");
