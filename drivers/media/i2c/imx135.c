// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX135 cameras.
 * Copyright (C) 2024, Shruggie SRL
 *
 * Based on Sony imx219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 *
 */

/*
 * This driver was written based on imx219, so the basic structure (of this driver)
 * should be the same, but it could be that some controls don't work well.
 *
 * These are known to work fine (values are for example):
 *		v4l2-ctl --set-ctrl=digital_gain=4
 *		v4l2-ctl --set-ctrl=analogue_gain=140
 *		v4l2-ctl --set-ctrl=exposure=3000
 * They can be adjusted in imx135_set_ctrl().
 *
 * The Digital Gain for imx135 and some regs, is interesting.
 * It seems that there are 2 regs for each R,G,B.
 * So, the base values are represented as imx135_digital_gain_base_reg_values[].
 * The default base values, seem to give a good white-balance.
 * But may need more tuning.
 *
 * Some of the register set that works here is an adaptation from this kernel:
 *   https://android.googlesource.com/kernel/tegra/+/2268683075e741190919217a72fcf13eb174dc57/drivers/media/platform/tegra/imx135.c
 * For 1280x720, the register set (in the tegra imx135 driver) is 'static struct imx135_reg mode_1280x720[]' or '720p 30fps'
 * The adjustments that were done are:
 *     {0x0108, 0x03}   ->  {0x0108, 0x01}   (From 4 Lanes to 2 Lanes)
 *     {0x0309, 0x05}   ->  {0x0309, 0x0A}   (2x on a PLL divider)
 * The inspiration for this conversion came from diff-ing
 *     https://github.com/ArduCAM/ArduCAM_USB_Camera_Shield/blob/master/ROS/arducam_usb2_ros/camera_config_files/IMX135_MIPI_4L_13MP.cfg
 *     https://github.com/ArduCAM/ArduCAM_USB_Camera_Shield/blob/master/ROS/arducam_usb2_ros/camera_config_files/IMX135_MIPI_2L_13MP.cfg
 *
 *
 * For 1920x1080, register set (in the tegra imx135 driver) is 'static struct imx135_reg mode_1920x1080[]'
 * The adjustments that were done are:
 *     {0x0108, 0x03}   ->  {0x0108, 0x01}   (From 4 Lanes to 2 Lanes)
 *     {0x0309, 0x05}   ->  {0x0309, 0x0A}   (2x on a PLL divider)
 *
 * The inspiration for the changes below, came from trying out different combinations found here:
 *     https://github.com/torvalds/linux/blob/ef674997e49760137ca9a90aac41a9922ac399b2/drivers/staging/media/atomisp/i2c/imx/imx135.h#L2854
 *
 *     {0x0340, 0x0A},  ->  {0x0340, 0x04},  (These 2 registers are the number of lines per frame)
 *     {0x0341, 0x40},  ->  {0x0341, 0xCA},  So, 2624 -> 1226
 *
 *     {0x0342, 0x11},  -> {0x0342, 0x23},   (These 2 registers are the pixels_per_line)
 *     {0x0343, 0xDC},  -> {0x0343, 0xB8},   So, 4572 -> 9144
 *
 *
 * For 2104x1560, register set is from:
 *    https://github.com/ArduCAM/ArduCAM_USB_Camera_Shield/blob/master/ROS/arducam_usb2_ros/camera_config_files/IMX135_MIPI_4L_3MP.cfg
 * The adjustments that were done are:
 *     {0x0108, 0x03}   ->  {0x0108, 0x01}   (From 4 Lanes to 2 Lanes)
 *     {0x0309, 0x05}   ->  {0x0309, 0x0A}   (2x on a PLL divider)
 *
 * For some details about what some other registers mean, this can be reviewed:
 *   https://github.com/torvalds/linux/blob/ef674997e49760137ca9a90aac41a9922ac399b2/drivers/staging/media/atomisp/i2c/imx/imx135.h#L667
 * Some of the registers have comments about what they are/mean, but especially for the
 * size-setting registers, the correlation between some of the sizes is not clear.
 *
 * This suggests that a tool from Sony generates the register sets, based on desired configuration.
 *
 */


#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <asm/unaligned.h>

#define IMX135_REG_VALUE_08BIT		1
#define IMX135_REG_VALUE_16BIT		2

#define IMX135_REG_MODE_SELECT		0x0100
#define IMX135_MODE_STANDBY		0x00
#define IMX135_MODE_STREAMING		0x01

/* Chip ID */
#define IMX135_REG_CHIP_ID		0x0016
#define IMX135_CHIP_ID			0x0135

/* External clock frequency is 24.0M */
#define IMX135_XCLK_FREQ		24000000

#define IMX135_PIXEL_RATE		182400000

#define IMX135_DEFAULT_LINK_FREQ	456000000

/* V_TIMING internal */
#define IMX135_REG_VTS			0x0340
#define IMX135_VTS_MAX			0xffff

#define IMX135_VBLANK_MIN		4

/*Frame Length Line*/
#define IMX135_FLL_MIN			0x08a6
#define IMX135_FLL_MAX			0xffff
#define IMX135_FLL_STEP			1
#define IMX135_FLL_DEFAULT		0x08f2

/* HBLANK control - read only */
#define IMX135_PPL_DEFAULT		3448

/* Exposure control */
#define IMX135_REG_EXPOSURE		0x0202
#define IMX135_EXPOSURE_MIN		4
#define IMX135_EXPOSURE_STEP		1
#define IMX135_EXPOSURE_DEFAULT		0x664
#define IMX135_EXPOSURE_MAX		0xfffb

/* Analog gain control */
#define IMX135_REG_ANALOG_GAIN		0x0205
#define IMX135_ANA_GAIN_MIN		0
#define IMX135_ANA_GAIN_MAX		0xff
#define IMX135_ANA_GAIN_STEP		1
#define IMX135_ANA_GAIN_DEFAULT		0x0

#define IMX135_REG_SHORT_AGC_GAIN	0x0233

/*
 * Digital gain control, there are 8 registers that control this value.
 * Which means that the Digital Gain Control needs a bit of tweaking
 * in the driver, such that the sensor maintains a good white-balance.
 */
#define IMX135_REG_DIGITAL_GAIN		0x020e	/* The first Digital Gain Reg */
#define IMX135_DGTL_GAIN_MIN		0
#define IMX135_DGTL_GAIN_MAX		15
#define IMX135_DGTL_GAIN_DEFAULT	4
#define IMX135_DGTL_GAIN_STEP		1

#define IMX135_REG_ORIENTATION		0x0101

/* Binning  Mode */
#define IMX135_REG_BINNING_ENABLE	0x0390
#define IMX135_REG_BINNING_FACTOR	0x0391
#define IMX135_BINNING_NONE		0x00
#define IMX135_BINNING_1X1		0x11
#define IMX135_BINNING_2X2		0x22

/* Test Pattern Control */
#define IMX135_REG_TEST_PATTERN		0x0600
#define IMX135_TEST_PATTERN_DISABLE	0
#define IMX135_TEST_PATTERN_SOLID_COLOR	1
#define IMX135_TEST_PATTERN_COLOR_BARS	2
#define IMX135_TEST_PATTERN_GREY_COLOR	3
#define IMX135_TEST_PATTERN_PN9		4

/* Test pattern colour components */
#define IMX135_REG_TESTP_RED		0x0602
#define IMX135_REG_TESTP_GREENR		0x0604
#define IMX135_REG_TESTP_BLUE		0x0606
#define IMX135_REG_TESTP_GREENB		0x0608
#define IMX135_TESTP_COLOUR_MIN		0
#define IMX135_TESTP_COLOUR_MAX		0x03ff
#define IMX135_TESTP_COLOUR_STEP	1
#define IMX135_TESTP_RED_DEFAULT	IMX135_TESTP_COLOUR_MAX
#define IMX135_TESTP_GREENR_DEFAULT	0
#define IMX135_TESTP_BLUE_DEFAULT	0
#define IMX135_TESTP_GREENB_DEFAULT	0

/* IMX135 native and active pixel array size. */
#define IMX135_NATIVE_WIDTH		3296U
#define IMX135_NATIVE_HEIGHT		2480U
#define IMX135_PIXEL_ARRAY_LEFT		8U
#define IMX135_PIXEL_ARRAY_TOP		8U
#define IMX135_PIXEL_ARRAY_WIDTH	3280U
#define IMX135_PIXEL_ARRAY_HEIGHT	2464U

enum pad_types {
	IMAGE_PAD
};

struct imx135_reg {
	u16 address;
	u8 val;
};

struct imx135_reg_list {
	unsigned int num_of_regs;
	const struct imx135_reg *regs;
};

/* Mode : resolution and related config&values */
struct imx135_mode {
	/* Frame width */
	unsigned int width;
	/* Frame height */
	unsigned int height;

	/* V-timing */
	unsigned int vts_def;

	/* Default register values */
	struct imx135_reg_list reg_list;
};

/* No common regs yet; well, we need to organize them once the resolution set is final */
static const struct imx135_reg imx135_common_regs[] = {
};

static const struct imx135_reg mode_1280_720_video_regs[] = {
	/* global settings */
	{0x0101, 0x00},
	{0x0105, 0x01},
	{0x0110, 0x00},
	{0x0220, 0x01},
	{0x3302, 0x11},
	{0x3833, 0x20},
	{0x3893, 0x00},
	{0x3906, 0x08},
	{0x3907, 0x01},
	{0x391B, 0x01},
	{0x3C09, 0x01},
	{0x600A, 0x00},
	{0x3008, 0xB0},
	{0x320A, 0x01},
	{0x320D, 0x10},
	{0x3216, 0x2E},
	{0x322C, 0x02},
	{0x3409, 0x0C},
	{0x340C, 0x2D},
	{0x3411, 0x39},
	{0x3414, 0x1E},
	{0x3427, 0x04},
	{0x3480, 0x1E},
	{0x3484, 0x1E},
	{0x3488, 0x1E},
	{0x348C, 0x1E},
	{0x3490, 0x1E},
	{0x3494, 0x1E},
	{0x3511, 0x8F},
	{0x364F, 0x2D},
	/* Clock Setting */
	{0x011E, 0x18},
	{0x011F, 0x00},
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0305, 0x0C},
	{0x0309, 0x0A},
	{0x030B, 0x02},
	{0x030C, 0x01},
	{0x030D, 0xC2},
	{0x030E, 0x01},
	{0x3A06, 0x12},
	/* Mode Settings */
	{0x0108, 0x01},
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0390, 0x01},
	{0x0391, 0x22},
	{0x0392, 0x00},
	{0x0401, 0x02},
	{0x0404, 0x00},
	{0x0405, 0x1A},
	{0x4082, 0x00},
	{0x4083, 0x00},
	{0x7006, 0x04},
	/* Optional/Function settings */
	{0x0700, 0x00},
	{0x3A63, 0x00},
	{0x4100, 0xF8},
	{0x4203, 0xFF},
	{0x4344, 0x00},
	{0x441C, 0x01},
	/* Size Setting */
	{0x0340, 0x0A},
	{0x0341, 0x40},
	{0x0342, 0x11},
	{0x0343, 0xDC},
	{0x0344, 0x00},
	{0x0345, 0x18},
	{0x0346, 0x01},
	{0x0347, 0x88},
	{0x0348, 0x10},
	{0x0349, 0x57},
	{0x034A, 0x0A},
	{0x034B, 0xAB},
	{0x034C, 0x05},
	{0x034D, 0x00},
	{0x034E, 0x02},
	{0x034F, 0xD0},
	{0x0350, 0x00},
	{0x0351, 0x00},
	{0x0352, 0x00},
	{0x0353, 0x00},
	{0x0354, 0x08},
	{0x0355, 0x20},
	{0x0356, 0x04},
	{0x0357, 0x92},
	{0x301D, 0x30},
	{0x3310, 0x05},
	{0x3311, 0x00},
	{0x3312, 0x02},
	{0x3313, 0xD0},
	{0x331C, 0x02},
	{0x331D, 0x18},
	{0x4084, 0x05},
	{0x4085, 0x00},
	{0x4086, 0x02},
	{0x4087, 0xD0},
	{0x4400, 0x00},
	/* Global Timing Setting */
	{0x0830, 0x67},
	{0x0831, 0x27},
	{0x0832, 0x47},
	{0x0833, 0x27},
	{0x0834, 0x27},
	{0x0835, 0x1F},
	{0x0836, 0x87},
	{0x0837, 0x2F},
	{0x0839, 0x1F},
	{0x083A, 0x17},
	{0x083B, 0x02},
	/* Integration Time Setting */
	{0x0202, 0x0A},
	{0x0203, 0x3C},
	/* Gain Setting */
	{0x0205, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	/* HDR Setting */
	{0x0230, 0x00},
	{0x0231, 0x00},
	{0x0233, 0x00},
	{0x0234, 0x00},
	{0x0235, 0x40},
	{0x0238, 0x01},
	{0x0239, 0x04},
	{0x023B, 0x00},
	{0x023C, 0x01},
	{0x33B0, 0x04},
	{0x33B1, 0x00},
	{0x33B3, 0x00},
	{0x33B4, 0x01},
	{0x3800, 0x00},
	{0x3A43, 0x01},};

static const struct imx135_reg mode_1920_1080_video_regs[] = {
	/* global settings */
	{0x0101, 0x00},
	{0x0105, 0x01},
	{0x0110, 0x00},
	{0x0220, 0x01},
	{0x3302, 0x11},
	{0x3833, 0x20},
	{0x3893, 0x00},
	{0x3906, 0x08},
	{0x3907, 0x01},
	{0x391B, 0x01},
	{0x3C09, 0x01},
	{0x600A, 0x00},
	{0x3008, 0xB0},
	{0x320A, 0x01},
	{0x320D, 0x10},
	{0x3216, 0x2E},
	{0x322C, 0x02},
	{0x3409, 0x0C},
	{0x340C, 0x2D},
	{0x3411, 0x39},
	{0x3414, 0x1E},
	{0x3427, 0x04},
	{0x3480, 0x1E},
	{0x3484, 0x1E},
	{0x3488, 0x1E},
	{0x348C, 0x1E},
	{0x3490, 0x1E},
	{0x3494, 0x1E},
	{0x3511, 0x8F},
	{0x364F, 0x2D},
	/* Clock Setting */
	{0x011E, 0x18},
	{0x011F, 0x00},
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0305, 0x0C},
	{0x0309, 0x0A},
	{0x030B, 0x02},
	{0x030C, 0x01},
	{0x030D, 0xC2},
	{0x030E, 0x01},
	{0x3A06, 0x12},
	/* Mode Settings */
	{0x0108, 0x01},
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0390, 0x01},
	{0x0391, 0x22},
	{0x0392, 0x00},
	{0x0401, 0x02},
	{0x0404, 0x00},
	{0x0405, 0x11},
	{0x4082, 0x00},
	{0x4083, 0x00},
	{0x7006, 0x04},
	/* Optional/Function settings */
	{0x0700, 0x00},
	{0x3A63, 0x00},
	{0x4100, 0xF8},
	{0x4203, 0xFF},
	{0x4344, 0x00},
	{0x441C, 0x01},
	/* Size Setting */
	{0x0340, 0x04},
	{0x0341, 0xCA},
	{0x0342, 0x23},
	{0x0343, 0xB8},
	{0x0344, 0x00},
	{0x0345, 0x40},
	{0x0346, 0x01},
	{0x0347, 0x9C},
	{0x0348, 0x10},
	{0x0349, 0x2F},
	{0x034A, 0x0A},
	{0x034B, 0x93},
	{0x034C, 0x07},
	{0x034D, 0x80},
	{0x034E, 0x04},
	{0x034F, 0x38},
	{0x0350, 0x00},
	{0x0351, 0x00},
	{0x0352, 0x00},
	{0x0353, 0x00},
	{0x0354, 0x07},
	{0x0355, 0xF8},
	{0x0356, 0x04},
	{0x0357, 0x7C},
	{0x301D, 0x30},
	{0x3310, 0x07},
	{0x3311, 0x80},
	{0x3312, 0x04},
	{0x3313, 0x38},
	{0x331C, 0x00},
	{0x331D, 0xD2},
	{0x4084, 0x07},
	{0x4085, 0x80},
	{0x4086, 0x04},
	{0x4087, 0x38},
	{0x4400, 0x00},
	/* Global Timing Setting */
	{0x0830, 0x67},
	{0x0831, 0x27},
	{0x0832, 0x47},
	{0x0833, 0x27},
	{0x0834, 0x27},
	{0x0835, 0x1F},
	{0x0836, 0x87},
	{0x0837, 0x2F},
	{0x0839, 0x1F},
	{0x083A, 0x17},
	{0x083B, 0x02},
	/* Integration Time Setting */
	{0x0202, 0x0A},
	{0x0203, 0x3C},
	/* Gain Setting */
	{0x0205, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	/* HDR Setting */
	{0x0230, 0x00},
	{0x0231, 0x00},
	{0x0233, 0x00},
	{0x0234, 0x00},
	{0x0235, 0x40},
	{0x0238, 0x01},
	{0x0239, 0x04},
	{0x023B, 0x00},
	{0x023C, 0x01},
	{0x33B0, 0x04},
	{0x33B1, 0x00},
	{0x33B3, 0x00},
	{0x33B4, 0x01},
	{0x3800, 0x00},
	{0x3A43, 0x01},
};

static const struct imx135_reg mode_2104_1560_video_regs[] = {
	{0x0101, 0x00},
	{0x0105, 0x01},
	{0x0110, 0x00},
	{0x0220, 0x01},
	{0x3302, 0x11},
	{0x3833, 0x20},
	{0x3893, 0x00},
	{0x3906, 0x08},
	{0x3907, 0x01},
	{0x391b, 0x01},
	{0x3c09, 0x01},
	{0x600a, 0x00},
	{0x3008, 0xb0},
	{0x320a, 0x01},
	{0x320d, 0x10},
	{0x3216, 0x2e},
	{0x322c, 0x02},
	{0x3409, 0x0c},
	{0x340c, 0x2d},
	{0x3411, 0x39},
	{0x3414, 0x1e},
	{0x3427, 0x04},
	{0x3480, 0x1e},
	{0x3484, 0x1e},
	{0x3488, 0x1e},
	{0x348c, 0x1e},
	{0x3490, 0x1e},
	{0x3494, 0x1e},
	{0x3511, 0x8f},
	{0x364f, 0x2d},

	/* quality */

	/* defect correction recommended setting */

	{0x380a, 0x00},
	{0x380b, 0x00},
	{0x4103, 0x00},

	/* color artifact recommended setting */

	{0x4243, 0x9a},
	{0x4330, 0x01},
	{0x4331, 0x90},
	{0x4332, 0x02},
	{0x4333, 0x58},
	{0x4334, 0x03},
	{0x4335, 0x20},
	{0x4336, 0x03},
	{0x4337, 0x84},
	{0x433c, 0x01},
	{0x4340, 0x02},
	{0x4341, 0x58},
	{0x4342, 0x03},
	{0x4343, 0x52},

	/* Moire reduction parameter setting */

	{0x4364, 0x0b},
	{0x4368, 0x00},
	{0x4369, 0x0f},
	{0x436a, 0x03},
	{0x436b, 0xa8},
	{0x436c, 0x00},
	{0x436d, 0x00},
	{0x436e, 0x00},
	{0x436f, 0x06},

	/* CNR parameter setting */

	{0x4281, 0x21},
	{0x4282, 0x18},
	{0x4283, 0x04},
	{0x4284, 0x08},
	{0x4287, 0x7f},
	{0x4288, 0x08},
	{0x428b, 0x7f},
	{0x428c, 0x08},
	{0x428f, 0x7f},
	{0x4297, 0x00},
	{0x4298, 0x7e},
	{0x4299, 0x7e},
	{0x429a, 0x7e},
	{0x42a4, 0xfb},
	{0x42a5, 0x7e},
	{0x42a6, 0xdf},
	{0x42a7, 0xb7},
	{0x42af, 0x03},

	/* ARNR Parameter setting */
	{0x4207, 0x03},
	{0x4216, 0x08},
	{0x4217, 0x08},

	/* DLC Parammeter setting */
	{0x4218, 0x00},
	{0x421b, 0x20},
	{0x421f, 0x04},
	{0x4222, 0x02},
	{0x4223, 0x22},
	{0x422e, 0x54},
	{0x422f, 0xfb},
	{0x4230, 0xff},
	{0x4231, 0xfe},
	{0x4232, 0xff},
	{0x4235, 0x58},
	{0x4236, 0xf7},
	{0x4237, 0xfd},
	{0x4239, 0x4e},
	{0x423a, 0xfc},
	{0x423b, 0xfd},

	/* HDR */

	/* LSC setting */
	{0x452a, 0x02},

	/* white balance setting */
	{0x0712, 0x01},
	{0x0713, 0x00},
	{0x0714, 0x01},
	{0x0715, 0x00},
	{0x0716, 0x01},
	{0x0717, 0x00},
	{0x0718, 0x01},
	{0x0719, 0x00},

	/* shading setting */
	{0x4500, 0x1f},

	{0x0100, 0x00},

	/* PLL setting */
	{0x011e, 0x18},
	{0x011f, 0x00},
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0305, 0x0b},
	{0x0309, 0x0a},
	{0x030b, 0x01},
	{0x030c, 0x01},
	{0x030d, 0x09},
	{0x030e, 0x01},
	{0x3a06, 0x11},

	/* Mode setting */
	{0x0108, 0x01},
	{0x0112, 0x0a},
	{0x0113, 0x0a},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0390, 0x01},
	{0x0391, 0x22},
	{0x0392, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x4082, 0x01},
	{0x4083, 0x01},
	{0x7006, 0x04},

	/* Optional function setting */
	{0x0700, 0x00},
	{0x3a63, 0x00},
	{0x4100, 0xf8},
	{0x4203, 0xff},
	{0x4344, 0x00},
	{0x441c, 0x01},

	/* Size setting */
	{0x0340, 0x06},//0a
	{0x0341, 0x68},//40
	{0x0342, 0x11},
	{0x0343, 0xdc},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x10},
	{0x0349, 0x6f},
	{0x034a, 0x0c},
	{0x034b, 0x2f},
	{0x034c, 0x08},
	{0x034d, 0x38},
	{0x034e, 0x06},
	{0x034f, 0x18},
	{0x0350, 0x00},
	{0x0351, 0x00},
	{0x0352, 0x00},
	{0x0353, 0x00},
	{0x0354, 0x08},
	{0x0355, 0x38},
	{0x0356, 0x06},
	{0x0357, 0x18},
	{0x301d, 0x30},
	{0x3310, 0x08},
	{0x3311, 0x38},
	{0x3312, 0x06},
	{0x3313, 0x18},
	{0x331c, 0x04},
	{0x331d, 0xab},
	{0x4084, 0x00},
	{0x4085, 0x00},
	{0x4086, 0x00},
	{0x4087, 0x00},
	{0x4400, 0x00},

	/* global timing setting */
	{0x0830, 0x6f},
	{0x0831, 0x27},
	{0x0832, 0x4f},
	{0x0833, 0x2f},
	{0x0834, 0x2f},
	{0x0835, 0x2f},
	{0x0836, 0x9f},
	{0x0837, 0x37},
	{0x0839, 0x1f},
	{0x083a, 0x17},
	{0x083b, 0x02},
};

/* Digital gain values, which maintain (a relatively good) white-balance */
static const int imx135_digital_gain_base_reg_values[] = {
	0x01, 0x00, 0x02, 0x10, 0x02, 0x10, 0x01, 0x00
};

static const char * const imx135_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Solid Color",
	"Grey Color Bars",
	"PN9"
};

static const int imx135_test_pattern_val[] = {
	IMX135_TEST_PATTERN_DISABLE,
	IMX135_TEST_PATTERN_COLOR_BARS,
	IMX135_TEST_PATTERN_SOLID_COLOR,
	IMX135_TEST_PATTERN_GREY_COLOR,
	IMX135_TEST_PATTERN_PN9,
};

/* regulator supplies */
static const char * const imx135_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.8V) supply */
	"VDDL",  /* IF (1.2V) supply */
};

#define IMX135_NUM_SUPPLIES ARRAY_SIZE(imx135_supply_name)

/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 codes[] = {
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

#define IMX135_XCLR_MIN_DELAY_US	30000
#define IMX135_XCLR_DELAY_RANGE_US	10000

/* Mode configs */
static const struct imx135_mode supported_modes[] = {
	{
		.width = 1280,
		.height = 720,
		.vts_def = 4572,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1280_720_video_regs),
			.regs = mode_1280_720_video_regs,
		},
	},
	{
		.width = 1920,
		.height = 1080,
		.vts_def = 9144,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920_1080_video_regs),
			.regs = mode_1920_1080_video_regs,
		},
	},
	{
		.width = 2104,
		.height = 1560,
		.vts_def = 9144,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2104_1560_video_regs),
			.regs = mode_2104_1560_video_regs,
		},
	},
};

struct imx135 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_mbus_framefmt fmt;

	struct clk *xclk; /* system clock to IMX135 */
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX135_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct imx135_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
};

static inline struct imx135 *to_imx135(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx135, sd);
}

/* Read registers up to 2 at a time */
static int imx135_read_reg(struct imx135 *imx135, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx135->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/* Write registers up to 2 at a time */
static int imx135_write_reg(struct imx135 *imx135, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx135->sd);
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/* Write a list of registers */
static int imx135_write_regs(struct imx135 *imx135,
			     const struct imx135_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx135->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = imx135_write_reg(imx135, regs[i].address,
				       IMX135_REG_VALUE_08BIT, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

/* Get bayer order based on flip setting. */
static u32 imx135_get_format_code(struct imx135 *imx135, u32 code)
{
	unsigned int i;

	lockdep_assert_held(&imx135->mutex);

	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == code)
			break;

	if (i >= ARRAY_SIZE(codes))
		i = 0;

	i = (i & ~3) | (imx135->vflip->val ? 2 : 0) |
	    (imx135->hflip->val ? 1 : 0);

	return codes[i];
}

static void imx135_set_default_format(struct imx135 *imx135)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = &imx135->fmt;
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = supported_modes[0].width;
	fmt->height = supported_modes[0].height;
	fmt->field = V4L2_FIELD_NONE;
}

static int imx135_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx135 *imx135 = to_imx135(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_state_get_format(fh->state, IMAGE_PAD);

	mutex_lock(&imx135->mutex);

	/* Initialize try_fmt */
	try_fmt->width = supported_modes[0].width;
	try_fmt->height = supported_modes[0].height;
	try_fmt->code = imx135_get_format_code(imx135,
					       MEDIA_BUS_FMT_SRGGB10_1X10);
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&imx135->mutex);

	return 0;
}

static int imx135_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx135 *imx135 =
		container_of(ctrl->handler, struct imx135, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx135->sd);
	int i, ret;

	if (ctrl->id == V4L2_CID_VBLANK) {
		int exposure_max, exposure_def;

		/* Update max exposure while meeting expected vblanking */
		exposure_max = imx135->mode->height + ctrl->val - 4;
		exposure_def = (exposure_max < IMX135_EXPOSURE_DEFAULT) ?
			exposure_max : IMX135_EXPOSURE_DEFAULT;
		__v4l2_ctrl_modify_range(imx135->exposure,
					 imx135->exposure->minimum,
					 exposure_max, imx135->exposure->step,
					 exposure_def);
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx135_write_reg(imx135, IMX135_REG_ANALOG_GAIN,
				       IMX135_REG_VALUE_08BIT, ctrl->val);
		ret |= imx135_write_reg(imx135, IMX135_REG_SHORT_AGC_GAIN,
					IMX135_REG_VALUE_08BIT, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx135_write_reg(imx135, IMX135_REG_EXPOSURE,
				       IMX135_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = 0;
		/* IMX135 (and similar parts) have 8 Digital Gain Registers */
		for (i = 0; i < 8; i++) {
			u32 val = imx135_digital_gain_base_reg_values[i];

			pr_err("%s %d reg %04x val %02x\n", __func__, __LINE__, IMX135_REG_DIGITAL_GAIN + i, val);
			val *= ctrl->val;
			ret |= imx135_write_reg(imx135,
						IMX135_REG_DIGITAL_GAIN + i,
						IMX135_REG_VALUE_08BIT, val);
		}
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx135_write_reg(imx135, IMX135_REG_TEST_PATTERN,
				       IMX135_REG_VALUE_16BIT,
				       imx135_test_pattern_val[ctrl->val]);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = imx135_write_reg(imx135, IMX135_REG_ORIENTATION, 1,
				       imx135->hflip->val |
				       imx135->vflip->val << 1);
		break;
	case V4L2_CID_VBLANK:
		ret = imx135_write_reg(imx135, IMX135_REG_VTS,
				       IMX135_REG_VALUE_16BIT,
				       imx135->mode->height + ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		ret = imx135_write_reg(imx135, IMX135_REG_TESTP_RED,
				       IMX135_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = imx135_write_reg(imx135, IMX135_REG_TESTP_GREENR,
				       IMX135_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = imx135_write_reg(imx135, IMX135_REG_TESTP_BLUE,
				       IMX135_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = imx135_write_reg(imx135, IMX135_REG_TESTP_GREENB,
				       IMX135_REG_VALUE_16BIT, ctrl->val);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx135_ctrl_ops = {
	.s_ctrl = imx135_set_ctrl,
};

static int imx135_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx135 *imx135 = to_imx135(sd);

	if (code->pad == IMAGE_PAD) {
		if (code->index >= (ARRAY_SIZE(codes) / 4))
			return -EINVAL;

		code->code = imx135_get_format_code(imx135, codes[code->index * 4]);
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}
	return 0;
}

static int imx135_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx135 *imx135 = to_imx135(sd);

	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != imx135_get_format_code(imx135, fse->code))
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void imx135_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx135_update_pad_format(struct imx135 *imx135,
				     const struct imx135_mode *mode,
				     struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx135_reset_colorspace(&fmt->format);
}

static int __imx135_get_pad_format(struct imx135 *imx135,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_state_get_format(sd_state, fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = imx135_get_format_code(imx135, try_fmt->code);
		fmt->format = *try_fmt;
	} else {
		imx135_update_pad_format(imx135, imx135->mode, fmt);
		fmt->format.code = imx135_get_format_code(imx135,
							  imx135->fmt.code);
	}

	return 0;
}

static int imx135_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx135 *imx135 = to_imx135(sd);
	int ret;

	mutex_lock(&imx135->mutex);
	ret = __imx135_get_pad_format(imx135, sd_state, fmt);
	mutex_unlock(&imx135->mutex);

	return ret;
}

static int imx135_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx135 *imx135 = to_imx135(sd);
	const struct imx135_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;
	int exposure_max, exposure_def, hblank;
	unsigned int i;

	mutex_lock(&imx135->mutex);

	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == fmt->format.code)
			break;
	if (i >= ARRAY_SIZE(codes))
		i = 0;

	/* Bayer order varies with flips */
	fmt->format.code = imx135_get_format_code(imx135, codes[i]);

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);
	imx135_update_pad_format(imx135, mode, fmt);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		*framefmt = fmt->format;
	} else if (imx135->mode != mode ||
		   imx135->fmt.code != fmt->format.code) {
		imx135->fmt = fmt->format;
		imx135->mode = mode;
		/* Update limits and set FPS to default */
		__v4l2_ctrl_modify_range(imx135->vblank, IMX135_VBLANK_MIN,
					 IMX135_VTS_MAX - mode->height, 1,
					 mode->vts_def - mode->height);
		__v4l2_ctrl_s_ctrl(imx135->vblank,
				   mode->vts_def - mode->height);
		/* Update max exposure while meeting expected vblanking */
		exposure_max = mode->vts_def - 4;
		exposure_def = (exposure_max < IMX135_EXPOSURE_DEFAULT) ?
			exposure_max : IMX135_EXPOSURE_DEFAULT;
		__v4l2_ctrl_modify_range(imx135->exposure,
					 imx135->exposure->minimum,
					 exposure_max, imx135->exposure->step,
					 exposure_def);
		/*
		 * Currently PPL is IMX135_PPL_DEFAULT, so hblank
		 * depends on mode->width only, and is not changeble in any
		 * way other than changing the mode.
		 */
		hblank = IMX135_PPL_DEFAULT - mode->width;
		__v4l2_ctrl_modify_range(imx135->hblank, hblank, hblank, 1,
					 hblank);
	}

	mutex_unlock(&imx135->mutex);

	return 0;
}

static int imx135_set_framefmt(struct imx135 *imx135)
{
	/* Not implemented yet */

	switch (imx135->fmt.code) {
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		return 0;
	}

	return -EINVAL;
}

static int imx135_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = IMX135_NATIVE_WIDTH;
		sel->r.height = IMX135_NATIVE_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int imx135_start_streaming(struct imx135 *imx135)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx135->sd);
	const struct imx135_reg_list *reg_list;
	int ret;

	ret = pm_runtime_get_sync(&client->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(&client->dev);
		return ret;
	}

	/* Send all registers that are common to all modes */
	ret = imx135_write_regs(imx135, imx135_common_regs, ARRAY_SIZE(imx135_common_regs));
	if (ret) {
		dev_err(&client->dev, "%s failed to send common regs\n", __func__);
		goto err_rpm_put;
	}

	/* Apply default values of current mode */
	reg_list = &imx135->mode->reg_list;
	ret = imx135_write_regs(imx135, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		goto err_rpm_put;
	}

	ret = imx135_set_framefmt(imx135);
	if (ret) {
		dev_err(&client->dev, "%s failed to set frame format: %d\n",
			__func__, ret);
		goto err_rpm_put;
	}

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx135->sd.ctrl_handler);
	if (ret)
		goto err_rpm_put;

	/* set stream on register */
	ret = imx135_write_reg(imx135, IMX135_REG_MODE_SELECT,
			       IMX135_REG_VALUE_08BIT, IMX135_MODE_STREAMING);
	if (ret)
		goto err_rpm_put;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx135->vflip, true);
	__v4l2_ctrl_grab(imx135->hflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static void imx135_stop_streaming(struct imx135 *imx135)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx135->sd);
	int ret;

	/* set stream off register */
	ret = imx135_write_reg(imx135, IMX135_REG_MODE_SELECT,
			       IMX135_REG_VALUE_08BIT, IMX135_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);

	__v4l2_ctrl_grab(imx135->vflip, false);
	__v4l2_ctrl_grab(imx135->hflip, false);

	pm_runtime_put(&client->dev);
}

static int imx135_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx135 *imx135 = to_imx135(sd);
	int ret = 0;

	mutex_lock(&imx135->mutex);
	if (imx135->streaming == enable) {
		mutex_unlock(&imx135->mutex);
		return 0;
	}

	if (enable) {
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx135_start_streaming(imx135);
		if (ret)
			goto err_unlock;
	} else {
		imx135_stop_streaming(imx135);
	}

	imx135->streaming = enable;

	mutex_unlock(&imx135->mutex);

	return ret;

err_unlock:
	mutex_unlock(&imx135->mutex);

	return ret;
}

/* Power/clock management functions */
static int imx135_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx135 *imx135 = to_imx135(sd);
	int ret;

	ret = regulator_bulk_enable(IMX135_NUM_SUPPLIES,
				    imx135->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx135->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx135->reset_gpio, 1);
	usleep_range(IMX135_XCLR_MIN_DELAY_US,
		     IMX135_XCLR_MIN_DELAY_US + IMX135_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(IMX135_NUM_SUPPLIES, imx135->supplies);

	return ret;
}

static int imx135_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx135 *imx135 = to_imx135(sd);

	return 0;
	gpiod_set_value_cansleep(imx135->reset_gpio, 0);
	regulator_bulk_disable(IMX135_NUM_SUPPLIES, imx135->supplies);
	clk_disable_unprepare(imx135->xclk);

	return 0;
}

static int __maybe_unused imx135_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx135 *imx135 = to_imx135(sd);

	if (imx135->streaming)
		imx135_stop_streaming(imx135);

	return 0;
}

static int __maybe_unused imx135_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx135 *imx135 = to_imx135(sd);
	int ret;

	if (imx135->streaming) {
		ret = imx135_start_streaming(imx135);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx135_stop_streaming(imx135);
	imx135->streaming = false;

	return ret;
}

static int imx135_get_regulators(struct imx135 *imx135)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx135->sd);
	unsigned int i;

	for (i = 0; i < IMX135_NUM_SUPPLIES; i++)
		imx135->supplies[i].supply = imx135_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       IMX135_NUM_SUPPLIES,
				       imx135->supplies);
}

/* Verify chip ID */
static int imx135_identify_module(struct imx135 *imx135)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx135->sd);
	int ret;
	u32 val;

	ret = imx135_read_reg(imx135, IMX135_REG_CHIP_ID,
			      IMX135_REG_VALUE_16BIT, &val);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x\n",
			IMX135_CHIP_ID);
		return ret;
	}

	if (val != IMX135_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x\n",
			IMX135_CHIP_ID, val);
		return -EIO;
	}

	return 0;
}

static const struct v4l2_subdev_core_ops imx135_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx135_video_ops = {
	.s_stream = imx135_set_stream,
};

static const struct v4l2_subdev_pad_ops imx135_pad_ops = {
	.enum_mbus_code = imx135_enum_mbus_code,
	.get_fmt = imx135_get_pad_format,
	.set_fmt = imx135_set_pad_format,
	.get_selection = imx135_get_selection,
	.enum_frame_size = imx135_enum_frame_size,
};

static const struct v4l2_subdev_ops imx135_subdev_ops = {
	.core = &imx135_core_ops,
	.video = &imx135_video_ops,
	.pad = &imx135_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx135_internal_ops = {
	.open = imx135_open,
};

/* Initialize control handlers */
static int imx135_init_controls(struct imx135 *imx135)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx135->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	unsigned int height = imx135->mode->height;
	struct v4l2_fwnode_device_properties props;
	int exposure_max, exposure_def, hblank;
	int i, ret;

	ctrl_hdlr = &imx135->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 11);
	if (ret)
		return ret;

	mutex_init(&imx135->mutex);
	ctrl_hdlr->lock = &imx135->mutex;

	/* By default, PIXEL_RATE is read only */
	imx135->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx135_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       IMX135_PIXEL_RATE,
					       IMX135_PIXEL_RATE, 1,
					       IMX135_PIXEL_RATE);

	/* Initial vblank/hblank/exposure parameters based on current mode */
	imx135->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx135_ctrl_ops,
					   V4L2_CID_VBLANK, IMX135_VBLANK_MIN,
					   IMX135_VTS_MAX - height, 1,
					   imx135->mode->vts_def - height);
	hblank = IMX135_PPL_DEFAULT - imx135->mode->width;
	imx135->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx135_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank,
					   1, hblank);
	if (imx135->hblank)
		imx135->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	exposure_max = imx135->mode->vts_def - 4;
	exposure_def = (exposure_max < IMX135_EXPOSURE_DEFAULT) ?
		exposure_max : IMX135_EXPOSURE_DEFAULT;
	imx135->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx135_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX135_EXPOSURE_MIN, exposure_max,
					     IMX135_EXPOSURE_STEP,
					     exposure_def);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx135_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX135_ANA_GAIN_MIN, IMX135_ANA_GAIN_MAX,
			  IMX135_ANA_GAIN_STEP, IMX135_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx135_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX135_DGTL_GAIN_MIN, IMX135_DGTL_GAIN_MAX,
			  IMX135_DGTL_GAIN_STEP, IMX135_DGTL_GAIN_DEFAULT);

	imx135->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx135_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx135->hflip)
		imx135->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx135->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx135_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx135->vflip)
		imx135->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx135_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx135_test_pattern_menu) - 1,
				     0, 0, imx135_test_pattern_menu);
	for (i = 0; i < 4; i++) {
		/*
		 * The assumption is that
		 * V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		 * V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		 * V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		 */
		v4l2_ctrl_new_std(ctrl_hdlr, &imx135_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  IMX135_TESTP_COLOUR_MIN,
				  IMX135_TESTP_COLOUR_MAX,
				  IMX135_TESTP_COLOUR_STEP,
				  IMX135_TESTP_COLOUR_MAX);
		/* The "Solid color" pattern is white by default */
	}

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx135_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	imx135->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx135->mutex);

	return ret;
}

static void imx135_free_controls(struct imx135 *imx135)
{
	v4l2_ctrl_handler_free(imx135->sd.ctrl_handler);
	mutex_destroy(&imx135->mutex);
}

static int imx135_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
		dev_err(dev, "only 2 data lanes are currently supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != 1 ||
	    ep_cfg.link_frequencies[0] != IMX135_DEFAULT_LINK_FREQ) {
		dev_err(dev, "Link frequency not supported: %lld\n",
			ep_cfg.link_frequencies[0]);
		goto error_out;
	}

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static int imx135_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx135 *imx135;
	int ret;

	imx135 = devm_kzalloc(&client->dev, sizeof(*imx135), GFP_KERNEL);
	if (!imx135)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx135->sd, client, &imx135_subdev_ops);

	/* Check the hardware configuration in device tree */
	if (imx135_check_hwcfg(dev))
		return -EINVAL;

	/* Get system clock (xclk) */
	imx135->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx135->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx135->xclk);
	}

	imx135->xclk_freq = clk_get_rate(imx135->xclk);
	if (imx135->xclk_freq != IMX135_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			imx135->xclk_freq);
		return -EINVAL;
	}

	ret = imx135_get_regulators(imx135);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	imx135->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for imx135_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = imx135_power_on(dev);
	if (ret)
		return ret;

	ret = imx135_identify_module(imx135);
	if (ret)
		goto error_power_off;

	/* Set default mode to max resolution */
	imx135->mode = &supported_modes[0];

	/* sensor doesn't enter LP-11 state upon power up until and unless
	 * streaming is started, so upon power up switch the modes to:
	 * streaming -> standby
	 */
	ret = imx135_write_reg(imx135, IMX135_REG_MODE_SELECT,
			       IMX135_REG_VALUE_08BIT, IMX135_MODE_STREAMING);
	if (ret < 0)
		goto error_power_off;
	usleep_range(100, 110);

	/* put sensor back to standby mode */
	ret = imx135_write_reg(imx135, IMX135_REG_MODE_SELECT,
			       IMX135_REG_VALUE_08BIT, IMX135_MODE_STANDBY);
	if (ret < 0)
		goto error_power_off;
	usleep_range(100, 110);

	ret = imx135_init_controls(imx135);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	imx135->sd.internal_ops = &imx135_internal_ops;
	imx135->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx135->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	imx135->pad.flags = MEDIA_PAD_FL_SOURCE;

	/* Initialize default format */
	imx135_set_default_format(imx135);

	ret = media_entity_pads_init(&imx135->sd.entity, 1, &imx135->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx135->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&imx135->sd.entity);

error_handler_free:
	imx135_free_controls(imx135);

error_power_off:
	imx135_power_off(dev);

	return ret;
}

static void imx135_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx135 *imx135 = to_imx135(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx135_free_controls(imx135);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx135_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id imx135_dt_ids[] = {
	{ .compatible = "sony,imx135" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx135_dt_ids);

static const struct dev_pm_ops imx135_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx135_suspend, imx135_resume)
	SET_RUNTIME_PM_OPS(imx135_power_off, imx135_power_on, NULL)
};

static struct i2c_driver imx135_i2c_driver = {
	.driver = {
		.name = "imx135",
		.of_match_table	= imx135_dt_ids,
		.pm = &imx135_pm_ops,
	},
	.probe = imx135_probe,
	.remove = imx135_remove,
};

module_i2c_driver(imx135_i2c_driver);

MODULE_AUTHOR("Alexandru Ardelean <alex@shruggie.ro");
MODULE_DESCRIPTION("Sony IMX135 sensor driver");
