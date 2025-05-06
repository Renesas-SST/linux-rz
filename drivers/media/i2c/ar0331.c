// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Aptina/OnSemi AR0331 cameras.
 *
 * Based on Sony imx219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <asm/unaligned.h>

#define AR0331_REG_VALUE_08BIT		1
#define AR0331_REG_VALUE_16BIT		2

/* Chip ID */
#define AR0331_REG_CHIP_ID		0x3000
#define AR0330_CHIP_ID			0x2604
#define AR0331_CHIP_ID			0x0000 // look for this FIXME

/* External clock frequency is 24.0M */
#define AR0331_XCLK_FREQ		24000000

/* Pixel rate is fixed at 182.4M for all the modes */
#define AR0331_PIXEL_RATE		182400000

/* V_TIMING internal */
#define AR0331_REG_VTS			0x300A
#define AR0331_VTS_MIN			0x0004
#define AR0331_VTS_MAX			0xffff
#define AR0331_VTS_DEF			1308

/* Exposure control */
#define AR0331_REG_EXPOSURE		0x3012
#define AR0331_EXPOSURE_MIN		4
#define AR0331_EXPOSURE_STEP		1
#define AR0331_EXPOSURE_DEFAULT		0x640
#define AR0331_EXPOSURE_MAX		65535

/* Analog gain control */
#define AR0331_REG_ANALOG_GAIN		0x3060
#define AR0331_ANA_GAIN_MIN		0
#define AR0331_ANA_GAIN_MAX		65535
#define AR0331_ANA_GAIN_STEP		1
#define AR0331_ANA_GAIN_DEFAULT		0x0

/* Digital gain control */
#define AR0331_REG_GREEN1_GAIN		0x3056
#define AR0331_REG_BLUE_GAIN		0x3058
#define AR0331_REG_RED_GAIN		0x305A
#define AR0331_REG_GREEN2_GAIN		0x305C
#define AR0331_DGTL_GAIN_MIN		0x0000
#define AR0331_DGTL_GAIN_MAX		0xffff
#define AR0331_DGTL_GAIN_DEFAULT	0x0100
#define AR0331_DGTL_GAIN_STEP		1

/* AR0331 native and active pixel array size. */
#define AR0331_PIXEL_ARRAY_TOP		0U
#define AR0331_PIXEL_ARRAY_LEFT		0U
#define AR0331_PIXEL_ARRAY_WIDTH	2048U
#define AR0331_PIXEL_ARRAY_HEIGHT	1536U

#define AR0330_PIXEL_ARRAY_TOP		0U
#define AR0330_PIXEL_ARRAY_LEFT		0U
#define AR0330_PIXEL_ARRAY_WIDTH	2304U
#define AR0330_PIXEL_ARRAY_HEIGHT	1536U

#define AR0330_TABLE_WAIT_MS 0
#define AR0330_TABLE_END 1
#define AR0330_WAIT_MS 100

struct ar0331_reg {
	u16 address;
	u16 val;
};

/* Mode : resolution and related config&values */
struct ar0331_mode {
	/* Frame width */
	unsigned int width;
	/* Frame height */
	unsigned int height;

	/* V-timing */
	unsigned int vts_def;

	/* Default register values */
	const struct ar0331_reg *reg_list;
};

static const struct regmap_config ar0331_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.cache_type = REGCACHE_RBTREE,
};

/*
 * Register sets taken from
 *   https://nv-tegra.nvidia.com/r/gitweb?p=linux-3.10.git;a=commit;h=7a84cd9698c8c8ecb56b45a5a5ed20efeee6dca8
 */
static const struct ar0331_reg mode_2304x1536_regs[] = {
	{0x3052, 0xa114},
	{0x304A, 0x0070},
	{AR0330_TABLE_WAIT_MS, AR0330_WAIT_MS},
	{0x301A, 0x0058},
	{0x302A, 0x0005},
	{0x302C, 0x0004},
	{0x302E, 0x0003},
	{0x3030, 0x005F},
	{0x3036, 0x000A},
	{0x3038, 0x0001},
	{0x31AC, 0x0A0A},
	{0x31AE, 0x0201},
	{0x31B0, 0x003D},
	{0x31B2, 0x0018},
	{0x31B4, 0x4F56},
	{0x31B6, 0x4214},
	{0x31B8, 0x308B},
	{0x31BA, 0x028A},
	{0x31BC, 0x8008},
	{0x3002, 0x0006},
	{0x3004, 0x0006},
	{0x3006, 0x0605},
	{0x3008, 0x0905},
	{0x300A, 0x0611},
	{0x300C, 0x04E0},
	{0x3012, 0x0610},
	{0x3014, 0x0000},
	{0x30A2, 0x0001},
	{0x30A6, 0x0001},
	{0x3040, 0x0000},
	{0x3042, 0x0000},
	{0x30BA, 0x006C},
	{0x31E0, 0x0303},
	{0x3064, 0x1802},
	{0x3ED2, 0x0146},
	{0x3ED4, 0x8F6C},
	{0x3ED6, 0x66CC},
	{0x3ED8, 0x8C42},
	{0x3EDA, 0x88BC},
	{0x3EDC, 0xAA63},
	{0x305E, 0x00A0},
	{0x3088, 0x80BA},
	{0x3086, 0x0253},
	/* {0x30CE, 0x0010}, */
	{0x301A, 0x025C},
	{AR0330_TABLE_END, 0x00}
};

static const struct ar0331_reg mode_1280x720_regs[] = {
	{0x3052, 0xa114},
	{0x304A, 0x0070},
	{AR0330_TABLE_WAIT_MS, AR0330_WAIT_MS},
	{0x301A, 0x0058},
	{0x302A, 0x0005},
	{0x302C, 0x0004},
	{0x302E, 0x0003},
	{0x3030, 0x005F},
	{0x3036, 0x000A},
	{0x3038, 0x0001},
	{0x31AC, 0x0A0A},
	{0x31AE, 0x0201},
	{0x31B0, 0x003D},
	{0x31B2, 0x0018},
	{0x31B4, 0x4F56},
	{0x31B6, 0x4214},
	{0x31B8, 0x308B},
	{0x31BA, 0x028A},
	{0x31BC, 0x8008},
	{0x3002, 0x019E},
	{0x3004, 0x0206},
	{0x3006, 0x046D},
	{0x3008, 0x0705},
	{0x300A, 0x0449},
	{0x300C, 0x0482},
	{0x3012, 0x0448},
	{0x3014, 0x0000},
	{0x30A2, 0x0001},
	{0x30A6, 0x0001},
	{0x3040, 0x0000},
	{0x3042, 0x0000},
	{0x30BA, 0x006C},
	{0x31E0, 0x0303},
	{0x3064, 0x1802},
	{0x3ED2, 0x0146},
	{0x3ED4, 0x8F6C},
	{0x3ED6, 0x66CC},
	{0x3ED8, 0x8C42},
	{0x3EDA, 0x88BC},
	{0x3EDC, 0xAA63},
	{0x305E, 0x00A0},
	{0x3088, 0x80BA},
	{0x3086, 0x0253},
	/* {0x30CE, 0x0010}, */
	{0x301A, 0x025C},
	{AR0330_TABLE_END, 0x00}
};

static const struct ar0331_reg mode_1280x960_regs[] = {
	{0x3052, 0xa114},
	{0x304A, 0x0070},
	{AR0330_TABLE_WAIT_MS, AR0330_WAIT_MS},
	{0x301A, 0x0058},
	{0x302A, 0x0005},
	{0x302C, 0x0004},
	{0x302E, 0x0003},
	{0x3030, 0x005F},
	{0x3036, 0x000A},
	{0x3038, 0x0001},
	{0x31AC, 0x0A0A},
	{0x31AE, 0x0201},
	{0x31B0, 0x003D},
	{0x31B2, 0x0018},
	{0x31B4, 0x4F56},
	{0x31B6, 0x4214},
	{0x31B8, 0x308B},
	{0x31BA, 0x028A},
	{0x31BC, 0x8008},
	{0x3002, 0x0126},
	{0x3004, 0x0206},
	{0x3006, 0x04E5},
	{0x3008, 0x0705},
	{0x300A, 0x0449},
	{0x300C, 0x0482},
	{0x3012, 0x0448},
	{0x3014, 0x0000},
	{0x30A2, 0x0001},
	{0x30A6, 0x0001},
	{0x3040, 0x0000},
	{0x3042, 0x0000},
	{0x30BA, 0x006C},
	{0x31E0, 0x0303},
	{0x3064, 0x1802},
	{0x3ED2, 0x0146},
	{0x3ED4, 0x8F6C},
	{0x3ED6, 0x66CC},
	{0x3ED8, 0x8C42},
	{0x3EDA, 0x88BC},
	{0x3EDC, 0xAA63},
	{0x305E, 0x00A0},
	{0x3088, 0x80BA},
	{0x3086, 0x0253},
	/* {0x30CE, 0x0010}, */
	{0x301A, 0x025C},
	{AR0330_TABLE_END, 0x00}
};

static struct ar0331_reg tp_colorbar[] = {
	{0x301A, 0x0019},
	{AR0330_TABLE_WAIT_MS, 10},
	{0x301A, 0x0218},
	{0x31B0, 0x0062},
	{0x31B2, 0x0046},
	{0x31B4, 0x3248},
	{0x31B6, 0x22A6},
	{0x31B8, 0x1832},
	{0x31BA, 0x1052},
	{0x31BC, 0x0408},
	{0x31AE, 0x0201},
	{AR0330_TABLE_WAIT_MS, 1},
	{0x3044, 0x0590},
	{0x3EE6, 0x60AD},
	{0x3EDC, 0xDBFA},
	{0x301A, 0x0218},
	{AR0330_TABLE_WAIT_MS, 10},
	{0x3D00, 0x0481},
	{0x3D02, 0xFFFF},
	{0x3D04, 0xFFFF},
	{0x3D06, 0xFFFF},
	{0x3D08, 0x6600},
	{0x3D0A, 0x0311},
	{0x3D0C, 0x8C67},
	{0x3D0E, 0x0808},
	{0x3D10, 0x4380},
	{0x3D12, 0x4343},
	{0x3D14, 0x8043},
	{0x3D16, 0x4330},
	{0x3D18, 0x0543},
	{0x3D1A, 0x4381},
	{0x3D1C, 0x4C85},
	{0x3D1E, 0x2022},
	{0x3D20, 0x8020},
	{0x3D22, 0xA093},
	{0x3D24, 0x5A8A},
	{0x3D26, 0x4C81},
	{0x3D28, 0x5981},
	{0x3D2A, 0x1E00},
	{0x3D2C, 0x5F83},
	{0x3D2E, 0x5C80},
	{0x3D30, 0x5C81},
	{0x3D32, 0x5F58},
	{0x3D34, 0x6880},
	{0x3D36, 0x1060},
	{0x3D38, 0x8541},
	{0x3D3A, 0xB350},
	{0x3D3C, 0x5F10},
	{0x3D3E, 0x6050},
	{0x3D40, 0x5780},
	{0x3D42, 0x6880},
	{0x3D44, 0x2220},
	{0x3D46, 0x805D},
	{0x3D48, 0x8140},
	{0x3D4A, 0x864B},
	{0x3D4C, 0x8524},
	{0x3D4E, 0x08A0},
	{0x3D50, 0x55B8},
	{0x3D52, 0x429C},
	{0x3D54, 0x4281},
	{0x3D56, 0x4081},
	{0x3D58, 0x2808},
	{0x3D5A, 0x2810},
	{0x3D5C, 0x5727},
	{0x3D5E, 0x1069},
	{0x3D60, 0x4B52},
	{0x3D62, 0x8265},
	{0x3D64, 0x8A65},
	{0x3D66, 0xA95E},
	{0x3D68, 0x5080},
	{0x3D6A, 0x5250},
	{0x3D6C, 0x6080},
	{0x3D6E, 0x6922},
	{0x3D70, 0x2080},
	{0x3D72, 0x5D80},
	{0x3D74, 0x4080},
	{0x3D76, 0x5681},
	{0x3D78, 0x5781},
	{0x3D7A, 0x4B86},
	{0x3D7C, 0x2408},
	{0x3D7E, 0x9345},
	{0x3D80, 0x8144},
	{0x3D82, 0x4481},
	{0x3D84, 0x4586},
	{0x3D86, 0x4E80},
	{0x3D88, 0x4FCD},
	{0x3D8A, 0x4685},
	{0x3D8C, 0x0006},
	{0x3D8E, 0x8143},
	{0x3D90, 0x4380},
	{0x3D92, 0x4343},
	{0x3D94, 0x8043},
	{0x3D96, 0x4380},
	{0x3D98, 0x4343},
	{0x3D9A, 0x8043},
	{0x3D9C, 0x4380},
	{0x3D9E, 0x4343},
	{0x3DA0, 0x8648},
	{0x3DA2, 0x4880},
	{0x3DA4, 0x6B6B},
	{0x3DA6, 0x814C},
	{0x3DA8, 0x864D},
	{0x3DAA, 0xA442},
	{0x3DAC, 0x8641},
	{0x3DAE, 0x804D},
	{0x3DB0, 0x864C},
	{0x3DB2, 0x8A45},
	{0x3DB4, 0x8144},
	{0x3DB6, 0x4481},
	{0x3DB8, 0x4583},
	{0x3DBA, 0x46B7},
	{0x3DBC, 0x7386},
	{0x3DBE, 0x4685},
	{0x3DC0, 0x0006},
	{0x3DC2, 0x8143},
	{0x3DC4, 0x4380},
	{0x3DC6, 0x4343},
	{0x3DC8, 0x8043},
	{0x3DCA, 0x4380},
	{0x3DCC, 0x4343},
	{0x3DCE, 0x8043},
	{0x3DD0, 0x4380},
	{0x3DD2, 0x4343},
	{0x3DD4, 0x8648},
	{0x3DD6, 0x4880},
	{0x3DD8, 0x6A6A},
	{0x3DDA, 0x814C},
	{0x3DDC, 0x864D},
	{0x3DDE, 0xA442},
	{0x3DE0, 0x8641},
	{0x3DE2, 0x804D},
	{0x3DE4, 0x864C},
	{0x3DE6, 0x8A45},
	{0x3DE8, 0x8144},
	{0x3DEA, 0x4481},
	{0x3DEC, 0x4583},
	{0x3DEE, 0x4686},
	{0x3DF0, 0x73FF},
	{0x3DF2, 0xD358},
	{0x3DF4, 0x835B},
	{0x3DF6, 0x825A},
	{0x3DF8, 0x8153},
	{0x3DFA, 0x5467},
	{0x3DFC, 0x6363},
	{0x3DFE, 0x2640},
	{0x3E00, 0x6470},
	{0x3E02, 0xFFFF},
	{0x3E04, 0xFFFF},
	{0x3E06, 0xFFED},
	{0x3E08, 0x4580},
	{0x3E0A, 0x4384},
	{0x3E0C, 0x4380},
	{0x3E0E, 0x0280},
	{0x3E10, 0x8402},
	{0x3E12, 0x8080},
	{0x3E14, 0x6A84},
	{0x3E16, 0x6A80},
	{0x3E18, 0x4484},
	{0x3E1A, 0x4480},
	{0x3E1C, 0x4578},
	{0x3E1E, 0x8270},
	{0x3E20, 0x0000},
	{0x3E22, 0x0000},
	{0x3E24, 0x0000},
	{0x3E26, 0x0000},
	{0x3E28, 0x0000},
	{0x3E2A, 0x0000},
	{0x3E2C, 0x0000},
	{0x3E2E, 0x0000},
	{0x3E30, 0x0000},
	{0x3E32, 0x0000},
	{0x3E34, 0x0000},
	{0x3E36, 0x0000},
	{0x3E38, 0x0000},
	{0x3E3A, 0x0000},
	{0x3E3C, 0x0000},
	{0x3E3E, 0x0000},
	{0x3E40, 0x0000},
	{0x3E42, 0x0000},
	{0x3E44, 0x0000},
	{0x3E46, 0x0000},
	{0x3E48, 0x0000},
	{0x3E4A, 0x0000},
	{0x3E4C, 0x0000},
	{0x3E4E, 0x0000},
	{0x3E50, 0x0000},
	{0x3E52, 0x0000},
	{0x3E54, 0x0000},
	{0x3E56, 0x0000},
	{0x3E58, 0x0000},
	{0x3E5A, 0x0000},
	{0x3E5C, 0x0000},
	{0x3E5E, 0x0000},
	{0x3E60, 0x0000},
	{0x3E62, 0x0000},
	{0x3E64, 0x0000},
	{0x3E66, 0x0000},
	{0x3E68, 0x0000},
	{0x3E6A, 0x0000},
	{0x3E6C, 0x0000},
	{0x3E6E, 0x0000},
	{0x3E70, 0x0000},
	{0x3E72, 0x0000},
	{0x3E74, 0x0000},
	{0x3E76, 0x0000},
	{0x3E78, 0x0000},
	{0x3E7A, 0x0000},
	{0x3E7C, 0x0000},
	{0x3E7E, 0x0000},
	{0x3E80, 0x0000},
	{0x3E82, 0x0000},
	{0x3E84, 0x0000},
	{0x3E86, 0x0000},
	{0x3E88, 0x0000},
	{0x3E8A, 0x0000},
	{0x3E8C, 0x0000},
	{0x3E8E, 0x0000},
	{0x3E90, 0x0000},
	{0x3E92, 0x0000},
	{0x3E94, 0x0000},
	{0x3E96, 0x0000},
	{0x3E98, 0x0000},
	{0x3E9A, 0x0000},
	{0x3E9C, 0x0000},
	{0x3E9E, 0x0000},
	{0x3EA0, 0x0000},
	{0x3EA2, 0x0000},
	{0x3EA4, 0x0000},
	{0x3EA6, 0x0000},
	{0x3EA8, 0x0000},
	{0x3EAA, 0x0000},
	{0x3EAC, 0x0000},
	{0x3EAE, 0x0000},
	{0x3EB0, 0x0000},
	{0x3EB2, 0x0000},
	{0x3EB4, 0x0000},
	{0x3EB6, 0x0000},
	{0x3EB8, 0x0000},
	{0x3EBA, 0x0000},
	{0x3EBC, 0x0000},
	{0x3EBE, 0x0000},
	{0x3EC0, 0x0000},
	{0x3EC2, 0x0000},
	{0x3EC4, 0x0000},
	{0x3EC6, 0x0000},
	{0x3EC8, 0x0000},
	{0x3ECA, 0x0000},
	{0x301A, 0x021C},
	{0x0342, 0x10CC},
	{0x0340, 0x04A4},
	{0x0202, 0x0496},
	{0x0312, 0x045D},
	{0x31AE, 0x0201},
	{0x0300, 0x0005},
	{0x0302, 0x0001},
	{0x0304, 0x0202},
	{0x0306, 0x4040},
	{0x0308, 0x000A},
	{0x030A, 0x0001},
	{0x0344, 0x0008},
	{0x0348, 0x0787},
	{0x0346, 0x0008},
	{0x034A, 0x043F},
	{0x034C, 0x0780},
	{0x034E, 0x0438},
	{0x3040, 0x0041},
	{0x0104, 0x0001},
	{0x3ECC, 0x008F},
	{0x3ECE, 0xA8F0},
	{0x3ED0, 0xFFFF},
	{0x3ED6, 0x7193},
	{0x3ED8, 0x8A11},
	{0x30D2, 0x0020},
	{0x30D4, 0x0040},
	{0x3180, 0x80FF},
	{0x0104, 0x0000},
	{0x3044, 0x0000},
	{0x30CA, 0x0001},
	{0x30D4, 0x0000},
	{0x31E0, 0x0000},
	{0x301A, 0x0000},
	{0x301E, 0x0000},
	{0x3070, 0x0002},
	{0x301A, 0x001C},
	{AR0330_TABLE_END, 0x0000}
};

#define AR0331_TEST_PATTERN_DISABLE	0
#define AR0331_TEST_PATTERN_COLOR_BARS	1

static const char * const ar0331_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
};

static const int ar0331_test_pattern_val[] = {
	AR0331_TEST_PATTERN_DISABLE,
	AR0331_TEST_PATTERN_COLOR_BARS,
};

/* regulator supplies */
static const char * const ar0331_supply_name[] = {
	/* Supplies can be enabled in any order */
	"vana",  /* Analog (2.7V) supply */
	"vdig",  /* Digital Core (1.2V) supply */
	"vif",  /* IF (1.8V) supply */
};

#define AR0331_NUM_SUPPLIES ARRAY_SIZE(ar0331_supply_name)

/*
 * The chip supports MEDIA_BUS_FMT_SRGGB8_1X8 as well, but we only
 * test with MEDIA_BUS_FMT_SRGGB10_1X10
 */
static const u32 codes[] = {
	MEDIA_BUS_FMT_SGBRG10_1X10,
};

#define AR0331_START_MIN_DELAY_US	6200
#define AR0331_START_DELAY_RANGE_US	1000

static const struct ar0331_mode ar0331_supported_modes[] = {
	{
		/* 720P mode */
		.width = 1280,
		.height = 720,
		.reg_list = mode_1280x720_regs,
	},
	{
		/* 1280x960 mode */
		.width = 1280,
		.height = 960,
		.reg_list = mode_1280x960_regs,
	},
};

static const struct ar0331_mode ar0330_supported_modes[] = {
	{
		/* 3MPix mode */
		.width = 2304,
		.height = 1536,
		.reg_list = mode_2304x1536_regs,
	},
	{
		/* 720P mode */
		.width = 1280,
		.height = 720,
		.reg_list = mode_1280x720_regs,
	},
	{
		/* 1280x960 mode */
		.width = 1280,
		.height = 960,
		.reg_list = mode_1280x960_regs,
	},
};

struct ar0331_chip_info {
	const struct ar0331_mode *supported_modes;
	const unsigned int supported_modes_count;

	unsigned int native_top;
	unsigned int native_left;
	unsigned int native_width;
	unsigned int native_height;
};

enum {
	AR0330_CHIP_IDX,
	AR0331_CHIP_IDX,
};

static const struct ar0331_chip_info ar0331_chip_info[] = {
	[AR0330_CHIP_IDX] = {
		.supported_modes = ar0330_supported_modes,
		.supported_modes_count = ARRAY_SIZE(ar0330_supported_modes),
		.native_top = AR0330_PIXEL_ARRAY_TOP,
		.native_left = AR0330_PIXEL_ARRAY_LEFT,
		.native_width = AR0330_PIXEL_ARRAY_WIDTH,
		.native_height = AR0330_PIXEL_ARRAY_HEIGHT,
	},
	[AR0331_CHIP_IDX] = {
		.supported_modes = ar0331_supported_modes,
		.supported_modes_count = ARRAY_SIZE(ar0331_supported_modes),
		.native_top = AR0331_PIXEL_ARRAY_TOP,
		.native_left = AR0331_PIXEL_ARRAY_LEFT,
		.native_width = AR0331_PIXEL_ARRAY_WIDTH,
		.native_height = AR0331_PIXEL_ARRAY_HEIGHT,
	},
};

struct ar0331 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_mbus_framefmt fmt;

	const struct ar0331_chip_info *chip_info;

	struct clk *xclk; /* system clock to AR0331 */
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[AR0331_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vblank;

	/* Current mode */
	const struct ar0331_mode *mode;

	struct regmap *regmap;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
};

static inline struct ar0331 *to_ar0331(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct ar0331, sd);
}

/* Read registers up to 2 at a time */
static int ar0331_read_reg(struct ar0331 *ar0331, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0331->sd);
	int ret;

	if (len == 2) {
		u8 buf[2];

		ret = regmap_raw_read(ar0331->regmap, reg, buf, sizeof(buf));
		if (ret == 0)
			*val = (u16)buf[0] << 8 | buf[1];
	} else if (len == 1) {
		u32 _val;

		ret = regmap_read(ar0331->regmap, reg, &_val);
		if (ret == 0)
			*val = _val & 0xFF;
	} else
		return -EINVAL;

	if (ret)
		dev_err(&client->dev, "i2c read failed, %x, err %d\n",
			reg, ret);

	return ret;
}

/* Write registers up to 2 at a time */
static int ar0331_write_reg(struct ar0331 *ar0331, u16 reg, u32 len, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0331->sd);
	unsigned char buf[2];
	int ret;

	if (len == 2) {
		buf[0] = (u8) (val >> 8);
		buf[1] = (u8) (val & 0xff);
		ret = regmap_raw_write(ar0331->regmap, reg, buf, sizeof(buf));
	} else if (len == 1) {
		ret = regmap_write(ar0331->regmap, reg, val & 0xff);
	} else
		return -EINVAL;

	if (ret)
		dev_err(&client->dev, "i2c write failed, %x = %x, err %d\n",
			reg, val, ret);
	return ret;
}

/* Write a list of registers */
static int ar0331_write_regs(struct ar0331 *ar0331,
			     const struct ar0331_reg *regs)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0331->sd);
	unsigned int i;
	int ret;

	for (i = 0; regs[i].address != AR0330_TABLE_END; i++) {
		ret = ar0331_write_reg(ar0331, regs[i].address, 2, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

static void ar0331_set_default_format(struct ar0331 *ar0331)
{
	struct v4l2_mbus_framefmt *fmt;
	const struct ar0331_chip_info *info = ar0331->chip_info;

	fmt = &ar0331->fmt;
	fmt->code = codes[0];
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = info->supported_modes[0].width;
	fmt->height = info->supported_modes[0].height;
	fmt->field = V4L2_FIELD_NONE;
}

static int ar0331_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ar0331 *ar0331 = to_ar0331(sd);
	const struct ar0331_chip_info *info = ar0331->chip_info;
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_state_get_format(fh->state, 0);
	struct v4l2_rect *try_crop;

	mutex_lock(&ar0331->mutex);

	/* Initialize try_fmt */
	try_fmt->width = info->supported_modes[0].width;
	try_fmt->height = info->supported_modes[0].height;
	try_fmt->code = codes[0];
	try_fmt->field = V4L2_FIELD_NONE;

	/* Initialize try_crop rectangle. */
	try_crop = v4l2_subdev_state_get_crop(fh->state, 0);
	try_crop->top = info->native_top;
	try_crop->left = info->native_left;
	try_crop->width = info->native_width;
	try_crop->height = info->native_height;

	mutex_unlock(&ar0331->mutex);

	return 0;
}

static int ar0331_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0331 *ar0331 =
		container_of(ctrl->handler, struct ar0331, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&ar0331->sd);
	int ret;

	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ar0331_write_reg(ar0331, AR0331_REG_ANALOG_GAIN,
				       AR0331_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = ar0331_write_reg(ar0331, AR0331_REG_EXPOSURE,
				       AR0331_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		/* FIXME: Digital gain requires more tuning */
		ret = ar0331_write_reg(ar0331, AR0331_REG_GREEN1_GAIN,
				       AR0331_REG_VALUE_16BIT, ctrl->val);
		ret |= ar0331_write_reg(ar0331, AR0331_REG_BLUE_GAIN,
				       AR0331_REG_VALUE_16BIT, ctrl->val);
		ret |= ar0331_write_reg(ar0331, AR0331_REG_RED_GAIN,
				       AR0331_REG_VALUE_16BIT, ctrl->val);
		ret |= ar0331_write_reg(ar0331, AR0331_REG_GREEN2_GAIN,
				       AR0331_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		if (ctrl->val == AR0331_TEST_PATTERN_COLOR_BARS)
			ret = ar0331_write_regs(ar0331, tp_colorbar);
		else
			ret = ar0331_write_regs(ar0331, ar0331->mode->reg_list);
		break;
	case V4L2_CID_VBLANK:
		ret = ar0331_write_reg(ar0331, AR0331_REG_VTS,
				       AR0331_REG_VALUE_16BIT,
				       ctrl->val);
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

static const struct v4l2_ctrl_ops ar0331_ctrl_ops = {
	.s_ctrl = ar0331_set_ctrl,
};

static int ar0331_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(codes))
		return -EINVAL;

	code->code = codes[code->index];

	return 0;
}

static int ar0331_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ar0331 *ar0331 = to_ar0331(sd);
	const struct ar0331_chip_info *info = ar0331->chip_info;

	if (fse->index >= info->supported_modes_count)
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;

	fse->min_width = info->supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = info->supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void ar0331_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void ar0331_update_pad_format(struct ar0331 *ar0331,
				     const struct ar0331_mode *mode,
				     struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	ar0331_reset_colorspace(&fmt->format);
}

static int __ar0331_get_pad_format(struct ar0331 *ar0331,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_state_get_format(sd_state, fmt->pad);
		fmt->format = *try_fmt;
	} else {
		ar0331_update_pad_format(ar0331, ar0331->mode, fmt);
	}

	return 0;
}

static int ar0331_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct ar0331 *ar0331 = to_ar0331(sd);
	int ret;

	mutex_lock(&ar0331->mutex);
	ret = __ar0331_get_pad_format(ar0331, sd_state, fmt);
	mutex_unlock(&ar0331->mutex);

	return ret;
}

static int ar0331_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct ar0331 *ar0331 = to_ar0331(sd);
	const struct ar0331_chip_info *info = ar0331->chip_info;
	const struct ar0331_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;

	mutex_lock(&ar0331->mutex);

	mode = v4l2_find_nearest_size(info->supported_modes,
				      info->supported_modes_count,
				      width, height,
				      fmt->format.width, fmt->format.height);
	ar0331_update_pad_format(ar0331, mode, fmt);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		*framefmt = fmt->format;
	} else if (ar0331->mode != mode ||
		   ar0331->fmt.code != fmt->format.code) {
		ar0331->fmt = fmt->format;
		ar0331->mode = mode;
	}

	mutex_unlock(&ar0331->mutex);

	return 0;
}

static int ar0331_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct ar0331 *ar0331 = to_ar0331(sd);
	const struct ar0331_chip_info *info = ar0331->chip_info;

	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = info->native_top;
		sel->r.left = info->native_left;
		sel->r.width = info->native_width;
		sel->r.height = info->native_height;

		return 0;
	default:
		return -EINVAL;
	}
}

static int ar0331_start_streaming(struct ar0331 *ar0331)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0331->sd);
	int ret;

	/* Just take the sensor out of sleep and write the mode registers to it */
	ret = pm_runtime_get_sync(&client->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(&client->dev);
		return ret;
	}

	/* Apply default values of current mode */
	ret = ar0331_write_regs(ar0331, ar0331->mode->reg_list);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		goto err_rpm_put;
	}

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(ar0331->sd.ctrl_handler);
	if (ret)
		goto err_rpm_put;

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static void ar0331_stop_streaming(struct ar0331 *ar0331)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0331->sd);

	pm_runtime_put(&client->dev);
}

static int ar0331_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar0331 *ar0331 = to_ar0331(sd);
	int ret = 0;

	mutex_lock(&ar0331->mutex);
	if (ar0331->streaming == enable) {
		mutex_unlock(&ar0331->mutex);
		return 0;
	}

	if (enable) {
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = ar0331_start_streaming(ar0331);
		if (ret)
			goto err_unlock;
	} else {
		ar0331_stop_streaming(ar0331);
	}

	ar0331->streaming = enable;

	mutex_unlock(&ar0331->mutex);

	return ret;

err_unlock:
	mutex_unlock(&ar0331->mutex);

	return ret;
}

/* Power/clock management functions */
static int ar0331_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0331 *ar0331 = to_ar0331(sd);
	int ret;

	ret = regulator_bulk_enable(AR0331_NUM_SUPPLIES,
				    ar0331->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(ar0331->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(ar0331->reset_gpio, 1);
	usleep_range(AR0331_START_MIN_DELAY_US,
		     AR0331_START_MIN_DELAY_US + AR0331_START_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(AR0331_NUM_SUPPLIES, ar0331->supplies);

	return ret;
}

static int ar0331_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0331 *ar0331 = to_ar0331(sd);

	gpiod_set_value_cansleep(ar0331->reset_gpio, 0);
	regulator_bulk_disable(AR0331_NUM_SUPPLIES, ar0331->supplies);
	clk_disable_unprepare(ar0331->xclk);

	return 0;
}

static int __maybe_unused ar0331_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0331 *ar0331 = to_ar0331(sd);

	if (ar0331->streaming)
		ar0331_stop_streaming(ar0331);

	return 0;
}

static int __maybe_unused ar0331_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0331 *ar0331 = to_ar0331(sd);
	int ret;

	if (ar0331->streaming) {
		ret = ar0331_start_streaming(ar0331);
		if (ret)
			goto error;
	}

	return 0;

error:
	ar0331_stop_streaming(ar0331);
	ar0331->streaming = false;

	return ret;
}

static int ar0331_get_regulators(struct ar0331 *ar0331)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0331->sd);
	unsigned int i;

	for (i = 0; i < AR0331_NUM_SUPPLIES; i++)
		ar0331->supplies[i].supply = ar0331_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       AR0331_NUM_SUPPLIES,
				       ar0331->supplies);
}

/* Verify chip ID */
static int ar0331_identify_module(struct ar0331 *ar0331)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0331->sd);
	int ret;
	u32 val;

	ret = ar0331_read_reg(ar0331, AR0331_REG_CHIP_ID,
			      AR0331_REG_VALUE_16BIT, &val);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id\n");
		//return ret;
		return 0;  // FIXME: ignoring this for now
	}

	switch (val) {
	case AR0330_CHIP_ID:
		return 0;
	default:
		dev_err(&client->dev, "chip id unknown: %x\n", val);
		// return -EIO;
		return 0; // FIXME: find out the chip ID for AR0331
	}
}

static const struct v4l2_subdev_core_ops ar0331_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops ar0331_video_ops = {
	.s_stream = ar0331_set_stream,
};

static const struct v4l2_subdev_pad_ops ar0331_pad_ops = {
	.enum_mbus_code = ar0331_enum_mbus_code,
	.get_fmt = ar0331_get_pad_format,
	.set_fmt = ar0331_set_pad_format,
	.get_selection = ar0331_get_selection,
	.enum_frame_size = ar0331_enum_frame_size,
};

static const struct v4l2_subdev_ops ar0331_subdev_ops = {
	.core = &ar0331_core_ops,
	.video = &ar0331_video_ops,
	.pad = &ar0331_pad_ops,
};

static const struct v4l2_subdev_internal_ops ar0331_internal_ops = {
	.open = ar0331_open,
};

/* Initialize control handlers */
static int ar0331_init_controls(struct ar0331 *ar0331)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0331->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_fwnode_device_properties props;
	int ret;

	ctrl_hdlr = &ar0331->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 6);
	if (ret)
		return ret;

	mutex_init(&ar0331->mutex);
	ctrl_hdlr->lock = &ar0331->mutex;

	/* By default, PIXEL_RATE is read only */
	ar0331->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &ar0331_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       AR0331_PIXEL_RATE,
					       AR0331_PIXEL_RATE, 1,
					       AR0331_PIXEL_RATE);

	ar0331->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &ar0331_ctrl_ops,
					   V4L2_CID_VBLANK, AR0331_VTS_MIN,
					   AR0331_VTS_MAX, 1,
					   AR0331_VTS_DEF);
	ar0331->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &ar0331_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     AR0331_EXPOSURE_MIN,
					     AR0331_EXPOSURE_MAX,
					     AR0331_EXPOSURE_STEP,
					     AR0331_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &ar0331_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  AR0331_ANA_GAIN_MIN, AR0331_ANA_GAIN_MAX,
			  AR0331_ANA_GAIN_STEP, AR0331_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &ar0331_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  AR0331_DGTL_GAIN_MIN, AR0331_DGTL_GAIN_MAX,
			  AR0331_DGTL_GAIN_STEP, AR0331_DGTL_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &ar0331_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ar0331_test_pattern_menu) - 1,
				     0, 0, ar0331_test_pattern_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &ar0331_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	ar0331->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&ar0331->mutex);

	return ret;
}

static void ar0331_free_controls(struct ar0331 *ar0331)
{
	v4l2_ctrl_handler_free(ar0331->sd.ctrl_handler);
	mutex_destroy(&ar0331->mutex);
}

static int ar0331_check_hwcfg(struct device *dev)
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
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 1) {
		dev_err(dev, "only 1 data lane are currently supported\n");
		goto error_out;
	}

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static int ar0331_probe(struct i2c_client *client)
{
	const struct ar0331_chip_info *info;
	struct device *dev = &client->dev;
	struct ar0331 *ar0331;
	int ret;

	info = of_device_get_match_data(&client->dev);
	if (!info)
		return -ENODEV;

	ar0331 = devm_kzalloc(&client->dev, sizeof(*ar0331), GFP_KERNEL);
	if (!ar0331)
		return -ENOMEM;

	ar0331->chip_info = info;
	v4l2_i2c_subdev_init(&ar0331->sd, client, &ar0331_subdev_ops);

	/* Check the hardware configuration in device tree */
	if (ar0331_check_hwcfg(dev))
		return -EINVAL;

	/* Get system clock (xclk) */
	ar0331->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(ar0331->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(ar0331->xclk);
	}

	ar0331->xclk_freq = clk_get_rate(ar0331->xclk);
	if (ar0331->xclk_freq != AR0331_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			ar0331->xclk_freq);
		return -EINVAL;
	}

	ret = ar0331_get_regulators(ar0331);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	ar0331->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	ar0331->regmap = devm_regmap_init_i2c(client, &ar0331_regmap_config);
	if (IS_ERR(ar0331->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(ar0331->regmap));
		return -ENODEV;
	}

	/*
	 * The sensor must be powered for ar0331_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = ar0331_power_on(dev);
	if (ret)
		return ret;

	ret = ar0331_identify_module(ar0331);
	if (ret)
		goto error_power_off;

	ar0331->mode = &info->supported_modes[0];

	ret = ar0331_init_controls(ar0331);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	ar0331->sd.internal_ops = &ar0331_internal_ops;
	ar0331->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ar0331->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	ar0331->pad.flags = MEDIA_PAD_FL_SOURCE;

	/* Initialize default format */
	ar0331_set_default_format(ar0331);

	ret = media_entity_pads_init(&ar0331->sd.entity, 1, &ar0331->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&ar0331->sd);
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
	media_entity_cleanup(&ar0331->sd.entity);

error_handler_free:
	ar0331_free_controls(ar0331);

error_power_off:
	ar0331_power_off(dev);

	return ret;
}

static void ar0331_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0331 *ar0331 = to_ar0331(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	ar0331_free_controls(ar0331);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		ar0331_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id ar0331_dt_ids[] = {
	{ .compatible = "aptina,ar0330", .data = &ar0331_chip_info[AR0330_CHIP_IDX] },
	{ .compatible = "aptina,ar0331", .data = &ar0331_chip_info[AR0331_CHIP_IDX] },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ar0331_dt_ids);

static const struct dev_pm_ops ar0331_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ar0331_suspend, ar0331_resume)
	SET_RUNTIME_PM_OPS(ar0331_power_off, ar0331_power_on, NULL)
};

static struct i2c_driver ar0331_i2c_driver = {
	.driver = {
		.name = "ar0331",
		.of_match_table	= ar0331_dt_ids,
		.pm = &ar0331_pm_ops,
	},
	.probe = ar0331_probe,
	.remove = ar0331_remove,
};

module_i2c_driver(ar0331_i2c_driver);

MODULE_AUTHOR("Alexandru Ardelean <alex@shruggie.ro");
MODULE_DESCRIPTION("Aptina/OnSemi AR0331 sensor driver");
MODULE_LICENSE("GPL v2");
