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

#ifndef _AP1302_H_
#define _AP1302_H_

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>

#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>



#define DRIVER_NAME "ap1302"

#define AP1302_FW_WINDOW_SIZE			0x2000
#define AP1302_FW_WINDOW_OFFSET			0x8000

#define AP1302_MIN_WIDTH			24U
#define AP1302_MIN_HEIGHT			16U
#define AP1302_MAX_WIDTH			4224U
#define AP1302_MAX_HEIGHT			4092U

#define AP1302_REG_16BIT(n)			((2 << 24) | (n))
#define AP1302_REG_32BIT(n)			((4 << 24) | (n))
#define AP1302_REG_SIZE(n)			((n) >> 24)
#define AP1302_REG_ADDR(n)			((n) & 0x0000ffff)
#define AP1302_REG_PAGE(n)			((n) & 0x00ff0000)
#define AP1302_REG_PAGE_MASK			0x00ff0000

/* Info Registers */
#define AP1302_CHIP_VERSION			AP1302_REG_16BIT(0x0000)
#define AP1302_CHIP_ID				0x0265
#define AP1302_FRAME_CNT			AP1302_REG_16BIT(0x0002)
#define AP1302_ERROR				AP1302_REG_16BIT(0x0006)
#define AP1302_ERR_FILE				AP1302_REG_32BIT(0x0008)
#define AP1302_ERR_LINE				AP1302_REG_16BIT(0x000c)
#define AP1302_SIPM_ERR_0			AP1302_REG_16BIT(0x0014)
#define AP1302_SIPM_ERR_1			AP1302_REG_16BIT(0x0016)
#define AP1302_CHIP_REV				AP1302_REG_16BIT(0x0050)
#define AP1302_CON_BUF(n)			AP1302_REG_16BIT(0x0a2c + (n))
#define AP1302_CON_BUF_SIZE			512

/* Control Registers */
#define AP1302_DZ_TGT_FCT			AP1302_REG_16BIT(0x1010)
#define AP1302_SFX_MODE				AP1302_REG_16BIT(0x1016)
#define AP1302_SFX_MODE_SFX_NORMAL		(0U << 0)
#define AP1302_SFX_MODE_SFX_ALIEN		(1U << 0)
#define AP1302_SFX_MODE_SFX_ANTIQUE		(2U << 0)
#define AP1302_SFX_MODE_SFX_BW			(3U << 0)
#define AP1302_SFX_MODE_SFX_EMBOSS		(4U << 0)
#define AP1302_SFX_MODE_SFX_EMBOSS_COLORED	(5U << 0)
#define AP1302_SFX_MODE_SFX_GRAYSCALE		(6U << 0)
#define AP1302_SFX_MODE_SFX_NEGATIVE		(7U << 0)
#define AP1302_SFX_MODE_SFX_BLUISH		(8U << 0)
#define AP1302_SFX_MODE_SFX_GREENISH		(9U << 0)
#define AP1302_SFX_MODE_SFX_REDISH		(10U << 0)
#define AP1302_SFX_MODE_SFX_POSTERIZE1		(11U << 0)
#define AP1302_SFX_MODE_SFX_POSTERIZE2		(12U << 0)
#define AP1302_SFX_MODE_SFX_SEPIA1		(13U << 0)
#define AP1302_SFX_MODE_SFX_SEPIA2		(14U << 0)
#define AP1302_SFX_MODE_SFX_SKETCH		(15U << 0)
#define AP1302_SFX_MODE_SFX_SOLARIZE		(16U << 0)
#define AP1302_SFX_MODE_SFX_FOGGY		(17U << 0)
#define AP1302_BUBBLE_OUT_FMT			AP1302_REG_16BIT(0x1164)
#define AP1302_BUBBLE_OUT_FMT_FT_YUV		(3U << 4)
#define AP1302_BUBBLE_OUT_FMT_FT_RGB		(4U << 4)
#define AP1302_BUBBLE_OUT_FMT_FT_YUV_JFIF	(5U << 4)
#define AP1302_BUBBLE_OUT_FMT_FST_RGB_888	(0U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_RGB_565	(1U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_RGB_555M	(2U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_RGB_555L	(3U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_YUV_422	(0U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_YUV_420	(1U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_YUV_400	(2U << 0)
#define AP1302_ATOMIC				AP1302_REG_16BIT(0x1184)
#define AP1302_ATOMIC_MODE			BIT(2)
#define AP1302_ATOMIC_FINISH			BIT(1)
#define AP1302_ATOMIC_RECORD			BIT(0)

/*
 * Preview Context Registers (preview_*). AP1302 supports 3 "contexts"
 * (Preview, Snapshot, Video). These can be programmed for different size,
 * format, FPS, etc. There is no functional difference between the contexts,
 * so the only potential benefit of using them is reduced number of register
 * writes when switching output modes (if your concern is atomicity, see
 * "atomic" register).
 * So there's virtually no benefit in using contexts for this driver and it
 * would significantly increase complexity. Let's use preview context only.
 */
#define AP1302_PREVIEW_WIDTH			AP1302_REG_16BIT(0x2000)
#define AP1302_PREVIEW_HEIGHT			AP1302_REG_16BIT(0x2002)
#define AP1302_PREVIEW_ROI_X0			AP1302_REG_16BIT(0x2004)
#define AP1302_PREVIEW_ROI_Y0			AP1302_REG_16BIT(0x2006)
#define AP1302_PREVIEW_ROI_X1			AP1302_REG_16BIT(0x2008)
#define AP1302_PREVIEW_ROI_Y1			AP1302_REG_16BIT(0x200a)
#define AP1302_PREVIEW_ENABLE			AP1302_REG_16BIT(0x2010)
#define AP1302_PREVIEW_OUT_FMT			AP1302_REG_16BIT(0x2012)
#define AP1302_PREVIEW_OUT_FMT_IPIPE_BYPASS	BIT(13)
#define AP1302_PREVIEW_OUT_FMT_SS		BIT(12)
#define AP1302_PREVIEW_OUT_FMT_FAKE_EN		BIT(11)
#define AP1302_PREVIEW_OUT_FMT_ST_EN		BIT(10)
#define AP1302_PREVIEW_OUT_FMT_IIS_NONE		(0U << 8)
#define AP1302_PREVIEW_OUT_FMT_IIS_POST_VIEW	(1U << 8)
#define AP1302_PREVIEW_OUT_FMT_IIS_VIDEO	(2U << 8)
#define AP1302_PREVIEW_OUT_FMT_IIS_BUBBLE	(3U << 8)
#define AP1302_PREVIEW_OUT_FMT_FT_JPEG_422	(0U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_JPEG_420	(1U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_YUV		(3U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_RGB		(4U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_YUV_JFIF	(5U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_RAW8		(8U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_RAW10		(9U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_RAW12		(10U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_RAW16		(11U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_DNG8		(12U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_DNG10		(13U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_DNG12		(14U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_DNG16		(15U << 4)
#define AP1302_PREVIEW_OUT_FMT_FST_JPEG_ROTATE	BIT(2)
#define AP1302_PREVIEW_OUT_FMT_FST_JPEG_SCAN	(0U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_JPEG_JFIF	(1U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_JPEG_EXIF	(2U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RGB_888	(0U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RGB_565	(1U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RGB_555M	(2U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RGB_555L	(3U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_YUV_422	(0U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_YUV_420	(1U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_YUV_400	(2U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_SENSOR	(0U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_CAPTURE	(1U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_CP	(2U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_BPC	(3U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_IHDR	(4U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_PP	(5U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_DENSH	(6U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_PM	(7U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_GC	(8U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_CURVE	(9U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_CCONV	(10U << 0)
#define AP1302_PREVIEW_SENSOR_MODE		AP1302_REG_16BIT(0x2014)
#define AP1302_PREVIEW_LINE_TIME		AP1302_REG_32BIT(0x201C)
#define AP1302_PREVIEW_MAX_FPS			AP1302_REG_16BIT(0x2020)
#define AP1302_PREVIEW_AE_UPPER_ET		AP1302_REG_32BIT(0x2024)
#define AP1302_PREVIEW_AE_MAX_ET		AP1302_REG_32BIT(0x2028)
#define AP1302_PREVIEW_HINF_CTRL		AP1302_REG_16BIT(0x2030)
#define AP1302_PREVIEW_HINF_CTRL_BT656_LE	BIT(15)
#define AP1302_PREVIEW_HINF_CTRL_BT656_16BIT	BIT(14)
#define AP1302_PREVIEW_HINF_CTRL_MUX_DELAY(n)	((n) << 8)
#define AP1302_PREVIEW_HINF_CTRL_LV_POL		BIT(7)
#define AP1302_PREVIEW_HINF_CTRL_FV_POL		BIT(6)
#define AP1302_PREVIEW_HINF_CTRL_MIPI_CONT_CLK	BIT(5)
#define AP1302_PREVIEW_HINF_CTRL_SPOOF		BIT(4)
#define AP1302_PREVIEW_HINF_CTRL_MIPI_MODE	BIT(3)
#define AP1302_PREVIEW_HINF_CTRL_MIPI_LANES(n)	((n) << 0)

/* IQ Registers */
#define AP1302_AE_CTRL				AP1302_REG_16BIT(0x5002)
#define AP1302_AE_CTRL_STATS_SEL		BIT(11)
#define AP1302_AE_CTRL_IMM			BIT(10)
#define AP1302_AE_CTRL_ROUND_ISO		BIT(9)
#define AP1302_AE_CTRL_UROI_FACE		BIT(7)
#define AP1302_AE_CTRL_UROI_LOCK		BIT(6)
#define AP1302_AE_CTRL_UROI_BOUND		BIT(5)
#define AP1302_AE_CTRL_IMM1			BIT(4)
#define AP1302_AE_CTRL_MANUAL_EXP_TIME_GAIN	(0U << 0)
#define AP1302_AE_CTRL_MANUAL_BV_EXP_TIME	(1U << 0)
#define AP1302_AE_CTRL_MANUAL_BV_GAIN		(2U << 0)
#define AP1302_AE_CTRL_MANUAL_BV_ISO		(3U << 0)
#define AP1302_AE_CTRL_AUTO_BV_EXP_TIME		(9U << 0)
#define AP1302_AE_CTRL_AUTO_BV_GAIN		(10U << 0)
#define AP1302_AE_CTRL_AUTO_BV_ISO		(11U << 0)
#define AP1302_AE_CTRL_FULL_AUTO		(12U << 0)
#define AP1302_AE_CTRL_MODE_MASK		0x000f
#define AP1302_AE_MANUAL_GAIN		AP1302_REG_16BIT(0x5006)
#define AP1302_AE_BV_OFF			AP1302_REG_16BIT(0x5014)
#define AP1302_AE_MET				AP1302_REG_16BIT(0x503E)
#define AP1302_AWB_CTRL				AP1302_REG_16BIT(0x5100)
#define AP1302_AWB_CTRL_RECALC			BIT(13)
#define AP1302_AWB_CTRL_POSTGAIN		BIT(12)
#define AP1302_AWB_CTRL_UNGAIN			BIT(11)
#define AP1302_AWB_CTRL_CLIP			BIT(10)
#define AP1302_AWB_CTRL_SKY			BIT(9)
#define AP1302_AWB_CTRL_FLASH			BIT(8)
#define AP1302_AWB_CTRL_FACE_OFF		(0U << 6)
#define AP1302_AWB_CTRL_FACE_IGNORE		(1U << 6)
#define AP1302_AWB_CTRL_FACE_CONSTRAINED	(2U << 6)
#define AP1302_AWB_CTRL_FACE_ONLY		(3U << 6)
#define AP1302_AWB_CTRL_IMM			BIT(5)
#define AP1302_AWB_CTRL_IMM1			BIT(4)
#define AP1302_AWB_CTRL_MODE_OFF		(0U << 0)
#define AP1302_AWB_CTRL_MODE_HORIZON		(1U << 0)
#define AP1302_AWB_CTRL_MODE_A			(2U << 0)
#define AP1302_AWB_CTRL_MODE_CWF		(3U << 0)
#define AP1302_AWB_CTRL_MODE_D50		(4U << 0)
#define AP1302_AWB_CTRL_MODE_D65		(5U << 0)
#define AP1302_AWB_CTRL_MODE_D75		(6U << 0)
#define AP1302_AWB_CTRL_MODE_MANUAL		(7U << 0)
#define AP1302_AWB_CTRL_MODE_MEASURE		(8U << 0)
#define AP1302_AWB_CTRL_MODE_AUTO		(15U << 0)
#define AP1302_AWB_CTRL_MODE_MASK		0x000f
#define AP1302_FLICK_CTRL			AP1302_REG_16BIT(0x5440)
#define AP1302_FLICK_CTRL_FREQ(n)		((n) << 8)
#define AP1302_FLICK_CTRL_ETC_IHDR_UP		BIT(6)
#define AP1302_FLICK_CTRL_ETC_DIS		BIT(5)
#define AP1302_FLICK_CTRL_FRC_OVERRIDE_MAX_ET	BIT(4)
#define AP1302_FLICK_CTRL_FRC_OVERRIDE_UPPER_ET	BIT(3)
#define AP1302_FLICK_CTRL_FRC_EN		BIT(2)
#define AP1302_FLICK_CTRL_MODE_DISABLED		(0U << 0)
#define AP1302_FLICK_CTRL_MODE_MANUAL		(1U << 0)
#define AP1302_FLICK_CTRL_MODE_AUTO		(2U << 0)
#define AP1302_SCENE_CTRL			AP1302_REG_16BIT(0x5454)
#define AP1302_SCENE_CTRL_MODE_NORMAL		(0U << 0)
#define AP1302_SCENE_CTRL_MODE_PORTRAIT		(1U << 0)
#define AP1302_SCENE_CTRL_MODE_LANDSCAPE	(2U << 0)
#define AP1302_SCENE_CTRL_MODE_SPORT		(3U << 0)
#define AP1302_SCENE_CTRL_MODE_CLOSE_UP		(4U << 0)
#define AP1302_SCENE_CTRL_MODE_NIGHT		(5U << 0)
#define AP1302_SCENE_CTRL_MODE_TWILIGHT		(6U << 0)
#define AP1302_SCENE_CTRL_MODE_BACKLIGHT	(7U << 0)
#define AP1302_SCENE_CTRL_MODE_HIGH_SENSITIVE	(8U << 0)
#define AP1302_SCENE_CTRL_MODE_NIGHT_PORTRAIT	(9U << 0)
#define AP1302_SCENE_CTRL_MODE_BEACH		(10U << 0)
#define AP1302_SCENE_CTRL_MODE_DOCUMENT		(11U << 0)
#define AP1302_SCENE_CTRL_MODE_PARTY		(12U << 0)
#define AP1302_SCENE_CTRL_MODE_FIREWORKS	(13U << 0)
#define AP1302_SCENE_CTRL_MODE_SUNSET		(14U << 0)
#define AP1302_SCENE_CTRL_MODE_AUTO		(0xffU << 0)

/* System Registers */
#define AP1302_BOOTDATA_STAGE			AP1302_REG_16BIT(0x6002)
#define AP1302_WARNING(n)			AP1302_REG_16BIT(0x6004 + (n) * 2)
#define AP1302_SENSOR_SELECT			AP1302_REG_16BIT(0x600c)
#define AP1302_SENSOR_SELECT_TP_MODE(n)		((n) << 8)
#define AP1302_SENSOR_SELECT_PATTERN_ON		BIT(7)
#define AP1302_SENSOR_SELECT_MODE_3D_ON		BIT(6)
#define AP1302_SENSOR_SELECT_CLOCK		BIT(5)
#define AP1302_SENSOR_SELECT_SINF_MIPI		BIT(4)
#define AP1302_SENSOR_SELECT_YUV		BIT(2)
#define AP1302_SENSOR_SELECT_SENSOR_TP		(0U << 0)
#define AP1302_SENSOR_SELECT_SENSOR(n)		(((n) + 1) << 0)

#define AP1302_SENSOR_TP_MODE_MASK		AP1302_SENSOR_SELECT_TP_MODE(0xF)
#define AP1302_SENSOR_SELECT_SENSOR_MASK        (0x3 << 0)

#define AP1302_SYS_START			AP1302_REG_16BIT(0x601a)
#define AP1302_SYS_START_LOAD_OTP		BIT(12)
#define AP1302_SYS_START_STALL_STATUS		BIT(9)
#define AP1302_SYS_START_STALL_EN		BIT(8)
#define AP1302_SYS_START_STALL_MODE_FRAME	(0U << 6)
#define AP1302_SYS_START_STALL_MODE_DISABLED	(1U << 6)
#define AP1302_SYS_START_STALL_MODE_POWER_DOWN	(2U << 6)
#define AP1302_SYS_START_PLL_INIT		BIT(0)
#define AP1302_SYSTEM_FREQ_IN			AP1302_REG_32BIT(0x6024)
#define AP1302_HINF_MIPI_FREQ_TGT		AP1302_REG_32BIT(0x6034)
#define AP1302_DMA_SRC				AP1302_REG_32BIT(0x60a0)
#define AP1302_DMA_DST				AP1302_REG_32BIT(0x60a4)
#define AP1302_DMA_SIP_SIPM(n)			((n) << 26)
#define AP1302_DMA_SIP_DATA_16_BIT		BIT(25)
#define AP1302_DMA_SIP_ADDR_16_BIT		BIT(24)
#define AP1302_DMA_SIP_ID(n)			((n) << 17)
#define AP1302_DMA_SIP_REG(n)			((n) << 0)
#define AP1302_DMA_SIZE				AP1302_REG_32BIT(0x60a8)
#define AP1302_DMA_CTRL				AP1302_REG_16BIT(0x60ac)
#define AP1302_DMA_CTRL_SCH_NORMAL		(0 << 12)
#define AP1302_DMA_CTRL_SCH_NEXT		(1 << 12)
#define AP1302_DMA_CTRL_SCH_NOW			(2 << 12)
#define AP1302_DMA_CTRL_DST_REG			(0 << 8)
#define AP1302_DMA_CTRL_DST_SRAM		(1 << 8)
#define AP1302_DMA_CTRL_DST_SPI			(2 << 8)
#define AP1302_DMA_CTRL_DST_SIP			(3 << 8)
#define AP1302_DMA_CTRL_SRC_REG			(0 << 4)
#define AP1302_DMA_CTRL_SRC_SRAM		(1 << 4)
#define AP1302_DMA_CTRL_SRC_SPI			(2 << 4)
#define AP1302_DMA_CTRL_SRC_SIP			(3 << 4)
#define AP1302_DMA_CTRL_MODE_32_BIT		BIT(3)
#define AP1302_DMA_CTRL_MODE_MASK		(7 << 0)
#define AP1302_DMA_CTRL_MODE_IDLE		(0 << 0)
#define AP1302_DMA_CTRL_MODE_SET		(1 << 0)
#define AP1302_DMA_CTRL_MODE_COPY		(2 << 0)
#define AP1302_DMA_CTRL_MODE_MAP		(3 << 0)
#define AP1302_DMA_CTRL_MODE_UNPACK		(4 << 0)
#define AP1302_DMA_CTRL_MODE_OTP_READ		(5 << 0)
#define AP1302_DMA_CTRL_MODE_SIP_PROBE		(6 << 0)

#define AP1302_BOOTDATA_CHECKSUM		AP1302_REG_16BIT(0x6134)

#define AP1302_BRIGHTNESS			AP1302_REG_16BIT(0x7000)
#define AP1302_CONTRAST				AP1302_REG_16BIT(0x7002)
#define AP1302_SATURATION			AP1302_REG_16BIT(0x7006)
#define AP1302_GAMMA				AP1302_REG_16BIT(0x700A)

/* Misc Registers */
#define AP1302_REG_ADV_START			0xe000
#define AP1302_ADVANCED_BASE			AP1302_REG_32BIT(0xf038)
#define AP1302_SIP_CRC				AP1302_REG_16BIT(0xf052)
#define AP1302_SIPS_SLEW_CTRL			AP1302_REG_16BIT(0xf05a)

/* Advanced System Registers */
#define AP1302_ADV_SYS_RST_EN_0			AP1302_REG_32BIT(0x00200014)
#define AP1302_ADV_SYS_CLK_EN_0			AP1302_REG_32BIT(0x0020000C)
#define AP1302_ADV_SYS_DIV_EN			AP1302_REG_32BIT(0x00200008)

#define AP1302_ADV_IRQ_SYS_INTE			AP1302_REG_32BIT(0x00230000)
#define AP1302_ADV_IRQ_SYS_INTE_TEST_COUNT	BIT(25)
#define AP1302_ADV_IRQ_SYS_INTE_HINF_1		BIT(24)
#define AP1302_ADV_IRQ_SYS_INTE_HINF_0		BIT(23)
#define AP1302_ADV_IRQ_SYS_INTE_SINF_B_MIPI_L	(7U << 20)
#define AP1302_ADV_IRQ_SYS_INTE_SINF_B_MIPI	BIT(19)
#define AP1302_ADV_IRQ_SYS_INTE_SINF_A_MIPI_L	(15U << 14)
#define AP1302_ADV_IRQ_SYS_INTE_SINF_A_MIPI	BIT(13)
#define AP1302_ADV_IRQ_SYS_INTE_SINF		BIT(12)
#define AP1302_ADV_IRQ_SYS_INTE_IPIPE_S		BIT(11)
#define AP1302_ADV_IRQ_SYS_INTE_IPIPE_B		BIT(10)
#define AP1302_ADV_IRQ_SYS_INTE_IPIPE_A		BIT(9)
#define AP1302_ADV_IRQ_SYS_INTE_IP		BIT(8)
#define AP1302_ADV_IRQ_SYS_INTE_TIMER		BIT(7)
#define AP1302_ADV_IRQ_SYS_INTE_SIPM		(3U << 6)
#define AP1302_ADV_IRQ_SYS_INTE_SIPS_ADR_RANGE	BIT(5)
#define AP1302_ADV_IRQ_SYS_INTE_SIPS_DIRECT_WRITE	BIT(4)
#define AP1302_ADV_IRQ_SYS_INTE_SIPS_FIFO_WRITE	BIT(3)
#define AP1302_ADV_IRQ_SYS_INTE_SPI		BIT(2)
#define AP1302_ADV_IRQ_SYS_INTE_GPIO_CNT	BIT(1)
#define AP1302_ADV_IRQ_SYS_INTE_GPIO_PIN	BIT(0)

#define AP1302_ADV_GPIO_DI			AP1302_REG_32BIT(0x002A0008)

/* Advanced Slave MIPI Registers */
#define AP1302_ADV_SINF_MIPI_INTERNAL_p_LANE_n_STAT(p, n) \
	AP1302_REG_32BIT(0x00420008 + (p) * 0x50000 + (n) * 0x20)
#define AP1302_LANE_ERR_LP_VAL(n)		(((n) >> 30) & 3)
#define AP1302_LANE_ERR_STATE(n)		(((n) >> 24) & 0xf)
#define AP1302_LANE_ERR				BIT(18)
#define AP1302_LANE_ABORT			BIT(17)
#define AP1302_LANE_LP_VAL(n)			(((n) >> 6) & 3)
#define AP1302_LANE_STATE(n)			((n) & 0xf)
#define AP1302_LANE_STATE_STOP_S		0x0
#define AP1302_LANE_STATE_HS_REQ_S		0x1
#define AP1302_LANE_STATE_LP_REQ_S		0x2
#define AP1302_LANE_STATE_HS_S			0x3
#define AP1302_LANE_STATE_LP_S			0x4
#define AP1302_LANE_STATE_ESC_REQ_S		0x5
#define AP1302_LANE_STATE_TURN_REQ_S		0x6
#define AP1302_LANE_STATE_ESC_S			0x7
#define AP1302_LANE_STATE_ESC_0			0x8
#define AP1302_LANE_STATE_ESC_1			0x9
#define AP1302_LANE_STATE_TURN_S		0xa
#define AP1302_LANE_STATE_TURN_MARK		0xb
#define AP1302_LANE_STATE_ERROR_S		0xc

#define AP1302_ADV_SYS_STBY_GPIO_OSEL		AP1302_REG_32BIT(0x002B0020)
#define AP1302_ADV_SYS_STBY_GPIO_IN_MASK	AP1302_REG_32BIT(0x002B0028)

#define AP1302_ADV_CAPTURE_A_FV_CNT		AP1302_REG_32BIT(0x00490040)
#define AP1302_ADV_HINF_MIPI_T3			AP1302_REG_32BIT(0x840014)
#define AP1302_TCLK_POST_MASK			0xFF
#define AP1302_TCLK_POST_SHIFT			0x0
#define AP1302_TCLK_PRE_MASK			0xFF00
#define AP1302_TCLK_PRE_SHIFT			0x8

struct ap1302_device;

enum {
	/* imx8_media_dev requires that the source pad be equal to 0 */
	AP1302_PAD_SOURCE,
	AP1302_PAD_SINK_0,
	AP1302_PAD_SINK_1,
	AP1302_PAD_MAX,
};

enum {
	AR1335_PAD_SOURCE,
};

struct ap1302_format_info {
	unsigned int code;
	u16 out_fmt;
};

/**
 *
 */
struct ap1302_format {
	/// V4L2 format
	struct v4l2_mbus_framefmt format;

	/// AP1302 register value
	const struct ap1302_format_info *info;
};

struct ap1302_size {
	unsigned int width;
	unsigned int height;
};

struct ap1302_sensor_supply {
	const char *name;
	unsigned int post_delay_us;
};

struct ap1302_sensor_info {
	const char *model;
	const char *name;
	unsigned int i2c_addr;
	struct ap1302_size resolution;
	u32 format;
};

struct ap1302_sensor {
	struct ap1302_device *ap1302;
	unsigned int index;

	struct device_node *of_node;
	bool present;

	struct v4l2_subdev sd;
	struct media_pad pad;
};

struct ap1302_mode {
	struct ap1302_size resolution;
	struct v4l2_fract min_frame_interval;
};

struct ap1302_sensor_mode {
	unsigned int mode;
	struct ap1302_size resolution;
	struct v4l2_fract min_frame_interval;
};

static inline struct ap1302_sensor *to_ap1302_sensor(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ap1302_sensor, sd);
}

struct ap1302_device {
	struct device *dev;
	struct spi_driver spidrv;
	struct spi_device *spi_dev;
	struct i2c_client *client;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *standby_gpio;
	struct gpio_desc *power_gpio;
	struct clk *clock;
	struct regmap *regmap16;
	struct regmap *regmap32;
	u32 reg_page;

	u32 system_freq_in;
	u32 hinf_mipi_freq_tgt;

	const struct firmware *fw;

	struct v4l2_fwnode_endpoint bus_cfg;

	struct mutex lock;	/* Protects formats */

	struct v4l2_subdev sd;
	struct media_pad pads[AP1302_PAD_MAX];
	struct ap1302_format formats[AP1302_PAD_MAX];

	bool streaming;

	struct v4l2_fract frame_interval;

	struct v4l2_ctrl_handler ctrls;

	const struct ap1302_sensor_info *sensor_info;
	struct ap1302_sensor sensors[2];

	struct {
		struct dentry *dir;
		struct mutex lock;
		u32 sipm_addr;
		u32 isp_addr;
	} debugfs;
};

static inline struct ap1302_device *to_ap1302(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ap1302_device, sd);
}

// Core

int ap1302_read(struct ap1302_device *ap1302, u32 reg, u32 *val);
int ap1302_write(struct ap1302_device *ap1302, u32 reg, u32 val, int *err);
int ap1302_stall(struct ap1302_device *ap1302, bool stall);

// Debug

void ap1302_debugfs_init(struct ap1302_device *ap1302);
void ap1302_debugfs_cleanup(struct ap1302_device *ap1302);

// Firmware

int ap1302_request_firmware(struct ap1302_device *ap1302);
int ap1302_load_firmware(struct ap1302_device *ap1302);

// sysfs

int create_sysfs_attributes(struct ap1302_device *ap1302);
void remove_sysfs_attributes(struct ap1302_device *ap1302);

// V4L2 Controls

int ap1302_ctrls_init(struct ap1302_device *ap1302);
void ap1302_ctrls_cleanup(struct ap1302_device *ap1302);

// SPI

int ap1302_spi_init(struct ap1302_device *ap1302);
int ap1302_spi_write(struct ap1302_device *ap1302, u32 reg, u32 val, int *err);
int ap1302_spi_write_block(struct ap1302_device *ap1302, u32 addr, const u8 *data, u32 len);
int ap1302_spi_read(struct ap1302_device *ap1302, u32 reg, u32 *val);

int ap1302_register_spi_driver(struct ap1302_device *ap1302);
void ap1302_unregister_spi_driver(struct ap1302_device *ap1302);

#endif
