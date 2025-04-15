// SPDX-License-Identifier: GPL-2.0+
/*
 * RZ/G2L Display Unit CRTCs
 *
 * Copyright (C) 2023 Renesas Electronics Corporation
 *
 * Based on rcar_du_crtc.c
 */

#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_vblank.h>

#include "rzg2l_du_crtc.h"
#include "rzg2l_du_drv.h"
#include "rzg2l_du_encoder.h"
#include "rzg2l_du_kms.h"
#include "rzg2l_du_vsp.h"
#include "rzg2l_mipi_dsi.h"

#define DU_MCR0			0x00
#define DU_MCR0_DPI_OE		BIT(0)
#define DU_MCR0_DI_EN		BIT(8)

#define DU_DITR0		0x10
#define DU_DITR0_DEMD_HIGH	(BIT(8) | BIT(9))
#define DU_DITR0_VSPOL		BIT(16)
#define DU_DITR0_HSPOL		BIT(17)

#define DU_DITR1		0x14
#define DU_DITR1_VSA(x)		((x) << 0)
#define DU_DITR1_VACTIVE(x)	((x) << 16)

#define DU_DITR2		0x18
#define DU_DITR2_VBP(x)		((x) << 0)
#define DU_DITR2_VFP(x)		((x) << 16)

#define DU_DITR3		0x1c
#define DU_DITR3_HSA(x)		((x) << 0)
#define DU_DITR3_HACTIVE(x)	((x) << 16)

#define DU_DITR4		0x20
#define DU_DITR4_HBP(x)		((x) << 0)
#define DU_DITR4_HFP(x)		((x) << 16)

#define DU_MCR1			0x40
#define DU_MCR1_PB_AUTOCLR	BIT(16)

#define DU_PBCR0		0x4c
#define DU_PBCR0_PB_DEP(x)	((x) << 0)

/* -----------------------------------------------------------------------------
 * Hardware Setup
 */

static void rzg2l_du_crtc_set_display_timing(struct rzg2l_du_crtc *rcrtc)
{
	const struct drm_display_mode *mode = &rcrtc->crtc.state->adjusted_mode;
	unsigned long mode_clock = mode->clock * 1000;
	u32 ditr0, ditr1, ditr2, ditr3, ditr4, pbcr0;
	struct rzg2l_du_device *rcdu = rcrtc->dev;
	struct device_node *dsi_node;
	struct platform_device *pdev_dsi;

	dsi_node = of_find_node_by_name(NULL, "dsi");
	pdev_dsi = of_find_device_by_node(dsi_node);

	if (of_device_is_available(dsi_node)) {
		/* DSI handle */
		void __iomem *cpg_base = ioremap(0x11010000, 0x1000);
		u32 i;
		u32 parallel_out;
		struct cpg_param param;
		int lanes, bpp;
		u32 pix_clk = mode->clock * 1000;
		unsigned long long hs_clk;
		unsigned long long pll5_clk;
		unsigned long long divide_val;
		u32 dsi_div;

		/* Common settings */
		param.frequency = 0;
		param.pl5_refdiv = 1;
		param.pl5_divval = 0;
		param.pl5_spread = 0x16;

		lanes = rzg2l_mipi_dsi_get_data_lanes(pdev_dsi);
		bpp = rzg2l_mipi_dsi_get_bpp(pdev_dsi);

		parallel_out = 0;

		/* Recommended values */
		param.pl5_postdiv1 = 1;
		param.pl5_postdiv2 = 1;

		/* Calculate MIPI DSI High Speed clock and PLL clock(16x) */
		hs_clk = ((long long)bpp * pix_clk) / (8 * lanes);
		pll5_clk = hs_clk * 16;
		if (pll5_clk > 1500000000) {
			if (pll5_clk > 3000000000) {
				dev_err(rcdu->dev, "Exceeded max frequency\n");
				return;
			}
			param.sel_pll5_4 = 0;	/* 3.0 GHz */
		} else {
			param.sel_pll5_4 = 1;	/* 1.5 GHz */
		}

		/* Divide raw bit clock by source clock. */
		/* Numerator portion (integer) */
		divide_val = pll5_clk * param.pl5_refdiv * param.pl5_postdiv1 * param.pl5_postdiv2;
		param.pl5_intin = divide_val / OSCLK_HZ;

		/* Denominator portion (multiplied by 16k to become an integer) */
		/* Remove integer portion */
		divide_val = divide_val % OSCLK_HZ;
		/* Convert from decimal to integer */
		divide_val = divide_val * 16 * 1024 * 1024;
		/* Now we can divide */
		divide_val = divide_val / OSCLK_HZ;
		param.pl5_fracin = divide_val;

		/* How much we need to divide own our PLL */
		dsi_div = pll5_clk / pix_clk;

		/* Clock source is 3G or 1.5G? */
		if (param.sel_pll5_4)
			dsi_div /= 2;

		/* Find possible clock divide ratios.
		 * The equation is: dsi_div = (2 ^ dis_div_a) * (1 + dis_div_b)
		 * With div_a fixed, we get: dis_div_b = (dsi_div / (2 ^ dis_div_a)) - 1
		 *   div_a can be 0-4
		 *   div_b can be 0-16
		 */
		for (i = 0; i < 4; i++) {
			param.dsi_div_a = i;
			param.dsi_div_b = (dsi_div / (1 << i)) - 1;
			if (param.dsi_div_b > 16)
				continue;
			break;
		}

		if (i == 4) {
			/* Could not find any combinations */
			dev_err(rcdu->dev, "Cannot calculate frequency.\n");
			return;
		}

		/* CPG_PLL5_STBY: RESETB=0 */
		reg_write(cpg_base + 0x0140, 0x00150000);

		/* CPG_OTHERFUNC1_REG: SEL_PLL5_3 clock (1.5GHz or 3.0GHz)*/
		if (!parallel_out)
			reg_write(cpg_base + 0xbe8, 0x10000 | param.sel_pll5_4);

		/* CPG_PL2_DDIV: DIV_DSI_LPCLK */
		reg_write(cpg_base + 0x0204, 0x10000000 |
				(CPG_LPCLK_DIV << 12));
		/* CPG_PL5_SDIV: DIV_DSI_A, DIV_DSI_B */
		reg_write(cpg_base + 0x0420, 0x01010000 |
				(param.dsi_div_a << 0) |
				(param.dsi_div_b << 8));
		/* CPG_PLL5_CLK1: POSTDIV1, POSTDIV2, REFDIV */
		reg_write(cpg_base + 0x0144,
				(param.pl5_postdiv1 << 0) |
				(param.pl5_postdiv2 << 4) |
				(param.pl5_refdiv << 8));
		/* CPG_PLL5_CLK3: DIVVAL=6, FRACIN */
		reg_write(cpg_base + 0x014C,
				(param.pl5_divval << 0) |
				(param.pl5_fracin << 8));
		/* CPG_PLL5_CLK4: INTIN */
		reg_write(cpg_base + 0x0150, 0x000000ff |
				(param.pl5_intin << 16));
		/* CPG_PLL5_CLK5: SPREAD */
		reg_write(cpg_base + 0x0154,
				(param.pl5_intin << 16));

		/* CPG_PLL5_STBY: RESETB=1 */
		reg_write(cpg_base + 0x0140, 0x00150001);

		iounmap(cpg_base);
		clk_prepare_enable(rcrtc->rzg2l_clocks.dclk);
	} else {
		clk_prepare_enable(rcrtc->rzg2l_clocks.dclk);
		clk_set_rate(rcrtc->rzg2l_clocks.dclk, mode_clock);
	}

	ditr0 = (DU_DITR0_DEMD_HIGH
	      | ((mode->flags & DRM_MODE_FLAG_PVSYNC) ? DU_DITR0_VSPOL : 0)
	      | ((mode->flags & DRM_MODE_FLAG_PHSYNC) ? DU_DITR0_HSPOL : 0));

	ditr1 = DU_DITR1_VSA(mode->vsync_end - mode->vsync_start)
	      | DU_DITR1_VACTIVE(mode->vdisplay);

	ditr2 = DU_DITR2_VBP(mode->vtotal - mode->vsync_end)
	      | DU_DITR2_VFP(mode->vsync_start - mode->vdisplay);

	ditr3 = DU_DITR3_HSA(mode->hsync_end - mode->hsync_start)
	      | DU_DITR3_HACTIVE(mode->hdisplay);

	ditr4 = DU_DITR4_HBP(mode->htotal - mode->hsync_end)
	      | DU_DITR4_HFP(mode->hsync_start - mode->hdisplay);

	pbcr0 = DU_PBCR0_PB_DEP(0x1f);

	writel(ditr0, rcdu->mmio + DU_DITR0);
	writel(ditr1, rcdu->mmio + DU_DITR1);
	writel(ditr2, rcdu->mmio + DU_DITR2);
	writel(ditr3, rcdu->mmio + DU_DITR3);
	writel(ditr4, rcdu->mmio + DU_DITR4);
	writel(pbcr0, rcdu->mmio + DU_PBCR0);

	/* Enable auto clear */
	writel(DU_MCR1_PB_AUTOCLR, rcdu->mmio + DU_MCR1);
}

/* -----------------------------------------------------------------------------
 * Page Flip
 */

void rzg2l_du_crtc_finish_page_flip(struct rzg2l_du_crtc *rcrtc)
{
	struct drm_pending_vblank_event *event;
	struct drm_device *dev = rcrtc->crtc.dev;
	unsigned long flags;

	spin_lock_irqsave(&dev->event_lock, flags);
	event = rcrtc->event;
	rcrtc->event = NULL;
	spin_unlock_irqrestore(&dev->event_lock, flags);

	if (!event)
		return;

	spin_lock_irqsave(&dev->event_lock, flags);
	drm_crtc_send_vblank_event(&rcrtc->crtc, event);
	wake_up(&rcrtc->flip_wait);
	spin_unlock_irqrestore(&dev->event_lock, flags);

	drm_crtc_vblank_put(&rcrtc->crtc);
}

static bool rzg2l_du_crtc_page_flip_pending(struct rzg2l_du_crtc *rcrtc)
{
	struct drm_device *dev = rcrtc->crtc.dev;
	unsigned long flags;
	bool pending;

	spin_lock_irqsave(&dev->event_lock, flags);
	pending = rcrtc->event;
	spin_unlock_irqrestore(&dev->event_lock, flags);

	return pending;
}

static void rzg2l_du_crtc_wait_page_flip(struct rzg2l_du_crtc *rcrtc)
{
	struct rzg2l_du_device *rcdu = rcrtc->dev;

	if (wait_event_timeout(rcrtc->flip_wait,
			       !rzg2l_du_crtc_page_flip_pending(rcrtc),
			       msecs_to_jiffies(50)))
		return;

	dev_warn(rcdu->dev, "page flip timeout\n");

	rzg2l_du_crtc_finish_page_flip(rcrtc);
}

/* -----------------------------------------------------------------------------
 * Start/Stop and Suspend/Resume
 */

static void rzg2l_du_crtc_setup(struct rzg2l_du_crtc *rcrtc)
{
	/* Configure display timings and output routing */
	rzg2l_du_crtc_set_display_timing(rcrtc);

	/* Enable the VSP compositor. */
	rzg2l_du_vsp_enable(rcrtc);

	/* Turn vertical blanking interrupt reporting on. */
	drm_crtc_vblank_on(&rcrtc->crtc);
}

static int rzg2l_du_crtc_get(struct rzg2l_du_crtc *rcrtc)
{
	int ret;

	/*
	 * Guard against double-get, as the function is called from both the
	 * .atomic_enable() and .atomic_flush() handlers.
	 */
	if (rcrtc->initialized)
		return 0;

	ret = clk_prepare_enable(rcrtc->rzg2l_clocks.aclk);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(rcrtc->rzg2l_clocks.pclk);
	if (ret < 0)
		goto error_bus_clock;

	ret = reset_control_deassert(rcrtc->rstc);
	if (ret < 0)
		goto error_peri_clock;

	rzg2l_du_crtc_setup(rcrtc);
	rcrtc->initialized = true;

	return 0;

error_peri_clock:
	clk_disable_unprepare(rcrtc->rzg2l_clocks.pclk);
error_bus_clock:
	clk_disable_unprepare(rcrtc->rzg2l_clocks.aclk);
	return ret;
}

static void rzg2l_du_crtc_put(struct rzg2l_du_crtc *rcrtc)
{
	clk_disable_unprepare(rcrtc->rzg2l_clocks.dclk);
	reset_control_assert(rcrtc->rstc);
	clk_disable_unprepare(rcrtc->rzg2l_clocks.pclk);
	clk_disable_unprepare(rcrtc->rzg2l_clocks.aclk);

	rcrtc->initialized = false;
}

static void rzg2l_du_start_stop(struct rzg2l_du_crtc *rcrtc, bool start)
{
	struct rzg2l_du_crtc_state *rstate = to_rzg2l_crtc_state(rcrtc->crtc.state);
	struct rzg2l_du_device *rcdu = rcrtc->dev;
	u32 val = DU_MCR0_DI_EN;

	if (rstate->outputs & BIT(RZG2L_DU_OUTPUT_DPAD0))
		val |= DU_MCR0_DPI_OE;

	writel(start ? val : 0, rcdu->mmio + DU_MCR0);
}

static void rzg2l_du_crtc_start(struct rzg2l_du_crtc *rcrtc)
{
	rzg2l_du_start_stop(rcrtc, true);
}

static void rzg2l_du_crtc_stop(struct rzg2l_du_crtc *rcrtc)
{
	struct drm_crtc *crtc = &rcrtc->crtc;

	/*
	 * Disable vertical blanking interrupt reporting. We first need to wait
	 * for page flip completion before stopping the CRTC as userspace
	 * expects page flips to eventually complete.
	 */
	rzg2l_du_crtc_wait_page_flip(rcrtc);
	drm_crtc_vblank_off(crtc);

	/* Disable the VSP compositor. */
	rzg2l_du_vsp_disable(rcrtc);

	rzg2l_du_start_stop(rcrtc, false);
}

/* -----------------------------------------------------------------------------
 * CRTC Functions
 */

static int rzg2l_du_crtc_atomic_check(struct drm_crtc *crtc,
	struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state,
						crtc);
	struct rzg2l_du_crtc_state *rstate = to_rzg2l_crtc_state(crtc_state);
	struct drm_encoder *encoder;

	/* Store the routes from the CRTC output to the DU outputs. */
	rstate->outputs = 0;

	drm_for_each_encoder_mask(encoder, crtc->dev,
	crtc_state->encoder_mask) {
		struct rzg2l_du_encoder *renc;

		/* Skip the writeback encoder. */
		if (encoder->encoder_type == DRM_MODE_ENCODER_VIRTUAL)
			continue;

		renc = to_rzg2l_encoder(encoder);
		rstate->outputs |= BIT(renc->output);
	}

	return 0;
}

static void rzg2l_du_crtc_atomic_enable(struct drm_crtc *crtc,
					struct drm_atomic_state *state)
{
	struct rzg2l_du_crtc *rcrtc = to_rzg2l_crtc(crtc);

	rzg2l_du_crtc_get(rcrtc);

	rzg2l_du_crtc_start(rcrtc);
}

static void rzg2l_du_crtc_atomic_disable(struct drm_crtc *crtc,
					 struct drm_atomic_state *state)
{
	struct rzg2l_du_crtc *rcrtc = to_rzg2l_crtc(crtc);

	rzg2l_du_crtc_stop(rcrtc);
	rzg2l_du_crtc_put(rcrtc);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static void rzg2l_du_crtc_atomic_flush(struct drm_crtc *crtc,
				       struct drm_atomic_state *state)
{
	struct rzg2l_du_crtc *rcrtc = to_rzg2l_crtc(crtc);
	struct drm_device *dev = rcrtc->crtc.dev;
	unsigned long flags;

	WARN_ON(!crtc->state->enable);

	if (crtc->state->event) {
		WARN_ON(drm_crtc_vblank_get(crtc) != 0);

		spin_lock_irqsave(&dev->event_lock, flags);
		rcrtc->event = crtc->state->event;
		crtc->state->event = NULL;
		spin_unlock_irqrestore(&dev->event_lock, flags);
	}

	rzg2l_du_vsp_atomic_flush(rcrtc);
}

static enum drm_mode_status
rzg2l_du_crtc_mode_valid(struct drm_crtc *crtc,
			const struct drm_display_mode *mode)
{
	struct rzg2l_du_crtc *rcrtc = to_rzg2l_crtc(crtc);
	struct rzg2l_du_crtc_state *rstate =
					to_rzg2l_crtc_state(rcrtc->crtc.state);

	if ((rstate->outputs & BIT(RZG2L_DU_OUTPUT_DPAD0)) &&
		mode->clock > 83500)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static const struct drm_crtc_helper_funcs crtc_helper_funcs = {
	.atomic_check = rzg2l_du_crtc_atomic_check,
	.atomic_flush = rzg2l_du_crtc_atomic_flush,
	.atomic_enable = rzg2l_du_crtc_atomic_enable,
	.atomic_disable = rzg2l_du_crtc_atomic_disable,
	.mode_valid = rzg2l_du_crtc_mode_valid,
};

static struct drm_crtc_state *
rzg2l_du_crtc_atomic_duplicate_state(struct drm_crtc *crtc)
{
	struct rzg2l_du_crtc_state *state;
	struct rzg2l_du_crtc_state *copy;

	if (WARN_ON(!crtc->state))
		return NULL;

	state = to_rzg2l_crtc_state(crtc->state);
	copy = kmemdup(state, sizeof(*state), GFP_KERNEL);
	if (!copy)
		return NULL;

	__drm_atomic_helper_crtc_duplicate_state(crtc, &copy->state);

	return &copy->state;
}

static void rzg2l_du_crtc_atomic_destroy_state(struct drm_crtc *crtc,
					       struct drm_crtc_state *state)
{
	__drm_atomic_helper_crtc_destroy_state(state);
	kfree(to_rzg2l_crtc_state(state));
}

static void rzg2l_du_crtc_reset(struct drm_crtc *crtc)
{
	struct rzg2l_du_crtc_state *state;

	if (crtc->state) {
		rzg2l_du_crtc_atomic_destroy_state(crtc, crtc->state);
		crtc->state = NULL;
	}

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return;

	__drm_atomic_helper_crtc_reset(crtc, &state->state);
}

static int rzg2l_du_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct rzg2l_du_crtc *rcrtc = to_rzg2l_crtc(crtc);

	rcrtc->vblank_enable = true;

	return 0;
}

static void rzg2l_du_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct rzg2l_du_crtc *rcrtc = to_rzg2l_crtc(crtc);

	rcrtc->vblank_enable = false;
}

static const struct drm_crtc_funcs crtc_funcs_rz = {
	.reset = rzg2l_du_crtc_reset,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = rzg2l_du_crtc_atomic_duplicate_state,
	.atomic_destroy_state = rzg2l_du_crtc_atomic_destroy_state,
	.enable_vblank = rzg2l_du_crtc_enable_vblank,
	.disable_vblank = rzg2l_du_crtc_disable_vblank,
};

/* -----------------------------------------------------------------------------
 * Initialization
 */

int rzg2l_du_crtc_create(struct rzg2l_du_device *rcdu)
{
	struct rzg2l_du_crtc *rcrtc = &rcdu->crtcs[0];
	struct drm_crtc *crtc = &rcrtc->crtc;
	struct drm_plane *primary;
	int ret;

	rcrtc->rstc = devm_reset_control_get_shared(rcdu->dev, NULL);
	if (IS_ERR(rcrtc->rstc)) {
		dev_err(rcdu->dev, "can't get cpg reset\n");
		return PTR_ERR(rcrtc->rstc);
	}

	rcrtc->rzg2l_clocks.aclk = devm_clk_get(rcdu->dev, "aclk");
	if (IS_ERR(rcrtc->rzg2l_clocks.aclk)) {
		dev_err(rcdu->dev, "no axi clock for DU\n");
		return PTR_ERR(rcrtc->rzg2l_clocks.aclk);
	}

	rcrtc->rzg2l_clocks.pclk = devm_clk_get(rcdu->dev, "pclk");
	if (IS_ERR(rcrtc->rzg2l_clocks.pclk)) {
		dev_err(rcdu->dev, "no peripheral clock for DU\n");
		return PTR_ERR(rcrtc->rzg2l_clocks.pclk);
	}

	rcrtc->rzg2l_clocks.dclk = devm_clk_get(rcdu->dev, "vclk");
	if (IS_ERR(rcrtc->rzg2l_clocks.dclk)) {
		dev_err(rcdu->dev, "no video clock for DU\n");
		return PTR_ERR(rcrtc->rzg2l_clocks.dclk);
	}

	init_waitqueue_head(&rcrtc->flip_wait);
	rcrtc->dev = rcdu;

	primary = rzg2l_du_vsp_get_drm_plane(rcrtc, rcrtc->vsp_pipe);
	if (IS_ERR(primary))
		return PTR_ERR(primary);

	ret = drmm_crtc_init_with_planes(&rcdu->ddev, crtc, primary, NULL,
					 &crtc_funcs_rz, NULL);
	if (ret < 0)
		return ret;

	drm_crtc_helper_add(crtc, &crtc_helper_funcs);

	return 0;
}
