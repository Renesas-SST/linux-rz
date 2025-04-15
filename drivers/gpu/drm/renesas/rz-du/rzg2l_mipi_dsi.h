/* SPDX-License-Identifier: GPL-2.0 */
/*
 * RZ/G2L MIPI DSI Encoder Header
 *
 * Copyright (C) 2021 Renesas Electronics Corporation
 */

#ifndef __RZG2L_MIPI_DSI_H__
#define __RZG2L_MIPI_DSI_H__

struct platform_device;

#if IS_ENABLED(CONFIG_DRM_RZG2L_MIPI_DSI)
int rzg2l_mipi_dsi_get_data_lanes(struct platform_device *pdev);
int rzg2l_mipi_dsi_get_bpp(struct platform_device *pdev);
#else
static inline int rzg2l_mipi_dsi_get_data_lanes(struct platform_device *pdev)
{
	return 0;
}
static inline int rzg2l_mipi_dsi_get_bpp(struct platform_device *pdev)
{
	return 0;
}
#endif /* CONFIG_DRM_RZG2L_MIPI_DSI */
#endif /* __RZG2L_MIPI_DSI_H__ */
