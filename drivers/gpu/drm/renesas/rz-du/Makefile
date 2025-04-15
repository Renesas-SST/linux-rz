# SPDX-License-Identifier: GPL-2.0

CFLAGS_rzg2l_du_crtc.o := -I$(srctree)/drivers/gpu/drm/renesas/rcar-du

rzg2l-du-drm-y := rzg2l_du_crtc.o \
		  rzg2l_du_drv.o \
		  rzg2l_du_encoder.o \
		  rzg2l_du_kms.o \

rzg2l-du-drm-$(CONFIG_VIDEO_RENESAS_VSP1)	+= rzg2l_du_vsp.o
obj-$(CONFIG_DRM_RZG2L_DU)		+= rzg2l-du-drm.o

obj-$(CONFIG_DRM_RZG2L_MIPI_DSI)	+= rzg2l_mipi_dsi.o
