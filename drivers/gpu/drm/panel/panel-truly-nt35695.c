// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2021 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct truly_nt35695 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator *supply;
	struct gpio_desc *reset_gpio;
	bool prepared;
};

static inline struct truly_nt35695 *to_truly_nt35695(struct drm_panel *panel)
{
	return container_of(panel, struct truly_nt35695, panel);
}

#define dsi_dcs_write_seq(dsi, seq...) do {				\
		static const u8 d[] = { seq };				\
		int ret;						\
		ret = mipi_dsi_dcs_write_buffer(dsi, d, ARRAY_SIZE(d));	\
		if (ret < 0)						\
			return ret;					\
	} while (0)

static void truly_nt35695_reset(struct truly_nt35695 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);
}

static int truly_nt35695_on(struct truly_nt35695 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi_dcs_write_seq(dsi, 0xff, 0x20);
	usleep_range(1000, 2000);
	dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	dsi_dcs_write_seq(dsi, 0x00, 0x01);
	dsi_dcs_write_seq(dsi, 0x01, 0x55);
	dsi_dcs_write_seq(dsi, 0x02, 0x45);
	dsi_dcs_write_seq(dsi, 0x03, 0x55);
	dsi_dcs_write_seq(dsi, 0x05, 0x50);
	dsi_dcs_write_seq(dsi, 0x04, 0x00);
	dsi_dcs_write_seq(dsi, 0x06, 0x9e);
	dsi_dcs_write_seq(dsi, 0x07, 0xa8);
	dsi_dcs_write_seq(dsi, 0x08, 0x0c);
	dsi_dcs_write_seq(dsi, 0x09, 0x73);
	dsi_dcs_write_seq(dsi, 0x0a, 0x73);
	dsi_dcs_write_seq(dsi, 0x0d, 0x00);
	dsi_dcs_write_seq(dsi, 0x0b, 0x96);
	dsi_dcs_write_seq(dsi, 0x0c, 0x96);
	dsi_dcs_write_seq(dsi, 0x0e, 0x00);
	dsi_dcs_write_seq(dsi, 0x0f, 0x00);
	dsi_dcs_write_seq(dsi, 0x11, 0x27);
	dsi_dcs_write_seq(dsi, 0x12, 0x63);
	dsi_dcs_write_seq(dsi, 0x13, 0xf3);
	dsi_dcs_write_seq(dsi, 0x14, 0x4a);
	dsi_dcs_write_seq(dsi, 0x15, 0x93);
	dsi_dcs_write_seq(dsi, 0x16, 0x93);
	dsi_dcs_write_seq(dsi, 0x17, 0x63);
	dsi_dcs_write_seq(dsi, 0x18, 0x63);
	dsi_dcs_write_seq(dsi, 0x19, 0x63);
	dsi_dcs_write_seq(dsi, 0x1a, 0x63);
	dsi_dcs_write_seq(dsi, 0x68, 0x03);
	dsi_dcs_write_seq(dsi, 0x69, 0xa0);
	dsi_dcs_write_seq(dsi, 0x6a, 0x33);
	dsi_dcs_write_seq(dsi, 0x6b, 0x40);
	dsi_dcs_write_seq(dsi, 0x6c, 0x33);
	dsi_dcs_write_seq(dsi, 0x6d, 0x68);
	dsi_dcs_write_seq(dsi, 0x75, 0x00);
	dsi_dcs_write_seq(dsi, 0x76, 0x00);
	dsi_dcs_write_seq(dsi, 0x77, 0x00);
	dsi_dcs_write_seq(dsi, 0x78, 0x22);
	dsi_dcs_write_seq(dsi, 0x79, 0x00);
	dsi_dcs_write_seq(dsi, 0x7a, 0x46);
	dsi_dcs_write_seq(dsi, 0x7b, 0x00);
	dsi_dcs_write_seq(dsi, 0x7c, 0x5c);
	dsi_dcs_write_seq(dsi, 0x7d, 0x00);
	dsi_dcs_write_seq(dsi, 0x7e, 0x76);
	dsi_dcs_write_seq(dsi, 0x7f, 0x00);
	dsi_dcs_write_seq(dsi, 0x80, 0x8d);
	dsi_dcs_write_seq(dsi, 0x81, 0x00);
	dsi_dcs_write_seq(dsi, 0x82, 0xa6);
	dsi_dcs_write_seq(dsi, 0x83, 0x00);
	dsi_dcs_write_seq(dsi, 0x84, 0xb8);
	dsi_dcs_write_seq(dsi, 0x85, 0x00);
	dsi_dcs_write_seq(dsi, 0x86, 0xc7);
	dsi_dcs_write_seq(dsi, 0x87, 0x00);
	dsi_dcs_write_seq(dsi, 0x88, 0xf6);
	dsi_dcs_write_seq(dsi, 0x89, 0x01);
	dsi_dcs_write_seq(dsi, 0x8a, 0x1d);
	dsi_dcs_write_seq(dsi, 0x8b, 0x01);
	dsi_dcs_write_seq(dsi, 0x8c, 0x54);
	dsi_dcs_write_seq(dsi, 0x8d, 0x01);
	dsi_dcs_write_seq(dsi, 0x8e, 0x81);
	dsi_dcs_write_seq(dsi, 0x8f, 0x01);
	dsi_dcs_write_seq(dsi, 0x90, 0xcb);
	dsi_dcs_write_seq(dsi, 0x91, 0x02);
	dsi_dcs_write_seq(dsi, 0x92, 0x05);
	dsi_dcs_write_seq(dsi, 0x93, 0x02);
	dsi_dcs_write_seq(dsi, 0x94, 0x07);
	dsi_dcs_write_seq(dsi, 0x95, 0x02);
	dsi_dcs_write_seq(dsi, 0x96, 0x47);
	dsi_dcs_write_seq(dsi, 0x97, 0x02);
	dsi_dcs_write_seq(dsi, 0x98, 0x82);
	dsi_dcs_write_seq(dsi, 0x99, 0x02);
	dsi_dcs_write_seq(dsi, 0x9a, 0xab);
	dsi_dcs_write_seq(dsi, 0x9b, 0x02);
	dsi_dcs_write_seq(dsi, 0x9c, 0xdc);
	dsi_dcs_write_seq(dsi, 0x9d, 0x03);
	dsi_dcs_write_seq(dsi, 0x9e, 0x01);
	dsi_dcs_write_seq(dsi, 0x9f, 0x03);
	dsi_dcs_write_seq(dsi, 0xa0, 0x3a);
	dsi_dcs_write_seq(dsi, MIPI_DCS_READ_PPS_START, 0x03);
	dsi_dcs_write_seq(dsi, 0xa3, 0x56);
	dsi_dcs_write_seq(dsi, 0xa4, 0x03);
	dsi_dcs_write_seq(dsi, 0xa5, 0x6d);
	dsi_dcs_write_seq(dsi, 0xa6, 0x03);
	dsi_dcs_write_seq(dsi, 0xa7, 0x89);
	dsi_dcs_write_seq(dsi, MIPI_DCS_READ_PPS_CONTINUE, 0x03);
	dsi_dcs_write_seq(dsi, 0xaa, 0xa3);
	dsi_dcs_write_seq(dsi, 0xab, 0x03);
	dsi_dcs_write_seq(dsi, 0xac, 0xc9);
	dsi_dcs_write_seq(dsi, 0xad, 0x03);
	dsi_dcs_write_seq(dsi, 0xae, 0xdd);
	dsi_dcs_write_seq(dsi, 0xaf, 0x03);
	dsi_dcs_write_seq(dsi, 0xb0, 0xf5);
	dsi_dcs_write_seq(dsi, 0xb1, 0x03);
	dsi_dcs_write_seq(dsi, 0xb2, 0xff);
	dsi_dcs_write_seq(dsi, 0xb3, 0x00);
	dsi_dcs_write_seq(dsi, 0xb4, 0x00);
	dsi_dcs_write_seq(dsi, 0xb5, 0x00);
	dsi_dcs_write_seq(dsi, 0xb6, 0x22);
	dsi_dcs_write_seq(dsi, 0xb7, 0x00);
	dsi_dcs_write_seq(dsi, 0xb8, 0x46);
	dsi_dcs_write_seq(dsi, 0xb9, 0x00);
	dsi_dcs_write_seq(dsi, 0xba, 0x5c);
	dsi_dcs_write_seq(dsi, 0xbb, 0x00);
	dsi_dcs_write_seq(dsi, 0xbc, 0x76);
	dsi_dcs_write_seq(dsi, 0xbd, 0x00);
	dsi_dcs_write_seq(dsi, 0xbe, 0x8d);
	dsi_dcs_write_seq(dsi, 0xbf, 0x00);
	dsi_dcs_write_seq(dsi, 0xc0, 0xa6);
	dsi_dcs_write_seq(dsi, 0xc1, 0x00);
	dsi_dcs_write_seq(dsi, 0xc2, 0xb8);
	dsi_dcs_write_seq(dsi, 0xc3, 0x00);
	dsi_dcs_write_seq(dsi, 0xc4, 0xc7);
	dsi_dcs_write_seq(dsi, 0xc5, 0x00);
	dsi_dcs_write_seq(dsi, 0xc6, 0xf6);
	dsi_dcs_write_seq(dsi, 0xc7, 0x01);
	dsi_dcs_write_seq(dsi, 0xc8, 0x1d);
	dsi_dcs_write_seq(dsi, 0xc9, 0x01);
	dsi_dcs_write_seq(dsi, 0xca, 0x54);
	dsi_dcs_write_seq(dsi, 0xcb, 0x01);
	dsi_dcs_write_seq(dsi, 0xcc, 0x81);
	dsi_dcs_write_seq(dsi, 0xcd, 0x01);
	dsi_dcs_write_seq(dsi, 0xce, 0xcb);
	dsi_dcs_write_seq(dsi, 0xcf, 0x02);
	dsi_dcs_write_seq(dsi, 0xd0, 0x05);
	dsi_dcs_write_seq(dsi, 0xd1, 0x02);
	dsi_dcs_write_seq(dsi, 0xd2, 0x07);
	dsi_dcs_write_seq(dsi, 0xd3, 0x02);
	dsi_dcs_write_seq(dsi, 0xd4, 0x47);
	dsi_dcs_write_seq(dsi, 0xd5, 0x02);
	dsi_dcs_write_seq(dsi, 0xd6, 0x82);
	dsi_dcs_write_seq(dsi, 0xd7, 0x02);
	dsi_dcs_write_seq(dsi, 0xd8, 0xab);
	dsi_dcs_write_seq(dsi, 0xd9, 0x02);
	dsi_dcs_write_seq(dsi, 0xda, 0xdc);
	dsi_dcs_write_seq(dsi, 0xdb, 0x03);
	dsi_dcs_write_seq(dsi, 0xdc, 0x01);
	dsi_dcs_write_seq(dsi, 0xdd, 0x03);
	dsi_dcs_write_seq(dsi, 0xde, 0x3a);
	dsi_dcs_write_seq(dsi, 0xdf, 0x03);
	dsi_dcs_write_seq(dsi, 0xe0, 0x56);
	dsi_dcs_write_seq(dsi, 0xe1, 0x03);
	dsi_dcs_write_seq(dsi, 0xe2, 0x6d);
	dsi_dcs_write_seq(dsi, 0xe3, 0x03);
	dsi_dcs_write_seq(dsi, 0xe4, 0x89);
	dsi_dcs_write_seq(dsi, 0xe5, 0x03);
	dsi_dcs_write_seq(dsi, 0xe6, 0xa3);
	dsi_dcs_write_seq(dsi, 0xe7, 0x03);
	dsi_dcs_write_seq(dsi, 0xe8, 0xc9);
	dsi_dcs_write_seq(dsi, 0xe9, 0x03);
	dsi_dcs_write_seq(dsi, 0xea, 0xdd);
	dsi_dcs_write_seq(dsi, 0xeb, 0x03);
	dsi_dcs_write_seq(dsi, 0xec, 0xf5);
	dsi_dcs_write_seq(dsi, 0xed, 0x03);
	dsi_dcs_write_seq(dsi, 0xee, 0xff);
	dsi_dcs_write_seq(dsi, 0xef, 0x00);
	dsi_dcs_write_seq(dsi, 0xf0, 0x00);
	dsi_dcs_write_seq(dsi, 0xf1, 0x00);
	dsi_dcs_write_seq(dsi, 0xf2, 0x22);
	dsi_dcs_write_seq(dsi, 0xf3, 0x00);
	dsi_dcs_write_seq(dsi, 0xf4, 0x46);
	dsi_dcs_write_seq(dsi, 0xf5, 0x00);
	dsi_dcs_write_seq(dsi, 0xf6, 0x5c);
	dsi_dcs_write_seq(dsi, 0xf7, 0x00);
	dsi_dcs_write_seq(dsi, 0xf8, 0x76);
	dsi_dcs_write_seq(dsi, 0xf9, 0x00);
	dsi_dcs_write_seq(dsi, 0xfa, 0x8d);
	dsi_dcs_write_seq(dsi, 0xff, 0x21);
	usleep_range(1000, 2000);
	dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	dsi_dcs_write_seq(dsi, 0x00, 0x00);
	dsi_dcs_write_seq(dsi, 0x01, 0xa6);
	dsi_dcs_write_seq(dsi, 0x02, 0x00);
	dsi_dcs_write_seq(dsi, 0x03, 0xb8);
	dsi_dcs_write_seq(dsi, 0x04, 0x00);
	dsi_dcs_write_seq(dsi, 0x05, 0xc7);
	dsi_dcs_write_seq(dsi, 0x06, 0x00);
	dsi_dcs_write_seq(dsi, 0x07, 0xf6);
	dsi_dcs_write_seq(dsi, 0x08, 0x01);
	dsi_dcs_write_seq(dsi, 0x09, 0x1d);
	dsi_dcs_write_seq(dsi, 0x0a, 0x01);
	dsi_dcs_write_seq(dsi, 0x0b, 0x54);
	dsi_dcs_write_seq(dsi, 0x0c, 0x01);
	dsi_dcs_write_seq(dsi, 0x0d, 0x81);
	dsi_dcs_write_seq(dsi, 0x0e, 0x01);
	dsi_dcs_write_seq(dsi, 0x0f, 0xcb);
	dsi_dcs_write_seq(dsi, 0x10, 0x02);
	dsi_dcs_write_seq(dsi, 0x11, 0x05);
	dsi_dcs_write_seq(dsi, 0x12, 0x02);
	dsi_dcs_write_seq(dsi, 0x13, 0x07);
	dsi_dcs_write_seq(dsi, 0x14, 0x02);
	dsi_dcs_write_seq(dsi, 0x15, 0x47);
	dsi_dcs_write_seq(dsi, 0x16, 0x02);
	dsi_dcs_write_seq(dsi, 0x17, 0x82);
	dsi_dcs_write_seq(dsi, 0x18, 0x02);
	dsi_dcs_write_seq(dsi, 0x19, 0xab);
	dsi_dcs_write_seq(dsi, 0x1a, 0x02);
	dsi_dcs_write_seq(dsi, 0x1b, 0xdc);
	dsi_dcs_write_seq(dsi, 0x1c, 0x03);
	dsi_dcs_write_seq(dsi, 0x1d, 0x01);
	dsi_dcs_write_seq(dsi, 0x1e, 0x03);
	dsi_dcs_write_seq(dsi, 0x1f, 0x3a);
	dsi_dcs_write_seq(dsi, 0x20, 0x03);
	dsi_dcs_write_seq(dsi, 0x21, 0x56);
	dsi_dcs_write_seq(dsi, 0x22, 0x03);
	dsi_dcs_write_seq(dsi, 0x23, 0x6d);
	dsi_dcs_write_seq(dsi, 0x24, 0x03);
	dsi_dcs_write_seq(dsi, 0x25, 0x89);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_GAMMA_CURVE, 0x03);
	dsi_dcs_write_seq(dsi, 0x27, 0xa3);
	dsi_dcs_write_seq(dsi, 0x28, 0x03);
	dsi_dcs_write_seq(dsi, 0x29, 0xc9);
	dsi_dcs_write_seq(dsi, 0x2a, 0x03);
	dsi_dcs_write_seq(dsi, 0x2b, 0xdd);
	dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_LUT, 0x03);
	dsi_dcs_write_seq(dsi, 0x2f, 0xf5);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_PARTIAL_ROWS, 0x03);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_PARTIAL_COLUMNS, 0xff);
	dsi_dcs_write_seq(dsi, 0x32, 0x00);
	dsi_dcs_write_seq(dsi, 0x33, 0x00);
	dsi_dcs_write_seq(dsi, 0x34, 0x00);
	dsi_dcs_write_seq(dsi, 0x35, 0x22);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_ADDRESS_MODE, 0x00);
	dsi_dcs_write_seq(dsi, 0x37, 0x46);
	dsi_dcs_write_seq(dsi, 0x38, 0x00);
	dsi_dcs_write_seq(dsi, 0x39, 0x5c);

	ret = mipi_dsi_dcs_set_pixel_format(dsi, 0x00);
	if (ret < 0) {
		dev_err(dev, "Failed to set pixel format: %d\n", ret);
		return ret;
	}

	dsi_dcs_write_seq(dsi, 0x3b, 0x76);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_3D_CONTROL, 0x00);
	dsi_dcs_write_seq(dsi, 0x3f, 0x8d);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_VSYNC_TIMING, 0x00);
	dsi_dcs_write_seq(dsi, 0x41, 0xa6);
	dsi_dcs_write_seq(dsi, 0x42, 0x00);
	dsi_dcs_write_seq(dsi, 0x43, 0xb8);
	dsi_dcs_write_seq(dsi, 0x44, 0x00);
	dsi_dcs_write_seq(dsi, MIPI_DCS_GET_SCANLINE, 0xc7);
	dsi_dcs_write_seq(dsi, 0x46, 0x00);
	dsi_dcs_write_seq(dsi, 0x47, 0xf6);
	dsi_dcs_write_seq(dsi, 0x48, 0x01);
	dsi_dcs_write_seq(dsi, 0x49, 0x1d);
	dsi_dcs_write_seq(dsi, 0x4a, 0x01);
	dsi_dcs_write_seq(dsi, 0x4b, 0x54);
	dsi_dcs_write_seq(dsi, 0x4c, 0x01);
	dsi_dcs_write_seq(dsi, 0x4d, 0x81);
	dsi_dcs_write_seq(dsi, 0x4e, 0x01);
	dsi_dcs_write_seq(dsi, 0x4f, 0xcb);
	dsi_dcs_write_seq(dsi, 0x50, 0x02);

	ret = mipi_dsi_dcs_set_display_brightness(dsi, 0x0005);
	if (ret < 0) {
		dev_err(dev, "Failed to set display brightness: %d\n", ret);
		return ret;
	}

	dsi_dcs_write_seq(dsi, 0x52, 0x02);
	dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x07);
	dsi_dcs_write_seq(dsi, 0x54, 0x02);
	dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_POWER_SAVE, 0x47);
	dsi_dcs_write_seq(dsi, 0x56, 0x02);
	dsi_dcs_write_seq(dsi, 0x58, 0x82);
	dsi_dcs_write_seq(dsi, 0x59, 0x02);
	dsi_dcs_write_seq(dsi, 0x5a, 0xab);
	dsi_dcs_write_seq(dsi, 0x5b, 0x02);
	dsi_dcs_write_seq(dsi, 0x5c, 0xdc);
	dsi_dcs_write_seq(dsi, 0x5d, 0x03);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_CABC_MIN_BRIGHTNESS, 0x01);
	dsi_dcs_write_seq(dsi, 0x5f, 0x03);
	dsi_dcs_write_seq(dsi, 0x60, 0x3a);
	dsi_dcs_write_seq(dsi, 0x61, 0x03);
	dsi_dcs_write_seq(dsi, 0x62, 0x56);
	dsi_dcs_write_seq(dsi, 0x63, 0x03);
	dsi_dcs_write_seq(dsi, 0x64, 0x6d);
	dsi_dcs_write_seq(dsi, 0x65, 0x03);
	dsi_dcs_write_seq(dsi, 0x66, 0x89);
	dsi_dcs_write_seq(dsi, 0x67, 0x03);
	dsi_dcs_write_seq(dsi, 0x68, 0xa3);
	dsi_dcs_write_seq(dsi, 0x69, 0x03);
	dsi_dcs_write_seq(dsi, 0x6a, 0xc9);
	dsi_dcs_write_seq(dsi, 0x6b, 0x03);
	dsi_dcs_write_seq(dsi, 0x6c, 0xdd);
	dsi_dcs_write_seq(dsi, 0x6d, 0x03);
	dsi_dcs_write_seq(dsi, 0x6e, 0xf5);
	dsi_dcs_write_seq(dsi, 0x6f, 0x03);
	dsi_dcs_write_seq(dsi, 0x70, 0xff);
	dsi_dcs_write_seq(dsi, 0x71, 0x00);
	dsi_dcs_write_seq(dsi, 0x72, 0x00);
	dsi_dcs_write_seq(dsi, 0x73, 0x00);
	dsi_dcs_write_seq(dsi, 0x74, 0x22);
	dsi_dcs_write_seq(dsi, 0x75, 0x00);
	dsi_dcs_write_seq(dsi, 0x76, 0x46);
	dsi_dcs_write_seq(dsi, 0x77, 0x00);
	dsi_dcs_write_seq(dsi, 0x78, 0x5c);
	dsi_dcs_write_seq(dsi, 0x79, 0x00);
	dsi_dcs_write_seq(dsi, 0x7a, 0x76);
	dsi_dcs_write_seq(dsi, 0x7b, 0x00);
	dsi_dcs_write_seq(dsi, 0x7c, 0x8d);
	dsi_dcs_write_seq(dsi, 0x7d, 0x00);
	dsi_dcs_write_seq(dsi, 0x7e, 0xa6);
	dsi_dcs_write_seq(dsi, 0x7f, 0x00);
	dsi_dcs_write_seq(dsi, 0x80, 0xb8);
	dsi_dcs_write_seq(dsi, 0x81, 0x00);
	dsi_dcs_write_seq(dsi, 0x82, 0xc7);
	dsi_dcs_write_seq(dsi, 0x83, 0x00);
	dsi_dcs_write_seq(dsi, 0x84, 0xf6);
	dsi_dcs_write_seq(dsi, 0x85, 0x01);
	dsi_dcs_write_seq(dsi, 0x86, 0x1d);
	dsi_dcs_write_seq(dsi, 0x87, 0x01);
	dsi_dcs_write_seq(dsi, 0x88, 0x54);
	dsi_dcs_write_seq(dsi, 0x89, 0x01);
	dsi_dcs_write_seq(dsi, 0x8a, 0x81);
	dsi_dcs_write_seq(dsi, 0x8b, 0x01);
	dsi_dcs_write_seq(dsi, 0x8c, 0xcb);
	dsi_dcs_write_seq(dsi, 0x8d, 0x02);
	dsi_dcs_write_seq(dsi, 0x8e, 0x05);
	dsi_dcs_write_seq(dsi, 0x8f, 0x02);
	dsi_dcs_write_seq(dsi, 0x90, 0x07);
	dsi_dcs_write_seq(dsi, 0x91, 0x02);
	dsi_dcs_write_seq(dsi, 0x92, 0x47);
	dsi_dcs_write_seq(dsi, 0x93, 0x02);
	dsi_dcs_write_seq(dsi, 0x94, 0x82);
	dsi_dcs_write_seq(dsi, 0x95, 0x02);
	dsi_dcs_write_seq(dsi, 0x96, 0xab);
	dsi_dcs_write_seq(dsi, 0x97, 0x02);
	dsi_dcs_write_seq(dsi, 0x98, 0xdc);
	dsi_dcs_write_seq(dsi, 0x99, 0x03);
	dsi_dcs_write_seq(dsi, 0x9a, 0x01);
	dsi_dcs_write_seq(dsi, 0x9b, 0x03);
	dsi_dcs_write_seq(dsi, 0x9c, 0x3a);
	dsi_dcs_write_seq(dsi, 0x9d, 0x03);
	dsi_dcs_write_seq(dsi, 0x9e, 0x56);
	dsi_dcs_write_seq(dsi, 0x9f, 0x03);
	dsi_dcs_write_seq(dsi, 0xa0, 0x6d);
	dsi_dcs_write_seq(dsi, MIPI_DCS_READ_PPS_START, 0x03);
	dsi_dcs_write_seq(dsi, 0xa3, 0x89);
	dsi_dcs_write_seq(dsi, 0xa4, 0x03);
	dsi_dcs_write_seq(dsi, 0xa5, 0xa3);
	dsi_dcs_write_seq(dsi, 0xa6, 0x03);
	dsi_dcs_write_seq(dsi, 0xa7, 0xc9);
	dsi_dcs_write_seq(dsi, MIPI_DCS_READ_PPS_CONTINUE, 0x03);
	dsi_dcs_write_seq(dsi, 0xaa, 0xdd);
	dsi_dcs_write_seq(dsi, 0xab, 0x03);
	dsi_dcs_write_seq(dsi, 0xac, 0xf5);
	dsi_dcs_write_seq(dsi, 0xad, 0x03);
	dsi_dcs_write_seq(dsi, 0xae, 0xff);
	dsi_dcs_write_seq(dsi, 0xaf, 0x00);
	dsi_dcs_write_seq(dsi, 0xb0, 0x00);
	dsi_dcs_write_seq(dsi, 0xb1, 0x00);
	dsi_dcs_write_seq(dsi, 0xb2, 0x22);
	dsi_dcs_write_seq(dsi, 0xb3, 0x00);
	dsi_dcs_write_seq(dsi, 0xb4, 0x46);
	dsi_dcs_write_seq(dsi, 0xb5, 0x00);
	dsi_dcs_write_seq(dsi, 0xb6, 0x5c);
	dsi_dcs_write_seq(dsi, 0xb7, 0x00);
	dsi_dcs_write_seq(dsi, 0xb8, 0x76);
	dsi_dcs_write_seq(dsi, 0xb9, 0x00);
	dsi_dcs_write_seq(dsi, 0xba, 0x8d);
	dsi_dcs_write_seq(dsi, 0xbb, 0x00);
	dsi_dcs_write_seq(dsi, 0xbc, 0xa6);
	dsi_dcs_write_seq(dsi, 0xbd, 0x00);
	dsi_dcs_write_seq(dsi, 0xbe, 0xb8);
	dsi_dcs_write_seq(dsi, 0xbf, 0x00);
	dsi_dcs_write_seq(dsi, 0xc0, 0xc7);
	dsi_dcs_write_seq(dsi, 0xc1, 0x00);
	dsi_dcs_write_seq(dsi, 0xc2, 0xf6);
	dsi_dcs_write_seq(dsi, 0xc3, 0x01);
	dsi_dcs_write_seq(dsi, 0xc4, 0x1d);
	dsi_dcs_write_seq(dsi, 0xc5, 0x01);
	dsi_dcs_write_seq(dsi, 0xc6, 0x54);
	dsi_dcs_write_seq(dsi, 0xc7, 0x01);
	dsi_dcs_write_seq(dsi, 0xc8, 0x81);
	dsi_dcs_write_seq(dsi, 0xc9, 0x01);
	dsi_dcs_write_seq(dsi, 0xca, 0xcb);
	dsi_dcs_write_seq(dsi, 0xcb, 0x02);
	dsi_dcs_write_seq(dsi, 0xcc, 0x05);
	dsi_dcs_write_seq(dsi, 0xcd, 0x02);
	dsi_dcs_write_seq(dsi, 0xce, 0x07);
	dsi_dcs_write_seq(dsi, 0xcf, 0x02);
	dsi_dcs_write_seq(dsi, 0xd0, 0x47);
	dsi_dcs_write_seq(dsi, 0xd1, 0x02);
	dsi_dcs_write_seq(dsi, 0xd2, 0x82);
	dsi_dcs_write_seq(dsi, 0xd3, 0x02);
	dsi_dcs_write_seq(dsi, 0xd4, 0xab);
	dsi_dcs_write_seq(dsi, 0xd5, 0x02);
	dsi_dcs_write_seq(dsi, 0xd6, 0xdc);
	dsi_dcs_write_seq(dsi, 0xd7, 0x03);
	dsi_dcs_write_seq(dsi, 0xd8, 0x01);
	dsi_dcs_write_seq(dsi, 0xd9, 0x03);
	dsi_dcs_write_seq(dsi, 0xda, 0x3a);
	dsi_dcs_write_seq(dsi, 0xdb, 0x03);
	dsi_dcs_write_seq(dsi, 0xdc, 0x56);
	dsi_dcs_write_seq(dsi, 0xdd, 0x03);
	dsi_dcs_write_seq(dsi, 0xde, 0x6d);
	dsi_dcs_write_seq(dsi, 0xdf, 0x03);
	dsi_dcs_write_seq(dsi, 0xe0, 0x89);
	dsi_dcs_write_seq(dsi, 0xe1, 0x03);
	dsi_dcs_write_seq(dsi, 0xe2, 0xa3);
	dsi_dcs_write_seq(dsi, 0xe3, 0x03);
	dsi_dcs_write_seq(dsi, 0xe4, 0xc9);
	dsi_dcs_write_seq(dsi, 0xe5, 0x03);
	dsi_dcs_write_seq(dsi, 0xe6, 0xdd);
	dsi_dcs_write_seq(dsi, 0xe7, 0x03);
	dsi_dcs_write_seq(dsi, 0xe8, 0xf5);
	dsi_dcs_write_seq(dsi, 0xe9, 0x03);
	dsi_dcs_write_seq(dsi, 0xea, 0xff);
	dsi_dcs_write_seq(dsi, 0xff, 0x24);
	usleep_range(1000, 2000);
	dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	dsi_dcs_write_seq(dsi, 0x00, 0x0f);
	dsi_dcs_write_seq(dsi, 0x02, 0x00);
	dsi_dcs_write_seq(dsi, 0x03, 0x00);
	dsi_dcs_write_seq(dsi, 0x04, 0x0b);
	dsi_dcs_write_seq(dsi, 0x05, 0x0c);
	dsi_dcs_write_seq(dsi, 0x06, 0x00);
	dsi_dcs_write_seq(dsi, 0x07, 0x00);
	dsi_dcs_write_seq(dsi, 0x08, 0x00);
	dsi_dcs_write_seq(dsi, 0x09, 0x00);
	dsi_dcs_write_seq(dsi, 0x0a, 0x03);
	dsi_dcs_write_seq(dsi, 0x0b, 0x04);
	dsi_dcs_write_seq(dsi, 0x0c, 0x01);
	dsi_dcs_write_seq(dsi, 0x0d, 0x13);
	dsi_dcs_write_seq(dsi, 0x0e, 0x15);
	dsi_dcs_write_seq(dsi, 0x0f, 0x17);
	dsi_dcs_write_seq(dsi, 0x10, 0x0f);
	dsi_dcs_write_seq(dsi, 0x12, 0x00);
	dsi_dcs_write_seq(dsi, 0x13, 0x00);
	dsi_dcs_write_seq(dsi, 0x14, 0x0b);
	dsi_dcs_write_seq(dsi, 0x15, 0x0c);
	dsi_dcs_write_seq(dsi, 0x16, 0x00);
	dsi_dcs_write_seq(dsi, 0x17, 0x00);
	dsi_dcs_write_seq(dsi, 0x18, 0x00);
	dsi_dcs_write_seq(dsi, 0x19, 0x00);
	dsi_dcs_write_seq(dsi, 0x1a, 0x03);
	dsi_dcs_write_seq(dsi, 0x1b, 0x04);
	dsi_dcs_write_seq(dsi, 0x1c, 0x01);
	dsi_dcs_write_seq(dsi, 0x1d, 0x13);
	dsi_dcs_write_seq(dsi, 0x1e, 0x15);
	dsi_dcs_write_seq(dsi, 0x1f, 0x17);
	dsi_dcs_write_seq(dsi, 0x20, 0x09);
	dsi_dcs_write_seq(dsi, 0x21, 0x01);
	dsi_dcs_write_seq(dsi, 0x22, 0x00);
	dsi_dcs_write_seq(dsi, 0x23, 0x00);
	dsi_dcs_write_seq(dsi, 0x24, 0x00);
	dsi_dcs_write_seq(dsi, 0x25, 0x6d);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_GAMMA_CURVE, 0x00);
	dsi_dcs_write_seq(dsi, 0x27, 0x00);
	dsi_dcs_write_seq(dsi, 0x29, 0x58);
	dsi_dcs_write_seq(dsi, 0x2a, 0x16);
	dsi_dcs_write_seq(dsi, 0x2f, 0x02);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_PARTIAL_ROWS, 0x04);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_PARTIAL_COLUMNS, 0x49);
	dsi_dcs_write_seq(dsi, 0x32, 0x23);
	dsi_dcs_write_seq(dsi, 0x33, 0x01);
	dsi_dcs_write_seq(dsi, 0x34, 0x01);
	dsi_dcs_write_seq(dsi, 0x35, 0x6e);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_ADDRESS_MODE, 0x00);
	dsi_dcs_write_seq(dsi, 0x37, 0x2d);
	dsi_dcs_write_seq(dsi, 0x38, 0x08);
	dsi_dcs_write_seq(dsi, 0x39, 0x01);

	ret = mipi_dsi_dcs_set_pixel_format(dsi, 0x6e);
	if (ret < 0) {
		dev_err(dev, "Failed to set pixel format: %d\n", ret);
		return ret;
	}

	dsi_dcs_write_seq(dsi, 0x3b, 0x00);
	dsi_dcs_write_seq(dsi, 0x59, 0x02);
	dsi_dcs_write_seq(dsi, 0x5a, 0x08);
	dsi_dcs_write_seq(dsi, 0x5b, 0x00);
	dsi_dcs_write_seq(dsi, 0x5f, 0x75);
	dsi_dcs_write_seq(dsi, 0x63, 0x00);
	dsi_dcs_write_seq(dsi, 0x67, 0x04);
	dsi_dcs_write_seq(dsi, 0x5c, 0x05);
	dsi_dcs_write_seq(dsi, 0x60, 0x75);
	dsi_dcs_write_seq(dsi, 0x64, 0x01);
	dsi_dcs_write_seq(dsi, 0x68, 0x04);
	dsi_dcs_write_seq(dsi, 0x6e, 0x10);
	dsi_dcs_write_seq(dsi, 0x72, 0x00);
	dsi_dcs_write_seq(dsi, 0x73, 0x00);
	dsi_dcs_write_seq(dsi, 0x74, 0x11);
	dsi_dcs_write_seq(dsi, 0x75, 0x1a);
	dsi_dcs_write_seq(dsi, 0x76, 0x06);
	dsi_dcs_write_seq(dsi, 0x77, 0x03);
	dsi_dcs_write_seq(dsi, 0x78, 0x00);
	dsi_dcs_write_seq(dsi, 0x79, 0x00);
	dsi_dcs_write_seq(dsi, 0x7a, 0x00);
	dsi_dcs_write_seq(dsi, 0x7b, 0x80);
	dsi_dcs_write_seq(dsi, 0x7c, 0xd8);
	dsi_dcs_write_seq(dsi, 0x7d, 0x60);
	dsi_dcs_write_seq(dsi, 0x7e, 0x11);
	dsi_dcs_write_seq(dsi, 0x7f, 0x1a);
	dsi_dcs_write_seq(dsi, 0x80, 0x00);
	dsi_dcs_write_seq(dsi, 0x81, 0x06);
	dsi_dcs_write_seq(dsi, 0x82, 0x03);
	dsi_dcs_write_seq(dsi, 0x83, 0x00);
	dsi_dcs_write_seq(dsi, 0x84, 0x03);
	dsi_dcs_write_seq(dsi, 0x85, 0x07);
	dsi_dcs_write_seq(dsi, 0x86, 0x1b);
	dsi_dcs_write_seq(dsi, 0x87, 0x39);
	dsi_dcs_write_seq(dsi, 0x88, 0x1b);
	dsi_dcs_write_seq(dsi, 0x89, 0x39);
	dsi_dcs_write_seq(dsi, 0x8a, 0x33);
	dsi_dcs_write_seq(dsi, 0x8b, 0xf4);
	dsi_dcs_write_seq(dsi, 0x8c, 0x01);
	dsi_dcs_write_seq(dsi, 0x8e, 0x02);
	dsi_dcs_write_seq(dsi, 0x90, 0x95);
	dsi_dcs_write_seq(dsi, 0x91, 0xc8);
	dsi_dcs_write_seq(dsi, 0x92, 0x95);
	dsi_dcs_write_seq(dsi, 0x93, 0x02);
	dsi_dcs_write_seq(dsi, 0x94, 0x08);
	dsi_dcs_write_seq(dsi, 0x95, 0x2b);
	dsi_dcs_write_seq(dsi, 0x96, 0x95);
	dsi_dcs_write_seq(dsi, 0x98, 0x00);
	dsi_dcs_write_seq(dsi, 0x99, 0x33);
	dsi_dcs_write_seq(dsi, 0x9a, 0x03);
	dsi_dcs_write_seq(dsi, 0x9b, 0x0f);
	dsi_dcs_write_seq(dsi, 0x9c, 0x01);
	dsi_dcs_write_seq(dsi, 0x9d, 0xb0);
	dsi_dcs_write_seq(dsi, 0xa0, 0x33);
	dsi_dcs_write_seq(dsi, 0xa6, 0x03);
	dsi_dcs_write_seq(dsi, 0xde, 0xff);
	dsi_dcs_write_seq(dsi, 0xb3, 0x28);
	dsi_dcs_write_seq(dsi, 0xb4, 0x05);
	dsi_dcs_write_seq(dsi, 0xb5, 0x10);
	dsi_dcs_write_seq(dsi, 0xc2, 0x80);
	dsi_dcs_write_seq(dsi, 0xc3, 0x10);
	dsi_dcs_write_seq(dsi, 0xc4, 0x24);
	dsi_dcs_write_seq(dsi, 0xc5, 0x30);
	dsi_dcs_write_seq(dsi, 0xc6, 0x00);
	dsi_dcs_write_seq(dsi, 0xd1, 0x00);
	dsi_dcs_write_seq(dsi, 0xd2, 0x00);
	dsi_dcs_write_seq(dsi, 0xff, 0x10);
	usleep_range(1000, 2000);
	dsi_dcs_write_seq(dsi, 0x3b, 0x03, 0x08, 0x02, 0x04, 0x04);
	dsi_dcs_write_seq(dsi, 0xbb, 0x03);
	dsi_dcs_write_seq(dsi, 0xfb, 0x01);

	ret = mipi_dsi_dcs_set_display_brightness(dsi, 0x00ff);
	if (ret < 0) {
		dev_err(dev, "Failed to set display brightness: %d\n", ret);
		return ret;
	}

	dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x2c);
	dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_POWER_SAVE, 0x01);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_CABC_MIN_BRIGHTNESS, 0x06);

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear on: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	msleep(120);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}

	return 0;
}

static int truly_nt35695_off(struct truly_nt35695 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}
	msleep(50);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}
	msleep(120);

	return 0;
}

static int truly_nt35695_prepare(struct drm_panel *panel)
{
	struct truly_nt35695 *ctx = to_truly_nt35695(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (ctx->prepared)
		return 0;

	ret = regulator_enable(ctx->supply);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulator: %d\n", ret);
		return ret;
	}

	truly_nt35695_reset(ctx);

	ret = truly_nt35695_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_disable(ctx->supply);
		return ret;
	}

	ctx->prepared = true;
	return 0;
}

static int truly_nt35695_unprepare(struct drm_panel *panel)
{
	struct truly_nt35695 *ctx = to_truly_nt35695(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = truly_nt35695_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->supply);

	ctx->prepared = false;
	return 0;
}

static const struct drm_display_mode truly_nt35695_mode = {
	.clock = (1080 + 90 + 20 + 60) * (1920 + 9 + 2 + 2) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 90,
	.hsync_end = 1080 + 90 + 20,
	.htotal = 1080 + 90 + 20 + 60,
	.vdisplay = 1920,
	.vsync_start = 1920 + 9,
	.vsync_end = 1920 + 9 + 2,
	.vtotal = 1920 + 9 + 2 + 2,
	.width_mm = 62,
	.height_mm = 110,
};

static int truly_nt35695_get_modes(struct drm_panel *panel,
				   struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &truly_nt35695_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs truly_nt35695_panel_funcs = {
	.prepare = truly_nt35695_prepare,
	.unprepare = truly_nt35695_unprepare,
	.get_modes = truly_nt35695_get_modes,
};

static int truly_nt35695_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct truly_nt35695 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(ctx->supply))
		return dev_err_probe(dev, PTR_ERR(ctx->supply),
				     "Failed to get power regulator\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;

	drm_panel_init(&ctx->panel, dev, &truly_nt35695_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static int truly_nt35695_remove(struct mipi_dsi_device *dsi)
{
	struct truly_nt35695 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id truly_nt35695_of_match[] = {
	{ .compatible = "truly,nt35695" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, truly_nt35695_of_match);

static struct mipi_dsi_driver truly_nt35695_driver = {
	.probe = truly_nt35695_probe,
	.remove = truly_nt35695_remove,
	.driver = {
		.name = "panel-truly-nt35695",
		.of_match_table = truly_nt35695_of_match,
	},
};
module_mipi_dsi_driver(truly_nt35695_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for TRULY NT35695 1080P video mode dsi panel");
MODULE_LICENSE("GPL v2");
