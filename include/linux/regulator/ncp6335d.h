/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved. */

#ifndef __NCP6335D_H__
#define __NCP6335D_H__

enum {
	NCP6335D_VSEL0,
	NCP6335D_VSEL1,
};

struct ncp6335d_platform_data {
	struct regulator_init_data *init_data;
	int default_vsel;
	int slew_rate_ns;
	int discharge_enable;
	bool sleep_enable;
};

#endif
