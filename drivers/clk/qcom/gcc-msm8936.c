// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018, 2020, Pavel Dubrova <pashadubrova@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>

#include <dt-bindings/clock/qcom,gcc-msm8936.h>
#include <dt-bindings/reset/qcom,gcc-msm8936.h>

#include "common.h"
#include "clk-regmap.h"
#include "clk-pll.h"
#include "clk-rcg.h"
#include "clk-branch.h"
#include "reset.h"
#include "gdsc.h"

#define F_APCS_PLL(f, l, m, n) { (f), (l), (m), (n), 0 }

enum {
	P_GPLL0_OUT_MAIN,
	P_GPLL0_OUT_AUX,
	P_GPLL0_MISC,
	P_GPLL1_OUT_MAIN,
	P_GPLL2_OUT_MAIN,
	P_GPLL2_OUT_AUX,
	P_GPLL2_GFX3D,
	P_GPLL3_OUT_MAIN,
	P_GPLL3_OUT_AUX,
	P_GPLL4_OUT_MAIN,
	P_GPLL6_OUT_MAIN,
	P_GPLL6_MCLK,
	P_DSI0_PHYPLL_BYTE,
	P_DSI0_PHYPLL_DSI,
	P_XO_A,
	P_XO,
};

static const struct parent_map gcc_gpll0_map[] = {
	{ P_GPLL0_OUT_MAIN, 1 },
};

static const char * const gcc_gpll0[] = {
	"gpll0_out_main",
};

static const struct parent_map gcc_gpll0m_map[] = {
	{ P_GPLL0_MISC, 2 },
};

static const char * const gcc_gpll0m[] = {
	"gpll0_out_main",
};

static const struct parent_map gcc_gpll0_gpll2_map[] = {
	{ P_GPLL0_OUT_MAIN, 1 },
	{ P_GPLL2_OUT_MAIN, 2 },
};

static const char * const gcc_gpll0_gpll2[] = {
	"gpll0_out_main",
	"gpll2_out_main",
};

static const struct parent_map gcc_gpll0a_gpll1_gpll3a_map[] = {
	{ P_GPLL0_OUT_AUX, 5 },
	{ P_GPLL1_OUT_MAIN, 1 },
	{ P_GPLL3_OUT_AUX, 4 },
};

static const char * const gcc_gpll0a_gpll1_gpll3a[] = {
	"gpll0_out_main",
	"gpll1_out_main",
	"gpll3_out_main",
};

static const struct parent_map gcc_gpll0_gpll2a_gpll4_map[] = {
	{ P_GPLL0_OUT_MAIN, 1 },
	{ P_GPLL2_OUT_AUX, 3 },
	{ P_GPLL4_OUT_MAIN, 2 },
};

static const char * const gcc_gpll0_gpll2a_gpll4[] = {
	"gpll0_out_main",
	"gpll2_out_main",
	"gpll4_out_main",
};

static const struct parent_map gcc_gpll0_gpll6m_map[] = {
	{ P_GPLL0_OUT_MAIN, 1 },
	{ P_GPLL6_MCLK, 3 },
};

static const char * const gcc_gpll0_gpll6m[] = {
	"gpll0_out_main",
	"gpll6_out_main",
};

static const struct parent_map gcc_xo_map[] = {
	{ P_XO, 0 },
};

static const char * const gcc_xo[] = {
	"xo",
};

static const struct parent_map gcc_gpll6_map[] = {
	{ P_GPLL6_OUT_MAIN, 1 },
};

static const char * const gcc_gpll6[] = {
	"gpll6_out_main",
};

static const struct parent_map gcc_xo_gpll0_map[] = {
	{ P_XO_A, 0 },
	{ P_GPLL0_OUT_MAIN, 1 },
};

static const char * const gcc_xo_gpll0[] = {
	"xo_a",
	"gpll0_out_main",
};

static const struct parent_map gcc_xo_gpll0m_map[] = {
	{ P_XO, 0 },
	{ P_GPLL0_MISC, 2 },
};

static const char * const gcc_xo_gpll0m[] = {
	"xo",
	"gpll0_out_main",
};

static const struct parent_map gcc_xo_gpll0_gpll2g_gpll3_map[] = {
	{ P_XO, 0 },
	{ P_GPLL0_OUT_MAIN, 1 },
	{ P_GPLL2_GFX3D, 4 },
	{ P_GPLL3_OUT_MAIN, 2 },
};

static const char * const gcc_xo_gpll0_gpll2g_gpll3[] = {
	"xo",
	"gpll0_out_main",
	"gpll2_out_main",
	"gpll3_out_main",
};

static struct clk_fixed_factor xo = {
	.mult = 1,
	.div = 1,
	.hw.init = &(struct clk_init_data){
		.name = "xo",
		.parent_names = (const char *[]){ "cxo" },
		.num_parents = 1,
		.ops = &clk_fixed_factor_ops,
	},
};

static struct clk_fixed_factor xo_a = {
	.mult = 1,
	.div = 1,
	.hw.init = &(struct clk_init_data){
		.name = "xo_a",
		.parent_names = (const char *[]){ "cxo_a" },
		.num_parents = 1,
		.ops = &clk_fixed_factor_ops,
	},
};

static struct clk_fixed_factor wcnss_m_clk = {
	.hw.init = &(struct clk_init_data){
		.name = "wcnss_m_clk",
		.ops = &clk_fixed_factor_ops,
	},
};

static struct clk_pll gpll0 = {
	.l_reg = 0x21004,
	.m_reg = 0x21008,
	.n_reg = 0x2100c,
	.config_reg = 0x21014,
	.mode_reg = 0x21000,
	.status_reg = 0x2101c,
	.status_bit = 17,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "gpll0",
		.parent_names = (const char *[]){ "xo" },
		.num_parents = 1,
		.ops = &clk_pll_ops,
	},
};

static struct clk_regmap gpll0_out_main = {
	.enable_reg = 0x45000,
	.enable_mask = BIT(0),
	.hw.init = &(struct clk_init_data){
		.name = "gpll0_out_main",
		.parent_names = (const char *[]){ "gpll0" },
		.num_parents = 1,
		.ops = &clk_pll_vote_ops,
	},
};

static struct clk_regmap gpll0_ao = {
	.enable_reg = 0x45000,
	.enable_mask = BIT(0),
	.hw.init = &(struct clk_init_data){
		.name = "gpll0_ao",
		.parent_names = (const char *[]){ "gpll0" },
		.num_parents = 1,
		.ops = &clk_pll_vote_ops,
	},
};

static struct clk_pll gpll1 = {
	.l_reg = 0x20004,
	.m_reg = 0x20008,
	.n_reg = 0x2000c,
	.config_reg = 0x20014,
	.mode_reg = 0x20000,
	.status_reg = 0x2001c,
	.status_bit = 17,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "gpll1",
		.parent_names = (const char *[]){ "xo" },
		.num_parents = 1,
		.ops = &clk_pll_ops,
	},
};

static struct clk_regmap gpll1_out_main = {
	.enable_reg = 0x45000,
	.enable_mask = BIT(1),
	.hw.init = &(struct clk_init_data){
		.name = "gpll1_out_main",
		.parent_names = (const char *[]){ "gpll1" },
		.num_parents = 1,
		.ops = &clk_pll_vote_ops,
	},
};

static struct clk_pll gpll2 = {
	.l_reg = 0x4a004,
	.m_reg = 0x4a008,
	.n_reg = 0x4a00c,
	.config_reg = 0x4a014,
	.mode_reg = 0x4a000,
	.status_reg = 0x4a01c,
	.status_bit = 17,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "gpll2",
		.parent_names = (const char *[]){ "xo" },
		.num_parents = 1,
		.ops = &clk_pll_ops,
	},
};

static struct clk_regmap gpll2_out_main = {
	.enable_reg = 0x45000,
	.enable_mask = BIT(2),
	.hw.init = &(struct clk_init_data){
		.name = "gpll2_out_main",
		.parent_names = (const char *[]){ "gpll2" },
		.num_parents = 1,
		.ops = &clk_pll_vote_ops,
	},
};

#define F_GPLL(f, l, m, n) { (f), (l), (m), (n), 0 }

static struct pll_freq_tbl ftbl_gcc_gpll3[] = {
	F_GPLL(1100000000, 57, 7, 24),
};

static struct clk_pll gpll3 = {
	.l_reg		= 0x22004,
	.m_reg		= 0x22008,
	.n_reg		= 0x2200c,
	.config_reg	= 0x22014,
	.mode_reg	= 0x22000,
	.status_reg	= 0x2201c,
	.status_bit	= 17,
	.freq_tbl	= ftbl_gcc_gpll3,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "gpll3",
		.parent_names = (const char *[]) { "xo" },
		.num_parents = 1,
		.ops = &clk_pll_ops,
	},
};

static struct clk_regmap gpll3_out_main = {
	.enable_reg = 0x45000,
	.enable_mask = BIT(4),
	.hw.init = &(struct clk_init_data){
		.name = "gpll3_out_main",
		.parent_names = (const char *[]){ "gpll3" },
		.num_parents = 1,
		.ops = &clk_pll_vote_ops,
	},
};

/* GPLL3 at 1100 MHz, main output enabled. */
static struct pll_config gpll3_config = {
	.l = 57,
	.m = 7,
	.n = 24,
	.vco_val = 0x0,
	.vco_mask = BIT(20),
	.pre_div_val = 0x0,
	.pre_div_mask = BIT(12),
	.post_div_val = 0x0,
	.post_div_mask = 0x3 << 8,
	.mn_ena_mask = BIT(24),
	.main_output_mask = BIT(0),
	.aux_output_mask = BIT(1),
};

static struct clk_pll gpll4 = {
	.l_reg = 0x24004,
	.m_reg = 0x24008,
	.n_reg = 0x2400c,
	.config_reg = 0x24014,
	.mode_reg = 0x24000,
	.status_reg = 0x2401c,
	.status_bit = 17,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "gpll4",
		.parent_names = (const char *[]){ "xo" },
		.num_parents = 1,
		.ops = &clk_pll_ops,
	},
};

static struct clk_regmap gpll4_out_main = {
	.enable_reg = 0x45000,
	.enable_mask = BIT(5),
	.hw.init = &(struct clk_init_data){
		.name = "gpll4_out_main",
		.parent_names = (const char *[]){ "gpll4" },
		.num_parents = 1,
		.ops = &clk_pll_vote_ops,
	},
};

/* GPLL4 at 1200 MHz, main output enabled. */
static struct pll_config gpll4_config = {
	.l = 62,
	.m = 1,
	.n = 2,
	.vco_val = 0x0,
	.vco_mask = BIT(20),
	.pre_div_val = 0x0,
	.pre_div_mask = BIT(12),
	.post_div_val = 0x0,
	.post_div_mask = 0x3 << 8,
	.mn_ena_mask = BIT(24),
	.main_output_mask = BIT(0),
};

static struct clk_pll gpll6 = {
	.mode_reg = 0x37000,
	.l_reg = 0x37004,
	.m_reg = 0x37008,
	.n_reg = 0x3700c,
	.config_reg = 0x37014,
	.mode_reg = 0x37000,
	.status_reg = 0x3701c,
	.status_bit = 17,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "gpll6",
		.parent_names = (const char *[]){ "xo" },
		.num_parents = 1,
		.ops = &clk_pll_ops,
	},
};

static struct clk_regmap gpll6_out_main = {
	.enable_reg = 0x45000,
	.enable_mask = BIT(7),
	.hw.init = &(struct clk_init_data){
		.name = "gpll6_out_main",
		.parent_names = (const char *[]){ "gpll6" },
		.num_parents = 1,
		.ops = &clk_pll_vote_ops,
	},
};

static const struct pll_freq_tbl apcs_c0_pll_freq[] = {
	F_APCS_PLL(998400000,	52, 0x0, 0x1),
	F_APCS_PLL(1113600000,	58, 0x0, 0x1),
	F_APCS_PLL(1209600000,	63, 0x0, 0x1),
};

static struct clk_pll a53ss_c0_pll = {
	.l_reg = 0x00004,
	.m_reg = 0x00008,
	.n_reg = 0x0000c,
	.config_reg = 0x00010,
	.mode_reg = 0x00000,
	.status_reg = 0x0001c,
	.status_bit = 17,
	.freq_tbl = apcs_c0_pll_freq,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "a53ss_c0_pll",
		.parent_names = (const char *[]){ "xo_a" },
		.num_parents = 1,
		.ops = &clk_pll_sr2_ops,
	},
};

static const struct pll_freq_tbl apcs_c1_pll_freq[] = {
	F_APCS_PLL(652800000,	34, 0x0, 0x1),
	F_APCS_PLL(691200000,	36, 0x0, 0x1),
	F_APCS_PLL(729600000,	38, 0x0, 0x1),
	F_APCS_PLL(806400000,	42, 0x0, 0x1),
	F_APCS_PLL(844800000,	44, 0x0, 0x1),
	F_APCS_PLL(883200000,	46, 0x0, 0x1),
	F_APCS_PLL(960000000,	50, 0x0, 0x1),
	F_APCS_PLL(998400000,	52, 0x0, 0x1),
	F_APCS_PLL(1036800000,	54, 0x0, 0x1),
	F_APCS_PLL(1113600000,	58, 0x0, 0x1),
	F_APCS_PLL(1209600000,	63, 0x0, 0x1),
	F_APCS_PLL(1190400000,	62, 0x0, 0x1),
	F_APCS_PLL(1267200000,	66, 0x0, 0x1),
	F_APCS_PLL(1344000000,	70, 0x0, 0x1),
	F_APCS_PLL(1363200000,	71, 0x0, 0x1),
	F_APCS_PLL(1420800000,	74, 0x0, 0x1),
	F_APCS_PLL(1459200000,	76, 0x0, 0x1),
	F_APCS_PLL(1497600000,	78, 0x0, 0x1),
	F_APCS_PLL(1536000000,	80, 0x0, 0x1),
	F_APCS_PLL(1574400000,	82, 0x0, 0x1),
	F_APCS_PLL(1612800000,	84, 0x0, 0x1),
	F_APCS_PLL(1632000000,	85, 0x0, 0x1),
	F_APCS_PLL(1651200000,	86, 0x0, 0x1),
	F_APCS_PLL(1689600000,	88, 0x0, 0x1),
	F_APCS_PLL(1708800000,	89, 0x0, 0x1),
};

static struct clk_pll a53ss_c1_pll = {
	.l_reg = 0x00004,
	.m_reg = 0x00008,
	.n_reg = 0x0000c,
	.config_reg = 0x00010,
	.mode_reg = 0x00000,
	.status_reg = 0x0001c,
	.status_bit = 17,
	.freq_tbl = apcs_c1_pll_freq,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "a53ss_c1_pll",
		.parent_names = (const char *[]){ "xo_a" },
		.num_parents = 1,
		.ops = &clk_pll_sr2_ops,
	},
};

static const struct pll_freq_tbl apcs_cci_pll_freq[] = {
	F_APCS_PLL(403200000, 21, 0x0, 0x1),
	F_APCS_PLL(595200000, 31, 0x0, 0x1),
};

static struct clk_pll a53ss_cci_pll = {
	.l_reg = 0x00004,
	.m_reg = 0x00008,
	.n_reg = 0x0000c,
	.config_reg = 0x00010,
	.mode_reg = 0x00000,
	.status_reg = 0x0001c,
	.status_bit = 17,
	.freq_tbl = apcs_cci_pll_freq,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "a53ss_cci_pll",
		.parent_names = (const char *[]){ "xo_a" },
		.num_parents = 1,
		.ops = &clk_pll_sr2_ops,
	},
};

static const struct freq_tbl ftbl_apss_ahb_clk[] = {
	F(19200000,	P_XO_A,			1, 0, 0),
	F(50000000,	P_GPLL0_OUT_MAIN,	16, 0, 0),
	F(100000000,	P_GPLL0_OUT_MAIN,	8, 0, 0),
	F(133330000,	P_GPLL0_OUT_MAIN,	6, 0, 0),
	{ }
};

static struct clk_rcg2 apss_ahb_clk_src = {
	.cmd_rcgr = 0x46000,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0_map,
	.freq_tbl = ftbl_apss_ahb_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "apss_ahb_clk_src",
		.parent_names = gcc_xo_gpll0,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_camss_csi0_1_2_clk[] = {
	F(100000000, P_GPLL0_OUT_MAIN, 8, 0, 0),
	F(200000000, P_GPLL0_OUT_MAIN, 4, 0, 0),
	{ }
};

static struct clk_rcg2 csi0_clk_src = {
	.cmd_rcgr = 0x4e020,
	.hid_width = 5,
	.parent_map = gcc_gpll0_map,
	.freq_tbl = ftbl_gcc_camss_csi0_1_2_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "csi0_clk_src",
		.parent_names = gcc_gpll0,
		.num_parents = ARRAY_SIZE(gcc_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_rcg2 csi1_clk_src = {
	.cmd_rcgr = 0x4f020,
	.hid_width = 5,
	.parent_map = gcc_gpll0_map,
	.freq_tbl = ftbl_gcc_camss_csi0_1_2_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "csi1_clk_src",
		.parent_names = gcc_gpll0,
		.num_parents = ARRAY_SIZE(gcc_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_rcg2 csi2_clk_src = {
	.cmd_rcgr = 0x3c020,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0_map,
	.freq_tbl = ftbl_gcc_camss_csi0_1_2_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "csi2_clk_src",
		.parent_names = gcc_gpll0,
		.num_parents = ARRAY_SIZE(gcc_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_camss_vfe0_clk[] = {
	F(50000000,	P_GPLL0_OUT_MAIN,	16,	0, 0),
	F(80000000,	P_GPLL0_OUT_MAIN,	10,	0, 0),
	F(100000000,	P_GPLL0_OUT_MAIN,	8,	0, 0),
	F(160000000,	P_GPLL0_OUT_MAIN,	5,	0, 0),
	F(177780000,	P_GPLL0_OUT_MAIN,	4.5,	0, 0),
	F(200000000,	P_GPLL0_OUT_MAIN,	4,	0, 0),
	F(266670000,	P_GPLL0_OUT_MAIN,	3,	0, 0),
	F(320000000,	P_GPLL0_OUT_MAIN,	2.5,	0, 0),
	F(400000000,	P_GPLL0_OUT_MAIN,	2,	0, 0),
	F(465000000,	P_GPLL2_OUT_AUX,	2,	0, 0),
	F(480000000,	P_GPLL4_OUT_MAIN,	2.5,	0, 0),
	F(600000000,	P_GPLL4_OUT_MAIN,	2,	0, 0),
	{ }
};

static struct clk_rcg2 vfe0_clk_src = {
	.cmd_rcgr = 0x58000,
	.hid_width = 5,
	.parent_map = gcc_gpll0_gpll2a_gpll4_map,
	.freq_tbl = ftbl_gcc_camss_vfe0_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "vfe0_clk_src",
		.parent_names = gcc_gpll0_gpll2a_gpll4,
		.num_parents = ARRAY_SIZE(gcc_gpll0_gpll2a_gpll4),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_mdss_mdp_clk[] = {
	F(50000000,	P_GPLL0_OUT_AUX,	16,	0, 0),
	F(80000000,	P_GPLL0_OUT_AUX,	10,	0, 0),
	F(100000000,	P_GPLL0_OUT_AUX,	8,	0, 0),
	F(145500000,	P_GPLL0_OUT_AUX,	5.5,	0, 0),
	F(153600000,	P_GPLL1_OUT_MAIN,	4,	0, 0),
	F(160000000,	P_GPLL0_OUT_AUX,	5,	0, 0),
	F(177780000,	P_GPLL0_OUT_AUX,	4.5,	0, 0),
	F(200000000,	P_GPLL0_OUT_AUX,	4,	0, 0),
	F(266670000,	P_GPLL0_OUT_AUX,	3,	0, 0),
	F(307200000,	P_GPLL1_OUT_MAIN,	2,	0, 0),
	F(366670000,	P_GPLL3_OUT_AUX,	3,	0, 0),
	{ }
};

static struct clk_rcg2 mdp_clk_src = {
	.cmd_rcgr = 0x4d014,
	.hid_width = 5,
	.parent_map = gcc_gpll0a_gpll1_gpll3a_map,
	.freq_tbl = ftbl_gcc_mdss_mdp_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "mdp_clk_src",
		.parent_names = gcc_gpll0a_gpll1_gpll3a,
		.num_parents = ARRAY_SIZE(gcc_gpll0a_gpll1_gpll3a_map),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_oxili_gfx3d_clk[] = {
	F(19200000,	P_XO,			1,	0, 0),
	F(50000000,	P_GPLL0_OUT_MAIN,	16,	0, 0),
	F(80000000,	P_GPLL0_OUT_MAIN,	10,	0, 0),
	F(100000000,	P_GPLL0_OUT_MAIN,	8,	0, 0),
	F(160000000,	P_GPLL0_OUT_MAIN,	5,	0, 0),
	F(200000000,	P_GPLL0_OUT_MAIN,	4,	0, 0),
	F(220000000,	P_GPLL3_OUT_MAIN,	5,	0, 0),
	F(266670000,	P_GPLL0_OUT_MAIN,	3,	0, 0),
	F(310000000,	P_GPLL2_GFX3D,		3,	0, 0),
	F(400000000,	P_GPLL0_OUT_MAIN,	2,	0, 0),
	F(465000000,	P_GPLL2_GFX3D,		2,	0, 0),
	F(550000000,	P_GPLL3_OUT_MAIN,	2,	0, 0),
	{ }
};

static struct clk_rcg2 gfx3d_clk_src = {
	.cmd_rcgr = 0x59000,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0_gpll2g_gpll3_map,
	.freq_tbl = ftbl_gcc_oxili_gfx3d_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "gfx3d_clk_src",
		.parent_names = gcc_xo_gpll0_gpll2g_gpll3,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_gpll2g_gpll3),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_blsp1_qup1_6_i2c_apps_clk[] = {
	F(19200000, P_XO,		1,	0, 0),
	F(50000000, P_GPLL0_OUT_MAIN,	16,	0, 0),
	{ }
};

static struct clk_rcg2 blsp1_qup1_i2c_apps_clk_src = {
	.cmd_rcgr = 0x0200c,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0_map,
	.freq_tbl = ftbl_gcc_blsp1_qup1_6_i2c_apps_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "blsp1_qup1_i2c_apps_clk_src",
		.parent_names = gcc_xo_gpll0,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_rcg2 blsp1_qup2_i2c_apps_clk_src = {
	.cmd_rcgr = 0x03000,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0_map,
	.freq_tbl = ftbl_gcc_blsp1_qup1_6_i2c_apps_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "blsp1_qup2_i2c_apps_clk_src",
		.parent_names = gcc_xo_gpll0,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_rcg2 blsp1_qup3_i2c_apps_clk_src = {
	.cmd_rcgr = 0x04000,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0_map,
	.freq_tbl = ftbl_gcc_blsp1_qup1_6_i2c_apps_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "blsp1_qup3_i2c_apps_clk_src",
		.parent_names = gcc_xo_gpll0,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_rcg2 blsp1_qup4_i2c_apps_clk_src = {
	.cmd_rcgr = 0x05000,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0_map,
	.freq_tbl = ftbl_gcc_blsp1_qup1_6_i2c_apps_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "blsp1_qup4_i2c_apps_clk_src",
		.parent_names = gcc_xo_gpll0,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_rcg2 blsp1_qup5_i2c_apps_clk_src = {
	.cmd_rcgr = 0x06000,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0_map,
	.freq_tbl = ftbl_gcc_blsp1_qup1_6_i2c_apps_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "blsp1_qup5_i2c_apps_clk_src",
		.parent_names = gcc_xo_gpll0,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_rcg2 blsp1_qup6_i2c_apps_clk_src = {
	.cmd_rcgr = 0x07000,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0_map,
	.freq_tbl = ftbl_gcc_blsp1_qup1_6_i2c_apps_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "blsp1_qup6_i2c_apps_clk_src",
		.parent_names = gcc_xo_gpll0,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_blsp1_uart1_6_apps_clk[] = {
	F(3686400,	P_GPLL0_OUT_MAIN,	1,	72,	15625),
	F(7372800,	P_GPLL0_OUT_MAIN,	1,	144,	15625),
	F(14745600,	P_GPLL0_OUT_MAIN,	1,	288,	15625),
	F(16000000,	P_GPLL0_OUT_MAIN,	10,	1,	5),
	F(19200000,	P_XO,			1,	0,	0),
	F(24000000,	P_GPLL0_OUT_MAIN,	1,	3,	100),
	F(25000000,	P_GPLL0_OUT_MAIN,	16,	1,	2),
	F(32000000,	P_GPLL0_OUT_MAIN,	1,	1,	25),
	F(40000000,	P_GPLL0_OUT_MAIN,	1,	1,	20),
	F(46400000,	P_GPLL0_OUT_MAIN,	1,	29,	500),
	F(48000000,	P_GPLL0_OUT_MAIN,	1,	3,	50),
	F(51200000,	P_GPLL0_OUT_MAIN,	1,	8,	125),
	F(56000000,	P_GPLL0_OUT_MAIN,	1,	7,	100),
	F(58982400,	P_GPLL0_OUT_MAIN,	1,	1152,	15625),
	F(60000000,	P_GPLL0_OUT_MAIN,	1,	3,	40),
	{ }
};

static struct clk_rcg2 blsp1_uart1_apps_clk_src = {
	.cmd_rcgr = 0x02044,
	.mnd_width = 16,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0_map,
	.freq_tbl = ftbl_gcc_blsp1_uart1_6_apps_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "blsp1_uart1_apps_clk_src",
		.parent_names = gcc_xo_gpll0,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_rcg2 blsp1_uart2_apps_clk_src = {
	.cmd_rcgr = 0x03034,
	.mnd_width = 16,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0_map,
	.freq_tbl = ftbl_gcc_blsp1_uart1_6_apps_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "blsp1_uart2_apps_clk_src",
		.parent_names = gcc_xo_gpll0,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_camss_cci_clk[] = {
	F(19200000, P_XO,		1, 0, 0),
	F(37500000, P_GPLL0_MISC,	1, 3, 64),
	{ }
};

static struct clk_rcg2 cci_clk_src = {
	.cmd_rcgr = 0x51000,
	.mnd_width = 8,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0m_map,
	.freq_tbl = ftbl_gcc_camss_cci_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "cci_clk_src",
		.parent_names = gcc_xo_gpll0m,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0m),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_camss_gp0_1_clk[] = {
	F(100000000, P_GPLL0_OUT_MAIN, 8, 0, 0),
	F(200000000, P_GPLL0_OUT_MAIN, 4, 0, 0),
	{ }
};

static struct clk_rcg2 camss_gp0_clk_src = {
	.cmd_rcgr = 0x54000,
	.mnd_width = 8,
	.hid_width = 5,
	.parent_map = gcc_gpll0_map,
	.freq_tbl = ftbl_gcc_camss_gp0_1_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "camss_gp0_clk_src",
		.parent_names = gcc_gpll0,
		.num_parents = ARRAY_SIZE(gcc_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_rcg2 camss_gp1_clk_src = {
	.cmd_rcgr = 0x55000,
	.mnd_width = 8,
	.hid_width = 5,
	.parent_map = gcc_gpll0_map,
	.freq_tbl = ftbl_gcc_camss_gp0_1_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "camss_gp1_clk_src",
		.parent_names = gcc_gpll0,
		.num_parents = ARRAY_SIZE(gcc_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_camss_jpeg0_clk[] = {
	F(133330000, P_GPLL0_OUT_MAIN, 6,	0, 0),
	F(266670000, P_GPLL0_OUT_MAIN, 3,	0, 0),
	F(320000000, P_GPLL0_OUT_MAIN, 2.5,	0, 0),
	{ }
};

static struct clk_rcg2 jpeg0_clk_src = {
	.cmd_rcgr = 0x57000,
	.hid_width = 5,
	.parent_map = gcc_gpll0_map,
	.freq_tbl = ftbl_gcc_camss_jpeg0_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "jpeg0_clk_src",
		.parent_names = gcc_gpll0,
		.num_parents = ARRAY_SIZE(gcc_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_camss_mclk0_1_clk[] = {
	F(23880000, P_GPLL6_MCLK,	1,	1, 45),
	F(66670000, P_GPLL0_OUT_MAIN,	12,	0, 0),
	{ }
};

static struct clk_rcg2 mclk0_clk_src = {
	.cmd_rcgr = 0x52000,
	.mnd_width = 8,
	.hid_width = 5,
	.parent_map = gcc_gpll0_gpll6m_map,
	.freq_tbl = ftbl_gcc_camss_mclk0_1_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "mclk0_clk_src",
		.parent_names = gcc_gpll0_gpll6m,
		.num_parents = ARRAY_SIZE(gcc_gpll0_gpll6m),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_rcg2 mclk1_clk_src = {
	.cmd_rcgr = 0x53000,
	.mnd_width = 8,
	.hid_width = 5,
	.parent_map = gcc_gpll0_gpll6m_map,
	.freq_tbl = ftbl_gcc_camss_mclk0_1_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "mclk1_clk_src",
		.parent_names = gcc_gpll0_gpll6m,
		.num_parents = ARRAY_SIZE(gcc_gpll0_gpll6m),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_camss_mclk2_clk[] = {
	F(66670000, P_GPLL0_OUT_MAIN, 12, 0, 0),
	{ }
};

static struct clk_rcg2 mclk2_clk_src = {
	.cmd_rcgr = 0x5c000,
	.mnd_width = 8,
	.hid_width = 5,
	.parent_map = gcc_gpll0_map,
	.freq_tbl = ftbl_gcc_camss_mclk2_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "mclk2_clk_src",
		.parent_names = gcc_gpll0,
		.num_parents = ARRAY_SIZE(gcc_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_camss_csi0_1phytimer_clk[] = {
	F(100000000, P_GPLL0_OUT_MAIN, 8, 0, 0),
	F(200000000, P_GPLL0_OUT_MAIN, 4, 0, 0),
	{ }
};

static struct clk_rcg2 csi0phytimer_clk_src = {
	.cmd_rcgr = 0x4e000,
	.hid_width = 5,
	.parent_map = gcc_gpll0_map,
	.freq_tbl = ftbl_gcc_camss_csi0_1phytimer_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "csi0phytimer_clk_src",
		.parent_names = gcc_gpll0,
		.num_parents = ARRAY_SIZE(gcc_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_rcg2 csi1phytimer_clk_src = {
	.cmd_rcgr = 0x4f000,
	.hid_width = 5,
	.parent_map = gcc_gpll0_map,
	.freq_tbl = ftbl_gcc_camss_csi0_1phytimer_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "csi1phytimer_clk_src",
		.parent_names = gcc_gpll0,
		.num_parents = ARRAY_SIZE(gcc_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_camss_cpp_clk[] = {
	F(160000000, P_GPLL0_OUT_MAIN, 5,	0, 0),
	F(200000000, P_GPLL0_OUT_MAIN, 4,	0, 0),
	F(228570000, P_GPLL0_OUT_MAIN, 3.5,	0, 0),
	F(266670000, P_GPLL0_OUT_MAIN, 3,	0, 0),
	F(320000000, P_GPLL0_OUT_MAIN, 2.5,	0, 0),
	F(465000000, P_GPLL2_OUT_MAIN, 2,	0, 0),
	{ }
};

static struct clk_rcg2 cpp_clk_src = {
	.cmd_rcgr = 0x58018,
	.hid_width = 5,
	.parent_map = gcc_gpll0_gpll2_map,
	.freq_tbl = ftbl_gcc_camss_cpp_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "cpp_clk_src",
		.parent_names = gcc_gpll0_gpll2,
		.num_parents = ARRAY_SIZE(gcc_gpll0_gpll2),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_gp1_3_clk[] = {
	F(19200000, P_XO, 1, 0, 0),
	{ }
};

static struct clk_rcg2 gp1_clk_src = {
	.cmd_rcgr = 0x08004,
	.mnd_width = 8,
	.hid_width = 5,
	.parent_map = gcc_xo_map,
	.freq_tbl = ftbl_gcc_gp1_3_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "gp1_clk_src",
		.parent_names = gcc_xo,
		.num_parents = ARRAY_SIZE(gcc_xo),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_rcg2 gp2_clk_src = {
	.cmd_rcgr = 0x09004,
	.mnd_width = 8,
	.hid_width = 5,
	.parent_map = gcc_xo_map,
	.freq_tbl = ftbl_gcc_gp1_3_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "gp2_clk_src",
		.parent_names = gcc_xo,
		.num_parents = ARRAY_SIZE(gcc_xo),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_rcg2 gp3_clk_src = {
	.cmd_rcgr = 0x0a004,
	.mnd_width = 8,
	.hid_width = 5,
	.parent_map = gcc_xo_map,
	.freq_tbl = ftbl_gcc_gp1_3_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "gp3_clk_src",
		.parent_names = gcc_xo,
		.num_parents = ARRAY_SIZE(gcc_xo),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_mdss_esc0_1_clk[] = {
	F(19200000, P_XO, 1, 0, 0),
	{ }
};

static struct clk_rcg2 esc0_clk_src = {
	.cmd_rcgr = 0x4d05c,
	.hid_width = 5,
	.parent_map = gcc_xo_map,
	.freq_tbl = ftbl_gcc_mdss_esc0_1_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "esc0_clk_src",
		.parent_names = gcc_xo,
		.num_parents = ARRAY_SIZE(gcc_xo),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_mdss_vsync_clk[] = {
	F(19200000, P_XO, 1, 0, 0),
	{ }
};

static struct clk_rcg2 vsync_clk_src = {
	.cmd_rcgr = 0x4d02c,
	.hid_width = 5,
	.parent_map = gcc_xo_map,
	.freq_tbl = ftbl_gcc_mdss_vsync_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "vsync_clk_src",
		.parent_names = gcc_xo,
		.num_parents = ARRAY_SIZE(gcc_xo),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_pdm2_clk[] = {
	F(64000000, P_GPLL0_OUT_MAIN, 12.5, 0, 0),
	{ }
};

static struct clk_rcg2 pdm2_clk_src = {
	.cmd_rcgr = 0x44010,
	.hid_width = 5,
	.parent_map = gcc_gpll0_map,
	.freq_tbl = ftbl_gcc_pdm2_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "pdm2_clk_src",
		.parent_names = gcc_gpll0,
		.num_parents = ARRAY_SIZE(gcc_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_sdcc1_2_apps_clk[] = {
	F(144000,	P_XO,			16,	3, 25),
	F(400000,	P_XO,			12,	1, 4),
	F(20000000,	P_GPLL0_OUT_MAIN,	10,	1, 4),
	F(25000000,	P_GPLL0_OUT_MAIN,	16,	1, 2),
	F(50000000,	P_GPLL0_OUT_MAIN,	16,	0, 0),
	F(100000000,	P_GPLL0_OUT_MAIN,	8,	0, 0),
	F(177770000,	P_GPLL0_OUT_MAIN,	4.5,	0, 0),
	F(200000000,	P_GPLL0_OUT_MAIN,	4,	0, 0),
	{ }
};

static struct clk_rcg2 sdcc1_apps_clk_src = {
	.cmd_rcgr = 0x42004,
	.mnd_width = 8,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0_map,
	.freq_tbl = ftbl_gcc_sdcc1_2_apps_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "sdcc1_apps_clk_src",
		.parent_names = gcc_xo_gpll0,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_rcg2 sdcc2_apps_clk_src = {
	.cmd_rcgr = 0x43004,
	.mnd_width = 8,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0_map,
	.freq_tbl = ftbl_gcc_sdcc1_2_apps_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "sdcc2_apps_clk_src",
		.parent_names = gcc_xo_gpll0,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_usb_hs_system_clk[] = {
	F(57140000,	P_GPLL0_OUT_MAIN, 14,	0, 0),
	F(80000000,	P_GPLL0_OUT_MAIN, 10,	0, 0),
	F( 100000000,	P_GPLL0_OUT_MAIN, 8,	0, 0),
	{ }
};

static struct clk_rcg2 usb_hs_system_clk_src = {
	.cmd_rcgr = 0x41010,
	.hid_width = 5,
	.parent_map = gcc_gpll0_map,
	.freq_tbl = ftbl_gcc_usb_hs_system_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "usb_hs_system_clk_src",
		.parent_names = gcc_gpll0,
		.num_parents = ARRAY_SIZE(gcc_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_usb_fs_system_clk[] = {
	F(64000000, P_GPLL0_MISC, 12.5, 0, 0),
	{ }
};

static struct clk_rcg2 usb_fs_system_clk_src = {
	.cmd_rcgr = 0x3f010,
	.mnd_width = 8,
	.hid_width = 5,
	.parent_map = gcc_gpll0m_map,
	.freq_tbl = ftbl_gcc_usb_fs_system_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "usb_fs_system_clk",
		.parent_names = gcc_gpll0m,
		.num_parents = ARRAY_SIZE(gcc_gpll0m),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_usb_ic_clk[] = {
	F(60000000, P_GPLL6_OUT_MAIN, 1, 1, 18),
	{ }
};

static struct clk_rcg2 usb_fs_ic_clk_src = {
	.cmd_rcgr = 0xfF034,
	.mnd_width = 8,
	.hid_width = 5,
	.parent_map = gcc_gpll6_map,
	.freq_tbl = ftbl_gcc_usb_ic_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "usb_fs_ic_clk_src",
		.parent_names = gcc_gpll6,
		.num_parents = ARRAY_SIZE(gcc_gpll6_map),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_gcc_venus0_vcodec0_clk[] = {
	F(133330000, P_GPLL0_OUT_MAIN, 6, 0, 0),
	F(200000000, P_GPLL0_OUT_MAIN, 4, 0, 0),
	F(266670000, P_GPLL0_OUT_MAIN, 3, 0, 0),
	{ }
};

static struct clk_rcg2 vcodec0_clk_src = {
	.cmd_rcgr = 0x4C000,
	.mnd_width = 8,
	.hid_width = 5,
	.parent_map = gcc_gpll0_map,
	.freq_tbl = ftbl_gcc_venus0_vcodec0_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "vcodec0_clk_src",
		.parent_names = gcc_gpll0,
		.num_parents = ARRAY_SIZE(gcc_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_branch gcc_blsp1_ahb_clk = {
	.halt_reg = 0x01008,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x45004,
		.enable_mask = BIT(10),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_blsp1_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_boot_rom_ahb_clk = {
	.halt_reg = 0x1300c,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x45004,
		.enable_mask = BIT(7),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_boot_rom_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_prng_ahb_clk = {
	.halt_reg = 0x13004,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x45004,
		.enable_mask = BIT(8),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_prng_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_apss_tcu_clk = {
	.halt_reg = 0x12018,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(1),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_apss_tcu_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_gfx_tbu_clk = {
	.halt_reg = 0x12010,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500C,
		.enable_mask = BIT(3),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_gfx_tbu_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_gfx_tcu_clk = {
	.halt_reg = 0x12020,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_gfx_tcu_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_gtcu_ahb_clk = {
	.halt_reg = 0x12044,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(13),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_gtcu_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_jpeg_tbu_clk = {
	.halt_reg = 0x12034,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(10),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_jpeg_tbu_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_mdp_tbu_clk = {
	.halt_reg = 0x1201c,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(4),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_mdp_tbu_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_smmu_cfg_clk = {
	.halt_reg = 0x12038,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(12),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_smmu_cfg_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_venus_tbu_clk = {
	.halt_reg = 0x12014,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(5),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_venus_tbu_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_vfe_tbu_clk = {
	.halt_reg = 0x1203c,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_vfe_tbu_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_cpp_tbu_clk = {
	.halt_reg = 0x12040,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500C,
		.enable_mask = BIT(14),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_cpp_tbu_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_mdp_rt_tbu_clk = {
	.halt_reg = 0x1201c,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500C,
		.enable_mask = BIT(15),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_mdp_rt_tbu_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_blsp1_qup1_i2c_apps_clk = {
	.halt_reg = 0x02008,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x02008,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_blsp1_qup1_i2c_apps_clk",
			.parent_names = (const char *[]){
				"blsp1_qup1_i2c_apps_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_blsp1_qup2_i2c_apps_clk = {
	.halt_reg = 0x03010,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x03010,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_blsp1_qup2_i2c_apps_clk",
			.parent_names = (const char *[]){
				"blsp1_qup2_i2c_apps_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_blsp1_qup3_i2c_apps_clk = {
	.halt_reg = 0x04020,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x04020,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_blsp1_qup3_i2c_apps_clk",
			.parent_names = (const char *[]){
				"blsp1_qup3_i2c_apps_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_blsp1_qup4_i2c_apps_clk = {
	.halt_reg = 0x05020,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x05020,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_blsp1_qup4_i2c_apps_clk",
			.parent_names = (const char *[]){
				"blsp1_qup4_i2c_apps_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_blsp1_qup5_i2c_apps_clk = {
	.halt_reg = 0x06020,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x06020,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_blsp1_qup5_i2c_apps_clk",
			.parent_names = (const char *[]){
				"blsp1_qup5_i2c_apps_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_blsp1_qup6_i2c_apps_clk = {
	.halt_reg = 0x07020,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x07020,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_blsp1_qup6_i2c_apps_clk",
			.parent_names = (const char *[]){
				"blsp1_qup6_i2c_apps_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_blsp1_uart1_apps_clk = {
	.halt_reg = 0x0203c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x0203c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_blsp1_uart1_apps_clk",
			.parent_names = (const char *[]){
				"blsp1_uart1_apps_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_blsp1_uart2_apps_clk = {
	.halt_reg = 0x0302c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x0302c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_blsp1_uart2_apps_clk",
			.parent_names = (const char *[]){
				"blsp1_uart2_apps_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_cci_ahb_clk = {
	.halt_reg = 0x5101c,
	.clkr = {
		.enable_reg = 0x5101c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_cci_ahb_clk",
			.parent_names = (const char *[]){
				"gcc_camss_ahb_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_cci_clk = {
	.halt_reg = 0x51018,
	.clkr = {
		.enable_reg = 0x51018,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_cci_clk",
			.parent_names = (const char *[]){
				"cci_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi0_ahb_clk = {
	.halt_reg = 0x4e040,
	.clkr = {
		.enable_reg = 0x4e040,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_csi0_ahb_clk",
			.parent_names = (const char *[]){
				"gcc_camss_ahb_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi0_clk = {
	.halt_reg = 0x4e03c,
	.clkr = {
		.enable_reg = 0x4e03c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_csi0_clk",
			.parent_names = (const char *[]){
				"csi0_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi0phy_clk = {
	.halt_reg = 0x4e048,
	.clkr = {
		.enable_reg = 0x4e048,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_csi0phy_clk",
			.parent_names = (const char *[]){
				"csi0_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi0pix_clk = {
	.halt_reg = 0x4e058,
	.clkr = {
		.enable_reg = 0x4e058,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_csi0pix_clk",
			.parent_names = (const char *[]){
				"csi0_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi0rdi_clk = {
	.halt_reg = 0x4e050,
	.clkr = {
		.enable_reg = 0x4e050,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_csi0rdi_clk",
			.parent_names = (const char *[]){
				"csi0_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi1_ahb_clk = {
	.halt_reg = 0x4f040,
	.clkr = {
		.enable_reg = 0x4f040,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_csi1_ahb_clk",
			.parent_names = (const char *[]){
				"gcc_camss_ahb_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi1_clk = {
	.halt_reg = 0x4f03c,
	.clkr = {
		.enable_reg = 0x4f03c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_csi1_clk",
			.parent_names = (const char *[]){
				"csi1_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi1phy_clk = {
	.halt_reg = 0x4f048,
	.clkr = {
		.enable_reg = 0x4f048,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_csi1phy_clk",
			.parent_names = (const char *[]){
				"csi1_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi1pix_clk = {
	.halt_reg = 0x4f058,
	.clkr = {
		.enable_reg = 0x4f058,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_csi1pix_clk",
			.parent_names = (const char *[]){
				"csi1_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi1rdi_clk = {
	.halt_reg = 0x4f050,
	.clkr = {
		.enable_reg = 0x4f050,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_csi1rdi_clk",
			.parent_names = (const char *[]){
				"csi1_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi2_ahb_clk = {
	.halt_reg = 0x3c040,
	.clkr = {
		.enable_reg = 0x3c040,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name ="gcc_camss_csi2_ahb_clk",
			.parent_names = (const char*[]) {
				"camss_top_ahb_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi2_clk = {
	.halt_reg = 0x3c03c,
	.clkr = {
		.enable_reg = 0x3c03c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name ="gcc_camss_csi2_clk",
			.parent_names = (const char*[]) {
				"csi2_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi2phy_clk = {
	.halt_reg = 0x3c048,
	.clkr = {
		.enable_reg = 0x3c048,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name ="gcc_camss_csi2phy_clk",
			.parent_names = (const char*[]) {
				"csi2_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi2pix_clk = {
	.halt_reg = 0x3c058,
	.clkr = {
		.enable_reg = 0x3c058,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name ="gcc_camss_csi2pix_clk",
			.parent_names = (const char*[]) {
				"csi2_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi2rdi_clk = {
	.halt_reg = 0x3c050,
	.clkr = {
		.enable_reg = 0x3c050,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name ="gcc_camss_csi2rdi_clk",
			.parent_names = (const char*[]) {
				"csi2_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi_vfe0_clk = {
	.halt_reg = 0x58050,
	.clkr = {
		.enable_reg = 0x58050,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_csi_vfe0_clk",
			.parent_names = (const char *[]){
				"vfe0_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_gp0_clk = {
	.halt_reg = 0x54018,
	.clkr = {
		.enable_reg = 0x54018,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_gp0_clk",
			.parent_names = (const char *[]){
				"camss_gp0_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_gp1_clk = {
	.halt_reg = 0x55018,
	.clkr = {
		.enable_reg = 0x55018,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_gp1_clk",
			.parent_names = (const char *[]){
				"camss_gp1_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_ispif_ahb_clk = {
	.halt_reg = 0x50004,
	.clkr = {
		.enable_reg = 0x50004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_ispif_ahb_clk",
			.parent_names = (const char *[]){
				"gcc_camss_ahb_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_jpeg0_clk = {
	.halt_reg = 0x57020,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x57020,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_jpeg0_clk",
			.parent_names = (const char *[]){
				"jpeg0_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_jpeg_ahb_clk = {
	.halt_reg = 0x57024,
	.clkr = {
		.enable_reg = 0x57024,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_jpeg_ahb_clk",
			.parent_names = (const char *[]){
				"gcc_camss_ahb_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_jpeg_axi_clk = {
	.halt_reg = 0x57028,
	.clkr = {
		.enable_reg = 0x57028,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_jpeg_axi_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_mclk0_clk = {
	.halt_reg = 0x52018,
	.clkr = {
		.enable_reg = 0x52018,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_mclk0_clk",
			.parent_names = (const char *[]){
				"mclk0_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_mclk1_clk = {
	.halt_reg = 0x53018,
	.clkr = {
		.enable_reg = 0x53018,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_mclk1_clk",
			.parent_names = (const char *[]){
				"mclk1_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_mclk2_clk = {
	.halt_reg = 0x5c018,
	.clkr = {
		.enable_reg = 0x5c018,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name ="gcc_camss_mclk2_clk",
			.parent_names = (const char*[]) {
				"mclk2_clk_src",
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_micro_ahb_clk = {
	.halt_reg = 0x5600c,
	.clkr = {
		.enable_reg = 0x5600c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_micro_ahb_clk",
			.parent_names = (const char *[]){
				"gcc_camss_ahb_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi0phytimer_clk = {
	.halt_reg = 0x4e01c,
	.clkr = {
		.enable_reg = 0x4e01c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_csi0phytimer_clk",
			.parent_names = (const char *[]){
				"csi0phytimer_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_csi1phytimer_clk = {
	.halt_reg = 0x4f01c,
	.clkr = {
		.enable_reg = 0x4f01c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_csi1phytimer_clk",
			.parent_names = (const char *[]){
				"csi1phytimer_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_ahb_clk = {
	.halt_reg = 0x56004,
	.clkr = {
		.enable_reg = 0x5a014,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_top_ahb_clk = {
	.halt_reg = 0x5a014,
	.clkr = {
		.enable_reg = 0x56004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_top_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_cpp_ahb_clk = {
	.halt_reg = 0x58040,
	.clkr = {
		.enable_reg = 0x58040,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_cpp_ahb_clk",
			.parent_names = (const char *[]){
				"gcc_camss_ahb_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_cpp_clk = {
	.halt_reg = 0x5803c,
	.clkr = {
		.enable_reg = 0x5803c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_cpp_clk",
			.parent_names = (const char *[]){
				"cpp_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_vfe0_clk = {
	.halt_reg = 0x58038,
	.clkr = {
		.enable_reg = 0x58038,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_vfe0_clk",
			.parent_names = (const char *[]){
				"vfe0_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_vfe_ahb_clk = {
	.halt_reg = 0x58044,
	.clkr = {
		.enable_reg = 0x58044,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_vfe_ahb_clk",
			.parent_names = (const char *[]){
				"gcc_camss_ahb_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_camss_vfe_axi_clk = {
	.halt_reg = 0x58048,
	.clkr = {
		.enable_reg = 0x58048,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_camss_vfe_axi_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_oxili_gmem_clk = {
	.halt_reg = 0x59024,
	.clkr = {
		.enable_reg = 0x59024,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_oxili_gmem_clk",
			.parent_names = (const char *[]){
				"gfx3d_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_gp1_clk = {
	.halt_reg = 0x08000,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x08000,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_gp1_clk",
			.parent_names = (const char *[]){
				"gp1_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_gp2_clk = {
	.halt_reg = 0x09000,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x09000,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_gp2_clk",
			.parent_names = (const char *[]){
				"gp2_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_gp3_clk = {
	.halt_reg = 0x0a000,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x0a000,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_gp3_clk",
			.parent_names = (const char *[]){
				"gp3_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_mdss_ahb_clk = {
	.halt_reg = 0x4d07c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4d07c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_mdss_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_mdss_axi_clk = {
	.halt_reg = 0x4d080,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4d080,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_mdss_axi_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_mdss_esc0_clk = {
	.halt_reg = 0x4d098,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4d098,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_mdss_esc0_clk",
			.parent_names = (const char *[]){
				"esc0_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_mdss_mdp_clk = {
	.halt_reg = 0x4d088,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4D088,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_mdss_mdp_clk",
			.parent_names = (const char *[]){
				"mdp_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_mdss_vsync_clk = {
	.halt_reg = 0x4d090,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4d090,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_mdss_vsync_clk",
			.parent_names = (const char *[]){
				"vsync_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_mss_cfg_ahb_clk = {
	.halt_reg = 0x49000,
	.clkr = {
		.enable_reg = 0x49000,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_mss_cfg_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_mss_q6_bimc_axi_clk = {
	.halt_reg = 0x49004,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x49004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name ="gcc_mss_q6_bimc_axi_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_oxili_ahb_clk = {
	.halt_reg = 0x59028,
	.clkr = {
		.enable_reg = 0x59028,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_oxili_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_oxili_timer_clk = {
	.halt_reg = 0x59040,
	.clkr = {
		.enable_reg = 0x59040,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name ="gcc_oxili_timer_clk",
			.parent_names = (const char *[]) { "xo", },
			.num_parents = 1,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_oxili_gfx3d_clk = {
	.halt_reg = 0x59020,
	.clkr = {
		.enable_reg = 0x59020,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_oxili_gfx3d_clk",
			.parent_names = (const char *[]){
				"gfx3d_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_pdm2_clk = {
	.halt_reg = 0x4400c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4400c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_pdm2_clk",
			.parent_names = (const char *[]){
				"pdm2_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_pdm_ahb_clk = {
	.halt_reg = 0x44004,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x44004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_pdm_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_sdcc1_ahb_clk = {
	.halt_reg = 0x4201c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4201c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_sdcc1_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_sdcc1_apps_clk = {
	.halt_reg = 0x42018,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x42018,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_sdcc1_apps_clk",
			.parent_names = (const char *[]){
				"sdcc1_apps_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_sdcc2_ahb_clk = {
	.halt_reg = 0x4301c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4301c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_sdcc2_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_sdcc2_apps_clk = {
	.halt_reg = 0x43018,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x43018,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_sdcc2_apps_clk",
			.parent_names = (const char *[]){
				"sdcc2_apps_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_usb2a_phy_sleep_clk = {
	.halt_reg = 0x4102c,
	.clkr = {
		.enable_reg = 0x4102c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_usb2a_phy_sleep_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_usb_hs_ahb_clk = {
	.halt_reg = 0x41008,
	.clkr = {
		.enable_reg = 0x41008,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_usb_hs_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_usb_hs_system_clk = {
	.halt_reg = 0x41004,
	.clkr = {
		.enable_reg = 0x41004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_usb_hs_system_clk",
			.parent_names = (const char *[]){
				"usb_hs_system_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_usb_fs_ahb_clk = {
	.halt_reg = 0x3f008,
	.clkr = {
		.enable_reg = 0x3f008,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name ="gcc_usb_fs_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_usb_fs_ic_clk = {
	.halt_reg = 0x3f030,
	.clkr = {
		.enable_reg = 0x3f030,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name ="gcc_usb_fs_ic_clk",
			.parent_names = (const char*[]) {
				"usb_fs_ic_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_usb_fs_system_clk = {
	.halt_reg = 0x3f004,
	.clkr = {
		.enable_reg = 0x3f004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name ="gcc_usb_fs_system_clk",
			.parent_names = (const char*[]) {
				"usb_fs_system_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_venus0_ahb_clk = {
	.halt_reg = 0x4c020,
	.clkr = {
		.enable_reg = 0x4c020,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_venus0_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_venus0_axi_clk = {
	.halt_reg = 0x4c024,
	.clkr = {
		.enable_reg = 0x4c024,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_venus0_axi_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_venus0_vcodec0_clk = {
	.halt_reg = 0x4c01c,
	.clkr = {
		.enable_reg = 0x4c01c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_venus0_vcodec0_clk",
			.parent_names = (const char *[]){
				"vcodec0_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_venus0_core0_vcodec0_clk = {
	.halt_reg = 0x4c02c,
	.clkr = {
		.enable_reg = 0x4c02c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name ="gcc_venus0_core0_vcodec0_clk",
			.parent_names = (const char*[]) {
				"vcodec0_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_venus0_core1_vcodec0_clk = {
	.halt_reg = 0x4c034,
	.clkr = {
		.enable_reg = 0x4c034,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name ="gcc_venus0_core1_vcodec0_clk",
			.parent_names = (const char*[]) {
				"vcodec0_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_bimc_gfx_clk = {
	.halt_reg = 0x31024,
	.clkr = {
		.enable_reg = 0x31024,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_bimc_gfx_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_bimc_gpu_clk = {
	.halt_reg = 0x31040,
	.clkr = {
		.enable_reg = 0x31040,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_bimc_gpu_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_crypto_clk = {
	.halt_reg = 0x1601c,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x45004,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_crypto_clk",
			.parent_names = (const char *[]){
				"crypto_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_crypto_ahb_clk = {
	.halt_reg = 0x16024,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x45004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_crypto_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_crypto_axi_clk = {
	.halt_reg = 0x16020,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x45004,
		.enable_mask = BIT(1),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_crypto_axi_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static const struct freq_tbl ftbl_gcc_crypto_clk[] = {
	F(50000000,	P_GPLL0_OUT_MAIN, 16,	0, 0),
	F(80000000,	P_GPLL0_OUT_MAIN, 10,	0, 0),
	F(100000000,	P_GPLL0_OUT_MAIN, 8,	0, 0),
	F(160000000,	P_GPLL0_OUT_MAIN, 5,	0, 0),
	{ }
};

static struct clk_rcg2 crypto_clk_src = {
	.cmd_rcgr = 0x16004,
	.hid_width = 5,
	.parent_map = gcc_gpll0_map,
	.freq_tbl = ftbl_gcc_crypto_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "crypto_clk_src",
		.parent_names = gcc_gpll0,
		.num_parents = ARRAY_SIZE(gcc_gpll0),
		.ops = &clk_rcg2_ops,
	},
};

/*static struct gate_clk gcc_snoc_qosgen_clk = {
	.en_mask = BIT(0),
	.en_reg = SNOC_QOSGEN,
	.base = &virt_bases[GCC_BASE],
	.clkr = {
		.enable_reg = 0x2601c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_snoc_qosgen_clk",
			.ops = &clk_gate2_ops,
		},
	},
};*/

static struct clk_regmap *gcc_msm8936_clocks[] = {
	/* PLLs */
	[GPLL0]				= &gpll0.clkr,
	[GPLL0_CLK_SRC]			= &gpll0_out_main,
	[GPLL0_AO]			= &gpll0_ao,
	[GPLL1]				= &gpll1.clkr,
	[GPLL1_CLK_SRC]			= &gpll1_out_main,
	[GPLL2]				= &gpll2.clkr,
	[GPLL2_CLK_SRC]			= &gpll2_out_main,
	[GPLL3]				= &gpll3.clkr,
	[GPLL3_CLK_SRC]			= &gpll3_out_main,
	[GPLL4]				= &gpll4.clkr,
	[GPLL4_CLK_SRC]			= &gpll4_out_main,
	[GPLL6]				= &gpll6.clkr,
	[GPLL6_CLK_SRC]			= &gpll6_out_main,
	[CLK_A53SS_C0_PLL]		= &a53ss_c0_pll.clkr,
	[CLK_A53SS_C1_PLL]		= &a53ss_c1_pll.clkr,
	[CLK_A53SS_CCI_PLL]		= &a53ss_cci_pll.clkr,

	/* RCGs */
	[APSS_AHB_CLK_SRC]		= &apss_ahb_clk_src.clkr,
	[CSI0_CLK_SRC]			= &csi0_clk_src.clkr,
	[CSI1_CLK_SRC]			= &csi1_clk_src.clkr,
	[CSI2_CLK_SRC]			= &csi2_clk_src.clkr,
	[VFE0_CLK_SRC]			= &vfe0_clk_src.clkr,
	[MDP_CLK_SRC]			= &mdp_clk_src.clkr,
	[GFX3D_CLK_SRC]			= &gfx3d_clk_src.clkr,
	[BLSP1_QUP1_I2C_APPS_CLK]	= &blsp1_qup1_i2c_apps_clk_src.clkr,
	[BLSP1_QUP2_I2C_APPS_CLK]	= &blsp1_qup2_i2c_apps_clk_src.clkr,
	[BLSP1_QUP3_I2C_APPS_CLK]	= &blsp1_qup3_i2c_apps_clk_src.clkr,
	[BLSP1_QUP4_I2C_APPS_CLK]	= &blsp1_qup4_i2c_apps_clk_src.clkr,
	[BLSP1_QUP5_I2C_APPS_CLK]	= &blsp1_qup5_i2c_apps_clk_src.clkr,
	[BLSP1_QUP6_I2C_APPS_CLK]	= &blsp1_qup6_i2c_apps_clk_src.clkr,
	[BLSP1_UART1_APPS_CLK]		= &blsp1_uart1_apps_clk_src.clkr,
	[BLSP1_UART2_APPS_CLK]		= &blsp1_uart2_apps_clk_src.clkr,
	[CCI_CLK_SRC]			= &cci_clk_src.clkr,
	[CAMSS_GP0_CLK_SRC]		= &camss_gp0_clk_src.clkr,
	[CAMSS_GP1_CLK_SRC]		= &camss_gp1_clk_src.clkr,
	[JPEG0_CLK_SRC]			= &jpeg0_clk_src.clkr,
	[MCLK0_CLK_SRC]			= &mclk0_clk_src.clkr,
	[MCLK1_CLK_SRC]			= &mclk1_clk_src.clkr,
	[MCLK2_CLK_SRC]			= &mclk2_clk_src.clkr,
	[CSI0PHYTIMER_CLK_SRC]		= &csi0phytimer_clk_src.clkr,
	[CSI1PHYTIMER_CLK_SRC]		= &csi1phytimer_clk_src.clkr,
	[CPP_CLK_SRC]			= &cpp_clk_src.clkr,
	[GP1_CLK_SRC]			= &gp1_clk_src.clkr,
	[GP2_CLK_SRC]			= &gp2_clk_src.clkr,
	[GP3_CLK_SRC]			= &gp3_clk_src.clkr,
	[ESC0_CLK_SRC]			= &esc0_clk_src.clkr,
	[VSYNC_CLK_SRC]			= &vsync_clk_src.clkr,
	[PDM2_CLK_SRC]			= &pdm2_clk_src.clkr,
	[SDCC1_APPS_CLK_SRC]		= &sdcc1_apps_clk_src.clkr,
	[SDCC2_APPS_CLK_SRC]		= &sdcc2_apps_clk_src.clkr,
	[USB_HS_SYSTEM_CLK_SRC]		= &usb_hs_system_clk_src.clkr,
	[USB_FS_SYSTEM_CLK_SRC]		= &usb_fs_system_clk_src.clkr,
	[USB_FS_IC_CLK_SRC]		= &usb_fs_ic_clk_src.clkr,
	[VCODEC0_CLK_SRC]		= &vcodec0_clk_src.clkr,

	/* Voteable Clocks */
	[GCC_BLSP1_AHB_CLK]		= &gcc_blsp1_ahb_clk.clkr,
	[GCC_BOOT_ROM_AHB_CLK]		= &gcc_boot_rom_ahb_clk.clkr,
	[GCC_PRNG_AHB_CLK]		= &gcc_prng_ahb_clk.clkr,
	[GCC_APSS_TCU_CLK]		= &gcc_apss_tcu_clk.clkr,
	[GCC_GFX_TBU_CLK]		= &gcc_gfx_tbu_clk.clkr,
	[GCC_GFX_TCU_CLK]		= &gcc_gfx_tcu_clk.clkr,
	[GCC_GTCU_AHB_CLK]		= &gcc_gtcu_ahb_clk.clkr,
	[GCC_JPEG_TBU_CLK]		= &gcc_jpeg_tbu_clk.clkr,
	[GCC_MDP_TBU_CLK]		= &gcc_mdp_tbu_clk.clkr,
	[GCC_SMMU_CFG_CLK]		= &gcc_smmu_cfg_clk.clkr,
	[GCC_VENUS_TBU_CLK]		= &gcc_venus_tbu_clk.clkr,
	[GCC_VFE_TBU_CLK]		= &gcc_vfe_tbu_clk.clkr,
	[GCC_CPP_TBU_CLK]		= &gcc_cpp_tbu_clk.clkr,
	[GCC_MDP_RT_TBU_CLK]		= &gcc_mdp_rt_tbu_clk.clkr,

	/* Branches */
	[GCC_BLSP1_QUP1_I2C_APPS_CLK]	= &gcc_blsp1_qup1_i2c_apps_clk.clkr,
	[GCC_BLSP1_QUP2_I2C_APPS_CLK]	= &gcc_blsp1_qup2_i2c_apps_clk.clkr,
	[GCC_BLSP1_QUP3_I2C_APPS_CLK]	= &gcc_blsp1_qup3_i2c_apps_clk.clkr,
	[GCC_BLSP1_QUP4_I2C_APPS_CLK]	= &gcc_blsp1_qup4_i2c_apps_clk.clkr,
	[GCC_BLSP1_QUP5_I2C_APPS_CLK]	= &gcc_blsp1_qup5_i2c_apps_clk.clkr,
	[GCC_BLSP1_QUP6_I2C_APPS_CLK]	= &gcc_blsp1_qup6_i2c_apps_clk.clkr,
	[GCC_BLSP1_UART1_APPS_CLK]	= &gcc_blsp1_uart1_apps_clk.clkr,
	[GCC_BLSP1_UART2_APPS_CLK]	= &gcc_blsp1_uart2_apps_clk.clkr,
	[GCC_CAMSS_CCI_AHB_CLK]		= &gcc_camss_cci_ahb_clk.clkr,
	[GCC_CAMSS_CCI_CLK]		= &gcc_camss_cci_clk.clkr,
	[GCC_CAMSS_CSI0_AHB_CLK]	= &gcc_camss_csi0_ahb_clk.clkr,
	[GCC_CAMSS_CSI0_CLK]		= &gcc_camss_csi0_clk.clkr,
	[GCC_CAMSS_CSI0PHY_CLK]		= &gcc_camss_csi0phy_clk.clkr,
	[GCC_CAMSS_CSI0PIX_CLK]		= &gcc_camss_csi0pix_clk.clkr,
	[GCC_CAMSS_CSI0RDI_CLK]		= &gcc_camss_csi0rdi_clk.clkr,
	[GCC_CAMSS_CSI1_AHB_CLK]	= &gcc_camss_csi1_ahb_clk.clkr,
	[GCC_CAMSS_CSI1_CLK]		= &gcc_camss_csi1_clk.clkr,
	[GCC_CAMSS_CSI1PHY_CLK]		= &gcc_camss_csi1phy_clk.clkr,
	[GCC_CAMSS_CSI1PIX_CLK]		= &gcc_camss_csi1pix_clk.clkr,
	[GCC_CAMSS_CSI1RDI_CLK]		= &gcc_camss_csi1rdi_clk.clkr,
	[GCC_CAMSS_CSI2_AHB_CLK]	= &gcc_camss_csi2_ahb_clk.clkr,
	[GCC_CAMSS_CSI2_CLK]		= &gcc_camss_csi2_clk.clkr,
	[GCC_CAMSS_CSI2PHY_CLK]		= &gcc_camss_csi2phy_clk.clkr,
	[GCC_CAMSS_CSI2PIX_CLK]		= &gcc_camss_csi2pix_clk.clkr,
	[GCC_CAMSS_CSI2RDI_CLK]		= &gcc_camss_csi2rdi_clk.clkr,
	[GCC_CAMSS_CSI_VFE0_CLK]	= &gcc_camss_csi_vfe0_clk.clkr,
	[GCC_CAMSS_GP0_CLK]		= &gcc_camss_gp0_clk.clkr,
	[GCC_CAMSS_GP1_CLK]		= &gcc_camss_gp1_clk.clkr,
	[GCC_CAMSS_ISPIF_AHB_CLK]	= &gcc_camss_ispif_ahb_clk.clkr,
	[GCC_CAMSS_JPEG0_CLK]		= &gcc_camss_jpeg0_clk.clkr,
	[GCC_CAMSS_JPEG_AHB_CLK]	= &gcc_camss_jpeg_ahb_clk.clkr,
	[GCC_CAMSS_JPEG_AXI_CLK]	= &gcc_camss_jpeg_axi_clk.clkr,
	[GCC_CAMSS_MCLK0_CLK]		= &gcc_camss_mclk0_clk.clkr,
	[GCC_CAMSS_MCLK1_CLK]		= &gcc_camss_mclk1_clk.clkr,
	[GCC_CAMSS_MCLK2_CLK]		= &gcc_camss_mclk2_clk.clkr,
	[GCC_CAMSS_MICRO_AHB_CLK]	= &gcc_camss_micro_ahb_clk.clkr,
	[GCC_CAMSS_CSI0PHYTIMER_CLK]	= &gcc_camss_csi0phytimer_clk.clkr,
	[GCC_CAMSS_CSI1PHYTIMER_CLK]	= &gcc_camss_csi1phytimer_clk.clkr,
	[GCC_CAMSS_AHB_CLK]		= &gcc_camss_ahb_clk.clkr,
	[GCC_CAMSS_TOP_AHB_CLK]		= &gcc_camss_top_ahb_clk.clkr,
	[GCC_CAMSS_CPP_AHB_CLK]		= &gcc_camss_cpp_ahb_clk.clkr,
	[GCC_CAMSS_CPP_CLK]		= &gcc_camss_cpp_clk.clkr,
	[GCC_CAMSS_VFE0_CLK]		= &gcc_camss_vfe0_clk.clkr,
	[GCC_CAMSS_VFE_AHB_CLK]		= &gcc_camss_vfe_ahb_clk.clkr,
	[GCC_CAMSS_VFE_AXI_CLK]		= &gcc_camss_vfe_axi_clk.clkr,
	[GCC_OXILI_GMEM_CLK]		= &gcc_oxili_gmem_clk.clkr,
	[GCC_GP1_CLK]			= &gcc_gp1_clk.clkr,
	[GCC_GP2_CLK]			= &gcc_gp2_clk.clkr,
	[GCC_GP3_CLK]			= &gcc_gp3_clk.clkr,
	[GCC_MDSS_AHB_CLK]		= &gcc_mdss_ahb_clk.clkr,
	[GCC_MDSS_AXI_CLK]		= &gcc_mdss_axi_clk.clkr,
	[GCC_MDSS_ESC0_CLK]		= &gcc_mdss_esc0_clk.clkr,
	[GCC_MDSS_MDP_CLK]		= &gcc_mdss_mdp_clk.clkr,
	[GCC_MDSS_VSYNC_CLK]		= &gcc_mdss_vsync_clk.clkr,
	[GCC_MSS_CFG_AHB_CLK]		= &gcc_mss_cfg_ahb_clk.clkr,
	[GCC_MSS_Q6_BIMC_AXI_CLK]	= &gcc_mss_q6_bimc_axi_clk.clkr,
	[GCC_OXILI_AHB_CLK]		= &gcc_oxili_ahb_clk.clkr,
	[GCC_OXILI_TIMER_CLK]		= &gcc_oxili_timer_clk.clkr,
	[GCC_OXILI_GFX3D_CLK]		= &gcc_oxili_gfx3d_clk.clkr,
	[GCC_PDM2_CLK]			= &gcc_pdm2_clk.clkr,
	[GCC_PDM_AHB_CLK]		= &gcc_pdm_ahb_clk.clkr,
	[GCC_SDCC1_AHB_CLK]		= &gcc_sdcc1_ahb_clk.clkr,
	[GCC_SDCC1_APPS_CLK]		= &gcc_sdcc1_apps_clk.clkr,
	[GCC_SDCC2_AHB_CLK]		= &gcc_sdcc2_ahb_clk.clkr,
	[GCC_SDCC2_APPS_CLK]		= &gcc_sdcc2_apps_clk.clkr,
	[GCC_USB2A_PHY_SLEEP_CLK]	= &gcc_usb2a_phy_sleep_clk.clkr,
	[GCC_USB_HS_AHB_CLK]		= &gcc_usb_hs_ahb_clk.clkr,
	[GCC_USB_HS_SYSTEM_CLK]		= &gcc_usb_hs_system_clk.clkr,
	[GCC_USB_FS_AHB_CLK]		= &gcc_usb_fs_ahb_clk.clkr,
	[GCC_USB_FS_IC_CLK]		= &gcc_usb_fs_ic_clk.clkr,
	[GCC_USB_FS_SYSTEM_CLK]		= &gcc_usb_fs_system_clk.clkr,
	[GCC_VENUS0_AHB_CLK]		= &gcc_venus0_ahb_clk.clkr,
	[GCC_VENUS0_AXI_CLK]		= &gcc_venus0_axi_clk.clkr,
	[GCC_VENUS0_VCODEC0_CLK]	= &gcc_venus0_vcodec0_clk.clkr,
	[GCC_VENUS0_CORE0_VCODEC0_CLK]	= &gcc_venus0_core0_vcodec0_clk.clkr,
	[GCC_VENUS0_CORE1_VCODEC0_CLK]	= &gcc_venus0_core1_vcodec0_clk.clkr,
	[GCC_BIMC_GFX_CLK]		= &gcc_bimc_gfx_clk.clkr,
	[GCC_BIMC_GPU_CLK]		= &gcc_bimc_gpu_clk.clkr,

	/* Crypto clocks */
	[GCC_CRYPTO_CLK]		= &gcc_crypto_clk.clkr,
	[GCC_CRYPTO_AHB_CLK]		= &gcc_crypto_ahb_clk.clkr,
	[GCC_CRYPTO_AXI_CLK]		= &gcc_crypto_axi_clk.clkr,
	[CRYPTO_CLK_SRC]		= &crypto_clk_src.clkr,

	/* QoS Reference clock */
//	[GCC_SNOC_QOSGEN_CLK]   = &gcc_snoc_qosgen_clk.clkr,
};

static struct clk_hw *gcc_msm8936_hws[] = {
	[GCC_XO]		= &xo.hw,
	[GCC_XO_A]		= &xo_a.hw,
	[WCNSS_M_CLK]		= &wcnss_m_clk.hw,
};

static const struct qcom_reset_map gcc_msm8936_resets[] = {
	[GCC_CAMSS_MICRO_BCR]	= { 0x56008 },
	[GCC_USB_HS_BCR]	= { 0x41000 },
};

static const struct regmap_config gcc_msm8936_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x80000,
	.fast_io	= true,
};

static const struct qcom_cc_desc gcc_msm8936_desc = {
	.config		= &gcc_msm8936_regmap_config,
	.clks		= gcc_msm8936_clocks,
	.num_clks	= ARRAY_SIZE(gcc_msm8936_clocks),
	.resets		= gcc_msm8936_resets,
	.num_resets	= ARRAY_SIZE(gcc_msm8936_resets),
};

static const struct of_device_id gcc_msm8936_match_table[] = {
	{ .compatible = "qcom,gcc-msm8936" },
	{ },
};
MODULE_DEVICE_TABLE(of, gcc_msm8936_match_table);

#define GCC_REG_BASE 0x1800000
static int gcc_msm8936_probe(struct platform_device *pdev)
{
	struct regmap *regmap;
	int i, ret;
	u32 val;

	regmap = qcom_cc_map(pdev, &gcc_msm8936_desc);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	/* Vote for GPLL0 to turn on. Needed by acpuclock. */
	regmap_update_bits(regmap, 0x45000, BIT(0), BIT(0));

	/* Register the hws */
	for (i = 0; i < ARRAY_SIZE(gcc_msm8936_hws); i++) {
		ret = devm_clk_hw_register(&pdev->dev, gcc_msm8936_hws[i]);
		if (ret)
			return ret;
	}

	ret = qcom_cc_really_probe(pdev, &gcc_msm8936_desc, regmap);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register GCC clocks\n");
		return ret;
	}

	clk_set_rate(apss_ahb_clk_src.clkr.hw.clk, 19200000);
	clk_prepare_enable(apss_ahb_clk_src.clkr.hw.clk);

	regmap_read(regmap, 0x59024, &val);
	val ^= 0xFF0;
	val |= (0 << 8);
	val |= (0 << 4);
	regmap_write(regmap, 0x59024, val);

	clk_pll_configure_sr_hpm_lp(&gpll3, regmap,
					&gpll3_config, true);
	clk_pll_configure_sr_hpm_lp(&gpll4, regmap,
					&gpll4_config, true);

	clk_set_rate(gpll3.clkr.hw.clk, 1100000000);

	/* Enable AUX2 clock for APSS */
	regmap_update_bits(regmap, 0x60000, BIT(2), BIT(2));

	dev_info(&pdev->dev, "Registered GCC clocks\n");

	return 0;
}

static struct platform_driver gcc_msm8936_driver = {
	.probe		= gcc_msm8936_probe,
	.driver		= {
		.name	= "gcc-msm8936",
		.of_match_table = gcc_msm8936_match_table,
	},
};

static int __init gcc_msm8936_init(void)
{
	return platform_driver_register(&gcc_msm8936_driver);
}
core_initcall(gcc_msm8936_init);

static void __exit gcc_msm8936_exit(void)
{
	platform_driver_unregister(&gcc_msm8936_driver);
}
module_exit(gcc_msm8936_exit);


#if 0
/*
 * MDSS BLOCK BEGIN
 */
static const struct parent_map gcc_xo_gpll0a_dsibyte_map[] = {
	{ P_XO, 0 },
	{ P_DSI0_PHYPLL_BYTE, 1 },
};

static const char * const gcc_xo_gpll0a_dsibyte[] = {
	"xo",
	"dsi0pllbyte",
};

static struct clk_rcg2 byte0_clk_src = {
	.cmd_rcgr = 0x4d044,
	.mnd_width = 0,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0a_dsibyte_map,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "byte0_clk_src",
		.parent_names = gcc_xo_gpll0a_dsibyte,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0a_dsibyte),
		.ops = &clk_byte2_ops,
		.flags = CLK_SET_RATE_PARENT | CLK_GET_RATE_NOCACHE,
	},
};

static const struct parent_map gcc_xo_gpll0a_dsiphy_map[] = {
	{ P_XO, 0 },
	{ P_DSI0_PHYPLL_DSI, 1 },
};

static const char * const gcc_xo_gpll0a_dsiphy[] = {
	"xo",
	"dsi0pll",
};

static struct clk_rcg2 pclk0_clk_src = {
	.cmd_rcgr = 0x4d000,
	.mnd_width = 8,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0a_dsiphy_map,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "pclk0_clk_src",
		.parent_names = gcc_xo_gpll0a_dsiphy,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0a_dsiphy),
		.ops = &clk_pixel_ops,
		.flags = CLK_SET_RATE_PARENT | CLK_GET_RATE_NOCACHE,
	},
};

static struct clk_branch gcc_mdss_byte0_clk = {
	.halt_reg = 0x4d094,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4d094,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name ="gcc_mdss_byte0_clk",
			.parent_names = (const char*[]) {
				"byte0_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_mdss_pclk0_clk = {
	.halt_reg = 0x4d084,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4d084,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name ="gcc_mdss_pclk0_clk",
			.parent_names = (const char*[]) {
				"pclk0_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

/* MDSS Clocks */
static struct clk_regmap *gcc_mdss_msm8936_clocks[] = {
	[BYTE0_CLK_SRC]		= &byte0_clk_src.clkr,
	[PCLK0_CLK_SRC]		= &pclk0_clk_src.clkr,
	[GCC_MDSS_BYTE0_CLK]	= &gcc_mdss_byte0_clk.clkr,
	[GCC_MDSS_PCLK0_CLK]	= &gcc_mdss_pclk0_clk.clkr,
};

static const struct qcom_cc_desc gcc_mdss_msm8936_desc = {
	.config		= &gcc_msm8936_regmap_config,
	.clks		= gcc_mdss_msm8936_clocks,
	.num_clks	= ARRAY_SIZE(gcc_mdss_msm8936_clocks),
};

static struct of_device_id gcc_mdss_msm8936_match_table[] = {
	{ .compatible = "qcom,gcc-mdss-msm8936" },
	{ },
};

static int gcc_mdss_msm8936_probe(struct platform_device *pdev)
{
	void __iomem *base;
	struct resource *res;
	struct regmap *regmap;
	int ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Failed to get MDSS resources.\n");
		return -EINVAL;
	}

	base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR(base)) {
		dev_err(&pdev->dev, "Unable to map MDSS clock controller.\n");
		return PTR_ERR(base);
	}

	regmap = devm_regmap_init_mmio(&pdev->dev, base, gcc_mdss_msm8936_desc.config);
	if (IS_ERR(regmap)) {
		dev_err(&pdev->dev, "Unable to map MDSS MMIO.\n");
		return PTR_ERR(regmap);
	}

	ret = qcom_cc_really_probe(pdev, &gcc_mdss_msm8936_desc, regmap);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register MDSS clocks\n");
		return ret;
	}

	dev_info(&pdev->dev, "Registered GCC MDSS clocks.\n");

	return ret;
}

static struct platform_driver gcc_mdss_msm8936_driver = {
	.probe = gcc_mdss_msm8936_probe,
	.driver = {
		.name = "gcc-mdss-msm8936",
		.of_match_table = gcc_mdss_msm8936_match_table,
		.owner = THIS_MODULE,
	},
};

int __init gcc_mdss_msm8936_init(void)
{
	return platform_driver_register(&gcc_mdss_msm8936_driver);
}
fs_initcall_sync(gcc_mdss_msm8936_init);
/*
 * MDSS BLOCK END
 */
#endif

MODULE_DESCRIPTION("Qualcomm GCC MSM8936 Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:gcc-msm8936");