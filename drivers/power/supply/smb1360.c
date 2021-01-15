// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/completion.h>
#include <linux/extcon-provider.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

/* Charger Registers */
#define CFG_BATT_CHG_REG		0x00
#define CHG_ITERM_MASK			GENMASK(2, 0)
#define RECHG_MV_MASK			GENMASK(6, 5)
#define RECHG_MV_SHIFT			5
#define OTG_CURRENT_MASK		GENMASK(4, 3)
#define OTG_CURRENT_SHIFT		3

#define CFG_BATT_CHG_ICL_REG		0x05
#define AC_INPUT_ICL_PIN_BIT		BIT(7)
#define AC_INPUT_PIN_HIGH_BIT		BIT(6)

#define CFG_GLITCH_FLT_REG		0x06
#define AICL_ENABLED_BIT		BIT(0)
#define INPUT_UV_GLITCH_FLT_20MS_BIT	BIT(7)

#define CFG_CHG_MISC_REG		0x7
#define CHG_EN_BY_PIN_BIT		BIT(7)
#define CHG_EN_ACTIVE_LOW_BIT		BIT(6)
#define PRE_TO_FAST_REQ_CMD_BIT		BIT(5)
#define CFG_BAT_OV_ENDS_CHG_CYC		BIT(4)
#define CHG_CURR_TERM_DIS_BIT		BIT(3)
#define CFG_AUTO_RECHG_DIS_BIT		BIT(2)
#define CFG_CHG_INHIBIT_EN_BIT		BIT(0)

#define CFG_STAT_CTRL_REG		0x09
#define CHG_STAT_IRQ_ONLY_BIT		BIT(4)
#define CHG_TEMP_CHG_ERR_BLINK_BIT	BIT(3)
#define CHG_STAT_ACTIVE_HIGH_BIT	BIT(1)
#define CHG_STAT_DISABLE_BIT		BIT(0)

#define CFG_SFY_TIMER_CTRL_REG		0x0A
#define SAFETY_TIME_EN_BIT		BIT(4)
#define SAFETY_TIME_DISABLE_BIT		BIT(5)
#define SAFETY_TIME_MINUTES_SHIFT	2
#define SAFETY_TIME_MINUTES_MASK	GENMASK(3, 2)

#define CFG_FG_BATT_CTRL_REG		0x0E

#define IRQ_CFG_REG			0x0F
#define IRQ_INTERNAL_TEMPERATURE_BIT	BIT(0)
#define IRQ_DCIN_UV_BIT			BIT(2)
#define IRQ_BAT_HOT_COLD_SOFT_BIT	BIT(6)
#define IRQ_HOT_COLD_HARD_BIT		BIT(7)

#define IRQ2_CFG_REG			0x10
#define IRQ2_VBAT_LOW_BIT		BIT(0)
#define IRQ2_BATT_MISSING_BIT		BIT(1)
#define IRQ2_POWER_OK_BIT		BIT(2)
#define IRQ2_CHG_PHASE_CHANGE_BIT	BIT(4)
#define IRQ2_CHG_ERR_BIT		BIT(6)
#define IRQ2_SAFETY_TIMER_BIT		BIT(7)

#define IRQ3_CFG_REG			0x11
#define SOC_FULL_BIT			BIT(0)
#define SOC_EMPTY_BIT			BIT(1)
#define SOC_MAX_BIT			BIT(2)
#define SOC_MIN_BIT			BIT(3)
#define SOC_CHANGE_BIT			BIT(4)
#define FG_ACCESS_OK_BIT		BIT(6)

#define BATT_CHG_FLT_VTG_REG		0x15
#define VFLOAT_MASK			GENMASK(6, 0)

/* Command Registers */
#define CMD_I2C_REG			0x40
#define ALLOW_VOLATILE_BIT		BIT(6)
#define FG_ACCESS_ENABLED_BIT		BIT(5)
#define FG_RESET_BIT			BIT(4)
#define CYCLE_STRETCH_CLEAR_BIT		BIT(3)

#define CMD_CHG_REG			0x42
#define CMD_CHG_EN			BIT(1)
#define CMD_OTG_EN_BIT			BIT(0)

/* Status Registers */
#define STATUS_3_REG			0x4B
#define CHG_HOLD_OFF_BIT		BIT(3)
#define CHG_TYPE_MASK			GENMASK(2, 1)
#define CHG_TYPE_SHIFT			1
#define BATT_NOT_CHG_VAL		0x0
#define BATT_PRE_CHG_VAL		0x1
#define BATT_FAST_CHG_VAL		0x2
#define BATT_TAPER_CHG_VAL		0x3

#define REVISION_CTRL_REG		0x4F
#define DEVICE_REV_MASK			GENMASK(3, 0)

/* IRQ Status Registers */
#define IRQ_REG				0x50

#define IRQ_A_REG			0x50
#define IRQ_A_HOT_HARD_BIT		BIT(6)
#define IRQ_A_COLD_HARD_BIT		BIT(4)
#define IRQ_A_HOT_SOFT_BIT		BIT(2)
#define IRQ_A_COLD_SOFT_BIT		BIT(0)

#define IRQ_C_REG			0x52
#define IRQ_C_CHG_TERM_BIT		BIT(0)

#define IRQ_E_REG			0x54
#define IRQ_E_USBIN_UV_BIT		BIT(0)
#define IRQ_E_INHIBIT_BIT		BIT(6)

#define IRQ_F_REG			0x54
#define IRQ_F_OTG_FAIL_BIT		BIT(4)
#define IRQ_F_OTG_OC_BIT		BIT(6)

#define IRQ_G_REG			0x56
#define IRQ_G_SOC_CHANGE_BIT		BIT(0)

#define IRQ_H_REG			0x57
#define IRQ_H_EMPTY_SOC_BIT		BIT(4)

#define IRQ_I_REG			0x58
#define IRQ_I_FG_ACCESS_ALLOWED_BIT	BIT(0)

/* FG registers - IRQ config register */
#define SOC_DELTA_REG			0x28
#define VTG_MIN_REG			0x2B
#define SOC_MAX_REG			0x24
#define SOC_MIN_REG			0x25
#define VTG_EMPTY_REG			0x26

#define SOC_DELTA_VAL			1
#define SOC_MIN_VAL			5
#define SOC_MAX_VAL			100

/* FG SHADOW registers */
#define SHDW_FG_ESR_ACTUAL		0x20
#define SHDW_FG_BATT_STATUS		0x60
#define SHDW_FG_MSYS_SOC		0x61
#define SHDW_FG_CAPACITY		0x62
#define SHDW_FG_VTG_NOW			0x69
#define SHDW_FG_CURR_NOW		0x6B
#define SHDW_FG_BATT_TEMP		0x6D

#define SHDN_CTRL_REG			0x1A
#define SHDN_CMD_USE_BIT		BIT(1)
#define SHDN_CMD_POLARITY_BIT		BIT(2)

#define VOLTAGE_PREDICTED_REG		0x80
#define CC_TO_SOC_COEFF			0xBA

#define CMD_IL_REG			0x41
#define USB_CTRL_MASK			GENMASK(1, 0)
#define USB_100_BIT			0x00
#define USB_500_BIT			0x01
#define USB_AC_BIT			0x11
#define SHDN_CMD_BIT			BIT(7)

/* Constants */
#define FG_RESET_THRESHOLD_MV		15
#define SMB1360_REV_1			0x01

#define MIN_FLOAT_MV			3460
#define MAX_FLOAT_MV			4730
#define VFLOAT_STEP_MV			10

#define MIN_RECHG_MV			50
#define MAX_RECHG_MV			300

#define SMB1360_FG_ACCESS_TIMEOUT_MS	15000

#define SMB1360_POWERON_DELAY_MS	2000
#define SMB1360_FG_RESET_DELAY_MS	1500

#define CFG_FG_OTP_BACK_UP_ENABLE	BIT(7)

#define BATT_PROFILE_SELECT_MASK	GENMASK(3, 0)
#define BATT_PROFILEA_MASK		0x0
#define BATT_PROFILEB_MASK		0xF

#define BATT_ID_RESULT_BIT		GENMASK(6, 4)
#define BATT_ID_SHIFT			4

#define BATTERY_PROFILE_BIT		BIT(0)

#define FG_I2C_CFG_MASK			GENMASK(2, 1)
#define FG_CFG_I2C_ADDR			0x2
#define CURRENT_GAIN_LSB_REG		0x1D
#define CURRENT_GAIN_MSB_REG		0x1E

#define OTP_WRITABLE_REG_1		0xE0
#define OTP_WRITABLE_REG_2		0xE1
#define OTP_WRITABLE_REG_3		0xE2
#define OTP_WRITABLE_REG_4		0xE3
#define OTP_WRITABLE_REG_5		0xE4
#define OTP_WRITABLE_REG_6		0xE5
#define OTP_WRITABLE_REG_7		0xE6
#define OTP_WRITABLE_REG_8		0xE7
#define OTP_BACKUP_MAP_REG		0xF0
#define CURRENT_GAIN_BITMAP		0x5000
#define HARD_JEITA_BITMAP		0x0500

#define OTP_GAIN_FIRST_HALF_REG_ADDR	0x1D
#define OTP_GAIN_SECOND_HALF_REG_ADDR	0x1E
#define OTP_HARD_COLD_REG_ADDR		0x12
#define OTP_HARD_HOT_REG_ADDR		0x13

#define NOMINAL_CAPACITY_REG		0xBC
#define ACTUAL_CAPACITY_REG		0xBE

#define CFG_CHG_FUNC_CTRL_REG		0x08
#define CHG_RECHG_THRESH_FG_SRC_BIT	BIT(1)

#define FG_IBATT_STANDBY_REG		0xCF
#define FG_AUTO_RECHARGE_SOC		0xD2
#define FG_SYS_CUTOFF_V_REG		0xD3
#define FG_CC_TO_CV_V_REG		0xD5
#define FG_ITERM_REG			0xD9
#define FG_THERM_C1_COEFF_REG		0xDB

#define TEMP_THRE_SET(x) (((x) + 300) / 10)

enum {
	BATTERY_PROFILE_A,
	BATTERY_PROFILE_B,
	BATTERY_PROFILE_MAX,
};

enum {
	IRQ_A, IRQ_B, IRQ_C, IRQ_D, IRQ_E, IRQ_F, IRQ_G, IRQ_H, IRQ_I,
	IRQ_COUNT
};

static const u32 smb1360_usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_NONE,
};

static const u32 smb1360_irq_changed_mask[IRQ_COUNT] = {
	[IRQ_A] = IRQ_A_HOT_HARD_BIT | IRQ_A_COLD_HARD_BIT
		  | IRQ_A_HOT_SOFT_BIT | IRQ_A_COLD_SOFT_BIT,
	[IRQ_C] = IRQ_C_CHG_TERM_BIT,
	[IRQ_E] = IRQ_E_USBIN_UV_BIT | IRQ_E_INHIBIT_BIT,
	[IRQ_G] = IRQ_G_SOC_CHANGE_BIT,
	[IRQ_H] = IRQ_H_EMPTY_SOC_BIT,
};

struct smb1360_battery {
	struct device			*dev;
	struct regmap			*regmap;
	struct regmap			*fg_regmap;
	struct power_supply		*psy;
	struct extcon_dev		*edev;
	struct regulator_dev		*otg_vreg;
	struct completion		fg_mem_access_granted;
	struct delayed_work		delayed_init_work;

	u32 revision;
	u32 irqstat[IRQ_COUNT];
	u32 connected_rid;
	u32 profile_rid[BATTERY_PROFILE_MAX];

	bool shdn_after_pwroff;
	bool rsense_10mohm;
};

static enum power_supply_property smb1360_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP
};

#define EXPONENT_MASK		0xF800
#define MANTISSA_MASK		0x3FF
#define SIGN_MASK		0x400
#define EXPONENT_SHIFT		11
#define SIGN_SHIFT		10
#define MICRO_UNIT		1000000ULL
static s64 float_decode(u16 reg)
{
	s64 final_val, exponent_val, mantissa_val;
	int exponent, mantissa, n;
	bool sign;

	exponent = (reg & EXPONENT_MASK) >> EXPONENT_SHIFT;
	mantissa = (reg & MANTISSA_MASK);
	sign = !!(reg & SIGN_MASK);

	mantissa_val = mantissa * MICRO_UNIT;

	n = exponent - 15;
	if (n < 0)
		exponent_val = MICRO_UNIT >> -n;
	else
		exponent_val = MICRO_UNIT << n;

	n = n - 10;
	if (n < 0)
		mantissa_val >>= -n;
	else
		mantissa_val <<= n;

	final_val = exponent_val + mantissa_val;

	if (sign)
		final_val *= -1;

	return final_val;
}

#define MAX_MANTISSA (1023 * 1000000ULL)
unsigned int float_encode(s64 float_val)
{
	int exponent = 0, sign = 0;
	unsigned int final_val = 0;

	if (float_val == 0)
		return 0;

	if (float_val < 0) {
		sign = 1;
		float_val = -float_val;
	}

	/* Reduce large mantissa until it fits into 10 bit */
	while (float_val >= MAX_MANTISSA) {
		exponent++;
		float_val >>= 1;
	}

	/* Increase small mantissa to improve precision */
	while (float_val < MAX_MANTISSA && exponent > -25) {
		exponent--;
		float_val <<= 1;
	}

	exponent = exponent + 25;

	/* Convert mantissa from micro-units to units */
	float_val = div_s64((float_val + MICRO_UNIT), (int)MICRO_UNIT);

	if (float_val == 1024) {
		exponent--;
		float_val <<= 1;
	}

	float_val -= 1024;

	/* Ensure that resulting number is within range */
	if (float_val > MANTISSA_MASK)
		float_val = MANTISSA_MASK;

	/* Convert to 5 bit exponent, 11 bit mantissa */
	final_val = (float_val & MANTISSA_MASK) | (sign << SIGN_SHIFT) |
		((exponent << EXPONENT_SHIFT) & EXPONENT_MASK);

	return final_val;
}

static int smb1360_update_le16(struct smb1360_battery *battery, u8 reg, const char *prop, s16 scale)
{
	int ret;
	u32 temp;
	__le16 val;

	if (device_property_read_u32(battery->dev, prop, &temp))
		return 0;

	if (scale > 0)
		temp = div_u64(temp * S16_MAX, scale);
	else if (scale < 0)
		temp = div_s64(temp * S16_MAX, scale);

	val = cpu_to_le16(temp);
	ret = regmap_bulk_write(battery->regmap, reg, &val, sizeof(val));

	if (ret)
		dev_err(battery->dev, "writing %s failed: %d\n", prop, ret);
	return ret;
}

static int smb1360_read_voltage(struct smb1360_battery *battery, u8 reg,
				int *voltage)
{
	__le16 val;
	int ret;

	ret = regmap_bulk_read(battery->regmap, reg, &val, sizeof(val));
	if (ret)
		return ret;

	*voltage = div_u64(le16_to_cpu(val) * 5000, S16_MAX);
	return 0;
}

static int smb1360_get_prop_batt_status(struct smb1360_battery *battery,
					union power_supply_propval *val)
{
	int ret;
	u32 reg, chg_type;

	if (battery->irqstat[IRQ_C] & IRQ_C_CHG_TERM_BIT) {
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	ret = regmap_read(battery->regmap, STATUS_3_REG, &reg);
	if (ret) {
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		return ret;
	}

	if (reg & CHG_HOLD_OFF_BIT) {
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	if (chg_type == BATT_NOT_CHG_VAL) {
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	val->intval = POWER_SUPPLY_STATUS_CHARGING;
	return 0;
}

static int smb1360_get_prop_charge_type(struct smb1360_battery *battery,
					union power_supply_propval *val)
{
	int ret;
	u32 reg, chg_type;

	ret = regmap_read(battery->regmap, STATUS_3_REG, &reg);
	if (ret) {
		val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		return ret;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	switch (chg_type) {
	case BATT_NOT_CHG_VAL:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case BATT_FAST_CHG_VAL:
	case BATT_TAPER_CHG_VAL:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case BATT_PRE_CHG_VAL:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	default:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	}

	return 0;
}

static int smb1360_get_prop_batt_health(struct smb1360_battery *battery,
					union power_supply_propval *val)
{
	if (battery->irqstat[IRQ_A] & IRQ_A_HOT_HARD_BIT)
		val->intval = POWER_SUPPLY_HEALTH_HOT;
	else if (battery->irqstat[IRQ_A] & IRQ_A_HOT_SOFT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_WARM;
	else if (battery->irqstat[IRQ_A] & IRQ_A_COLD_HARD_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COLD;
	else if (battery->irqstat[IRQ_A] & IRQ_A_COLD_SOFT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COOL;
	else
		val->intval = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int smb1360_get_prop_online(struct smb1360_battery *battery,
				   union power_supply_propval *val)
{
	val->intval = !(battery->irqstat[IRQ_E] & IRQ_E_USBIN_UV_BIT);
	return 0;
}

static int smb1360_get_prop_voltage_now(struct smb1360_battery *battery,
					union power_supply_propval *val)
{
	return smb1360_read_voltage(battery, SHDW_FG_VTG_NOW, &val->intval);
}

static int smb1360_get_prop_current_now(struct smb1360_battery *battery,
					union power_supply_propval *val)
{
	__le16 temp;
	int ret;

	ret = regmap_bulk_read(battery->regmap, SHDW_FG_CURR_NOW, &temp, sizeof(temp));
	if (ret)
		return ret;

	val->intval = div_s64(((s16)le16_to_cpu(temp)) * 2500, S16_MAX) * 1000;

	return 0;
}

static int smb1360_get_prop_chg_full_design(struct smb1360_battery *battery,
					    union power_supply_propval *val)
{
	__le16 fcc_mah;
	int ret;

	ret = regmap_bulk_read(battery->regmap, SHDW_FG_CAPACITY, &fcc_mah, sizeof(fcc_mah));
	if (ret)
		return ret;

	val->intval = le16_to_cpu(fcc_mah) * 1000;
	return 0;
}

static int smb1360_get_prop_batt_capacity(struct smb1360_battery *battery,
					  union power_supply_propval *val)
{
	int ret, soc = 0;
	u32 reg;

	if (battery->irqstat[IRQ_H] & IRQ_H_EMPTY_SOC_BIT) {
		val->intval = 0;
		return 0;
	}

	ret = regmap_read(battery->regmap, SHDW_FG_MSYS_SOC, &reg);
	if (ret)
		return ret;

	soc = DIV_ROUND_CLOSEST((100 * reg), U8_MAX);
	val->intval = clamp(soc, 0, 100);

	return 0;
}

static int smb1360_get_prop_batt_temp(struct smb1360_battery *battery,
				      union power_supply_propval *val)
{
	__le16 temp;
	int ret;

	ret = regmap_bulk_read(battery->regmap, SHDW_FG_BATT_TEMP, &temp, sizeof(temp));
	if (ret)
		return ret;

	temp = div_u64(le16_to_cpu(temp) * 625, 10000UL); /* temperature in K */
	val->intval = (temp - 273) * 10; /* temperature in decideg */

	return 0;
}

static int smb1360_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct smb1360_battery *battery = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return smb1360_get_prop_batt_status(battery, val);
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		return smb1360_get_prop_charge_type(battery, val);
	case POWER_SUPPLY_PROP_HEALTH:
		return smb1360_get_prop_batt_health(battery, val);
	case POWER_SUPPLY_PROP_ONLINE:
		return smb1360_get_prop_online(battery, val);
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return smb1360_get_prop_voltage_now(battery, val);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return smb1360_get_prop_current_now(battery, val);
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		return smb1360_get_prop_chg_full_design(battery, val);
	case POWER_SUPPLY_PROP_CAPACITY:
		return smb1360_get_prop_batt_capacity(battery, val);
	case POWER_SUPPLY_PROP_TEMP:
		return smb1360_get_prop_batt_temp(battery, val);
	default:
		return -EINVAL;
	}
}

static irqreturn_t smb1360_bat_irq(int irq, void *data)
{
	struct smb1360_battery *battery = data;
	u32 irqstat;
	int ret, i;
	bool changed = false;

	for (i = IRQ_A; i < IRQ_COUNT; ++i) {
		ret = regmap_read(battery->regmap, IRQ_REG + i, &irqstat);
		if (ret < 0)
			return IRQ_NONE;

		if ((irqstat ^ battery->irqstat[i]) & smb1360_irq_changed_mask[i])
			changed = true;

		battery->irqstat[i] = irqstat;
	}

	extcon_set_state_sync(battery->edev, EXTCON_USB,
			      !(battery->irqstat[IRQ_E] & IRQ_E_USBIN_UV_BIT));

	if (battery->irqstat[IRQ_F] & (IRQ_F_OTG_FAIL_BIT | IRQ_F_OTG_OC_BIT)) {
		dev_warn(battery->dev, "otg error: %d\n", battery->irqstat[IRQ_F]);
		regulator_disable_regmap(battery->otg_vreg);
	}

	if (battery->irqstat[IRQ_I] & IRQ_I_FG_ACCESS_ALLOWED_BIT)
		complete_all(&battery->fg_mem_access_granted);

	if (changed)
		power_supply_changed(battery->psy);

	return IRQ_HANDLED;
}

static const struct regulator_ops smb1360_regulator_ops = {
	.is_enabled	= regulator_is_enabled_regmap,
	.enable		= regulator_enable_regmap,
	.disable	= regulator_disable_regmap,
};

static const struct regulator_desc smb1360_regulator_descriptor = {
	.name		= "usb_otg_vbus",
	.of_match	= "usb-otg-vbus",
	.ops		= &smb1360_regulator_ops,
	.type		= REGULATOR_VOLTAGE,
	.owner		= THIS_MODULE,
	.enable_reg	= CMD_CHG_REG,
	.enable_mask	= CMD_OTG_EN_BIT,
	.enable_val	= CMD_OTG_EN_BIT,
	.fixed_uV	= 5000000,
	.n_voltages	= 1,
};

static const struct regulator_init_data smb1360_vbus_init_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
};

static int smb1360_register_vbus_regulator(struct smb1360_battery *battery)
{
	int ret = 0;
	struct regulator_config cfg = {
		.dev = battery->dev,
		.init_data = &smb1360_vbus_init_data,
	};

	battery->otg_vreg = devm_regulator_register(battery->dev,
						    &smb1360_regulator_descriptor, &cfg);
	if (IS_ERR(battery->otg_vreg)) {
		ret = PTR_ERR(battery->otg_vreg);
		dev_err(battery->dev, "can't register regulator: %d\n", ret);
	}

	return ret;
}

static int smb1360_enable_fg_access(struct smb1360_battery *battery)
{
	int ret = 0;
	u32 reg;

	ret = regmap_read(battery->regmap, IRQ_I_REG, &reg);
	if (ret || reg & IRQ_I_FG_ACCESS_ALLOWED_BIT)
		return ret;

	/* request FG access */
	ret = regmap_update_bits(battery->regmap, CMD_I2C_REG,
				 FG_ACCESS_ENABLED_BIT, FG_ACCESS_ENABLED_BIT);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&battery->fg_mem_access_granted,
					  msecs_to_jiffies(SMB1360_FG_ACCESS_TIMEOUT_MS));

	if (ret == 0) {
		/* Clear the FG access bit if request failed */
		regmap_update_bits(battery->regmap, CMD_I2C_REG,
				   FG_ACCESS_ENABLED_BIT, 0);
		return -ETIMEDOUT;
	}

	return 0;
}

static int smb1360_disable_fg_access(struct smb1360_battery *battery)
{
	int ret;

	ret = regmap_update_bits(battery->regmap, CMD_I2C_REG,
				 FG_ACCESS_ENABLED_BIT, 0);
	if (ret)
		dev_err(battery->dev, "couldn't disable FG access: %d\n", ret);

	reinit_completion(&battery->fg_mem_access_granted);

	return ret;
}

static int smb1360_force_fg_reset(struct smb1360_battery *battery)
{
	int ret;

	ret = regmap_update_bits(battery->regmap, CMD_I2C_REG, FG_RESET_BIT,
				 FG_RESET_BIT);
	if (ret) {
		dev_err(battery->dev, "couldn't reset FG: %d\n", ret);
		return ret;
	}

	msleep(SMB1360_FG_RESET_DELAY_MS);

	ret = regmap_update_bits(battery->regmap, CMD_I2C_REG, FG_RESET_BIT, 0);
	if (ret)
		dev_err(battery->dev, "couldn't un-reset FG: %d\n", ret);

	return ret;
}

static int smb1360_fg_reset(struct smb1360_battery *battery)
{
	int ret, temp, v_predicted, v_now;
	u32 val;

	if (!device_property_read_bool(battery->dev, "qcom,fg-reset-at-pon"))
		return 0;

	ret = smb1360_read_voltage(battery, VOLTAGE_PREDICTED_REG, &v_predicted);
	if (ret)
		return ret;
	ret = smb1360_read_voltage(battery, SHDW_FG_VTG_NOW, &v_now);
	if (ret)
		return ret;

	dev_info(battery->dev, "predicted: %d", v_predicted);
	dev_info(battery->dev, "now: %d", v_now);

	if (device_property_read_u32(battery->dev, "qcom,fg-reset-threshold-mv", &val))
		val = FG_RESET_THRESHOLD_MV;

	temp = abs(v_predicted - v_now);
	if (temp >= val) {
		/* delay for the FG access to settle */
		msleep(1500);

		ret = smb1360_force_fg_reset(battery);
		if (ret)
			return ret;
	}

	return 0;
}

static int smb1360_update_battery_capacity(struct smb1360_battery *battery)
{
	int ret;

	ret = smb1360_update_le16(battery, ACTUAL_CAPACITY_REG, "qcom,fg-batt-capacity-mah", 0);
	if (ret)
		return ret;

	ret = smb1360_update_le16(battery, NOMINAL_CAPACITY_REG, "qcom,fg-batt-capacity-mah", 0);
	if (ret)
		return ret;

	ret = smb1360_update_le16(battery, CC_TO_SOC_COEFF, "qcom,fg-cc-soc-coeff", 0);
	return ret;
}

static int smb1360_update_autorecharge_soc_threshold(struct smb1360_battery *battery)
{
	int ret;
	u32 val;

	if (device_property_read_u32(battery->dev, "qcom,fg-auto-recharge-soc", &val))
		return 0;

	ret = regmap_update_bits(battery->regmap, CFG_CHG_FUNC_CTRL_REG,
				 CHG_RECHG_THRESH_FG_SRC_BIT, CHG_RECHG_THRESH_FG_SRC_BIT);
	if (ret)
		return ret;

	val = DIV_ROUND_UP(val * U8_MAX, 100);
	ret = regmap_write(battery->regmap, FG_AUTO_RECHARGE_SOC, val);

	return ret;
}

static int smb1360_update_soc_bounds(struct smb1360_battery *battery)
{
	int ret;
	u32 val;

	val = abs(((SOC_DELTA_VAL * U8_MAX) / 100) - 1);
	ret = regmap_write(battery->regmap, SOC_DELTA_REG, val);
	if (ret)
		return ret;

	val = DIV_ROUND_UP(SOC_MIN_VAL * U8_MAX, 100);
	ret = regmap_write(battery->regmap, SOC_MIN_REG, val);
	if (ret)
		return ret;

	val = DIV_ROUND_UP(SOC_MAX_VAL * U8_MAX, 100);
	ret = regmap_write(battery->regmap, SOC_MAX_REG, val);

	return ret;
}

static int smb1360_update_voltage_bounds(struct smb1360_battery *battery)
{
	int ret;
	u32 val;

	if (device_property_read_u32(battery->dev, "qcom,fg-voltage-min-mv", &val))
		return 0;

	val = (val - 2500) * U8_MAX;
	val = DIV_ROUND_UP(val, 2500);
	ret = regmap_write(battery->regmap, VTG_MIN_REG, val);
	if (ret)
		return ret;

	if (device_property_read_u32(battery->dev, "qcom,fg-voltage-empty-mv", &val))
		return 0;

	val = (val - 2500) * U8_MAX;
	val = DIV_ROUND_UP(val, 2500);
	ret = regmap_write(battery->regmap, VTG_EMPTY_REG, val);

	return ret;
}

static int smb1360_fg_config(struct smb1360_battery *battery)
{
	int ret;

	ret = smb1360_enable_fg_access(battery);
	if (ret)
		return ret;

	ret = smb1360_fg_reset(battery);
	if (ret)
		dev_err(battery->dev, "smb1360_fg_reset failed");

	ret = smb1360_update_battery_capacity(battery);
	if (ret) {
		dev_err(battery->dev, "smb1360_update_battery_capacity failed");
		smb1360_disable_fg_access(battery);
		return ret;
	}

	ret = smb1360_update_le16(battery, FG_SYS_CUTOFF_V_REG, "qcom,fg-cutoff-voltage-mv", 5000);
	if (ret) {
		dev_err(battery->dev, "smb1360_update_cutoff_voltage failed");
		smb1360_disable_fg_access(battery);
		return ret;
	}

	ret = smb1360_update_le16(battery, FG_ITERM_REG, "qcom,fg-iterm-ma", -2500);
	if (ret) {
		dev_err(battery->dev, "smb1360_update_fg_iterm failed");
		smb1360_disable_fg_access(battery);
		return ret;
	}

	ret = smb1360_update_le16(battery, FG_IBATT_STANDBY_REG, "qcom,fg-ibatt-standby-ma", 2500);
	if (ret) {
		dev_err(battery->dev, "smb1360_update_fg_iterm_standby failed");
		smb1360_disable_fg_access(battery);
		return ret;
	}

	ret = smb1360_update_le16(battery, FG_CC_TO_CV_V_REG, "qcom,fg-cc-to-cv-mv", 5000);
	if (ret) {
		dev_err(battery->dev, "smb1360_update_cc_to_cv_threshold failed");
		smb1360_disable_fg_access(battery);
		return ret;
	}

	ret = smb1360_update_le16(battery, FG_THERM_C1_COEFF_REG, "qcom,thermistor-c1-coeff", 0);
	if (ret) {
		dev_err(battery->dev, "smb1360_update_thermistor_c1_coefficient failed");
		smb1360_disable_fg_access(battery);
		return ret;
	}

	ret = smb1360_update_autorecharge_soc_threshold(battery);
	if (ret) {
		dev_err(battery->dev, "smb1360_update_autorecharge_soc_threshold failed");
		smb1360_disable_fg_access(battery);
		return ret;
	}

	smb1360_disable_fg_access(battery);

	if (battery->revision == SMB1360_REV_1)
		return 0;

	ret = smb1360_update_soc_bounds(battery);
	if (ret) {
		dev_err(battery->dev, "smb1360_update_soc_bounds failed");
		return ret;
	}

	ret = smb1360_update_voltage_bounds(battery);
	if (ret)
		dev_err(battery->dev, "smb1360_update_voltage_bounds failed");

	return ret;
}

static int smb1360_float_voltage_set(struct smb1360_battery *battery)
{
	u32 val;

	if (device_property_read_u32(battery->dev, "qcom,float-voltage-mv", &val))
		return 0;

	if (val < MIN_FLOAT_MV || val > MAX_FLOAT_MV)
		return -EINVAL;

	val = (val - MIN_FLOAT_MV) / VFLOAT_STEP_MV;

	return regmap_update_bits(battery->regmap,
				  BATT_CHG_FLT_VTG_REG, VFLOAT_MASK, val);
}

static int smb1360_recharge_threshold_set(struct smb1360_battery *battery)
{
	u32 val;

	if (device_property_read_u32(battery->dev, "qcom,recharge-thresh-mv", &val))
		return 0;

	if (val < MIN_RECHG_MV || val > MAX_RECHG_MV)
		return -EINVAL;

	val = (val / 100) << RECHG_MV_SHIFT;

	return regmap_update_bits(battery->regmap,
				  CFG_BATT_CHG_REG, RECHG_MV_MASK, val);
}

static int smb1360_iterm_set(struct smb1360_battery *battery)
{
	int ret, iterm_ma;
	u8 val;

	if (device_property_read_bool(battery->dev, "qcom,iterm-disabled"))
		return regmap_update_bits(battery->regmap, CFG_CHG_MISC_REG,
					  CHG_CURR_TERM_DIS_BIT,
					  CHG_CURR_TERM_DIS_BIT);

	if (device_property_read_u32(battery->dev, "qcom,iterm-ma", &iterm_ma))
		return 0;

	if (battery->rsense_10mohm)
		iterm_ma = iterm_ma / 2;

	val = clamp(iterm_ma, 25, 200);
	val = DIV_ROUND_UP(val, 25) - 1;

	ret = regmap_update_bits(battery->regmap, CFG_BATT_CHG_REG,
				 CHG_ITERM_MASK, val);
	if (ret)
		return ret;

	ret = regmap_update_bits(battery->regmap, CFG_CHG_MISC_REG,
				 CHG_CURR_TERM_DIS_BIT, 0);
	if (ret)
		return ret;

	return 0;
}

static int smb1360_safety_time_set(struct smb1360_battery *battery)
{
	int ret, i;
	int chg_time[] = { 192, 384, 768, 1536, };

	u32 val;
	u8 mask, data;

	if (device_property_read_u32(battery->dev, "qcom,charging-timeout", &val))
		return 0;

	if (val > chg_time[ARRAY_SIZE(chg_time) - 1])
		return -EINVAL;

	mask = SAFETY_TIME_DISABLE_BIT;
	data = SAFETY_TIME_DISABLE_BIT;

	if (val != 0) {
		mask = SAFETY_TIME_DISABLE_BIT | SAFETY_TIME_MINUTES_MASK;

		for (i = 0; i < ARRAY_SIZE(chg_time); i++) {
			if (val <= chg_time[i]) {
				data = i << SAFETY_TIME_MINUTES_SHIFT;
				break;
			}
		}
	}

	ret = regmap_update_bits(battery->regmap,
				 CFG_SFY_TIMER_CTRL_REG, mask, data);
	if (ret < 0) {
		dev_err(battery->dev, "couldn't update safety timer: %d\n", ret);
		return ret;
	}

	return 0;
}

static int smb1360_enable(struct smb1360_battery *battery, bool enable)
{
	int ret = 0;
	u32 val = 0;
	bool polarity;

	ret = regmap_read(battery->regmap, SHDN_CTRL_REG, &val);
	if (ret < 0) {
		dev_err(battery->dev, "couldn't read SHDN_CTRL_REG: %d\n", ret);
		return ret;
	}

	if (!(val & SHDN_CMD_USE_BIT))
		return 0;

	polarity = !!(val & SHDN_CMD_POLARITY_BIT);
	val = (polarity != enable) ? SHDN_CMD_BIT : 0;

	ret = regmap_update_bits(battery->regmap, CMD_IL_REG, SHDN_CMD_BIT, val);
	if (ret < 0)
		dev_err(battery->dev, "couldn't shutdown: %d\n", ret);

	return ret;
}

static inline int smb1360_poweroff(struct smb1360_battery *battery)
{
	return smb1360_enable(battery, false);
}

static inline int smb1360_poweron(struct smb1360_battery *battery)
{
	return smb1360_enable(battery, true);
}

static int smb1360_configure_irq(struct smb1360_battery *battery)
{
	int ret;

	/* enabling only interesting interrupts */
	ret = regmap_write(battery->regmap, IRQ_CFG_REG,
			   IRQ_INTERNAL_TEMPERATURE_BIT
			   | IRQ_DCIN_UV_BIT
			   | IRQ_BAT_HOT_COLD_SOFT_BIT
			   | IRQ_HOT_COLD_HARD_BIT);
	if (ret) {
		dev_err(battery->dev, "couldn't set irq1: %d\n", ret);
		return ret;
	}

	ret = regmap_write(battery->regmap, IRQ2_CFG_REG,
			   IRQ2_VBAT_LOW_BIT
			   | IRQ2_BATT_MISSING_BIT
			   | IRQ2_POWER_OK_BIT
			   | IRQ2_CHG_PHASE_CHANGE_BIT
			   | IRQ2_CHG_ERR_BIT
			   | IRQ2_SAFETY_TIMER_BIT);
	if (ret) {
		dev_err(battery->dev, "couldn't set irq2: %d\n", ret);
		return ret;
	}

	ret = regmap_write(battery->regmap, IRQ3_CFG_REG,
			   SOC_FULL_BIT
			   | SOC_EMPTY_BIT
			   | SOC_MAX_BIT
			   | SOC_MIN_BIT
			   | SOC_CHANGE_BIT
			   | FG_ACCESS_OK_BIT);
	if (ret)
		dev_err(battery->dev, "couldn't set irq3: %d\n", ret);

	return ret;
}

static int smb1360_set_otg_current_limit(struct smb1360_battery *battery)
{
	int ret, i;
	int otg_curr_ma[] = {350, 550, 950, 1500};
	u32 val;

	if (device_property_read_u32(battery->dev, "qcom,otg-batt-curr-limit", &val))
		return 0;

	for (i = 0; i < ARRAY_SIZE(otg_curr_ma); i++) {
		if (otg_curr_ma[i] >= val)
			break;
	}

	if (i == ARRAY_SIZE(otg_curr_ma))
		i = i - 1;

	ret = regmap_update_bits(battery->regmap, CFG_BATT_CHG_REG, OTG_CURRENT_MASK,
				 i << OTG_CURRENT_SHIFT);

	return ret;
}

static int smb1360_hw_init(struct i2c_client *client)
{
	struct smb1360_battery *battery = i2c_get_clientdata(client);
	int ret;
	u8 val;

	ret = regmap_update_bits(battery->regmap, CMD_I2C_REG,
				 ALLOW_VOLATILE_BIT, ALLOW_VOLATILE_BIT);
	if (ret < 0) {
		dev_err(battery->dev, "couldn't configure volatile: %d\n", ret);
		return ret;
	}

	/* Bring SMB1360 out of shutdown, if it was enabled by default */
	ret = smb1360_poweron(battery);
	if (ret < 0) {
		dev_err(battery->dev, "smb1360 power on failed\n");
		return ret;
	}

	/* en chg by cmd reg, en chg by writing bit 1, en auto pre to fast */
	ret = regmap_update_bits(battery->regmap, CFG_CHG_MISC_REG,
				 CHG_EN_BY_PIN_BIT | CHG_EN_ACTIVE_LOW_BIT
				 | PRE_TO_FAST_REQ_CMD_BIT, 0);
	if (ret < 0)
		return ret;

	/* USB/AC pin settings */
	ret = regmap_update_bits(battery->regmap, CFG_BATT_CHG_ICL_REG,
				 AC_INPUT_ICL_PIN_BIT | AC_INPUT_PIN_HIGH_BIT,
				 AC_INPUT_PIN_HIGH_BIT);
	if (ret < 0)
		return ret;

	/* AICL enable and set input-uv glitch flt to 20ms */
	ret = regmap_update_bits(battery->regmap, CFG_GLITCH_FLT_REG,
				 AICL_ENABLED_BIT | INPUT_UV_GLITCH_FLT_20MS_BIT,
				 AICL_ENABLED_BIT | INPUT_UV_GLITCH_FLT_20MS_BIT);
	if (ret < 0)
		return ret;

	/* set the float voltage */
	ret = smb1360_float_voltage_set(battery);
	if (ret < 0)
		return ret;

	/* set iterm */
	ret = smb1360_iterm_set(battery);
	if (ret < 0)
		return ret;

	ret = smb1360_safety_time_set(battery);
	if (ret < 0)
		return ret;

	/* configure resume threshold, auto recharge and charge inhibit */
	ret = smb1360_recharge_threshold_set(battery);
	if (ret)
		return ret;

	val = device_property_read_bool(battery->dev, "qcom,recharge-disabled") ?
		CFG_AUTO_RECHG_DIS_BIT : 0;

	ret = regmap_update_bits(battery->regmap, CFG_CHG_MISC_REG,
				 CFG_AUTO_RECHG_DIS_BIT, val);
	if (ret) {
		dev_err(battery->dev, "couldn't set rechg-cfg: %d\n", ret);
		return ret;
	}

	val = device_property_read_bool(battery->dev, "qcom,chg-inhibit-disabled") ?
		0 : CFG_CHG_INHIBIT_EN_BIT;

	ret = regmap_update_bits(battery->regmap, CFG_CHG_MISC_REG,
				 CFG_CHG_INHIBIT_EN_BIT, val);
	if (ret) {
		dev_err(battery->dev, "couldn't set chg_inhibit: %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(battery->regmap, CFG_CHG_MISC_REG,
				 CFG_BAT_OV_ENDS_CHG_CYC, CFG_BAT_OV_ENDS_CHG_CYC);
	if (ret) {
		dev_err(battery->dev, "couldn't set charge ends on overvoltage bit: %d\n", ret);
		return ret;
	}

	/* interrupt enabling - active low */
	if (client->irq) {
		ret = regmap_update_bits(battery->regmap, CFG_STAT_CTRL_REG,
					 CHG_STAT_IRQ_ONLY_BIT
					 | CHG_STAT_ACTIVE_HIGH_BIT
					 | CHG_STAT_DISABLE_BIT
					 | CHG_TEMP_CHG_ERR_BLINK_BIT,
					 CHG_STAT_IRQ_ONLY_BIT);
		if (ret < 0) {
			dev_err(battery->dev, "couldn't set irq: %d\n", ret);
			return ret;
		}

		ret = smb1360_configure_irq(battery);
		if (ret < 0) {
			dev_err(battery->dev, "couldn't configure irq: %d\n", ret);
			return ret;
		}
	}

	ret = smb1360_set_otg_current_limit(battery);
	if (ret)
		dev_err(battery->dev, "couldn't set otg current limit: %d\n", ret);

	return ret;
}

static int smb1360_check_batt_profile(struct smb1360_battery *battery)
{
	int ret, i, timeout;
	u8 loaded_profile, new_profile = 0, bid_mask;
	u32 val = 0;

	if (!battery->connected_rid)
		return 0;

	ret = regmap_read(battery->regmap, SHDW_FG_BATT_STATUS, &val);
	if (ret)
		return ret;

	loaded_profile = !!(val & BATTERY_PROFILE_BIT) ?
		BATTERY_PROFILE_B : BATTERY_PROFILE_A;

	for (i = 0; i < BATTERY_PROFILE_MAX; i++) {
		if (abs(battery->profile_rid[i] - battery->connected_rid) <
				(div_u64(battery->connected_rid, 10)))
			break;
	}

	if (i == BATTERY_PROFILE_MAX || i == loaded_profile)
		return 0;

	new_profile = (loaded_profile == BATTERY_PROFILE_A) ?
		BATTERY_PROFILE_B : BATTERY_PROFILE_A;
	bid_mask = (new_profile == BATTERY_PROFILE_A) ?
		BATT_PROFILEA_MASK : BATT_PROFILEB_MASK;

	/* set the BID mask */
	ret = regmap_update_bits(battery->regmap, CFG_FG_BATT_CTRL_REG,
				 BATT_PROFILE_SELECT_MASK, bid_mask);
	if (ret)
		return ret;

	ret = smb1360_enable_fg_access(battery);
	if (ret)
		return ret;

	/* delay after handshaking for profile-switch to continue */
	msleep(1500);

	ret = smb1360_force_fg_reset(battery);
	if (ret) {
		smb1360_disable_fg_access(battery);
		return ret;
	}
	ret = smb1360_disable_fg_access(battery);
	if (ret)
		return ret;

	timeout = 10;
	while (timeout) {
		/* delay for profile to change */
		msleep(500);
		ret = regmap_read(battery->regmap, SHDW_FG_BATT_STATUS, &val);
		if (ret) {
			smb1360_disable_fg_access(battery);
			return ret;
		}

		val = !!(val & BATTERY_PROFILE_BIT);
		if (val == new_profile)
			break;
		timeout--;
	}

	if (!timeout)
		return -EBUSY;

	return 0;
}

static int smb1360_adjust_otp_current_gain(struct smb1360_battery *battery)
{
	int ret;
	u8 val[4];
	__le16 current_gain;
	u16 current_gain_encoded;

	ret = regmap_bulk_read(battery->fg_regmap, CURRENT_GAIN_LSB_REG,
			       &current_gain, sizeof(current_gain));
	if (ret)
		return ret;

	current_gain_encoded = le16_to_cpu(current_gain);
	current_gain_encoded = float_encode(MICRO_UNIT + (2 * float_decode(current_gain_encoded)));

	val[0] = OTP_GAIN_FIRST_HALF_REG_ADDR;
	val[1] = current_gain_encoded & 0xFF;
	val[2] = OTP_GAIN_SECOND_HALF_REG_ADDR;
	val[3] = (current_gain_encoded & 0xFF00) >> 8;

	return regmap_bulk_write(battery->fg_regmap, OTP_WRITABLE_REG_1, val, ARRAY_SIZE(val));
}

static int smb1360_set_otp_hard_jeita_threshold(struct smb1360_battery *battery)
{
	u8 val[4];
	u32 hot_val;
	s32 cold_val;

	if (device_property_read_u32(battery->dev, "qcom,otp-hot-bat-decidegc", &hot_val))
		return -EINVAL;

	if (device_property_read_u32(battery->dev, "qcom,otp-cold-bat-decidegc", &cold_val))
		return -EINVAL;

	val[0] = OTP_HARD_HOT_REG_ADDR;
	val[1] = TEMP_THRE_SET(hot_val);
	val[2] = OTP_HARD_COLD_REG_ADDR;
	val[3] = TEMP_THRE_SET(cold_val);

	if (val[1] < 0 || val[3] < 0)
		return -EINVAL;

	return regmap_bulk_write(battery->fg_regmap, OTP_WRITABLE_REG_5, val, ARRAY_SIZE(val));
}

static int smb1360_reconf_otp(struct smb1360_battery *battery)
{
	bool hard_jeita = device_property_read_bool(battery->dev, "qcom,otp-hard-jeita-config");
	u16 backup_map = 0;
	__be16 val;
	int ret;

	if (!battery->rsense_10mohm && !hard_jeita)
		return 0;

	ret = smb1360_enable_fg_access(battery);
	if (ret)
		return ret;

	if (battery->rsense_10mohm) {
		ret = smb1360_adjust_otp_current_gain(battery);
		if (ret)
			dev_err(battery->dev,
				"couldn't reconfigure gain for lower resistance: %d\n", ret);
		else
			backup_map |= CURRENT_GAIN_BITMAP;
	}

	if (hard_jeita) {
		ret = smb1360_set_otp_hard_jeita_threshold(battery);
		if (ret)
			dev_err(battery->dev, "unable to modify otp hard jeita: %d\n", ret);
		else
			backup_map |= HARD_JEITA_BITMAP;
	}

	val = cpu_to_be16(backup_map);
	ret = regmap_bulk_write(battery->fg_regmap, OTP_BACKUP_MAP_REG, &val, sizeof(val));
	if (ret)
		goto out;

	ret = regmap_update_bits(battery->regmap, CFG_FG_BATT_CTRL_REG,
				 CFG_FG_OTP_BACK_UP_ENABLE,
				 CFG_FG_OTP_BACK_UP_ENABLE);
	if (ret)
		dev_err(battery->dev, "failed to enable OTP back-up: %d\n", ret);

out:
	return smb1360_disable_fg_access(battery);
}

static int smb1360_delayed_hw_init(struct smb1360_battery *battery)
{
	int ret;

	ret = smb1360_check_batt_profile(battery);
	if (ret) {
		dev_err(battery->dev, "unable to modify battery profile: %d\n", ret);
		return ret;
	}

	ret = smb1360_reconf_otp(battery);
	if (ret) {
		dev_err(battery->dev, "couldn't reconfigure OTP\n");
		return ret;
	}

	ret = smb1360_fg_config(battery);
	if (ret)
		dev_err(battery->dev, "couldn't configure FG\n");

	return ret;
}

static void smb1360_delayed_init_work_fn(struct work_struct *work)
{
	int ret = 0;
	struct smb1360_battery *battery = container_of(work,
						       struct smb1360_battery,
						       delayed_init_work.work);

	ret = smb1360_delayed_hw_init(battery);
	if (!ret) {
		power_supply_changed(battery->psy);
	} else if (ret == -ETIMEDOUT) {
		ret = smb1360_force_fg_reset(battery);
		if (ret)
			return;
		schedule_delayed_work(&battery->delayed_init_work, 0);
	}
}

static int smb1360_initial_status(struct smb1360_battery *battery)
{
	int ret, i;

	for (i = IRQ_A; i < IRQ_COUNT; ++i) {
		ret = regmap_read(battery->regmap, IRQ_REG + i, &battery->irqstat[i]);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int smb1360_parse_batt_id(struct smb1360_battery *battery)
{
	int ret = 0, rpull = 0, vref = 0, val;
	s64 denom, batt_id_uv;
	struct device *dev = battery->dev;
	struct iio_channel *channel;

	channel = devm_iio_channel_get(dev, "batt-id");
	if (IS_ERR(channel)) {
		dev_err(dev, "error getting channel: %ld\n", PTR_ERR(channel));
		return PTR_ERR(channel);
	}

	ret = device_property_read_u32(dev, "qcom,profile-a-rid-kohm",
				       &battery->profile_rid[0]);
	if (ret < 0)
		return ret;

	ret = device_property_read_u32(dev, "qcom,profile-b-rid-kohm",
				       &battery->profile_rid[1]);
	if (ret < 0)
		return ret;

	ret = device_property_read_u32(dev, "qcom,batt-id-vref-uv", &vref);
	if (ret < 0)
		return ret;

	ret = device_property_read_u32(dev, "qcom,batt-id-rpullup-kohm", &rpull);
	if (ret < 0)
		return ret;

	/* read battery ID */
	ret = iio_read_channel_processed(channel, &val);
	iio_channel_release(channel);
	if (ret < 0)
		return ret;

	batt_id_uv = val * 1000;

	/* vadc not correct or batt id line grounded, report 0 kohms */
	if (batt_id_uv == 0)
		return 0;

	/* batt id connector might be open, return 0 kohms */
	denom = div64_s64(vref * 1000000LL, batt_id_uv) - 1000000LL;
	if (denom == 0)
		return 0;

	battery->connected_rid = div64_s64(rpull * 1000000LL + denom / 2, denom);

	return 0;
}

static int smb1360_parse_properties(struct smb1360_battery *battery)
{
	int ret = 0;
	struct device *dev = battery->dev;

	battery->shdn_after_pwroff = device_property_read_bool(dev, "qcom,shdn-after-pwroff");
	battery->rsense_10mohm = device_property_read_bool(dev, "qcom,rsense-10mhom");

	if (device_property_read_bool(dev, "qcom,batt-profile-select")) {
		ret = smb1360_parse_batt_id(battery);
		if (ret < 0)
			return ret;
	}

	if (device_property_read_bool(dev, "qcom,recharge-disabled") &&
	    device_property_read_bool(dev, "qcom,chg-inhibit-disabled")) {
		dev_err(battery->dev, "recharge: both disabled and mv set\n");
		return -EINVAL;
	}

	return 0;
}

static const struct regmap_config smb1360_battery_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
};

static void smb1360_external_power_changed(struct power_supply *psy)
{
	power_supply_changed(psy);
}

static const struct power_supply_desc smb1360_battery_desc = {
	.name			= "smb1360-battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.get_property		= smb1360_battery_get_property,
	.properties		= smb1360_battery_props,
	.num_properties		= ARRAY_SIZE(smb1360_battery_props),
	.external_power_changed	= smb1360_external_power_changed,
};

static int smb1360_probe(struct i2c_client *client)
{
	int ret;
	struct power_supply_config psy_cfg = {};
	struct device *dev = &client->dev;
	struct i2c_client *fg_client;
	struct smb1360_battery *battery;
	u16 fg_address;

	battery = devm_kzalloc(&client->dev, sizeof(*battery), GFP_KERNEL);
	if (!battery)
		return -EINVAL;

	battery->dev = dev;

	battery->regmap = devm_regmap_init_i2c(client,
					       &smb1360_battery_regmap_config);
	if (IS_ERR(battery->regmap)) {
		dev_err(&client->dev, "failed to init regmap\n");
		return -EINVAL;
	}

	INIT_DELAYED_WORK(&battery->delayed_init_work,
			  smb1360_delayed_init_work_fn);
	init_completion(&battery->fg_mem_access_granted);

	ret = regmap_read(battery->regmap, REVISION_CTRL_REG, &battery->revision);
	if (ret) {
		dev_err(battery->dev, "couldn't read revision: %d\n", ret);
		return ret;
	}

	fg_address = client->addr << 0x1;
	fg_address = (fg_address & ~FG_I2C_CFG_MASK) | FG_CFG_I2C_ADDR;
	fg_address = fg_address >> 0x1;

	fg_client = devm_i2c_new_dummy_device(dev, client->adapter, fg_address);
	if (IS_ERR(fg_client)) {
		dev_err(&client->dev, "failed to init fg i2c client\n");
		return -EINVAL;
	}

	battery->fg_regmap = devm_regmap_init_i2c(fg_client, &smb1360_battery_regmap_config);
	if (IS_ERR(battery->fg_regmap)) {
		dev_err(&client->dev, "failed to init fg regmap\n");
		return -EINVAL;
	}

	battery->revision &= DEVICE_REV_MASK;

	dev_info(battery->dev, "device revision: %d\n", battery->revision);

	ret = smb1360_parse_properties(battery);
	if (ret) {
		dev_err(&client->dev, "error parsing device tree: %d\n", ret);
		return ret;
	}

	device_init_wakeup(battery->dev, 1);
	i2c_set_clientdata(client, battery);

	ret = smb1360_hw_init(client);
	if (ret < 0) {
		dev_err(&client->dev, "unable to initialize hw: %d\n", ret);
		return ret;
	}

	ret = smb1360_initial_status(battery);
	if (ret < 0) {
		dev_err(&client->dev,
			"unable to determine init status: %d\n", ret);
		return ret;
	}

	battery->edev = devm_extcon_dev_allocate(dev, smb1360_usb_extcon_cable);
	if (IS_ERR(battery->edev))
		return PTR_ERR(battery->edev);

	ret = devm_extcon_dev_register(dev, battery->edev);
	if (ret < 0)
		return ret;

	extcon_set_state_sync(battery->edev, EXTCON_USB,
			      !(battery->irqstat[IRQ_E] & IRQ_E_USBIN_UV_BIT));

	ret = smb1360_register_vbus_regulator(battery);
	if (ret < 0)
		return ret;

	psy_cfg.drv_data = battery;
	battery->psy = devm_power_supply_register(&client->dev,
						  &smb1360_battery_desc,
						  &psy_cfg);
	if (IS_ERR(battery->psy)) {
		dev_err(&client->dev, "failed to register power supply\n");
		ret = PTR_ERR(battery->psy);
		return ret;
	}

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
						smb1360_bat_irq, IRQF_ONESHOT,
						NULL, battery);
		if (ret) {
			dev_err(&client->dev,
				"request irq %d failed\n", client->irq);
			return ret;
		}

		enable_irq_wake(client->irq);
	}

	schedule_delayed_work(&battery->delayed_init_work,
			      msecs_to_jiffies(SMB1360_POWERON_DELAY_MS));

	return 0;
}

static void smb1360_shutdown(struct i2c_client *client)
{
	int ret;
	struct smb1360_battery *battery = i2c_get_clientdata(client);

	ret = regulator_disable_regmap(battery->otg_vreg);
	if (ret)
		dev_err(battery->dev, "couldn't disable OTG: %d\n", ret);

	if (battery->shdn_after_pwroff) {
		ret = smb1360_poweroff(battery);
		if (ret)
			dev_err(battery->dev, "couldn't shutdown: %d\n", ret);
	}
}

static int smb1360_suspend(struct device *dev)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct smb1360_battery *battery = i2c_get_clientdata(client);

	ret = regmap_write(battery->regmap, IRQ_CFG_REG, IRQ_DCIN_UV_BIT
						| IRQ_BAT_HOT_COLD_SOFT_BIT
						| IRQ_HOT_COLD_HARD_BIT);
	if (ret < 0)
		dev_err(battery->dev, "couldn't set irq_cfg: %d\n", ret);

	ret = regmap_write(battery->regmap, IRQ2_CFG_REG, IRQ2_BATT_MISSING_BIT
						| IRQ2_VBAT_LOW_BIT
						| IRQ2_POWER_OK_BIT);
	if (ret < 0)
		dev_err(battery->dev, "couldn't set irq2_cfg: %d\n", ret);

	ret = regmap_write(battery->regmap, IRQ3_CFG_REG, SOC_FULL_BIT
					| SOC_MIN_BIT
					| SOC_EMPTY_BIT);
	if (ret < 0)
		dev_err(battery->dev, "couldn't set irq3_cfg: %d\n", ret);

	return 0;
}

static int smb1360_resume(struct device *dev)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct smb1360_battery *battery = i2c_get_clientdata(client);

	ret = smb1360_configure_irq(battery);
	if (ret)
		return ret;

	power_supply_changed(battery->psy);

	return 0;
}

static const struct dev_pm_ops smb1360_pm_ops = {
	.resume = smb1360_resume,
	.suspend = smb1360_suspend,
};

#ifdef CONFIG_OF
static const struct of_device_id smb1360_match_table[] = {
	{ .compatible = "qcom,smb1360" },
	{ },
};
MODULE_DEVICE_TABLE(of, smb1360_match_table);
#endif

static struct i2c_driver smb1360_driver = {
	.driver	= {
		.name = "smb1360-chg-fg",
		.of_match_table = of_match_ptr(smb1360_match_table),
		.pm = &smb1360_pm_ops,
	},
	.probe_new = smb1360_probe,
	.shutdown = smb1360_shutdown,
};

module_i2c_driver(smb1360_driver);

MODULE_DESCRIPTION("SMB1360 Charger and Fuel Gauge");
MODULE_LICENSE("GPL v2");
