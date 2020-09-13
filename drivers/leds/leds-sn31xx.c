// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for Si-En SN31XX LED IC
 * Structure taken from awinic2013 driver,
 * register writes taken from Alcatel/TCL downstream driver
 */

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>

#define SN31XX_MAX_LEDS	2

#define SN31XX_VDD_MIN_UV	2000000
#define SN31XX_VDD_MAX_UV	3300000
#define SN31XX_VIO_MIN_UV	1750000
#define SN31XX_VIO_MAX_UV	1950000

/* Registers */
#define SN31XX_RST1	0x00
#define SN31XX_MODE	0x02
#define SN31XX_IMAX	0x03
#define SN31XX_ISET	0x04
#define SN31XX_IUPD	0x07
#define SN31XX_T0	0x0a
#define SN31XX_T1T2	0x10 
#define SN31XX_T3T4	0x16
#define SN31XX_BLINK	0x1c
#define SN31XX_UNUSED	0x1d
#define SN31XX_RESET	0x2f

#define SN31XX_REG_MAX	0x2f

struct sn31xx_chip;

struct sn31xx_led {
	struct sn31xx_chip *chip;
	struct led_classdev cdev;
        struct gpio_desc *enable_gpiod;
        struct gpio_desc *enable2_gpiod;
	u32 num;
	unsigned int imax;
};

struct sn31xx_chip {
	struct mutex mutex; /* held when writing to registers */
	struct regulator *vio_regulator;
	struct i2c_client *client;
	struct sn31xx_led leds[SN31XX_MAX_LEDS];
	struct regmap *regmap;
	int num_leds;
	bool enabled;
};

static int sn31xx_chip_init(struct sn31xx_chip *chip)
{
	return regmap_write(chip->regmap, SN31XX_RESET, 0x0);
}

static void sn31xx_chip_disable(struct sn31xx_chip *chip)
{
	int ret;

	if (!chip->enabled)
		return;

	ret = regulator_disable(chip->vio_regulator);
	if (ret) {
		dev_err(&chip->client->dev,
			"Failed to disable regulator: %d\n", ret);
		return;
	}

	chip->enabled = false;
}

static int sn31xx_chip_enable(struct sn31xx_chip *chip)
{
	int ret;

	if (chip->enabled)
		return 0;

	ret = regulator_enable(chip->vio_regulator);
	if (ret) {
		dev_err(&chip->client->dev,
			"Failed to enable regulator: %d\n", ret);
		return ret;
	}
	chip->enabled = true;

	ret = sn31xx_chip_init(chip);
	if (ret)
		sn31xx_chip_disable(chip);

	msleep(80); //TODO: check

	return ret;
}

static bool sn31xx_chip_in_use(struct sn31xx_chip *chip)
{
	int i;

	for (i = 0; i < chip->num_leds; i++)
		if (chip->leds[i].cdev.brightness)
			return true;

	return false;
}

static const struct reg_sequence sn31xx_reg_reset[] = {
	{ SN31XX_RESET, 0x00 },
	{ SN31XX_RST1, 0x20 },
};

static const struct reg_sequence sn31xx_reg_imax[] = {
	{ SN31XX_IMAX, 0x08 },
	{ SN31XX_ISET, 0x0f },
	{ SN31XX_IUPD, 0xff },
};

static int sn31xx_brightness_set(struct led_classdev *cdev,
				 enum led_brightness brightness)
{
	struct sn31xx_led *led = container_of(cdev, struct sn31xx_led, cdev);
	int ret, num;

	mutex_lock(&led->chip->mutex);

	if (sn31xx_chip_in_use(led->chip)) {
		ret = sn31xx_chip_enable(led->chip);
		if (ret)
			goto error;
	}

	num = led->num;

	gpiod_set_value_cansleep(led->enable_gpiod, 1);

	if (brightness) {
		msleep(100); //TODO: keep ?
		ret = regmap_multi_reg_write(led->chip->regmap, sn31xx_reg_reset,
					     ARRAY_SIZE(sn31xx_reg_reset));
		ret = regmap_write(led->chip->regmap, SN31XX_MODE, 0x0);
		ret = regmap_multi_reg_write(led->chip->regmap, sn31xx_reg_imax,
					     ARRAY_SIZE(sn31xx_reg_imax));

	} else {
		ret = regmap_write(led->chip->regmap, SN31XX_RESET, 0x0);
		ret = regmap_write(led->chip->regmap, SN31XX_RST1, 0x1);
		sn31xx_chip_disable(led->chip);
		gpiod_set_value_cansleep(led->enable_gpiod, 0);

	}

error:
	mutex_unlock(&led->chip->mutex);
	return ret;
};

static int sn31xx_blink_set(struct led_classdev *cdev,
			    unsigned long *delay_on, unsigned long *delay_off)
{
	struct sn31xx_led *led = container_of(cdev, struct sn31xx_led, cdev);
	int ret, num;

	if (sn31xx_chip_in_use(led->chip)) {
		ret = sn31xx_chip_enable(led->chip);
		if (ret)
			goto error;
	}

	num = led->num;

	gpiod_set_value_cansleep(led->enable_gpiod, 1);

	mutex_lock(&led->chip->mutex);

	/* Blink off ? */
	if (!*delay_off && !*delay_on) {
		pr_info("sn31xx: don't blink\n");
		ret = regmap_multi_reg_write(led->chip->regmap, sn31xx_reg_reset,
					     ARRAY_SIZE(sn31xx_reg_reset));
		ret = regmap_write(led->chip->regmap, SN31XX_MODE, 0x00);
		ret = regmap_multi_reg_write(led->chip->regmap, sn31xx_reg_imax,
					     ARRAY_SIZE(sn31xx_reg_imax));

	} else { /* blink mode 2 in downstream */
		pr_info("sn31xx: blink mode 2\n");
		ret = regmap_write(led->chip->regmap, SN31XX_RESET, 0x00);
		ret = regmap_write(led->chip->regmap, SN31XX_MODE, 0x20);
		ret = regmap_write(led->chip->regmap, SN31XX_MODE, 0x20);
		ret = regmap_multi_reg_write(led->chip->regmap, sn31xx_reg_imax,
					     ARRAY_SIZE(sn31xx_reg_imax));
		ret = regmap_write(led->chip->regmap, SN31XX_T0, 0x00);
		ret = regmap_write(led->chip->regmap, SN31XX_T1T2, 0x60);
		ret = regmap_write(led->chip->regmap, SN31XX_T3T4, 0x06);
		ret = regmap_write(led->chip->regmap, SN31XX_BLINK, 0x00);
	}

	mutex_unlock(&led->chip->mutex);

	return 0;

error:
	return ret;
};

static const struct regmap_config sn31xx_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SN31XX_REG_MAX,
};

static int sn31xx_probe_dt(struct sn31xx_chip *chip)
{
	struct device_node *np = chip->client->dev.of_node, *child;
	int count, ret = 0, i = 0;
	struct sn31xx_led *led;

	count = of_get_child_count(np);
	if (!count || count > SN31XX_MAX_LEDS)
		return -EINVAL;

	for_each_available_child_of_node(np, child) {
		struct led_init_data init_data = {};
		u32 source;
		u32 imax;

		ret = of_property_read_u32(child, "reg", &source);
		if (ret != 0 || source >= SN31XX_MAX_LEDS) {
			dev_err(&chip->client->dev,
				"Couldn't read LED address: %d\n", ret);
			count--;
			continue;
		}

		led = &chip->leds[i];
		led->num = source;
		led->chip = chip;
		init_data.fwnode = of_fwnode_handle(child);

		if (!of_property_read_u32(child, "led-max-microamp", &imax)) {
			led->imax = min_t(u32, imax / 5000, 3);
		} else {
			led->imax = 1; // 5mA
			dev_info(&chip->client->dev,
				 "DT property led-max-microamp is missing\n");
		}

		of_property_read_string(child, "linux,default-trigger",
					&led->cdev.default_trigger);

		led->cdev.brightness_set_blocking = sn31xx_brightness_set;
		led->cdev.blink_set = sn31xx_blink_set;

		led->enable_gpiod = devm_fwnode_get_gpiod_from_child(
			&chip->client->dev, "enable", init_data.fwnode, GPIOD_OUT_HIGH, "enable1");
		if (IS_ERR(led->enable_gpiod))
			return PTR_ERR(led->enable_gpiod);

		/* This gpio is optional... TODO: fix driver not probing if not present in DT */
		led->enable2_gpiod = devm_fwnode_get_gpiod_from_child(
			&chip->client->dev, "enable2", init_data.fwnode, GPIOD_OUT_HIGH, "enable2");

		ret = devm_led_classdev_register_ext(&chip->client->dev,
						     &led->cdev, &init_data);
		if (ret < 0)
			return ret;

		i++;
	}

	if (!count)
		return -EINVAL;
	
	chip->num_leds = i;

	return 0;
}

static int sn31xx_probe(struct i2c_client *client)
{
	struct sn31xx_chip *chip;
	int ret;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	mutex_init(&chip->mutex);
	mutex_lock(&chip->mutex);

	chip->client = client;
	i2c_set_clientdata(client, chip);

	chip->regmap = devm_regmap_init_i2c(client, &sn31xx_regmap_config);
	if (IS_ERR(chip->regmap)) {
		ret = PTR_ERR(chip->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		goto error;
	}

	chip->vio_regulator = devm_regulator_get(&client->dev, "vio");
	ret = PTR_ERR_OR_ZERO(chip->vio_regulator);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&client->dev,
				"Failed to request regulator: %d\n", ret);
		goto error;
	}

	ret = regulator_enable(chip->vio_regulator);
	if (ret) {
		dev_err(&client->dev,
			"Failed to enable regulator: %d\n", ret);
		goto error;
	}

	ret = sn31xx_probe_dt(chip);
	if (ret < 0)
		goto error_reg;

	ret = regulator_disable(chip->vio_regulator);
	if (ret) {
		dev_err(&client->dev,
			"Failed to disable regulator: %d\n", ret);
		goto error;
	}

	mutex_unlock(&chip->mutex);

	return 0;

error_reg:
	regulator_disable(chip->vio_regulator);

error:
	mutex_destroy(&chip->mutex);
	return ret;
}

static int sn31xx_remove(struct i2c_client *client)
{
	struct sn31xx_chip *chip = i2c_get_clientdata(client);

	sn31xx_chip_disable(chip);

	mutex_destroy(&chip->mutex);

	return 0;
}

static const struct of_device_id sn31xx_match_table[] = {
	{ .compatible = "si-en,sn31xx", },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, sn31xx_match_table);

static struct i2c_driver sn31xx_driver = {
	.driver = {
		.name = "leds-sn31xx",
		.of_match_table = of_match_ptr(sn31xx_match_table),
	},
	.probe_new = sn31xx_probe,
	.remove = sn31xx_remove,
};

module_i2c_driver(sn31xx_driver);

MODULE_AUTHOR("Vincent Knecht <vincent.knecht@mailoo.org");
MODULE_DESCRIPTION("SN31XX LED driver");
MODULE_LICENSE("GPL v2");
