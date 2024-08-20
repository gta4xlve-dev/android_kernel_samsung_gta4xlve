/*
 * s2mu107.c - mfd core driver for the s2mu107
 *
 * Copyright (C) 2019 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/mfd/samsung/s2mu107.h>
#include <linux/of_gpio.h>

static struct mfd_cell s2mu107_devs[] = {
#if defined(CONFIG_PM_S2MU107)
	{ .name = "s2mu107-powermeter", },
#endif
#if defined(CONFIG_CHARGER_S2MU107)
	{ .name = "s2mu107-switching-charger", },
#endif
#if defined(CONFIG_LEDS_S2MU107_FLASH)
	{ .name = "leds-s2mu107", },
#endif
#if defined(CONFIG_MUIC_S2MU107)
	{ .name = "s2mu107-muic", },
#endif
#if defined(CONFIG_HV_MUIC_S2MU107_AFC)
	{ .name = "s2mu107-afc", },
#endif
#if defined(CONFIG_CHARGER_S2MU107_DIRECT)
	{ .name = "s2mu107-direct-charger", },
#endif
};

int s2mu107_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct s2mu107_dev *s2mu107 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&s2mu107->i2c_lock);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	mutex_unlock(&s2mu107->i2c_lock);
	if (ret < 0) {
		pr_err("%s:%s reg(0x%x), ret(%d)\n", MFD_DEV_NAME,
			__func__, reg, ret);
		return ret;
	}

	ret &= 0xff;
	*dest = ret;
	return 0;
}
EXPORT_SYMBOL_GPL(s2mu107_read_reg);

int s2mu107_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mu107_dev *s2mu107 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&s2mu107->i2c_lock);
	ret = i2c_smbus_read_i2c_block_data(i2c, reg, count, buf);
	mutex_unlock(&s2mu107->i2c_lock);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(s2mu107_bulk_read);

int s2mu107_read_word(struct i2c_client *i2c, u8 reg)
{
	struct s2mu107_dev *s2mu107 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&s2mu107->i2c_lock);
	ret = i2c_smbus_read_word_data(i2c, reg);
	mutex_unlock(&s2mu107->i2c_lock);
	if (ret < 0)
		return ret;

	return ret;
}
EXPORT_SYMBOL_GPL(s2mu107_read_word);

int s2mu107_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct s2mu107_dev *s2mu107 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&s2mu107->i2c_lock);
	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	mutex_unlock(&s2mu107->i2c_lock);
	if (ret < 0)
		pr_err("%s:%s reg(0x%x), ret(%d)\n",
			MFD_DEV_NAME, __func__, reg, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(s2mu107_write_reg);

int s2mu107_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mu107_dev *s2mu107 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&s2mu107->i2c_lock);
	ret = i2c_smbus_write_i2c_block_data(i2c, reg, count, buf);
	mutex_unlock(&s2mu107->i2c_lock);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(s2mu107_bulk_write);

int s2mu107_write_word(struct i2c_client *i2c, u8 reg, u16 value)
{
	struct s2mu107_dev *s2mu107 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&s2mu107->i2c_lock);
	ret = i2c_smbus_write_word_data(i2c, reg, value);
	mutex_unlock(&s2mu107->i2c_lock);
	if (ret < 0)
		return ret;
	return 0;
}
EXPORT_SYMBOL_GPL(s2mu107_write_word);

int s2mu107_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct s2mu107_dev *s2mu107 = i2c_get_clientdata(i2c);
	int ret;
	u8 old_val, new_val;

	mutex_lock(&s2mu107->i2c_lock);
	/* read */
	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret < 0) {
		pr_err("%s:%s reg(0x%x), ret(%d)\n",
			MFD_DEV_NAME, __func__, reg, ret);
		goto err_update_reg;
	}

	/* update value */
	old_val = ret & 0xff;
	new_val = (val & mask) | (old_val & (~mask));

	/* write */
	ret = i2c_smbus_write_byte_data(i2c, reg, new_val);
	if (ret < 0) {
		pr_err("%s:%s reg(0x%x), ret(%d)\n",
			MFD_DEV_NAME, __func__, reg, ret);
	}

err_update_reg:
	mutex_unlock(&s2mu107->i2c_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mu107_update_reg);

#if defined(CONFIG_OF)
static int of_s2mu107_dt(struct device *dev,
			 struct s2mu107_platform_data *pdata)
{
	struct device_node *np_s2mu107 = dev->of_node;
	int ret = 0;

	if (!np_s2mu107)
		return -EINVAL;

	ret = of_get_named_gpio(np_s2mu107, "s2mu107,irq-gpio", 0);
	if (ret < 0)
		return ret;
	else
		pdata->irq_gpio = ret;

	pdata->wakeup = of_property_read_bool(np_s2mu107, "s2mu107,wakeup");

	pr_debug("%s: irq-gpio: %u\n", __func__, pdata->irq_gpio);

	return 0;
}
#else
static int of_s2mu107_dt(struct device *dev,
			 struct max77834_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

static int s2mu107_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *dev_id)
{
	struct s2mu107_dev *s2mu107;
	struct s2mu107_platform_data *pdata = i2c->dev.platform_data;

	int ret = 0;
	u8 temp = 0;

	pr_info("%s start\n", __func__);

	s2mu107 = devm_kzalloc(&i2c->dev, sizeof(struct s2mu107_dev),
			       GFP_KERNEL);
	if (!s2mu107) {
		dev_err(&i2c->dev, "%s: Failed to alloc mem for s2mu107\n",
				__func__);
		return -ENOMEM;
	}

	if (i2c->dev.of_node) {
		pdata = devm_kzalloc(&i2c->dev,
				     sizeof(struct s2mu107_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			dev_err(&i2c->dev, "Failed to allocate memory\n");
			ret = -ENOMEM;
			goto err;
		}

		ret = of_s2mu107_dt(&i2c->dev, pdata);
		if (ret < 0) {
			dev_err(&i2c->dev, "Failed to get device of_node\n");
			goto err;
		}

		i2c->dev.platform_data = pdata;
	} else
		pdata = i2c->dev.platform_data;

	s2mu107->dev = &i2c->dev;
	s2mu107->i2c = i2c;
	s2mu107->irq = i2c->irq;
	if (pdata) {
		s2mu107->pdata = pdata;

		pdata->irq_base = irq_alloc_descs(-1, 0, S2MU107_IRQ_NR, -1);
		if (pdata->irq_base < 0) {
			pr_err("%s:%s irq_alloc_descs Fail! ret(%d)\n",
				MFD_DEV_NAME, __func__, pdata->irq_base);
			ret = -EINVAL;
			goto err;
		} else
			s2mu107->irq_base = pdata->irq_base;

		s2mu107->irq_gpio = pdata->irq_gpio;
		s2mu107->wakeup = pdata->wakeup;
	} else {
		ret = -EINVAL;
		goto err;
	}
	mutex_init(&s2mu107->i2c_lock);

	i2c_set_clientdata(i2c, s2mu107);

	s2mu107_read_reg(s2mu107->i2c, S2MU107_REG_ESREV_NUM, &temp);
	if (temp < 0)
		pr_err("[s2mu107 mfd] %s : i2c read error\n", __func__);

	s2mu107->pmic_rev = temp & S2MU107_REG_REV_MASK;
	s2mu107->pmic_es = temp & S2MU107_REG_ES_MASK;
	pr_err("%s : rev=0x%x, es=0x%x\n", __func__, s2mu107->pmic_rev, s2mu107->pmic_es);

	/* I2C enable for AFC, MUIC, and Powermeter */
	s2mu107->muic = i2c_new_dummy(i2c->adapter, I2C_ADDR_7C_SLAVE);
	i2c_set_clientdata(s2mu107->muic, s2mu107);

	/* I2C enable for Switching Charger and Direct Charger  */
	s2mu107->chg = i2c_new_dummy(i2c->adapter, I2C_ADDR_7A_SLAVE);
	i2c_set_clientdata(s2mu107->chg, s2mu107);

	ret = s2mu107_irq_init(s2mu107);

	if (ret < 0)
		goto err_irq_init;

	ret = mfd_add_devices(s2mu107->dev, -1, s2mu107_devs,
			      ARRAY_SIZE(s2mu107_devs), NULL, 0, NULL);
	if (ret < 0)
		goto err_mfd;

	device_init_wakeup(s2mu107->dev, pdata->wakeup);

	return ret;

err_mfd:
	mfd_remove_devices(s2mu107->dev);
err_irq_init:
	mutex_destroy(&s2mu107->i2c_lock);
	i2c_unregister_device(s2mu107->i2c);
	i2c_set_clientdata(i2c, NULL);
err:
	kfree(s2mu107);

	return ret;
}

static int s2mu107_i2c_remove(struct i2c_client *i2c)
{
	struct s2mu107_dev *s2mu107 = i2c_get_clientdata(i2c);

	mfd_remove_devices(s2mu107->dev);
	i2c_unregister_device(s2mu107->i2c);
	i2c_set_clientdata(i2c, NULL);

	return 0;
}

static const struct i2c_device_id s2mu107_i2c_id[] = {
	{ MFD_DEV_NAME, TYPE_S2MU107 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, s2mu107_i2c_id);

#if defined(CONFIG_OF)
static struct of_device_id s2mu107_i2c_dt_ids[] = {
	{.compatible = "samsung,s2mu107mfd"},
	{ },
};
#endif /* CONFIG_OF */

#if defined(CONFIG_PM)
static int s2mu107_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct s2mu107_dev *s2mu107 = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev))
		enable_irq_wake(s2mu107->irq);

	disable_irq(s2mu107->irq);

	return 0;
}

static int s2mu107_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct s2mu107_dev *s2mu107 = i2c_get_clientdata(i2c);

	pr_debug("%s:%s\n", MFD_DEV_NAME, __func__);

	if (device_may_wakeup(dev))
		disable_irq_wake(s2mu107->irq);

	enable_irq(s2mu107->irq);

	return 0;
}
#else
#define s2mu107_suspend	NULL
#define s2mu107_resume	NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops s2mu107_pm = {
	.suspend = s2mu107_suspend,
	.resume = s2mu107_resume,
};

static struct i2c_driver s2mu107_i2c_driver = {
	.driver		= {
		.name	= MFD_DEV_NAME,
		.owner	= THIS_MODULE,
#if defined(CONFIG_PM)
		.pm	= &s2mu107_pm,
#endif /* CONFIG_PM */
#if defined(CONFIG_OF)
		.of_match_table	= s2mu107_i2c_dt_ids,
#endif /* CONFIG_OF */
	},
	.probe		= s2mu107_i2c_probe,
	.remove		= s2mu107_i2c_remove,
	.id_table	= s2mu107_i2c_id,
};

static int __init s2mu107_i2c_init(void)
{
	pr_info("%s:%s\n", MFD_DEV_NAME, __func__);
	return i2c_add_driver(&s2mu107_i2c_driver);
}
subsys_initcall(s2mu107_i2c_init);

static void __exit s2mu107_i2c_exit(void)
{
	i2c_del_driver(&s2mu107_i2c_driver);
}
module_exit(s2mu107_i2c_exit);

MODULE_DESCRIPTION("s2mu107 multi-function core driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
