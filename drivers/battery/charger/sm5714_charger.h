/*
 * sm5714-charger.h - header file of SM5714 Charger device driver
 *
 * Copyright (C) 2017 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __SM5714_CHARGER_H__
#define __SM5714_CHARGER_H__

#include <linux/mfd/sm/sm5714/sm5714.h>
#include <linux/mfd/sm/sm5714/sm5714-private.h>
#include "../common/sec_battery.h"

enum {
	CHIP_ID = 0,
	DATA = 1,
	DATA_1 = 2,
	EN_BYPASS_MODE = 3,
};

ssize_t sm5714_chg_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sm5714_chg_store_attrs(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count);

#define SM5714_CHARGER_ATTR(_name)				\
{							                    \
	.attr = {.name = #_name, .mode = 0664},	    \
	.show = sm5714_chg_show_attrs,			    \
	.store = sm5714_chg_store_attrs,			\
}

enum {
	AICL_TH_V_4_3   = 0x0,
	AICL_TH_V_4_4   = 0x1,
	AICL_TH_V_4_5   = 0x2,
	AICL_TH_V_4_6   = 0x3,
};

enum {
	DISCHG_LIMIT_C_5_4   = 0x0,
	DISCHG_LIMIT_C_6_0   = 0x1,
	DISCHG_LIMIT_C_6_6   = 0x2,
	DISCHG_LIMIT_C_7_2   = 0x3,
	DISCHG_LIMIT_C_7_8   = 0x4,
	DISCHG_LIMIT_C_8_4   = 0x5,
	DISCHG_LIMIT_C_9_0   = 0x6,
	DISCHG_LIMIT_DISABLE = 0x7,
};

enum {
	DISCHG_TIME_MS_24    = 0x0,
	DISCHG_TIME_MS_100   = 0x1,
	DISCHG_TIME_MS_200   = 0x2,
	DISCHG_TIME_MS_400   = 0x3,
};

enum {
	AUTO_SHIP_MODE_VREF_V_2_6	= 0x0,
	AUTO_SHIP_MODE_VREF_V_2_8	= 0x1,
	AUTO_SHIP_MODE_VREF_V_3_0	= 0x2,
	AUTO_SHIP_MODE_VREF_V_3_2	= 0x3,
};

enum {
	AUTO_SHIP_MODE_TIME_S_0_5	= 0x0,
	AUTO_SHIP_MODE_TIME_S_1_0	= 0x1,
	AUTO_SHIP_MODE_TIME_S_2_0	= 0x2,
	AUTO_SHIP_MODE_TIME_S_4_0	= 0x3,
};



enum {
	WDT_TIME_S_30   = 0x0,
	WDT_TIME_S_60   = 0x1,
	WDT_TIME_S_90   = 0x2,
	WDT_TIME_S_120  = 0x3,
};

enum {
	TOPOFF_TIME_M_10 = 0x0,
	TOPOFF_TIME_M_20 = 0x1,
	TOPOFF_TIME_M_30 = 0x2,
	TOPOFF_TIME_M_45 = 0x3,
};


struct sm5714_charger_platform_data {
	int chg_float_voltage;
	int chg_ocp_current;
};

#define REDUCE_CURRENT_STEP						100
#define MINIMUM_INPUT_CURRENT					300
#define SLOW_CHARGING_CURRENT_STANDARD          400

struct sm5714_charger_data {
	struct device *dev;
	struct i2c_client *i2c;
	struct mutex charger_mutex;

	struct sm5714_charger_platform_data *pdata;
	struct power_supply	*psy_chg;
	struct power_supply	*psy_otg;

	int read_reg;

	int status;
	int cable_type;
	int input_current;
	int charging_current;
	int topoff_current;
	int float_voltage;
	int charge_mode;
	int unhealth_cnt;
	bool is_charging;
	bool otg_on;

	/* sm5714 Charger-IRQs */
	int irq_vbuspok;
	int irq_aicl;
	int irq_aicl_enabled;
	int irq_vsysovp;
	int irq_otgfail;

	int irq_batovp;
	int irq_done;
	int irq_vbusuvlo;

	int pmic_rev;

	/* for slow-rate-charging noti */
	bool slow_rate_chg_mode;
	struct workqueue_struct *wqueue;
	struct delayed_work aicl_work;
	struct wakeup_source *aicl_ws;
};

#endif  /* __SM5714_CHARGER_H__ */
