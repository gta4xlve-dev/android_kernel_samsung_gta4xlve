/*
 *  Copyright (C) 2021,Imagis Technology Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/pm_wakeup.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/power_supply.h>
#include <linux/sensor/sensors_core.h>
#include <linux/vmalloc.h>
#if IS_ENABLED(CONFIG_CCIC_NOTIFIER) || IS_ENABLED(CONFIG_PDIC_NOTIFIER)
#include <linux/usb/typec/common/pdic_notifier.h>
#endif
#if IS_ENABLED(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
#include <linux/usb/typec/manager/usb_typec_manager_notifier.h>
#endif
#if IS_ENABLED(CONFIG_HALL_NOTIFIER)
#include <linux/hall/hall_ic_notifier.h>
#define HALL_NAME		"hall"
#define HALL_CERT_NAME		"certify_hall"
#define HALL_FLIP_NAME		"flip"
#define HALL_ATTACH		1
#define HALL_DETACH		0
#else
#define HALLIC_PATH		"/sys/class/sec/hall_ic/hall_detect"
#define WACOM_HALLIC_PATH	"/sys/class/sec/hall_ic/hall_wacom_detect"
#endif

#include "isg6320_reg.h"

#define VENDOR_NAME			 "IMAGIS"

#define ISG6320_MODE_SLEEP	  0
#define ISG6320_MODE_NORMAL	 1
#define ISG6320_DIFF_AVR_CNT	10
#define ISG6320_DISPLAY_TIME	30
#define ISG6320_TAG			 "[ISG6320]"

#define ISG6320_INIT_DELAYEDWORK
#define GRIP_LOG_TIME			5

#define SHCEDULE_INTERVAL       2000  // 2 sec * 5 = 10 sec
#define SHCEDULE_INTERVAL_MAX   20000 // 20 sec * 5 = 100 sec

#ifdef CONFIG_USE_MULTI_CHANNEL
#if !IS_ENABLED(CONFIG_SAMSUNG_PRODUCT_SHIP)
#define SENSOR_ATTR_SIZE 60
#else
#define SENSOR_ATTR_SIZE 45
#endif
#endif

#define TYPE_USB   1
#define TYPE_HALL  2
#define TYPE_BOOT  3
#define TYPE_FORCE 4
#define TYPE_COVER 5

#define COUNTRY_KOR 1
#define COUNTRY_ETC 0

#pragma pack(1)
typedef struct {
	char cmd;
	u8 addr;
	u8 val;
} direct_info;
#pragma pack()

#ifdef CONFIG_USE_MULTI_CHANNEL
struct multi_channel {
	int state_a;
	int state_b;

	u32 cdc_b;
	u32 base_b;
	s32 diff_b;
	s32 max_diff_b;
	s32 max_normal_diff_b;

	int diff_cnt_b;
	int diff_sum_b;
	int diff_avg_b;
	int cdc_sum_b;
	int cdc_avg_b;

	u32 cfcal_th_b;
	u16 normal_th_b;
	u16 fine_coarse_b;
	int is_unknown_mode;
	bool first_working;
};
#endif

struct isg6320_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct input_dev *noti_input_dev;
	struct device *dev;
	struct work_struct irq_work;
	struct work_struct cfcal_work;
	struct delayed_work cal_work;
#ifdef ISG6320_INIT_DELAYEDWORK
	struct delayed_work init_work;
#endif
	struct wakeup_source *grip_ws;
	struct mutex lock;
#if IS_ENABLED(CONFIG_CCIC_NOTIFIER) || IS_ENABLED(CONFIG_PDIC_NOTIFIER)
	struct notifier_block cpuidle_ccic_nb;
	int pdic_status;
	int pdic_pre_attach;
#endif
#if IS_ENABLED(CONFIG_HALL_NOTIFIER)
	struct notifier_block hall_nb;
#else
	int hall_flag;
	int wacom_hall_flag;
	unsigned char hall_ic[6];
	unsigned char wacom_hall_ic[6];
#endif
#if IS_ENABLED(CONFIG_FLIP_COVER_DETECTOR_NOTIFIER)
	struct notifier_block fcd_nb;
#endif
	direct_info direct;

	int gpio_int;
	int enable;
	int noti_enable;
	int country_code;
	int initialized;
	int reg_size;

	int irq_count;
	u16 schedule_time;
	u8 abnormal_mode;
	u8 debug_cnt;

	int pre_attach;

	int state;

	u32 multi_use;
	u32 cdc;
	u32 base;
	s32 diff;
	s32 max_diff;
	s32 max_normal_diff;

	int diff_cnt;
	int diff_sum;
	int diff_avg;
	int cdc_sum;
	int cdc_avg;

	u32 cfcal_th;
	u16 normal_th;
	u16 fine_coarse;

#ifdef CONFIG_USE_MULTI_CHANNEL
	struct multi_channel *mul_ch;
#endif

	u8 setup_reg[320];

	u8 ic_num;

	bool skip_data;
	bool setup_reg_exist;
	bool in_suspend;

#if !IS_ENABLED(CONFIG_SAMSUNG_PRODUCT_SHIP)
	u32 debug_cdc[4];
	u32 debug_base[2];
	s32 debug_diff[4];

	int irq_debug_size;
	u8 irq_debug_addr;

	u8 freq_step;
	u8 freq_value;
#endif
#if defined(CONFIG_TABLET_MODEL_CONCEPT)
	u8 lsum_a;
	u8 lsum_b;
#endif
#if !defined(CONFIG_SEC_FACTORY) && defined(CONFIG_SUPPORT_MCC_THRESHOLD_CHANGE)
	struct work_struct mcc_work;
	int mcc;
	u8 default_threshold;
	u8 default_hysteresis;
	u8 mcc_threshold;
	u8 mcc_hysteresis;
#endif
	int ldo_en;
	int otg_attach_state;
	int is_unknown_mode;
	int motion;
	bool first_working;
};

static void isg6320_set_debug_work(struct isg6320_data *data, bool enable,
					unsigned int delay_ms);

static int check_hallic_state(char *file_path, unsigned char hall_ic_status[])
{
	int ret = 0;
	mm_segment_t old_fs;
	struct file *filep;
	u8 hall_sysfs[5];

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	filep = filp_open(file_path, O_RDONLY, 0666);
	if (IS_ERR(filep)) {
		ret = PTR_ERR(filep);
		set_fs(old_fs);
		goto exit;
	}

	ret = filep->f_op->read(filep, hall_sysfs,
				 sizeof(hall_sysfs), &filep->f_pos);

	if (ret != sizeof(hall_sysfs)) {
		pr_err("%s %s: file read fail %d\n", ISG6320_TAG, __func__, ret);
		ret = -EIO;
	} else {
		strncpy(hall_ic_status, hall_sysfs, sizeof(hall_sysfs));
	}

	filp_close(filep, current->files);
	set_fs(old_fs);

exit:
	return ret;
}

static int isg6320_i2c_write(struct isg6320_data *data, u8 cmd, u8 val)
{
	int ret;
	u8 buf[2];
	struct i2c_msg msg;

	buf[0] = cmd;
	buf[1] = val;

	msg.addr = data->client->addr;
	msg.flags = 0; /*I2C_M_WR*/
	msg.len = 2;
	msg.buf = buf;

	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret < 0)
		pr_err("[GRIP_%d] %s fail(%d)\n", data->ic_num, __func__,
			ret);

	return ret;
}

static int isg6320_i2c_read(struct isg6320_data *data, u8 cmd, u8 *val,
				 int len)
{
	int ret;
	struct i2c_msg msgs[2] = {
		{
			.addr = data->client->addr,
			.flags = 0,
			.len = sizeof(cmd),
			.buf = &cmd,
		},
		{
			.addr = data->client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = val,
		},
	};

	ret = i2c_transfer(data->client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("[GRIP_%d] %s fail(%d)\n", data->ic_num, __func__,
			ret);

	return ret;
}

static int isg6320_reset(struct isg6320_data *data)
{
	int ret = 0;
	int cnt = 0;
	u8 val;

	pr_info("[GRIP_%d] %s\n", data->ic_num, __func__);

	if (data->initialized == OFF)
		usleep_range(5000, 5100);

	ret = isg6320_i2c_read(data, ISG6320_IRQSRC_REG, &val, 1);
	if (ret < 0) {
		pr_err("[GRIP_%d] irq to high failed(%d)\n", data->ic_num, ret);
		return ret;
	}

	while (gpio_get_value_cansleep(data->gpio_int) == 0 && cnt++ < 10)
		usleep_range(5000, 5100);

	if (cnt >= 10)
		pr_err("[GRIP_%d] wait irq to high failed\n", data->ic_num);

	ret = isg6320_i2c_write(data, ISG6320_PROTECT_REG, ISG6320_PRT_VALUE);
	if (ret < 0) {
		pr_err("[GRIP_%d] unlock protect failed(%d)\n", data->ic_num, ret);
		return ret;
	}

	ret = isg6320_i2c_write(data, ISG6320_SOFTRESET_REG, ISG6320_RST_VALUE);
	if (ret < 0) {
		pr_err("[GRIP_%d] soft reset failed(%d)\n", data->ic_num, ret);
		return ret;
	}

	usleep_range(1000, 1100);

	return ret;
}

static int isg6320_force_calibration(struct isg6320_data *data)
{
	int ret = 0;
	int retry = 3;

	isg6320_set_debug_work(data, OFF, 0);
	mutex_lock(&data->lock);
#if IS_ENABLED(CONFIG_CCIC_NOTIFIER) || IS_ENABLED(CONFIG_PDIC_NOTIFIER)
	if (data->pdic_status == ON) {
		if (data->initialized == ON) {
#endif
#if defined(CONFIG_TABLET_MODEL_CONCEPT)
			isg6320_i2c_write(data, ISG6320_SCANCTRL1_REG, ISG6320_DFE_ENABLE);
			usleep_range(10000, 10010);
			isg6320_i2c_write(data, ISG6320_SCANCTRL1_REG, ISG6320_SCAN_STOP);
			usleep_range(10000, 10010);
			isg6320_i2c_write(data, ISG6320_A_PROXCTL4_REG, 0xFF);
			isg6320_i2c_write(data, ISG6320_A_LSUM_TYPE_REG, 0x10);
#ifdef CONFIG_USE_MULTI_CHANNEL
			if (data->multi_use) {
				isg6320_i2c_write(data, ISG6320_B_PROXCTL4_REG, 0xFF);
				isg6320_i2c_write(data, ISG6320_B_LSUM_TYPE_REG, 0x10);
			}
#endif
#endif
#if IS_ENABLED(CONFIG_CCIC_NOTIFIER) || IS_ENABLED(CONFIG_PDIC_NOTIFIER)
		}
	}
#endif
#if IS_ENABLED(CONFIG_CCIC_NOTIFIER) || IS_ENABLED(CONFIG_PDIC_NOTIFIER)
	else if (data->pdic_status == OFF) {
		if (data->initialized == ON) {
#endif
#if defined(CONFIG_TABLET_MODEL_CONCEPT)
			isg6320_i2c_write(data, ISG6320_SCANCTRL1_REG, ISG6320_DFE_ENABLE);
			usleep_range(10000, 10010);
			isg6320_i2c_write(data, ISG6320_SCANCTRL1_REG, ISG6320_SCAN_STOP);
			usleep_range(10000, 10010);
			isg6320_i2c_write(data, ISG6320_A_PROXCTL4_REG, 0x58);
			isg6320_i2c_write(data, ISG6320_A_LSUM_TYPE_REG, data->lsum_a);
#ifdef CONFIG_USE_MULTI_CHANNEL
			if (data->multi_use) {
				isg6320_i2c_write(data, ISG6320_B_PROXCTL4_REG, 0x58);
				isg6320_i2c_write(data, ISG6320_B_LSUM_TYPE_REG, data->lsum_b);
			}
#endif
#endif
#if IS_ENABLED(CONFIG_CCIC_NOTIFIER) || IS_ENABLED(CONFIG_PDIC_NOTIFIER)
		}
	}
#endif
	pr_info("[GRIP_%d] %s\n", data->ic_num, __func__);

	while (retry--) {
		u8 val = 0;

		ret = 0;

		isg6320_i2c_write(data, ISG6320_SCANCTRL1_REG, ISG6320_DFE_ENABLE);
		usleep_range(10000, 10010);
		isg6320_i2c_write(data, ISG6320_SCANCTRL1_REG, ISG6320_SCAN_STOP);
		usleep_range(10000, 10010);
#if defined(CONFIG_TABLET_MODEL_CONCEPT)
		isg6320_i2c_write(data, ISG6320_PROTECT_REG, ISG6320_RST_VALUE);
		isg6320_i2c_write(data, ISG6320_RESETCON_REG, ISG6320_DFE_RESET_ON);
		usleep_range(10000, 10010);
		isg6320_i2c_write(data, ISG6320_PROTECT_REG, ISG6320_RST_VALUE);
		isg6320_i2c_write(data, ISG6320_RESETCON_REG, ISG6320_DFE_RESET_OFF);
		usleep_range(10000, 10010);
#endif
		isg6320_i2c_write(data, ISG6320_SCANCTRL1_REG, ISG6320_CFCAL_START);
		msleep(450);

		isg6320_i2c_read(data, ISG6320_CFCAL_RTN_REG, &val, 1);
		pr_info("[GRIP_%d] reg read : %02x\n", data->ic_num, val);
		if (!(val & ISG6320_CAL_RTN_A_MASK)) {
			pr_err("[GRIP_%d] fail calibration(%d)\n", data->ic_num, retry);
			ret = -EAGAIN;
		}
#ifdef CONFIG_USE_MULTI_CHANNEL
		if (data->multi_use) {
			if (!(val & ISG6320_CAL_RTN_B_MASK)) {
				pr_err("[GRIP_%d] [B] fail calibration(%d)\n", data->ic_num, retry);
				ret = -EAGAIN;
			}
		}
#endif
		if (!ret)
			break;
	}

	mutex_unlock(&data->lock);
	if (!data->in_suspend)
		isg6320_set_debug_work(data, ON, SHCEDULE_INTERVAL + (data->ic_num << 3));
	return ret;
}

static inline unsigned char str2int(unsigned char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';

	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;

	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;

	return 0;
}

#if !IS_ENABLED(CONFIG_SAMSUNG_PRODUCT_SHIP)
static void isg6320_irq_debug(struct isg6320_data *data)
{
	int ret;
	u8 *buf8;

	buf8 = kzalloc(data->irq_debug_size, GFP_KERNEL);
	if (buf8) {
		int i = 0;
		pr_info("[GRIP_%d] Intr_debug1 (0x%02X)\n", data->ic_num,
			data->irq_debug_addr);
		ret = isg6320_i2c_read(data, data->irq_debug_addr, buf8,
					data->irq_debug_size);
		if (ret < 0) {
			pr_err("[GRIP_%d] fail to read irq_debug_addr(%d)\n",
				data->ic_num, ret);
			kfree(buf8);
			return;
		}

		for (; i < data->irq_debug_size; i++)
			pr_info("[GRIP_%d] \t%02X\n", data->ic_num, buf8[i]);
		kfree(buf8);
	}
}
#endif

static int isg6320_get_raw_data(struct isg6320_data *data, bool log_print)
{
	int ret = 0;
	u8 buf[4];
	u16 cpbuf;
	u32 temp, temp1;
	bool valid[2] = {true, true};

	mutex_lock(&data->lock);
	ret = isg6320_i2c_read(data, ISG6320_CDC16_TA_H_REG, buf, sizeof(buf));

	if (ret < 0) {
		pr_err("[GRIP_%d] fail to get data\n", data->ic_num);
	} else {
		temp = ((u32)buf[0] << 8) | (u32)buf[1];
		temp1 = ((u32)buf[2] << 8) | (u32)buf[3];

		if (!((temp != 0) && (temp < 0x7FFF))) {
			valid[0] = false;
			pr_err("[GRIP_%d] cdc is invalid(%04x)\n", data->ic_num, temp);
		}

		if (!((temp1 != 0) && (temp1 < 0x7FFF))) {
			valid[1] = false;
			pr_err("[GRIP_%d] base is invalid(%04x)\n", data->ic_num, temp1);
		}

		if (valid[0] && valid[1]) {
			data->cdc = temp;
			data->base = temp1;
		}

		data->diff = (s32)data->cdc - (s32)data->base;

		ret = isg6320_i2c_read(data, ISG6320_A_COARSE_OUT_REG,
							   (u8 *)&cpbuf, 2);
		if (ret < 0)
			pr_err("[GRIP_%d] fail to get capMain\n", data->ic_num);
		else
			data->fine_coarse = cpbuf;
	}

#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->multi_use) {
		ret = isg6320_i2c_read(data, ISG6320_CDC16_TB_H_REG, buf, sizeof(buf));

		if (ret < 0) {
			pr_err("[GRIP_%d] [B] fail to get data\n", data->ic_num);
		} else {
			valid[0] = valid[1] = true;
			temp = ((u32)buf[0] << 8) | (u32)buf[1];
			temp1 = ((u32)buf[2] << 8) | (u32)buf[3];

			if (!((temp != 0) && (temp < 0x7FFF))) {
				valid[0] = false;
				pr_err("[GRIP_%d] [B] cdc is invalid(%04x)\n", data->ic_num, temp);
			}

			if (!((temp1 != 0) && (temp1 < 0x7FFF))) {
				valid[1] = false;
				pr_err("[GRIP_%d] [B] base is invalid(%04x)\n", data->ic_num, temp1);
			}

			if (valid[0] && valid[1]) {
				data->mul_ch->cdc_b = temp;
				data->mul_ch->base_b = temp1;
			}

			data->mul_ch->diff_b = (s32)data->mul_ch->cdc_b - (s32)data->mul_ch->base_b;

			ret = isg6320_i2c_read(data, ISG6320_B_COARSE_OUT_REG,
								   (u8 *)&cpbuf, 2);
			if (ret < 0)
				pr_err("[GRIP_%d] [B] fail to get capMain\n", data->ic_num);
			else
				data->mul_ch->fine_coarse_b = cpbuf;
		}
	}
#endif
	mutex_unlock(&data->lock);

	if (log_print || (data->debug_cnt >= GRIP_LOG_TIME)) {
		pr_info("[GRIP_%d] CapMain: %d%02d, cdc: %d, baseline:%d, diff:%d, "
				"skip_data:%d\n",
				data->ic_num, (data->fine_coarse & 0xFF),
				((data->fine_coarse >> 8) & 0x3F), data->cdc, data->base,
				data->diff, data->skip_data);
#ifdef CONFIG_USE_MULTI_CHANNEL
		if (data->multi_use) {
			pr_info("[GRIP_%d] [B] CapMain: %d%02d, cdc: %d, baseline:%d, diff:%d, "
					"skip_data:%d\n",
					data->ic_num, (data->mul_ch->fine_coarse_b & 0xFF),
					((data->mul_ch->fine_coarse_b >> 8) & 0x3F), data->mul_ch->cdc_b,
					data->mul_ch->base_b, data->mul_ch->diff_b, data->skip_data);
		}
#endif
		data->debug_cnt = 0;
	} else {
		data->debug_cnt++;
	}

	return ret;
}

static void force_far_grip(struct isg6320_data *data)
{
	if (data->state == CLOSE) {
		pr_info("[GRIP_%d] %s\n", data->ic_num, __func__);

		if (data->skip_data == true)
			return;

		input_report_rel(data->input_dev, REL_MISC, 2);
		input_report_rel(data->input_dev, REL_X, data->is_unknown_mode);
		input_sync(data->input_dev);
		data->state = FAR;
	}
#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->multi_use) {
		if (data->mul_ch->state_b == CLOSE) {
			pr_info("[GRIP_%d] [B] %s\n", data->ic_num, __func__);

			if (data->skip_data == true)
				return;

			input_report_rel(data->input_dev, REL_DIAL, 2);
			input_report_rel(data->input_dev, REL_Y, data->mul_ch->is_unknown_mode);
			input_sync(data->input_dev);
			data->mul_ch->state_b = FAR;
		}
	}
#endif
}

static void report_event_data(struct isg6320_data *data, u8 irq_msg)
{
	int state_a;
#ifdef CONFIG_USE_MULTI_CHANNEL
	int state_b;
#endif
	if (data->skip_data == true) {
		pr_info("[GRIP_%d] skip grip event\n", data->ic_num);
		return;
	}

	state_a = (irq_msg & (1 << ISG6320_PROX_A_STATE)) ? CLOSE : FAR;
#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->multi_use) {
		state_b = (irq_msg & (1 << ISG6320_PROX_B_STATE)) ? CLOSE : FAR;
	}
#endif

	if (data->abnormal_mode) {
		if (state_a == CLOSE) {
			if (data->max_diff < data->diff)
				data->max_diff = data->diff;
			data->irq_count++;
		}
#ifdef CONFIG_USE_MULTI_CHANNEL
		if (data->multi_use) {
			if (state_b == CLOSE) {
				if (data->mul_ch->max_diff_b < data->mul_ch->diff_b)
					data->mul_ch->max_diff_b = data->mul_ch->diff_b;
				data->irq_count++;
			}
		}
#endif
	}

	if (state_a == CLOSE) {
		if (data->state == FAR) {
			pr_info("[GRIP_%d] CLOSE\n", data->ic_num);
			data->state = CLOSE;
		}
	} else {
		if (data->state == CLOSE) {
			pr_info("[GRIP_%d] FAR\n", data->ic_num);
			data->state = FAR;
		}
	}
#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->multi_use) {
		if (state_b == CLOSE) {
			if (data->mul_ch->state_b == FAR) {
				pr_info("[GRIP_%d] [B] CLOSE\n", data->ic_num);
				data->mul_ch->state_b = CLOSE;
			}
		} else {
			if (data->mul_ch->state_b == CLOSE) {
				pr_info("[GRIP_%d] [B] FAR\n", data->ic_num);
				data->mul_ch->state_b = FAR;
			}
		}
	}
#endif

	if (data->state == CLOSE) {
		input_report_rel(data->input_dev, REL_MISC, 1);
		if (data->is_unknown_mode == UNKNOWN_ON && data->motion)
			data->first_working = true;
	} else {
		input_report_rel(data->input_dev, REL_MISC, 2);
		if (data->is_unknown_mode == UNKNOWN_ON && data->motion) {
			if (data->first_working) {
				pr_info("[GRIP_%d] unknown mode off\n", data->ic_num);
				data->is_unknown_mode = UNKNOWN_OFF;
				data->first_working = false;
			}
		}
	}
#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->multi_use) {
		if (data->mul_ch->state_b == CLOSE) {
			input_report_rel(data->input_dev, REL_DIAL, 1);
			if (data->mul_ch->is_unknown_mode == UNKNOWN_ON && data->motion)
				data->mul_ch->first_working = true;
		} else {
			input_report_rel(data->input_dev, REL_DIAL, 2);
			if (data->mul_ch->is_unknown_mode == UNKNOWN_ON && data->motion) {
				if (data->mul_ch->first_working) {
				pr_info("[GRIP_%d] [B] unknown mode off\n", data->ic_num);
					data->mul_ch->is_unknown_mode = UNKNOWN_OFF;
					data->mul_ch->first_working = false;
				}
			}
		}
	}
#endif
	input_report_rel(data->input_dev, REL_X, data->is_unknown_mode);
#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->multi_use) {
		input_report_rel(data->input_dev, REL_Y, data->mul_ch->is_unknown_mode);
	}
#endif
	input_sync(data->input_dev);
}

static u8 isg6320_read_irqstate(struct isg6320_data *data)
{
	int ret = 0;
	u8 irq_msg = 0;
	int retry = 3;
	
	while (retry--) {
		ret = isg6320_i2c_read(data, ISG6320_IRQSRC_REG, &irq_msg, 1);
		if (ret < 0) {
			pr_err("[GRIP_%d] fail to read source(%d)\n", data->ic_num, ret);
			usleep_range(10000, 10010);
		}
		else
			break;
	}

#if !IS_ENABLED(CONFIG_SAMSUNG_PRODUCT_SHIP)
	if (data->irq_debug_size > 0)
		isg6320_irq_debug(data);
#endif
	retry = 3;
	
	while (retry--) {
		ret = isg6320_i2c_read(data, ISG6320_IRQSTS_REG, &irq_msg, 1);
		if (ret < 0) {
			pr_err("[GRIP_%d] fail to read state(%d)\n", data->ic_num, ret);
			usleep_range(10000, 10010);
		}
		else
			break;
	}

	return irq_msg;
}

static void irq_work_func(struct work_struct *work)
{
	struct isg6320_data *data = container_of((struct work_struct *)work,
						struct isg6320_data, irq_work);
	int ret;
	u8 irq_msg;

	irq_msg = isg6320_read_irqstate(data);
	pr_info("[GRIP_%d] irq_msg: 0x%02X\n", data->ic_num, irq_msg);

	msleep(100);
	ret = isg6320_get_raw_data(data, true);
	if (ret < 0)
		pr_err("[GRIP_%d] fail to update data\n", data->ic_num);

	report_event_data(data, irq_msg);
}

static void cfcal_work_func(struct work_struct *work)
{
	struct isg6320_data *data = container_of((struct work_struct *)work,
						struct isg6320_data, cfcal_work);

	data->schedule_time = SHCEDULE_INTERVAL;
	isg6320_force_calibration(data);
}

#if !defined(CONFIG_SEC_FACTORY) && defined(CONFIG_SUPPORT_MCC_THRESHOLD_CHANGE)
static void mcc_work_func(struct work_struct *work)
{
	struct isg6320_data *data = container_of((struct work_struct *)work,
						struct isg6320_data, mcc_work);
	u8 threshold, hysteresis;
	u8 val;

	// 001 : call box, 440/441 : jpn, 450 : kor, 460 : chn
	if (data->mcc != 450) {
		pr_info("[GRIP_%d] %s - default threshold %u, %u\n", data->ic_num, __func__,
			data->default_threshold * 4, data->default_threshold * 4 - data->default_hysteresis);
		threshold = data->default_threshold;
		hysteresis = data->default_hysteresis;
	} else {
		threshold = data->mcc_threshold;
		hysteresis = data->mcc_hysteresis;
	}

	cancel_work_sync(&data->cfcal_work);
	isg6320_set_debug_work(data, OFF, 0);

	mutex_lock(&data->lock);
	isg6320_i2c_write(data, ISG6320_SCANCTRL1_REG, ISG6320_DFE_ENABLE);
	usleep_range(10000, 10010);
	isg6320_i2c_write(data, ISG6320_SCANCTRL1_REG, ISG6320_SCAN_STOP);
	usleep_range(10000, 10010);

	isg6320_i2c_write(data, ISG6320_A_PROXCTL4_REG, threshold);
	isg6320_i2c_write(data, ISG6320_A_PROXCTL8_REG, hysteresis);

	isg6320_i2c_write(data, ISG6320_SCANCTRL1_REG, ISG6320_CFCAL_START);
	msleep(450);

	pr_info("[GRIP_%d] %s - change threshold %d,%d\n", data->ic_num, __func__,
		threshold * 4, threshold * 4 - hysteresis);

	isg6320_i2c_read(data, ISG6320_CFCAL_RTN_REG, &val, 1);
	if (!(val & ISG6320_CAL_RTN_A_MASK))
		pr_err("[GRIP_%d] %s - fail calibration\n", data->ic_num, __func__);

	mutex_unlock(&data->lock);

	isg6320_set_debug_work(data, ON, SHCEDULE_INTERVAL + (data->ic_num << 3));
}
#endif

static irqreturn_t isg6320_irq_thread(int irq, void *ptr)
{
	struct isg6320_data *data = (struct isg6320_data *)ptr;

	if (data->initialized == OFF)
		return IRQ_HANDLED;

	__pm_wakeup_event(data->grip_ws, jiffies_to_msecs(3 * HZ));
	schedule_work(&data->irq_work);

	return IRQ_HANDLED;
}

static void isg6320_enter_unknown_mode(struct isg6320_data *data, int type)
{
	if (data->noti_enable) {
		data->motion = 0;
		data->first_working = false;
		if (data->is_unknown_mode == UNKNOWN_OFF) {
			data->is_unknown_mode = UNKNOWN_ON;
			if (!data->skip_data) {
				input_report_rel(data->input_dev, REL_X, data->is_unknown_mode);
				input_sync(data->input_dev);
			}
			pr_info("[GRIP_%d] UNKNOWN Re-enter\n", data->ic_num);
		} else {
			pr_info("[GRIP_%d] already UNKNOWN\n", data->ic_num);
		}
#ifdef CONFIG_USE_MULTI_CHANNEL
		if (data->multi_use) {
			data->mul_ch->first_working = false;
				if (data->mul_ch->is_unknown_mode == UNKNOWN_OFF) {
					data->mul_ch->is_unknown_mode = UNKNOWN_ON;
					if (!data->skip_data) {
						input_report_rel(data->input_dev, REL_Y, data->mul_ch->is_unknown_mode);
						input_sync(data->input_dev);
					}
					pr_info("[GRIP_%d] [B] UNKNOWN Re-enter\n", data->ic_num);
				} else {
					pr_info("[GRIP_%d] [B] already UNKNOWN\n", data->ic_num);
				}
		}
#endif
		input_report_rel(data->noti_input_dev, REL_X, type);
		input_sync(data->noti_input_dev);
	}
}

static int isg6320_set_normal_mode(struct isg6320_data *data)
{
	int ret = -EINVAL;
	u8 state;

	pr_info("[GRIP_%d] %s\n", data->ic_num, __func__);

	ret = isg6320_i2c_read(data, ISG6320_IRQSRC_REG, &state, 1);
	if (ret < 0) {
		pr_err("[GRIP_%d] %s - i2c read fail(%d)\n", data->ic_num, __func__, ret);
		return ret;
	}

	schedule_work(&data->cfcal_work);

	return ret;
}

static void isg6320_initialize(struct isg6320_data *data)
{
	int ret;
	u8 val;
	u8 buf8[2] = {0, 0};

	pr_info("[GRIP_%d] %s\n", data->ic_num, __func__);
	mutex_lock(&data->lock);
	force_far_grip(data);

	ret = isg6320_i2c_read(data, ISG6320_IRQSRC_REG, &val, 1);
	if (ret < 0) {
		pr_err("[GRIP_%d] %s IRQSRC read fail\n", data->ic_num, __func__);
		mutex_unlock(&data->lock);
		return;
	}
	ret = isg6320_i2c_write(data, ISG6320_SCANCTRL1_REG, ISG6320_SCAN_STOP);
	if (ret < 0) {
		pr_err("[GRIP_%d] %s SCANCTRL write fail\n", data->ic_num, __func__);
		mutex_unlock(&data->lock);
		return;
	}
	msleep(30);

	if (data->setup_reg_exist) {
		int i = 0;
		for (; i < data->reg_size ; i++) {
			int index = i * 2;
			isg6320_i2c_write(data, data->setup_reg[index],
					data->setup_reg[index + 1]);
		}
	}

	ret = isg6320_i2c_write(data, ISG6320_IRQFUNC_REG, ISG6320_IRQ_DISABLE);
	if (ret < 0) {
		pr_err("[GRIP_%d] %s IRQFUNC write fail\n", data->ic_num, __func__);
	}

	if (data->normal_th > 0) {
		val = data->normal_th / 4;
		isg6320_i2c_write(data, ISG6320_A_PROXCTL4_REG, val);
	}
#if !defined(CONFIG_SEC_FACTORY) && defined(CONFIG_SUPPORT_MCC_THRESHOLD_CHANGE)
	if (data->ic_num == MAIN_GRIP) {
		isg6320_i2c_read(data, ISG6320_A_PROXCTL4_REG, &val, 1);
		data->default_threshold = val;
		if (data->mcc_threshold == 0)
			data->mcc_threshold = data->default_threshold;

		isg6320_i2c_read(data, ISG6320_A_PROXCTL8_REG, &val, 1);
		data->default_hysteresis = val;
		if (data->mcc_hysteresis == 0)
			data->mcc_hysteresis = data->default_hysteresis;
	}
#endif

#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->multi_use) {
		if (data->mul_ch->normal_th_b > 0) {
			val = data->mul_ch->normal_th_b / 4;
			isg6320_i2c_write(data, ISG6320_B_PROXCTL4_REG, val);
		}
	}
#endif

	ret = isg6320_i2c_read(data, ISG6320_A_DIGITAL_ACC_REG, &val, 1);
	if (ret < 0)
		pr_err("[GRIP_%d] DIGITAL ACC read fail\n", data->ic_num);
	else
		data->cfcal_th = ISG6320_CS_RESET_CONDITION * val / 8;

#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->multi_use) {
		ret = isg6320_i2c_read(data, ISG6320_B_DIGITAL_ACC_REG, &val, 1);
		if (ret < 0)
			pr_err("[GRIP_%d] [B] DIGITAL ACC read fail\n", data->ic_num);
		else
			data->mul_ch->cfcal_th_b = ISG6320_CS_RESET_CONDITION * val / 8;
	}
#endif
#ifdef CONFIG_USE_IC_TYPE
	{
		u16 target;
		u8 sample;
		u8 isum_val;

		ret = isg6320_i2c_read(data, ISG6320_IC_TYPE_REG, &sample, 1);
		if (ret < 0) {
			pr_err("[GRIP_%d] IC Type read fail\n", data->ic_num);
		} else {
			if (sample == ES) {
				pr_info("[GRIP_%d] %s - ES Sample\n", data->ic_num, __func__);
				target = ISG6320_ES_TARGET;
				data->cfcal_th = ISG6320_ES_RESET_CONDITION * val / 8;
#ifdef CONFIG_USE_MULTI_CHANNEL
				if (data->multi_use)
					data->mul_ch->cfcal_th_b = ISG6320_ES_RESET_CONDITION * val / 8;
#endif
			} else {
				pr_info("[GRIP_%d] %s - CS Sample\n", data->ic_num, __func__);
				target = ISG6320_CS_TARGET;
			}

			isg6320_i2c_write(data, ISG6320_A_ACALCTL4_REG, (target >> 8) & 0xFF);
			isg6320_i2c_write(data, ISG6320_A_ACALCTL5_REG, target & 0xFF);
			isg6320_i2c_write(data, ISG6320_B_ACALCTL4_REG, (target >> 8) & 0xFF);
			isg6320_i2c_write(data, ISG6320_B_ACALCTL5_REG, target & 0xFF);
		}

		if (sample == ES) {
			//integrate count
			ret = isg6320_i2c_read(data, ISG6320_A_LSUM_TYPE_REG, &isum_val, 1);
			if (ret < 0) {
				pr_err("[GRIP_%d] %s A_LSUM read fail\n", data->ic_num, __func__);
			} else {
				isum_val = isum_val & 0xF0;
				val = isum_val >> 4;

				if (val <= 6)
					val = val >> 1;
				else if (val <= 14)
					val = val - 4;
				else
					val = val - 3;

				isum_val = isum_val | val;
				isg6320_i2c_write(data, ISG6320_A_LSUM_TYPE_REG, isum_val);
			}
#if defined(CONFIG_TABLET_MODEL_CONCEPT)
			ret = isg6320_i2c_read(data, ISG6320_A_LSUM_TYPE_REG, &isum_val, 1);
			data->lsum_a = isum_val;
			if (ret < 0) {
				pr_err("[GRIP_%d] %s A_LSUM read fail\n", data->ic_num, __func__);
				mutex_unlock(&data->lock);
				return;
			}
#endif
#ifdef CONFIG_USE_MULTI_CHANNEL
			if (data->multi_use) {
				ret = isg6320_i2c_read(data, ISG6320_B_LSUM_TYPE_REG, &isum_val, 1);
				if (ret < 0) {
					pr_err("[GRIP_%d] %s B_LSUM read fail\n", data->ic_num, __func__);
				} else {
					isum_val = isum_val & 0xF0;
					val = isum_val >> 4;

					if (val <= 6)
						val = val >> 1;
					else if (val <= 14)
						val = val - 4;
					else
						val = val - 3;

					isum_val = isum_val | val;
					isg6320_i2c_write(data, ISG6320_B_LSUM_TYPE_REG, isum_val);
				}
			}
#if defined(CONFIG_TABLET_MODEL_CONCEPT)
			ret = isg6320_i2c_read(data, ISG6320_B_LSUM_TYPE_REG, &isum_val, 1);
			data->lsum_b = isum_val;
			if (ret < 0) {
				pr_err("[GRIP_%d] %s B_LSUM read fail\n", data->ic_num, __func__);
				mutex_unlock(&data->lock);
				return;
			}
#endif
#endif
		}
	}
#endif
	mutex_unlock(&data->lock);

	data->initialized = ON;

	isg6320_set_normal_mode(data);

	isg6320_i2c_read(data, ISG6320_A_PROXCTL4_REG, buf8, sizeof(buf8));
	data->normal_th = (u32)buf8[0] * 4;
#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->multi_use) {
		isg6320_i2c_read(data, ISG6320_B_PROXCTL4_REG, buf8, sizeof(buf8));
		data->mul_ch->normal_th_b = (u32)buf8[0] * 4;
	}
#endif
}

static void isg6320_set_debug_work(struct isg6320_data *data, bool enable,
					unsigned int delay_ms)
{
	if (enable == ON) {
		data->debug_cnt = GRIP_LOG_TIME;
		schedule_delayed_work(&data->cal_work, msecs_to_jiffies(delay_ms));
	} else {
		cancel_delayed_work_sync(&data->cal_work);
	}
}

static void isg6320_set_enable(struct isg6320_data *data, int enable)
{
	u8 state;
	int ret = 0;
	int retry = 3;

	pr_info("[GRIP_%d] %s\n", data->ic_num, __func__);

	if (data->enable == enable) {
		pr_info("[GRIP_%d] already enabled\n", data->ic_num);
		return;
	}

	if (enable == ON) {
		pr_info("[GRIP_%d] %s enable\n", data->ic_num, __func__);

		data->diff_avg = 0;
		data->diff_cnt = 0;
		data->cdc_avg = 0;

#ifdef CONFIG_USE_MULTI_CHANNEL
		if (data->multi_use) {
			data->mul_ch->diff_avg_b = 0;
			data->mul_ch->diff_cnt_b = 0;
			data->mul_ch->cdc_avg_b = 0;
		}
#endif
		while (retry--) {
			ret = isg6320_i2c_read(data, ISG6320_IRQSTS_REG, &state, 1);
			if (ret < 0)
				pr_err("[GRIP_%d] %s IRQSTS read fail\n", data->ic_num, __func__);
			else
				break;
		}

		isg6320_get_raw_data(data, true);

		if (data->skip_data == true) {
			input_report_rel(data->input_dev, REL_MISC, 2);
#ifdef CONFIG_USE_MULTI_CHANNEL
			if (data->multi_use) {
				input_report_rel(data->input_dev, REL_DIAL, 2);
			}
#endif
			input_report_rel(data->input_dev, REL_X, UNKNOWN_OFF);
#ifdef CONFIG_USE_MULTI_CHANNEL
			if (data->multi_use) {
				input_report_rel(data->input_dev, REL_Y, UNKNOWN_OFF);
			}
#endif
		} else {
			if (state & (1 << ISG6320_PROX_A_STATE)) {
				data->state = CLOSE;
				input_report_rel(data->input_dev, REL_MISC, 1);
			} else {
				data->state = FAR;
				input_report_rel(data->input_dev, REL_MISC, 2);
			}
#ifdef CONFIG_USE_MULTI_CHANNEL
			if (data->multi_use) {
				if (state & (1 << ISG6320_PROX_B_STATE)) {
					data->mul_ch->state_b = CLOSE;
					input_report_rel(data->input_dev, REL_DIAL, 1);
				} else {
					data->mul_ch->state_b = FAR;
					input_report_rel(data->input_dev, REL_DIAL, 2);
				}
			}
#endif
			input_report_rel(data->input_dev, REL_X, data->is_unknown_mode);
#ifdef CONFIG_USE_MULTI_CHANNEL
			if (data->multi_use) {
				input_report_rel(data->input_dev, REL_Y, data->mul_ch->is_unknown_mode);
			}
#endif
		}
		input_sync(data->input_dev);

		isg6320_i2c_read(data, ISG6320_IRQSRC_REG, &state, 1);
#ifdef CONFIG_USE_MULTI_CHANNEL
		if (data->multi_use) {
#ifdef CONFIG_USE_SINGLE_IRQ_B
			isg6320_i2c_write(data, ISG6320_IRQFUNC_REG, ISG6320_IRQ_ENABLE_B);
#else
			isg6320_i2c_write(data, ISG6320_IRQFUNC_REG, ISG6320_IRQ_ENABLE);
#endif
		}
		else
			isg6320_i2c_write(data, ISG6320_IRQFUNC_REG, ISG6320_IRQ_ENABLE_A);
#else
		isg6320_i2c_write(data, ISG6320_IRQFUNC_REG, ISG6320_IRQ_ENABLE_A);
#endif

		enable_irq(data->client->irq);
		enable_irq_wake(data->client->irq);
	} else {
		pr_info("[GRIP_%d] %s disable\n", data->ic_num, __func__);

		while (retry--) {
			ret = isg6320_i2c_write(data, ISG6320_IRQFUNC_REG, ISG6320_IRQ_DISABLE);
			if (ret < 0)
				pr_err("[GRIP_%d] %s IRQFUNC write fail\n", data->ic_num, __func__);
			else
				break;
		}

		disable_irq(data->client->irq);
		disable_irq_wake(data->client->irq);
	}

	data->enable = enable;
}

static ssize_t isg6320_name_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	pr_info("[GRIP_%d] %s %s\n", data->ic_num, __func__,
			device_name[data->ic_num]);

	return sprintf(buf, "%s\n", device_name[data->ic_num]);
}

static ssize_t isg6320_vendor_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	pr_info("[GRIP_%d] %s%s\n", data->ic_num, __func__, VENDOR_NAME);

	return sprintf(buf, "%s\n", VENDOR_NAME);
}

static ssize_t isg6320_mode_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "1\n");
}

static ssize_t isg6320_manual_acal_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "OK\n");
}

static ssize_t isg6320_acal_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "2,0,0\n");
}

static ssize_t isg6320_onoff_store(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	u8 val;
	int ret;
	struct isg6320_data *data = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 2, &val);
	if (ret) {
		pr_err("[GRIP_%d] invalid argument\n", data->ic_num);
		return ret;
	}

	if (val == 0) {
		data->skip_data = true;
		if (data->enable == ON) {
			data->state = FAR;
			input_report_rel(data->input_dev, REL_MISC, 2);
#ifdef CONFIG_USE_MULTI_CHANNEL
			if (data->multi_use) {
				data->mul_ch->state_b = FAR;
				input_report_rel(data->input_dev, REL_DIAL, 2);
			}
#endif
			input_report_rel(data->input_dev, REL_X, UNKNOWN_OFF);
#ifdef CONFIG_USE_MULTI_CHANNEL
			if (data->multi_use) {
				input_report_rel(data->input_dev, REL_Y, UNKNOWN_OFF);
			}
#endif
			input_sync(data->input_dev);
		}
		data->motion = 1;
		data->is_unknown_mode = UNKNOWN_OFF;
		data->first_working = false;
#ifdef CONFIG_USE_MULTI_CHANNEL
		if (data->multi_use) {
			data->mul_ch->is_unknown_mode = UNKNOWN_OFF;
			data->mul_ch->first_working = false;
		}
#endif
	} else {
		data->skip_data = false;
	}

	pr_info("[GRIP_%d] %d\n", data->ic_num, (int)val);

	return count;
}

static ssize_t isg6320_onoff_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", !data->skip_data);
}

static ssize_t isg6320_sw_reset_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	int ret = 0;

	pr_info("[GRIP_%d] %s\n", data->ic_num, __func__);

	cancel_delayed_work_sync(&data->cal_work);

	ret = isg6320_force_calibration(data);
	isg6320_get_raw_data(data, true);

	schedule_delayed_work(&data->cal_work, msecs_to_jiffies(1000));

	if (ret)
		return sprintf(buf, "-1\n");

	return sprintf(buf, "%d\n", 0);
}

static ssize_t isg6320_normal_threshold_store(struct device *dev,
						   struct device_attribute *attr, const char *buf, size_t size)
{
	int val = 0;
	u8 buf8;
	struct isg6320_data *data = dev_get_drvdata(dev);

	sscanf(buf, "%d", &val);

	if (val < 0) {
		pr_err("[GRIP_%d] invalid argument\n", data->ic_num);
		return size;
	}

	pr_info("[GRIP_%d] change threshold(%d->%d)\n", data->ic_num,
		data->normal_th, val);

	data->normal_th = val;

	buf8 = data->normal_th / 4;

	isg6320_i2c_write(data, ISG6320_A_PROXCTL4_REG, buf8);

	return size;
}

static ssize_t isg6320_normal_threshold_show(struct device *dev,
						  struct device_attribute *attr, char *buf)
{
	u32 threshold = 0;
	u32 hyst = 0;
	u8 buf8[2];
	struct isg6320_data *data = dev_get_drvdata(dev);

	isg6320_i2c_read(data, ISG6320_A_PROXCTL4_REG, buf8, sizeof(buf8));

	threshold = (u32)buf8[0] * 4;
	hyst = buf8[1];

	return sprintf(buf, "%d,%d\n", threshold, threshold - hyst);
}

static ssize_t isg6320_raw_data_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	isg6320_get_raw_data(data, true);
	if (data->diff_cnt == 0) {
		data->diff_sum = data->diff;
		data->cdc_sum = data->cdc;
	} else {
		data->diff_sum += data->diff;
		data->cdc_sum += data->cdc;
	}

	if (++data->diff_cnt >= ISG6320_DIFF_AVR_CNT) {
		data->diff_avg = data->diff_sum / ISG6320_DIFF_AVR_CNT;
		data->cdc_avg = data->cdc_sum / ISG6320_DIFF_AVR_CNT;
		data->diff_cnt = 0;
	}

	return sprintf(buf, "%d%02d,%d,%d,%d,%d\n", (data->fine_coarse & 0xFF),
			   ((data->fine_coarse >> 8) & 0x3F), data->cdc,
			   data->fine_coarse, data->diff, data->base);
}

static ssize_t isg6320_diff_avg_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->diff_avg);
}

static ssize_t isg6320_cdc_avg_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->cdc_avg);
}

static ssize_t isg6320_ch_state_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	int count;
	struct isg6320_data *data = dev_get_drvdata(dev);

	if (data->skip_data == true)
		count = snprintf(buf, PAGE_SIZE, "%d,%d\n", NONE_ENABLE, NONE_ENABLE);
#ifdef CONFIG_USE_MULTI_CHANNEL
	else if (data->enable == ON && data->multi_use)
		count = snprintf(buf, PAGE_SIZE, "%d,%d\n", data->state,
				data->mul_ch->state_b);
#else
	else if (data->enable == ON)
		count = snprintf(buf, PAGE_SIZE, "%d,%d\n", data->state,
				NONE_ENABLE);
#endif
	else
		count = snprintf(buf, PAGE_SIZE, "%d,%d\n", NONE_ENABLE, NONE_ENABLE);

	return count;
}

static ssize_t isg6320_hysteresis_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	u8 buf8;
	struct isg6320_data *data = dev_get_drvdata(dev);

	isg6320_i2c_read(data, ISG6320_A_PROXCTL8_REG, &buf8, 1);


	return sprintf(buf, "%d\n", buf8);
}

static ssize_t isg6320_sampling_freq_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	u8 buff;
	int sampling_freq;

	isg6320_i2c_read(data, ISG6320_NUM_OF_CLK, &buff, 1);
	sampling_freq = (int)(8000 / ((int)buff + 1));

	return snprintf(buf, PAGE_SIZE, "%dkHz\n", sampling_freq);
}

static ssize_t isg6320_isum_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	const char *table[16] = {
		"1", "2", "4", "6", "8", "10", "12", "14", "16",
		"20", "24", "28", "32", "40", "48", "64"
	};
	u8 buff = 0;

	isg6320_i2c_read(data, ISG6320_A_LSUM_TYPE_REG, &buff, 1);
	buff = (buff & 0xf0) >> 4;

	return snprintf(buf, PAGE_SIZE, "%s\n", table[buff]);
}

static ssize_t isg6320_scan_period_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	u8 buff[2];
	int scan_period;

	isg6320_i2c_read(data, ISG6320_WUTDATA_REG, (u8 *)&buff, sizeof(buff));

	scan_period = (int)(((u16)buff[1] & 0xff) | (((u16)buff[0] & 0x3f) << 8));
	if (!scan_period)
		return snprintf(buf, PAGE_SIZE, "%d\n", scan_period);

	scan_period = (int)(4000 / (scan_period + 1));

	return snprintf(buf, PAGE_SIZE, "%d\n", scan_period);
}

static ssize_t isg6320_again_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	u8 buff;
	u8 temp1, temp2;

	isg6320_i2c_read(data, ISG6320_A_ANALOG_GAIN, &buff, 1);
	temp1 = (buff & 0x38) >> 3;
	temp2 = (buff & 0x07);

	return snprintf(buf, PAGE_SIZE, "%d/%d\n", (int)temp2 + 1, (int)temp1 + 1);
}

static ssize_t isg6320_cdc_up_coef_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	u8 buff;
	int coef;

	isg6320_i2c_read(data, ISG6320_A_CDC_UP_COEF_REG, &buff, 1);
	coef = (int)buff;

	return snprintf(buf, PAGE_SIZE, "0x%x, %d\n", buff, coef);
}

static ssize_t isg6320_cdc_down_coef_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	u8 buff;
	int coef;

	isg6320_i2c_read(data, ISG6320_A_CDC_DN_COEF_REG, &buff, 1);
	coef = (int)buff;

	return snprintf(buf, PAGE_SIZE, "0x%x, %d\n", buff, coef);
}

static ssize_t isg6320_temp_enable_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	u8 buff;

	isg6320_i2c_read(data, ISG6320_A_TEMPERATURE_ENABLE_REG, &buff, 1);

	return snprintf(buf, PAGE_SIZE, "%d\n", ((buff & 0x80) >> 7));
}

static ssize_t isg6320_irq_count_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	int ret = 0;
	s16 max_diff_val;

	if (data->irq_count) {
		ret = -1;
		max_diff_val = data->max_diff;
	} else {
		max_diff_val = data->max_normal_diff;
	}

	pr_info("[GRIP_%d] %s - called\n", data->ic_num, __func__);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n", ret, data->irq_count,
			max_diff_val);
}

static ssize_t isg6320_irq_count_store(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	int ret;
	u8 onoff;

	ret = kstrtou8(buf, 10, &onoff);
	if (ret < 0) {
		pr_err("[GRIP_%d] invalid arg\n", data->ic_num);
		return count;
	}

	mutex_lock(&data->lock);

	if (onoff == 0) {
		data->abnormal_mode = OFF;
	} else if (onoff == 1) {
		data->abnormal_mode = ON;
		data->schedule_time = SHCEDULE_INTERVAL;
		data->irq_count = 0;
		data->max_diff = 0;
		data->max_normal_diff = 0;
	} else {
		pr_err("[GRIP_%d] invalid val %d\n", data->ic_num, onoff);
	}

	mutex_unlock(&data->lock);

	pr_info("[GRIP_%d] %s - %d\n", data->ic_num, __func__, onoff);

	return count;
}
static ssize_t isg6320_motion_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	if (data->motion)
		return snprintf(buf, PAGE_SIZE, "motion_detect\n");
	else
		return snprintf(buf, PAGE_SIZE, "motion_non_detect\n");
}

static ssize_t isg6320_motion_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	int ret;
	struct isg6320_data *data = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 10, &val);
	if (ret) {
		pr_info("[GRIP_%d] %s - Invalid Argument\n", data->ic_num, __func__);
		return ret;
	}

	if (val == 0) {
		pr_info("[GRIP_%d] %s - motion event off\n", data->ic_num, __func__);
		data->motion = val;
	} else if (val == 1) {
		pr_info("[GRIP_%d] %s - motion event\n", data->ic_num, __func__);
		data->motion = val;
	} else {
		pr_info("[GRIP_%d] %s - Invalid Argument : %u\n", data->ic_num, __func__, val);
	}
	pr_info("[GRIP_%d] %s - %u\n", data->ic_num, __func__, val);
	return count;
}

static ssize_t isg6320_unknown_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	
	return snprintf(buf, PAGE_SIZE, "%s\n",
		(data->is_unknown_mode == 1) ? "UNKNOWN" : "NORMAL");
}

static ssize_t isg6320_unknown_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	int ret;
	struct isg6320_data *data = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 10, &val);
	if (ret) {
		pr_info("[GRIP_%d] %s - Invalid Argument\n", data->ic_num, __func__);
		return ret;
	}

	if (val == 1)
		isg6320_enter_unknown_mode(data, TYPE_FORCE);
	else if (val == 0) {
		data->is_unknown_mode = UNKNOWN_OFF;
#ifdef CONFIG_USE_MULTI_CHANNEL
		if (data->multi_use)
			data->mul_ch->is_unknown_mode = UNKNOWN_OFF;
#endif
	}
	else
		pr_info("[GRIP_%d] %s - Invalid Argument(%d)\n", data->ic_num, __func__, val);

	pr_info("[GRIP_%d] %s - %u\n", data->ic_num, __func__, val);
	return count;
}
#if 0
static ssize_t isg6320_cml_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s/%s\n",
		(data->motion) ? "MOTION_DETECT" : "MOTION_NON_DETECT", (data->is_unknown_mode == 1) ? "UNKNOWN" : "NORMAL");
}
#endif
#ifdef CONFIG_USE_MULTI_CHANNEL
static ssize_t isg6320_normal_threshold_b_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int val = 0;
	u8 buf8;
	struct isg6320_data *data = dev_get_drvdata(dev);

	sscanf(buf, "%d", &val);

	if (val < 0) {
		pr_err("[GRIP_%d] [B] invalid arg\n", data->ic_num);
		return size;
	}

	pr_info("[GRIP_%d] [B] change threshold(%d->%d)\n",
			data->ic_num, data->mul_ch->normal_th_b, val);

	data->mul_ch->normal_th_b = val;

	buf8 = data->mul_ch->normal_th_b / 4;

	isg6320_i2c_write(data, ISG6320_B_PROXCTL4_REG, buf8);

	return size;
}

static ssize_t isg6320_normal_threshold_b_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 threshold = 0;
	u32 hyst = 0;
	u8 buf8[2];
	struct isg6320_data *data = dev_get_drvdata(dev);

	isg6320_i2c_read(data, ISG6320_B_PROXCTL4_REG, buf8, sizeof(buf8));

	threshold = (u32)buf8[0] * 4;
	hyst = buf8[1];

	return sprintf(buf, "%d,%d\n", threshold, threshold - hyst);
}

static ssize_t isg6320_raw_data_b_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	isg6320_get_raw_data(data, true);
	if (data->mul_ch->diff_cnt_b == 0) {
		data->mul_ch->diff_sum_b = data->mul_ch->diff_b;
		data->mul_ch->cdc_sum_b = data->mul_ch->cdc_b;
	} else {
		data->mul_ch->diff_sum_b += data->mul_ch->diff_b;
		data->mul_ch->cdc_sum_b += data->mul_ch->cdc_b;
	}

	if (++data->mul_ch->diff_cnt_b >= ISG6320_DIFF_AVR_CNT) {
		data->mul_ch->diff_avg_b = data->mul_ch->diff_sum_b / ISG6320_DIFF_AVR_CNT;
		data->mul_ch->cdc_avg_b = data->mul_ch->cdc_sum_b / ISG6320_DIFF_AVR_CNT;
		data->mul_ch->diff_cnt_b = 0;
	}

	return sprintf(buf, "%d%02d,%d,%d,%d,%d\n", (data->mul_ch->fine_coarse_b & 0xFF),
			((data->mul_ch->fine_coarse_b >> 8) & 0x3F), data->mul_ch->cdc_b,
			data->mul_ch->fine_coarse_b, data->mul_ch->diff_b, data->mul_ch->base_b);
}

static ssize_t isg6320_debug_data_b_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d,%d,%d\n", data->mul_ch->cdc_b, data->mul_ch->base_b,
			data->mul_ch->diff_b);
}

static ssize_t isg6320_diff_avg_b_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->mul_ch->diff_avg_b);
}

static ssize_t isg6320_cdc_avg_b_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", data->mul_ch->cdc_avg_b);
}

static ssize_t isg6320_hysteresis_b_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 buf8;
	struct isg6320_data *data = dev_get_drvdata(dev);

	isg6320_i2c_read(data, ISG6320_B_PROXCTL8_REG, &buf8, 1);

	return sprintf(buf, "%d\n", buf8);
}

static ssize_t isg6320_isum_b_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	const char *table[16] = {
		"1", "2", "4", "6", "8", "10", "12", "14", "16",
		"20", "24", "28", "32", "40", "48", "64"
	};
	u8 buff = 0;

	isg6320_i2c_read(data, ISG6320_B_LSUM_TYPE_REG, &buff, 1);
	buff = (buff & 0xf0) >> 4;

	return snprintf(buf, PAGE_SIZE, "%s\n", table[buff]);
}

static ssize_t isg6320_again_b_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	u8 buff;
	u8 temp1, temp2;

	isg6320_i2c_read(data, ISG6320_B_ANALOG_GAIN, &buff, 1);
	temp1 = (buff & 0x38) >> 3;
	temp2 = (buff & 0x07);

	return snprintf(buf, PAGE_SIZE, "%d/%d\n", (int)temp2 + 1, (int)temp1 + 1);
}

static ssize_t isg6320_cdc_up_coef_b_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	u8 buff;
	int coef;

	isg6320_i2c_read(data, ISG6320_B_CDC_UP_COEF_REG, &buff, 1);
	coef = (int)buff;

	return snprintf(buf, PAGE_SIZE, "0x%x, %d\n", buff, coef);
}

static ssize_t isg6320_cdc_down_coef_b_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	u8 buff;
	int coef;

	isg6320_i2c_read(data, ISG6320_B_CDC_DN_COEF_REG, &buff, 1);
	coef = (int)buff;

	return snprintf(buf, PAGE_SIZE, "0x%x, %d\n", buff, coef);
}

static ssize_t isg6320_temp_enable_b_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	u8 buff;

	isg6320_i2c_read(data, ISG6320_B_TEMPERATURE_ENABLE_REG, &buff, 1);

	return snprintf(buf, PAGE_SIZE, "%d\n", ((buff & 0x80) >> 7));
}

static ssize_t isg6320_irq_count_b_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	int ret = 0;
	s16 max_diff_b_val = 0;

	if (data->irq_count) {
		ret = -1;
		max_diff_b_val = data->mul_ch->max_diff_b;
	} else {
		max_diff_b_val = data->mul_ch->max_normal_diff_b;
	}

	pr_info("[GRIP_%d] %s called\n", data->ic_num, __func__);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n", ret, data->irq_count,
			max_diff_b_val);
}

static ssize_t isg6320_irq_count_b_store(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	u8 onoff;
	int ret;

	ret = kstrtou8(buf, 10, &onoff);
	if (ret < 0) {
		pr_err("[GRIP_%d] invalid arg\n", data->ic_num);
		return count;
	}

	mutex_lock(&data->lock);

	if (onoff == 0) {
		data->abnormal_mode = OFF;
	} else if (onoff == 1) {
		data->abnormal_mode = ON;
		data->schedule_time = SHCEDULE_INTERVAL;
		data->irq_count = 0;
		data->mul_ch->max_diff_b = 0;
		data->mul_ch->max_normal_diff_b = 0;
	} else {
		pr_err("[GRIP_%d] invalid val %d\n", data->ic_num, onoff);
	}

	mutex_unlock(&data->lock);

	pr_info("[GRIP_%d] %s %d\n", data->ic_num, __func__, onoff);

	return count;
}

static ssize_t isg6320_sampling_freq_b_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	u8 buff;
	int sampling_freq;

	isg6320_i2c_read(data, ISG6320_SCANCTRL13_REG, &buff, 1);

	if (buff & 0x04)
		isg6320_i2c_read(data, ISG6320_NUM_OF_CLK_B, &buff, 1);
	else
		isg6320_i2c_read(data, ISG6320_NUM_OF_CLK, &buff, 1);

	sampling_freq = (int)(8000 / ((int)buff + 1));

	return snprintf(buf, PAGE_SIZE, "%dkHz\n", sampling_freq);
}

static ssize_t isg6320_unknown_state_2ch_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);
	
	return snprintf(buf, PAGE_SIZE, "%s\n",
		(data->mul_ch->is_unknown_mode == 1) ? "UNKNOWN" : "NORMAL");
}

#endif

static ssize_t isg6320_enable_store(struct device *dev,
					 struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	u8 enable;
	struct isg6320_data *data = dev_get_drvdata(dev);
	int pre_enable = data->enable;

	ret = kstrtou8(buf, 2, &enable);
	if (ret) {
		pr_err("[GRIP_%d] invalid arg\n", data->ic_num);
		return size;
	}

	pr_info("[GRIP_%d] new_val=%d old_val=%d\n", data->ic_num, (int)enable,
		pre_enable);

	if (pre_enable == enable)
		return size;

	isg6320_set_enable(data, (int)enable);

	return size;
}

static ssize_t isg6320_enable_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->enable);
}

#if !IS_ENABLED(CONFIG_SAMSUNG_PRODUCT_SHIP)
static ssize_t isg6320_debug_raw_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	u8 buff[6];
	u8 buff2[6];
	u16 temp;
	struct isg6320_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->lock);
	ret = isg6320_i2c_read(data, ISG6320_CDC16_A_H_REG, buff, sizeof(buff));
	if (ret < 0) {
		pr_info("[GRIP_%d] fail to get A data\n", data->ic_num);
	} else {
		ret = isg6320_i2c_read(data, ISG6320_CDC16_B_H_REG, buff2,
				sizeof(buff2));
		if (ret < 0) {
			pr_info("[GRIP_%d] fail to get B data\n", data->ic_num);
		} else {
			temp = ((u32)buff[0] << 8) | (u32)buff[1];
			if ((temp != 0) && (temp != 0xFFFF))
				data->debug_cdc[0] = temp;

			temp = ((u32)buff[2] << 8) | (u32)buff[3];
			if ((temp != 0) && (temp != 0xFFFF))
				data->debug_cdc[1] = temp;

			temp = ((u32)buff[4] << 8) | (u32)buff[5];
			if ((temp != 0) && (temp != 0xFFFF))
				data->debug_base[0] = temp;

			data->debug_diff[0] =
				(s32)data->debug_cdc[0] - (s32)data->debug_base[0];

			data->debug_diff[1] =
				(s32)data->debug_cdc[1] - (s32)data->debug_base[0];

			temp = ((u32)buff2[0] << 8) | (u32)buff2[1];
			if ((temp != 0) && (temp != 0xFFFF))
				data->debug_cdc[2] = temp;

			temp = ((u32)buff2[2] << 8) | (u32)buff2[3];
			if ((temp != 0) && (temp != 0xFFFF))
				data->debug_cdc[3] = temp;

			temp = ((u32)buff2[4] << 8) | (u32)buff2[5];
			if ((temp != 0) && (temp != 0xFFFF))
				data->debug_base[1] = temp;

			data->debug_diff[2] =
				(s32)data->debug_cdc[2] - (s32)data->debug_base[1];

			data->debug_diff[3] =
				(s32)data->debug_cdc[3] - (s32)data->debug_base[1];
		}
	}
	mutex_unlock(&data->lock);

	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", data->debug_cdc[0],
			data->debug_diff[0], data->debug_base[0], data->debug_cdc[1],
			data->debug_diff[1], data->debug_cdc[2], data->debug_diff[2],
			data->debug_base[1], data->debug_cdc[3], data->debug_diff[3]);
}

static ssize_t isg6320_debug_data_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d,%d,%d\n", data->cdc, data->base, data->diff);
}

static ssize_t isg6320_reg_update_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	int enable_backup;
	struct isg6320_data *data = dev_get_drvdata(dev);

	enable_backup = data->enable;

	if (enable_backup)
		isg6320_set_enable(data, OFF);
	isg6320_reset(data);
	isg6320_initialize(data);
	if (enable_backup)
		isg6320_set_enable(data, ON);

	return sprintf(buf, "OK\n");
}

#define DIRECT_CMD_WRITE			'w'
#define DIRECT_CMD_READ				'r'
#define DIRECT_BUF_COUNT			16
static ssize_t isg6320_direct_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	int i, count = 0;
	int ret = 0;
	int len;
	u8 addr;
	const int msg_len = 256;
	char msg[256];
	struct isg6320_data *data = dev_get_drvdata(dev);
	direct_info *direct = (direct_info *)&data->direct;
	u8 buf8[DIRECT_BUF_COUNT] = {0,};
	int max_len = DIRECT_BUF_COUNT;

	if (direct->cmd != DIRECT_CMD_READ)
		return sprintf(buf, "ex) echo r addr len size(display) > direct\n");

	len = direct->val;
	addr = direct->addr;

	while (len > 0) {
		if (len < max_len) max_len = len;

		ret = isg6320_i2c_read(data, addr, buf8, max_len);
		if (ret < 0) {
			count = sprintf(buf, "i2c read fail\n");
			break;
		}
		addr += max_len;

		for (i = 0; i < max_len; i++) {
			count += snprintf(msg, msg_len, "0x%02X ", buf8[i]);
			strncat(buf, msg, msg_len);
		}
		count += snprintf(msg, msg_len, "\n");
		strncat(buf, msg, msg_len);

		len -= max_len;
	}

	return count;
}

static ssize_t isg6320_direct_store(struct device *dev,
					 struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = -EPERM;
	u32 tmp1, tmp2;
	struct isg6320_data *data = dev_get_drvdata(dev);
	direct_info *direct = (direct_info *)&data->direct;

	sscanf(buf, "%c %x %x", &direct->cmd, &tmp1, &tmp2);

	direct->addr = tmp1;
	direct->val = tmp2;

	pr_info("[GRIP_%d] direct cmd: %c, addr: %x, val: %x\n", data->ic_num,
		direct->cmd, direct->addr, direct->val);

	if ((direct->cmd != DIRECT_CMD_WRITE) && (direct->cmd != DIRECT_CMD_READ)) {
		pr_err("[GRIP_%d] direct cmd is not correct!\n", data->ic_num);
		return size;
	}

	if (direct->cmd == DIRECT_CMD_WRITE) {
		ret = isg6320_i2c_write(data, direct->addr, direct->val);
		if (ret < 0)
			pr_err("[GRIP_%d] direct write fail\n", data->ic_num);
		else
			pr_info("[GRIP_%d] direct write addr: %x, val: %x\n", data->ic_num,
				direct->addr, direct->val);
	}

	return size;
}

static ssize_t isg6320_intr_debug_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	pr_info("[GRIP_%d] intr debug addr: 0x%x, count: %d\n",
		data->ic_num, data->irq_debug_addr, data->irq_debug_size);

	return sprintf(buf, "intr debug addr: 0x%x, count: %d\n",
			   data->irq_debug_addr, data->irq_debug_size);
}

static ssize_t isg6320_intr_debug_store(struct device *dev,
					 struct device_attribute *attr, const char *buf, size_t size)
{
	u32 tmp1;
	struct isg6320_data *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%x %d", &tmp1, &data->irq_debug_size) != 2) {
		pr_err("[GRIP_%d] The number of data are wrong\n", data->ic_num);

		return -EINVAL;
	}

	data->irq_debug_addr = tmp1;

	pr_info("[GRIP_%d] intr debug addr: 0x%x, count: %d\n",
		data->ic_num, data->irq_debug_addr, data->irq_debug_size);

	return size;
}

static ssize_t isg6320_cp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	u16 buff;
	struct isg6320_data *data = dev_get_drvdata(dev);

	ret = isg6320_i2c_read(data, ISG6320_A_COARSE_REG, (u8 *)&buff, 2);
	if (ret < 0) {
		pr_info("[GRIP_%d] fail to get cp\n", data->ic_num);
	} else {
		data->fine_coarse = buff;
		pr_info("[GRIP_%d] coarse: %04X\n", data->ic_num, data->fine_coarse);
	}

#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->multi_use) {
		ret = isg6320_i2c_read(data, ISG6320_B_COARSE_REG, (u8 *)&buff, 2);
		if (ret < 0) {
			pr_info("[GRIP_%d] [B] fail to get cp\n", data->ic_num);
		} else {
			data->mul_ch->fine_coarse_b = buff;
			pr_info("[GRIP_%d] [B] coarse: %04X\n", data->ic_num,
				data->mul_ch->fine_coarse_b);
		}

		return sprintf(buf, "%d%02d,%d%02d\n", (data->fine_coarse & 0xFF),
				(data->fine_coarse >> 8) & 0x3F, (data->mul_ch->fine_coarse_b & 0xFF),
				(data->mul_ch->fine_coarse_b >> 8) & 0x3F);
	}
#endif
	return sprintf(buf, "%d%02d,0\n", (data->fine_coarse & 0xFF),
			(data->fine_coarse >> 8) & 0x3F);
}

#define SCAN_INT			0x10
#define FAR_CLOSE_INT	   0x08
static ssize_t isg6320_scan_int_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	int ret;
	struct isg6320_data *data = dev_get_drvdata(dev);

	ret = isg6320_i2c_write(data, ISG6320_IRQCON_REG, SCAN_INT);
	if (ret < 0) {
		pr_err("[GRIP_%d] fail to set scan done int\n", data->ic_num);
		return sprintf(buf, "FAIL\n");
	} else {
		pr_info("[GRIP_%d] set scan done int\n", data->ic_num);
		return sprintf(buf, "OK\n");
	}
}

static ssize_t isg6320_far_close_int_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	int ret;
	struct isg6320_data *data = dev_get_drvdata(dev);

	ret = isg6320_i2c_write(data, ISG6320_IRQCON_REG, FAR_CLOSE_INT);
	if (ret < 0) {
		pr_err("[GRIP_%d] fail to set normal int\n", data->ic_num);
		return sprintf(buf, "FAIL\n");
	} else {
		pr_info("[GRIP_%d] set normal int\n", data->ic_num);
		return sprintf(buf, "OK\n");
	}
}

static ssize_t isg6320_toggle_enable_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	int enable;
	struct isg6320_data *data = dev_get_drvdata(dev);

	enable = (data->enable == OFF) ? ON : OFF;
	isg6320_set_enable(data, (int)enable);

	return sprintf(buf, "%d\n", data->enable);
}

static ssize_t isg6320_init_freq_test_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	data->freq_step = 1;
	data->freq_value = 0;

	return sprintf(buf, "OK\n");
}

static ssize_t isg6320_change_freq_step_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	data->freq_step++;

	if (data->freq_step > ISG6320_MAX_FREQ_STEP)
		data->freq_step = 1;

	return sprintf(buf, "%d\n", data->freq_step);
}

static ssize_t isg6320_change_freq_value_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	data->freq_value += data->freq_step;

	return sprintf(buf, "%d\n", data->freq_value);
}

static ssize_t isg6320_change_freq_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	pr_info("[GRIP_%d] %s\n", data->ic_num, __func__);

	mutex_lock(&data->lock);
	isg6320_i2c_write(data, ISG6320_SCANCTRL1_REG, ISG6320_DFE_ENABLE);
	usleep_range(10000, 10010);
	isg6320_i2c_write(data, ISG6320_SCANCTRL1_REG, ISG6320_SCAN_STOP);
	usleep_range(10000, 10010);
	isg6320_i2c_write(data, ISG6320_NUM_OF_CLK, data->freq_value);
	isg6320_i2c_write(data, ISG6320_SCANCTRL1_REG, ISG6320_CFCAL_START);
	mutex_unlock(&data->lock);

	msleep(400);

	return sprintf(buf, "OK\n");
}

#ifdef CONFIG_USE_IC_TYPE
static ssize_t isg6320_ic_type_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	int ret;
	u8 val;
	struct isg6320_data *data = dev_get_drvdata(dev);

	ret = isg6320_i2c_read(data, ISG6320_IC_TYPE_REG, &val, 1);
	if (ret < 0)
		pr_err("[GRIP_%d] fail to read IC Type(%d)\n", data->ic_num, ret);

	return sprintf(buf, "%s\n", (val == ES) ? "ES Sample" : "CS Sample");
}
#endif
#endif

#if !defined(CONFIG_SEC_FACTORY) && defined(CONFIG_SUPPORT_MCC_THRESHOLD_CHANGE)
static ssize_t isg6320_mcc_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret, mcc;
	struct isg6320_data *data = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 10, &mcc);
	if (ret) {
		pr_err("[GRIP_%d] %s - Invalid Argument(%d)\n", data->ic_num, __func__, ret);
		return count;
	}

	data->mcc = mcc;
	pr_info("[GRIP_%d] %s - mcc value %d\n", data->ic_num, __func__, data->mcc);

	if (data->ic_num == MAIN_GRIP)
		schedule_work(&data->mcc_work);

	return count;
}

static ssize_t isg6320_mcc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->mcc);
}

static DEVICE_ATTR(mcc, 0664, isg6320_mcc_show, isg6320_mcc_store);
#endif

static ssize_t isg6320_noti_enable_store(struct device *dev,
				     struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	u8 enable;
	struct isg6320_data *data = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 2, &enable);
	if (ret) {
		pr_err("[GRIP_%d] %s - invalid argument\n", data->ic_num, __func__);
		return size;
	}

	pr_info("[GRIP_%d] %s - new_value=%d\n", data->ic_num, __func__, (int)enable);

	data->noti_enable = enable;

	if (data->noti_enable)
		isg6320_enter_unknown_mode(data, TYPE_BOOT);

	return size;
}

static ssize_t isg6320_noti_enable_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	pr_info("[GRIP_%d] %s - noti_enable = %d\n", data->ic_num, __func__, data->noti_enable);
	return sprintf(buf, "%d\n", data->noti_enable);
}

static ssize_t isg6320_country_code_store(struct device *dev,
				     struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	u8 country_code;
	struct isg6320_data *data = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 2, &country_code);
	if (ret) {
		pr_err("[GRIP_%d] %s - invalid argument\n", data->ic_num, __func__);
		return size;
	}

	pr_info("[GRIP_%d] %s - country_code=%d\n", data->ic_num, __func__, (int)country_code);

	data->country_code = country_code;

	return size;
}

static ssize_t isg6320_country_code_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	pr_info("[GRIP_%d] %s - country_code = %s\n", data->ic_num, __func__, 
				data->country_code == 1 ? "KOR" : "ETC(EUR,JPN,CHN)");
	return sprintf(buf, "%d\n", data->country_code);
}

static DEVICE_ATTR(name, 0444, isg6320_name_show, NULL);
static DEVICE_ATTR(vendor, 0444, isg6320_vendor_show, NULL);
static DEVICE_ATTR(mode, 0444, isg6320_mode_show, NULL);
static DEVICE_ATTR(manual_acal, 0444, isg6320_manual_acal_show, NULL);
static DEVICE_ATTR(calibration, 0444, isg6320_acal_show, NULL);
static DEVICE_ATTR(onoff, 0664, isg6320_onoff_show, isg6320_onoff_store);
static DEVICE_ATTR(reset, 0444, isg6320_sw_reset_show, NULL);
static DEVICE_ATTR(normal_threshold, 0664,
		   isg6320_normal_threshold_show, isg6320_normal_threshold_store);
static DEVICE_ATTR(raw_data, 0444, isg6320_raw_data_show, NULL);
static DEVICE_ATTR(diff_avg, 0444, isg6320_diff_avg_show, NULL);
static DEVICE_ATTR(cdc_avg, 0444, isg6320_cdc_avg_show, NULL);
static DEVICE_ATTR(useful_avg, 0444, isg6320_cdc_avg_show, NULL);
static DEVICE_ATTR(ch_state, 0444, isg6320_ch_state_show, NULL);
static DEVICE_ATTR(hysteresis, 0444, isg6320_hysteresis_show, NULL);
static DEVICE_ATTR(sampling_freq, 0440, isg6320_sampling_freq_show, NULL);
static DEVICE_ATTR(isum, 0444, isg6320_isum_show, NULL);
static DEVICE_ATTR(scan_period, 0444, isg6320_scan_period_show, NULL);
static DEVICE_ATTR(analog_gain, 0444, isg6320_again_show, NULL);
static DEVICE_ATTR(cdc_up, 0444, isg6320_cdc_up_coef_show, NULL);
static DEVICE_ATTR(cdc_down, 0444, isg6320_cdc_down_coef_show, NULL);
static DEVICE_ATTR(temp_enable, 0444, isg6320_temp_enable_show, NULL);
static DEVICE_ATTR(irq_count, 0664,
		   isg6320_irq_count_show, isg6320_irq_count_store);
static DEVICE_ATTR(motion, 0664, isg6320_motion_show, isg6320_motion_store);
static DEVICE_ATTR(unknown_state, 0664,
	isg6320_unknown_state_show, isg6320_unknown_state_store);
static DEVICE_ATTR(noti_enable, 0664, isg6320_noti_enable_show, isg6320_noti_enable_store);
static DEVICE_ATTR(country_code, 0664, isg6320_country_code_show, isg6320_country_code_store);
//static DEVICE_ATTR(cml, S_IRUGO, isg6320_cml_show, NULL);
#ifdef CONFIG_USE_MULTI_CHANNEL
static DEVICE_ATTR(normal_threshold_b, 0664,
		isg6320_normal_threshold_b_show, isg6320_normal_threshold_b_store);
static DEVICE_ATTR(raw_data_b, 0444, isg6320_raw_data_b_show, NULL);
static DEVICE_ATTR(debug_data_b, 0444, isg6320_debug_data_b_show, NULL);
static DEVICE_ATTR(diff_avg_b, 0444, isg6320_diff_avg_b_show, NULL);
static DEVICE_ATTR(cdc_avg_b, 0444, isg6320_cdc_avg_b_show, NULL);
static DEVICE_ATTR(useful_avg_b, 0444, isg6320_cdc_avg_b_show, NULL);
static DEVICE_ATTR(hysteresis_b, 0444, isg6320_hysteresis_b_show, NULL);
static DEVICE_ATTR(isum_b, 0444, isg6320_isum_b_show, NULL);
static DEVICE_ATTR(analog_gain_b, 0444, isg6320_again_b_show, NULL);
static DEVICE_ATTR(cdc_down_b, 0444, isg6320_cdc_down_coef_b_show, NULL);
static DEVICE_ATTR(cdc_up_b, 0444, isg6320_cdc_up_coef_b_show, NULL);
static DEVICE_ATTR(temp_enable_b, 0444, isg6320_temp_enable_b_show, NULL);
static DEVICE_ATTR(irq_count_b, 0664,
		   isg6320_irq_count_b_show, isg6320_irq_count_b_store);
static DEVICE_ATTR(sampling_freq_b, 0440, isg6320_sampling_freq_b_show, NULL);
static DEVICE_ATTR(unknown_state_2ch, 0664,
	isg6320_unknown_state_2ch_show, isg6320_unknown_state_store);
#endif
#if !IS_ENABLED(CONFIG_SAMSUNG_PRODUCT_SHIP)
static DEVICE_ATTR(debug_raw_data, 0444, isg6320_debug_raw_data_show, NULL);
static DEVICE_ATTR(debug_data, 0444, isg6320_debug_data_show, NULL);
static DEVICE_ATTR(reg_update, 0444, isg6320_reg_update_show, NULL);
static DEVICE_ATTR(direct, 0664, isg6320_direct_show, isg6320_direct_store);
static DEVICE_ATTR(intr_debug, 0664,
		   isg6320_intr_debug_show, isg6320_intr_debug_store);
static DEVICE_ATTR(cp, 0444, isg6320_cp_show, NULL);
static DEVICE_ATTR(scan_int, 0444, isg6320_scan_int_show, NULL);
static DEVICE_ATTR(far_close_int, 0444, isg6320_far_close_int_show, NULL);
static DEVICE_ATTR(toggle_enable, 0444, isg6320_toggle_enable_show, NULL);
static DEVICE_ATTR(init_freq_test, 0444, isg6320_init_freq_test_show, NULL);
static DEVICE_ATTR(change_freq_step, 0444,
		   isg6320_change_freq_step_show, NULL);
static DEVICE_ATTR(change_freq_value, 0444,
		   isg6320_change_freq_value_show, NULL);
static DEVICE_ATTR(change_freq, 0444, isg6320_change_freq_show, NULL);
#ifdef CONFIG_USE_IC_TYPE
static DEVICE_ATTR(adc_filt, 0440, isg6320_ic_type_show, NULL);
#endif
#endif

static struct device_attribute *sensor_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_mode,
	&dev_attr_manual_acal,
	&dev_attr_calibration,
	&dev_attr_onoff,
	&dev_attr_reset,
	&dev_attr_normal_threshold,
	&dev_attr_raw_data,
	&dev_attr_diff_avg,
	&dev_attr_useful_avg,
	&dev_attr_cdc_avg,
	&dev_attr_ch_state,
	&dev_attr_hysteresis,
	&dev_attr_sampling_freq,
	&dev_attr_isum,
	&dev_attr_scan_period,
	&dev_attr_analog_gain,
	&dev_attr_cdc_up,
	&dev_attr_cdc_down,
	&dev_attr_temp_enable,
	&dev_attr_irq_count,
	&dev_attr_motion,
	&dev_attr_unknown_state,
	&dev_attr_noti_enable,
	&dev_attr_country_code,
	//&dev_attr_cml,
#if !defined(CONFIG_SEC_FACTORY) && defined(CONFIG_SUPPORT_MCC_THRESHOLD_CHANGE)
	&dev_attr_mcc,
#endif
#if !IS_ENABLED(CONFIG_SAMSUNG_PRODUCT_SHIP)
	&dev_attr_debug_raw_data,
	&dev_attr_debug_data,
	&dev_attr_reg_update,
	&dev_attr_direct,
	&dev_attr_intr_debug,
	&dev_attr_cp,
	&dev_attr_scan_int,
	&dev_attr_far_close_int,
	&dev_attr_toggle_enable,
	&dev_attr_init_freq_test,
	&dev_attr_change_freq_step,
	&dev_attr_change_freq_value,
	&dev_attr_change_freq,
#ifdef CONFIG_USE_IC_TYPE
	&dev_attr_adc_filt,
#endif
#endif
	NULL,
};

#ifdef CONFIG_USE_MULTI_CHANNEL
static struct device_attribute *multi_sensor_attrs[] = {
	&dev_attr_normal_threshold_b,
	&dev_attr_raw_data_b,
	&dev_attr_debug_data_b,
	&dev_attr_diff_avg_b,
	&dev_attr_cdc_avg_b,
	&dev_attr_useful_avg_b,
	&dev_attr_hysteresis_b,
	&dev_attr_isum_b,
	&dev_attr_analog_gain_b,
	&dev_attr_cdc_down_b,
	&dev_attr_cdc_up_b,
	&dev_attr_temp_enable_b,
	&dev_attr_irq_count_b,
	&dev_attr_sampling_freq_b,
	&dev_attr_unknown_state_2ch,
	NULL,
};
#endif

static DEVICE_ATTR(enable, 0664, isg6320_enable_show, isg6320_enable_store);

static struct attribute *isg6320_attributes[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute_group isg6320_attribute_group = {
	.attrs = isg6320_attributes,
};

#ifdef ISG6320_INIT_DELAYEDWORK
static void init_work_func(struct work_struct *work)
{
	struct delayed_work *delayed_work = to_delayed_work(work);
	struct isg6320_data *data = container_of(delayed_work,
						  struct isg6320_data, init_work);

	isg6320_initialize(data);
	isg6320_set_debug_work(data, ON, SHCEDULE_INTERVAL);
}
#endif

static void isg6320_check_first_working(struct isg6320_data *data, int channel_num)
{
	if (data->noti_enable && data->motion) {
		if (channel_num == 1) {
			if (data->normal_th < data->diff) {
				if (!data->first_working) {
					data->first_working = true;
					pr_info("[GRIP_%d] first working detected %d\n", data->ic_num, data->diff);
				}
			} else {
				if (data->first_working &&
					(data->is_unknown_mode == UNKNOWN_ON)) {
					data->is_unknown_mode = UNKNOWN_OFF;
					pr_info("[GRIP_%d] Release detected %d unknown mode off\n", data->ic_num, data->diff);
				}
			}
		}
#ifdef CONFIG_USE_MULTI_CHANNEL
		else if (channel_num == 2) {
			if (data->multi_use) {
				if (data->mul_ch->normal_th_b < data->mul_ch->diff_b) {
					if (!data->mul_ch->first_working) {
						data->mul_ch->first_working = true;
						pr_info("[GRIP_%d] [B] first working detected %d\n", data->ic_num, data->mul_ch->diff_b);
					}
				} else {
					if (data->mul_ch->first_working &&
						(data->mul_ch->is_unknown_mode == UNKNOWN_ON)) {
						data->mul_ch->is_unknown_mode = UNKNOWN_OFF;
						pr_info("[GRIP_%d] [B] Release detected %d unknown mode off\n", data->ic_num, data->mul_ch->diff_b);
					}
				}
			}
		}
#endif
	}
}

static void cal_work_func(struct work_struct *work)
{

	struct delayed_work *delayed_work = to_delayed_work(work);
	struct isg6320_data *data = container_of(delayed_work,
						  struct isg6320_data, cal_work);
	bool force_cal = false;
	int ret = 0;

	ret = check_hallic_state(WACOM_HALLIC_PATH, data->wacom_hall_ic);
	if (ret < 0)
		pr_err("[GRIP_%d] wacom hallic detect fail = %d\n", data->ic_num, ret);

	if (strcmp(data->wacom_hall_ic, "CLOSE") == 0) {
		if (data->wacom_hall_flag) {
			pr_info("[GRIP_%d] wacom hall IC is closed\n", data->ic_num);
			schedule_work(&data->cfcal_work);
			data->wacom_hall_flag = 0;
		}
	} else {
		data->wacom_hall_flag = 1;
	}

	if (data->abnormal_mode && data->enable == ON) {
		ret = isg6320_get_raw_data(data, true);
		if (ret < 0) {
			schedule_delayed_work(&data->cal_work, msecs_to_jiffies(SHCEDULE_INTERVAL));
			return;
		}
		if (data->max_normal_diff < data->diff)
			data->max_normal_diff = data->diff;
#ifdef CONFIG_USE_MULTI_CHANNEL
		if (data->multi_use) {
			if (data->mul_ch->max_normal_diff_b < data->mul_ch->diff_b)
				data->mul_ch->max_normal_diff_b = data->mul_ch->diff_b;
		}
#endif
	} else {
		ret = isg6320_get_raw_data(data, false);
		if (data->is_unknown_mode == UNKNOWN_ON && data->motion && !data->first_working)
			isg6320_check_first_working(data, 1);
#ifdef CONFIG_USE_MULTI_CHANNEL
		if (data->multi_use) {
			if (data->mul_ch->is_unknown_mode == UNKNOWN_ON && data->motion && !data->mul_ch->first_working)
				isg6320_check_first_working(data, 2);
		}
#endif
		if (ret < 0) {
			schedule_delayed_work(&data->cal_work, msecs_to_jiffies(SHCEDULE_INTERVAL));
			return;
		}
	}

	if (data->cdc < data->cfcal_th) {
		pr_info("[GRIP_%d] cdc %d cfcal_th %d\n", data->ic_num, data->cdc,
			data->cfcal_th);
		force_cal = true;
	}
#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->multi_use) {
		if (data->mul_ch->cdc_b < data->mul_ch->cfcal_th_b) {
			pr_info("[GRIP_%d] [B] cdc %d cfcal_th_b %d\n", data->ic_num,
				data->mul_ch->cdc_b, data->mul_ch->cfcal_th_b);
			force_cal = true;
		}
	}
#endif
	if (force_cal) {
		data->schedule_time = SHCEDULE_INTERVAL;
		schedule_work(&data->cfcal_work);
	} else if (data->abnormal_mode == OFF) {
#if 0
		if (data->schedule_time < SHCEDULE_INTERVAL_MAX)
			data->schedule_time += SHCEDULE_INTERVAL + (data->ic_num << 2);
#else
		data->schedule_time = SHCEDULE_INTERVAL;
#endif
	}

	schedule_delayed_work(&data->cal_work,
			msecs_to_jiffies(data->schedule_time));
}

#if (IS_ENABLED(CONFIG_CCIC_NOTIFIER) || IS_ENABLED(CONFIG_PDIC_NOTIFIER)) && IS_ENABLED(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
static int isg6320_ccic_handle_notification(struct notifier_block *nb,
					     unsigned long action, void *data)
{
	PD_NOTI_ATTACH_TYPEDEF usb_typec_info = *(PD_NOTI_ATTACH_TYPEDEF *)data;
	struct isg6320_data *pdata = container_of(nb, struct isg6320_data, cpuidle_ccic_nb);

	if (usb_typec_info.id != PDIC_NOTIFY_ID_ATTACH)
		return 0;

	if (pdata->pre_attach == usb_typec_info.attach)
		return 0;

	pr_info("[GRIP_%d] src %d id %d attach %d rprd %d\n", pdata->ic_num,
		usb_typec_info.src, usb_typec_info.id, usb_typec_info.attach, usb_typec_info.rprd);

	//usb host (otg)
	if (usb_typec_info.rprd == PDIC_NOTIFY_HOST) {
		pdata->otg_attach_state = usb_typec_info.rprd;
		pr_info("[GRIP_%d] otg attach, grip reset skip\n", pdata->ic_num);
	} else if (pdata->otg_attach_state) {
		pdata->otg_attach_state = usb_typec_info.rprd;
		pr_info("[GRIP_%d] otg detach, grip reset skip\n", pdata->ic_num);
	} else {
		if (pdata->initialized == ON) {
			schedule_work(&pdata->cfcal_work);
			isg6320_enter_unknown_mode(pdata, TYPE_USB);
		}
	}

	pdata->pre_attach = usb_typec_info.attach;

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_HALL_NOTIFIER)
static int isg6320_hall_notifier(struct notifier_block *nb,
				unsigned long action, void *hall_data)
{
	struct hall_notifier_context *hall_notifier;
	struct isg6320_data *data =
			container_of(nb, struct isg6320_data, hall_nb);
	hall_notifier = hall_data;

	if (action == HALL_ATTACH) {
		pr_info("[GRIP_%d] %s attach\n", data->ic_num, hall_notifier->name);
		schedule_work(&data->cfcal_work);
	} else {
		pr_info("[GRIP_%d] %s detach\n",
			data->ic_num, hall_notifier->name);
	}
	isg6320_enter_unknown_mode(data, TYPE_HALL);
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_FLIP_COVER_DETECTOR_NOTIFIER)
static int isg6320_fcd_notifier(struct notifier_block *nb,
				unsigned long action, void *fcd_data)
{
	struct isg6320_data *data = container_of(nb, struct isg6320_data, fcd_nb);

	if (action == FCD_ATTACH) {
		pr_info("[GRIP_%d] fcd attach\n", data->ic_num);
		schedule_work(&data->cfcal_work);
		isg6320_enter_unknown_mode(data, TYPE_COVER);
	}

	return 0;
}
#endif

static int isg6320_parse_dt(struct isg6320_data *data, struct device *dev)
{
	struct device_node *node = dev->of_node;
	enum of_gpio_flags flags;
	int ret = 0;

	data->ldo_en = of_get_named_gpio_flags(node, "isg6320,ldo-en", 0, &flags);
	if (data->ldo_en < 0) {
		pr_err("[GRIP_%d] get ldo_en err\n", data->ic_num);
		return -ENODEV;
	} else {
		pr_info("[GRIP_%d] get ldo_en %d %d\n", data->ic_num, data->ldo_en, gpio_get_value_cansleep(data->ldo_en));
	}

	if (data->ic_num == MAIN_GRIP)
		data->gpio_int = of_get_named_gpio_flags(node, "isg6320,irq-gpio", 0, &flags);
#if defined(CONFIG_SENSORS_ISG6320_SUB)
	else if (data->ic_num == SUB_GRIP)
		data->gpio_int = of_get_named_gpio_flags(node, "isg6320_sub,irq-gpio", 0,
							&flags);
#endif
#if defined(CONFIG_SENSORS_ISG6320_WIFI)
	else if (data->ic_num == WIFI_GRIP)
		data->gpio_int = of_get_named_gpio_flags(node, "isg6320_wifi,irq-gpio", 0,
							&flags);
#endif
	if (data->gpio_int < 0) {
		pr_err("[GRIP_%d] get gpio_int err\n", data->ic_num);
		return -ENODEV;
	}

	pr_info("[GRIP_%d] gpio_int:%d\n", data->ic_num, data->gpio_int);

	if (data->ic_num == MAIN_GRIP)
		ret = of_property_read_u32(node, "isg6320,reg_num", &data->reg_size);
#if defined(CONFIG_SENSORS_ISG6320_SUB)
	else if (data->ic_num == SUB_GRIP)
		ret = of_property_read_u32(node, "isg6320_sub,reg_num", &data->reg_size);
#endif
#if defined(CONFIG_SENSORS_ISG6320_WIFI)
	else if (data->ic_num == WIFI_GRIP)
		ret = of_property_read_u32(node, "isg6320_wifi,reg_num", &data->reg_size);
#endif

	if(ret < 0)
		data->reg_size = 68;

	if (data->ic_num == MAIN_GRIP)
		ret = of_property_read_u8_array(node, "isg6320,set_reg", data->setup_reg,
						data->reg_size * 2);
#if defined(CONFIG_SENSORS_ISG6320_SUB)
	else if (data->ic_num == SUB_GRIP)
		ret = of_property_read_u8_array(node, "isg6320_sub,set_reg", data->setup_reg,
						data->reg_size * 2);
#endif
#if defined(CONFIG_SENSORS_ISG6320_WIFI)
	else if (data->ic_num == WIFI_GRIP)
		ret = of_property_read_u8_array(node, "isg6320_wifi,set_reg", data->setup_reg,
						data->reg_size * 2);
#endif
	if (ret < 0) {
		pr_err("[GRIP_%d] set_reg fail\n", data->ic_num);
		data->setup_reg_exist = false;
	} else {
		data->setup_reg_exist = true;
	}
#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->ic_num == MAIN_GRIP)
		ret = of_property_read_u32(node, "isg6320,multi_use", &data->multi_use);
#if defined(CONFIG_SENSORS_ISG6320_SUB)
	else if (data->ic_num == SUB_GRIP)
		ret = of_property_read_u32(node, "isg6320_sub,multi_use", &data->multi_use);
#endif
#if defined(CONFIG_SENSORS_ISG6320_WIFI)
	else if (data->ic_num == WIFI_GRIP)
		ret = of_property_read_u32(node, "isg6320_wifi,multi_use", &data->multi_use);
#endif
	if (ret < 0) {
		pr_err("[GRIP_%d] multi_use set err\n", data->ic_num);
		data->multi_use = 0;
	}
#if !defined(CONFIG_SEC_FACTORY) && defined(CONFIG_SUPPORT_MCC_THRESHOLD_CHANGE)
	if (data->ic_num == MAIN_GRIP) {
		ret = of_property_read_u8(node, "isg6320,mcc_threshold", &data->mcc_threshold);
		if (ret < 0) {
			pr_err("[GRIP_%d] mcc_threshold set err\n", data->ic_num);
			data->mcc_threshold = 0;
		}
		pr_info("[GRIP_%d] mcc_threshold = 0x%X\n", data->ic_num, data->mcc_threshold);

		ret = of_property_read_u8(node, "isg6320,mcc_hysteresis", &data->mcc_hysteresis);
		if (ret < 0) {
			pr_err("[GRIP_%d] mcc_hysteresis set err\n", data->ic_num);
			data->mcc_hysteresis = 0;
		}
		pr_info("[GRIP_%d] mcc_hysteresis = 0x%X\n", data->ic_num, data->mcc_hysteresis);
	}
#endif
#endif

	return 0;
}

static int isg6320_gpio_init(struct isg6320_data *data)
{
	int ret = 0;

	ret = gpio_request(data->gpio_int, "isg6320_irq");
	if (ret < 0) {
		pr_err("[GRIP_%d] gpio %d req fail\n", data->ic_num, data->gpio_int);
		return ret;
	}

	ret = gpio_direction_input(data->gpio_int);
	if (ret < 0) {
		pr_err("[GRIP_%d] fail to set gpio %d(%d)\n", data->ic_num, data->gpio_int,
			   ret);
		gpio_free(data->gpio_int);
		return ret;
	}

	if (data->ic_num == MAIN_GRIP) {
		ret = gpio_request(data->ldo_en, "isg6320_ldo-en");
		if (ret < 0) {
			pr_err("[GRIP_%d] gpio %d request failed\n", data->ic_num, data->ldo_en);
			return ret;
		}

		ret = gpio_direction_output(data->ldo_en, 1);
		if (ret < 0) {
			pr_err("[GRIP_%d] failed to set gpio %d(%d)\n", data->ic_num, data->ldo_en,
				   ret);
			gpio_free(data->ldo_en);
			return ret;
		} else {
			pr_info("[GRIP_%d] set gpio %d as output (%d)\n", data->ic_num, data->ldo_en, gpio_get_value_cansleep(data->ldo_en));
			msleep(100);
		}
	}

	return ret;
}

#ifdef CONFIG_USE_MULTI_CHANNEL
static int isg6320_sensor_attr_offset(void)
{
	int i;
	int offset_max = SENSOR_ATTR_SIZE - 
				(sizeof(multi_sensor_attrs) / sizeof(ssize_t *));

	for (i = 0; i < offset_max; i++) {
		if (sensor_attrs[i] == NULL)
			return i;
	}

	return -1;
}
#endif

static int isg6320_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int ret = -ENODEV;
	struct isg6320_data *data;
	struct input_dev *input_dev;
	struct input_dev *noti_input_dev;
	int ic_num = MAIN_GRIP;
#ifdef CONFIG_USE_MULTI_CHANNEL
	struct device_attribute *grip_sensor_attrs[SENSOR_ATTR_SIZE];
#endif
	if (strcmp(client->name, "isg6320") == 0)
		ic_num = MAIN_GRIP;
#if defined(CONFIG_SENSORS_ISG6320_SUB)
	else if (strcmp(client->name, "isg6320_sub") == 0)
		ic_num = SUB_GRIP;
#endif
#if defined(CONFIG_SENSORS_ISG6320_WIFI)
	else if (strcmp(client->name, "isg6320_wifi") == 0)
		ic_num = WIFI_GRIP;
#endif
	else
		goto err;

	pr_info("[GRIP_%d] %s # probe #\n", ic_num, device_name[ic_num]);	

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_info("[GRIP_%d] i2c_check_functionality err\n", ic_num);
		goto err;
	}

	data = kzalloc(sizeof(struct isg6320_data), GFP_KERNEL);
	if (!data) {
		pr_info("[GRIP_%d] fail to alloc mem\n", ic_num);
		goto err_kzalloc;
	}

	data->ic_num = ic_num;

	ret = isg6320_parse_dt(data, &client->dev);
	if (ret) {
		pr_err("[GRIP_%d] fail to parse dt\n", data->ic_num);
		goto err_parse_dt;
	}

	pr_info("[GRIP_%d] multi_channel : %d", data->ic_num, data->multi_use);
#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->multi_use) {
		data->mul_ch = kzalloc(sizeof(struct multi_channel), GFP_KERNEL);
		if (!data->mul_ch) {
			pr_err("[GRIP_%d] multi_channel alloc failed", data->ic_num);
			data->multi_use = 0;
		}
	}
#endif

	ret = isg6320_gpio_init(data);
	if (ret) {
		pr_err("[GRIP_%d] fail to init sys\n", data->ic_num);
		goto err_gpio_init;
	}

	data->client = client;
	i2c_set_clientdata(client, data);

	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("[GRIP_%d] input_allocate_device fail\n", data->ic_num);
		goto err_input_alloc;
	}

	data->dev = &client->dev;
	data->input_dev = input_dev;

	input_dev->name = module_name[data->ic_num];
	input_dev->id.bustype = BUS_I2C;

	input_set_capability(input_dev, EV_REL, REL_MISC);
	input_set_capability(input_dev, EV_REL, REL_X);
#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->multi_use) {
		input_set_capability(input_dev, EV_REL, REL_DIAL);
		input_set_capability(input_dev, EV_REL, REL_Y);
	}
#endif
	input_set_drvdata(input_dev, data);

	noti_input_dev = input_allocate_device();
	if (!noti_input_dev) {
		pr_err("[GRIP_%d] input_allocate_device failed\n", data->ic_num);
		goto err_noti_input_alloc;
	}

	data->dev = &client->dev;
	data->noti_input_dev = noti_input_dev;

	noti_input_dev->name = NOTI_MODULE_NAME;
	noti_input_dev->id.bustype = BUS_I2C;

	input_set_capability(noti_input_dev, EV_REL, REL_X);
	input_set_drvdata(noti_input_dev, data);

	ret = isg6320_reset(data);
	if (ret < 0) {
		pr_err("[GRIP_%d] reset fail\n", data->ic_num);
		goto err_soft_reset;
	}

	data->skip_data = false;
	data->state = FAR;
	data->cfcal_th = ISG6320_CS_RESET_CONDITION;
	data->schedule_time = SHCEDULE_INTERVAL;

	data->is_unknown_mode = UNKNOWN_OFF;
	data->first_working = false;
#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->multi_use) {
		data->mul_ch->is_unknown_mode = UNKNOWN_OFF;
		data->mul_ch->first_working = false;
	}
#endif
	data->motion = 1;

#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->multi_use) {
		data->mul_ch->state_b = FAR;
		data->mul_ch->cfcal_th_b = ISG6320_CS_RESET_CONDITION;
	}
#endif
#if !IS_ENABLED(CONFIG_SAMSUNG_PRODUCT_SHIP)
	data->freq_step = 1;
#endif

	client->irq = gpio_to_irq(data->gpio_int);
	ret = request_threaded_irq(client->irq, NULL, isg6320_irq_thread,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT, device_name[data->ic_num], data);

	if (ret < 0) {
		pr_err("[GRIP_%d] fail to reg client->irq %d err %d\n", client->irq, data->ic_num, ret);
		goto err_irq;
	}
	disable_irq(client->irq);
	mutex_init(&data->lock);

	ret = input_register_device(input_dev);
	if (ret) {
		input_free_device(input_dev);
		pr_err("[GRIP_%d] fail to reg input dev %d\n", data->ic_num, ret);
		goto err_register_input_dev;
	}

	ret = sensors_create_symlink(&input_dev->dev.kobj,
					 input_dev->name);
	if (ret < 0) {
		pr_err("[GRIP_%d] fail to create symlink %d\n", data->ic_num, ret);
		goto err_create_symlink;
	}

	ret = sysfs_create_group(&data->input_dev->dev.kobj, &isg6320_attribute_group);
	if (ret < 0) {
		pr_err("[GRIP_%d] fail to create sysfs group (%d)\n", data->ic_num, ret);
		goto err_sysfs_create_group;
	}

#ifdef CONFIG_USE_MULTI_CHANNEL
	memcpy(grip_sensor_attrs, sensor_attrs, sizeof(sensor_attrs));
	if (data->multi_use) {
		int offset = isg6320_sensor_attr_offset();

		if (offset < 0) {
			data->multi_use = 0;
			pr_err("[GRIP_%d] fail mem size of attr is exceeded\n",
				data->ic_num);
		} else
			memcpy(grip_sensor_attrs + offset, multi_sensor_attrs, sizeof(multi_sensor_attrs));
	}
	ret = sensors_register(&data->dev, data, grip_sensor_attrs,
				(char *)module_name[data->ic_num]);
#else
	ret = sensors_register(&data->dev, data, sensor_attrs,
				(char *)module_name[data->ic_num]);
#endif

	if (ret) {
		pr_err("[GRIP_%d] fail to reg sensor(%d)\n", data->ic_num, ret);
		goto err_sensor_register;
	}
	ret = input_register_device(noti_input_dev);
	if (ret) {
		input_free_device(noti_input_dev);
		pr_err("[GRIP_U] failed to register input dev for noti (%d)\n", ret);
		goto err_register_input_dev_noti;
	}

	data->grip_ws = wakeup_source_register(&client->dev, "grip_wake_lock");
	INIT_WORK(&data->irq_work, irq_work_func);
	INIT_WORK(&data->cfcal_work, cfcal_work_func);
	INIT_DELAYED_WORK(&data->cal_work, cal_work_func);
#if !defined(CONFIG_SEC_FACTORY) && defined(CONFIG_SUPPORT_MCC_THRESHOLD_CHANGE)
	INIT_WORK(&data->mcc_work, mcc_work_func);
#endif
#ifdef ISG6320_INIT_DELAYEDWORK
	INIT_DELAYED_WORK(&data->init_work, init_work_func);
	schedule_delayed_work(&data->init_work, msecs_to_jiffies(SHCEDULE_INTERVAL));
#else
	isg6320_initialize(data);
	isg6320_set_debug_work(data, ON, SHCEDULE_INTERVAL);
#endif
#if IS_ENABLED(CONFIG_PDIC_NOTIFIER) && IS_ENABLED(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
	data->pdic_status = OFF;
	data->pdic_pre_attach = 0;
	manager_notifier_register(&data->cpuidle_ccic_nb,
								isg6320_ccic_handle_notification,
								MANAGER_NOTIFY_PDIC_SENSORHUB);
#endif

#if IS_ENABLED(CONFIG_FLIP_COVER_DETECTOR_NOTIFIER)
	pr_info("[GRIP_%d] reg fcd notifier\n", data->ic_num);
	data->fcd_nb.priority = 1;
	data->fcd_nb.notifier_call = isg6320_fcd_notifier;
	fcd_notifier_register(&data->fcd_nb);
#endif

#if IS_ENABLED(CONFIG_HALL_NOTIFIER)
	pr_info("[GRIP_%d] reg hall notifier\n", data->ic_num);
	data->hall_nb.priority = 1;
	data->hall_nb.notifier_call = isg6320_hall_notifier;
	hall_notifier_register(&data->hall_nb);
#else
	data->hall_flag = 1;
	data->wacom_hall_flag = 1;
#endif

	pr_info("[GRIP_%d] # probe done #\n", data->ic_num);

	return 0;

err_sensor_register:
err_register_input_dev_noti:
	sysfs_remove_group(&input_dev->dev.kobj, &isg6320_attribute_group);
err_sysfs_create_group:
	sensors_remove_symlink(&data->input_dev->dev.kobj, data->input_dev->name);
err_create_symlink:
	input_unregister_device(input_dev);
err_register_input_dev:
	mutex_destroy(&data->lock);
	free_irq(client->irq, data);
err_irq:
err_soft_reset:
err_noti_input_alloc:
err_input_alloc:
	gpio_free(data->gpio_int);
err_gpio_init:
err_parse_dt:
#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->multi_use)
		kfree(data->mul_ch);
#endif
	kfree(data);
err_kzalloc:
err:
	pr_err("[GRIP_%d] # probe fail \n", ic_num);

	return -ENODEV;
}

static int isg6320_remove(struct i2c_client *client)
{
	struct isg6320_data *data = i2c_get_clientdata(client);

	pr_info("[GRIP_%d] %s\n", data->ic_num, __func__);

	isg6320_set_debug_work(data, OFF, 0);
	if (data->enable == ON)
		isg6320_set_enable(data, OFF);

	free_irq(client->irq, data);
	gpio_free(data->gpio_int);

	wakeup_source_unregister(data->grip_ws);
	sensors_unregister(data->dev, sensor_attrs);
	sensors_remove_symlink(&data->input_dev->dev.kobj, data->input_dev->name);
	sysfs_remove_group(&data->input_dev->dev.kobj, &isg6320_attribute_group);
	input_unregister_device(data->input_dev);
	mutex_destroy(&data->lock);

#ifdef CONFIG_USE_MULTI_CHANNEL
	if (data->multi_use)
		kfree(data->mul_ch);
#endif
#if IS_ENABLED(CONFIG_FLIP_COVER_DETECTOR_NOTIFIER)
	fcd_notifier_unregister(&data->fcd_nb);
#endif
	kfree(data);

	return 0;
}

static int isg6320_suspend(struct device *dev)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	data->in_suspend = true;
	pr_info("[GRIP_%d] %s\n", data->ic_num, __func__);
	isg6320_set_debug_work(data, OFF, 0);

	return 0;
}

static int isg6320_resume(struct device *dev)
{
	struct isg6320_data *data = dev_get_drvdata(dev);

	pr_info("[GRIP_%d] %s\n", data->ic_num, __func__);
	isg6320_set_debug_work(data, ON, SHCEDULE_INTERVAL + (data->ic_num << 4));
	data->in_suspend = false;

	return 0;
}

static void isg6320_shutdown(struct i2c_client *client)
{
	struct isg6320_data *data = i2c_get_clientdata(client);

	pr_info("[GRIP_%d] %s\n", data->ic_num, __func__);

	isg6320_set_debug_work(data, OFF, 0);
	if (data->enable == ON)
		isg6320_set_enable(data, OFF);
}

static const struct dev_pm_ops isg6320_pm_ops = {
	.suspend = isg6320_suspend,
	.resume = isg6320_resume,
};

static const struct of_device_id isg6320_match_table[] = {
	{ .compatible = "isg6320", },
	{ },
};

static struct i2c_device_id isg6320_id_table[] = {
	{"ISG6320", 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, isg6320_id_table);


static struct i2c_driver isg6320_driver = {
	.driver = {
		.name = isg6320_id_table[0].name,
		.owner = THIS_MODULE,
		.of_match_table = isg6320_match_table,
		.pm = &isg6320_pm_ops,
	},
	.id_table = isg6320_id_table,
	.probe = isg6320_probe,
	.remove = isg6320_remove,
	.shutdown = isg6320_shutdown,
};

#if defined(CONFIG_SENSORS_ISG6320_SUB)
static const struct of_device_id isg6320_sub_match_table[] = {
	{ .compatible = "isg6320_sub", },
	{ },
};

static struct i2c_device_id isg6320_sub_id_table[] = {
	{"ISG6320_SUB", 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, isg6320_sub_id_table);

static struct i2c_driver isg6320_sub_driver = {
	.driver = {
		.name = isg6320_sub_id_table[0].name,
		.owner = THIS_MODULE,
		.of_match_table = isg6320_sub_match_table,
		.pm = &isg6320_pm_ops,
	},
	.id_table = isg6320_sub_id_table,
	.probe = isg6320_probe,
	.remove = isg6320_remove,
	.shutdown = isg6320_shutdown,
};
#endif

#if defined(CONFIG_SENSORS_ISG6320_WIFI)
static const struct of_device_id isg6320_wifi_match_table[] = {
	{ .compatible = "isg6320_wifi", },
	{ },
};

static struct i2c_device_id isg6320_wifi_id_table[] = {
	{"ISG6320_WIFI", 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, isg6320_wifi_id_table);

static struct i2c_driver isg6320_wifi_driver = {
	.driver = {
		.name = isg6320_wifi_id_table[0].name,
		.owner = THIS_MODULE,
		.of_match_table = isg6320_wifi_match_table,
		.pm = &isg6320_pm_ops,
	},
	.id_table = isg6320_wifi_id_table,
	.probe = isg6320_probe,
	.remove = isg6320_remove,
	.shutdown = isg6320_shutdown,
};
#endif

static int __init isg6320_init(void)
{
	int ret = 0;
#if 0 //IS_ENABLED(CONFIG_BATTERY_SAMSUNG)
	if (lpcharge) {
		pr_err("lpm : Do not load driver\n");
		return 0;
	}
#endif
	ret = i2c_add_driver(&isg6320_driver);
	if (ret != 0)
		pr_err("isg6320_driver probe fail\n");
#if defined(CONFIG_SENSORS_ISG6320_SUB)
	ret = i2c_add_driver(&isg6320_sub_driver);
	if (ret != 0)
		pr_err("isg6320_sub_driver probe fail\n");
#endif
#if defined(CONFIG_SENSORS_ISG6320_WIFI)
	ret = i2c_add_driver(&isg6320_wifi_driver);
	if (ret != 0)
		pr_err("isg6320_wifi_driver probe fail\n");
#endif

	return ret;
}

static void __exit isg6320_exit(void)
{
	i2c_del_driver(&isg6320_driver);
#if defined(CONFIG_SENSORS_ISG6320_SUB)
	i2c_del_driver(&isg6320_sub_driver);
#endif
#if defined(CONFIG_SENSORS_ISG6320_WIFI)
	i2c_del_driver(&isg6320_wifi_driver);
#endif
}

module_init(isg6320_init);
module_exit(isg6320_exit);

MODULE_DESCRIPTION("Imagis Grip Sensor driver");
MODULE_LICENSE("GPL");
