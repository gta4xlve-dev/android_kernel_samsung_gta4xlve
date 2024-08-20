/*
 * =================================================================
 *
 *
 *	Description:  samsung display common file
 *
 *	Author: jb09.kim
 *	Company:  Samsung Electronics
 *
 * ================================================================
 */
/*
<one line to give the program's name and a brief idea of what it does.>
Copyright (C) 2015, Samsung Electronics. All rights reserved.

 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "ss_dsi_panel_common.h"
#include <linux/preempt.h>

static void ss_panel_recovery(struct samsung_display_driver_data *vdd);
static void ss_event_osc_te_fitting(
		struct samsung_display_driver_data *vdd, int event, void *arg);
static irqreturn_t samsung_te_check_handler(int irq, void *handle);
static void samsung_te_check_done_work(struct work_struct *work);
static void ss_event_esd_recovery_init(
		struct samsung_display_driver_data *vdd, int event, void *arg);
static void samsung_display_delay_disp_on_work(struct work_struct *work);
static void read_panel_data_work_fn(struct work_struct *work);

struct samsung_display_driver_data vdd_data[MAX_DISPLAY_NDX];

void __iomem *virt_mmss_gp_base;
static DEFINE_MUTEX(dyn_mipi_clock);

LIST_HEAD(vdds_list);

char ss_panel_id0_get(struct samsung_display_driver_data *vdd)
{
	return (vdd->manufacture_id_dsi & 0xFF0000) >> 16;
}

char ss_panel_id1_get(struct samsung_display_driver_data *vdd)
{
	return (vdd->manufacture_id_dsi & 0xFF00) >> 8;
}

char ss_panel_id2_get(struct samsung_display_driver_data *vdd)
{
	return vdd->manufacture_id_dsi & 0xFF;
}

char ss_panel_rev_get(struct samsung_display_driver_data *vdd)
{
	return vdd->manufacture_id_dsi & 0x0F;
}

int ss_panel_attach_get(struct samsung_display_driver_data *vdd)
{
	return vdd->panel_attach_status;
}

int ss_panel_attach_set(struct samsung_display_driver_data *vdd, bool attach)
{
	/* 0bit->DSI0 1bit->DSI1 */
	/* check the lcd id for DISPLAY_1 and DISPLAY_2 */
	if (likely(ss_panel_attached(vdd->ndx) && attach))
		vdd->panel_attach_status = true;
	else
		vdd->panel_attach_status = false;

	LCD_INFO("panel_attach_status : %d\n", vdd->panel_attach_status);

	return vdd->panel_attach_status;
}

/*
 * Check the lcd id for DISPLAY_1 and DISPLAY_2 using the ndx
 */
int ss_panel_attached(int ndx)
{
	int lcd_id = 0;

	/*
	 * ndx 0 means DISPLAY_1 and ndx 1 means DISPLAY_2
	 */
	if (ndx == PRIMARY_DISPLAY_NDX)
		lcd_id = get_lcd_attached("GET");
	else if (ndx == SECONDARY_DISPLAY_NDX)
		lcd_id = get_lcd_attached_secondary("GET");

	/*
	 * The 0xFFFFFF is the id for PBA booting
	 * if the id is same with 0xFFFFFF, this function
	 * will return 0
	 */
	return !(lcd_id == PBA_ID);
}

static int ss_parse_panel_id(char *panel_id)
{
	char *pt;
	int lcd_id = 0;

	if (!IS_ERR_OR_NULL(panel_id))
		pt = panel_id;
	else
		return lcd_id;

	for (pt = panel_id; *pt != 0; pt++)  {
		lcd_id <<= 4;
		switch (*pt) {
		case '0' ... '9':
			lcd_id += *pt - '0';
			break;
		case 'a' ... 'f':
			lcd_id += 10 + *pt - 'a';
			break;
		case 'A' ... 'F':
			lcd_id += 10 + *pt - 'A';
			break;
		}
	}

	return lcd_id;
}

int get_lcd_attached(char *mode)
{
	static int lcd_id = -EINVAL;

	LCD_DEBUG("%s", mode);

	if (mode == NULL)
		return true;

	if (!strncmp(mode, "GET", 3))
		goto end;
	else
		lcd_id = ss_parse_panel_id(mode);

	LCD_ERR("LCD_ID = 0x%X\n", lcd_id);
end:

	/* case 03830582: Sometimes, bootloader fails to save lcd_id value in cmdline
	 * which is located in TZ.
	 * W/A: if panel name is PBA panel while lcd_id is not PBA ID, force to set lcd_id to PBA ID.
	 */
	if (unlikely(lcd_id == -EINVAL)) {
		char panel_name[MAX_CMDLINE_PARAM_LEN];
		char panel_string[] = "ss_dsi_panel_PBA_BOOTING_FHD";

		ss_get_primary_panel_name_cmdline(panel_name);
		if (!strncmp(panel_string, panel_name, strlen(panel_string)) && lcd_id != PBA_ID) {
			LCD_INFO("pba panel name, force lcd id: 0x%X -> 0xFFFFFF\n", lcd_id);
			lcd_id =  PBA_ID;
		}
	}

	return lcd_id;
}
EXPORT_SYMBOL(get_lcd_attached);
__setup("lcd_id=0x", get_lcd_attached);

int get_lcd_attached_secondary(char *mode)
{
	static int lcd_id = -EINVAL;

	LCD_DEBUG("%s", mode);

	if (mode == NULL)
		return true;

	if (!strncmp(mode, "GET", 3))
		goto end;
	else
		lcd_id = ss_parse_panel_id(mode);


	LCD_ERR("LCD_ID = 0x%X\n", lcd_id);
end:

	/* case 03830582: Sometimes, bootloader fails to save lcd_id value in cmdline
	 * which is located in TZ.
	 * W/A: if panel name is PBA panel while lcd_id is not PBA ID, force to set lcd_id to PBA ID.
	 */
	if (unlikely(lcd_id == -EINVAL)) {
		char panel_name[MAX_CMDLINE_PARAM_LEN];
		char panel_string[] = "ss_dsi_panel_PBA_BOOTING_FHD_DSI1";

		ss_get_secondary_panel_name_cmdline(panel_name);
		if (!strncmp(panel_string, panel_name, strlen(panel_string)) && lcd_id != PBA_ID) {
			LCD_INFO("pba panel name, force lcd id: 0x%X -> 0xFFFFFF\n", lcd_id);
			lcd_id =  PBA_ID;
		}
	}

	return lcd_id;
}
EXPORT_SYMBOL(get_lcd_attached_secondary);
__setup("lcd_id1=0x", get_lcd_attached_secondary);

static void ss_event_frame_update_pre(
		struct samsung_display_driver_data *vdd, void *arg)
{
	/* Block frame update during exclusive mode on.*/
	if (unlikely(vdd->exclusive_tx.enable)) {
		if (unlikely(!vdd->exclusive_tx.permit_frame_update)) {
			LCD_INFO("exclusive mode on, stop frame update\n");
			wait_event(vdd->exclusive_tx.ex_tx_waitq,
				!vdd->exclusive_tx.enable);
			LCD_INFO("continue frame update\n");
		}
	}
}

static void ss_event_frame_update(
		struct samsung_display_driver_data *vdd, int event, void *arg)
{
	struct panel_func *panel_func = NULL;
	static u8 frame_count = 1;
	s64 cur_time_64;
	int wait_time_32;
	s64 wait_time_64;
	int sleep_out_delay_32;
	s64 sleep_out_delay_64;

	panel_func = &vdd->panel_func;

	sleep_out_delay_32 = vdd->dtsi_data.samsung_wait_after_sleep_out_delay;
	sleep_out_delay_64 = (s64)sleep_out_delay_32;

	cur_time_64 = ktime_to_ms(ktime_get());
	wait_time_64 = sleep_out_delay_64 - (cur_time_64 - vdd->sleep_out_time_64);

	/* To protect 64bit overflow & underflow */
	if (wait_time_64 <= 0)
		wait_time_32 = 0;
	else if (wait_time_64 > sleep_out_delay_64)
		wait_time_32 = sleep_out_delay_32;
	else
		wait_time_32 = (s32)wait_time_64;

	if (vdd->dtsi_data.samsung_osc_te_fitting &&
			!(vdd->te_fitting_info.status & TE_FITTING_DONE)) {
		if (panel_func->ss_event_osc_te_fitting)
			panel_func->ss_event_osc_te_fitting(vdd, event, arg);
	}

	if (vdd->display_status_dsi.wait_disp_on) {
		/* TODO: Add condition to check the susupend state
		 * insted of !msm_is_suspend_state(GET_DRM_DEV(vdd))
		 */

		/* Skip a number of frames to avoid garbage image output from wakeup */
		if (frame_count <= vdd->dtsi_data.samsung_delayed_display_on) {
			LCD_DEBUG("Skip %d frame\n", frame_count);
			frame_count++;
			goto skip_display_on;
		}
		frame_count = 1;

		if (vdd->dtsi_data.samsung_wait_after_sleep_out_delay) {
			if (wait_time_32 > 0) {
				LCD_ERR("sleep_out_delay:%d sleep_out_t:%llu cur_t:%llu wait_t:%d start\n", sleep_out_delay_32,
					  vdd->sleep_out_time_64, cur_time_64, wait_time_32);
				usleep_range(wait_time_32*1000, wait_time_32*1000);
				LCD_ERR("wait_t: %d end\n", wait_time_32);
			} else
				LCD_ERR("sleep_out_delay:%d sleep_out_t:%llu cur_t:%llu wait_t:%d skip\n", sleep_out_delay_32,
					  vdd->sleep_out_time_64, cur_time_64, wait_time_32);
		}

		ss_send_cmd(vdd, TX_DISPLAY_ON);
		vdd->display_status_dsi.wait_disp_on = false;

		if (vdd->mdnie.support_mdnie) {
			vdd->mdnie.lcd_on_notifiy = true;
			update_dsi_tcon_mdnie_register(vdd);
			if (vdd->mdnie.support_trans_dimming)
				vdd->mdnie.disable_trans_dimming = false;
		}

		if (vdd->panel_func.samsung_backlight_late_on)
			vdd->panel_func.samsung_backlight_late_on(vdd);

		if (vdd->dtsi_data.hmt_enabled &&
				ss_is_panel_on(vdd)) {
			if (vdd->hmt_stat.hmt_on) {
				LCD_INFO("hmt reset ..\n");
				#if 0
				vdd->hmt_stat.hmt_enable(vdd);
				vdd->hmt_stat.hmt_reverse_update(vdd, 1);
				vdd->hmt_stat.hmt_bright_update(vdd);
				#endif
			}
		}

		if (ss_is_esd_check_enabled(vdd))
			vdd->esd_recovery.is_enabled_esd_recovery = true;

		/* For read flash gamma data before first brightness set */
		if (vdd->dtsi_data.flash_gamma_support && !vdd->panel_br_info.flash_data.init_done) {
			if (!work_busy(&vdd->flash_br_work.work)) {
				queue_delayed_work(vdd->flash_br_workqueue, &vdd->flash_br_work, msecs_to_jiffies(0));
			}
		}

		LCD_INFO("DISPLAY_ON\n");
	}

	/* copr - check actual display on (frame - copr > 0) debug */
	/* work thread will be stopped if copr is over 0 */
	if (vdd->display_status_dsi.wait_actual_disp_on && ss_is_panel_on(vdd)) {
		if (vdd->copr.read_copr_wq && vdd->copr.copr_on)
			queue_work(vdd->copr.read_copr_wq, &vdd->copr.read_copr_work);
	}

skip_display_on:
	return;
}

static void ss_send_esd_recovery_cmd(struct samsung_display_driver_data *vdd)
{
	static bool toggle;

	if (toggle)
		ss_send_cmd(vdd, TX_ESD_RECOVERY_1);
	else
		ss_send_cmd(vdd, TX_ESD_RECOVERY_2);
	toggle = !toggle;
}

static void ss_check_te(struct samsung_display_driver_data *vdd)
{
	unsigned int disp_te_gpio;
	int rc, te_count = 0;
	int te_max = 20000; /*sampling 200ms */

	disp_te_gpio = ss_get_te_gpio(vdd);

	LCD_INFO("============ start waiting for TE ============\n");
	if (gpio_is_valid(disp_te_gpio)) {
		for (te_count = 0 ; te_count < te_max ; te_count++) {
			rc = gpio_get_value(disp_te_gpio);
			if (rc == 1) {
				LCD_INFO("LDI generate TE within = %d.%dms\n", te_count/100, te_count%100);
				break;
			}
			/* usleep suspends the calling thread whereas udelay is a
			 * busy wait. Here the value of te_gpio is checked in a loop of
			 * max count = 250. If this loop has to iterate multiple
			 * times before the te_gpio is 1, the calling thread will end
			 * up in suspend/wakeup sequence multiple times if usleep is
			 * used, which is an overhead. So use udelay instead of usleep.
			 */
			udelay(10);
		}
		if (te_count == te_max) {
			LCD_ERR("LDI doesn't generate TE");
			SS_XLOG(0xbad);
			inc_dpui_u32_field(DPUI_KEY_QCT_NO_TE, 1);
		}
	} else
		LCD_ERR("disp_te_gpio is not valid\n");
	LCD_INFO("============ end waiting for TE ============\n");
}
/* SAMSUNG_FINGERPRINT */
static void ss_wait_for_te_gpio(struct samsung_display_driver_data *vdd, int num_of_te, int delay_after_te)
{
	unsigned int disp_te_gpio;
	int rc, te_count = 0;
	int te_max = 20000; /*sampling 100ms */
	int iter;
	s64 start_time_1_64, start_time_3_64;

	preempt_disable();
	disp_te_gpio = ss_get_te_gpio(vdd);
	for(iter = 0 ; iter < num_of_te ; iter++) {
		start_time_1_64 = ktime_to_us(ktime_get());
		if (gpio_is_valid(disp_te_gpio)) {
			for (te_count = 0 ; te_count < te_max ; te_count++) {
				rc = gpio_get_value(disp_te_gpio);
				if (rc == 1) {
					start_time_3_64 = ktime_to_us(ktime_get());
					LCD_ERR("ss_wait_for_te_gpio  = %llu\n", start_time_3_64- start_time_1_64);
					break;
				}
				ndelay(5000);
			}
		}
		if (te_count == te_max)
			LCD_ERR("LDI doesn't generate TE");
		udelay(200);
	}
	udelay(delay_after_te);
	preempt_enable();
}

/* SAMSUNG_FINGERPRINT */
void ss_send_hbm_fingermask_image_tx(struct samsung_display_driver_data *vdd, bool on)
{
	LCD_INFO("++ %s\n",on?"on":"off");
	if (on) {
		ss_brightness_dcs(vdd, 0, BACKLIGHT_FINGERMASK_ON);
	} else {
		ss_brightness_dcs(vdd, 0, BACKLIGHT_FINGERMASK_OFF);
	}
	LCD_INFO("--\n");
}

static void ss_event_fb_event_callback(struct samsung_display_driver_data *vdd, int event, void *arg)
{
	struct panel_func *panel_func = NULL;

	panel_func = &vdd->panel_func;

	if (IS_ERR_OR_NULL(panel_func)) {
		LCD_ERR("Invalid data panel_func : 0x%zx\n",
				(size_t)panel_func);
		return;
	}

	if (panel_func->ss_event_esd_recovery_init)
		panel_func->ss_event_esd_recovery_init(vdd, event, arg);
}

static int _ss_dsi_panel_event_handler(
		struct samsung_display_driver_data *vdd,
		enum mdss_intf_events event, void *arg)
{
	struct panel_func *panel_func = NULL;

	panel_func = &vdd->panel_func;

	if (IS_ERR_OR_NULL(panel_func)) {
		LCD_ERR("Invalid data panel_func : 0x%zx\n", (size_t)panel_func);
		return -EINVAL;
	}

	switch (event) {
	case SS_EVENT_FRAME_UPDATE_POST:
		if (!IS_ERR_OR_NULL(panel_func->ss_event_frame_update))
			panel_func->ss_event_frame_update(vdd, event, arg);
		break;
	case SS_EVENT_FRAME_UPDATE_PRE:
		ss_event_frame_update_pre(vdd, arg);
		break;
	case SS_EVENT_FB_EVENT_CALLBACK:
		if (!IS_ERR_OR_NULL(panel_func->ss_event_fb_event_callback))
			panel_func->ss_event_fb_event_callback(vdd, event, arg);
		break;
	case SS_EVENT_PANEL_ON:
		if (likely(!vdd->is_factory_mode))
			ss_panel_lpm_ctrl(vdd, false);
		break;
	case SS_EVENT_PANEL_OFF:
		if (likely(!vdd->is_factory_mode))
			ss_panel_lpm_ctrl(vdd, true);
		break;
	case SS_EVENT_PANEL_RECOVERY:
		ss_panel_recovery(vdd);
		break;
	case SS_EVENT_PANEL_ESD_RECOVERY:
		if (vdd->esd_recovery.send_esd_recovery)
			ss_send_esd_recovery_cmd(vdd);
		break;
	case SS_EVENT_CHECK_TE:
			ss_check_te(vdd);
		break;
	case SS_EVENT_SDE_HW_CATALOG_INIT:
		if (unlikely(vdd->panel_func.samsung_pba_config))
			vdd->panel_func.samsung_pba_config(vdd, arg);
		break;
	default:
		LCD_DEBUG("unhandled event=%d\n", event);
		break;
	}

	return 0;
}

int ss_dsi_panel_event_handler(
		int display_ndx, enum mdss_intf_events event, void *arg)
{

	struct samsung_display_driver_data *vdd = ss_get_vdd(display_ndx);

	if (unlikely(!vdd))
		return -EINVAL;

	return _ss_dsi_panel_event_handler(vdd, event, arg);
}

/* CP notity format (HEX raw format)
 * 10 00 AA BB 27 01 03 XX YY YY YY YY ZZ ZZ ZZ ZZ
 *
 * 00 10 (0x0010) - len
 * AA BB - not used
 * 27 - MAIN CMD (SYSTEM CMD : 0x27)
 * 01 - SUB CMD (CP Channel Info : 0x01)
 * 03 - NOTI CMD (0x03)
 * XX - RAT MODE
 * YY YY YY YY - BAND MODE
 * ZZ ZZ ZZ ZZ - FREQ INFO
 */

int ss_rf_info_notify_callback(struct notifier_block *nb,
				unsigned long size, void *data)
{
	struct dyn_mipi_clk *dyn_mipi_clk = container_of(nb, struct dyn_mipi_clk, notifier);
	struct samsung_display_driver_data *vdd =
		container_of(dyn_mipi_clk, struct samsung_display_driver_data, dyn_mipi_clk);

	struct dev_ril_bridge_msg *msg;

	msg = (struct dev_ril_bridge_msg *)data;
	LCD_INFO("RIL noti: ndx: %d, size: %lu, dev_id: %d, len: %d\n",
			vdd->ndx, size, msg->dev_id, msg->data_len);
	if (msg->dev_id == IPC_SYSTEM_CP_CHANNEL_INFO // #define IPC_SYSTEM_CP_CHANNEL_INFO	0x01
			&& msg->data_len == sizeof(struct rf_info)) {
		mutex_lock(&dyn_mipi_clk->dyn_mipi_lock);
		memcpy(&dyn_mipi_clk->rf_info, msg->data, sizeof(struct rf_info));
		mutex_unlock(&dyn_mipi_clk->dyn_mipi_lock);

		queue_work(dyn_mipi_clk->change_clk_wq, &dyn_mipi_clk->change_clk_work);

		LCD_INFO("RIL noti: RAT(%d), BAND(%d), ARFCN(%d)\n",
				dyn_mipi_clk->rf_info.rat,
				dyn_mipi_clk->rf_info.band,
				dyn_mipi_clk->rf_info.arfcn);
	}

	return NOTIFY_DONE;
}

static int ss_find_dyn_mipi_clk_timing_idx(struct samsung_display_driver_data *vdd)
{
	int idx = -EINVAL;
	int loop;
	int rat, band, arfcn;
	struct clk_sel_table sel_table = vdd->dyn_mipi_clk.clk_sel_table;

	if (!sel_table.tab_size) {
		LCD_ERR("Table is NULL");
		return -ENOENT;
	}

	rat = vdd->dyn_mipi_clk.rf_info.rat;
	band = vdd->dyn_mipi_clk.rf_info.band;
	arfcn = vdd->dyn_mipi_clk.rf_info.arfcn;

	for (loop = 0 ; loop < sel_table.tab_size ; loop++) {
		if ((rat == sel_table.rat[loop]) && (band == sel_table.band[loop])) {
			if ((arfcn >= sel_table.from[loop]) && (arfcn <= sel_table.end[loop])) {
				idx = sel_table.target_clk_idx[loop];
				break;
			}
		}
	}

	LCD_INFO("RAT(%d), BAND(%d), ARFCN(%d), Clock Index(%d)\n",
		rat, band, arfcn, idx);

	return idx;

}

/* refer to sysfs_dynamic_dsi_clk_write() */
extern ssize_t sysfs_dynamic_dsi_clk_write(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static int _ss_change_dyn_mipi_clk_timing(struct samsung_display_driver_data *vdd, int clk_rate)
{
	struct dsi_display *display = GET_DSI_DISPLAY(vdd);
	struct device *dev = &display->pdev->dev;
	char clk_rate_buf[12];
	const char *buf;
	size_t count = 12;

	LCD_INFO("+++: ndx: %d, clk_rate: %d, cached_clk_rate: %d\n",
			vdd->ndx, clk_rate, display->cached_clk_rate);
	sprintf(clk_rate_buf, "%d", clk_rate);
	buf = clk_rate_buf;

	sysfs_dynamic_dsi_clk_write(dev, NULL, buf, count);
	LCD_INFO("---: ndx: %d, clk_rate: %d\n", vdd->ndx, clk_rate);

	return 0;
}


int ss_change_dyn_mipi_clk_timing(struct samsung_display_driver_data *vdd)
{
	struct clk_timing_table timing_table = vdd->dyn_mipi_clk.clk_timing_table;
	int idx;
	int clk_rate;
	int timeout = 60; // 60 seconds
	int ret = 0;

	if (!vdd->dyn_mipi_clk.is_support) {
		LCD_ERR("Dynamic MIPI Clock does not support\n");
		return -ENODEV;
	}

	if (!timing_table.tab_size) {
		LCD_ERR("Table is NULL");
		return -ENODEV;
	}

	mutex_lock(&dyn_mipi_clock);

	LCD_INFO("+++: ndx=%d", vdd->ndx);

	/*  wait for dispaly init done */
	while (timeout-- && !vdd->display_status_dsi.disp_on_pre) {
		msleep(1000);
		LCD_INFO("wait time: %ds, disp_on_pre: %d\n",
				timeout, vdd->display_status_dsi.disp_on_pre);

		if (!timeout) {
			LCD_ERR("err: display init timeout...\n");
			goto err;
		}
	}

	mutex_lock(&vdd->dyn_mipi_clk.dyn_mipi_lock);
	idx = ss_find_dyn_mipi_clk_timing_idx(vdd);
	mutex_unlock(&vdd->dyn_mipi_clk.dyn_mipi_lock);
	if (idx < 0 ) {
		LCD_ERR("Failed to find MIPI clock timing (%d)\n", idx);
		goto err;
	}

	clk_rate = timing_table.clk_rate[idx];
	LCD_INFO("clk idx: %d, clk_rate: %d\n", idx, clk_rate);

	/* update dynamic mipi clk */
	ret = _ss_change_dyn_mipi_clk_timing(vdd, clk_rate);
	if (ret) {
		LCD_ERR("fail to change mipi clk(%d), ret:%d\n", clk_rate, ret);
		goto err;
	}

	/* update mipi global timing: skip this.. not required...  */
err:
	LCD_INFO("---: ndx=%d", vdd->ndx);
	mutex_unlock(&dyn_mipi_clock);

	return 0;
}

int ss_dyn_mipi_clk_tx_ffc(struct samsung_display_driver_data *vdd)
{
	struct dsi_panel_cmd_set *ffc_set;
	struct dsi_panel_cmd_set *dyn_ffc_set;
	int idx;
	int ffc_cmds_line_position = 1;

	mutex_lock(&vdd->dyn_mipi_clk.dyn_mipi_lock);
	idx = ss_find_dyn_mipi_clk_timing_idx(vdd);
	mutex_unlock(&vdd->dyn_mipi_clk.dyn_mipi_lock);

	if (idx < 0 ) {
		LCD_ERR("Failed to find MIPI clock timing (%d)\n", idx);
		return -EINVAL;
	}

	LCD_INFO("+++ clk idx: %d, tx FFC\n", idx);

	ffc_set = ss_get_cmds(vdd, TX_FFC);
	dyn_ffc_set = ss_get_cmds(vdd, TX_DYNAMIC_FFC_SET);
	if (SS_IS_CMDS_NULL(ffc_set) || SS_IS_CMDS_NULL(dyn_ffc_set)) {
		LCD_ERR("No cmds for TX_FFC..\n");
		return -EINVAL;
	}

	ffc_cmds_line_position = vdd->ffc_cmds_line_position;
	memcpy(ffc_set->cmds[ffc_cmds_line_position].msg.tx_buf, dyn_ffc_set->cmds[idx].msg.tx_buf, ffc_set->cmds[ffc_cmds_line_position].msg.tx_len);

	ss_send_cmd(vdd, TX_FFC);

	LCD_INFO("--- clk idx: %d, ffc_cmds_line_position : %d tx FFC\n", idx, ffc_cmds_line_position);

	return 0;
}

static int ss_parse_dyn_mipi_clk_timing_table(struct device_node *np,
		void *tbl, char *keystring)
{
	struct clk_timing_table *table = (struct clk_timing_table *) tbl;
	const __be32 *data;
	int len = 0, i = 0, data_offset = 0;

	data = of_get_property(np, keystring, &len);

	if (!data) {
		LCD_ERR("%d, Unable to read table %s ", __LINE__, keystring);
		return -EINVAL;
	}

	table->tab_size = len / sizeof(int);
	table->clk_rate = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->clk_rate)
		return -ENOMEM;

	for (i = 0 ; i < table->tab_size; i++)
		table->clk_rate[i] = be32_to_cpup(&data[data_offset++]);

	LCD_INFO("%s tab_size (%d)\n", keystring, table->tab_size);

	return 0;
}

static int ss_parse_dyn_mipi_clk_sel_table(struct device_node *np,
		void *tbl, char *keystring)
{
	struct clk_sel_table *table = (struct clk_sel_table *) tbl;
	const __be32 *data;
	int len = 0, i = 0, data_offset = 0;
	int col_size = 0;

	data = of_get_property(np, keystring, &len);

	if (data)
		LCD_INFO("Success to read table %s\n", keystring);
	else {
		LCD_ERR("%d, Unable to read table %s ", __LINE__, keystring);
		return -EINVAL;
	}

	col_size = 5;

	if ((len % col_size) != 0) {
		LCD_ERR("%d, Incorrect table entries for %s , len : %d",
					__LINE__, keystring, len);
		return -EINVAL;
	}

	table->tab_size = len / (sizeof(int) * col_size);

	table->rat = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->rat)
		return -ENOMEM;
	table->band = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->band)
		goto error;
	table->from = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->from)
		goto error;
	table->end = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->end)
		goto error;
	table->target_clk_idx = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->target_clk_idx)
		goto error;

	for (i = 0 ; i < table->tab_size; i++) {
		table->rat[i] = be32_to_cpup(&data[data_offset++]);
		table->band[i] = be32_to_cpup(&data[data_offset++]);
		table->from[i] = be32_to_cpup(&data[data_offset++]);
		table->end[i] = be32_to_cpup(&data[data_offset++]);
		table->target_clk_idx[i] = be32_to_cpup(&data[data_offset++]);
		LCD_DEBUG("%dst : %d %d %d %d %d\n",
			i, table->rat[i], table->band[i], table->from[i],
			table->end[i], table->target_clk_idx[i]);
	}

	LCD_INFO("%s tab_size (%d)\n", keystring, table->tab_size);

	return 0;

error:
	LCD_ERR("Allocation Fail\n");
	kfree(table->rat);
	kfree(table->band);
	kfree(table->from);
	kfree(table->end);
	kfree(table->target_clk_idx);

	return -ENOMEM;
}

int ss_parse_candella_mapping_table(struct device_node *np,
		void *tbl, char *keystring)
{
	struct candela_map_table *table = (struct candela_map_table *) tbl;
	const __be32 *data;
	int len = 0, i = 0, data_offset = 0;
	int col_size = 0;

	data = of_get_property(np, keystring, &len);
	if (!data) {
		LCD_DEBUG("%d, Unable to read table %s ", __LINE__, keystring);
		return -EINVAL;
	} else
		LCD_ERR("Success to read table %s\n", keystring);

	col_size = 4;

	if ((len % col_size) != 0) {
		LCD_ERR("%d, Incorrect table entries for %s , len : %d",
					__LINE__, keystring, len);
		return -EINVAL;
	}

	table->tab_size = len / (sizeof(int) * col_size);

	table->cd = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->cd)
		goto error;
	table->idx = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->idx)
		goto error;
	table->from = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->from)
		goto error;
	table->end = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->end)
		goto error;

	for (i = 0 ; i < table->tab_size; i++) {
		table->idx[i] = be32_to_cpup(&data[data_offset++]);		/* field => <idx> */
		table->from[i] = be32_to_cpup(&data[data_offset++]);	/* field => <from> */
		table->end[i] = be32_to_cpup(&data[data_offset++]);		/* field => <end> */
		table->cd[i] = be32_to_cpup(&data[data_offset++]);		/* field => <cd> */
	}

	table->min_lv = table->from[0];
	table->max_lv = table->end[table->tab_size-1];

	LCD_INFO("tab_size (%d), hbm min/max lv (%d/%d)\n", table->tab_size, table->min_lv, table->max_lv);

	return 0;

error:
	kfree(table->cd);
	kfree(table->idx);
	kfree(table->from);
	kfree(table->end);

	return -ENOMEM;
}

int ss_parse_pac_candella_mapping_table(struct device_node *np,
		void *tbl, char *keystring)
{
	struct candela_map_table *table = (struct candela_map_table *) tbl;
	const __be32 *data;
	int len = 0, i = 0, data_offset = 0;
	int col_size = 0;

	data = of_get_property(np, keystring, &len);
	if (!data) {
		LCD_DEBUG("%d, Unable to read table %s ", __LINE__, keystring);
		return -EINVAL;
	} else
		LCD_ERR("Success to read table %s\n", keystring);

	col_size = 6;

	if ((len % col_size) != 0) {
		LCD_ERR("%d, Incorrect table entries for %s , len : %d",
					__LINE__, keystring, len);
		return -EINVAL;
	}

	table->tab_size = len / (sizeof(int) * col_size);

	table->interpolation_cd = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->interpolation_cd)
		return -ENOMEM;
	table->cd = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->cd)
		goto error;
	table->scaled_idx = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->scaled_idx)
		goto error;
	table->idx = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->idx)
		goto error;
	table->from = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->from)
		goto error;
	table->end = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->end)
		goto error;
/*
	table->auto_level = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->auto_level)
		goto error;
*/
	for (i = 0 ; i < table->tab_size; i++) {
		table->scaled_idx[i] = be32_to_cpup(&data[data_offset++]);	/* field => <scaeled idx> */
		table->idx[i] = be32_to_cpup(&data[data_offset++]);			/* field => <idx> */
		table->from[i] = be32_to_cpup(&data[data_offset++]);		/* field => <from> */
		table->end[i] = be32_to_cpup(&data[data_offset++]);			/* field => <end> */
		table->cd[i] = be32_to_cpup(&data[data_offset++]);			/* field => <cd> */
		table->interpolation_cd[i] = be32_to_cpup(&data[data_offset++]);	/* field => <interpolation cd> */
	}

	table->min_lv = table->from[0];
	table->max_lv = table->end[table->tab_size-1];

	LCD_INFO("tab_size (%d), hbm min/max lv (%d/%d)\n", table->tab_size, table->min_lv, table->max_lv);

	return 0;

error:
	kfree(table->interpolation_cd);
	kfree(table->cd);
	kfree(table->scaled_idx);
	kfree(table->idx);
	kfree(table->from);
	kfree(table->end);

	return -ENOMEM;
}

int ss_parse_gamma_mode2_candella_mapping_table(struct device_node *np,
		void *tbl, char *keystring)
{
	struct candela_map_table *table = (struct candela_map_table *) tbl;
	const __be32 *data;
	int len = 0, i = 0, data_offset = 0;
	int col_size = 0;

	data = of_get_property(np, keystring, &len);
	if (!data) {
		LCD_DEBUG("%d, Unable to read table %s ", __LINE__, keystring);
		return -EINVAL;
	} else
		LCD_ERR("Success to read table %s\n", keystring);

	col_size = 5;

	if ((len % col_size) != 0) {
		LCD_ERR("%d, Incorrect table entries for %s , len : %d",
					__LINE__, keystring, len);
		return -EINVAL;
	}

	table->tab_size = len / (sizeof(int) * col_size);

	table->gamma_mode2_cd = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->gamma_mode2_cd)
		return -ENOMEM;

	table->cd = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->cd)
		goto error;
	table->idx = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->idx)
		goto error;
	table->from = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->from)
		goto error;
	table->end = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->end)
		goto error;

	for (i = 0 ; i < table->tab_size; i++) {
		table->idx[i] = be32_to_cpup(&data[data_offset++]);	/* field => <idx> */
		table->from[i] = be32_to_cpup(&data[data_offset++]);	/* field => <from> */
		table->end[i] = be32_to_cpup(&data[data_offset++]);	/* field => <end> */
		table->cd[i] = be32_to_cpup(&data[data_offset++]);	/* field => <cd> */
		table->gamma_mode2_cd[i] = be32_to_cpup(&data[data_offset++]);	/* field => <gamma_mode2_cd (panel_real_cd)> */
		LCD_DEBUG("[%d] %d %d %d %d %d\n", i, table->idx[i], table->from[i], table->end[i], table->cd[i], table->gamma_mode2_cd[i]);
	}

	table->min_lv = table->from[0];
	table->max_lv = table->end[table->tab_size-1];

	LCD_INFO("tab_size (%d), hbm min/max lv (%d/%d)\n", table->tab_size, table->min_lv, table->max_lv);

	return 0;

error:
	kfree(table->idx);
	kfree(table->from);
	kfree(table->end);
	kfree(table->cd);
	kfree(table->gamma_mode2_cd);

	return -ENOMEM;
}

int ss_parse_hbm_candella_mapping_table(struct device_node *np,
		void *tbl, char *keystring)
{
	struct candela_map_table *table = (struct candela_map_table *) tbl;
	const __be32 *data;
	int data_offset = 0, len = 0, i = 0;

	data = of_get_property(np, keystring, &len);
	if (!data) {
		LCD_DEBUG("%d, Unable to read table %s ", __LINE__, keystring);
		return -EINVAL;
	} else
		LCD_ERR("Success to read table %s\n", keystring);

	if ((len % 4) != 0) {
		LCD_ERR("%d, Incorrect table entries for %s",
					__LINE__, keystring);
		return -EINVAL;
	}

	table->tab_size = len / (sizeof(int)*5);

	table->interpolation_cd = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->interpolation_cd)
		return -ENOMEM;
	table->cd = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->cd)
		goto error;
	table->scaled_idx = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->scaled_idx)
		goto error;
	table->idx = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->idx)
		goto error;
	table->from = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->from)
		goto error;
	table->end = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->end)
		goto error;
	table->auto_level = kzalloc((sizeof(int) * table->tab_size), GFP_KERNEL);
	if (!table->auto_level)
		goto error;

	for (i = 0 ; i < table->tab_size; i++) {
		table->idx[i] = be32_to_cpup(&data[data_offset++]);		/* 1st field => <idx> */
		table->from[i] = be32_to_cpup(&data[data_offset++]);		/* 2nd field => <from> */
		table->end[i] = be32_to_cpup(&data[data_offset++]);			/* 3rd field => <end> */
		table->cd[i] = be32_to_cpup(&data[data_offset++]);		/* 4th field => <candella> */
		table->auto_level[i] = be32_to_cpup(&data[data_offset++]);  /* 5th field => <auto brightness level> */
	}

	table->min_lv = table->from[0];
	table->max_lv = table->end[table->tab_size-1];

	LCD_INFO("tab_size (%d), hbm min/max lv (%d/%d)\n", table->tab_size, table->min_lv, table->max_lv);

	return 0;
error:
	kfree(table->cd);
	kfree(table->idx);
	kfree(table->from);
	kfree(table->end);
	kfree(table->auto_level);

	return -ENOMEM;
}

int ss_parse_panel_table(struct device_node *np,
		void *tbl, char *keystring)
{
	struct cmd_map *table = (struct cmd_map *) tbl;
	const __be32 *data;
	int  data_offset, len = 0, i = 0;

	data = of_get_property(np, keystring, &len);
	if (!data) {
		LCD_DEBUG("%d, Unable to read table %s\n", __LINE__, keystring);
		return -EINVAL;
	} else
		LCD_ERR("Success to read table %s\n", keystring);

	if ((len % 2) != 0) {
		LCD_ERR("%d, Incorrect table entries for %s",
					__LINE__, keystring);
		return -EINVAL;
	}

	table->size = len / (sizeof(int)*2);
	table->bl_level = kzalloc((sizeof(int) * table->size), GFP_KERNEL);
	if (!table->bl_level)
		return -ENOMEM;

	table->cmd_idx = kzalloc((sizeof(int) * table->size), GFP_KERNEL);
	if (!table->cmd_idx)
		goto error;

	data_offset = 0;
	for (i = 0 ; i < table->size; i++) {
		table->bl_level[i] = be32_to_cpup(&data[data_offset++]);
		table->cmd_idx[i] = be32_to_cpup(&data[data_offset++]);
	}

	return 0;
error:
	kfree(table->cmd_idx);

	return -ENOMEM;
}

int ss_send_cmd(struct samsung_display_driver_data *vdd,
		enum dsi_cmd_set_type type)
{
	struct dsi_panel *panel = GET_DSI_PANEL(vdd);
	struct dsi_display *dsi_display = GET_DSI_DISPLAY(vdd);
	struct dsi_panel_cmd_set *set;
	bool is_vdd_locked = false;
	int rc = 0;

	if (IS_ERR_OR_NULL(vdd)) {
		LCD_ERR("vdd is null\n");
		return -ENODEV;
	}

	/* Make not to turn on the panel power when ub_con_det gpio is high (ub is not connected) */
	if (unlikely(vdd->is_factory_mode)) {
		if (gpio_is_valid(vdd->ub_con_det.gpio) && gpio_get_value(vdd->ub_con_det.gpio)) {
			LCD_ERR("ub_con_det.gpio = %d\n", gpio_get_value(vdd->ub_con_det.gpio));
			return -EAGAIN;
		}
	}

	if (!ss_panel_attach_get(vdd)) {
		LCD_ERR("ss_panel_attach_get(%d) : %d\n",
				vdd->ndx, ss_panel_attach_get(vdd));
		return -EAGAIN;
	}

	/* Skip to lock vdd_lock for commands that has exclusive_pass token
	 * to prevent deadlock between vdd_lock and ex_tx_waitq.
	 * cmd_lock guarantees exclusive tx cmds without vdd_lock.
	 */
	set = ss_get_cmds(vdd, type);
	/*
	if (SS_IS_CMDS_NULL(set)) {
		LCD_ERR("No cmds for type(%d) \n", type);
		return -EINVAL;
	}
*/
	if (likely(!vdd->exclusive_tx.enable || !set->exclusive_pass)) {
		mutex_lock(&vdd->vdd_lock);
		is_vdd_locked = true;
	}

	if (ss_is_panel_off(vdd) || (vdd->panel_func.samsung_lvds_write_reg)) {
		LCD_ERR("skip to tx cmd (%d), panel off (%d)\n",
				type, ss_is_panel_off(vdd));
		goto error;
	}

	if (vdd->debug_data->print_cmds)
		LCD_INFO("Send cmd(%d): %s ++\n", type, ss_get_cmd_name(type));
	else
		LCD_DEBUG("Send cmd(%d): %s ++\n", type, ss_get_cmd_name(type));

	/* case 03063186:
	 * To prevent deadlock between phandle->phandle_lock and panel->panel_lock,
	 * dsi_display_clk_ctrl() should be called without locking panel_lock.
	 */
	rc = dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
			DSI_ALL_CLKS, DSI_CLK_ON);
	if (rc) {
		pr_err("[%s] failed to enable DSI core clocks, rc=%d\n",
				dsi_display->name, rc);
		goto error;
	}

	dsi_panel_tx_cmd_set(panel, type);

	rc = dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
			DSI_ALL_CLKS, DSI_CLK_OFF);
	if (rc) {
		pr_err("[%s] failed to disable DSI core clocks, rc=%d\n",
				dsi_display->name, rc);
		goto error;
	}

	if (vdd->debug_data->print_cmds)
		LCD_INFO("Send cmd(%d): %s --\n", type, ss_get_cmd_name(type));
	else
		LCD_DEBUG("Send cmd(%d): %s --\n", type, ss_get_cmd_name(type));

error:
	if (likely(is_vdd_locked))
		mutex_unlock(&vdd->vdd_lock);

	return rc;
}

#include <linux/cpufreq.h>
#define CPUFREQ_MAX	100
#define CPUFREQ_ALT_HIGH	70
struct cluster_cpu_info {
	int cpu_man; /* cpu number that manages the policy */
	int max_freq;
};

static void __ss_set_cpufreq_cpu(struct samsung_display_driver_data *vdd,
				int enable, int cpu, int max_percent)
{
	struct cpufreq_policy *policy;
	static unsigned int org_min[NR_CPUS + 1];
	static unsigned int org_max[NR_CPUS + 1];

	policy = cpufreq_cpu_get(cpu);
	if (policy == NULL) {
		LCD_ERR("policy is null..\n");
		return;
	}

	if (enable) {
		org_min[cpu] = policy->min;
		org_max[cpu] = policy->max;

		/* max_freq's unit is kHz, and it should not cause overflow.
		 * After applying user_policy, cpufreq driver will find closest
		 * frequency in freq_table, and set the frequency.
		 */
		policy->user_policy.min = policy->user_policy.max =
			policy->cpuinfo.max_freq * max_percent / 100;
	} else {
		policy->user_policy.min = org_min[cpu];
		policy->user_policy.max = org_max[cpu];
	}

	cpufreq_update_policy(cpu);
	cpufreq_cpu_put(policy);

	LCD_DEBUG("en=%d, cpu=%d, per=%d, min=%d, org=(%d-%d)\n",
			enable, cpu, max_percent, policy->user_policy.min,
			org_min[cpu], org_max[cpu]);
}

void ss_set_max_cpufreq(struct samsung_display_driver_data *vdd,
				int enable, enum mdss_cpufreq_cluster cluster)
{
	struct cpufreq_policy *policy;
	int cpu;
	struct cluster_cpu_info cluster_info[NR_CPUS];
	int cnt_cluster;
//	int bigp_cpu_man;
	int big_cpu_man = 0;
	int little_cpu_man = 0;
	int i = 0;

	//return;

	LCD_DEBUG("en=%d, cluster=%d\n", enable, cluster);
	get_online_cpus();

	/* find clusters */
	cnt_cluster = 0;
	for_each_online_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		/* In big-little octa core system,
		 * cpu0 ~ cpu3 has same policy (policy0) and
		 * policy->cpu would be 0. (cpu0 manages policy0)
		 * cpu4 ~ cpu7 has same policy (policy4) and
		 * policy->cpu would be 4. (cpu4 manages policy0)
		 * In this case, cnt_cluster would be two, and
		 * cluster_info[0].cpu_man = 0, cluster_info[1].cpu_man = 4.
		 */
		if (policy && policy->cpu == cpu) {
			cluster_info[cnt_cluster].cpu_man = cpu;
			cluster_info[cnt_cluster].max_freq =
				policy->cpuinfo.max_freq;
			cnt_cluster++;
			//LCD_INFO("cpu(%d)\n", cpu);
		}
		cpufreq_cpu_put(policy);
	}

	if (!cnt_cluster || cnt_cluster > 3) {
		LCD_ERR("cluster count err (%d)\n", cnt_cluster);
		goto end;
	}

	if (cnt_cluster == 1) {
		/* single policy (none big-little case) */
		LCD_INFO("cluster count is one...\n");

		if (cluster == CPUFREQ_CLUSTER_ALL)
			/* set all cores' freq to max*/
			__ss_set_cpufreq_cpu(vdd, enable,
					cluster_info[0].cpu_man, CPUFREQ_MAX);
		else
			/* set all cores' freq to max * 70 percent */
			__ss_set_cpufreq_cpu(vdd, enable,
				cluster_info[0].cpu_man, CPUFREQ_ALT_HIGH);
		goto end;
	}

#if 0
	/* big-little */
	if (cluster_info[0].max_freq > cluster_info[1].max_freq) {
		if (cluster_info[1].max_freq > cluster_info[2].max_freq) {
			/* 0 > 1 > 2 */
			big_cpu_man = cluster_info[0].cpu_man;
			little_cpu_man = cluster_info[2].cpu_man;
		}
		else if (cluster_info[2].max_freq > cluster_info[0].max_freq) {
			/* 2 > 0 > 1 */
			big_cpu_man = cluster_info[2].cpu_man;
			little_cpu_man = cluster_info[1].cpu_man;
		}
		else {
			/* 0 > 2 > 1 */
			big_cpu_man = cluster_info[0].cpu_man;
			little_cpu_man = cluster_info[1].cpu_man;
		}
	} else {
		if (cluster_info[0].max_freq > cluster_info[2].max_freq) {
			/* 1 > 0 > 2 */
			big_cpu_man = cluster_info[1].cpu_man;
			little_cpu_man = cluster_info[2].cpu_man;
		}
		else if (cluster_info[2].max_freq > cluster_info[1].max_freq) {
			/* 2 > 1 > 0 */
			big_cpu_man = cluster_info[2].cpu_man;
			little_cpu_man = cluster_info[0].cpu_man;
		}
		else {
			/* 1 > 2 > 0 */
			big_cpu_man = cluster_info[1].cpu_man;
			little_cpu_man = cluster_info[0].cpu_man;
		}
	}
#endif

	if (cluster == CPUFREQ_CLUSTER_BIG) {
		__ss_set_cpufreq_cpu(vdd, enable, big_cpu_man,
				CPUFREQ_MAX);
	} else if (cluster == CPUFREQ_CLUSTER_LITTLE) {
		__ss_set_cpufreq_cpu(vdd, enable, little_cpu_man,
				CPUFREQ_MAX);
	} else if  (cluster == CPUFREQ_CLUSTER_ALL) {
		for (i = 0; i < cnt_cluster; i++) {
			__ss_set_cpufreq_cpu(vdd, enable, cluster_info[i].cpu_man,
				CPUFREQ_MAX);
		}
		/*
		__ss_set_cpufreq_cpu(vdd, enable, big_cpu_man,
				CPUFREQ_MAX);
		__ss_set_cpufreq_cpu(vdd, enable, little_cpu_man,
				CPUFREQ_MAX);
		*/
	}

end:
	put_online_cpus();
}

/* ss_set_exclusive_packet():
 * This shuold be called in ex_tx_lock lock.
 */
void ss_set_exclusive_tx_packet(
		struct samsung_display_driver_data *vdd,
		enum dsi_cmd_set_type cmd, int pass)
{
	struct dsi_panel_cmd_set *pcmds = NULL;

	pcmds = ss_get_cmds(vdd, cmd);
	pcmds->exclusive_pass = pass;
}

/*
 * ex_tx_lock should be locked before panel_lock if there is a dsi_panel_tx_cmd_set after panel_lock is locked.
 * because of there is a "ex_tx_lock -> panel lock" routine at the sequence of ss_gct_write.
 * So we need to add ex_tx_lock to protect all "dsi_panel_tx_cmd_set after panel_lock".
 */

void ss_set_exclusive_tx_lock_from_qct(struct samsung_display_driver_data *vdd, bool lock)
{
	if (lock)
		mutex_lock(&vdd->exclusive_tx.ex_tx_lock);
	else
		mutex_unlock(&vdd->exclusive_tx.ex_tx_lock);
}

#define _ALIGN_DOWN(addr, size)  ((addr)&(~((size)-1)))
//#define MAX_DSI_FIFO_SIZE_HS_MODE	(480) /* AP limitation */
/* TODO: find MAX_DSI_FIFO_SIZE_HS_MODE	for sdm845... if set to 480, cause fail of tx mipi */
#define MAX_DSI_FIFO_SIZE_HS_MODE	(50) /* AP limitation */
#define MAX_SIZE_IMG_PAYLOAD	_ALIGN_DOWN((MAX_DSI_FIFO_SIZE_HS_MODE - 1), 12)

#define GRAM_WR_MEM_START	0x2c
#define GRAM_WR_MEM_CONT	0x3c
#define SIDERAM_WR_MEM_START	0x4c
#define SIDERAM_WR_MEM_CONT	0x5c

int ss_write_ddi_ram(struct samsung_display_driver_data *vdd,
		int target, u8 *buffer, int len)
{
	u8 hdr_start;
	u8 hdr_continue;
	struct dsi_panel_cmd_set *set;
	struct dsi_cmd_desc *cmds;
	u8 *tx_buf;
	u8 *org_tx_buf;
	int loop;
	int remain;
	int psize;

	LCD_INFO("+\n");

	set = ss_get_cmds(vdd, TX_DDI_RAM_IMG_DATA);

	if (target == MIPI_TX_TYPE_GRAM) {
		hdr_start = GRAM_WR_MEM_START;
		hdr_continue = GRAM_WR_MEM_CONT;
	} else if (target == MIPI_TX_TYPE_SIDERAM) {
		hdr_start = SIDERAM_WR_MEM_START;
		hdr_continue = SIDERAM_WR_MEM_CONT;
	} else {
		LCD_ERR("invalid target (%d)\n", target);
		return -EINVAL;
	}

	set->state = DSI_CMD_SET_STATE_HS;
	set->count = DIV_ROUND_UP(len, MAX_SIZE_IMG_PAYLOAD);

	cmds = vzalloc(sizeof(struct dsi_cmd_desc) * set->count);
	tx_buf = vzalloc((MAX_SIZE_IMG_PAYLOAD + 1) * set->count);
	org_tx_buf = tx_buf;
	set->cmds = cmds;


	LCD_INFO("len=%d, cmds count=%d\n", len, set->count);

	/* split and fill image data to TX_DDI_RAM_IMG_DATA packet */
	loop = 0;
	remain = len;
	while (remain > 0) {
		cmds[loop].msg.type = MIPI_DSI_GENERIC_LONG_WRITE;
		cmds[loop].last_command = 1;
		cmds[loop].msg.tx_buf = tx_buf;

		if (loop == 0)
			*tx_buf = hdr_start;
		else
			*tx_buf = hdr_continue;
		tx_buf++;

		if (remain > MAX_SIZE_IMG_PAYLOAD)
			psize = MAX_SIZE_IMG_PAYLOAD;
		else
			psize = remain;

		memcpy(tx_buf, buffer, psize);
		cmds[loop].msg.tx_len = psize + 1;

		tx_buf += psize;
		buffer += psize;
		remain -= psize;
		loop++;
	}

	LCD_INFO("start tx gram\n");
	ss_send_cmd(vdd, TX_DDI_RAM_IMG_DATA);
	LCD_INFO("fin tx gram\n");

	vfree(org_tx_buf);
	vfree(cmds);
	LCD_INFO("-\n");

	return 0;
}

/**
 * controller have 4 registers can hold 16 bytes of rxed data
 * dcs packet: 4 bytes header + payload + 2 bytes crc
 * 1st read: 4 bytes header + 10 bytes payload + 2 crc
 * 2nd read: 14 bytes payload + 2 crc
 * 3rd read: 14 bytes payload + 2 crc
 */
#define MAX_LEN_RX_BUF	300
#define RX_SIZE_LIMIT	10
int ss_panel_data_read_no_gpara(struct samsung_display_driver_data *vdd,
		enum dsi_cmd_set_type type, u8 *buffer, int level_key)
{
	struct dsi_panel_cmd_set *set;
	char show_buffer[MAX_LEN_RX_BUF] = {0,};
	char temp_buffer[MAX_LEN_RX_BUF] = {0,};
	int orig_rx_len = 0;
	int orig_offset = 0;
	int pos = 0;
	int i;

	/* samsung mipi rx cmd feature supports only one command */
	set = ss_get_cmds(vdd, type);
	if (!ss_is_read_cmd(type) || set->count != 1) {
		LCD_ERR("invalid set(%d): %s, count (%d)\n",
				type, ss_get_cmd_name(type), set->count);
		return -EINVAL;
	}

	/* enable level key */
	if (level_key & LEVEL1_KEY)
		ss_send_cmd(vdd, TX_LEVEL1_KEY_ENABLE);
	if (level_key & LEVEL2_KEY)
		ss_send_cmd(vdd, TX_LEVEL2_KEY_ENABLE);
	if (level_key & POC_KEY)
		ss_send_cmd(vdd, TX_POC_KEY_ENABLE);

	orig_rx_len = set->cmds[0].msg.rx_len;
	orig_offset = set->read_startoffset;

	/* reset rx len/offset */
	set->cmds[0].msg.rx_len += orig_offset;
	set->read_startoffset = 0;

	set->cmds[0].msg.rx_buf = temp_buffer;

	LCD_DEBUG("orig_rx_len (%d) , orig_offset (%d) \n", orig_rx_len, orig_offset);
//	LCD_DEBUG("new_rx_len (%d) , new_offset (%d) \n", set->cmds[0].msg.rx_len, set->read_startoffset);

	/* RX */
	ss_send_cmd(vdd, type);

	/* oopy to buffer from original offset */
	if (buffer)
		memcpy(buffer, temp_buffer + orig_offset, orig_rx_len);

	/* show buffer */
	if (buffer)
		for (i = 0; i < orig_rx_len; i++)
			pos += snprintf(show_buffer + pos, sizeof(show_buffer) - pos, "%02x ",
				buffer[i]);

	/* restore rx len/offset */
	set->cmds[0].msg.rx_len = orig_rx_len;
	set->read_startoffset = orig_offset;

	/* disable level key */
	if (level_key & POC_KEY)
		ss_send_cmd(vdd, TX_POC_KEY_DISABLE);
	if (level_key & LEVEL1_KEY)
		ss_send_cmd(vdd, TX_LEVEL1_KEY_DISABLE);
	if (level_key & LEVEL2_KEY)
		ss_send_cmd(vdd, TX_LEVEL2_KEY_DISABLE);

	if (type != RX_FLASH_GAMMA && type != RX_POC_READ)
		LCD_INFO("[%d]%s, addr: 0x%x, off: %d, len: %d, buf: %s\n",
				type, ss_get_cmd_name(type),
				set->cmds[0].msg.tx_buf[0], orig_offset, orig_rx_len,
				show_buffer);

	return 0;
}

int ss_panel_data_read_gpara(struct samsung_display_driver_data *vdd,
		enum dsi_cmd_set_type type, u8 *buffer, int level_key)
{
	struct dsi_panel_cmd_set *set;
	struct dsi_panel_cmd_set *read_pos_cmd;
	static u8 rx_buffer[MAX_LEN_RX_BUF] = {0,};
	char show_buffer[MAX_LEN_RX_BUF] = {0,};
	char temp_buffer[MAX_LEN_RX_BUF] = {0,};
	char rx_addr = 0;
	int orig_rx_len = 0;
	int new_rx_len = 0;
	int orig_offset = 0;
	int new_offset = 0;
	int loop_limit = 0;
	int show_cnt = 0;
	int pos = 0;
	int i, j;

	/* samsung mipi rx cmd feature supports only one command */
	set = ss_get_cmds(vdd, type);
	if (!ss_is_read_cmd(type) || set->count != 1) {
		LCD_ERR("invalid set(%d): %s, count (%d)\n",
				type, ss_get_cmd_name(type), set->count);
		return -EINVAL;
	}

	/* enable level key */
	if (level_key & LEVEL1_KEY)
		ss_send_cmd(vdd, TX_LEVEL1_KEY_ENABLE);
	if (level_key & LEVEL2_KEY)
		ss_send_cmd(vdd, TX_LEVEL2_KEY_ENABLE);
	if (level_key & POC_KEY)
		ss_send_cmd(vdd, TX_POC_KEY_ENABLE);

	set->cmds[0].msg.rx_buf = rx_buffer;

	read_pos_cmd = ss_get_cmds(vdd, TX_REG_READ_POS);
	if (SS_IS_CMDS_NULL(read_pos_cmd)) {
		LCD_ERR("No cmds for TX_REG_READ_POS.. \n");
		return -EINVAL;
	}

	rx_addr = set->cmds[0].msg.tx_buf[0];
	orig_rx_len = set->cmds[0].msg.rx_len;
	orig_offset = new_offset = set->read_startoffset;

	loop_limit = (orig_rx_len + RX_SIZE_LIMIT - 1) / RX_SIZE_LIMIT;

	/* set pointing gpara */
	if (read_pos_cmd->cmds->msg.tx_len == 3)
		read_pos_cmd->cmds->msg.tx_buf[2] = rx_addr;

	LCD_DEBUG("orig_rx_len (%d) , orig_offset (%d) loop_limit (%d)\n", orig_rx_len, orig_offset, loop_limit);

	for (i = 0; i < loop_limit; i++) {
		/* gPara */
		read_pos_cmd->cmds->msg.tx_buf[1] = new_offset;
		new_rx_len = ((orig_rx_len - new_offset + orig_offset) < RX_SIZE_LIMIT) ?
						(orig_rx_len - new_offset + orig_offset) : RX_SIZE_LIMIT;

		LCD_DEBUG("new_offset (%d) new_rx_len (%d) \n", new_offset, new_rx_len);

		ss_send_cmd(vdd, TX_REG_READ_POS);

		set->cmds[0].msg.rx_len = new_rx_len;

		/* RX */
		ss_send_cmd(vdd, type);

		/* oopy to buffer */
		if (buffer)
			memcpy(&buffer[show_cnt], rx_buffer, new_rx_len);

		/* snprint */
		memcpy(temp_buffer, set->cmds[0].msg.rx_buf, new_rx_len);
		for (j = 0; j < new_rx_len; j++, show_cnt++) {
			pos += snprintf(show_buffer + pos, sizeof(show_buffer) - pos, "%02x ",
				temp_buffer[j]);
		}

		/* increase gPara offset */
		new_offset += new_rx_len;

		if (new_offset - orig_offset >= orig_rx_len)
			break;
	}

	/* restore rx len */
	set->cmds[0].msg.rx_len = orig_rx_len;

	/* disable level key */
	if (level_key & POC_KEY)
		ss_send_cmd(vdd, TX_POC_KEY_DISABLE);
	if (level_key & LEVEL1_KEY)
		ss_send_cmd(vdd, TX_LEVEL1_KEY_DISABLE);
	if (level_key & LEVEL2_KEY)
		ss_send_cmd(vdd, TX_LEVEL2_KEY_DISABLE);

	if (type != RX_FLASH_GAMMA && type != RX_POC_READ)
		LCD_INFO("[%d] %s, addr: 0x%x, off: %d, len: %d, buf: %s\n",
				type, ss_get_cmd_name(type),
				set->cmds[0].msg.tx_buf[0], orig_offset, orig_rx_len,
				show_buffer);

	return 0;
}

int ss_panel_data_read(struct samsung_display_driver_data *vdd,
		enum dsi_cmd_set_type type, u8 *buffer, int level_key)
{
	int ret;

	if (!ss_panel_attach_get(vdd)) {
		LCD_ERR("ss_panel_attach_get(%d) : %d\n",
				vdd->ndx, ss_panel_attach_get(vdd));
		return -EPERM;
	}

	if (ss_is_panel_off(vdd)) {
		LCD_ERR("skip to rx cmd (%d), panel off (%d)\n",
				type, ss_is_panel_off(vdd));
		return -EPERM;
	}

	/* To block read operation at esd-recovery */
	if (vdd->panel_dead) {
		LCD_ERR("esd recovery, skip %s\n", ss_get_cmd_name(type));
		return -EPERM;
	}

	if (vdd->gpara)
		ret = ss_panel_data_read_gpara(vdd, type, buffer, level_key);
	else
		ret = ss_panel_data_read_no_gpara(vdd, type, buffer, level_key);

	return ret;
}

struct STM_REG_OSSET stm_offset_list[] = {
	{.name = "stm_ctrl_en=", 		.offset = offsetof(struct STM_CMD, STM_CTRL_EN)},
	{.name = "stm_max_opt=", 		.offset = offsetof(struct STM_CMD, STM_MAX_OPT)},
	{.name = "stm_default_opt=", 	.offset = offsetof(struct STM_CMD, STM_DEFAULT_OPT)},
	{.name = "stm_dim_step=", 		.offset = offsetof(struct STM_CMD, STM_DIM_STEP)},
	{.name = "stm_frame_period=",	.offset = offsetof(struct STM_CMD, STM_FRAME_PERIOD)},
	{.name = "stm_min_sect=", 		.offset = offsetof(struct STM_CMD, STM_MIN_SECT)},
	{.name = "stm_pixel_period=",	.offset = offsetof(struct STM_CMD, STM_PIXEL_PERIOD)},
	{.name = "stm_line_period=", 	.offset = offsetof(struct STM_CMD, STM_LINE_PERIOD)},
	{.name = "stm_min_move=", 		.offset = offsetof(struct STM_CMD, STM_MIN_MOVE)},
	{.name = "stm_m_thres=", 		.offset = offsetof(struct STM_CMD, STM_M_THRES)},
	{.name = "stm_v_thres=", 		.offset = offsetof(struct STM_CMD, STM_V_THRES)},
};

int ss_stm_set_cmd_offset(struct STM_CMD *cmd, char* p)
{
	int list_size = ARRAY_SIZE(stm_offset_list);
	const char *name;
	int i, val;
	int *offset;

	for (i = 0; i < list_size; i++) {
		name = stm_offset_list[i].name;
		if (!strncmp(name, p, strlen(name))) {
			sscanf(p + strlen(name), "%d", &val);
			offset = (int *)((void*)cmd + stm_offset_list[i].offset);
			*offset = val;
			return 0;
		}
	}

	return -1;
}

void print_stm_cmd(struct STM_CMD cmd)
{
	LCD_INFO("CTRL_EN(%d) MAX_OPT(%d) DEFAULT_OPT(%d) DIM_STEP(%d)\n",
		cmd.STM_CTRL_EN, cmd.STM_MAX_OPT, cmd.STM_DEFAULT_OPT, cmd.STM_DIM_STEP);
	LCD_INFO("FRAME_PERIOD(%d) MIN_SECT(%d) PIXEL_PERIOD(%d) LINE_PEROID(%d) \n",
		cmd.STM_FRAME_PERIOD, cmd.STM_MIN_SECT, cmd.STM_PIXEL_PERIOD, cmd.STM_LINE_PERIOD);
	LCD_INFO("MIN_MOVE(%d) M_THRES(%d) V_THRES(%d)\n",
		cmd.STM_MIN_MOVE, cmd.STM_M_THRES, cmd.STM_V_THRES);
}

/**
 * ss_get_stm_orig_cmd - get original stm cmd from panel dtsi.
 */
int ss_get_stm_orig_cmd(struct samsung_display_driver_data *vdd)
{
	struct dsi_panel_cmd_set *pcmds = NULL;
	struct STM_CMD *cmd = &vdd->stm.orig_cmd;
	u8 *cmd_pload;
	int cmd_len;
	char buf[256];
	int i, len = 0;

	pcmds = ss_get_cmds(vdd, TX_STM_ENABLE);
	if (SS_IS_CMDS_NULL(pcmds)) {
		LCD_ERR("No cmds for TX_STM_ENABLE..\n");
		return -EINVAL;
	}

	cmd_pload = pcmds->cmds[1].msg.tx_buf;
	cmd_len = pcmds->cmds[1].msg.tx_len;

	cmd->STM_CTRL_EN = (cmd_pload[1] & 0x01) ? 1 : 0;
	cmd->STM_MAX_OPT = (cmd_pload[3] & 0xF0) >> 4;

	cmd->STM_DEFAULT_OPT = (cmd_pload[3] & 0x0F);

	cmd->STM_DIM_STEP = (cmd_pload[4] & 0x0F);

	cmd->STM_FRAME_PERIOD = (cmd_pload[5] & 0xF0) >> 4;
	cmd->STM_MIN_SECT = (cmd_pload[5] & 0x0F);

	cmd->STM_PIXEL_PERIOD = (cmd_pload[6] & 0xF0) >> 4;
	cmd->STM_LINE_PERIOD = (cmd_pload[6] & 0x0F);

	cmd->STM_MIN_MOVE = cmd_pload[7];
	cmd->STM_M_THRES = cmd_pload[8];
	cmd->STM_V_THRES = cmd_pload[9];

	/* init current cmd with origianl cmd */
	memcpy(&vdd->stm.cur_cmd, cmd, sizeof(struct STM_CMD));

	print_stm_cmd(vdd->stm.cur_cmd);

	for (i = 0; i < cmd_len; i++)
		len += snprintf(buf + len, sizeof(buf) - len,
						"%02x ", cmd_pload[i]);
	LCD_ERR("cmd[%d] : %s\n", cmd_len, buf);

	return 1;
}

/**
 * ss_stm_set_cmd - set stm mipi cmd using STM_CMD para.
 *
 * stm_cmd : new stm cmd to update.
 */
void ss_stm_set_cmd(struct samsung_display_driver_data *vdd, struct STM_CMD *cmd)
{
	struct dsi_panel_cmd_set *pcmds = NULL;
	int cmd_len;
	u8 *cmd_pload;
	char buf[256];
	int i, len = 0;

	pcmds = ss_get_cmds(vdd, TX_STM_ENABLE);
	if (SS_IS_CMDS_NULL(pcmds)) {
		LCD_ERR("No cmds for TX_STM_ENABLE..\n");
		return;
	}

	cmd_pload = pcmds->cmds[1].msg.tx_buf;
	cmd_len = pcmds->cmds[1].msg.tx_len;

	cmd_pload[1] = cmd->STM_CTRL_EN ? 0x01 : 0x00;

	cmd_pload[3] = ((cmd->STM_MAX_OPT & 0x0F) << 4);
	cmd_pload[3] |= (cmd->STM_DEFAULT_OPT & 0x0F);

	cmd_pload[4] |= (cmd->STM_DIM_STEP & 0x0F);

	cmd_pload[5] = ((cmd->STM_FRAME_PERIOD & 0x0F) << 4);
	cmd_pload[5] |= (cmd->STM_MIN_SECT & 0x0F);

	cmd_pload[6] = ((cmd->STM_PIXEL_PERIOD & 0x0F) << 4);
	cmd_pload[6] |= (cmd->STM_LINE_PERIOD & 0x0F);

	cmd_pload[7] = cmd->STM_MIN_MOVE & 0xFF;
	cmd_pload[8] = cmd->STM_M_THRES & 0xFF;
	cmd_pload[9] = cmd->STM_V_THRES & 0xFF;

	for (i = 0; i < cmd_len; i++)
		len += snprintf(buf + len, sizeof(buf) - len,
						"%02x ", cmd_pload[i]);
	LCD_ERR("cmd[%d] : %s\n", cmd_len, buf);

	/* reset current stm cmds */
	memcpy(&vdd->stm.cur_cmd, cmd, sizeof(struct STM_CMD));

	return;
}

int ss_panel_on_pre(struct samsung_display_driver_data *vdd)
{
	int rddpm, rddsm, errfg, dsierror;

	/* At this time, it already enabled SDE clock/power and  display power.
	 * It is possible to send mipi comamnd to display.
	 * To send mipi command, like mdnie setting or brightness setting,
	 * change panel_state: PANEL_PWR_OFF -> PANEL_PWR_ON_READY, here.
	 */
	vdd->panel_state = PANEL_PWR_ON_READY;

	vdd->display_status_dsi.disp_on_pre = 1;

	if (vdd->other_line_panel_work_cnt)
		vdd->other_line_panel_work_cnt = 0; /*stop open otherline dat file*/

#if !defined(CONFIG_SEC_FACTORY)
	/* LCD ID read every wake_up time incase of factory binary */
	if (vdd->dtsi_data.tft_common_support)
		return false;
#endif

	if (!ss_panel_attach_get(vdd)) {
		LCD_ERR("ss_panel_attach_get NG\n");
		return false;
	}

	LCD_INFO("+\n");

	if (unlikely(!vdd->esd_recovery.esd_recovery_init))
		ss_event_esd_recovery_init(vdd, 0, NULL);

	if (unlikely(vdd->is_factory_mode) &&
			vdd->dtsi_data.samsung_support_factory_panel_swap) {
		/* LCD ID read every wake_up time incase of factory binary */
		vdd->manufacture_id_dsi = PBA_ID;

		/* Factory Panel Swap*/
		vdd->manufacture_date_loaded_dsi = 0;
		vdd->ddi_id_loaded_dsi = 0;
		vdd->module_info_loaded_dsi = 0;
		vdd->cell_id_loaded_dsi = 0;
		vdd->octa_id_loaded_dsi = 0;
		vdd->hbm_loaded_dsi = 0;
		vdd->mdnie_loaded_dsi = 0;
		vdd->smart_dimming_loaded_dsi = 0;
		vdd->smart_dimming_hmt_loaded_dsi = 0;
		vdd->table_interpolation_loaded = 0;
	}

	if (vdd->manufacture_id_dsi == PBA_ID) {
		u8 recv_buf[3];

		/*
		*	At this time, panel revision it not selected.
		*	So last index(SUPPORT_PANEL_REVISION-1) used.
		*/
		vdd->panel_revision = SUPPORT_PANEL_REVISION-1;

		/*
		*	Some panel needs to update register at init time to read ID & MTP
		*	Such as, dual-dsi-control or sleep-out so on.
		*/
		if ((ss_get_cmds(vdd, RX_MANUFACTURE_ID)->count)) {
			ss_panel_data_read(vdd, RX_MANUFACTURE_ID,
					recv_buf, LEVEL1_KEY);
		} else {
			LCD_INFO("manufacture_read_pre_tx_cmds\n");
			ss_send_cmd(vdd, TX_MANUFACTURE_ID_READ_PRE);

			ss_panel_data_read(vdd, RX_MANUFACTURE_ID0,
					recv_buf, LEVEL1_KEY);
			ss_panel_data_read(vdd, RX_MANUFACTURE_ID1,
					recv_buf + 1, LEVEL1_KEY);
			ss_panel_data_read(vdd, RX_MANUFACTURE_ID2,
					recv_buf + 2, LEVEL1_KEY);
		}

		vdd->manufacture_id_dsi =
			(recv_buf[0] << 16) | (recv_buf[1] << 8) | recv_buf[2];
		LCD_INFO("manufacture id: 0x%x\n", vdd->manufacture_id_dsi);

		/* Panel revision selection */
		if (IS_ERR_OR_NULL(vdd->panel_func.samsung_panel_revision))
			LCD_ERR("no panel_revision_selection_error fucntion\n");
		else
			vdd->panel_func.samsung_panel_revision(vdd);

		LCD_INFO("Panel_Revision = %c %d\n", vdd->panel_revision + 'A', vdd->panel_revision);
	}

	/* Read panel status to check panel is ok from bootloader */
	if (!vdd->read_panel_status_from_lk) {
		rddpm = ss_read_rddpm(vdd);
		rddsm = ss_read_rddsm(vdd);
		errfg = ss_read_errfg(vdd);
		dsierror = ss_read_dsierr(vdd);
		ss_read_pps_data(vdd);

		SS_XLOG(rddpm, rddsm, errfg, dsierror);
		LCD_INFO("panel dbg: %x %x %x %x\n", rddpm, rddsm, errfg, dsierror);

		vdd->read_panel_status_from_lk = 1;
	}

	/* Module info */
	if (!vdd->module_info_loaded_dsi) {
		if (IS_ERR_OR_NULL(vdd->panel_func.samsung_module_info_read))
			LCD_ERR("no samsung_module_info_read function\n");
		else
			vdd->module_info_loaded_dsi = vdd->panel_func.samsung_module_info_read(vdd);
	}

	/* Manufacture date */
	if (!vdd->manufacture_date_loaded_dsi) {
		if (IS_ERR_OR_NULL(vdd->panel_func.samsung_manufacture_date_read))
			LCD_ERR("no samsung_manufacture_date_read function\n");
		else
			vdd->manufacture_date_loaded_dsi = vdd->panel_func.samsung_manufacture_date_read(vdd);
	}

	/* DDI ID */
	if (!vdd->ddi_id_loaded_dsi) {
		if (IS_ERR_OR_NULL(vdd->panel_func.samsung_ddi_id_read))
			LCD_ERR("no samsung_ddi_id_read function\n");
		else
			vdd->ddi_id_loaded_dsi = vdd->panel_func.samsung_ddi_id_read(vdd);
	}

	/* Panel Unique OCTA ID */
	if (!vdd->octa_id_loaded_dsi) {
		if (IS_ERR_OR_NULL(vdd->panel_func.samsung_octa_id_read))
			LCD_ERR("no samsung_octa_id_read function\n");
		else
			vdd->octa_id_loaded_dsi = vdd->panel_func.samsung_octa_id_read(vdd);
	}

	/* ELVSS read */
	if (!vdd->elvss_loaded_dsi) {
		if (IS_ERR_OR_NULL(vdd->panel_func.samsung_elvss_read))
			LCD_ERR("no samsung_elvss_read function\n");
		else
			vdd->elvss_loaded_dsi = vdd->panel_func.samsung_elvss_read(vdd);
	}

	/* IRC read */
	if (!vdd->irc_loaded_dsi) {
		if (IS_ERR_OR_NULL(vdd->panel_func.samsung_irc_read))
			LCD_ERR("no samsung_irc_read function\n");
		else
			vdd->irc_loaded_dsi = vdd->panel_func.samsung_irc_read(vdd);
	}

	/* HBM */
	if (!vdd->hbm_loaded_dsi) {
		if (IS_ERR_OR_NULL(vdd->panel_func.samsung_hbm_read))
			LCD_ERR("no samsung_hbm_read function\n");
		else
			vdd->hbm_loaded_dsi = vdd->panel_func.samsung_hbm_read(vdd);
	}

	/* MDNIE X,Y */
	if (!vdd->mdnie_loaded_dsi) {
		if (IS_ERR_OR_NULL(vdd->panel_func.samsung_mdnie_read))
			LCD_ERR("no samsung_mdnie_read function\n");
		else
			vdd->mdnie_loaded_dsi = vdd->panel_func.samsung_mdnie_read(vdd);
	}

	/* Panel Unique Cell ID */
	if (!vdd->cell_id_loaded_dsi) {
		if (IS_ERR_OR_NULL(vdd->panel_func.samsung_cell_id_read))
			LCD_ERR("no samsung_cell_id_read function\n");
		else
			vdd->cell_id_loaded_dsi = vdd->panel_func.samsung_cell_id_read(vdd);
	}

	/* Smart dimming*/
	if (!vdd->smart_dimming_loaded_dsi) {
		if (IS_ERR_OR_NULL(vdd->panel_func.samsung_smart_dimming_init))
			LCD_ERR("no samsung_smart_dimming_init function\n");
		else
			vdd->smart_dimming_loaded_dsi = vdd->panel_func.samsung_smart_dimming_init(vdd);
	}

	/* Smart dimming for hmt */
	if (vdd->dtsi_data.hmt_enabled) {
		if (!vdd->smart_dimming_hmt_loaded_dsi) {
			if (IS_ERR_OR_NULL(vdd->panel_func.samsung_smart_dimming_hmt_init))
				LCD_ERR("no samsung_smart_dimming_hmt_init function\n");
			else
				vdd->smart_dimming_hmt_loaded_dsi = vdd->panel_func.samsung_smart_dimming_hmt_init(vdd);
		}
	}

	/* self display */
	if (!vdd->self_display_loaded_dsi) {
		if (IS_ERR_OR_NULL(vdd->self_disp.data_init))
			LCD_ERR("no self display data init function\n");
		else
			vdd->self_display_loaded_dsi = vdd->self_disp.data_init(vdd);
	}

	/* copr */
	if (!vdd->copr_load_init_cmd && vdd->copr.copr_on) {
		vdd->copr_load_init_cmd = ss_get_copr_orig_cmd(vdd);
	}

	/* stm */
	if (!vdd->stm_load_init_cmd) {
		vdd->stm_load_init_cmd = ss_get_stm_orig_cmd(vdd);
	}

	if (!vdd->table_interpolation_loaded) {
		if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_interpolation_init)) {
			table_br_func(vdd);
			vdd->table_interpolation_loaded = vdd->panel_func.samsung_interpolation_init(vdd, TABLE_INTERPOLATION);
		}
	}

	if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_panel_on_pre))
		vdd->panel_func.samsung_panel_on_pre(vdd);

	LCD_INFO("-\n");

	return true;
}

int ss_panel_on_post(struct samsung_display_driver_data *vdd)
{
	if (!ss_panel_attach_get(vdd)) {
		LCD_ERR("ss_panel_attach_get NG\n");
		return false;
	}

	LCD_INFO("+\n");

/* TODO: enable debug file and activate below...
	ss_read_self_diag(vdd);
*/

	if (vdd->support_cabc && !vdd->br.auto_level)
		ss_cabc_update(vdd);
	else if (vdd->ss_panel_tft_outdoormode_update && vdd->br.auto_level)
		vdd->ss_panel_tft_outdoormode_update(vdd);
	else if (vdd->support_cabc && vdd->br.auto_level)
		ss_tft_autobrightness_cabc_update(vdd);

	if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_panel_on_post))
		vdd->panel_func.samsung_panel_on_post(vdd);

#if 0	// ccb is set by mdnie cmd.
	if ((vdd->panel_func.color_weakness_ccb_on_off) && vdd->color_weakness_mode)
		vdd->panel_func.color_weakness_ccb_on_off(vdd, vdd->color_weakness_mode);
#endif

	if (vdd->support_hall_ic) {
		/*
		* Brightenss cmds sent by samsung_display_hall_ic_status() at panel switching operation
		*/
		if (ss_is_bl_dcs(vdd) &&
			(vdd->hall_ic_mode_change_trigger == false))
			ss_brightness_dcs(vdd, USE_CURRENT_BL_LEVEL, BACKLIGHT_NORMAL);
	} else {
		if (ss_is_bl_dcs(vdd)) {
			struct backlight_device *bd = GET_SDE_BACKLIGHT_DEVICE(vdd);

			/* In case of backlight update in panel off,
			 * dsi_display_set_backlight() returns error
			 * without updating vdd->br.bl_level.
			 * Update bl_level from bd->props.brightness.
			 */
			if (bd && vdd->br.bl_level != bd->props.brightness) {
				LCD_INFO("update bl_level: %d -> %d\n",
					vdd->br.bl_level, bd->props.brightness);
				vdd->br.bl_level = bd->props.brightness;
			}

			ss_brightness_dcs(vdd, USE_CURRENT_BL_LEVEL, BACKLIGHT_NORMAL);
		}
	}

	if (vdd->mdnie.support_mdnie) {
		vdd->mdnie.lcd_on_notifiy = true;
		update_dsi_tcon_mdnie_register(vdd);
		if (vdd->mdnie.support_trans_dimming)
			vdd->mdnie.disable_trans_dimming = false;
	}

	/*
	 * Update Cover Control Status every Normal sleep & wakeup
	 * Do not update Cover_control at this point in case of AOD.
	 * Because, below update is done before entering AOD.
	 */
	if (vdd->panel_func.samsung_cover_control && vdd->cover_control
		&& !ss_is_panel_lpm(vdd))
		vdd->panel_func.samsung_cover_control(vdd);

	/* Work around: For folder model, the status bar resizes itself and old UI appears in sub-panel
	 * before premium watch or screen lock is on, so it needs to skip old UI.
	 */
	if (!vdd->lcd_flip_not_refresh &&
			vdd->support_hall_ic &&
			vdd->hall_ic_mode_change_trigger &&
			vdd->lcd_flip_delay_ms) {
		schedule_delayed_work(&vdd->delay_disp_on_work, msecs_to_jiffies(vdd->lcd_flip_delay_ms));
	} else
		vdd->display_status_dsi.wait_disp_on = true;

	vdd->display_status_dsi.wait_actual_disp_on = true;
	vdd->display_status_dsi.aod_delay = true;

	if (ss_is_esd_check_enabled(vdd)) {
		if (vdd->esd_recovery.esd_irq_enable)
			vdd->esd_recovery.esd_irq_enable(true, true, (void *)vdd);
	}

	if (vdd->dyn_mipi_clk.is_support){
		LCD_INFO("FFC Setting for Dynamic MIPI Clock\n");
		ss_send_cmd(vdd, TX_FFC);
	}

	if (vdd->copr.copr_on)
		ss_send_cmd(vdd, TX_COPR_ENABLE);

	if (unlikely(vdd->is_factory_mode))
		ss_send_cmd(vdd, TX_FD_ON);

	vdd->panel_state = PANEL_PWR_ON;

	if (vdd->poc_driver.check_read_case)
		vdd->poc_driver.read_case = vdd->poc_driver.check_read_case(vdd);

	LCD_INFO("-\n");

	return true;
}

int ss_panel_off_pre(struct samsung_display_driver_data *vdd)
{
	int rddpm, rddsm, errfg, dsierror;
	int ret = 0;

	LCD_INFO("+\n");
	rddpm = ss_read_rddpm(vdd);
	rddsm = ss_read_rddsm(vdd);
	errfg = ss_read_errfg(vdd);
	dsierror = ss_read_dsierr(vdd);
	ss_read_pps_data(vdd);
	SS_XLOG(rddpm, rddsm, errfg, dsierror);
	LCD_INFO("panel dbg: %x %x %x %x\n", rddpm, rddsm, errfg, dsierror);

	if (ss_is_esd_check_enabled(vdd)) {
		vdd->esd_recovery.is_wakeup_source = false;
		if (vdd->esd_recovery.esd_irq_enable)
			vdd->esd_recovery.esd_irq_enable(false, true, (void *)vdd);
		vdd->esd_recovery.is_enabled_esd_recovery = false;
	}

	if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_panel_off_pre))
		vdd->panel_func.samsung_panel_off_pre(vdd);

	mutex_lock(&vdd->vdd_lock);
	vdd->panel_state = PANEL_PWR_OFF;
	mutex_unlock(&vdd->vdd_lock);

	LCD_INFO("-\n");

	return ret;
}

/*
 * Do not call ss_send_cmd() or ss_panel_data_read() here.
 * Any MIPI Tx/Rx can not be alowed in here.
 */
int ss_panel_off_post(struct samsung_display_driver_data *vdd)
{
	int ret = 0;

	LCD_INFO("+\n");

	if (vdd->mdnie.support_trans_dimming)
		vdd->mdnie.disable_trans_dimming = true;

	if (vdd->support_hall_ic && vdd->lcd_flip_delay_ms)
		cancel_delayed_work(&vdd->delay_disp_on_work);

	if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_panel_off_post))
		vdd->panel_func.samsung_panel_off_post(vdd);

	/* gradual acl on/off - not used */
//	vdd->gradual_pre_acl_on = GRADUAL_ACL_UNSTABLE;

	/* Reset Self Display Status */
	vdd->self_disp.sa_info.en = false;
	vdd->self_disp.sd_info.en = false;
	vdd->self_disp.si_info.en = false;
	vdd->self_disp.sg_info.en = false;
	vdd->self_disp.time_set = false;
	vdd->self_disp.on = false;

	if (vdd->finger_mask)
		vdd->finger_mask = false;

	/* To prevent panel off without finger off */
	if (vdd->br.finger_mask_hbm_on)
		vdd->br.finger_mask_hbm_on = false;

	LCD_INFO("-\n");
	SS_XLOG(SS_XLOG_FINISH);

	return ret;
}

/*************************************************************
*
*		TFT BACKLIGHT GPIO FUNCTION BELOW.
*
**************************************************************/
int ss_backlight_tft_request_gpios(struct samsung_display_driver_data *vdd)
{
	int rc = 0, i;
	/*
	 * gpio_name[] named as gpio_name + num(recomend as 0)
	 * because of the num will increase depend on number of gpio
	 */
	char gpio_name[17] = "disp_bcklt_gpio0";
	static u8 gpio_request_status = -EINVAL;

	if (!gpio_request_status)
		goto end;

	if (IS_ERR_OR_NULL(vdd)) {
		LCD_ERR("Invalid data pinfo : 0x%zx\n", (size_t)vdd);
		goto end;
	}

	for (i = 0; i < MAX_BACKLIGHT_TFT_GPIO; i++) {
		if (gpio_is_valid(vdd->dtsi_data.backlight_tft_gpio[i])) {
			rc = gpio_request(vdd->dtsi_data.backlight_tft_gpio[i],
							gpio_name);
			if (rc) {
				LCD_ERR("request %s failed, rc=%d\n", gpio_name, rc);
				goto tft_backlight_gpio_err;
			}
		}
	}

	gpio_request_status = rc;
end:
	return rc;
tft_backlight_gpio_err:
	if (i) {
		do {
			if (gpio_is_valid(vdd->dtsi_data.backlight_tft_gpio[i]))
				gpio_free(vdd->dtsi_data.backlight_tft_gpio[i--]);
			LCD_ERR("i = %d\n", i);
		} while (i > 0);
	}

	return rc;
}

int ss_backlight_tft_gpio_config(struct samsung_display_driver_data *vdd, int enable)
{
	int ret = 0, i = 0, add_value = 1;

	if (IS_ERR_OR_NULL(vdd)) {
		LCD_ERR("Invalid data pinfo : 0x%zx\n",  (size_t)vdd);
		goto end;
	}

	LCD_INFO("++ enable(%d) ndx(%d)\n", enable, ss_get_display_ndx(vdd));

	if (ss_backlight_tft_request_gpios(vdd)) {
		LCD_ERR("fail to request tft backlight gpios");
		goto end;
	}

	LCD_DEBUG("%s tft backlight gpios\n", enable ? "enable" : "disable");

	/*
	 * The order of backlight_tft_gpio enable/disable
	 * 1. Enable : backlight_tft_gpio[0], [1], ... [MAX_BACKLIGHT_TFT_GPIO - 1]
	 * 2. Disable : backlight_tft_gpio[MAX_BACKLIGHT_TFT_GPIO - 1], ... [1], [0]
	 */
	if (!enable) {
		add_value = -1;
		i = MAX_BACKLIGHT_TFT_GPIO - 1;
	}

	do {
		if (gpio_is_valid(vdd->dtsi_data.backlight_tft_gpio[i])) {
			gpio_set_value(vdd->dtsi_data.backlight_tft_gpio[i], enable);
			LCD_DEBUG("set backlight tft gpio[%d] to %s\n",
						 vdd->dtsi_data.backlight_tft_gpio[i],
						enable ? "high" : "low");
			usleep_range(500, 500);
		}
	} while (((i += add_value) < MAX_BACKLIGHT_TFT_GPIO) && (i >= 0));

end:
	LCD_INFO("--\n");
	return ret;
}

/*************************************************************
*
*		ESD RECOVERY RELATED FUNCTION BELOW.
*
**************************************************************/

#if 0
static int ss_esd_check_status(struct samsung_display_driver_data *vdd)
{
	LCD_INFO("lcd esd - check_ststus\n");
	return 0;
}
static int ss_esd_read_status(struct mdss_dsi_ctrl_pdata *ctrl)
{
	LCD_INFO("lcd esd - check_read_status\n");
	// esd status must return 0 to go status_dead(blank->unblnk) in ss_check_dsi_ctrl_status.
	return 0;
}
#endif

/*
 * esd_irq_enable() - Enable or disable esd irq.
 *
 * @enable	: flag for enable or disabled
 * @nosync	: flag for disable irq with nosync
 * @data	: point ot struct ss_panel_info
 */
#define IRQS_PENDING	0x00000200
#define istate core_internal_state__do_not_mess_with_it
static void esd_irq_enable(bool enable, bool nosync, void *data)
{
	/* The irq will enabled when do the request_threaded_irq() */
	static bool is_enabled[MAX_DISPLAY_NDX] = {true, true};
	static bool is_wakeup_source[MAX_DISPLAY_NDX];
	int gpio;
	int irq[MAX_ESD_GPIO] = {0,};
	unsigned long flags;
	struct samsung_display_driver_data *vdd =
		(struct samsung_display_driver_data *)data;
	struct irq_desc *desc = NULL;
	u8 i = 0;

	if (IS_ERR_OR_NULL(vdd)) {
		LCD_ERR("Invalid data vdd : 0x%zx\n", (size_t)vdd);
		return;
	}

	for (i = 0; i < vdd->esd_recovery.num_of_gpio; i++) {
		gpio = vdd->esd_recovery.esd_gpio[i];

		if (!gpio_is_valid(gpio)) {
			LCD_ERR("Invalid ESD_GPIO : %d\n", gpio);
			continue;
		}
		irq[i] = gpio_to_irq(gpio);
	}

	spin_lock_irqsave(&vdd->esd_recovery.irq_lock, flags);

	if (enable == is_enabled[vdd->ndx]) {
		LCD_INFO("ESD irq already %s\n",
				enable ? "enabled" : "disabled");
		goto config_wakeup_source;
	}


	for (i = 0; i < vdd->esd_recovery.num_of_gpio; i++) {
		gpio = vdd->esd_recovery.esd_gpio[i];
		desc = irq_to_desc(irq[i]);
		if (enable) {
			/* clear esd irq triggered while it was disabled. */
			if (desc->istate & IRQS_PENDING) {
				LCD_INFO("clear esd irq pending status\n");
				desc->istate &= ~IRQS_PENDING;
			}
		}

		if (!gpio_is_valid(gpio)) {
			LCD_ERR("Invalid ESD_GPIO : %d\n", gpio);
			continue;
		}

		if (enable) {
			is_enabled[vdd->ndx] = true;
			enable_irq(irq[i]);
		} else {
			if (nosync)
				disable_irq_nosync(irq[i]);
			else
				disable_irq(irq[i]);
			is_enabled[vdd->ndx] = false;
		}
	}

	/* TODO: Disable log if the esd function stable */
	LCD_INFO("ndx=%d, ESD irq %s with %s\n",
				vdd->ndx, enable ? "enabled" : "disabled",
				nosync ? "nosync" : "sync");

config_wakeup_source:
	if (vdd->esd_recovery.is_wakeup_source == is_wakeup_source[vdd->ndx]) {
		LCD_DEBUG("[ESD] IRQs are already irq_wake %s\n",
				is_wakeup_source[vdd->ndx] ? "enabled" : "disabled");
		goto end;
	}

	for (i = 0; i < vdd->esd_recovery.num_of_gpio; i++) {
		gpio = vdd->esd_recovery.esd_gpio[i];

		if (!gpio_is_valid(gpio)) {
			LCD_ERR("Invalid ESD_GPIO : %d\n", gpio);
			continue;
		}

		is_wakeup_source[vdd->ndx] =
			vdd->esd_recovery.is_wakeup_source;

		if (is_wakeup_source[vdd->ndx])
			enable_irq_wake(irq[i]);
		else
			disable_irq_wake(irq[i]);
	}

	LCD_INFO("[ESD] ndx=%d, IRQs are set to irq_wake %s\n",
				vdd->ndx, is_wakeup_source[vdd->ndx] ? "enabled" : "disabled");

end:
	spin_unlock_irqrestore(&vdd->esd_recovery.irq_lock, flags);
}

static irqreturn_t esd_irq_handler(int irq, void *handle)
{
	struct samsung_display_driver_data *vdd =
		(struct samsung_display_driver_data *) handle;
	struct sde_connector *conn = GET_SDE_CONNECTOR(vdd);

	if (!vdd->esd_recovery.is_enabled_esd_recovery) {
		LCD_ERR("esd recovery is not enabled yet");
		goto end;
	}

	if (unlikely(vdd->is_factory_mode)) {
		if (gpio_is_valid(vdd->ub_con_det.gpio)) {
			pr_info("ub_con_det.gpio = %d\n", gpio_get_value(vdd->ub_con_det.gpio));
		}
	}

	LCD_INFO("++\n");

	esd_irq_enable(false, true, (void *)vdd);

	vdd->panel_lpm.esd_recovery = true;

	schedule_work(&conn->status_work.work);

	LCD_INFO("--\n");

end:
	return IRQ_HANDLED;
}

// refer to dsi_display_res_init() and dsi_panel_get(&display->pdev->dev, display->panel_of);
static void ss_panel_parse_dt_esd(struct device_node *np,
		struct samsung_display_driver_data *vdd)
{
	int rc = 0;
	const char *data;
	char esd_irq_gpio[] = "samsung,esd-irq-gpio1";
	char esd_irqflag[] = "qcom,mdss-dsi-panel-status-irq-trigger1";
	struct esd_recovery *esd = NULL;
	struct dsi_panel *panel = NULL;
	struct drm_panel_esd_config *esd_config = NULL;
	u8 i = 0;

	panel = (struct dsi_panel *)vdd->msm_private;
	esd_config = &panel->esd_config;

	esd = &vdd->esd_recovery;
	esd->num_of_gpio = 0;

	esd_config->esd_enabled = of_property_read_bool(np,
		"qcom,esd-check-enabled");

	if (!esd_config->esd_enabled)
		goto end;

	for (i = 0; i < MAX_ESD_GPIO; i++) {
		esd_irq_gpio[strlen(esd_irq_gpio) - 1] = '1' + i;
		esd->esd_gpio[esd->num_of_gpio] = of_get_named_gpio(np,
				esd_irq_gpio, 0);

		if (gpio_is_valid(esd->esd_gpio[esd->num_of_gpio])) {
			LCD_INFO("[ESD] gpio : %d, irq : %d\n",
					esd->esd_gpio[esd->num_of_gpio],
					gpio_to_irq(esd->esd_gpio[esd->num_of_gpio]));
			esd->num_of_gpio++;
		}
	}

	rc = of_property_read_string(np, "qcom,mdss-dsi-panel-status-check-mode", &data);
	if (!rc) {
		if (!strcmp(data, "irq_check"))
			esd_config->status_mode = ESD_MODE_PANEL_IRQ;
		else
			LCD_ERR("No valid panel-status-check-mode string\n");
	}

	for (i = 0; i < esd->num_of_gpio; i++) {
		esd_irqflag[strlen(esd_irqflag) - 1] = '1' + i;
		rc = of_property_read_string(np, esd_irqflag, &data);
		if (!rc) {
			esd->irqflags[i] =
				IRQF_ONESHOT | IRQF_NO_SUSPEND;

			if (!strcmp(data, "rising"))
				esd->irqflags[i] |= IRQF_TRIGGER_RISING;
			else if (!strcmp(data, "falling"))
				esd->irqflags[i] |= IRQF_TRIGGER_FALLING;
			else if (!strcmp(data, "high"))
				esd->irqflags[i] |= IRQF_TRIGGER_HIGH;
			else if (!strcmp(data, "low"))
				esd->irqflags[i] |= IRQF_TRIGGER_LOW;
		}
	}

end:
	LCD_INFO("samsung esd %s mode (%d)\n",
		esd_config->esd_enabled ? "enabled" : "disabled",
		esd_config->status_mode);
	return;
}

static void ss_event_esd_recovery_init(
		struct samsung_display_driver_data *vdd, int event, void *arg)
{
	// TODO: implement after qcomm esd bsp appllied...
	int ret;
	u8 i;
	int gpio, irqflags;
	struct esd_recovery *esd = NULL;
	struct dsi_panel *panel = NULL;
	struct drm_panel_esd_config *esd_config = NULL;

	panel = (struct dsi_panel *)vdd->msm_private;
	esd_config = &panel->esd_config;

	esd = &vdd->esd_recovery;

	if (IS_ERR_OR_NULL(vdd)) {
		LCD_ERR("Invalid data vdd : 0x%zx\n", (size_t)vdd);
		return;
	}

	LCD_INFO("++\n");

	if (unlikely(!esd->esd_recovery_init)) {
		esd->esd_recovery_init = true;
		esd->esd_irq_enable = esd_irq_enable;
		if (esd_config->status_mode  == ESD_MODE_PANEL_IRQ) {
			for (i = 0; i < esd->num_of_gpio; i++) {
				/* Set gpio num and irqflags */
				gpio = esd->esd_gpio[i];
				irqflags = esd->irqflags[i];
				if (!gpio_is_valid(gpio)) {
					LCD_ERR("[ESD] Invalid GPIO : %d\n", gpio);
					continue;
				}

				gpio_request(gpio, "esd_recovery");
				ret = request_threaded_irq(
						gpio_to_irq(gpio),
						NULL,
						esd_irq_handler,
						irqflags,
						"esd_recovery",
						(void *)vdd);
				if (ret)
					LCD_ERR("Failed to request_irq, ret=%d\n",
							ret);
				else
					LCD_INFO("request esd irq !!\n");
			}
			esd_irq_enable(false, true, (void *)vdd);
		}
	}

	LCD_INFO("--\n");
}

static void ss_panel_recovery(struct samsung_display_driver_data *vdd)
{
	struct sde_connector *conn = GET_SDE_CONNECTOR(vdd);

	if (IS_ERR_OR_NULL(vdd)) {
		LCD_ERR("Invalid data vdd : 0x%zx\n", (size_t)vdd);
		return;
	}

	if (!vdd->esd_recovery.is_enabled_esd_recovery) {
		LCD_ERR("esd recovery is not enabled yet");
		return;
	}
	LCD_INFO("Panel Recovery, Trial Count = %d\n", vdd->panel_recovery_cnt++);
	SS_XLOG(vdd->panel_recovery_cnt);
	inc_dpui_u32_field(DPUI_KEY_QCT_RCV_CNT, 1);

	esd_irq_enable(false, true, (void *)vdd);
	vdd->panel_lpm.esd_recovery = true;
	schedule_work(&conn->status_work.work);

	LCD_INFO("Panel Recovery --\n");

}

void ss_send_ub_uevent(struct samsung_display_driver_data *vdd)
{
	char *envp[3] = {"CONNECTOR_NAME=UB_CONNECT", "CONNECTOR_TYPE=HIGH_LEVEL", NULL};

	LCD_INFO("send uvent \n");
	kobject_uevent_env(&vdd->lcd_dev->dev.kobj, KOBJ_CHANGE, envp);

	return;
}

static irqreturn_t ss_ub_con_det_handler(int irq, void *handle)
{
	struct samsung_display_driver_data *vdd =
		(struct samsung_display_driver_data *) handle;

	LCD_INFO("ub_con_det for ndx%d is [%s] \n", vdd->ndx, vdd->ub_con_det.enabled ? "enabled" : "disabled");

	/* check gpio status one more */
	if (!gpio_get_value(vdd->ub_con_det.gpio))
		LCD_INFO("interrupt happens but panel is attached.\n");
	else {
		if (vdd->ub_con_det.enabled)
			ss_send_ub_uevent(vdd);
		vdd->ub_con_det.ub_con_cnt++;
	}

	LCD_ERR("-- cnt for ndx%d : %d\n", vdd->ndx, vdd->ub_con_det.ub_con_cnt);

	return IRQ_HANDLED;
}

static void ss_panel_parse_dt_ub_con(struct device_node *np,
		struct samsung_display_driver_data *vdd)
{
	struct ub_con_detect *ub_con_det;
	int ret;

	ub_con_det = &vdd->ub_con_det;

	ub_con_det->gpio = of_get_named_gpio(np,
			"samsung,ub-con-det", 0);

	LCD_INFO("request ub_con_det irq for ndx%d\n", vdd->ndx);

	if (gpio_is_valid(ub_con_det->gpio)) {
		ret = gpio_request(ub_con_det->gpio, "UB_CON_DET");
		if (ret)
			LCD_INFO("[ub_con_det ndx%d] fail to gpio_request.. %d\n", vdd->ndx, ret);
		else
			LCD_INFO("[ub_con_det ndx%d] gpio : %d, irq : %d\n",
					vdd->ndx, ub_con_det->gpio, gpio_to_irq(ub_con_det->gpio));
	} else {
		LCD_ERR("fail to gpio_request\n");
		return;
	}

	ret = request_threaded_irq(
				gpio_to_irq(ub_con_det->gpio),
				NULL,
				ss_ub_con_det_handler,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"UB_CON_DET",
				(void *) vdd);
	if (ret)
		LCD_ERR("Failed to request_irq, ret=%d\n", ret);

	return;
}

static void ss_panel_parse_dt_ub_fd(struct device_node *np,
		struct samsung_display_driver_data *vdd)
{
	struct ub_fast_discharge *ub_fd;
	int ret;
	u32 tmp[2];

	ub_fd = &vdd->ub_fd;

	ub_fd->gpio = of_get_named_gpio(np, "samsung,ub-fd-gpio", 0);

	LCD_INFO("request ub_fd gpio(%d) for ndx%d\n", ub_fd->gpio, vdd->ndx);

	if (gpio_is_valid(ub_fd->gpio)) {
		ret = gpio_request(ub_fd->gpio, "UB_FD_GPIO");
		if (ret) {
			LCD_INFO("[ub_fd ndx%d] fail to gpio_request.. %d\n", vdd->ndx, ret);
			ub_fd->enabled = 0;
		} else {
			ub_fd->enabled = 1;
			ret = of_property_read_u32(np, "samsung,ub-fd-on-count", tmp);
			ub_fd->fd_on_count = (!ret ? tmp[0] : 25);
			ret = of_property_read_u32(np, "samsung,ub-fd-off-count", tmp);
			ub_fd->fd_off_count = (!ret ? tmp[0] : 26);
			LCD_INFO("[ub_fd ndx%d] gpio : %d, on_cnt=%d, off_cnt=%d\n", vdd->ndx, ub_fd->gpio, ub_fd->fd_on_count, ub_fd->fd_off_count);
		}
	} else {
		LCD_ERR("fail to gpio_request\n");
		ub_fd->enabled = 0;
		return;
	}
	return;
}

/*************************************************************
*
*		OSC TE FITTING RELATED FUNCTION BELOW.
*
**************************************************************/
static void ss_event_osc_te_fitting(
		struct samsung_display_driver_data *vdd, int event, void *arg)
{
	struct osc_te_fitting_info *te_info = NULL;
	int ret, i, lut_count;
	unsigned int disp_te_gpio;

	te_info = &vdd->te_fitting_info;

	if (IS_ERR_OR_NULL(te_info)) {
		LCD_ERR("Invalid te data : 0x%zx\n",
				(size_t)te_info);
		return;
	}

	if (ss_is_seamless_mode(vdd)) {
		LCD_ERR("cont splash enabled\n");
		return;
	}

	if (!ss_is_vsync_enabled(vdd)) {
		LCD_DEBUG("vsync handler does not enabled yet\n");
		return;
	}

	te_info->status |= TE_FITTING_DONE;

	LCD_DEBUG("++\n");

	disp_te_gpio = ss_get_te_gpio(vdd);

	if (!(te_info->status & TE_FITTING_REQUEST_IRQ)) {
		te_info->status |= TE_FITTING_REQUEST_IRQ;


		ret = request_threaded_irq(
				gpio_to_irq(disp_te_gpio),
				samsung_te_check_handler,
				NULL,
				IRQF_TRIGGER_FALLING,
				"VSYNC_GPIO",
				(void *) vdd);
		if (ret)
			LCD_ERR("Failed to request_irq, ret=%d\n",
					ret);
		else
			disable_irq(gpio_to_irq(disp_te_gpio));
		te_info->te_time =
			kzalloc(sizeof(long long) * te_info->sampling_rate, GFP_KERNEL);
		INIT_WORK(&te_info->work, samsung_te_check_done_work);
	}

	for (lut_count = 0; lut_count < OSC_TE_FITTING_LUT_MAX; lut_count++) {
		init_completion(&te_info->te_check_comp);
		te_info->status |= TE_CHECK_ENABLE;
		te_info->te_duration = 0;

		LCD_DEBUG("osc_te_fitting _irq : %d\n",
				gpio_to_irq(disp_te_gpio));

		enable_irq(gpio_to_irq(disp_te_gpio));
		ret = wait_for_completion_timeout(
				&te_info->te_check_comp, 1000);

		if (ret <= 0)
			LCD_ERR("timeout\n");

		for (i = 0; i < te_info->sampling_rate; i++) {
			te_info->te_duration +=
				(i != 0 ? (te_info->te_time[i] - te_info->te_time[i-1]) : 0);
			LCD_DEBUG("vsync time : %lld, sum : %lld\n",
					te_info->te_time[i], te_info->te_duration);
		}
		do_div(te_info->te_duration, te_info->sampling_rate - 1);
		LCD_INFO("ave vsync time : %lld\n",
				te_info->te_duration);
		te_info->status &= ~TE_CHECK_ENABLE;

		if (vdd->panel_func.samsung_osc_te_fitting)
			ret = vdd->panel_func.samsung_osc_te_fitting(vdd);

		if (!ret)
			ss_send_cmd(vdd, TX_OSC_TE_FITTING);
		else
			break;
	}
	LCD_DEBUG("--\n");
}

static void samsung_te_check_done_work(struct work_struct *work)
{
	struct osc_te_fitting_info *te_info = NULL;

	te_info = container_of(work, struct osc_te_fitting_info, work);

	if (IS_ERR_OR_NULL(te_info)) {
		LCD_ERR("Invalid TE tuning data\n");
		return;
	}

	complete_all(&te_info->te_check_comp);
}

static irqreturn_t samsung_te_check_handler(int irq, void *handle)
{
	struct samsung_display_driver_data *vdd = NULL;
	struct osc_te_fitting_info *te_info = NULL;
	static bool skip_first_te = true;
	static u8 count;

	if (skip_first_te) {
		skip_first_te = false;
		goto end;
	}

	if (IS_ERR_OR_NULL(handle)) {
		LCD_ERR("handle is null\n");
		goto end;
	}

	te_info = &vdd->te_fitting_info;


	if (!(te_info->status & TE_CHECK_ENABLE))
		goto end;

	if (count < te_info->sampling_rate) {
		te_info->te_time[count++] =
			ktime_to_us(ktime_get());
	} else {
		disable_irq_nosync(gpio_to_irq(ss_get_te_gpio(vdd)));
		schedule_work(&te_info->work);
		skip_first_te = true;
		count = 0;
	}

end:
	return IRQ_HANDLED;
}

/*************************************************************
*
*		LDI FPS RELATED FUNCTION BELOW.
*
**************************************************************/
int ldi_fps(struct samsung_display_driver_data *vdd, unsigned int input_fps)
{
	int rc = 0;

	if (IS_ERR_OR_NULL(vdd)) {
		LCD_ERR("Invalid data vdd : 0x%zx\n", (size_t)vdd);
		return 0;
	}

	LCD_INFO("input_fps = %d\n", input_fps);

	if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_change_ldi_fps))
		rc = vdd->panel_func.samsung_change_ldi_fps(vdd, input_fps);
	else {
		LCD_ERR("samsung_change_ldi_fps function is NULL\n");
		return 0;
	}

	if (rc)
		ss_send_cmd(vdd, TX_LDI_FPS_CHANGE);

	return rc;
}
EXPORT_SYMBOL(ldi_fps);

int ss_set_backlight(struct samsung_display_driver_data *vdd, u32 bl_lvl)
{
	struct dsi_panel *panel = NULL;
	int ret = 0;

	if (IS_ERR_OR_NULL(vdd)) {
		ret = -EINVAL;
		goto end;
	}

	panel = GET_DSI_PANEL(vdd);
	if (IS_ERR_OR_NULL(panel)) {
		LCD_ERR("panel is NULL\n");
		ret = -EINVAL;
		goto end;
	}

	ret = dsi_panel_set_backlight(panel, bl_lvl);

end:
	return ret;
}

/*************************************************************
*
*		HMT RELATED FUNCTION BELOW.
*
**************************************************************/
int hmt_bright_update(struct samsung_display_driver_data *vdd)
{
	if (vdd->hmt_stat.hmt_on) {
		ss_brightness_dcs_hmt(vdd, vdd->hmt_stat.hmt_bl_level);
	} else {
		ss_brightness_dcs(vdd, USE_CURRENT_BL_LEVEL, BACKLIGHT_NORMAL);
		LCD_INFO("hmt off state!\n");
	}

	return 0;
}

int hmt_enable(struct samsung_display_driver_data *vdd)
{
	LCD_INFO("[HMT] HMT %s\n", vdd->hmt_stat.hmt_on ? "ON" : "OFF");

	if (vdd->hmt_stat.hmt_on) {
		ss_send_cmd(vdd, TX_HMT_ENABLE);
	} else {
		ss_send_cmd(vdd, TX_HMT_DISABLE);
	}

	return 0;
}

int hmt_reverse_update(struct samsung_display_driver_data *vdd, int enable)
{
	LCD_INFO("[HMT] HMT %s\n", enable ? "REVERSE" : "FORWARD");

	if (enable)
		ss_send_cmd(vdd, TX_HMT_REVERSE);
	else
		ss_send_cmd(vdd, TX_HMT_FORWARD);

	return 0;
}

static void parse_dt_data(struct device_node *np, void *data, size_t size,
		char *cmd_string, char panel_rev,
		int (*fnc)(struct device_node *, void *, char *))
{
	char string[PARSE_STRING];
	int ret = 0;

	/* Generate string to parsing from DT */
	snprintf(string, PARSE_STRING, "%s%c", cmd_string, 'A' + panel_rev);

	ret = fnc(np, data, string);

	/* If there is no dtsi data for panel rev B ~ T,
	 * use valid previous panel rev dtsi data.
	 * TODO: Instead of copying all data from previous panel rev,
	 * copy only the pointer...
	 */
	if (ret && (panel_rev > 0))
		memcpy(data, (u8 *) data - size, size);
}

static void ss_panel_parse_dt_bright_tables(struct device_node *np,
		struct samsung_display_driver_data *vdd)
{
	struct samsung_display_dtsi_data *dtsi_data = &vdd->dtsi_data;
	int panel_rev;

	for (panel_rev = 0; panel_rev < SUPPORT_PANEL_REVISION; panel_rev++) {
		parse_dt_data(np, &dtsi_data->vint_map_table[panel_rev],
				sizeof(struct cmd_map),
				"samsung,vint_map_table_rev", panel_rev,
				ss_parse_panel_table); /* VINT TABLE */

		parse_dt_data(np, &dtsi_data->candela_map_table[NORMAL][panel_rev],
				sizeof(struct candela_map_table),
				"samsung,candela_map_table_rev", panel_rev,
				ss_parse_candella_mapping_table);

		parse_dt_data(np, &dtsi_data->candela_map_table[AOD][panel_rev],
				sizeof(struct candela_map_table),
				"samsung,aod_candela_map_table_rev", panel_rev,
				ss_parse_candella_mapping_table);

		parse_dt_data(np, &dtsi_data->candela_map_table[HBM][panel_rev],
				sizeof(struct candela_map_table),
				"samsung,hbm_candela_map_table_rev", panel_rev,
				ss_parse_hbm_candella_mapping_table);

		parse_dt_data(np, &dtsi_data->candela_map_table[PAC_NORMAL][panel_rev],
				sizeof(struct candela_map_table),
				"samsung,pac_candela_map_table_rev", panel_rev,
				ss_parse_pac_candella_mapping_table);

		parse_dt_data(np, &dtsi_data->candela_map_table[PAC_HBM][panel_rev],
				sizeof(struct candela_map_table),
				"samsung,pac_hbm_candela_map_table_rev", panel_rev,
				ss_parse_hbm_candella_mapping_table);

		parse_dt_data(np, &dtsi_data->candela_map_table[GAMMA_MODE2_NORMAL][panel_rev],
				sizeof(struct candela_map_table),
				"samsung,gamma_mode2_candela_map_table_rev", panel_rev,
				ss_parse_gamma_mode2_candella_mapping_table);

		/* ACL */
		parse_dt_data(np, &dtsi_data->acl_map_table[panel_rev],
				sizeof(struct cmd_map),
				"samsung,acl_map_table_rev", panel_rev,
				ss_parse_panel_table); /* ACL TABLE */

		parse_dt_data(np, &dtsi_data->elvss_map_table[panel_rev],
				sizeof(struct cmd_map),
				"samsung,elvss_map_table_rev", panel_rev,
				ss_parse_panel_table); /* ELVSS TABLE */

		parse_dt_data(np, &dtsi_data->aid_map_table[panel_rev],
				sizeof(struct cmd_map),
				"samsung,aid_map_table_rev", panel_rev,
				ss_parse_panel_table); /* AID TABLE */

		parse_dt_data(np, &dtsi_data->smart_acl_elvss_map_table[panel_rev],
				sizeof(struct cmd_map),
				"samsung,smart_acl_elvss_map_table_rev", panel_rev,
				ss_parse_panel_table); /* TABLE */

		parse_dt_data(np, &dtsi_data->hmt_reverse_aid_map_table[panel_rev],
				sizeof(struct cmd_map),
				"samsung,hmt_reverse_aid_map_table_rev", panel_rev,
				ss_parse_panel_table); /* TABLE */

		parse_dt_data(np, &dtsi_data->candela_map_table[HMT][panel_rev],
				sizeof(struct candela_map_table),
				"samsung,hmt_candela_map_table_rev", panel_rev,
				ss_parse_candella_mapping_table);
#if 0
		parse_dt_data(np, &dtsi_data->scaled_level_map_table[panel_rev],
				sizeof(struct candela_map_table),
				"samsung,scaled_level_map_table_rev", panel_rev,
				ss_parse_candella_mapping_table);
#endif
	}
}

static void ss_panel_pbaboot_config(struct device_node *np,
		struct samsung_display_driver_data *vdd)
{
	// TODO: set appropriate value to drm display members... not pinfo...
#if 0
	struct ss_panel_info *pinfo = NULL;
	struct samsung_display_driver_data *vdd = NULL;
	bool need_to_force_vidoe_mode = false;

	pinfo = &ctrl->panel_data.panel_info;
	vdd = check_valid_ctrl(ctrl);

	if (vdd->support_hall_ic) {
		/* check the lcd id for DISPLAY_1 and DISPLAY_2 */
		if (!ss_panel_attached(DISPLAY_1) && !ss_panel_attached(DISPLAY_2))
			need_to_force_vidoe_mode = true;
	} else {
		/* check the lcd id for DISPLAY_1 */
		if (!ss_panel_attached(DISPLAY_1))
			need_to_force_vidoe_mode = true;
	}

	/* Support PBA boot without lcd */
	if (need_to_force_vidoe_mode &&
			!IS_ERR_OR_NULL(pinfo) &&
			!IS_ERR_OR_NULL(vdd) &&
			(pinfo->mipi.mode == DSI_CMD_MODE)) {
		LCD_ERR("force VIDEO_MODE : %d\n", vdd->ndx);
		pinfo->type = MIPI_VIDEO_PANEL;
		pinfo->mipi.mode = DSI_VIDEO_MODE;
		pinfo->mipi.traffic_mode = DSI_BURST_MODE;
		pinfo->mipi.bllp_power_stop = true;
		pinfo->mipi.te_sel = 0;
		pinfo->mipi.vsync_enable = 0;
		pinfo->mipi.hw_vsync_mode = 0;
		pinfo->mipi.force_clk_lane_hs = true;
		pinfo->mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;

#if 0
		pinfo->cont_splash_enabled = false;
#else
		ss_set_seamless_mode(vdd);
#endif
		pinfo->mipi.lp11_init = false;

		vdd->mdnie.support_mdnie = false;
		vdd->mdnie.lcd_on_notifiy = false;
		vdd->mdnie.support_trans_dimming = false;
		vdd->mdnie.disable_trans_dimming = false;

		if (!IS_ERR_OR_NULL(vdd->panel_func.parsing_otherline_pdata) && ss_panel_attached(DISPLAY_1)) {
			vdd->panel_func.parsing_otherline_pdata = NULL;
			destroy_workqueue(vdd->other_line_panel_support_workq);
		}

		pinfo->esd_check_enabled = false;
		ctrl->on_cmds.link_state = DSI_LP_MODE;
		ctrl->off_cmds.link_state = DSI_LP_MODE;

#if 0
		/* To avoid underrun panic*/
		mdd->logd.xlog_enable = 0;
#else
#endif
		vdd->dtsi_data.samsung_osc_te_fitting = false;
	}
#endif
}

static void ss_dynamic_mipi_clk_work(struct work_struct *work)
{
	struct samsung_display_driver_data *vdd = NULL;
	struct dyn_mipi_clk *dyn;

	dyn = container_of(work, struct dyn_mipi_clk, change_clk_work);
	vdd = container_of(dyn, struct samsung_display_driver_data, dyn_mipi_clk);

	ss_change_dyn_mipi_clk_timing(vdd);
}

static void ss_panel_parse_dt(struct samsung_display_driver_data *vdd)
{
	int rc, i;
	u32 tmp[2];
	int len;
	char backlight_tft_gpio[] = "samsung,panel-backlight-tft-gpio1";
	const char *data;
	const __be32 *data_32;
	struct device_node *np;

	np = ss_get_panel_of(vdd);

	if (!np)
		return;

	/* Set LP11 init flag */
	vdd->dtsi_data.samsung_lp11_init = of_property_read_bool(np, "samsung,dsi-lp11-init");
	LCD_ERR("LP11 init %s\n",vdd->dtsi_data.samsung_lp11_init ? "enabled" : "disabled");

	/*IMPORTANT NOTE :- below two flag should be used only if "qcom,mdss-dsi-lp11-init" is disable*/
	vdd->dtsi_data.samsung_reset_before_dsi_off = of_property_read_bool(np, "samsung,mdss-reset-before-dsi-off");
	vdd->dtsi_data.samsung_reset_after_dsi_on = of_property_read_bool(np, "samsung,mdss-reset-after-dsi-on");

	rc = of_property_read_u32(np, "samsung,mdss-power-on-reset-delay-us", tmp);
	vdd->dtsi_data.samsung_power_on_reset_delay = (!rc ? tmp[0] : 0);

	rc = of_property_read_u32(np, "samsung,mdss-dsi-off-reset-delay-us", tmp);
	vdd->dtsi_data.samsung_dsi_off_reset_delay = (!rc ? tmp[0] : 0);

	/* Set esc clk 128M */
	vdd->dtsi_data.samsung_esc_clk_128M = of_property_read_bool(np, "samsung,esc-clk-128M");
	LCD_ERR("ESC CLK 128M %s\n",
		vdd->dtsi_data.samsung_esc_clk_128M ? "enabled" : "disabled");

	vdd->dtsi_data.panel_lpm_enable = of_property_read_bool(np, "samsung,panel-lpm-enable");
	LCD_ERR("alpm enable %s\n",
		vdd->dtsi_data.panel_lpm_enable ? "enabled" : "disabled");

	/* Set HALL IC */
	vdd->support_hall_ic  = of_property_read_bool(np, "samsung,mdss_dsi_hall_ic_enable");
	LCD_ERR("hall_ic %s\n", vdd->support_hall_ic ? "enabled" : "disabled");

	rc = of_property_read_u32(np, "samsung,mdss_dsi_lcd_flip_delay_ms", tmp);
		vdd->lcd_flip_delay_ms = (!rc ? tmp[0] : 0);
		LCD_ERR("lcd_flip_delay_ms %d\n", vdd->lcd_flip_delay_ms);

	/*Set OSC TE fitting flag */
	vdd->dtsi_data.samsung_osc_te_fitting =
		of_property_read_bool(np, "samsung,osc-te-fitting-enable");

	if (vdd->dtsi_data.samsung_osc_te_fitting) {
		rc = of_property_read_u32_array(np, "samsung,osc-te-fitting-cmd-index", tmp, 2);
		if (!rc) {
			vdd->dtsi_data.samsung_osc_te_fitting_cmd_index[0] =
				tmp[0];
			vdd->dtsi_data.samsung_osc_te_fitting_cmd_index[1] =
				tmp[1];
		}

		rc = of_property_read_u32(np, "samsung,osc-te-fitting-sampling-rate", tmp);

		vdd->te_fitting_info.sampling_rate = !rc ? tmp[0] : 2;

	}

	LCD_INFO("OSC TE fitting %s\n",
		vdd->dtsi_data.samsung_osc_te_fitting ? "enabled" : "disabled");

	/* Set HMT flag */
	vdd->dtsi_data.hmt_enabled = of_property_read_bool(np, "samsung,hmt_enabled");
	if (vdd->dtsi_data.hmt_enabled)
		vdd->dtsi_data.hmt_enabled = true;

	LCD_INFO("hmt %s\n",
		vdd->dtsi_data.hmt_enabled ? "enabled" : "disabled");

	/* TCON Clk On Support */
	vdd->dtsi_data.samsung_tcon_clk_on_support =
		of_property_read_bool(np, "samsung,tcon-clk-on-support");
	LCD_INFO("tcon clk on support: %s\n",
			vdd->dtsi_data.samsung_tcon_clk_on_support ?
			"enabled" : "disabled");

	/* Set TFT flag */
	vdd->mdnie.tuning_enable_tft = of_property_read_bool(np,
				"samsung,mdnie-tuning-enable-tft");
	vdd->dtsi_data.tft_common_support  = of_property_read_bool(np,
		"samsung,tft-common-support");

	LCD_INFO("tft_common_support %s\n",
	vdd->dtsi_data.tft_common_support ? "enabled" : "disabled");

	vdd->dtsi_data.tft_module_name = of_get_property(np,
		"samsung,tft-module-name", NULL);  /* for tft tablet */

	vdd->dtsi_data.panel_vendor = of_get_property(np,
		"samsung,panel-vendor", NULL);

	vdd->dtsi_data.disp_model = of_get_property(np,
		"samsung,disp-model", NULL);

	vdd->dtsi_data.backlight_gpio_config = of_property_read_bool(np,
		"samsung,backlight-gpio-config");

	LCD_INFO("backlight_gpio_config %s\n",
	vdd->dtsi_data.backlight_gpio_config ? "enabled" : "disabled");

	/* Factory Panel Swap*/
	vdd->dtsi_data.samsung_support_factory_panel_swap = of_property_read_bool(np,
		"samsung,support_factory_panel_swap");

	/* Set tft backlight gpio */
	for (i = 0; i < MAX_BACKLIGHT_TFT_GPIO; i++) {
		backlight_tft_gpio[strlen(backlight_tft_gpio) - 1] = '1' + i;
		vdd->dtsi_data.backlight_tft_gpio[i] =
				 of_get_named_gpio(np,
						backlight_tft_gpio, 0);
		if (!gpio_is_valid(vdd->dtsi_data.backlight_tft_gpio[i]))
			LCD_ERR("%d, backlight_tft_gpio gpio%d not specified\n",
							__LINE__, i+1);
		else
			LCD_ERR("tft gpio num : %d\n", vdd->dtsi_data.backlight_tft_gpio[i]);
	}

	/* Set Mdnie lite HBM_CE_TEXT_MDNIE mode used */
	vdd->dtsi_data.hbm_ce_text_mode_support = of_property_read_bool(np, "samsung,hbm_ce_text_mode_support");

	/* Set Backlight IC discharge time */
	rc = of_property_read_u32(np, "samsung,blic-discharging-delay-us", tmp);
	vdd->dtsi_data.blic_discharging_delay_tft = (!rc ? tmp[0] : 6);

	/* Set cabc delay time */
	rc = of_property_read_u32(np, "samsung,cabc-delay-us", tmp);
	vdd->dtsi_data.cabc_delay = (!rc ? tmp[0] : 6);

	/* IRC */
	vdd->br.support_irc = of_property_read_bool(np, "samsung,support_irc");

	/* Gram Checksum Test */
	vdd->gct.is_support = of_property_read_bool(np, "samsung,support_gct");
	LCD_DEBUG("vdd->gct.is_support = %d\n", vdd->gct.is_support);

	/* POC Driver */
	vdd->poc_driver.is_support = of_property_read_bool(np, "samsung,support_poc_driver");
	LCD_INFO("[POC] is_support = %d\n", vdd->poc_driver.is_support);

	if (vdd->poc_driver.is_support) {
		rc = of_property_read_u32(np, "samsung,poc_image_size", tmp);
		vdd->poc_driver.image_size = (!rc ? tmp[0] : 0);

		LCD_INFO("[POC] image_size (%d)\n", vdd->poc_driver.image_size);

		/* ERASE */
		rc = of_property_read_u32(np, "samsung,poc_erase_delay_us", tmp);
		vdd->poc_driver.erase_delay_us = (!rc ? tmp[0] : 0);

		rc = of_property_read_u32_array(np, "samsung,poc_erase_sector_addr_idx",
				vdd->poc_driver.erase_sector_addr_idx, 3);
		if (rc) {
			vdd->poc_driver.erase_sector_addr_idx[0] = -1;
			LCD_INFO("fail to get poc_erase_sector_addr_idx\n");
		}

		LCD_INFO("[POC][ERASE] delay_us(%d) addr idx (%d %d %d)\n",
			vdd->poc_driver.erase_delay_us,
			vdd->poc_driver.erase_sector_addr_idx[0],
			vdd->poc_driver.erase_sector_addr_idx[1],
			vdd->poc_driver.erase_sector_addr_idx[2]);

		/* WRITE */
		rc = of_property_read_u32(np, "samsung,poc_write_delay_us", tmp);
		vdd->poc_driver.write_delay_us = (!rc ? tmp[0] : 0);
		rc = of_property_read_u32(np, "samsung,poc_write_data_size", tmp);
		vdd->poc_driver.write_data_size = (!rc ? tmp[0] : 0);
		rc = of_property_read_u32(np, "samsung,poc_write_loop_cnt", tmp);
		vdd->poc_driver.write_loop_cnt = (!rc ? tmp[0] : 0);

		rc = of_property_read_u32_array(np, "samsung,poc_write_addr_idx",
				vdd->poc_driver.write_addr_idx, 3);
		if (rc) {
			vdd->poc_driver.write_addr_idx[0] = -1;
			LCD_INFO("fail to get poc_write_addr_idx\n");
		}

		LCD_INFO("[POC][WRITE] delay_us(%d) data_size(%d) loo_cnt(%d) addr idx (%d %d %d)\n",
			vdd->poc_driver.write_delay_us,
			vdd->poc_driver.write_data_size,
			vdd->poc_driver.write_loop_cnt,
			vdd->poc_driver.write_addr_idx[0],
			vdd->poc_driver.write_addr_idx[1],
			vdd->poc_driver.write_addr_idx[2]);

		/* READ */
		rc = of_property_read_u32(np, "samsung,poc_read_delay_us", tmp);
		vdd->poc_driver.read_delay_us = (!rc ? tmp[0] : 0);

		rc = of_property_read_u32_array(np, "samsung,poc_read_addr_idx",
				vdd->poc_driver.read_addr_idx, 3);
		if (rc) {
			vdd->poc_driver.read_addr_idx[0] = -1;
			LCD_INFO("fail to get poc_read_addr_idx\n");
		} else
			LCD_INFO("read addr idx (%d %d %d)\n",
				vdd->poc_driver.read_addr_idx[0],
				vdd->poc_driver.read_addr_idx[1],
				vdd->poc_driver.read_addr_idx[2]);

		LCD_INFO("[POC][READ] delay_us(%d) addr idx (%d %d %d)\n",
			vdd->poc_driver.read_delay_us,
			vdd->poc_driver.read_addr_idx[0],
			vdd->poc_driver.read_addr_idx[1],
			vdd->poc_driver.read_addr_idx[2]);
	}

	/* PAC */
	vdd->br.pac = of_property_read_bool(np, "samsung,support_pac");
	LCD_INFO("vdd->br.pac = %d\n", vdd->br.pac);

	/* Gamma Mode 2 */
	vdd->br.gamma_mode2_support = of_property_read_bool(np, "samsung,support_gamma_mode2");
	LCD_INFO("vdd->br.gamma_mode2_support = %d\n", vdd->br.gamma_mode2_support);

	/* Global Para */
	vdd->gpara = of_property_read_bool(np, "samsung,support_gpara");
	LCD_INFO("vdd->support_gpara = %d\n", vdd->gpara);

	/* Self Display */
	vdd->self_disp.is_support = of_property_read_bool(np, "samsung,support_self_display");
	LCD_INFO("vdd->self_disp.is_support = %d\n", vdd->self_disp.is_support);

	if (vdd->self_disp.is_support) {
		/* Self Mask Check CRC data */
		data = of_get_property(np, "samsung,mask_crc_pass_data", &len);
		if (!data)
			LCD_ERR("fail to get samsung,mask_crc_pass_data .. \n");
		else {
			vdd->self_disp.mask_crc_pass_data = kzalloc(len, GFP_KERNEL);
			vdd->self_disp.mask_crc_read_data = kzalloc(len, GFP_KERNEL);
			vdd->self_disp.mask_crc_size = len;
			if (!vdd->self_disp.mask_crc_pass_data || !vdd->self_disp.mask_crc_read_data)
				LCD_ERR("fail to alloc for mask_crc_data \n");
			else {
				for (i = 0; i < len; i++) {
					vdd->self_disp.mask_crc_pass_data[i] = data[i];
					LCD_ERR("crc_data[%d] = %02x\n", i, vdd->self_disp.mask_crc_pass_data[i]);
				}
			}
		}
	}

	/* DDI SPI */
	vdd->samsung_support_ddi_spi = of_property_read_bool(np, "samsung,support_ddi_spi");
	if (vdd->samsung_support_ddi_spi) {
		rc = of_property_read_u32(np, "samsung,ddi_spi_cs_high_gpio_for_gpara", tmp);
		vdd->ddi_spi_cs_high_gpio_for_gpara = (!rc ? tmp[0] : -1);

		LCD_INFO("vdd->ddi_spi_cs_high_gpio_for_gpara = %d\n", vdd->ddi_spi_cs_high_gpio_for_gpara);
	}

	/* Set elvss_interpolation_temperature */
	data_32 = of_get_property(np, "samsung,elvss_interpolation_temperature", NULL);

	if (data_32)
		vdd->br.elvss_interpolation_temperature = (int)(be32_to_cpup(data_32));
	else
		vdd->br.elvss_interpolation_temperature = ELVSS_INTERPOLATION_TEMPERATURE;

	/* Set lux value for mdnie HBM */
	data_32 = of_get_property(np, "samsung,enter_hbm_ce_lux", NULL);
	if (data_32)
		vdd->mdnie.enter_hbm_ce_lux = (int)(be32_to_cpup(data_32));
	else
		vdd->mdnie.enter_hbm_ce_lux = ENTER_HBM_CE_LUX;

	/* SAMSUNG_FINGERPRINT */
	vdd->support_optical_fingerprint = of_property_read_bool(np, "samsung,support-optical-fingerprint");
	LCD_ERR("support_optical_fingerprint %s\n",
		vdd->support_optical_fingerprint ? "enabled" : "disabled");

	/* Power Control for LPM */
	vdd->panel_lpm.lpm_pwr.support_lpm_pwr_ctrl = of_property_read_bool(np, "samsung,lpm-power-control");
	LCD_INFO("lpm_power_control %s\n", vdd->panel_lpm.lpm_pwr.support_lpm_pwr_ctrl ? "enabled" : "disabled");

	if (vdd->panel_lpm.lpm_pwr.support_lpm_pwr_ctrl) {
		rc = of_property_read_string(np, "samsung,lpm-power-control-supply-name", &data);
		if (rc)
			LCD_ERR("error reading lpm-power name. rc=%d\n", rc);
		else
			snprintf(vdd->panel_lpm.lpm_pwr.lpm_pwr_ctrl_supply_name,
				ARRAY_SIZE((vdd->panel_lpm.lpm_pwr.lpm_pwr_ctrl_supply_name)), "%s", data);

		data_32 = of_get_property(np, "samsung,lpm-power-control-supply-min-voltage", NULL);
		if (data_32)
			vdd->panel_lpm.lpm_pwr.lpm_pwr_ctrl_supply_min_v = (int)(be32_to_cpup(data_32));
		else
			LCD_ERR("error reading lpm-power min_voltage\n");

		data_32 = of_get_property(np, "samsung,lpm-power-control-supply-max-voltage", NULL);
		if (data_32)
			vdd->panel_lpm.lpm_pwr.lpm_pwr_ctrl_supply_max_v = (int)(be32_to_cpup(data_32));
		else
			LCD_ERR("error reading lpm-power max_voltage\n");

		LCD_INFO("lpm_power_control Supply Name=%s, Min=%d, Max=%d\n",
			vdd->panel_lpm.lpm_pwr.lpm_pwr_ctrl_supply_name, vdd->panel_lpm.lpm_pwr.lpm_pwr_ctrl_supply_min_v,
			vdd->panel_lpm.lpm_pwr.lpm_pwr_ctrl_supply_max_v);

		rc = of_property_read_string(np, "samsung,lpm-power-control-elvss-name", &data);
		if (rc)
			LCD_ERR("error reading lpm-power-elvss name. rc=%d\n", rc);
		else
			snprintf(vdd->panel_lpm.lpm_pwr.lpm_pwr_ctrl_elvss_name,
				ARRAY_SIZE((vdd->panel_lpm.lpm_pwr.lpm_pwr_ctrl_elvss_name)), "%s", data);

		data_32 = of_get_property(np, "samsung,lpm-power-control-elvss-lpm-voltage", NULL);
		if (data_32)
			vdd->panel_lpm.lpm_pwr.lpm_pwr_ctrl_elvss_lpm_v = (int)(be32_to_cpup(data_32));
		else
			LCD_ERR("error reading lpm-power-elvss lpm voltage\n");

		data_32 = of_get_property(np, "samsung,lpm-power-control-elvss-normal-voltage", NULL);
		if (data_32)
			vdd->panel_lpm.lpm_pwr.lpm_pwr_ctrl_elvss_normal_v = (int)(be32_to_cpup(data_32));
		else
			LCD_ERR("error reading lpm-power-elvss normal voltage\n");

		LCD_INFO("lpm_power_control ELVSS Name=%s, lpm=%d, normal=%d\n",
			vdd->panel_lpm.lpm_pwr.lpm_pwr_ctrl_elvss_name, vdd->panel_lpm.lpm_pwr.lpm_pwr_ctrl_elvss_lpm_v,
			vdd->panel_lpm.lpm_pwr.lpm_pwr_ctrl_elvss_normal_v);
	}

	/* To reduce DISPLAY ON time */
	vdd->dtsi_data.samsung_reduce_display_on_time = of_property_read_bool(np,"samsung,reduce_display_on_time");
	vdd->dtsi_data.samsung_dsi_force_clock_lane_hs = of_property_read_bool(np,"samsung,dsi_force_clock_lane_hs");
	rc = of_property_read_u32(np, "samsung,wait_after_reset_delay", tmp);
	vdd->dtsi_data.samsung_wait_after_reset_delay = (!rc ? tmp[0] : 0);
	rc = of_property_read_u32(np, "samsung,wait_after_sleep_out_delay", tmp);
	vdd->dtsi_data.samsung_wait_after_sleep_out_delay = (!rc ? tmp[0] : 0);

	if (vdd->dtsi_data.samsung_reduce_display_on_time) {
		/*
			Please Check interrupt gpio is general purpose gpio
			1. pmic gpio or gpio-expender is not permitted.
			2. gpio should have unique interrupt number.
		*/

		if(gpio_is_valid(of_get_named_gpio(np, "samsung,home_key_irq_gpio", 0))) {
				vdd->dtsi_data.samsung_home_key_irq_num =
					gpio_to_irq(of_get_named_gpio(np, "samsung,home_key_irq_gpio", 0));
				LCD_ERR("%s gpio : %d (irq : %d)\n",
					np->name, of_get_named_gpio(np, "samsung,home_key_irq_gpio", 0),
					vdd->dtsi_data.samsung_home_key_irq_num);
		}

		if(gpio_is_valid(of_get_named_gpio(np, "samsung,fingerprint_irq_gpio", 0))) {
			vdd->dtsi_data.samsung_finger_print_irq_num =
				gpio_to_irq(of_get_named_gpio(np, "samsung,fingerprint_irq_gpio", 0));
			LCD_ERR("%s gpio : %d (irq : %d)\n",
				np->name, of_get_named_gpio(np, "samsung,fingerprint_irq_gpio", 0),
				vdd->dtsi_data.samsung_finger_print_irq_num);
		}
	}

	/* Dynamic MIPI Clock */
	vdd->dyn_mipi_clk.is_support = of_property_read_bool(np, "samsung,support_dynamic_mipi_clk");
	LCD_INFO("vdd->dyn_mipi_clk.is_support = %d\n", vdd->dyn_mipi_clk.is_support);

	if (vdd->dyn_mipi_clk.is_support) {
		ss_parse_dyn_mipi_clk_sel_table(np, &vdd->dyn_mipi_clk.clk_sel_table,
					"samsung,dynamic_mipi_clk_sel_table");
		ss_parse_dyn_mipi_clk_timing_table(np, &vdd->dyn_mipi_clk.clk_timing_table,
					"samsung,dynamic_mipi_clk_timing_table");

		vdd->dyn_mipi_clk.change_clk_wq = create_singlethread_workqueue("dyna_mipi_clk_wq");
		if (!vdd->dyn_mipi_clk.change_clk_wq) {
			LCD_ERR("failed to create read copr workqueue..\n");
			return;
		}
		INIT_WORK(&vdd->dyn_mipi_clk.change_clk_work, ss_dynamic_mipi_clk_work);

		vdd->dyn_mipi_clk.notifier.priority = 0;
		vdd->dyn_mipi_clk.notifier.notifier_call = ss_rf_info_notify_callback;
		register_dev_ril_bridge_event_notifier(&vdd->dyn_mipi_clk.notifier);

		rc = of_property_read_u32(np, "samsung,ffc_cmds_line_position", tmp);
		vdd->ffc_cmds_line_position = (!rc ? tmp[0] : 1);
		LCD_INFO("ffc_cmds_line_position (%d) \n", vdd->ffc_cmds_line_position);
	}

	ss_panel_parse_dt_bright_tables(np, vdd);
#if KERNEL_VER > 409
	ss_dsi_panel_parse_cmd_sets(vdd->dtsi_data.cmd_sets, GET_DSI_PANEL(vdd));
#else
	ss_dsi_panel_parse_cmd_sets(vdd->dtsi_data.cmd_sets, np);
#endif

	if (vdd->support_hall_ic) {
		vdd->hall_ic_notifier_display.priority = 0; /* Tsp is 1, Touch key is 2 */
		vdd->hall_ic_notifier_display.notifier_call = samsung_display_hall_ic_status;
#if defined(CONFIG_FOLDER_HALL)
		hall_ic_register_notify(&vdd->hall_ic_notifier_display);
#endif
	}

	// LCD SELECT
	vdd->select_panel_gpio = of_get_named_gpio(np,
			"samsung,mdss_dsi_lcd_sel_gpio", 0);
	if (gpio_is_valid(vdd->select_panel_gpio))
		gpio_request(vdd->select_panel_gpio, "lcd_sel_gpio");

	vdd->select_panel_use_expander_gpio = of_property_read_bool(np, "samsung,mdss_dsi_lcd_sel_use_expander_gpio");

	/* Panel LPM */
	rc = of_property_read_u32(np, "samsung,lpm-init-delay-ms", tmp);
	vdd->dtsi_data.samsung_lpm_init_delay = (!rc ? tmp[0] : 0);

	rc = of_property_read_u32(np, "samsung,delayed-display-on", tmp);
	vdd->dtsi_data.samsung_delayed_display_on = (!rc ? tmp[0] : 0);

	ss_panel_parse_dt_ub_con(np, vdd);
	ss_panel_parse_dt_ub_fd(np, vdd);
	ss_panel_parse_dt_esd(np, vdd);
	ss_panel_pbaboot_config(np, vdd);

	/* AOR & IRC INTERPOLATION (FLASH or NO FLASH SUPPORT)*/
	data_32 = of_get_property(np, "samsung,hbm_brightness", &len);
			vdd->dtsi_data.hbm_brightness_step = (data_32 ? len/sizeof(len) : 0);
	data_32 = of_get_property(np, "samsung,normal_brightness", &len);
			vdd->dtsi_data.normal_brightness_step = (data_32 ? len/sizeof(len) : 0);
	data_32 = of_get_property(np, "samsung,hmd_brightness", &len);
			vdd->dtsi_data.hmd_brightness_step = (data_32 ? len/sizeof(len) : 0);
	LCD_INFO("gamma_step hbm: %d normal: %d hmt: %d\n",
		vdd->dtsi_data.hbm_brightness_step,
		vdd->dtsi_data.normal_brightness_step,
		vdd->dtsi_data.hmd_brightness_step);

	rc = of_property_read_u32(np, "samsung,gamma_size", tmp);
			vdd->dtsi_data.gamma_size = (!rc ? tmp[0] : 34);
	rc = of_property_read_u32(np, "samsung,aor_size", tmp);
			vdd->dtsi_data.aor_size = (!rc ? tmp[0] : 2);
	rc = of_property_read_u32(np, "samsung,vint_size", tmp);
			vdd->dtsi_data.vint_size = (!rc ? tmp[0] : 1);
	rc = of_property_read_u32(np, "samsung,elvss_size", tmp);
			vdd->dtsi_data.elvss_size = (!rc ? tmp[0] : 3);
	rc = of_property_read_u32(np, "samsung,irc_size", tmp);
			vdd->dtsi_data.irc_size = (!rc ? tmp[0] : 17);
	LCD_INFO("flash_gamma_size gamma:%d aor:%d vint:%d elvss:%d irc:%d\n",
		vdd->dtsi_data.gamma_size,
		vdd->dtsi_data.aor_size,
		vdd->dtsi_data.vint_size,
		vdd->dtsi_data.elvss_size,
		vdd->dtsi_data.irc_size);

	rc = of_property_read_u32(np, "samsung,flash_table_hbm_aor_offset", tmp);
	vdd->dtsi_data.flash_table_hbm_aor_offset = (!rc ? tmp[0] : 0x09D4);
	rc = of_property_read_u32(np, "samsung,flash_table_hbm_vint_offset", tmp);
	vdd->dtsi_data.flash_table_hbm_vint_offset = (!rc ? tmp[0] : 0x0A80);
	rc = of_property_read_u32(np, "samsung,flash_table_hbm_elvss_offset", tmp);
	vdd->dtsi_data.flash_table_hbm_elvss_offset = (!rc ? tmp[0] : 0x0AD6);
	rc = of_property_read_u32(np, "samsung,flash_table_hbm_irc_offset", tmp);
	vdd->dtsi_data.flash_table_hbm_irc_offset = (!rc ? tmp[0] : 0x0AD6);
	LCD_INFO("flash_table_hbm_addr aor: 0x%x vint:0x%x elvss 0x%x hbm:0x%x\n",
		vdd->dtsi_data.flash_table_hbm_aor_offset,
		vdd->dtsi_data.flash_table_hbm_vint_offset,
		vdd->dtsi_data.flash_table_hbm_elvss_offset,
		vdd->dtsi_data.flash_table_hbm_irc_offset);

	rc = of_property_read_u32(np, "samsung,flash_table_normal_gamma_offset", tmp);
	vdd->dtsi_data.flash_table_normal_gamma_offset = (!rc ? tmp[0] : 0x0000);
	rc = of_property_read_u32(np, "samsung,flash_table_normal_aor_offset", tmp);
	vdd->dtsi_data.flash_table_normal_aor_offset = (!rc ? tmp[0] : 0x09EC);
	rc = of_property_read_u32(np, "samsung,flash_table_normal_vint_offset", tmp);
	vdd->dtsi_data.flash_table_normal_vint_offset = (!rc ? tmp[0] : 0x0A8C);
	rc = of_property_read_u32(np, "samsung,flash_table_normal_elvss_offset", tmp);
	vdd->dtsi_data.flash_table_normal_elvss_offset = (!rc ? tmp[0] : 0x0AFA);
	rc = of_property_read_u32(np, "samsung,flash_table_normal_irc_offset", tmp);
	vdd->dtsi_data.flash_table_normal_irc_offset = (!rc ? tmp[0] : 0x0CA4);
	LCD_INFO("flash_table__normal_addr gamma:0x%x aor: 0x%x vint:0x%x elvss 0x%x hbm:0x%x\n",
		vdd->dtsi_data.flash_table_normal_gamma_offset,
		vdd->dtsi_data.flash_table_normal_aor_offset,
		vdd->dtsi_data.flash_table_normal_vint_offset,
		vdd->dtsi_data.flash_table_normal_elvss_offset,
		vdd->dtsi_data.flash_table_normal_irc_offset);

	rc = of_property_read_u32(np, "samsung,flash_table_hmd_gamma_offset", tmp);
	vdd->dtsi_data.flash_table_hmd_gamma_offset = (!rc ? tmp[0] : 0x118E);
	rc = of_property_read_u32(np, "samsung,flash_table_hmd_aor_offset", tmp);
	vdd->dtsi_data.flash_table_hmd_aor_offset = (!rc ? tmp[0] : 0x1678);
	LCD_INFO("flash_table_hmt_addr gamma:0x%x aor: 0x%x\n",
		vdd->dtsi_data.flash_table_hmd_gamma_offset,
		vdd->dtsi_data.flash_table_hmd_aor_offset);

	/* ONLY FLASH GAMMA */
	vdd->dtsi_data.flash_gamma_support = of_property_read_bool(np, "samsung,support_flash_gamma");

	if (vdd->dtsi_data.flash_gamma_support) {
		vdd->flash_br_workqueue = create_singlethread_workqueue("flash_br_workqueue");
		INIT_DELAYED_WORK(&vdd->flash_br_work, flash_br_work_func);

		data_32 = of_get_property(np, "samsung,flash_gamma_data_read_addr", &len);
		/* 3byte address cover 15Mbyte */
		vdd->dtsi_data.flash_gamma_data_read_addr_len = (data_32 ? len/sizeof(len) : 3);
		vdd->dtsi_data.flash_gamma_data_read_addr =
				kzalloc(vdd->dtsi_data.flash_gamma_data_read_addr_len * sizeof(int *), GFP_KERNEL);
		rc = of_property_read_u32_array(np, "samsung,flash_gamma_data_read_addr",
				vdd->dtsi_data.flash_gamma_data_read_addr, vdd->dtsi_data.flash_gamma_data_read_addr_len);
		if (rc) {
			vdd->dtsi_data.flash_gamma_data_read_addr[0] = 8;
			vdd->dtsi_data.flash_gamma_data_read_addr[1] = 9;
			vdd->dtsi_data.flash_gamma_data_read_addr[2] = 10;
			LCD_INFO("fail to get flash_gamma_data_read_addr\n");
		}

		rc = of_property_read_u32(np, "samsung,flash_gamma_write_check_addr", tmp);
		vdd->dtsi_data.flash_gamma_write_check_address = (!rc ? tmp[0] : 0x0A16C4);
		LCD_INFO("write_check_addr: 0x%x \n", vdd->dtsi_data.flash_gamma_write_check_address);

		data_32 = of_get_property(np, "samsung,flash_gamma_start_bank", &len);
		vdd->dtsi_data.flash_gamma_bank_start_len = (data_32 ? len/sizeof(len) : 3);
		data_32 = of_get_property(np, "samsung,flash_gamma_end_bank", &len);
		vdd->dtsi_data.flash_gamma_bank_end_len = (data_32 ? len/sizeof(len) : 3);

		vdd->dtsi_data.flash_gamma_bank_start = kzalloc(vdd->dtsi_data.flash_gamma_bank_start_len * sizeof(int *), GFP_KERNEL);
		vdd->dtsi_data.flash_gamma_bank_end = kzalloc(vdd->dtsi_data.flash_gamma_bank_end_len * sizeof(int *), GFP_KERNEL);
		rc = of_property_read_u32_array(np, "samsung,flash_gamma_start_bank",
					vdd->dtsi_data.flash_gamma_bank_start ,
					vdd->dtsi_data.flash_gamma_bank_start_len);
		if (rc)
			LCD_INFO("fail to get samsung,flash_gamma_start_bank\n");

		rc = of_property_read_u32_array(np, "samsung,flash_gamma_end_bank",
					vdd->dtsi_data.flash_gamma_bank_end,
					vdd->dtsi_data.flash_gamma_bank_end_len);
		if (rc)
			LCD_INFO("fail to get samsung,flash_gamma_end_bank\n");

		LCD_INFO("start_bank[0] : 0x%x end_bank[0]: 0x%x\n",
			vdd->dtsi_data.flash_gamma_bank_start[0],
			vdd->dtsi_data.flash_gamma_bank_end[0]);

		rc = of_property_read_u32(np, "samsung,flash_gamma_check_sum_start_offset", tmp);
		vdd->dtsi_data.flash_gamma_check_sum_start_offset = (!rc ? tmp[0] : 0x16C2);
		rc = of_property_read_u32(np, "samsung,flash_gamma_check_sum_end_offset", tmp);
		vdd->dtsi_data.flash_gamma_check_sum_end_offset = (!rc ? tmp[0] : 0x16C3);
		LCD_INFO("check_sum_start_offset : 0x%x check_sum_end_offset: 0x%x\n",
			vdd->dtsi_data.flash_gamma_check_sum_start_offset,
			vdd->dtsi_data.flash_gamma_check_sum_end_offset);

		rc = of_property_read_u32(np, "samsung,flash_gamma_0xc8_start_offset", tmp);
		vdd->dtsi_data.flash_gamma_0xc8_start_offset = (!rc ? tmp[0] : 0x2000);
		rc = of_property_read_u32(np, "samsung,flash_gamma_0xc8_end_offset", tmp);
		vdd->dtsi_data.flash_gamma_0xc8_end_offset = (!rc ? tmp[0] : 0x2021);
		rc = of_property_read_u32(np, "samsung,flash_gamma_0xc8_size", tmp);
		vdd->dtsi_data.flash_gamma_0xc8_size = (!rc ? tmp[0] : 34);
		rc = of_property_read_u32(np, "samsung,flash_gamma_0xc8_check_sum_start_offset", tmp);
		vdd->dtsi_data.flash_gamma_0xc8_check_sum_start_offset = (!rc ? tmp[0] : 0x2022);
		rc = of_property_read_u32(np, "samsung,flash_gamma_0xc8_check_sum_end_offset", tmp);
		vdd->dtsi_data.flash_gamma_0xc8_check_sum_end_offset = (!rc ? tmp[0] : 0x2023);
		LCD_INFO("flash_gamma 0xC8 start_addr:0x%x end_addr: 0x%x size: 0x%x check_sum_start : 0x%x check_sum_end: 0x%x\n",
			vdd->dtsi_data.flash_gamma_0xc8_start_offset,
			vdd->dtsi_data.flash_gamma_0xc8_end_offset,
			vdd->dtsi_data.flash_gamma_0xc8_size,
			vdd->dtsi_data.flash_gamma_0xc8_check_sum_start_offset,
			vdd->dtsi_data.flash_gamma_0xc8_check_sum_end_offset);

		rc = of_property_read_u32(np, "samsung,flash_MCD1_R_addr", tmp);
		vdd->dtsi_data.flash_MCD1_R_address = (!rc ? tmp[0] : 0xB8000);
		rc = of_property_read_u32(np, "samsung,flash_MCD2_R_addr", tmp);
		vdd->dtsi_data.flash_MCD2_R_address = (!rc ? tmp[0] : 0xB8001);
		rc = of_property_read_u32(np, "samsung,flash_MCD1_L_addr", tmp);
		vdd->dtsi_data.flash_MCD1_L_address = (!rc ? tmp[0] : 0xB8004);
		rc = of_property_read_u32(np, "samsung,flash_MCD2_L_addr", tmp);
		vdd->dtsi_data.flash_MCD2_L_address = (!rc ? tmp[0] : 0xB8005);
		LCD_INFO("flash_gamma MCD1_R:0x%x MCD2_R:0x%x MCD1_L:0x%x MCD2_L:0x%x\n",
			vdd->dtsi_data.flash_MCD1_R_address,
			vdd->dtsi_data.flash_MCD2_R_address,
			vdd->dtsi_data.flash_MCD1_L_address,
			vdd->dtsi_data.flash_MCD2_L_address);
	}

	/* ccd success,fail value */
	rc = of_property_read_u32(np, "samsung,ccd_pass_val", tmp);
	vdd->ccd_pass_val = (!rc ? tmp[0] : 0);
	rc = of_property_read_u32(np, "samsung,ccd_fail_val", tmp);
	vdd->ccd_fail_val = (!rc ? tmp[0] : 0);
	LCD_INFO("CCD fail value [%02x] \n", vdd->ccd_fail_val);
}


/***********/
/* A2 line */
/***********/

#define OTHER_PANEL_FILE "/efs/FactoryApp/a2_line.dat"

int read_line(char *src, char *buf, int *pos, int len)
{
	int idx = 0;

	LCD_DEBUG("(%d) ++\n", *pos);

	while (*(src + *pos) != 10 && *(src + *pos) != 13) {
		buf[idx] = *(src + *pos);

		idx++;
		(*pos)++;

		if (idx > MAX_READ_LINE_SIZE) {
			LCD_ERR("overflow!\n");
			return idx;
		}

		if (*pos >= len) {
			LCD_ERR("End of File (%d) / (%d)\n", *pos, len);
			return idx;
		}
	}

	while (*(src + *pos) == 10 || *(src + *pos) == 13)
		(*pos)++;

	LCD_DEBUG("--\n");

	return idx;
}

int ss_read_otherline_panel_data(struct samsung_display_driver_data *vdd)
{
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
	int ret = 0;
	mm_segment_t fs;

	fs = get_fs();
	set_fs(get_ds());

	filp = filp_open(OTHER_PANEL_FILE, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		printk(KERN_ERR "%s File open failed\n", __func__);

		if (!IS_ERR_OR_NULL(vdd->panel_func.set_panel_fab_type))
			vdd->panel_func.set_panel_fab_type(BASIC_FB_PANLE_TYPE);/*to work as original line panel*/
		ret = -ENOENT;
		goto err;
	}

	l = filp->f_path.dentry->d_inode->i_size;
	LCD_INFO("Loading File Size : %ld(bytes)", l);

	dp = kmalloc(l + 10, GFP_KERNEL);
	if (dp == NULL) {
		LCD_INFO("Can't not alloc memory for tuning file load\n");
		filp_close(filp, current->files);
		ret = -1;
		goto err;
	}
	pos = 0;
	memset(dp, 0, l);

	LCD_INFO("before vfs_read()\n");
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	LCD_INFO("after vfs_read()\n");

	if (ret != l) {
		LCD_INFO("vfs_read() filed ret : %d\n", ret);
		kfree(dp);
		filp_close(filp, current->files);
		ret = -1;
		goto err;
	}

	if (!IS_ERR_OR_NULL(vdd->panel_func.parsing_otherline_pdata))
		ret = vdd->panel_func.parsing_otherline_pdata(filp, vdd, dp, l);

	filp_close(filp, current->files);

	set_fs(fs);

	kfree(dp);

	return ret;
err:
	set_fs(fs);
	return ret;
}

static void read_panel_data_work_fn(struct work_struct *work)
{
	struct samsung_display_driver_data *vdd =
		container_of(work, struct samsung_display_driver_data,
			other_line_panel_support_work.work);
	int ret = 1;

	ret = ss_read_otherline_panel_data(vdd);

	if (ret && vdd->other_line_panel_work_cnt) {
		queue_delayed_work(vdd->other_line_panel_support_workq,
				&vdd->other_line_panel_support_work,
				msecs_to_jiffies(OTHERLINE_WORKQ_DEALY));
		vdd->other_line_panel_work_cnt--;
	} else
		destroy_workqueue(vdd->other_line_panel_support_workq);

	if (vdd->other_line_panel_work_cnt == 0)
		LCD_ERR(" cnt (%d)\n", vdd->other_line_panel_work_cnt);
}

/*********************/
/* LPM control       */
/*********************/
void ss_panel_low_power_config(struct samsung_display_driver_data *vdd, int enable)
{
	if (!vdd->dtsi_data.panel_lpm_enable) {
		LCD_INFO("[Panel LPM] LPM(ALPM/HLPM) is not supported\n");
		return;
	}

	ss_panel_lpm_power_ctrl(vdd, enable);

	ss_panel_lpm_ctrl(vdd, enable);

	if (enable) {
		vdd->esd_recovery.is_wakeup_source = true;
	} else {
		vdd->esd_recovery.is_wakeup_source = false;
	}

	if (vdd->esd_recovery.esd_irq_enable)
		vdd->esd_recovery.esd_irq_enable(true, true, (void *)vdd);
}

/*
 * ss_find_reg_offset()
 * This function find offset for reg value
 * reg_list[X][0] is reg value
 * reg_list[X][1] is offset for reg value
 * cmd_list is the target cmds for searching reg value
 */
int ss_find_reg_offset(int (*reg_list)[2],
		struct dsi_panel_cmd_set *cmd_list[], int list_size)
{
	struct dsi_panel_cmd_set *lpm_cmds = NULL;
	int i = 0, j = 0, max_cmd_cnt;

	if (IS_ERR_OR_NULL(reg_list) || IS_ERR_OR_NULL(cmd_list))
		goto end;

	for (i = 0; i < list_size; i++) {
		lpm_cmds = cmd_list[i];
		max_cmd_cnt = lpm_cmds->count;

		for (j = 0; j < max_cmd_cnt; j++) {
			if (lpm_cmds->cmds[j].msg.tx_buf &&
					lpm_cmds->cmds[j].msg.tx_buf[0] == reg_list[i][0]) {
				reg_list[i][1] = j;
				break;
			}
		}
	}

end:
	for (i = 0; i < list_size; i++)
		LCD_DEBUG("offset[%d] : %d\n", i, reg_list[i][1]);
	return 0;
}

static bool is_new_lpm_version(struct samsung_display_driver_data *vdd)
{
	if (vdd->panel_lpm.ver == LPM_VER1)
		return true;
	else
		return false;
}

static void set_lpm_br_values(struct samsung_display_driver_data *vdd)
{
	int from, end;
	int left, right, p = 0;
	int loop = 0;
	struct candela_map_table *table;
	int bl_level = vdd->br.bl_level;

	table = &vdd->dtsi_data.candela_map_table[AOD][vdd->panel_revision];

	if (IS_ERR_OR_NULL(table->cd)) {
		LCD_ERR("No aod candela_map_table..\n");
		return;
	}

	LCD_DEBUG("table size (%d)\n", table->tab_size);

	if (bl_level > table->max_lv)
		bl_level = table->max_lv;

	left = 0;
	right = table->tab_size - 1;

	while (left <= right) {
		loop++;
		p = (left + right) / 2;
		from = table->from[p];
		end = table->end[p];
		LCD_DEBUG("[%d] from(%d) end(%d) / %d\n", p, from, end, bl_level);

		if (bl_level >= from && bl_level <= end)
			break;
		if (bl_level < from)
			right = p - 1;
		else
			left = p + 1;

		if (loop > table->tab_size) {
			pr_err("can not find (%d) level in table!\n", bl_level);
			p = table->tab_size - 1;
			break;
		}
	};
	vdd->panel_lpm.lpm_bl_level = table->cd[p];

	LCD_DEBUG("%s: (%d)->(%d)\n",
		__func__, vdd->br.bl_level, vdd->panel_lpm.lpm_bl_level);

}

int ss_panel_lpm_power_ctrl(struct samsung_display_driver_data *vdd, int enable)
{
	int i;
	int rc = 0;
	int get_voltage;
	struct regulator *elvss = NULL;
	struct dsi_panel *panel = NULL;
	struct dsi_vreg *target_vreg = NULL;
	struct dsi_regulator_info regs;
	struct lpm_pwr_ctrl *lpm_pwr;

	if (!vdd->dtsi_data.panel_lpm_enable) {
		LCD_INFO("[Panel LPM] LPM(ALPM/HLPM) is not supported\n");
		return -ENODEV;
	}

	panel = GET_DSI_PANEL(vdd);
	if (IS_ERR_OR_NULL(panel)) {
		pr_err("No Panel Data\n");
		return -ENODEV;
	}

	LCD_DEBUG("%s ++\n", enable == true ? "Enable" : "Disable");

	regs = panel->power_info;

	if (!regs.count) {
		pr_err("No regulator Data\n");
		return rc;
	}

	lpm_pwr = &vdd->panel_lpm.lpm_pwr;

	if (!lpm_pwr->support_lpm_pwr_ctrl) {
		pr_err("%s: No panel power control for LPM\n", __func__);
		return -ENODEV;
	}

	mutex_lock(&vdd->panel_lpm.lpm_lock);

	/* Find vreg for LPM setting */
	for (i = 0; i < regs.count; i++) {
		target_vreg = &regs.vregs[i];
		if (!strcmp(target_vreg->vreg_name, lpm_pwr->lpm_pwr_ctrl_supply_name)) {
			LCD_INFO("Found Voltage(%d)\n", i);
			break;
		}
	}

	/* To check previous voltage */
	get_voltage = regulator_get_voltage(target_vreg->vreg);

	if (enable) { /* AOD ON(Enter) */
		if (get_voltage != lpm_pwr->lpm_pwr_ctrl_supply_min_v) {
			rc = regulator_set_voltage(
					target_vreg->vreg,
					lpm_pwr->lpm_pwr_ctrl_supply_min_v,
					lpm_pwr->lpm_pwr_ctrl_supply_max_v);
			if (rc < 0) {
				LCD_ERR("Voltage Set Fail enable=%d voltage : %d rc : %d\n",
							enable, lpm_pwr->lpm_pwr_ctrl_supply_min_v, rc);
			} else {
				get_voltage = regulator_get_voltage(target_vreg->vreg);
				LCD_INFO("enable=%d, current get_voltage=%d rc : %d\n", enable, get_voltage, rc);
			}

			if (strlen(lpm_pwr->lpm_pwr_ctrl_elvss_name) > 0) {
				LCD_INFO("ELVSS Name(%s), Level(%d)\n", lpm_pwr->lpm_pwr_ctrl_elvss_name,
								lpm_pwr->lpm_pwr_ctrl_elvss_lpm_v);
				/* Elvss Regulator for Short Detection*/
				elvss = regulator_get(NULL, lpm_pwr->lpm_pwr_ctrl_elvss_name);
#if KERNEL_VER > 409
#if 0
				if (elvss) {
					rc = regulator_set_short_detection(elvss, true,
									vdd->panel_lpm.lpm_pwr.lpm_pwr_ctrl_elvss_lpm_v);
					if (rc < 0)
						LCD_ERR("Regulator Set for AOD Short Detection Fail\n");
					regulator_put(elvss);
				} else
					LCD_ERR("ELVSS Regulator Get Fail\n");
#endif
#endif
			} else
				LCD_ERR("No elvss name for lpm power control\n");
		} else
			LCD_DEBUG("enable=%d, previous voltage : %d\n", enable, get_voltage);

	} else { /* AOD OFF(Exit) */
		if (get_voltage != target_vreg->min_voltage) {
			rc = regulator_set_voltage(
						target_vreg->vreg,
						target_vreg->min_voltage,
						target_vreg->max_voltage);
			if (rc < 0) {
				LCD_ERR("Voltage Set Fail enable=%d voltage : %d rc : %d\n",
							enable, target_vreg->min_voltage, rc);
				panic("Voltage Set Fail to NORMAL");
			} else {
				get_voltage = regulator_get_voltage(target_vreg->vreg);
				LCD_INFO("enable=%d, current get_voltage=%d\n", enable, get_voltage);

				if (get_voltage != target_vreg->min_voltage)
					panic("Voltage Set Fail to NORMAL");
			}

			if (strlen(lpm_pwr->lpm_pwr_ctrl_elvss_name) > 0) {
				LCD_INFO("ELVSS Name(%s), Level(%d)\n", lpm_pwr->lpm_pwr_ctrl_elvss_name,
								lpm_pwr->lpm_pwr_ctrl_elvss_normal_v);
				/* Elvss Regulator for Short Detection*/
				elvss = regulator_get(NULL, lpm_pwr->lpm_pwr_ctrl_elvss_name);
#if KERNEL_VER > 409
#if 0

				if (elvss) {
					rc = regulator_set_short_detection(elvss, true,
									lpm_pwr->lpm_pwr_ctrl_elvss_normal_v);
					if (rc < 0)
						LCD_ERR("Regulator Set for Normal Short Detection Fail\n");
					regulator_put(elvss);
				} else
					LCD_ERR("ELVSS Regulator Get Fail\n");
#endif
#endif
			} else
				LCD_ERR("No elvss name for lpm power control\n");
		} else
			LCD_DEBUG("enable=%d, previous voltage : %d\n", enable, get_voltage);
	}
	if(elvss)
		regulator_put(elvss);

	mutex_unlock(&vdd->panel_lpm.lpm_lock);
	LCD_DEBUG("[Panel LPM] --\n");

	return rc;
}

void ss_panel_lpm_ctrl(struct samsung_display_driver_data *vdd, int enable)
{
	static int stored_bl_level; /* Used for factory mode only */
	int current_bl_level = 0;
	u32 lpm_init_delay = 0;

	LCD_INFO("[Panel LPM] ++\n");

	if (!vdd->dtsi_data.panel_lpm_enable) {
		LCD_INFO("[Panel LPM] LPM(ALPM/HLPM) is not supported\n");
		return;
	}

	if (ss_is_panel_off(vdd)) {
		LCD_INFO("[Panel LPM] Do not change mode\n");
		goto end;
	}

	lpm_init_delay = vdd->dtsi_data.samsung_lpm_init_delay;

	mutex_lock(&vdd->panel_lpm.lpm_lock);

	if (enable) { /* AOD ON(Enter) */
		if (unlikely(vdd->is_factory_mode) && !ss_is_panel_lpm(vdd)) {
			LCD_INFO("[Panel LPM] Set low brightness for factory mode (%d) \n", vdd->br.bl_level);
			stored_bl_level = vdd->br.bl_level;
			ss_brightness_dcs(vdd, 0, BACKLIGHT_NORMAL);
			LCD_INFO("[Panel LPM] Send DISPLAY_OFF for factory mode\n");
			ss_send_cmd(vdd, TX_DISPLAY_OFF);
		}

		if (vdd->panel_func.samsung_update_lpm_ctrl_cmd) {
			vdd->panel_func.samsung_update_lpm_ctrl_cmd(vdd);
			LCD_INFO("[Panel LPM] update lpm cmd done\n");
		}

		/* lpm init delay */
		if (vdd->display_status_dsi.aod_delay == true) {
			vdd->display_status_dsi.aod_delay = false;
			if (lpm_init_delay) {
				msleep(lpm_init_delay);
				LCD_INFO("%ums delay before turn on lpm mode", lpm_init_delay);
			}
		}

		/* Self Display Setting */
		if (vdd->self_disp.aod_enter)
			vdd->self_disp.aod_enter(vdd);

		if (unlikely(vdd->is_factory_mode))
			ss_send_cmd(vdd, TX_FD_OFF);

		ss_send_cmd(vdd, TX_LPM_ON);
 		LCD_INFO("[Panel LPM] Send panel LPM cmds\n");

		if (unlikely(vdd->is_factory_mode))
			ss_send_cmd(vdd, TX_DISPLAY_ON);
		else {
			/* The display_on cmd will be sent on next commit */
			vdd->display_status_dsi.wait_disp_on = true;
			vdd->display_status_dsi.wait_actual_disp_on = true;
			LCD_INFO("[Panel LPM] Set wait_disp_on to true\n");
		}

		vdd->panel_state = PANEL_PWR_LPM;

		/*
			Update mdnie to disable mdnie operation by scenario at AOD display status.
		*/
		if (vdd->mdnie.support_mdnie) {
			update_dsi_tcon_mdnie_register(vdd);
		}


	} else { /* AOD OFF(Exit) */
		/* Self Display Setting */
		if (vdd->self_disp.aod_exit)
			vdd->self_disp.aod_exit(vdd);

		/* Turn Off ALPM Mode */
		ss_send_cmd(vdd, TX_LPM_OFF);

		LCD_INFO("[Panel LPM] Send panel LPM off cmds\n");

		if (unlikely(vdd->is_factory_mode)) {
			LCD_INFO("[Panel LPM] restore bl_level for factory (%d) \n", stored_bl_level);
			current_bl_level = stored_bl_level;
		} else {
			current_bl_level = vdd->br.bl_level;
		}

		LCD_INFO("[Panel LPM] Restore brightness level (%d) \n", current_bl_level);

		vdd->panel_state = PANEL_PWR_ON;

		if ((vdd->support_optical_fingerprint) && (vdd->br.finger_mask_hbm_on)) {
			/* SAMSUNG_FINGERPRINT */
			ss_brightness_dcs(vdd, 0, BACKLIGHT_FINGERMASK_ON);
		}
		else
			ss_brightness_dcs(vdd, current_bl_level, BACKLIGHT_NORMAL);

		if (vdd->mdnie.support_mdnie) {
			vdd->mdnie.lcd_on_notifiy = true;
			update_dsi_tcon_mdnie_register(vdd);
			if (vdd->mdnie.support_trans_dimming)
				vdd->mdnie.disable_trans_dimming = false;
		}

		if (vdd->panel_func.samsung_cover_control && vdd->cover_control)
			vdd->panel_func.samsung_cover_control(vdd);

		if (unlikely(vdd->is_factory_mode)) {
			ss_send_cmd(vdd, TX_FD_ON);
			ss_send_cmd(vdd, TX_DISPLAY_ON);
			vdd->panel_state = PANEL_PWR_ON;
		} else {
			/* The display_on cmd will be sent on next commit */
			vdd->display_status_dsi.wait_disp_on = true;
			vdd->display_status_dsi.wait_actual_disp_on = true;
			LCD_INFO("[Panel LPM] Set wait_disp_on to true\n");
		}

		/* 1Frame Delay(33.4ms - 30FPS) Should be added */
		usleep_range(34*1000, 34*1000);
	}

	LCD_INFO("[Panel LPM] En/Dis : %s, LPM_MODE : %s, Hz : 30Hz, bl_level : %s\n",
				/* Enable / Disable */
				enable ? "Enable" : "Disable",
				/* Check LPM mode */
				vdd->panel_lpm.mode == ALPM_MODE_ON ? "ALPM" :
				vdd->panel_lpm.mode == HLPM_MODE_ON ? "HLPM" :
				vdd->panel_lpm.mode == LPM_MODE_OFF ? "MODE_OFF" : "UNKNOWN",
				/* Check current brightness level */
				vdd->panel_lpm.lpm_bl_level == LPM_2NIT ? "2NIT" :
				vdd->panel_lpm.lpm_bl_level == LPM_10NIT ? "10NIT" :
				vdd->panel_lpm.lpm_bl_level == LPM_30NIT ? "30NIT" :
				vdd->panel_lpm.lpm_bl_level == LPM_60NIT ? "60NIT" : "UNKNOWN");

	mutex_unlock(&vdd->panel_lpm.lpm_lock);
end:
	LCD_INFO("[Panel LPM] --\n");
}

/* This is depricated.. Get vdd pointer not from global variable. */
struct samsung_display_driver_data *ss_get_vdd(enum ss_display_ndx ndx)
{
	if (ndx >= MAX_DISPLAY_NDX || ndx < 0) {
		LCD_ERR("invalid ndx(%d)\n", ndx);
		return NULL;
	}

	return &vdd_data[ndx];
}

/*
 * @param:
 *	D0: hall_ic (the hall ic status, 0: foder open. 1: folder close)
 *	D8: flip_not_refresh (0: refresh after flipping. 1: don't refresh after flipping)
 */
int samsung_display_hall_ic_status(struct notifier_block *nb,
				unsigned long param, void *data)
{
	struct samsung_display_driver_data *vdd = container_of(nb,
			struct samsung_display_driver_data, hall_ic_notifier_display);
	bool hall_ic = (bool)(param & 0x1);
	//bool flip_not_refresh = (bool)(!!(param & LCD_FLIP_NOT_REFRESH));
	struct sde_connector *conn = GET_SDE_CONNECTOR(vdd);
	struct drm_event event;
	bool panel_dead = false;

	/*
		previous panel off -> current panel on
		foder open : 0, close : 1
	*/

	if (!vdd->support_hall_ic)
		return 0;

	LCD_ERR("mdss hall_ic : %s, start\n", hall_ic ? "CLOSE" : "OPEN");

#if 0
	if (ss_panel_attached(PRIMARY_DISPLAY_NDX) && ss_panel_attached(SECONDARY_DISPLAY_NDX)) {
		/* To check current blank mode */
		if ((ss_is_panel_on(vdd) ||
			ss_is_panel_lpm(vdd)) &&
			vdd->hall_ic_status != hall_ic) {

			/* set flag */
			vdd->hall_ic_mode_change_trigger = true;
			vdd->lcd_flip_not_refresh = flip_not_refresh;

			/* panel off */
			// TODO: off panel..
			// call msm_disable_outputs()..???
			LCD_ERR("should implement panel off...\n");

			/* set status */
			vdd->hall_ic_status = hall_ic;

			/* panel on */
			// TODO: off panel..
			// call complete_commit()..???
			LCD_ERR("should implement panel on...\n");

			/* clear flag */
			vdd->hall_ic_mode_change_trigger = false;
			vdd->lcd_flip_not_refresh = false;

			/* Brightness setting */
			// TODO: check if it controls right display in dual dsi or dual display...
			if (ss_is_bl_dcs(vdd))
				ss_brightness_dcs(vdd, vdd->br.bl_level, BACKLIGHT_NORMAL);

			/* display on */
			if (ss_is_video_mode(vdd))
				ss_send_cmd(vdd, TX_DISPLAY_ON);

			/* refresh a frame to panel */
			if (ss_is_cmd_mode(vdd) && !flip_not_refresh) {
				/* TODO: refresh a frame to panel.. with drm msm code...
				   call complete_commit()..??
				fbi->fbops->fb_pan_display(&fbi->var, fbi);
				*/
				LCD_ERR("need to refresh a frame to panel.. with drm msm code...\n");
			}
		} else {
			vdd->hall_ic_status = hall_ic;
			LCD_ERR("mdss skip display changing\n");
		}
	} else {
#endif
	/* check the lcd id for DISPLAY_1 and DISPLAY_2 */
	if (ss_panel_attached(PRIMARY_DISPLAY_NDX) && ss_panel_attached(SECONDARY_DISPLAY_NDX)) {
		/* set status */
		vdd->hall_ic_status_unhandled = hall_ic;
		//vdd->panel_dead = true; panel switch, should not set panel_dead, else MTP is not read

		panel_dead = true;
		event.type = DRM_EVENT_PANEL_DEAD;
		event.length = sizeof(bool);
		msm_mode_object_event_notify(&conn->base.base,
			conn->base.dev, &event, (u8 *)&panel_dead);
		/* TODO: send flip_not_refresh to HAL, and let HAL to handle it. */
	} else {
		/* check the lcd id for DISPLAY_1 */
		if (ss_panel_attached(PRIMARY_DISPLAY_NDX))
			vdd->hall_ic_status = HALL_IC_OPEN;

		/* check the lcd id for DISPLAY_2 */
		if (ss_panel_attached(SECONDARY_DISPLAY_NDX))
			vdd->hall_ic_status = HALL_IC_CLOSE;
	}

	return 0;
}

static void samsung_display_delay_disp_on_work(struct work_struct *work)
{
	struct samsung_display_driver_data *vdd =
		container_of(work, struct samsung_display_driver_data,
				delay_disp_on_work.work);

	LCD_INFO("wait_disp_on is set\n");
	vdd->display_status_dsi.wait_disp_on = true;
	ss_send_cmd(vdd, TX_DISPLAY_ON);
}

int get_hall_ic_status(char *mode)
{
	struct samsung_display_driver_data *vdd;
	int status;

	if (mode == NULL)
		return true;

	if (*mode - '0')
		status = HALL_IC_CLOSE;
	else
		status = HALL_IC_OPEN;


	vdd = ss_get_vdd(PRIMARY_DISPLAY_NDX);
	vdd->hall_ic_status = status;

	vdd = ss_get_vdd(SECONDARY_DISPLAY_NDX);
	vdd->hall_ic_status = status;

	LCD_ERR("hall_ic : %s\n", status ? "CLOSE" : "OPEN");

	return true;
}
EXPORT_SYMBOL(get_hall_ic_status);
__setup("hall_ic=0x", get_hall_ic_status);

/***************************************************************************************************
*		BRIGHTNESS RELATED FUNCTION.
****************************************************************************************************/
static void set_normal_br_values(struct samsung_display_driver_data *vdd)
{
	int from, end;
	int left, right, p = 0;
	int loop = 0;
	struct candela_map_table *table;

	if (vdd->br.pac)
		table = &vdd->dtsi_data.candela_map_table[PAC_NORMAL][vdd->panel_revision];
	else {
		if (vdd->br.gamma_mode2_support)
			table = &vdd->dtsi_data.candela_map_table[GAMMA_MODE2_NORMAL][vdd->panel_revision];
		else
			table = &vdd->dtsi_data.candela_map_table[NORMAL][vdd->panel_revision];
	}

	if (IS_ERR_OR_NULL(table->cd)) {
		LCD_ERR("No candela_map_table.. \n");
		return;
	}

	LCD_DEBUG("table size (%d)\n", table->tab_size);

	if (vdd->br.bl_level > table->max_lv)
		vdd->br.bl_level = table->max_lv;

	left = 0;
	right = table->tab_size - 1;

	while (left <= right) {
		loop++;
		p = (left + right) / 2;
		from = table->from[p];
		end = table->end[p];
		LCD_DEBUG("[%d] from(%d) end(%d) / %d\n", p, from, end, vdd->br.bl_level);

		if (vdd->br.bl_level >= from && vdd->br.bl_level <= end)
			break;
		if (vdd->br.bl_level < from)
			right = p - 1;
		else
			left = p + 1;
		LCD_DEBUG("left(%d) right(%d)\n", left, right);

		if (loop > table->tab_size) {
			pr_err("can not find (%d) level in table!\n", vdd->br.bl_level);
			p = table->tab_size - 1;
			break;
		}
	};

	// set values..
	vdd->br.cd_idx = table->idx[p];
	vdd->br.cd_level = table->cd[p];

	if (vdd->br.pac) {
		vdd->br.pac_cd_idx = table->scaled_idx[p];
		vdd->br.interpolation_cd = table->interpolation_cd[p];

		LCD_INFO("[%d] pac_cd_idx (%d) cd_idx (%d) cd (%d) interpolation_cd (%d)\n",
			p, vdd->br.pac_cd_idx, vdd->br.cd_idx, vdd->br.cd_level, vdd->br.interpolation_cd);
	} else {
		if (vdd->br.gamma_mode2_support) {
			vdd->br.gamma_mode2_cd = table->gamma_mode2_cd[p];
			LCD_INFO("[%d] cd_idx (%d) cd (%d) panel_real_cd (%d)\n",
				p, vdd->br.cd_idx, vdd->br.cd_level, vdd->br.gamma_mode2_cd);
		} else {
			LCD_INFO("[%d] cd_idx (%d) cd (%d) interpolation_cd (%d)\n",
				p, vdd->br.cd_idx, vdd->br.cd_level, vdd->br.interpolation_cd);
		}
	}
	return;
}

static void set_hbm_br_values(struct samsung_display_driver_data *vdd)
{
	int from, end;
	int left, right, p = 0;
	int loop = 0;
	struct candela_map_table *table;

	if (vdd->br.pac)
		table = &vdd->dtsi_data.candela_map_table[PAC_HBM][vdd->panel_revision];
	else
		table = &vdd->dtsi_data.candela_map_table[HBM][vdd->panel_revision];

	if (IS_ERR_OR_NULL(table->cd)) {
		LCD_ERR("No hbm candela_map_table..\n");
		return;
	}

	if (vdd->br.bl_level > table->max_lv)
		vdd->br.bl_level = table->max_lv;

	left = 0;
	right = table->tab_size - 1;

	while (left <= right) {
		loop++;
		p = (left + right) / 2;
		from = table->from[p];
		end = table->end[p];
		LCD_DEBUG("[%d] from(%d) end(%d) / %d\n", p, from, end, vdd->br.bl_level);

		if (vdd->br.bl_level >= from && vdd->br.bl_level <= end)
			break;
		if (vdd->br.bl_level < from)
			right = p - 1;
		else
			left = p + 1;
		LCD_DEBUG("left(%d) right(%d)\n", left, right);

		if (loop > table->tab_size) {
			pr_err("can not find (%d) level in table!\n", vdd->br.bl_level);
			p = table->tab_size - 1;
			break;
		}
	};

	// set values..
	vdd->br.cd_idx = table->idx[p];
	vdd->br.cd_level = table->cd[p];
	vdd->br.auto_level = table->auto_level[p];

	if (vdd->br.pac) {
		vdd->br.pac_cd_idx = table->scaled_idx[p];
		vdd->br.interpolation_cd = table->interpolation_cd[p];

		LCD_INFO("[%d] pac_cd_idx (%d) cd_idx (%d) cd (%d) interpolation_cd (%d) auto (%d)\n",
			p, vdd->br.pac_cd_idx, vdd->br.cd_idx, vdd->br.cd_level, vdd->br.interpolation_cd, vdd->br.auto_level);
	} else
		LCD_INFO("[%d] cd_idx (%d) cd (%d) interpolation_cd (%d) auto (%d) \n",
			p, vdd->br.cd_idx, vdd->br.cd_level, vdd->br.interpolation_cd, vdd->br.auto_level);

	return;
}

static void ss_update_brightness_packet(struct dsi_cmd_desc *packet,
		int *count, struct dsi_panel_cmd_set *tx_cmd)
{
	int loop = 0;

	if (IS_ERR_OR_NULL(packet)) {
		LCD_ERR("%ps no packet\n", __builtin_return_address(0));
		return;
	}

	if (SS_IS_CMDS_NULL(tx_cmd)) {
		LCD_ERR("%ps no tx_cmd\n", __builtin_return_address(0));
		return;
	}

	if (*count > (BRIGHTNESS_MAX_PACKET - 1))
		panic("over max brightness_packet size(%d).. !!",
				BRIGHTNESS_MAX_PACKET);

	for (loop = 0; loop < tx_cmd->count; loop++)
		packet[(*count)++] = tx_cmd->cmds[loop];
}

void update_packet_level_key_enable(struct samsung_display_driver_data *vdd,
		struct dsi_cmd_desc *packet, int *cmd_cnt, int level_key)
{
	if (!level_key)
		return;
	else {
		if (level_key & LEVEL0_KEY)
			ss_update_brightness_packet(packet, cmd_cnt, ss_get_cmds(vdd, TX_LEVEL0_KEY_ENABLE));

		if (level_key & LEVEL1_KEY)
			ss_update_brightness_packet(packet, cmd_cnt, ss_get_cmds(vdd, TX_LEVEL1_KEY_ENABLE));

		if (level_key & LEVEL2_KEY)
			ss_update_brightness_packet(packet, cmd_cnt, ss_get_cmds(vdd, TX_LEVEL2_KEY_ENABLE));
	}
}

void update_packet_level_key_disable(struct samsung_display_driver_data *vdd,
		struct dsi_cmd_desc *packet, int *cmd_cnt, int level_key)
{
	if (!level_key)
		return;
	else {
		if (level_key & LEVEL0_KEY)
			ss_update_brightness_packet(packet, cmd_cnt, ss_get_cmds(vdd, TX_LEVEL0_KEY_DISABLE));

		if (level_key & LEVEL1_KEY)
			ss_update_brightness_packet(packet, cmd_cnt, ss_get_cmds(vdd, TX_LEVEL1_KEY_DISABLE));

		if (level_key & LEVEL2_KEY)
			ss_update_brightness_packet(packet, cmd_cnt, ss_get_cmds(vdd, TX_LEVEL2_KEY_DISABLE));
	}
}

static int ss_hbm_brightness_packet_set(
		struct samsung_display_driver_data *vdd)
{
	int cmd_cnt = 0;
	int level_key = 0;
	struct dsi_panel_cmd_set *set;
	struct dsi_cmd_desc *packet = NULL;
	struct dsi_panel_cmd_set *tx_cmd = NULL;

	/* init packet */
	set = ss_get_cmds(vdd, TX_BRIGHT_CTRL);
	if (SS_IS_CMDS_NULL(set)) {
		LCD_ERR("No cmds for TX_BRIGHT_CTRL.. \n");
		return -EINVAL;
	}

	packet = set->cmds;

	set_hbm_br_values(vdd);

	/* IRC */
	if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_hbm_irc)) {
		level_key = false;
		tx_cmd = vdd->panel_func.samsung_hbm_irc(vdd, &level_key);

		update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
		ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
		update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
	}

	/* Gamma */
	if (ss_get_cmds(vdd, TX_HBM_GAMMA)->count) {
		if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_hbm_gamma)) {
			level_key = false;
			tx_cmd = vdd->panel_func.samsung_hbm_gamma(vdd, &level_key);

			update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
			ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
			update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
		}
	}

	/* hbm etc */
	if (ss_get_cmds(vdd, TX_HBM_ETC)->count) {
		if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_hbm_etc)) {
			level_key = false;
			tx_cmd = vdd->panel_func.samsung_hbm_etc(vdd, &level_key);

			update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
			ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
			update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
		}
	}

	return cmd_cnt;
}

static int ss_normal_brightness_packet_set(
		struct samsung_display_driver_data *vdd)
{
	int cmd_cnt = 0;
	int level_key = 0;
	struct dsi_panel_cmd_set *set;
	struct dsi_cmd_desc *packet = NULL;
	struct dsi_panel_cmd_set *tx_cmd = NULL;

	/* init packet */
	set = ss_get_cmds(vdd, TX_BRIGHT_CTRL);
	if (SS_IS_CMDS_NULL(set)) {
		LCD_ERR("No cmds for TX_BRIGHT_CTRL.. \n");
		return -EINVAL;
	}
	packet = set->cmds;

	vdd->br.auto_level = 0;

	set_normal_br_values(vdd);

	if (vdd->smart_dimming_loaded_dsi) { /* OCTA PANEL */
		/* hbm off */
		if (vdd->display_status_dsi.hbm_mode) {
			if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_hbm_off)) {
				level_key = false;
				tx_cmd = vdd->panel_func.samsung_brightness_hbm_off(vdd, &level_key);

				update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
				ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
				update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
			}
		}

		/* aid/aor */
		if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_aid)) {
			level_key = false;
			tx_cmd = vdd->panel_func.samsung_brightness_aid(vdd, &level_key);

			update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
			ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
			update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
		}

		/* acl */
		if (vdd->acl_status || vdd->siop_status) {
			if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_acl_on)) {
				level_key = false;
				tx_cmd = vdd->panel_func.samsung_brightness_acl_on(vdd, &level_key);

				update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
				ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
				update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
			}

			if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_pre_acl_percent)) {
				level_key = false;
				tx_cmd = vdd->panel_func.samsung_brightness_pre_acl_percent(vdd, &level_key);

				update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
				ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
				update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
			}

			if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_acl_percent)) {
				level_key = false;
				tx_cmd = vdd->panel_func.samsung_brightness_acl_percent(vdd, &level_key);

				update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
				ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
				update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
			}
		} else {
			if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_acl_off)) {
				level_key = false;
				tx_cmd = vdd->panel_func.samsung_brightness_acl_off(vdd, &level_key);

				update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
				ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
				update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
			}
		}

		/* elvss */
		if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_elvss)) {
			if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_pre_elvss)) {
				level_key = false;
				tx_cmd = vdd->panel_func.samsung_brightness_pre_elvss(vdd, &level_key);

				update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
				ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
				update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
			}

			level_key = false;
			tx_cmd = vdd->panel_func.samsung_brightness_elvss(vdd, &level_key);

			update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
			ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
			update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
		}

		/* temperature elvss */
		if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_elvss_temperature1)) {
			level_key = false;
			tx_cmd = vdd->panel_func.samsung_brightness_elvss_temperature1(vdd, &level_key);

			update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
			ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
			update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
		}

		if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_elvss_temperature2)) {
			level_key = false;
			tx_cmd = vdd->panel_func.samsung_brightness_elvss_temperature2(vdd, &level_key);

			update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
			ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
			update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
		}

		/* caps*/
		if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_caps)) {
			if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_pre_caps)) {
				level_key = false;
				tx_cmd = vdd->panel_func.samsung_brightness_pre_caps(vdd, &level_key);

				update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
				ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
				update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
			}
			level_key = false;
			tx_cmd = vdd->panel_func.samsung_brightness_caps(vdd, &level_key);

			update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
			ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
			update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
		}

		/* vint */
		if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_vint)) {
			level_key = false;
			tx_cmd = vdd->panel_func.samsung_brightness_vint(vdd, &level_key);

			update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
			ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
			update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
		}

		/* IRC */
		if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_irc)) {
			level_key = false;
			tx_cmd = vdd->panel_func.samsung_brightness_irc(vdd, &level_key);

			update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
			ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
			update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
		}

		/* gamma */
		if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_gamma)) {
			level_key = false;
			tx_cmd = vdd->panel_func.samsung_brightness_gamma(vdd, &level_key);

			update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
			ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
			update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
		}
	} else { /* TFT PANEL */
		if (vdd->acl_status || vdd->siop_status) {
			if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_acl_on)) {
				level_key = false;
				tx_cmd = vdd->panel_func.samsung_brightness_acl_on(vdd, &level_key);

				update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
				ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
				update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
			}

			if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_pre_acl_percent)) {
				level_key = false;
				tx_cmd = vdd->panel_func.samsung_brightness_pre_acl_percent(vdd, &level_key);

				update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
				ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
				update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
			}

			if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_acl_percent)) {
				level_key = false;
				tx_cmd = vdd->panel_func.samsung_brightness_acl_percent(vdd, &level_key);

				update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
				ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
				update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
			}
		} else {
			if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_acl_off)) {
				level_key = false;
				tx_cmd = vdd->panel_func.samsung_brightness_acl_off(vdd, &level_key);

				update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
				ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
				update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
			}
		}
		if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_tft_pwm_ldi)) {
			level_key = false;
			tx_cmd = vdd->panel_func.samsung_brightness_tft_pwm_ldi(vdd, &level_key);

			update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
			ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
			update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
		}
		if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_gamma_mode2)) {
			level_key = false;
			tx_cmd = vdd->panel_func.samsung_brightness_gamma_mode2(vdd, &level_key);

			update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
			ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
			update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);

			/* vint */
			if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_vint)) {
				level_key = false;
				tx_cmd = vdd->panel_func.samsung_brightness_vint(vdd, &level_key);

				update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
				ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
				update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
			}
		}

	}

	return cmd_cnt;
}

int ss_single_transmission_packet(struct dsi_panel_cmd_set *cmds)
{
	int loop;
	struct dsi_cmd_desc *packet = cmds->cmds;
	int packet_cnt = cmds->count;

	for (loop = 0; (loop < packet_cnt) && (loop < BRIGHTNESS_MAX_PACKET); loop++) {
		if (packet[loop].msg.type == MIPI_DSI_DCS_LONG_WRITE ||
			packet[loop].msg.type == MIPI_DSI_GENERIC_LONG_WRITE)
			packet[loop].last_command = false;
		else {
			if (loop > 0)
				packet[loop - 1].last_command = true; /*To ensure previous single tx packet */

			packet[loop].last_command = true;
		}
	}

	if (loop == BRIGHTNESS_MAX_PACKET)
		return false;
	else {
		packet[loop - 1].last_command = true; /* To make last packet flag */
		return true;
	}
}

bool is_hbm_level(struct samsung_display_driver_data *vdd)
{

	struct candela_map_table *table;

	if (vdd->br.pac)
		table = &vdd->dtsi_data.candela_map_table[PAC_HBM][vdd->panel_revision];
	else
		table = &vdd->dtsi_data.candela_map_table[HBM][vdd->panel_revision];

	if (vdd->br.bl_level < table->min_lv)
		return false;

	if (!table->max_lv)//max should not be zero
		return false;

	if (vdd->br.bl_level > table->max_lv) {
		LCD_ERR("bl_level(%d) is over max_level (%d), force set to max\n", vdd->br.bl_level, table->max_lv);
		vdd->br.bl_level = table->max_lv;
	}

	return true;
}

/* ss_brightness_dcs() is called not in locking status.
 * Instead, calls ss_set_backlight() when you need to controll backlight
 * in locking status.
 */
int ss_brightness_dcs(struct samsung_display_driver_data *vdd, int level, int backlight_origin)
{
	int cmd_cnt = 0;
	int ret = 0;
	int need_lpm_lock = 0;
	struct dsi_panel_cmd_set *brightness_cmds = NULL;
	struct dsi_panel *panel = GET_DSI_PANEL(vdd);
	static int backup_bl_level, backup_acl;

	/* FC2 change: set panle mode in SurfaceFlinger initialization, instead of kenrel booting... */
	if (!panel->cur_mode) {
		LCD_ERR("err: no panel mode yet...\n");
		return false;
	}

	/*
	 * panel_lpm.lpm_lock should be locked
	 * to avoid brightness mis-handling (LPM brightness <-> normal brightness) from other thread
	 * but running ss_panel_lpm_ctrl thread
	*/
	if (ss_is_panel_lpm(vdd)) need_lpm_lock = 1;
	if (need_lpm_lock) mutex_lock(&vdd->panel_lpm.lpm_lock);

	// need bl_lock..
	mutex_lock(&vdd->bl_lock);


	if (vdd->support_optical_fingerprint) {
		/* SAMSUNG_FINGERPRINT */
		/* From nomal at finger mask on state
		* 1. backup acl
		* 2. backup level if it is not the same value as vdd->br.bl_level
		*   note. we don't need to backup this case because vdd->br.bl_level is a finger_mask_bl_level now
		*/

		if ((vdd->br.finger_mask_hbm_on) && (backlight_origin == BACKLIGHT_NORMAL)) {/* finger mask hbm on & bl update from normal */
			backup_acl = vdd->acl_status;
			if (level != USE_CURRENT_BL_LEVEL)
				backup_bl_level = level;
			LCD_INFO("[FINGER_MASK]BACKLIGHT_NORMAL save backup_acl = %d, backup_level = %d, vdd->br.bl_level=%d\n", backup_acl, backup_bl_level, vdd->br.bl_level);
			goto skip_bl_update;
		}

		/* From finger mask on
		* 1. use finger_mask_bl_level(HBM) & acl 0
		* 2. backup previous bl_level & acl value
		*  From finger mask off
		* 1. restore backup bl_level & acl value
		*/

		if (backlight_origin == BACKLIGHT_FINGERMASK_ON) {
			vdd->br.finger_mask_hbm_on = true;
			ss_wait_for_te_gpio(vdd, 1, 2000);
			backup_acl = vdd->acl_status;
			if (vdd->finger_mask_updated) /* do not backup br.bl_level at on to on */
				backup_bl_level = vdd->br.bl_level;
			level = vdd->br.finger_mask_bl_level;
			vdd->acl_status = 0;

			LCD_INFO("[FINGER_MASK]BACKLIGHT_FINGERMASK_ON turn on finger hbm & back up acl = %d, level = %d\n", backup_acl, backup_bl_level);
		}
		else if(backlight_origin == BACKLIGHT_FINGERMASK_OFF) {
			vdd->br.finger_mask_hbm_on = false;
			ss_wait_for_te_gpio(vdd, 1, 2000);
			vdd->acl_status = backup_acl;
			level = backup_bl_level;

			LCD_INFO("[FINGER_MASK]BACKLIGHT_FINGERMASK_OFF turn off finger hbm & restore acl = %d, level = %d\n", vdd->acl_status, level);
		}
	}

	if (level != USE_CURRENT_BL_LEVEL)
		vdd->br.bl_level = level;

	/* check the lcd id for DISPLAY_1 or DISPLAY_2 */
	if (!ss_panel_attached(vdd->ndx))
		goto skip_bl_update;

	if (vdd->grayspot) {
		LCD_ERR("grayspot on.. %d \n", vdd->grayspot);
		goto skip_bl_update;
	}

	if (vdd->dtsi_data.flash_gamma_support &&
		!vdd->panel_br_info.flash_data.init_done) {
		LCD_ERR("flash_gamme not ready\n");
		goto skip_bl_update;
	}

	if (vdd->dtsi_data.panel_lpm_enable && is_new_lpm_version(vdd)) {
		set_lpm_br_values(vdd);

		if (ss_is_panel_lpm(vdd)) {
			LCD_ERR("[Panel LPM]: set brightness.(%d)->(%d)\n", vdd->br.bl_level, vdd->panel_lpm.lpm_bl_level);

			if (vdd->panel_func.samsung_set_lpm_brightness)
				vdd->panel_func.samsung_set_lpm_brightness(vdd);
			goto skip_bl_update;
		}
	}

	if (vdd->dtsi_data.hmt_enabled && vdd->hmt_stat.hmt_on) {
		LCD_ERR("HMT is on. do not set normal brightness..(%d)\n", level);
		goto skip_bl_update;
	}

	if (ss_is_seamless_mode(vdd)) {
		LCD_ERR("splash is not done..\n");
		goto skip_bl_update;
	}

	if (!vdd->dtsi_data.tft_common_support && is_hbm_level(vdd)) {
		cmd_cnt = ss_hbm_brightness_packet_set(vdd);
		cmd_cnt > 0 ? vdd->display_status_dsi.hbm_mode = true : false;
	} else {
		cmd_cnt = ss_normal_brightness_packet_set(vdd);
		cmd_cnt > 0 ? vdd->display_status_dsi.hbm_mode = false : false;
	}

	if (cmd_cnt) {
		/* setting tx cmds cmt */
		brightness_cmds = ss_get_cmds(vdd, TX_BRIGHT_CTRL);
		brightness_cmds->count = cmd_cnt;

		/* generate single tx packet */
		ret = ss_single_transmission_packet(brightness_cmds);

		/* sending tx cmds */
		if (ret) {
			ss_send_cmd(vdd, TX_BRIGHT_CTRL);

			if (!IS_ERR_OR_NULL(ss_get_cmds(vdd, TX_BLIC_DIMMING)->cmds)) {
				if (vdd->br.bl_level == 0)
					ss_get_cmds(vdd, TX_BLIC_DIMMING)->cmds->msg.tx_buf[1] = 0x24;
				else
					ss_get_cmds(vdd, TX_BLIC_DIMMING)->cmds->msg.tx_buf[1] = 0x2C;

				ss_send_cmd(vdd, TX_BLIC_DIMMING);
			}

			// copr sum after changing brightness to calculate brightness avg.
			if (vdd->copr.copr_on) {
				ss_set_copr_sum(vdd, COPR_CD_INDEX_0);
				ss_set_copr_sum(vdd, COPR_CD_INDEX_1);
			}
			/* send poc compensation */
			if (vdd->poc_driver.is_support)
				ss_poc_comp(vdd);

			LCD_INFO("level : %d  candela : %dCD hbm : %d (%d) itp : %s\n",
				vdd->br.bl_level, vdd->br.cd_level, vdd->display_status_dsi.hbm_mode, vdd->br.auto_level,
				vdd->panel_br_info.itp_mode == 0 ? "TABLE" : "FLASH");

			queue_work(vdd->br.br_wq, &vdd->br.br_work);

		} else
			LCD_INFO("single_transmission_fail error\n");
	} else
		LCD_INFO("level : %d skip\n", vdd->br.bl_level);

skip_bl_update:
	if (vdd->support_optical_fingerprint) {
	/* SAMSUNG_FINGERPRINT */
	/* hbm needs vdd->panel_hbm_entry_delay TE to be updated, where as normal needs no */
		if (backlight_origin == BACKLIGHT_FINGERMASK_ON) {
			ss_wait_for_te_gpio(vdd, vdd->panel_hbm_entry_delay, 200);
		}
		if ((backlight_origin >= BACKLIGHT_FINGERMASK_ON) && (vdd->finger_mask_updated)) {
			SDE_ERROR("finger_mask_updated/ sysfs_notify finger_mask_state = %d\n", vdd->finger_mask);
			sysfs_notify(&vdd->lcd_dev->dev.kobj, NULL, "actual_mask_brightness");
			vdd->finger_mask_updated = 0;
		}
	}
	mutex_unlock(&vdd->bl_lock);

	if (need_lpm_lock) mutex_unlock(&vdd->panel_lpm.lpm_lock);

	return 0;
}


int ss_reading_mode_dcs(struct samsung_display_driver_data *vdd)
{
	struct dsi_panel_cmd_set *pcmds;

	if (!vdd->reading_mode.support_reading_mode)
		return 0;

	pcmds = ss_get_cmds(vdd, TX_READING_MODE_TUNE);
	pcmds->cmds = vdd->reading_mode.reading_mode_tune_dsi[vdd->reading_mode.cur_level];
	pcmds->count = vdd->reading_mode.dsi_tune_size;

	ss_send_cmd(vdd, TX_READING_MODE_TUNE);

	return 0;
}
// HMT brightness
static void set_hmt_br_values(struct samsung_display_driver_data *vdd)
{
	int from, end;
	int left, right, p = 0;
	struct candela_map_table *table;

	table = &vdd->dtsi_data.candela_map_table[HMT][vdd->panel_revision];

	if (IS_ERR_OR_NULL(table->cd)) {
		LCD_ERR("No candela_map_table..\n");
		return;
	}

	LCD_DEBUG("table size (%d)\n", table->tab_size);

	if (vdd->hmt_stat.hmt_bl_level > table->max_lv)
		vdd->hmt_stat.hmt_bl_level = table->max_lv;

	left = 0;
	right = table->tab_size - 1;

	while (left <= right) {
		p = (left + right) / 2;
		from = table->from[p];
		end = table->end[p];
		LCD_DEBUG("[%d] from(%d) end(%d) / %d\n", p, from, end, vdd->hmt_stat.hmt_bl_level);

		if (vdd->hmt_stat.hmt_bl_level >= from && vdd->hmt_stat.hmt_bl_level <= end)
			break;
		if (vdd->hmt_stat.hmt_bl_level < from)
			right = p - 1;
		else
			left = p + 1;
	};

	// for elvess, vint etc.. which are using 74 steps.
	vdd->br.interpolation_cd = vdd->hmt_stat.candela_level_hmt = table->cd[p];
	vdd->hmt_stat.cmd_idx_hmt = table->idx[p];

	LCD_INFO("cd_idx (%d) cd_level (%d) \n", vdd->hmt_stat.cmd_idx_hmt, vdd->hmt_stat.candela_level_hmt);

	return;
}

int ss_hmt_brightenss_packet_set(
		struct samsung_display_driver_data *vdd)
{
	int cmd_cnt = 0;
	int level_key = 0;
	struct dsi_panel_cmd_set *set;
	struct dsi_cmd_desc *packet = NULL;
	struct dsi_panel_cmd_set *tx_cmd = NULL;

	LCD_DEBUG("++\n");

	set_hmt_br_values(vdd);

	/* init packet */
	set = ss_get_cmds(vdd, TX_BRIGHT_CTRL);
	if (SS_IS_CMDS_NULL(set)) {
		LCD_ERR("No cmds for TX_BRIGHT_CTRL.. \n");
		return -EINVAL;
	}
	packet = set->cmds;

	if (vdd->smart_dimming_hmt_loaded_dsi) {
		/* aid/aor B2 */
		if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_aid_hmt)) {
			level_key = false;
			tx_cmd = vdd->panel_func.samsung_brightness_aid_hmt(vdd, &level_key);

			update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
			ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
			update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
		}

		/* elvss B5 */
		if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_elvss_hmt)) {
			level_key = false;
			tx_cmd = vdd->panel_func.samsung_brightness_elvss_hmt(vdd, &level_key);

			update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
			ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
			update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
		}

		/* vint F4 */
		if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_vint_hmt)) {
			level_key = false;
			tx_cmd = vdd->panel_func.samsung_brightness_vint_hmt(vdd, &level_key);

			update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
			ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
			update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
		}

		/* gamma CA */
		if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_gamma_hmt)) {
			level_key = false;
			tx_cmd = vdd->panel_func.samsung_brightness_gamma_hmt(vdd, &level_key);

			update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
			ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
			update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
		}
    }else{
        if (!IS_ERR_OR_NULL(vdd->panel_func.samsung_brightness_hmt)) {
            level_key = false;
            tx_cmd = vdd->panel_func.samsung_brightness_hmt(vdd, &level_key);
            update_packet_level_key_enable(vdd, packet, &cmd_cnt, level_key);
            ss_update_brightness_packet(packet, &cmd_cnt, tx_cmd);
            update_packet_level_key_disable(vdd, packet, &cmd_cnt, level_key);
        }
	}

	LCD_DEBUG("--\n");

	return cmd_cnt;
}

int ss_brightness_dcs_hmt(struct samsung_display_driver_data *vdd,
		int level)
{
	struct dsi_panel_cmd_set *brightness_cmds = NULL;
	int cmd_cnt;
	int ret = 0;

	brightness_cmds = ss_get_cmds(vdd, TX_BRIGHT_CTRL);

	vdd->hmt_stat.hmt_bl_level = level;
	LCD_ERR("[HMT] hmt_bl_level(%d)\n", vdd->hmt_stat.hmt_bl_level);

	cmd_cnt = ss_hmt_brightenss_packet_set(vdd);

	/* sending tx cmds */
	if (cmd_cnt) {
		/* setting tx cmds cmt */
		brightness_cmds->count = cmd_cnt;

		/* generate single tx packet */
		ret = ss_single_transmission_packet(brightness_cmds);

		if (ret) {
			ss_send_cmd(vdd, TX_BRIGHT_CTRL);

			LCD_INFO("idx(%d), cd_level(%d), hmt_bl_level(%d) itp : %s",
				vdd->hmt_stat.cmd_idx_hmt, vdd->hmt_stat.candela_level_hmt, vdd->hmt_stat.hmt_bl_level,
				vdd->panel_br_info.itp_mode == 0 ? "TABLE" : "FLASH'");
		} else
			LCD_DEBUG("single_transmission_fail error\n");
	} else
		LCD_INFO("level : %d skip\n", vdd->br.bl_level);

	return cmd_cnt;
}

// TFT brightness

void ss_brightness_tft_pwm(struct samsung_display_driver_data *vdd, int level)
{
	if (vdd == NULL) {
		LCD_ERR("no PWM\n");
		return;
	}

	if (ss_is_panel_off(vdd))
		return;

	vdd->br.bl_level = level;

	if (vdd->panel_func.samsung_brightness_tft_pwm)
		vdd->panel_func.samsung_brightness_tft_pwm(vdd, level);
}

void ss_tft_autobrightness_cabc_update(struct samsung_display_driver_data *vdd)
{
	LCD_INFO("\n");

	switch (vdd->br.auto_level) {
	case 0:
		ss_cabc_update(vdd);
		break;
	case 1:
	case 2:
	case 3:
	case 4:
		ss_send_cmd(vdd, TX_CABC_ON);
		break;
	case 5:
	case 6:
		ss_send_cmd(vdd, TX_CABC_OFF);
		break;
	}
}

void ss_read_mtp(struct samsung_display_driver_data *vdd, int addr, int len, int pos, u8 *buf)
{
	struct dsi_panel_cmd_set *rx_cmds;

	if (IS_ERR_OR_NULL(vdd)) {
		LCD_ERR("no vdd");
		return;
	}

	if (!ss_is_ready_to_send_cmd(vdd)) {
		LCD_ERR("Panel is not ready. Panel State(%d)\n", vdd->panel_state);
		return;
	}

	if (addr > 0xFF || pos > 0xFF || len > 0xFF) {
		LCD_ERR("unvalind para addr(%x) pos(%d) len(%d)\n", addr, pos, len);
		return;
	}

	rx_cmds = ss_get_cmds(vdd, RX_MTP_READ_SYSFS);
	if (SS_IS_CMDS_NULL(rx_cmds)) {
		LCD_ERR("No cmds for RX_MTP_READ_SYSFS.. \n");
		return;
	}

	rx_cmds->cmds[0].msg.tx_buf[0] =  addr;
	rx_cmds->cmds[0].msg.tx_buf[1] =  len;
	rx_cmds->cmds[0].msg.tx_buf[2] =  pos;

	rx_cmds->cmds[0].msg.rx_len =  len;
	rx_cmds->read_startoffset = pos;

	mutex_lock(&vdd->exclusive_tx.ex_tx_lock);
	vdd->exclusive_tx.permit_frame_update = 1;
	vdd->exclusive_tx.enable = 1;

	ss_set_exclusive_tx_packet(vdd, RX_MTP_READ_SYSFS, 1);
	ss_set_exclusive_tx_packet(vdd, TX_LEVEL1_KEY_ENABLE, 1);
	ss_set_exclusive_tx_packet(vdd, TX_LEVEL1_KEY_DISABLE, 1);
	ss_set_exclusive_tx_packet(vdd, TX_REG_READ_POS, 1);

	ss_panel_data_read(vdd, RX_MTP_READ_SYSFS, buf, LEVEL1_KEY);

	ss_set_exclusive_tx_packet(vdd, RX_MTP_READ_SYSFS, 0);
	ss_set_exclusive_tx_packet(vdd, TX_LEVEL1_KEY_ENABLE, 0);
	ss_set_exclusive_tx_packet(vdd, TX_LEVEL1_KEY_DISABLE, 0);
	ss_set_exclusive_tx_packet(vdd, TX_REG_READ_POS, 0);

	vdd->exclusive_tx.permit_frame_update = 0;
	vdd->exclusive_tx.enable = 0;
	wake_up_all(&vdd->exclusive_tx.ex_tx_waitq);
	mutex_unlock(&vdd->exclusive_tx.ex_tx_lock);

	return;
}

void ss_write_mtp(struct samsung_display_driver_data *vdd, int len, u8 *buf)
{
	struct dsi_panel_cmd_set *tx_cmds;

	if (IS_ERR_OR_NULL(vdd)) {
		LCD_ERR("no vdd");
		return;
	}

	if (!ss_is_ready_to_send_cmd(vdd)) {
		LCD_ERR("Panel is not ready. Panel State(%d)\n", vdd->panel_state);
		return;
	}

	if (len > 0xFF) {
		LCD_ERR("unvalind para len(%d)\n", len);
		return;
	}

	tx_cmds = ss_get_cmds(vdd, TX_MTP_WRITE_SYSFS);
	if (SS_IS_CMDS_NULL(tx_cmds)) {
		LCD_ERR("No cmds for TX_MTP_WRITE_SYSFS.. \n");
		return;
	}

	tx_cmds->cmds[0].msg.tx_len = len;
	tx_cmds->cmds[0].msg.tx_buf = buf;

	ss_send_cmd(vdd, TX_LEVEL0_KEY_ENABLE);
	ss_send_cmd(vdd, TX_LEVEL1_KEY_ENABLE);
	ss_send_cmd(vdd, TX_LEVEL2_KEY_ENABLE);

	ss_send_cmd(vdd, TX_MTP_WRITE_SYSFS);

	ss_send_cmd(vdd, TX_LEVEL2_KEY_DISABLE);
	ss_send_cmd(vdd, TX_LEVEL1_KEY_DISABLE);
	ss_send_cmd(vdd, TX_LEVEL0_KEY_DISABLE);

	return;
}

static BLOCKING_NOTIFIER_HEAD(panel_notifier_list);

/**
 *	panel_notifier_register - register a client notifier
 *	@nb: notifier block to callback on events
 */
int panel_notifier_register(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&panel_notifier_list, nb);
}
EXPORT_SYMBOL(panel_notifier_register);

/**
 *	panel_notifier_unregister - unregister a client notifier
 *	@nb: notifier block to callback on events
 */
int panel_notifier_unregister(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&panel_notifier_list, nb);
}
EXPORT_SYMBOL(panel_notifier_unregister);

/**
 * panel_notifier_call_chain - notify clients
 *
 */
int panel_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&panel_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(panel_notifier_call_chain);

static void ss_brightness_work(struct work_struct *work)
{
	struct samsung_display_driver_data *vdd = NULL;
	struct brightness_info *br;
	struct panel_bl_event_data bl_evt_data;

	br = container_of(work, struct brightness_info, br_work);
	vdd = container_of(br, struct samsung_display_driver_data, br);

	LCD_DEBUG("brightness work ++\n");

	/* notify clients of brightness change */
	bl_evt_data.bl_level = vdd->br.bl_level;
	bl_evt_data.aor_data = vdd->br.aor_data;
	panel_notifier_call_chain(PANEL_EVENT_BL_CHANGED, &bl_evt_data);

	LCD_DEBUG("brightness work --\n");

	return;
}

void ss_panel_init(struct dsi_panel *panel)
{
	struct samsung_display_driver_data *vdd;
	enum ss_display_ndx ndx;
	char panel_name[MAX_CMDLINE_PARAM_LEN];
	char panel_secondary_name[MAX_CMDLINE_PARAM_LEN];

	/* compare panel name in command line and dsi_panel.
	 * primary panel: ndx = 0
	 * secondary panel: ndx = 1
	 */

	LCD_INFO("++ \n");

	ss_get_primary_panel_name_cmdline(panel_name);
	ss_get_secondary_panel_name_cmdline(panel_secondary_name);

	if (!strcmp(panel->name, panel_name)) {
		ndx = PRIMARY_DISPLAY_NDX;
	} else if (!strcmp(panel->name, panel_secondary_name)) {
		ndx = SECONDARY_DISPLAY_NDX;
	} else {
		/* If it fails to find panel name, it cannot turn on display,
		 * and this is critical error case...
		 */
		WARN(1, "fail to find panel name, panel=%s, cmdline=%s\n",
				panel->name, panel_name);
		return;
	}

	/* TODO: after using component_bind in samsung_panel_init,
	 * it doesn't have to use vdd_data..
	 * Remove vdd_data, and allocate vdd memory here.
	 * vdds will be managed by vdds_list...
	 */
	vdd = ss_get_vdd(ndx);

	ss_set_display_ndx(vdd, ndx);

	panel->panel_private = vdd;
	vdd->msm_private = panel;
	list_add(&vdd->vdd_list, &vdds_list);

	if (ss_panel_debug_init(vdd))
		LCD_ERR("Fail to create debugfs\n");

	if (ss_smmu_debug_init(vdd))
		LCD_ERR("Fail to create smmu debug\n");

	mutex_init(&vdd->vdd_lock);
	mutex_init(&vdd->cmd_lock);
	mutex_init(&vdd->bl_lock);
	mutex_init(&vdd->ss_spi_lock);

	/* To guarantee ALPM ON or OFF mode change operation*/
	mutex_init(&vdd->panel_lpm.lpm_lock);

	/* To guarantee dynamic MIPI clock change*/
	mutex_init(&vdd->dyn_mipi_clk.dyn_mipi_lock);

	if (ss_is_cmd_mode(vdd)) {
		vdd->panel_func.ss_event_osc_te_fitting =
			ss_event_osc_te_fitting;
	}

	vdd->panel_func.ss_event_frame_update =
		ss_event_frame_update;
	vdd->panel_func.ss_event_fb_event_callback =
		ss_event_fb_event_callback;
	vdd->panel_func.ss_event_esd_recovery_init =
		ss_event_esd_recovery_init;

	vdd->manufacture_id_dsi = PBA_ID;

	vdd->panel_dead = false;

	if (IS_ERR_OR_NULL(vdd->panel_func.samsung_panel_init))
		LCD_ERR("no samsung_panel_init fucn");
	else
		vdd->panel_func.samsung_panel_init(vdd);

	if ((vdd->mdnie.support_mdnie ||
			vdd->support_cabc) &&
			ss_panel_attached(ndx))
		vdd->mdnie.mdnie_tune_state_dsi =
			init_dsi_tcon_mdnie_class(vdd);
	else
		LCD_INFO("MDNIE is not supported\n");

	spin_lock_init(&vdd->esd_recovery.irq_lock);

	vdd->hmt_stat.hmt_enable = hmt_enable;
	vdd->hmt_stat.hmt_reverse_update = hmt_reverse_update;
	vdd->hmt_stat.hmt_bright_update = hmt_bright_update;

	ss_panel_attach_set(vdd, true);

	INIT_DELAYED_WORK(&vdd->delay_disp_on_work, samsung_display_delay_disp_on_work);

	/* Init Other line panel support */
	if (!IS_ERR_OR_NULL(vdd->panel_func.parsing_otherline_pdata) && ss_panel_attached(vdd->ndx)) {
		if (!IS_ERR_OR_NULL(vdd->panel_func.get_panel_fab_type)) {
			if (vdd->panel_func.get_panel_fab_type() == NEW_FB_PANLE_TYPE) {
				LCD_ERR("parsing_otherline_pdata (%d)\n", vdd->panel_func.get_panel_fab_type());

				INIT_DELAYED_WORK(&vdd->other_line_panel_support_work, read_panel_data_work_fn);
				vdd->other_line_panel_support_workq =
					create_singlethread_workqueue("other_line_panel_support_wq");

				if (vdd->other_line_panel_support_workq) {
					vdd->other_line_panel_work_cnt = OTHERLINE_WORKQ_CNT;
					queue_delayed_work(vdd->other_line_panel_support_workq,
							&vdd->other_line_panel_support_work,
							msecs_to_jiffies(OTHERLINE_WORKQ_DEALY));
				}
			}
		}
	}

	mutex_init(&vdd->exclusive_tx.ex_tx_lock);
	vdd->exclusive_tx.enable = 0;
	init_waitqueue_head(&vdd->exclusive_tx.ex_tx_waitq);

	ss_create_sysfs(vdd);

#if defined(CONFIG_SEC_FACTORY)
	vdd->is_factory_mode = true;
#endif

	/* parse display dtsi node */
	ss_panel_parse_dt(vdd);

	ss_spi_init(vdd);

	ss_dsi_poc_init(vdd);

	if (vdd->self_disp.init)
		vdd->self_disp.init(vdd);

	if (vdd->copr.panel_init)
		vdd->copr.panel_init(vdd);

	/* work thread for brightness change */
	vdd->br.br_wq = create_singlethread_workqueue("brightness_wq");
	if (vdd->br.br_wq == NULL)
		LCD_ERR("failed to create read brightness workqueue..\n");

	INIT_WORK(&vdd->br.br_work, (work_func_t)ss_brightness_work);

	LCD_INFO("-- \n");
}
