 /* =================================================================
 *
 *
 *	Description:  samsung display common file
 *
 *	Author:  jb09.kim@samsung.com
 *	Company:  Samsung Electronics
 *
 * ================================================================
 */
/*
<one line to give the program's name and a brief idea of what it does.>
Copyright (C) 2012, Samsung Electronics. All rights reserved.

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
#include "ss_interpolation_common.h"

static long long TOTAL_RESOLUTION;

char ss_dimming_mode_debug[][DIMMING_MODE_DEBUG_STRING] = {
	"SS_FLASH_DIMMING_MODE",
	"SS_S_DIMMING_MODE",
	"SS_S_DIMMING_EXIT_MODE_1",
	"SS_S_DIMMING_EXIT_MODE_2",
	"SS_A_DIMMING_MODE",
	"DIMMING_MODE_MAX",
};

static void init_hbm_interpolation(struct samsung_display_driver_data *vdd,
		struct ss_interpolation_brightness_table *hbm_table, int hbm_table_size)
{
	int loop, column;
	int hbm_interpolation_step;
	int gamma_size = vdd->dtsi_data.gamma_size;
	int irc_size = vdd->dtsi_data.irc_size;
	struct ss_hbm_interpolation *hbm_itp;

	/* update hbm interpolation step */
	for (hbm_interpolation_step =0, loop = 0 ; loop < hbm_table_size; loop++)
		hbm_interpolation_step += hbm_table[loop].steps;

	if (vdd->panel_br_info.itp_mode == FLASH_INTERPOLATION)
		hbm_itp = &vdd->flash_itp.hbm;
	else
		hbm_itp = &vdd->table_itp.hbm;

	hbm_itp->brightness_step = hbm_interpolation_step;

	/*
		free alloc memory : test purpose for slab memory integrity
	*/
	if (!IS_ERR_OR_NULL(hbm_itp->br_table)) {
		kfree(hbm_itp->br_table);
		hbm_itp->br_table = NULL;
	}

	if (!IS_ERR_OR_NULL(hbm_itp->gamma)) {
		for (column = 0; column < hbm_interpolation_step; column++) {
			kfree(hbm_itp->gamma[column]);
			kfree(hbm_itp->irc[column]);
		}

		kfree(hbm_itp->gamma);
		kfree(hbm_itp->irc);

		hbm_itp->gamma = NULL;
		hbm_itp->irc = NULL;
	}

	/*alloc 2 dimension matrix */
	if (!hbm_itp->br_table) {
		hbm_itp->br_table = kzalloc(hbm_interpolation_step * sizeof(struct ss_hbm_interpolation_br_table), GFP_KERNEL);
	}

	if (!hbm_itp->gamma) {
		hbm_itp->gamma = kzalloc(hbm_interpolation_step * sizeof(void *), GFP_KERNEL);
		hbm_itp->irc = kzalloc(hbm_interpolation_step * sizeof(void *), GFP_KERNEL);

		for (column = 0; column < hbm_interpolation_step; column++) {
			hbm_itp->gamma[column] = kzalloc(gamma_size, GFP_KERNEL);
			hbm_itp->irc[column] = kzalloc(irc_size, GFP_KERNEL);
		}
	}
}

static void gen_hbm_interpolation_platform_level_lux_mode(struct samsung_display_driver_data *vdd,
		struct ss_interpolation_brightness_table *hbml_table, int hbm_table_size)
{
	int loop, step, index;
	struct ss_hbm_interpolation *hbm_itp;
	long long platform_up, platform_down, platform_curr, platform_div;
	long long lux_up, lux_down, lux_add;
	long long result1, result2, result3;

	if (vdd->panel_br_info.itp_mode == FLASH_INTERPOLATION)
		hbm_itp = &vdd->flash_itp.hbm;
	else
		hbm_itp = &vdd->table_itp.hbm;

	/* Platform level & lux mode */
	for (index = 0, loop = 0; loop < hbm_table_size; loop++) {
		platform_up = hbml_table[loop].platform;
		lux_up = hbml_table[loop].lux_mode;

		if (loop == 0) {
			platform_down = hbml_table[0].platform;
			lux_add = lux_down = hbml_table[0].lux_mode;

		} else {
			platform_down = hbml_table[loop - 1].platform;
			lux_add = lux_down = hbml_table[loop - 1].lux_mode;
		}

		platform_div = hbml_table[loop].steps;

		platform_up *= MULTIPLY_x10000;
		platform_down *= MULTIPLY_x10000;

		for (step = 0; step < hbml_table[loop].steps; step++) {
			if (step == hbml_table[loop].steps - 1) {
				hbm_itp->br_table[index].platform_level_x10000 =
					hbml_table[loop].platform * MULTIPLY_x10000;

				hbm_itp->br_table[index].interpolation_br_x10000 = hbml_table[loop].lux_mode * MULTIPLY_x10000;
			} else {
				/* Platform level */
				result1 = (((platform_up - platform_down) * (step + 1))/ (platform_div)) + platform_down;

				hbm_itp->br_table[index].platform_level_x10000 =
					(unsigned int)(ROUNDING(result1 / MULTIPLY_x100) * MULTIPLY_x100);


				/* lux level */
				platform_curr = hbm_itp->br_table[index].platform_level_x10000;

				result1 = lux_up - lux_down;
				result2 = (platform_curr - platform_down) / ((platform_up - platform_down) / MULTIPLY_x10000);
				result3 = result1 * result2;

				hbm_itp->br_table[index].interpolation_br_x10000 = ROUNDING(result3) + (lux_add * MULTIPLY_x10000);
			}

			hbm_itp->br_table[index].lux_mode = hbm_itp->br_table[index].interpolation_br_x10000 / MULTIPLY_x10000;

			LCD_DEBUG("Platform : %3d %7d %7d %4d\n",
					index,
					hbm_itp->br_table[index].platform_level_x10000,
					hbm_itp->br_table[index].interpolation_br_x10000,
					hbm_itp->br_table[index].lux_mode);

			index++;
		}
	}
}

/* -1 means hbm_interpolation_table[0] normal max candela index */
#define BR_TABLE_NORMAL_MAX_SUB (1)
#define AUTO_LEVEL_START (6)
static void update_hbm_candela_map_table(struct samsung_display_driver_data *vdd)
{
	struct candela_map_table *table;
	struct ss_hbm_interpolation *hbm_itp;
	int interpolation_step;
	struct ss_hbm_interpolation_br_table *br_table;

	int column;

	if (vdd->panel_br_info.itp_mode == FLASH_INTERPOLATION)
		hbm_itp = &vdd->flash_itp.hbm;
	else
		hbm_itp = &vdd->table_itp.hbm;

	interpolation_step = hbm_itp->brightness_step - BR_TABLE_NORMAL_MAX_SUB;
	br_table = hbm_itp->br_table;

	if (vdd->br.pac)
		table = &vdd->dtsi_data.candela_map_table[PAC_HBM][vdd->panel_revision];
	else
		table = &vdd->dtsi_data.candela_map_table[HBM][vdd->panel_revision];

	/* check table size */
	if (table->tab_size != interpolation_step) {
		LCD_INFO("alloc for pac hbm candela_map_table");

		table->tab_size = interpolation_step;

		/* cd */
		if (!IS_ERR_OR_NULL(table->cd))
			kfree(table->cd);
		table->cd = kzalloc(interpolation_step * sizeof(int), GFP_KERNEL);

		/* idx */
		if (!IS_ERR_OR_NULL(table->idx))
			kfree(table->idx);
		table->idx = kzalloc(interpolation_step * sizeof(int), GFP_KERNEL);

		/* from */
		if (!IS_ERR_OR_NULL(table->from))
			kfree(table->from);
		table->from = kzalloc(interpolation_step * sizeof(int), GFP_KERNEL);

		/* end */
		if (!IS_ERR_OR_NULL(table->end))
			kfree(table->end);
		table->end = kzalloc(interpolation_step * sizeof(int), GFP_KERNEL);

		/* auto_level*/
		if (!IS_ERR_OR_NULL(table->auto_level))
			kfree(table->auto_level);
		table->auto_level = kzalloc(interpolation_step * sizeof(int), GFP_KERNEL);
	}

	/* update table information */
	for (column = 0; column < table->tab_size; column++) {
		/* cd */
		table->cd[column] = br_table[column + BR_TABLE_NORMAL_MAX_SUB].lux_mode;

		/* idx */
		table->idx[column] = column;

		/* from */
		table->from[column] = br_table[column + BR_TABLE_NORMAL_MAX_SUB].platform_level_x10000 / MULTIPLY_x100;

		/* end */
		if (column != table->tab_size - 1)
			table->end[column] = (br_table[column + BR_TABLE_NORMAL_MAX_SUB + 1].platform_level_x10000 / MULTIPLY_x100) - 1;
		else
			table->end[column] = table->from[column] + MULTIPLY_x10000; /* MULTIPLY_x10000 is hard coding */

		/* auto_level */
		table->auto_level[column] = AUTO_LEVEL_START + column;
	}

	/* hbm min_lv */
	table->min_lv = table->from[0];

	/* hbm max_lv */
	table->max_lv = table->end[table->tab_size-1];
		LCD_INFO("table->max_lv = %d\n", table->max_lv);
}


static void update_hbm_interpolation(struct samsung_display_driver_data *vdd,
		struct ss_interpolation_brightness_table *hbm_table, int hbm_table_size)
{
	/* 1st */
	gen_hbm_interpolation_platform_level_lux_mode(vdd, hbm_table, hbm_table_size);

	/* 2st */
	if (vdd->panel_func.gen_hbm_interpolation_gamma)
		vdd->panel_func.gen_hbm_interpolation_gamma(vdd, hbm_table, hbm_table_size);
	else
		LCD_ERR("No gen_hbm_interpolation_gamma !!\n");

	/* 3st */
	if (vdd->panel_func.gen_hbm_interpolation_irc)
		vdd->panel_func.gen_hbm_interpolation_irc(vdd, hbm_table, hbm_table_size);
	else
		LCD_ERR("No gen_hbm_interpolation_irc !!\n");

	/* 4st */
	update_hbm_candela_map_table(vdd);
}

static void init_normal_interpolation(struct samsung_display_driver_data *vdd,
		struct ss_interpolation_brightness_table *normal_table, int normal_table_size)
{
	int loop, column;
	int normal_interpolation_step;
	int irc_size = vdd->dtsi_data.irc_size;
	struct ss_interpolation *ss_itp;

	if (vdd->panel_br_info.itp_mode == FLASH_INTERPOLATION)
		ss_itp = &vdd->flash_itp;
	else
		ss_itp = &vdd->table_itp;

	/* update normal interpolation step */
	for (normal_interpolation_step = 0, loop = 0 ; loop < normal_table_size; loop++)
		normal_interpolation_step += normal_table[loop].steps;

	ss_itp->normal.brightness_step = normal_interpolation_step;

	/*
		free alloc memory : test purpose for slab memory integrity
	*/
	if (!IS_ERR_OR_NULL(ss_itp->normal.br_aor_table)) {
		kfree(ss_itp->normal.br_aor_table);
		ss_itp->normal.br_aor_table = NULL;
	}

	if (!IS_ERR_OR_NULL(ss_itp->normal.irc)) {
		for (column = 0; column < normal_interpolation_step; column++) {
			kfree(ss_itp->normal.irc[column]);
		}
		kfree(ss_itp->normal.irc);
		ss_itp->normal.irc = NULL;
	}

	/* alloc */
	if (!ss_itp->normal.br_aor_table)
		ss_itp->normal.br_aor_table = kzalloc(normal_interpolation_step * sizeof(struct ss_normal_interpolation_br_aor_table), GFP_KERNEL);

	if (!ss_itp->normal.irc) {
		ss_itp->normal.irc = kzalloc(normal_interpolation_step * sizeof(void *), GFP_KERNEL);

		for (column = 0; column < normal_interpolation_step; column++) {
			ss_itp->normal.irc[column] = kzalloc(irc_size, GFP_KERNEL);
		}
	}

	LCD_DEBUG("%pk %d\n", ss_itp->normal.br_aor_table, normal_interpolation_step);
}

static unsigned int A_DIMMING_AOR_CAL(
	long long aor_dec_up_x10000, long long aor_dec_down_x10000,
	long long platform_up_x10000, long long platform_down_x10000, long long platform_curr_x10000)
{
	long long aor_cal;
	unsigned int aor_curr;

	aor_cal = aor_dec_up_x10000 - aor_dec_down_x10000;
	aor_cal *= ((platform_curr_x10000 - platform_down_x10000)  * MULTIPLY_x10000) / (platform_up_x10000 - platform_down_x10000);
	aor_cal = (ROUNDING_NEGATIVE(aor_cal / MULTIPLY_x100) * MULTIPLY_x100) / MULTIPLY_x10000;

	aor_curr = (unsigned int)(aor_cal + aor_dec_down_x10000);
	aor_curr = (ROUNDING(aor_curr) / MULTIPLY_x100) * MULTIPLY_x100;

	return aor_curr;
}

static unsigned int S_DIMMING_AOR_CAL
	(long long aor_dec_up_x10000, long long lux_x10000, long long interpolation_br_x10000)
{
	long long virtual_base_lux;
	long long aor_cal, result1, result2;
	unsigned int aor_curr;

	virtual_base_lux = ((lux_x10000) * ( 100 * MULTIPLY_x10000)) / ((100 * MULTIPLY_x10000) - aor_dec_up_x10000);
	virtual_base_lux = ROUNDING(virtual_base_lux);

	result1 = (1 * MULTIPLY_x100 * MULTIPLY_x100000);
	result2 = (interpolation_br_x10000 * MULTIPLY_x100000) / (virtual_base_lux / MULTIPLY_x100);

	aor_cal = ROUNDING((result1 - result2)/MULTIPLY_x10);

	aor_curr = (unsigned int)(aor_cal);
	aor_curr = (aor_curr / MULTIPLY_x100) * MULTIPLY_x100;

	return aor_curr;
}

static void convert_dec_to_hex_str(unsigned int aor_hex, unsigned int aor_size, unsigned char *buf)
{
	unsigned char temp_buf[AOR_HEX_STRING_CNT];
	int cnt, index;

	memcpy(temp_buf, &aor_hex, AOR_HEX_STRING_CNT);

	for (index = 0, cnt = aor_size - 1; cnt >= 0; cnt--, index++)
		buf[index] = temp_buf[cnt];
}

static void gen_normal_interpolation_platform_level_lux_mode(struct samsung_display_driver_data *vdd,
		struct ss_interpolation_brightness_table *normal_table, int normal_table_size)
{

	int loop, step, index;
	struct ss_normal_interpolation *normal_itp;
	long long platform_up, platform_down, platform_div;

	if (vdd->panel_br_info.itp_mode == FLASH_INTERPOLATION)
		normal_itp = &vdd->flash_itp.normal;
	else
		normal_itp = &vdd->table_itp.normal;

	/* Platform level & lux mode */
	for (index = 0, loop = 0; loop < normal_table_size; loop++) {
		platform_up = normal_table[loop].platform;

		if (loop == 0)
			platform_down = normal_table[0].platform; /* kzalloc used : 0 */
		else
			platform_down = normal_table[loop - 1].platform;

		platform_div = normal_table[loop].steps;

		platform_up *= MULTIPLY_x10000;
		platform_down *= MULTIPLY_x10000;

		for (step = 0; step < normal_table[loop].steps; step++) {
			if (step == normal_table[loop].steps - 1) {
				normal_itp->br_aor_table[index].platform_level_x10000 =
					normal_table[loop].platform * MULTIPLY_x10000;
			} else {
				normal_itp->br_aor_table[index].platform_level_x10000 =
					ROUNDING((((platform_up - platform_down) * (step + 1))/ (platform_div)) + platform_down);
			}

			normal_itp->br_aor_table[index].lux_mode = normal_table[loop].lux_mode;

			LCD_DEBUG("Platform : %d %d %d\n", index, normal_itp->br_aor_table[index].platform_level_x10000, normal_itp->br_aor_table[index].lux_mode);

			index++;
		}
	}
}

static void gen_normal_interpolation_br(struct samsung_display_driver_data *vdd,
		struct ss_interpolation_brightness_table *normal_table, int normal_table_size)
{
	int loop, step, index;
	struct ss_normal_interpolation *normal_itp;
	long long platform_up, platform_down, platform_curr;
	long long lux_up, lux_down, lux_add;
	long long result1, result2, result3, result4;

	if (vdd->panel_br_info.itp_mode == FLASH_INTERPOLATION)
		normal_itp = &vdd->flash_itp.normal;
	else
		normal_itp = &vdd->table_itp.normal;

	/* Interpolation Br */
	for (index = 0, loop = 0; loop < normal_table_size; loop++) {
		lux_up = normal_table[loop].lux_mode;
		platform_up = normal_table[loop].platform;

		if (loop == 0) {
			platform_down = normal_table[0].platform;
			lux_down = normal_table[0].lux_mode;
			lux_add = normal_table[0].lux_mode;
		} else {
			platform_down = normal_table[loop - 1].platform;
			lux_down = normal_table[loop - 1].lux_mode;
			lux_add = normal_table[loop - 1].lux_mode;
		}

		lux_add *= MULTIPLY_x10000;

		platform_up *= MULTIPLY_x10000;
		platform_down *= MULTIPLY_x10000;

		for (step = 0; step < normal_table[loop].steps; step++) {
			platform_curr = normal_itp->br_aor_table[index].platform_level_x10000;

			result1 = (long long)lux_up - (long long)lux_down;

			result2 = (long long)platform_curr - (long long)platform_down;
			result2 *= MULTIPLY_x10000;

			result3 = (long long)platform_up - (long long)platform_down;

			result4 = (result1 * result2) / result3;

			normal_itp->br_aor_table[index].interpolation_br_x10000 = ROUNDING(result4 +lux_add);

			LCD_DEBUG("BR : %d %d\n", index, normal_itp->br_aor_table[index].interpolation_br_x10000);

			index++;
		}
	}
}

static void gen_normal_interpolation_aor(struct samsung_display_driver_data *vdd,
		struct ss_interpolation_brightness_table *normal_table, int normal_table_size)
{

	int loop, reverse_loop, step, index;
	struct ss_normal_interpolation *normal_itp;
	long long platform_up, platform_down, platform_curr;
	long long aor_hex_up, aor_hex_down, aor_hex_next_up, aor_hex_cnt;
	long long aor_dec_up_x10000, aor_dec_down_x10000, aor_dec_curr_x10000;
	enum ss_dimming_mode dimming_mode_curr = DIMMING_MODE_MAX;
	enum ss_dimming_mode s_dimming_step = DIMMING_MODE_MAX;
	unsigned int s_dimming_aor_hex = 0;
	unsigned int aor_size = vdd->dtsi_data.aor_size;

	if (vdd->panel_br_info.itp_mode == FLASH_INTERPOLATION)
		normal_itp = &vdd->flash_itp.normal;
	else
		normal_itp = &vdd->table_itp.normal;

	/* AOR */
	for (index = 0, loop = 0, reverse_loop = normal_table_size - 1;
		loop < normal_table_size; loop++, reverse_loop--) {
		platform_up = normal_table[loop].platform;

		/* use reverse_loop by flash aor data ordering */
		for (aor_hex_up = 0, aor_hex_cnt = 0; aor_hex_cnt < aor_size; aor_hex_cnt++) {
			aor_hex_up <<= (0x08 * aor_hex_cnt);
			aor_hex_up |= vdd->panel_br_info.normal.aor[reverse_loop][aor_hex_cnt];
		}
		aor_dec_up_x10000 = AOR_HEX_TO_PERCENT_X10000(aor_hex_up);

		if (loop == 0) {
			platform_down = normal_table[0].platform;
			aor_hex_down = aor_hex_up;
		} else {
			platform_down = normal_table[loop - 1].platform;
			aor_hex_down = normal_itp->br_aor_table[index - 1].aor_hex;
		}
		aor_dec_down_x10000 = AOR_HEX_TO_PERCENT_X10000(aor_hex_down);

		/* use reverse_loop by flash aor data ordering */
		if (reverse_loop < 1)
			aor_hex_next_up = aor_hex_up;
		else {
			for (aor_hex_next_up = 0, aor_hex_cnt = 0; aor_hex_cnt < aor_size; aor_hex_cnt++) {
				aor_hex_next_up <<= (0x08 * aor_hex_cnt);
				aor_hex_next_up |= vdd->panel_br_info.normal.aor[reverse_loop - 1][aor_hex_cnt];
			}
		}

		platform_up *= MULTIPLY_x10000;
		platform_down *= MULTIPLY_x10000;

		/*
			dimming mdoe check is consist of 2 step.
			dimming mode check 1 is for check sw calculation.
			dimming mode check 2 is for flash nand real data.
		*/
		/* dimming mode check 1 */
		if (aor_hex_up == aor_hex_next_up)	{
			dimming_mode_curr = SS_S_DIMMING_MODE;
			s_dimming_step = SS_S_DIMMING_MODE;
			s_dimming_aor_hex = aor_hex_up;
		} else if ((s_dimming_step == SS_S_DIMMING_MODE) && \
				((aor_hex_up == s_dimming_aor_hex) && (aor_hex_next_up != s_dimming_aor_hex))) {
			dimming_mode_curr = SS_S_DIMMING_MODE;
			s_dimming_step = SS_S_DIMMING_EXIT_MODE_1;
		} else if ((s_dimming_step == SS_S_DIMMING_EXIT_MODE_1) && \
				((aor_hex_up != s_dimming_aor_hex) && (aor_hex_next_up != s_dimming_aor_hex))) {
			/* SS_S_DIMMING_EXIT_MODE_2 is real exit for S_DIMMING */
			dimming_mode_curr = SS_S_DIMMING_MODE;

			/* reset flags after SS_S_DIMMING_EXIT_MODE_2 */
			s_dimming_aor_hex = 0;
			s_dimming_step = DIMMING_MODE_MAX;
		} else {
			dimming_mode_curr = SS_A_DIMMING_MODE;
		}

		for (step = 0; step < normal_table[loop].steps; step++) {
			platform_curr = normal_itp->br_aor_table[index].platform_level_x10000;

			/* dimming mode check 2 */
			if (step == normal_table[loop].steps - 1) {
				dimming_mode_curr = SS_FLASH_DIMMING_MODE;
			}

			normal_itp->br_aor_table[index].dimming_mode = dimming_mode_curr;

			if (dimming_mode_curr == SS_FLASH_DIMMING_MODE) {
				/* FLASH_DIMMING */
				aor_dec_curr_x10000 = aor_dec_up_x10000;
			} else if (dimming_mode_curr == SS_S_DIMMING_MODE) {
				/* S_DIMMING */
				aor_dec_curr_x10000 = S_DIMMING_AOR_CAL (
						aor_dec_up_x10000,
						(long long)(normal_itp->br_aor_table[index].lux_mode * MULTIPLY_x10000),
						(long long)(normal_itp->br_aor_table[index].interpolation_br_x10000));
			} else {
				/* A_DIMMING */
				aor_dec_curr_x10000 = A_DIMMING_AOR_CAL (
						aor_dec_up_x10000, aor_dec_down_x10000,
						platform_up, platform_down, platform_curr);
			}

			normal_itp->br_aor_table[index].aor_percent_x10000 = aor_dec_curr_x10000;
			normal_itp->br_aor_table[index].aor_hex = AOR_PERCENT_X1000_TO_HEX(aor_dec_curr_x10000);

			/* To convert dec to hex string format */
			convert_dec_to_hex_str(
				normal_itp->br_aor_table[index].aor_hex,
				aor_size,
				normal_itp->br_aor_table[index].aor_hex_string);

			LCD_DEBUG("AOR index : %3d  hex: 0x%04x percent_x10000: %6d mode : %s\n", index,
				normal_itp->br_aor_table[index].aor_hex,
				normal_itp->br_aor_table[index].aor_percent_x10000,
				ss_dimming_mode_debug[dimming_mode_curr]);

			index++;
		}
	}
}

static void update_candela_map_table(struct samsung_display_driver_data *vdd)
{
	struct candela_map_table *table;
	int interpolation_step;
	struct ss_normal_interpolation_br_aor_table *br_aor_table;
	struct ss_normal_interpolation *norma_itp;

	int column;
	int pre_lux = -1, lux_idx = -1;

	if (vdd->panel_br_info.itp_mode == FLASH_INTERPOLATION)
		norma_itp = &vdd->flash_itp.normal;
	else
		norma_itp = &vdd->table_itp.normal;

	interpolation_step = norma_itp->brightness_step;
	br_aor_table = norma_itp->br_aor_table;

	if (vdd->br.pac)
		table = &vdd->dtsi_data.candela_map_table[PAC_NORMAL][vdd->panel_revision];
	else
		table = &vdd->dtsi_data.candela_map_table[NORMAL][vdd->panel_revision];

	/* check table size */
	if (table->tab_size != interpolation_step) {
		LCD_INFO("alloc for pac_candela_map_table");

		table->tab_size = interpolation_step;
		/* scaled_idx */
		if (!IS_ERR_OR_NULL(table->scaled_idx))
			kfree(table->scaled_idx);
		table->scaled_idx = kzalloc(interpolation_step * sizeof(int), GFP_KERNEL);

		/* idx */
		if (!IS_ERR_OR_NULL(table->idx))
			kfree(table->idx);
		table->idx = kzalloc(interpolation_step * sizeof(int), GFP_KERNEL);

		/* from */
		if (!IS_ERR_OR_NULL(table->from))
			kfree(table->from);
		table->from = kzalloc(interpolation_step * sizeof(int), GFP_KERNEL);

		/* end */
		if (!IS_ERR_OR_NULL(table->end))
			kfree(table->end);
		table->end = kzalloc(interpolation_step * sizeof(int), GFP_KERNEL);

		/* cd */
		if (!IS_ERR_OR_NULL(table->cd))
			kfree(table->cd);
		table->cd = kzalloc(interpolation_step * sizeof(int), GFP_KERNEL);

		/* interpolation_cd (upscale not rounding)*/
		if (!IS_ERR_OR_NULL(table->interpolation_cd))
			kfree(table->interpolation_cd);
		table->interpolation_cd = kzalloc(interpolation_step * sizeof(int), GFP_KERNEL);
	}

	/* update table information */
	for (column = 0; column < table->tab_size; column++) {
		/* scaled_idx */
		table->scaled_idx[column] = column;

		/* idx */
		if (pre_lux != br_aor_table[column].lux_mode) {
			pre_lux = br_aor_table[column].lux_mode;
			lux_idx++;
		}
		table->idx[column] = lux_idx;

		/* from */
		if (column == 0)
			table->from[column] = 0;
		else
			table->from[column] = table->end[column - 1] + 1;

		/* end */
		table->end[column] = br_aor_table[column].platform_level_x10000 / MULTIPLY_x100;

		/* cd */
		table->cd[column] = br_aor_table[column].lux_mode;

		/* interpolation_cd (upscale not rounding)*/
		table->interpolation_cd[column] = (br_aor_table[column].interpolation_br_x10000 + (MULTIPLY_x10000 - 1)) / MULTIPLY_x10000;
	}

	/* min_lv */
	table->min_lv = table->from[0];
	/* max_lv */
	table->max_lv = table->end[table->tab_size-1];
	LCD_INFO("table->max_lv = %d\n", table->max_lv);
}

static void update_normal_interpolation(struct samsung_display_driver_data *vdd,
		struct ss_interpolation_brightness_table *normal_table, int normal_table_size)
{
	/* 1st */
	gen_normal_interpolation_platform_level_lux_mode(vdd, normal_table, normal_table_size);

	/* 2st */
	gen_normal_interpolation_br(vdd, normal_table, normal_table_size);

	/* 3st */
	gen_normal_interpolation_aor(vdd, normal_table, normal_table_size);

	/* 4st */
	if (vdd->panel_func.gen_normal_interpolation_irc)
		vdd->panel_func.gen_normal_interpolation_irc(vdd, normal_table, normal_table_size);
	else
		LCD_ERR("No gen_normal_interpolation_irc !!\n");

	/* 5st */
	update_candela_map_table(vdd);
}


void set_up_interpolation(struct samsung_display_driver_data *vdd,
	struct ss_interpolation_brightness_table *normal_table, int normal_table_size,
	struct ss_interpolation_brightness_table *hbm_table, int hbm_table_size)

{
	TOTAL_RESOLUTION = vdd->panel_br_info.vfp + vdd->panel_br_info.vbp + vdd->panel_br_info.resolution;

	/* init samsung normal interpolation data */
	init_normal_interpolation(vdd, normal_table, normal_table_size);
	update_normal_interpolation(vdd, normal_table, normal_table_size);

	/* init samsung hbm interpolation data */
	init_hbm_interpolation(vdd, hbm_table, hbm_table_size);
	update_hbm_interpolation(vdd, hbm_table, hbm_table_size);
}

static int find_hbm_candela(struct samsung_display_driver_data *vdd)
{
	int hbm_brightness_step = vdd->dtsi_data.hbm_brightness_step;
	int index = -1;
	int candela = vdd->br.cd_level;
	int loop;

	for(loop = 0; loop < hbm_brightness_step; loop++)
		if (candela == vdd->panel_br_info.hbm.candela_table[loop]) {
			index = loop;
			break;
		}

	/* find the high bound closed index */
	if (index < 0) {
		for(loop = hbm_brightness_step - 1; loop >= 0; loop--)
			if (vdd->panel_br_info.hbm.candela_table[loop] - candela >= 0) {
				LCD_DEBUG("index : %d lux : %d vdd_lux : %d\n", loop, vdd->panel_br_info.hbm.candela_table[loop], candela);
				index = loop;
				break;
			}
	}

	if (index < 0) {
		LCD_INFO("fail to find %d candela at hbm.candela_table\n", candela);
	}

	return index;
}

static int find_normal_candela(struct samsung_display_driver_data *vdd)
{
	int normal_brightness_step = vdd->dtsi_data.normal_brightness_step;
	int index = -1;
	int candela = vdd->br.cd_level;
	int loop;

	for(loop = 0; loop < normal_brightness_step; loop++)
		if (candela == vdd->panel_br_info.normal.candela_table[loop]) {
			index = loop;
			break;
		}

	/* find the high bound closed index */
	if (index < 0) {
		for(loop = normal_brightness_step - 1; loop >= 0; loop--)
			if (vdd->panel_br_info.normal.candela_table[loop] - candela >= 0) {
				LCD_DEBUG("index : %d lux : %d vdd_lux : %d\n", loop, vdd->panel_br_info.normal.candela_table[loop], candela);
				index = loop;
				break;
			}
	}

	if (index < 0) {
		LCD_INFO("fail to find %d candela at normal.candela_table\n", candela);
	}

	return index;
}

static int find_hmd_candela(struct samsung_display_driver_data *vdd)
{
	int hmd_brightness_step = vdd->dtsi_data.hmd_brightness_step;
	int index = -1;
	int candela = vdd->br.interpolation_cd;
	int loop;

	for(loop = 0; loop < hmd_brightness_step; loop++)
		if (candela == vdd->panel_br_info.hmd.candela_table[loop]) {
			index = loop;
			break;
		}

	if (index < 0) {
		LCD_INFO("fail to find %d candela at hmd.candela_table\n", candela);
	}

	return index;
}

static int find_hbm_interpolation_candela(struct samsung_display_driver_data *vdd)
{
	struct ss_interpolation *ss_itp;
	int hbm_interpolation_brightness_step;
	int index = -1;
	int candela = vdd->br.cd_level;
	int loop;

	if (vdd->panel_br_info.itp_mode == FLASH_INTERPOLATION) {
		ss_itp = &vdd->flash_itp;
	} else {
		ss_itp = &vdd->table_itp;
	}

	hbm_interpolation_brightness_step = ss_itp->hbm.brightness_step;

	for (loop = 0; loop < hbm_interpolation_brightness_step; loop++)
		if (candela == ss_itp->hbm.br_table[loop].lux_mode) {
			index = loop;
			break;
		}

	if (index < 0) {
		LCD_INFO("fail to find %d candela at interpolation.hbm.candela_table\n", candela);
	}

	return index;
}

void copy_cmd_debug(char *debug_str, char *cmds, int cmd_size)
{
	int data_cnt;
	char buf[FLASH_GAMMA_DBG_BUF_SIZE];

	memset(buf, '\n', sizeof(buf));

	snprintf(buf, FLASH_GAMMA_DBG_BUF_SIZE, "%s : ", debug_str);

	for (data_cnt = 0; data_cnt < cmd_size; data_cnt++)
		snprintf(buf + strlen(buf), FLASH_GAMMA_DBG_BUF_SIZE - strlen(buf),
				"%02x ", cmds[data_cnt]);

	LCD_INFO("%s\n", buf);

}

int br_interpolation_generate_event(struct samsung_display_driver_data *vdd, enum GEN_INTERPOLATION_EVENT event, char *buf)
{
	int candela_index = -1;
	int gamma_size = vdd->dtsi_data.gamma_size;
	int elvss_size = vdd->dtsi_data.elvss_size / INTERPOLATION_ELVSS_MAX_TEMP;
	int vint_size = vdd->dtsi_data.vint_size;
	int aor_size = vdd->dtsi_data.aor_size;
	int irc_size = vdd->dtsi_data.irc_size;
	struct ss_interpolation *ss_itp;

	if (vdd->panel_br_info.itp_mode == FLASH_INTERPOLATION) {
		ss_itp = &vdd->flash_itp;
	} else {
		ss_itp = &vdd->table_itp;
	}

	switch (event) {
	case GEN_HBM_GAMMA:
		LCD_INFO("not support event=%d\n", event);
		break;
	case GEN_NORMAL_GAMMA:
	case GEN_NORMAL_INTERPOLATION_GAMMA:
		candela_index = find_normal_candela(vdd);
		if (candela_index >= 0)
			memcpy(buf, vdd->panel_br_info.normal.gamma[candela_index], gamma_size);  // table need to copy
		break;
	case GEN_HMD_GAMMA:
		candela_index = find_hmd_candela(vdd);
		if (candela_index >= 0)
			memcpy(buf, vdd->panel_br_info.hmd.gamma[candela_index], gamma_size); // table need to copy
		break;
	case GEN_HBM_INTERPOLATION_GAMMA:
		candela_index = find_hbm_interpolation_candela(vdd);
		if (candela_index >= 0)
			memcpy(buf, ss_itp->hbm.gamma[candela_index], gamma_size);
		break;
	case GEN_HBM_AOR:
		LCD_INFO("not support event=%d\n", event);
		break;
	case GEN_NORMAL_AOR:
		LCD_INFO("not support event=%d\n", event);
		break;
	case GEN_HMD_AOR:
		candela_index = find_hmd_candela(vdd);
		if (candela_index >= 0)
			memcpy(buf, vdd->panel_br_info.hmd.aor[candela_index], aor_size);
		break;
	case GEN_NORMAL_INTERPOLATION_AOR:
		memcpy(buf, ss_itp->normal.br_aor_table[vdd->br.pac_cd_idx].aor_hex_string, aor_size);
		vdd->br.aor_data = ss_itp->normal.br_aor_table[vdd->br.pac_cd_idx].aor_percent_x10000 / 100;
		break;
	case GEN_HBM_INTERPOLATION_AOR:
		candela_index = find_hbm_candela(vdd);
		if (candela_index >= 0)
			memcpy(buf, vdd->panel_br_info.hbm.aor[candela_index], aor_size);
		break;
	case GEN_HBM_VINT:
		LCD_INFO("not support event=%d\n", event);
		break;
	case GEN_NORMAL_VINT:
		LCD_INFO("not support event=%d\n", event);
		break;
	case GEN_HMD_VINT:
		LCD_INFO("not support event=%d\n", event);
		break;
	case GEN_NORMAL_INTERPOLATION_VINT:
		candela_index = find_normal_candela(vdd);
		if (candela_index >= 0)
			memcpy(buf, vdd->panel_br_info.normal.vint[candela_index], vint_size);
		break;
	case GEN_HBM_INTERPOLATION_VINT:
		candela_index = find_hbm_candela(vdd);
		if (candela_index >= 0)
			memcpy(buf, vdd->panel_br_info.hbm.vint[candela_index], vint_size);
		break;
	case GEN_HBM_ELVSS:
		LCD_INFO("not support event=%d\n", event);
		break;
	case GEN_NORMAL_ELVSS:
		LCD_INFO("not support event=%d\n", event);
		break;
	case GEN_HMD_ELVSS:
		LCD_INFO("not support event=%d\n", event);
		break;
	case GEN_NORMAL_INTERPOLATION_ELVSS:
		candela_index = find_normal_candela(vdd);
		if (candela_index >= 0) {
			if (vdd->temperature > 0) {
				memcpy(buf, &vdd->panel_br_info.normal.elvss[candela_index][INTERPOLATION_ELVSS_HIGH_TEMP], elvss_size);
			}else if (vdd->temperature > vdd->br.elvss_interpolation_temperature) {
				memcpy(buf, &vdd->panel_br_info.normal.elvss[candela_index][INTERPOLATION_ELVSS_MID_TEMP], elvss_size);
			} else {
				memcpy(buf, &vdd->panel_br_info.normal.elvss[candela_index][INTERPOLATION_ELVSS_LOW_TEMP], elvss_size);
			}
		}
		break;
	case GEN_HBM_INTERPOLATION_ELVSS:
		candela_index = find_hbm_candela(vdd);
		if (candela_index >= 0) {
			if (vdd->temperature > 0) {
				memcpy(buf, &vdd->panel_br_info.hbm.elvss[candela_index][INTERPOLATION_ELVSS_HIGH_TEMP], elvss_size);
			}else if (vdd->temperature > vdd->br.elvss_interpolation_temperature) {
				memcpy(buf, &vdd->panel_br_info.hbm.elvss[candela_index][INTERPOLATION_ELVSS_MID_TEMP], elvss_size);
			} else {
				memcpy(buf, &vdd->panel_br_info.hbm.elvss[candela_index][INTERPOLATION_ELVSS_LOW_TEMP], elvss_size);
			}
		}
		break;
	case GEN_HBM_IRC:
		LCD_INFO("not support event=%d\n", event);
		break;
	case GEN_NORMAL_IRC:
		LCD_INFO("not support event=%d\n", event);
		break;
	case GEN_HMD_IRC:
		LCD_INFO("not support event=%d\n", event);
		break;
	case GEN_NORMAL_INTERPOLATION_IRC:
		memcpy(buf, ss_itp->normal.irc[vdd->br.pac_cd_idx], irc_size);
		break;
	case GEN_HBM_INTERPOLATION_IRC:
		candela_index = find_hbm_interpolation_candela(vdd);
		if (candela_index >= 0)
			memcpy(buf, ss_itp->hbm.irc[candela_index], irc_size);

		//copy_cmd_debug("GEN_HBM_INTERPOLATION_IRC", buf, irc_size);
		break;
	default:
		LCD_INFO("unhandled event=%d\n", event);
		break;
	}

	LCD_DEBUG("event=%d candela_index:%d\n", event, candela_index);

	return candela_index;
}

static void debug_normal_interpolation(struct samsung_display_driver_data *vdd)
{
	char buf[FLASH_GAMMA_DBG_BUF_SIZE];
	struct ss_normal_interpolation *normal_itp;
	int column, brightness_step, data_cnt;

	int irc_size = vdd->dtsi_data.irc_size;
	unsigned char **irc;

	struct candela_map_table *table;

	if (vdd->panel_br_info.itp_mode == FLASH_INTERPOLATION)
		normal_itp = &vdd->flash_itp.normal;
	else
		normal_itp = &vdd->table_itp.normal;

	brightness_step = normal_itp->brightness_step;
	irc = normal_itp->irc;

	for (column = 0; column < brightness_step; column++) {
		LCD_INFO("index: %3d Platform_x1000: %7d lux_mode: %3d BR_x1000: %7d AOR_x10000: %7d 0x%04X dimmng: %s\n",
				column,
				normal_itp->br_aor_table[column].platform_level_x10000,
				normal_itp->br_aor_table[column].lux_mode,
				normal_itp->br_aor_table[column].interpolation_br_x10000,
				normal_itp->br_aor_table[column].aor_percent_x10000,
				normal_itp->br_aor_table[column].aor_hex,
				ss_dimming_mode_debug[normal_itp->br_aor_table[column].dimming_mode]);
	}

	memset(buf, '\n', sizeof(buf));

	/* IRC */
	for (column = brightness_step - 1; column >= 0; column--) {
		if (!IS_ERR_OR_NULL(irc)) {
			snprintf(buf, FLASH_GAMMA_DBG_BUF_SIZE, "irc [%4d][%7d]: ", column + 1, normal_itp->br_aor_table[column].interpolation_br_x10000);
			for (data_cnt = 0; data_cnt < irc_size; data_cnt++)
				snprintf(buf + strlen(buf), FLASH_GAMMA_DBG_BUF_SIZE - strlen(buf), "%02X ", irc[column][data_cnt]);
			LCD_INFO("%s\n", buf);
			memset(buf, '\n', strlen(buf));
		}
	}

	/* candela_map_table debug */
	if (vdd->br.pac)
		table = &vdd->dtsi_data.candela_map_table[PAC_NORMAL][vdd->panel_revision];
	else
		table = &vdd->dtsi_data.candela_map_table[NORMAL][vdd->panel_revision];

	LCD_INFO("/* <scaled idx> <idx>  <from>  <end> <cd> <interpolation cd>*/\n");

	for (column =  0; column < table->tab_size; column++) {
		LCD_INFO("%4d %2d %5d %5d %3d %3d\n",
			table->scaled_idx[column],
			table->idx[column],
			table->from[column],
			table->end[column],
			table->cd[column],
			table->interpolation_cd[column]);
	}
}

static void debug_hbm_interpolation(struct samsung_display_driver_data *vdd)
{
	char buf[FLASH_GAMMA_DBG_BUF_SIZE];
	struct ss_hbm_interpolation *hbm_itp;
	int column, data_cnt;

	int brightness_step;

	int gamma_size = vdd->dtsi_data.gamma_size;
	unsigned char **gamma;

	int irc_size = vdd->dtsi_data.irc_size;
	unsigned char **irc;

	struct candela_map_table *table;

	memset(buf, '\n', sizeof(buf));

	if (vdd->panel_br_info.itp_mode == FLASH_INTERPOLATION)
		hbm_itp = &vdd->flash_itp.hbm;
	else
		hbm_itp = &vdd->table_itp.hbm;

	brightness_step = hbm_itp->brightness_step;
	gamma = hbm_itp->gamma;
	irc = hbm_itp->irc;

	/* Platform Level, Candela */
	for (column =  0; column < brightness_step; column++) {
		LCD_INFO("index: %3d Platform_x10000: %7d lux_mode_x10000: %3d\n",
			column,
			hbm_itp->br_table[column].platform_level_x10000,
			hbm_itp->br_table[column].interpolation_br_x10000);
	}

	/* GAMMA */
	for (column = brightness_step - 1; column >= 0; column--) {
		if (!IS_ERR_OR_NULL(gamma)) {
			snprintf(buf, FLASH_GAMMA_DBG_BUF_SIZE, "gamma [%2d][%d] : ", column, hbm_itp->br_table[column].lux_mode);
			for (data_cnt = 0; data_cnt < gamma_size; data_cnt++)
				snprintf(buf + strlen(buf), FLASH_GAMMA_DBG_BUF_SIZE - strlen(buf), "%02x ", gamma[column][data_cnt]);
			LCD_INFO("%s\n", buf);
			memset(buf, '\n', strlen(buf));
		}
	}

	/* IRC */
	for (column = brightness_step - 1; column >= 0; column--) {
		/* IRC */
		if (!IS_ERR_OR_NULL(irc)) {
			snprintf(buf, FLASH_GAMMA_DBG_BUF_SIZE, "irc [%2d][%d] : ", column, hbm_itp->br_table[column].lux_mode);
			for (data_cnt = 0; data_cnt < irc_size; data_cnt++)
				snprintf(buf + strlen(buf), FLASH_GAMMA_DBG_BUF_SIZE - strlen(buf), "%02X ", irc[column][data_cnt]);
			LCD_INFO("%s\n", buf);
			memset(buf, '\n', strlen(buf));
		}
	}

	/* candela_map_table */
	if (vdd->br.pac)
		table = &vdd->dtsi_data.candela_map_table[PAC_HBM][vdd->panel_revision];
	else
		table = &vdd->dtsi_data.candela_map_table[HBM][vdd->panel_revision];

	LCD_INFO("< idx from end cd auto >\n");

	for (column =  0; column < table->tab_size; column++) {
		LCD_INFO("%2d %d %d %d %2d\n",
			table->idx[column],
			table->from[column],
			table->end[column],
			table->cd[column],
			table->auto_level[column]);
	}
}

void debug_interpolation_log(struct samsung_display_driver_data *vdd)
{
	debug_normal_interpolation(vdd);
	debug_hbm_interpolation(vdd);
}

