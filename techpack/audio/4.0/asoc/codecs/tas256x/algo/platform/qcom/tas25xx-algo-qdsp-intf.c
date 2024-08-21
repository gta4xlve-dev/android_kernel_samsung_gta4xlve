/*
 ** ============================================================================
 ** Copyright (c) 2017  Texas Instruments Inc.
 **
 ** This program is free software; you can redistribute it and/or modify it
 ** under
 ** the terms of the GNU General Public License as published by the Free
 ** Software Foundation; version 2.
 **
 ** This program is distributed in the hope that it will be useful, but WITHOUT
 ** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 ** FITNESS
 ** FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 ** details
 **
 ** File:
 **     tas25xx-algo-qdsp-intf.c
 **
 ** Description:
 **     Qcom interface wrapper.
 **
 ** ============================================================================
 */
#include <linux/types.h>
#include <linux/of.h>
#include <dsp/tas_qualcomm.h>
#include "../../inc/tas_smart_amp_v2.h"

#define AFE_SMARTAMP_MODULE_RX 0x1000FC00
#define AFE_SMARTAMP_MODULE_TX 0x1000FD00
#define AFE_SMARTAMP_DFAULT_PORT 0x9000

static uint16_t g_afe_port_id_rx = 0xFFFF;
static int g_afe_initialized;
static uint32_t s_tisa_module_id[] = {
	AFE_SMARTAMP_MODULE_RX,
	AFE_SMARTAMP_MODULE_TX
};

static void set_port_id_rx(uint16_t port_id)
{
	g_afe_port_id_rx = port_id;
}

static inline uint16_t get_rx_port_id(void)
{
	return g_afe_port_id_rx;
}

static void tas25xx_afe_init(void)
{
	if (g_afe_initialized)
		return;

	afe_tas_smartamp_init(s_tisa_module_id[0], s_tisa_module_id[1]);
	g_afe_initialized = 1;
}

void tas25xx_parse_algo_dt_qdsp_intf(struct device_node *np)
{
	uint32_t port_id = 0;
	uint32_t module_id = 0;
	int32_t ret = 0;

	ret = of_property_read_u32(np, "ti,port_id", &port_id);
	if (!ret) {
		set_port_id_rx((uint16_t)port_id);
		pr_info("[TI-SmartPA:%s] ti,port_id=0x%x", __func__, port_id);
	}

	ret = of_property_read_u32(np, "ti,module_id_rx", &module_id);
	if (!ret) {
		pr_info("[TI-SmartPA:%s] ti,module_id_rx=0x%x", __func__,
			module_id);
		s_tisa_module_id[TISA_MOD_RX] = module_id;
	}

	ret = of_property_read_u32(np, "ti,module_id_tx", &module_id);
	if (!ret) {
		pr_info("[TI-SmartPA:%s] ti,module_id_tx=0x%x", __func__,
			module_id);
		s_tisa_module_id[TISA_MOD_TX] = module_id;
	}
}
EXPORT_SYMBOL(tas25xx_parse_algo_dt_qdsp_intf);

int tas25xx_smartamp_get_set(u8 *user_data, uint32_t param_id,
		uint8_t get_set, uint32_t length, enum module_id_t mod_id_i)
{
	int ret = 0;
	uint32_t module_id;

	if (mod_id_i > 2)
		return -EINVAL;

	module_id = s_tisa_module_id[mod_id_i];

	tas25xx_afe_init();

	switch (get_set) {
	case TAS_SET_PARAM:
		ret = afe_tas_smartamp_set_calib_data(module_id, param_id,
			length, user_data, get_rx_port_id());
		break;

	case TAS_GET_PARAM:
			memset(user_data, 0, length);
		ret = afe_tas_smartamp_get_calib_data(module_id,
			param_id, length, user_data, get_rx_port_id());
		break;

	default:
		break;
	}
	return ret;
}
EXPORT_SYMBOL(tas25xx_smartamp_get_set);

