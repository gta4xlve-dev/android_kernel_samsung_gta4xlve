/*
 * ALSA SoC - Samsung Adaptation driver
 *
 * Copyright (c) 2016 Samsung Electronics Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SEC_ADAPTATION_H
#define __SEC_ADAPTATION_H

#include <dsp/q6afe-v2.h>
#include <dsp/q6asm-v2.h>
#include <dsp/q6adm-v2.h>

int q6audio_get_afe_cal_validation(u16 port_id, u32 topology_id);
/****************************************************************************/
/*//////////////////////////// AUDIO SOLUTION //////////////////////////////*/
/****************************************************************************/
#define MODULE_ID_PP_SS_REC             0x10001050
#define PARAM_ID_PP_SS_REC_GETPARAMS    0x10001052

#define MODULE_ID_PP_SA                 0x10001fa0
#define PARAM_ID_PP_SA_PARAMS           0x10001fa1

#define MODULE_ID_PP_SA_VSP             0x10001fb0
#define PARAM_ID_PP_SA_VSP_PARAMS       0x10001fb1

#define MODULE_ID_PP_ADAPTATION_SOUND		 0x10001fc0
#define PARAM_ID_PP_ADAPTATION_SOUND_PARAMS  0x10001fc1

#define MODULE_ID_PP_LRSM               0x10001fe0
#define PARAM_ID_PP_LRSM_PARAMS         0x10001fe1

#define MODULE_ID_PP_SA_MSP             0x10001ff0
#define MODULE_ID_PP_SA_MSP_PARAM       0x10001ff1

#define MODULE_ID_PP_SB                 0x10001f01
#define PARAM_ID_PP_SB_PARAM            0x10001f04
#define PARAM_ID_PP_SB_ROTATION_PARAM	0x10001f02

#define MODULE_ID_PP_SA_UPSCALER_COLOR            0x10001f20
#define PARAM_ID_PP_SA_UPSCALER_COLOR_PARAMS      0x10001f21

#define MODULE_ID_PP_DOLBY_DAP 0x10001fd0
#define PARAM_ID_PP_DOLBY_DAP_PARAMS 0x10001fd1 

struct asm_stream_cmd_set_pp_params_sa {
	int16_t OutDevice;
	int16_t Preset;
	int32_t EqLev[9];
	int16_t m3Dlevel;
	int16_t BElevel;
	int16_t CHlevel;
	int16_t CHRoomSize;
	int16_t Clalevel;
	int16_t volume;
	int16_t Sqrow;
	int16_t Sqcol;
	int16_t TabInfo;
	int16_t NewUI;
	int16_t m3DPositionOn;
	int16_t reserved;
	int32_t m3DPositionAngle[2];
	int32_t m3DPositionGain[2];
	int32_t AHDRonoff;
} __packed;

struct asm_stream_cmd_set_pp_params_vsp {
	uint32_t speed_int;
} __packed;

struct asm_stream_cmd_set_pp_params_adaptation_sound {
	int32_t enable;
	int16_t gain[2][6];
	int16_t device;
} __packed;

struct asm_stream_cmd_set_pp_params_lrsm {
	int16_t sm;
	int16_t lr;
} __packed;

struct asm_stream_cmd_set_pp_params_msp {
	uint32_t msp_int;
} __packed;

struct adm_param_soundbooster_t {
	uint32_t sb_enable;
} __packed;

struct asm_stream_cmd_set_pp_params_upscaler {
	uint32_t upscaler_enable;
} __packed;

struct adm_param_sb_rotation {
	uint32_t sb_rotation;
} __packed;

struct asm_stream_cmd_set_pp_params_dolby_atmos {
	uint32_t enable;
	int16_t device;
	int16_t dolby_profile;
} __packed;

/****************************************************************************/
/*//////////////////////////// VOICE SOLUTION //////////////////////////////*/
/****************************************************************************/
/* NXP */
#define VPM_TX_SM_LVVEFQ_COPP_TOPOLOGY      0x1000BFF0
#define VPM_TX_DM_LVVEFQ_COPP_TOPOLOGY      0x1000BFF1
#define VPM_TX_QM_LVVEFQ_COPP_TOPOLOGY      0x1000BFF3
#define VPM_TX_SM_LVSAFQ_COPP_TOPOLOGY      0x1000B200
#define VPM_TX_DM_LVSAFQ_COPP_TOPOLOGY      0x1000B201

/* Fotemeia */
#define VOICE_TX_DIAMONDVOICE_FVSAM_SM      0x1000110B
#define VOICE_TX_DIAMONDVOICE_FVSAM_DM      0x1000110A
#define VOICE_TX_DIAMONDVOICE_FVSAM_QM      0x10001109
#define VOICE_TX_DIAMONDVOICE_FRSAM_DM      0x1000110C

/* Solomon Voice */
#define VOICE_TX_SOLOMONVOICE_SM            0x100010AA
#define VOICE_TX_SOLOMONVOICE_DM            0x100010AB
#define VOICE_TX_SOLOMONVOICE_QM            0x100010AC

#define VOICEPROC_MODULE_VENC				0x00010F07
#define VOICE_PARAM_LOOPBACK_ENABLE			0x00010E18
/* Rx */
#define VOICE_VOICEMODE_MODULE				0x10001001
#define VOICE_ADAPTATION_SOUND_PARAM        0x10001022
/* Tx */
#define VOICE_WISEVOICE_MODULE				0x10001031
#define VOICE_FVSAM_MODULE					0x10001041

#define VOICE_NBMODE_PARAM					0x10001023
#define VOICE_SPKMODE_PARAM					0x10001025
#define VOICE_RCVMODE_PARAM					0x10001027

#define VOICE_MODULE_LVVEFQ_TX              0x1000B500
#define TX_VOICE_SOLOMONVOICE               0x100010A0
#define VOICE_ECHO_REF_LCH_MUTE_PARAM       0x10001028
#define VOICE_NREC_MODE_DYNAMIC_PARAM       0x10001029

#define VOICE_MODULE_SET_DEVICE				0x10041000
#define VOICE_MODULE_SET_DEVICE_PARAM		0x10041001

struct vss_icommon_cmd_set_ui_property_v2_t {
	uint32_t module_id;
	uint16_t instance_id;
	uint16_t reserved;
	uint32_t param_id;
	uint32_t param_size;
	uint16_t enable;
	uint16_t reserved_field;
};

struct vss_icommon_cmd_set_loopback_enable_t {
	uint32_t module_id;
	/* Unique ID of the module. */
	uint32_t param_id;
	/* Unique ID of the parameter. */
	uint16_t param_size;
	/* Size of the parameter in bytes: MOD_ENABLE_PARAM_LEN */
	uint16_t reserved;
	/* Reserved; set to 0. */
	uint16_t loopback_enable;
	uint16_t reserved_field;
	/* Reserved, set to 0. */
};

struct cvs_set_loopback_enable_cmd {
	struct apr_hdr hdr;
	uint32_t mem_handle;
	uint32_t mem_address_lsw;
	uint32_t mem_address_msw;
	uint32_t mem_size;
	struct vss_icommon_cmd_set_loopback_enable_t vss_set_loopback;
} __packed;

struct cvp_adaptation_sound_parm_send_t {
	uint32_t module_id;
	/* Unique ID of the module. */
	uint32_t param_id;
	/* Unique ID of the parameter. */
	uint16_t param_size;
	/* Size of the parameter in bytes: MOD_ENABLE_PARAM_LEN */
	uint16_t reserved;
	/* Reserved; set to 0. */
	uint16_t eq_mode;
	uint16_t select;
	int16_t param[12];
} __packed;

struct cvp_adaptation_sound_parm_send_cmd {
	struct apr_hdr hdr;
	uint32_t mem_handle;
	uint32_t mem_address_lsw;
	uint32_t mem_address_msw;
	uint32_t mem_size;
	struct cvp_adaptation_sound_parm_send_t adaptation_sound_data;
} __packed;

struct cvp_set_nbmode_enable_cmd {
	struct apr_hdr hdr;
	struct vss_icommon_cmd_set_ui_property_v2_t cvp_set_nbmode;
} __packed;

struct cvp_set_rcvmode_enable_cmd {
	struct apr_hdr hdr;
	struct vss_icommon_cmd_set_ui_property_v2_t cvp_set_rcvmode;
} __packed;

struct cvp_set_spkmode_enable_cmd {
	struct apr_hdr hdr;
	struct vss_icommon_cmd_set_ui_property_v2_t cvp_set_spkmode;
} __packed;

struct cvp_set_device_info_cmd {
	struct apr_hdr hdr;
	struct vss_icommon_cmd_set_ui_property_v2_t cvp_set_device_info;
} __packed;

struct cvp_set_ref_lch_mute_enable_cmd {
	struct apr_hdr hdr;
	struct vss_icommon_cmd_set_ui_property_v2_t cvp_set_ref_lch_mute;
} __packed;

struct cvp_set_aec_effect_cmd {
	struct apr_hdr hdr;
	struct vss_icommon_cmd_set_ui_property_v2_t cvp_set_aec_effect;
} __packed;

void voice_sec_loopback_start_cmd(u32 session_id);
void voice_sec_loopback_end_cmd(u32 session_id);

#endif /* __SEC_ADAPTATION_H */
