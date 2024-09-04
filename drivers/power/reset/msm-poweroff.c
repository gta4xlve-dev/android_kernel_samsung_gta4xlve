/* Copyright (c) 2013-2020, The Linux Foundation. All rights reserved.
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/input/qpnp-power-on.h>
#include <linux/of_address.h>
#include <linux/syscore_ops.h>
#include <linux/crash_dump.h>

#include <asm/cacheflush.h>
#include <asm/system_misc.h>
#include <asm/memory.h>

#include <soc/qcom/scm.h>
#include <soc/qcom/restart.h>
#include <soc/qcom/watchdog.h>
#include <soc/qcom/minidump.h>

#define EMERGENCY_DLOAD_MAGIC1    0x322A4F99
#define EMERGENCY_DLOAD_MAGIC2    0xC67E4350
#define EMERGENCY_DLOAD_MAGIC3    0x77777777
#define EMMC_DLOAD_TYPE		0x2

#define SCM_IO_DISABLE_PMIC_ARBITER	1
#define SCM_IO_DEASSERT_PS_HOLD		2
#define SCM_WDOG_DEBUG_BOOT_PART	0x9
#define SCM_DLOAD_FULLDUMP		0X10
#define SCM_EDLOAD_MODE			0X01
#define SCM_DLOAD_CMD			0x10
#define SCM_DLOAD_MINIDUMP		0X20
#define SCM_DLOAD_BOTHDUMPS	(SCM_DLOAD_MINIDUMP | SCM_DLOAD_FULLDUMP)

#define PON_RESTART_REASON_NOT_HANDLE	PON_RESTART_REASON_MAX
#define RESTART_REASON_NOT_HANDLE	RESTART_REASON_END

#define KERNEL_SEC_DEBUG_LEVEL_LOW	(0x574F4C44)
#define KERNEL_SEC_DEBUG_LEVEL_MID	(0x44494D44)
#define KERNEL_SEC_DEBUG_LEVEL_HIGH	(0x47494844)

#define ANDROID_DEBUG_LEVEL_LOW		0x4f4c
#define ANDROID_DEBUG_LEVEL_MID		0x494d
#define ANDROID_DEBUG_LEVEL_HIGH	0x4948

#define ANDROID_CP_DEBUG_ON		0x5500
#define ANDROID_CP_DEBUG_OFF		0x55ff

#define DUMP_SINK_TO_SDCARD 0x73646364
#define DUMP_SINK_TO_BOOTDEV 0x42544456
#define DUMP_SINK_TO_USB 0x0

#define CDSP_SIGNOFF_BLOCK 0x2377
#define CDSP_SIGNOFF_ON 0x7277

enum sec_restart_reason_t {
	RESTART_REASON_NORMAL = 0x0,
	RESTART_REASON_BOOTLOADER = 0x77665500,
	RESTART_REASON_REBOOT = 0x77665501,
	RESTART_REASON_RECOVERY = 0x77665502,
	RESTART_REASON_RTC = 0x77665503,
	RESTART_REASON_DMVERITY_CORRUPTED = 0x77665508,
	RESTART_REASON_DMVERITY_ENFORCE = 0x77665509,
	RESTART_REASON_KEYS_CLEAR = 0x7766550a,
	RESTART_REASON_SEC_DEBUG_MODE = 0x776655ee,
	RESTART_REASON_RECOVERY_UPDATE = 0x776655cc,
	RESTART_REASON_END = 0xffffffff,
};

static enum sec_restart_reason_t __iomem *qcom_restart_reason;
void (*sec_nvmem_pon_write)(u8 pon_rr) = NULL;

static int restart_mode;
static void *restart_reason;
static bool scm_pmic_arbiter_disable_supported;
static bool scm_deassert_ps_hold_supported;
/* Download mode master kill-switch */
static void __iomem *msm_ps_hold;
static phys_addr_t tcsr_boot_misc_detect;
static void scm_disable_sdi(void);

/*
 * Runtime could be only changed value once.
 * There is no API from TZ to re-enable the registers.
 * So the SDI cannot be re-enabled when it already by-passed.
 */
static int download_mode = 1;
static bool force_warm_reboot;

#ifdef CONFIG_QCOM_DLOAD_MODE
#define EDL_MODE_PROP "qcom,msm-imem-emergency_download_mode"
#define DL_MODE_PROP "qcom,msm-imem-download_mode"
#ifdef CONFIG_RANDOMIZE_BASE
#define KASLR_OFFSET_PROP "qcom,msm-imem-kaslr_offset"
#endif

static int in_panic;
static struct kobject dload_kobj;
static int dload_type = SCM_DLOAD_FULLDUMP;
static void *dload_mode_addr;
static void *dload_type_addr;
static bool dload_mode_enabled;
static void *emergency_dload_mode_addr;
#ifdef CONFIG_RANDOMIZE_BASE
static void __iomem *kaslr_imem_addr;
#endif
static bool scm_dload_supported;

static int dload_set(const char *val, const struct kernel_param *kp);
/* interface for exporting attributes */
struct reset_attribute {
	struct attribute        attr;
	ssize_t (*show)(struct kobject *kobj, struct attribute *attr,
			char *buf);
	size_t (*store)(struct kobject *kobj, struct attribute *attr,
			const char *buf, size_t count);
};
#define to_reset_attr(_attr) \
	container_of(_attr, struct reset_attribute, attr)
#define RESET_ATTR(_name, _mode, _show, _store)	\
	static struct reset_attribute reset_attr_##_name = \
			__ATTR(_name, _mode, _show, _store)

module_param_call(download_mode, dload_set, param_get_int,
			&download_mode, 0644);

static int panic_prep_restart(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	in_panic = 1;
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call	= panic_prep_restart,
};

int scm_set_dload_mode(int arg1, int arg2)
{
	struct scm_desc desc = {
		.args[0] = arg1,
		.args[1] = arg2,
		.arginfo = SCM_ARGS(2),
	};

	if (!scm_dload_supported) {
		if (tcsr_boot_misc_detect)
			return scm_io_write(tcsr_boot_misc_detect, arg1);

		return 0;
	}
	if (!is_scm_armv8())
		return scm_call_atomic2(SCM_SVC_BOOT, SCM_DLOAD_CMD, arg1,
					arg2);

	return scm_call2_atomic(SCM_SIP_FNID(SCM_SVC_BOOT, SCM_DLOAD_CMD),
				&desc);
}

static void set_dload_mode(int on)
{
	int ret;

	if (dload_mode_addr) {
		__raw_writel(on ? 0xE47B337D : 0, dload_mode_addr);
		__raw_writel(on ? 0xCE14091A : 0,
		       dload_mode_addr + sizeof(unsigned int));
		/* Make sure the download cookie is updated */
		mb();
	}

	ret = scm_set_dload_mode(on ? dload_type : 0, 0);
	if (ret)
		pr_err("Failed to set secure DLOAD mode: %d\n", ret);

	dload_mode_enabled = on;
}

static bool get_dload_mode(void)
{
	return dload_mode_enabled;
}

static void enable_emergency_dload_mode(void)
{
	int ret;

	if (emergency_dload_mode_addr) {
		__raw_writel(EMERGENCY_DLOAD_MAGIC1,
				emergency_dload_mode_addr);
		__raw_writel(EMERGENCY_DLOAD_MAGIC2,
				emergency_dload_mode_addr +
				sizeof(unsigned int));
		__raw_writel(EMERGENCY_DLOAD_MAGIC3,
				emergency_dload_mode_addr +
				(2 * sizeof(unsigned int)));

		/* Need disable the pmic wdt, then the emergency dload mode
		 * will not auto reset.
		 */
		qpnp_pon_wd_config(0);
		/* Make sure all the cookied are flushed to memory */
		mb();
	}

	ret = scm_set_dload_mode(SCM_EDLOAD_MODE, 0);
	if (ret)
		pr_err("Failed to set secure EDLOAD mode: %d\n", ret);
}

static int dload_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	int old_val = download_mode;

	if (!download_mode) {
		pr_err("Error: SDI dynamic enablement is not supported\n");
		return -EINVAL;
	}

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	/* If download_mode is not zero or one, ignore. */
	if (download_mode >> 1) {
		download_mode = old_val;
		return -EINVAL;
	}

	set_dload_mode(download_mode);

	if (!download_mode)
		scm_disable_sdi();

	return 0;
}
#else
static void set_dload_mode(int on)
{
	return;
}

static void enable_emergency_dload_mode(void)
{
	pr_err("dload mode is not enabled on target\n");
}

static bool get_dload_mode(void)
{
	return false;
}
#endif

static void scm_disable_sdi(void)
{
	int ret;
	struct scm_desc desc = {
		.args[0] = 1,
		.args[1] = 0,
		.arginfo = SCM_ARGS(2),
	};

	/* Needed to bypass debug image on some chips */
	if (!is_scm_armv8())
		ret = scm_call_atomic2(SCM_SVC_BOOT,
			SCM_WDOG_DEBUG_BOOT_PART, 1, 0);
	else
		ret = scm_call2_atomic(SCM_SIP_FNID(SCM_SVC_BOOT,
			  SCM_WDOG_DEBUG_BOOT_PART), &desc);
	if (ret)
		pr_err("Failed to disable secure wdog debug: %d\n", ret);
}

void msm_set_restart_mode(int mode)
{
	restart_mode = mode;
}
EXPORT_SYMBOL(msm_set_restart_mode);

/*
 * Force the SPMI PMIC arbiter to shutdown so that no more SPMI transactions
 * are sent from the MSM to the PMIC.  This is required in order to avoid an
 * SPMI lockup on certain PMIC chips if PS_HOLD is lowered in the middle of
 * an SPMI transaction.
 */
static void halt_spmi_pmic_arbiter(void)
{
	struct scm_desc desc = {
		.args[0] = 0,
		.arginfo = SCM_ARGS(1),
	};

	if (scm_pmic_arbiter_disable_supported) {
		pr_crit("Calling SCM to disable SPMI PMIC arbiter\n");
		if (!is_scm_armv8())
			scm_call_atomic1(SCM_SVC_PWR,
					SCM_IO_DISABLE_PMIC_ARBITER, 0);
		else
			scm_call2_atomic(SCM_SIP_FNID(SCM_SVC_PWR,
				SCM_IO_DISABLE_PMIC_ARBITER), &desc);
	}
}

static inline void __sec_debug_set_restart_reason(
				enum sec_restart_reason_t __r)
{
	__raw_writel((u32)__r, qcom_restart_reason);
}

static enum pon_restart_reason __pon_restart_rory_start(
				unsigned long opt_code)
{
	return (PON_RESTART_REASON_RORY_START | opt_code);
}

static enum pon_restart_reason __pon_restart_set_debug_level(
				unsigned long opt_code)
{
	switch (opt_code) {
	case ANDROID_DEBUG_LEVEL_LOW:
		return PON_RESTART_REASON_DBG_LOW;
	case ANDROID_DEBUG_LEVEL_MID:
		return PON_RESTART_REASON_DBG_MID;
	case ANDROID_DEBUG_LEVEL_HIGH:
		return PON_RESTART_REASON_DBG_HIGH;
	}

	return PON_RESTART_REASON_UNKNOWN;
}

static enum pon_restart_reason __pon_restart_set_cpdebug(
				unsigned long opt_code)
{
	if (opt_code == ANDROID_CP_DEBUG_ON)
		return PON_RESTART_REASON_CP_DBG_ON;
	else if (opt_code == ANDROID_CP_DEBUG_OFF)
		return PON_RESTART_REASON_CP_DBG_OFF;

	return PON_RESTART_REASON_UNKNOWN;
}

static enum pon_restart_reason __pon_restart_force_upload(
				unsigned long opt_code)
{
	return (opt_code) ?
		PON_RESTART_REASON_FORCE_UPLOAD_ON :
		PON_RESTART_REASON_FORCE_UPLOAD_OFF;
}

static enum pon_restart_reason __pon_restart_dump_sink(
				unsigned long opt_code)
{
	switch (opt_code) {
	case DUMP_SINK_TO_BOOTDEV:
		return PON_RESTART_REASON_DUMP_SINK_BOOTDEV;
	case DUMP_SINK_TO_SDCARD:
		return PON_RESTART_REASON_DUMP_SINK_SDCARD;
	default:
		return PON_RESTART_REASON_DUMP_SINK_USB;
	}

	return PON_RESTART_REASON_UNKNOWN;
}

static enum pon_restart_reason __pon_restart_cdsp_signoff(
				unsigned long opt_code)
{
	switch (opt_code) {
	case CDSP_SIGNOFF_ON:
		return PON_RESTART_REASON_CDSP_ON;
	case CDSP_SIGNOFF_BLOCK:
		return PON_RESTART_REASON_CDSP_BLOCK;
	}

	return PON_RESTART_REASON_UNKNOWN;
}

static bool sec_check_leave_reboot_reason(enum pon_restart_reason pon_rr)
{
	bool result = false;

	result = (pon_rr == PON_RESTART_REASON_MULTICMD) || result;
	result = (pon_rr == PON_RESTART_REASON_LIMITED_DRAM_SETTING) || result;

	return result;
}

void sec_debug_update_restart_reason(const char *cmd, const int in_panic,
		const int restart_mode)
{
	struct __magic {
		const char *cmd;
		enum pon_restart_reason pon_rr;
		enum sec_restart_reason_t sec_rr;
		enum pon_restart_reason (*func)(unsigned long opt_code);
	} magic[] = {
		{ "sec_debug_hw_reset",
			PON_RESTART_REASON_NOT_HANDLE,
			RESTART_REASON_SEC_DEBUG_MODE, NULL},
		{ "download",
			PON_RESTART_REASON_DOWNLOAD,
			RESTART_REASON_NOT_HANDLE, NULL},
		{ "nvbackup",
			PON_RESTART_REASON_NVBACKUP,
			RESTART_REASON_NOT_HANDLE, NULL},
		{ "nvrestore",
			PON_RESTART_REASON_NVRESTORE,
			RESTART_REASON_NOT_HANDLE, NULL},
		{ "nverase",
			PON_RESTART_REASON_NVERASE,
			RESTART_REASON_NOT_HANDLE, NULL},
		{ "nvrecovery",
			PON_RESTART_REASON_NVRECOVERY,
			RESTART_REASON_NOT_HANDLE, NULL},
		{ "cpmem_on",
			PON_RESTART_REASON_CP_MEM_RESERVE_ON,
			RESTART_REASON_NOT_HANDLE, NULL},
		{ "cpmem_off",
			PON_RESTART_REASON_CP_MEM_RESERVE_OFF,
			RESTART_REASON_NOT_HANDLE, NULL},
		{ "mbsmem_on",
			PON_RESTART_REASON_MBS_MEM_RESERVE_ON,
			RESTART_REASON_NOT_HANDLE, NULL},
		{ "mbsmem_off",
			PON_RESTART_REASON_MBS_MEM_RESERVE_OFF,
			RESTART_REASON_NOT_HANDLE, NULL},
		{ "GlobalActions restart",
			PON_RESTART_REASON_NORMALBOOT,
			RESTART_REASON_NOT_HANDLE, NULL},
		{ "userrequested",
			PON_RESTART_REASON_NORMALBOOT,
			RESTART_REASON_NOT_HANDLE, NULL},
		{ "silent.sec",
			PON_RESTART_REASON_NORMALBOOT,
			RESTART_REASON_NOT_HANDLE, NULL },
		{ "oem-",
			PON_RESTART_REASON_UNKNOWN,
			RESTART_REASON_NORMAL, NULL},
		{ "sud",
			PON_RESTART_REASON_UNKNOWN,
			RESTART_REASON_NORMAL, __pon_restart_rory_start},
		{ "debug",
			PON_RESTART_REASON_UNKNOWN,
			RESTART_REASON_NORMAL, __pon_restart_set_debug_level},
		{ "cpdebug",
			PON_RESTART_REASON_UNKNOWN,
			RESTART_REASON_NORMAL, __pon_restart_set_cpdebug},
		{ "forceupload",
			PON_RESTART_REASON_UNKNOWN,
			RESTART_REASON_NORMAL, __pon_restart_force_upload},
		{ "dump_sink",
			PON_RESTART_REASON_UNKNOWN,
			RESTART_REASON_NORMAL, __pon_restart_dump_sink},
		{ "signoff",
			PON_RESTART_REASON_UNKNOWN,
			RESTART_REASON_NORMAL, __pon_restart_cdsp_signoff},
		{ "cross_fail",
			PON_RESTART_REASON_CROSS_FAIL,
			RESTART_REASON_NORMAL, NULL},
		{ "from_fastboot",
			PON_RESTART_REASON_NORMALBOOT,
			RESTART_REASON_NOT_HANDLE, NULL},
		{ "disallow,fastboot",
			PON_RESTART_REASON_NORMALBOOT,
			RESTART_REASON_NOT_HANDLE, NULL},
#ifdef CONFIG_MUIC_SUPPORT_RUSTPROOF
		{ "swsel",
			PON_RESTART_REASON_UNKNOWN,
			RESTART_REASON_NORMAL, __pon_restart_swsel},
#endif
#ifdef CONFIG_SEC_PERIPHERAL_SECURE_CHK
		{ "peripheral_hw_reset",
			PON_RESTART_REASON_SECURE_CHECK_FAIL,
			RESTART_REASON_NORMAL, NULL},
#endif
#ifdef CONFIG_SEC_QUEST_UEFI_USER
		{ "user_quefi_quest",
			PON_RESTART_REASON_QUEST_QUEFI_USER_START,
			RESTART_REASON_NORMAL, NULL},
		{ "user_suefi_quest",
			PON_RESTART_REASON_QUEST_SUEFI_USER_START,
			RESTART_REASON_NORMAL, NULL},
#endif
#ifdef CONFIG_SEC_QUEST_DDR_SCAN_USER
		{ "user_dram_test",
			PON_RESTART_REASON_USER_DRAM_TEST,
			RESTART_REASON_NORMAL, NULL},
#endif
		{ "multicmd",
			PON_RESTART_REASON_MULTICMD,
			RESTART_REASON_NORMAL, NULL},
		{ "dram",
			PON_RESTART_REASON_LIMITED_DRAM_SETTING,
			RESTART_REASON_NORMAL, NULL}
	};
	enum pon_restart_reason pon_rr = PON_RESTART_REASON_UNKNOWN;
	enum sec_restart_reason_t sec_rr = RESTART_REASON_NORMAL;
	char cmd_buf[256];
	size_t i;

	if (!in_panic && restart_mode != RESTART_DLOAD)
		pon_rr = PON_RESTART_REASON_NORMALBOOT;

#ifndef CONFIG_SEC_LOG_STORE_LAST_KMSG
__next_cmd:
#endif
	if (!cmd)
		pr_err("(%s) reboot cmd : NULL\n", __func__);
	else if (!strlen(cmd))
		pr_err("(%s) reboot cmd : Zero Length\n", __func__);
	else {
		strlcpy(cmd_buf, cmd, ARRAY_SIZE(cmd_buf));
		pr_err("(%s) reboot cmd : %s\n", __func__, cmd_buf);
	}

	if (!cmd || !strlen(cmd) || !strncmp(cmd, "adb", 3))
		goto __done;

	for (i = 0; i < ARRAY_SIZE(magic); i++) {
		size_t len = strlen(magic[i].cmd);

		if (strncmp(cmd, magic[i].cmd, len))
			continue;

#ifndef CONFIG_SEC_LOG_STORE_LAST_KMSG
		if (sec_check_leave_reboot_reason(magic[i].pon_rr)) {
			cmd = cmd + len + 1;
			goto __next_cmd;
		}
#endif
		pon_rr = magic[i].pon_rr;
		sec_rr = magic[i].sec_rr;

		if (magic[i].func != NULL) {
			unsigned long opt_code;
			char *ptr = strstr(cmd_buf, ":");

			if (ptr != NULL)
				*ptr = '\0';

			if (!kstrtoul(cmd_buf + len, 0, &opt_code))
				pon_rr = magic[i].func(opt_code);
		}

		goto __done;
	}

	__sec_debug_set_restart_reason(RESTART_REASON_REBOOT);

	return;

__done:
	if (pon_rr != PON_RESTART_REASON_NOT_HANDLE) {
		if (sec_nvmem_pon_write)
			sec_nvmem_pon_write(pon_rr);
		else
			qpnp_pon_set_restart_reason(pon_rr);
	}
	if (sec_rr != RESTART_REASON_NOT_HANDLE)
		__sec_debug_set_restart_reason(sec_rr);
	pr_err("(%s) restart_reason = 0x%x(0x%x)\n",
	       __func__, sec_rr, pon_rr);
}

static void msm_restart_prepare(const char *cmd)
{
	bool need_warm_reset = false;
#ifdef CONFIG_QCOM_DLOAD_MODE
	/* Write download mode flags if we're panic'ing
	 * Write download mode flags if restart_mode says so
	 * Kill download mode if master-kill switch is set
	 */
	if (!is_kdump_kernel())
		set_dload_mode(download_mode &&
			(in_panic || restart_mode == RESTART_DLOAD));
#endif

	if (qpnp_pon_check_hard_reset_stored()) {
		/* Set warm reset as true when device is in dload mode */
		if (get_dload_mode() ||
			((cmd != NULL && cmd[0] != '\0') &&
			!strcmp(cmd, "edl")))
			need_warm_reset = true;
	} else {
		need_warm_reset = (get_dload_mode() ||
				(cmd != NULL && cmd[0] != '\0'));
	}

	if (force_warm_reboot)
		pr_info("Forcing a warm reset of the system\n");

	/* Hard reset the PMIC unless memory contents must be maintained. */
	if (force_warm_reboot || need_warm_reset)
		qpnp_pon_system_pwr_off(PON_POWER_OFF_WARM_RESET);
	else
		qpnp_pon_system_pwr_off(PON_POWER_OFF_HARD_RESET);

	if (cmd != NULL) {
		if (!strncmp(cmd, "bootloader", 10)) {
			qpnp_pon_set_restart_reason(
				PON_RESTART_REASON_BOOTLOADER);
			__raw_writel(0x77665500, restart_reason);
		} else if (!strncmp(cmd, "recovery", 8)) {
			qpnp_pon_set_restart_reason(
				PON_RESTART_REASON_RECOVERY);
			__raw_writel(0x77665502, restart_reason);
		} else if (!strcmp(cmd, "rtc")) {
			qpnp_pon_set_restart_reason(
				PON_RESTART_REASON_RTC);
			__raw_writel(0x77665503, restart_reason);
		} else if (!strcmp(cmd, "dm-verity device corrupted")) {
			qpnp_pon_set_restart_reason(
				PON_RESTART_REASON_DMVERITY_CORRUPTED);
			__raw_writel(0x77665508, restart_reason);
		} else if (!strcmp(cmd, "dm-verity enforcing")) {
			qpnp_pon_set_restart_reason(
				PON_RESTART_REASON_DMVERITY_ENFORCE);
			__raw_writel(0x77665509, restart_reason);
		} else if (!strcmp(cmd, "keys clear")) {
			qpnp_pon_set_restart_reason(
				PON_RESTART_REASON_KEYS_CLEAR);
			__raw_writel(0x7766550a, restart_reason);
		} else if (!strncmp(cmd, "oem-", 4)) {
			unsigned long code;
			int ret;

			ret = kstrtoul(cmd + 4, 16, &code);
			if (!ret)
				__raw_writel(0x6f656d00 | (code & 0xff),
					     restart_reason);
		} else if (!strncmp(cmd, "edl", 3)) {
			enable_emergency_dload_mode();
		} else {
			__raw_writel(0x77665501, restart_reason);
		}
	}

	sec_debug_update_restart_reason(cmd, in_panic, restart_mode);
	flush_cache_all();

	/*outer_flush_all is not supported by 64bit kernel*/
#ifndef CONFIG_ARM64
	outer_flush_all();
#endif

}

/*
 * Deassert PS_HOLD to signal the PMIC that we are ready to power down or reset.
 * Do this by calling into the secure environment, if available, or by directly
 * writing to a hardware register.
 *
 * This function should never return.
 */
static void deassert_ps_hold(void)
{
	struct scm_desc desc = {
		.args[0] = 0,
		.arginfo = SCM_ARGS(1),
	};

	if (scm_deassert_ps_hold_supported) {
		/* This call will be available on ARMv8 only */
		scm_call2_atomic(SCM_SIP_FNID(SCM_SVC_PWR,
				 SCM_IO_DEASSERT_PS_HOLD), &desc);
	}

	/* Fall-through to the direct write in case the scm_call "returns" */
	__raw_writel(0, msm_ps_hold);
}

static void do_msm_restart(enum reboot_mode reboot_mode, const char *cmd)
{
	pr_notice("Going down for restart now\n");

	msm_restart_prepare(cmd);

#ifdef CONFIG_QCOM_DLOAD_MODE
	/*
	 * Trigger a watchdog bite here and if this fails,
	 * device will take the usual restart path.
	 */

	if (WDOG_BITE_ON_PANIC && in_panic)
		msm_trigger_wdog_bite();
#endif

	scm_disable_sdi();
	halt_spmi_pmic_arbiter();
	deassert_ps_hold();

	msleep(10000);
}

static void do_msm_poweroff(void)
{
	pr_notice("Powering off the SoC\n");

	set_dload_mode(0);
	scm_disable_sdi();
	qpnp_pon_system_pwr_off(PON_POWER_OFF_SHUTDOWN);

	halt_spmi_pmic_arbiter();
	deassert_ps_hold();

	msleep(10000);
	pr_err("Powering off has failed\n");
}

#ifdef CONFIG_QCOM_DLOAD_MODE
static ssize_t attr_show(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	struct reset_attribute *reset_attr = to_reset_attr(attr);
	ssize_t ret = -EIO;

	if (reset_attr->show)
		ret = reset_attr->show(kobj, attr, buf);

	return ret;
}

static ssize_t attr_store(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	struct reset_attribute *reset_attr = to_reset_attr(attr);
	ssize_t ret = -EIO;

	if (reset_attr->store)
		ret = reset_attr->store(kobj, attr, buf, count);

	return ret;
}

static const struct sysfs_ops reset_sysfs_ops = {
	.show	= attr_show,
	.store	= attr_store,
};

static struct kobj_type reset_ktype = {
	.sysfs_ops	= &reset_sysfs_ops,
};

static ssize_t show_emmc_dload(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	uint32_t read_val, show_val;

	if (!dload_type_addr)
		return -ENODEV;

	read_val = __raw_readl(dload_type_addr);
	if (read_val == EMMC_DLOAD_TYPE)
		show_val = 1;
	else
		show_val = 0;

	return snprintf(buf, sizeof(show_val), "%u\n", show_val);
}

static size_t store_emmc_dload(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	uint32_t enabled;
	int ret;

	if (!dload_type_addr)
		return -ENODEV;

	ret = kstrtouint(buf, 0, &enabled);
	if (ret < 0)
		return ret;

	if (!((enabled == 0) || (enabled == 1)))
		return -EINVAL;

	if (enabled == 1)
		__raw_writel(EMMC_DLOAD_TYPE, dload_type_addr);
	else
		__raw_writel(0, dload_type_addr);

	return count;
}

#ifdef CONFIG_QCOM_MINIDUMP
static DEFINE_MUTEX(tcsr_lock);

static ssize_t show_dload_mode(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "DLOAD dump type: %s\n",
		(dload_type == SCM_DLOAD_BOTHDUMPS) ? "both" :
		((dload_type == SCM_DLOAD_MINIDUMP) ? "mini" : "full"));
}

static size_t store_dload_mode(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	if (sysfs_streq(buf, "full")) {
		dload_type = SCM_DLOAD_FULLDUMP;
	} else if (sysfs_streq(buf, "mini")) {
		if (!msm_minidump_enabled()) {
			pr_err("Minidump is not enabled\n");
			return -ENODEV;
		}
		dload_type = SCM_DLOAD_MINIDUMP;
	} else if (sysfs_streq(buf, "both")) {
		if (!msm_minidump_enabled()) {
			pr_err("Minidump not enabled, setting fulldump only\n");
			dload_type = SCM_DLOAD_FULLDUMP;
			return count;
		}
		dload_type = SCM_DLOAD_BOTHDUMPS;
	} else{
		pr_err("Invalid Dump setup request..\n");
		pr_err("Supported dumps:'full', 'mini', or 'both'\n");
		return -EINVAL;
	}

	mutex_lock(&tcsr_lock);
	/*Overwrite TCSR reg*/
	set_dload_mode(dload_type);
	mutex_unlock(&tcsr_lock);
	return count;
}
RESET_ATTR(dload_mode, 0644, show_dload_mode, store_dload_mode);
#endif

RESET_ATTR(emmc_dload, 0644, show_emmc_dload, store_emmc_dload);

static struct attribute *reset_attrs[] = {
	&reset_attr_emmc_dload.attr,
#ifdef CONFIG_QCOM_MINIDUMP
	&reset_attr_dload_mode.attr,
#endif
	NULL
};

static struct attribute_group reset_attr_group = {
	.attrs = reset_attrs,
};
#endif

#if defined(CONFIG_RANDOMIZE_BASE) && defined(CONFIG_HIBERNATION)
static void msm_poweroff_syscore_resume(void)
{
#define KASLR_OFFSET_BIT_MASK	0x00000000FFFFFFFF
	if (kaslr_imem_addr) {
		__raw_writel(0xdead4ead, kaslr_imem_addr);
		__raw_writel(KASLR_OFFSET_BIT_MASK &
		(kimage_vaddr - KIMAGE_VADDR), kaslr_imem_addr + 4);
		__raw_writel(KASLR_OFFSET_BIT_MASK &
			((kimage_vaddr - KIMAGE_VADDR) >> 32),
			kaslr_imem_addr + 8);
	}
}

static struct syscore_ops msm_poweroff_syscore_ops = {
	.resume = msm_poweroff_syscore_resume,
};
#endif

static int msm_restart_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *mem;
	struct device_node *np;
	int ret = 0;

#ifdef CONFIG_QCOM_DLOAD_MODE
	if (scm_is_call_available(SCM_SVC_BOOT, SCM_DLOAD_CMD) > 0)
		scm_dload_supported = true;

	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
	np = of_find_compatible_node(NULL, NULL, DL_MODE_PROP);
	if (!np) {
		pr_err("unable to find DT imem DLOAD mode node\n");
	} else {
		dload_mode_addr = of_iomap(np, 0);
		if (!dload_mode_addr)
			pr_err("unable to map imem DLOAD offset\n");
	}

	np = of_find_compatible_node(NULL, NULL, EDL_MODE_PROP);
	if (!np) {
		pr_err("unable to find DT imem EDLOAD mode node\n");
	} else {
		emergency_dload_mode_addr = of_iomap(np, 0);
		if (!emergency_dload_mode_addr)
			pr_err("unable to map imem EDLOAD mode offset\n");
	}

#ifdef CONFIG_RANDOMIZE_BASE
#define KASLR_OFFSET_BIT_MASK	0x00000000FFFFFFFF
	np = of_find_compatible_node(NULL, NULL, KASLR_OFFSET_PROP);
	if (!np) {
		pr_err("unable to find DT imem KASLR_OFFSET node\n");
	} else {
		kaslr_imem_addr = of_iomap(np, 0);
		if (!kaslr_imem_addr)
			pr_err("unable to map imem KASLR offset\n");
	}

	if (kaslr_imem_addr) {
		__raw_writel(0xdead4ead, kaslr_imem_addr);
		__raw_writel(KASLR_OFFSET_BIT_MASK &
		(kimage_vaddr - KIMAGE_VADDR), kaslr_imem_addr + 4);
		__raw_writel(KASLR_OFFSET_BIT_MASK &
			((kimage_vaddr - KIMAGE_VADDR) >> 32),
			kaslr_imem_addr + 8);
	}

#ifdef CONFIG_HIBERNATION
	register_syscore_ops(&msm_poweroff_syscore_ops);
#endif
#endif
	np = of_find_compatible_node(NULL, NULL,
				"qcom,msm-imem-dload-type");
	if (!np) {
		pr_err("unable to find DT imem dload-type node\n");
		goto skip_sysfs_create;
	} else {
		dload_type_addr = of_iomap(np, 0);
		if (!dload_type_addr) {
			pr_err("unable to map imem dload-type offset\n");
			goto skip_sysfs_create;
		}
	}

	ret = kobject_init_and_add(&dload_kobj, &reset_ktype,
			kernel_kobj, "%s", "dload");
	if (ret) {
		pr_err("%s:Error in creation kobject_add\n", __func__);
		kobject_put(&dload_kobj);
		goto skip_sysfs_create;
	}

	ret = sysfs_create_group(&dload_kobj, &reset_attr_group);
	if (ret) {
		pr_err("%s:Error in creation sysfs_create_group\n", __func__);
		kobject_del(&dload_kobj);
	}
skip_sysfs_create:
#endif
	np = of_find_compatible_node(NULL, NULL,
				"qcom,msm-imem-restart_reason");
	if (!np) {
		pr_err("unable to find DT imem restart reason node\n");
	} else {
		restart_reason = of_iomap(np, 0);
		if (!restart_reason) {
			pr_err("unable to map imem restart reason offset\n");
			ret = -ENOMEM;
			goto err_restart_reason;
		}
	}

	mem = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pshold-base");
	msm_ps_hold = devm_ioremap_resource(dev, mem);
	if (IS_ERR(msm_ps_hold))
		return PTR_ERR(msm_ps_hold);

	mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "tcsr-boot-misc-detect");
	if (mem)
		tcsr_boot_misc_detect = mem->start;

	pm_power_off = do_msm_poweroff;
	arm_pm_restart = do_msm_restart;

	if (scm_is_call_available(SCM_SVC_PWR, SCM_IO_DISABLE_PMIC_ARBITER) > 0)
		scm_pmic_arbiter_disable_supported = true;

	if (scm_is_call_available(SCM_SVC_PWR, SCM_IO_DEASSERT_PS_HOLD) > 0)
		scm_deassert_ps_hold_supported = true;
	if (!is_kdump_kernel())
		set_dload_mode(download_mode);
	if (!download_mode)
		scm_disable_sdi();

	force_warm_reboot = of_property_read_bool(dev->of_node,
						"qcom,force-warm-reboot");

	qcom_restart_reason = of_iomap(np, 0);
	if (unlikely(!qcom_restart_reason)) {
		pr_err("unable to map imem restart reason offset\n");
		return -ENODEV;
	}

	return 0;

err_restart_reason:
#ifdef CONFIG_QCOM_DLOAD_MODE
	iounmap(emergency_dload_mode_addr);
	iounmap(dload_mode_addr);
#ifdef CONFIG_RANDOMIZE_BASE
	iounmap(kaslr_imem_addr);
#endif
#endif
	return ret;
}

static const struct of_device_id of_msm_restart_match[] = {
	{ .compatible = "qcom,pshold", },
	{},
};
MODULE_DEVICE_TABLE(of, of_msm_restart_match);

static struct platform_driver msm_restart_driver = {
	.probe = msm_restart_probe,
	.driver = {
		.name = "msm-restart",
		.of_match_table = of_match_ptr(of_msm_restart_match),
	},
};

static int __init msm_restart_init(void)
{
	return platform_driver_register(&msm_restart_driver);
}
pure_initcall(msm_restart_init);
