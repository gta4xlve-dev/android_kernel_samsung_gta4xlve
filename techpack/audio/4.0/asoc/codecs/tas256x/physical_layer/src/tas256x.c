#include "physical_layer/inc/tas256x.h"
#include "physical_layer/inc/tas2562.h"
#include "physical_layer/inc/tas2564.h"
#include "physical_layer/inc/tas256x-device.h"

#define LOG_TAG "[tas256x]"

#define TAS256X_MDELAY 0xFFFFFFFE
#define TAS256X_MSLEEP 0xFFFFFFFD
#define TAS256X_IVSENSER_ENABLE  1
#define TAS256X_IVSENSER_DISABLE 0
static char p_icn_threshold[] = {0x00, 0x01, 0x2f, 0x2c};
static char p_icn_hysteresis[] = {0x00, 0x01, 0x5d, 0xc0};

static unsigned int p_tas256x_classh_d_data[] = {
		/* reg address			size	values */
	TAS256X_CLASSHHEADROOM, 0x4, 0x09, 0x99, 0x99, 0x9a,
	TAS256X_CLASSHHYSTERESIS, 0x4, 0x0, 0x0, 0x0, 0x0,
	TAS256X_CLASSHMTCT, 0x4, 0xb, 0x0, 0x0, 0x0,
	TAS256X_VBATFILTER, 0x1, 0x38,
	TAS256X_CLASSHRELEASETIMER, 0x1, 0x3c,
	TAS256X_BOOSTSLOPE, 0x1, 0x78,
	TAS256X_TESTPAGECONFIGURATION, 0x1, 0xd,
	TAS256X_CLASSDCONFIGURATION3, 0x1, 0x8e,
	TAS256X_CLASSDCONFIGURATION2, 0x1, 0x49,
	TAS256X_CLASSDCONFIGURATION4, 0x1, 0x21,
	TAS256X_CLASSDCONFIGURATION1, 0x1, 0x80,
	TAS256X_EFFICIENCYCONFIGURATION, 0x1, 0xc1,
	0xFFFFFFFF, 0xFFFFFFFF
};

static char HPF_reverse_path[] = {
	0x7F, 0xFF,  0xFF,  0xFF,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00};

static char dvc_pcm[57][4] = {
	{0x00, 0x00, 0x0D, 0x43}, {0x00, 0x00, 0x10, 0xB3}, {0x00, 0x00, 0x15, 0x05},
	{0x00, 0x00, 0x1A, 0x77}, {0x00, 0x00, 0x21, 0x51}, {0x00, 0x00, 0x29, 0xF1},
	{0x00, 0x00, 0x34, 0xCE}, {0x00, 0x00, 0x42, 0x7A}, {0x00, 0x00, 0x53, 0xB0},
	{0x00, 0x00, 0x69, 0x5B}, {0x00, 0x00, 0x84, 0xA3}, {0x00, 0x00, 0xA6, 0xFA},
	{0x00, 0x00, 0xD2, 0x37}, {0x00, 0x01, 0x08, 0xA5}, {0x00, 0x01, 0x4D, 0x2A},
	{0x00, 0x01, 0xA3, 0x6E}, {0x00, 0x02, 0x10, 0x08}, {0x00, 0x02, 0x98, 0xC1},
	{0x00, 0x03, 0x44, 0xE0}, {0x00, 0x04, 0x1D, 0x90}, {0x00, 0x05, 0x2E, 0x5B},
	{0x00, 0x06, 0x85, 0xC8}, {0x00, 0x08, 0x36, 0x22}, {0x00, 0x0A, 0x56, 0x6D},
	{0x00, 0x0D, 0x03, 0xA7}, {0x00, 0x10, 0x62, 0x4E}, {0x00, 0x14, 0xA0, 0x51},
	{0x00, 0x19, 0xF7, 0x86}, {0x00, 0x20, 0xB0, 0xBD}, {0x00, 0x29, 0x27, 0x9E},
	{0x00, 0x33, 0xCF, 0x8E}, {0x00, 0x41, 0x39, 0xD3}, {0x00, 0x52, 0x1D, 0x51},
	{0x00, 0x67, 0x60, 0x45}, {0x00, 0x82, 0x24, 0x8A}, {0x00, 0xA3, 0xD7, 0x0A},
	{0x00, 0xCE, 0x43, 0x29}, {0x01, 0x03, 0xAB, 0x3D}, {0x01, 0x46, 0xE7, 0x5E},
	{0x01, 0x9B, 0x8C, 0x27}, {0x02, 0x06, 0x1B, 0x8A}, {0x02, 0x8C, 0x42, 0x40},
	{0x03, 0x35, 0x25, 0x29}, {0x04, 0x09, 0xC2, 0xB1}, {0x05, 0x15, 0x6D, 0x69},
	{0x06, 0x66, 0x66, 0x66}, {0x08, 0x0E, 0x9F, 0x97}, {0x0A, 0x24, 0xB0, 0x63},
	{0x0C, 0xC5, 0x09, 0xAC}, {0x10, 0x13, 0x79, 0x88}, {0x14, 0x3D, 0x13, 0x62},
	{0x19, 0x7A, 0x96, 0x7F}, {0x20, 0x13, 0x73, 0x9E}, {0x28, 0x61, 0x9A, 0xEA},
	{0x32, 0xD6, 0x46, 0x18}, {0x40, 0x00, 0x00, 0x00}, {0x50, 0x92, 0x3B, 0xE4}
};

static char lim_max_attn[16][4] = {
	{0x14, 0x49, 0x60, 0xC5}, {0x16, 0xC3, 0x10, 0xE3}, {0x19, 0x8A, 0x13, 0x57},
	{0x1C, 0xA7, 0xD7, 0x68}, {0x20, 0x26, 0xF3, 0x10}, {0x24, 0x13, 0x46, 0xF6},
	{0x28, 0x7A, 0x26, 0xC5}, {0x2D, 0x6A, 0x86, 0x6F}, {0x32, 0xF5, 0x2C, 0xFF},
	{0x39, 0x2C, 0xED, 0x8E}, {0x40, 0x26, 0xE7, 0x3D}, {0x47, 0xFA, 0xCC, 0xF0},
	{0x50, 0xC3, 0x35, 0xD4}, {0x5A, 0x9D, 0xF7, 0xAC}, {0x65, 0xAC, 0x8C, 0x2F},
	{0x72, 0x14, 0x82, 0xC0}
};

static char vbat_lim_max_thd[27][4] = {
	{0x14, 0x00, 0x00, 0x00}, {0x18, 0x00, 0x00, 0x00}, {0x1C, 0x00, 0x00, 0x00},
	{0x20, 0x00, 0x00, 0x00}, {0x24, 0x00, 0x00, 0x00}, {0x28, 0x00, 0x00, 0x00},
	{0x2C, 0x00, 0x00, 0x00}, {0x30, 0x00, 0x00, 0x00}, {0x34, 0x00, 0x00, 0x00},
	{0x38, 0x00, 0x00, 0x00}, {0x3C, 0x00, 0x00, 0x00}, {0x40, 0x00, 0x00, 0x00},
	{0x44, 0x00, 0x00, 0x00}, {0x48, 0x00, 0x00, 0x00}, {0x4C, 0x00, 0x00, 0x00},
	{0x50, 0x00, 0x00, 0x00}, {0x54, 0x00, 0x00, 0x00}, {0x58, 0x00, 0x00, 0x00},
	{0x5C, 0x00, 0x00, 0x00}, {0x60, 0x00, 0x00, 0x00}, {0x64, 0x00, 0x00, 0x00},
	{0x68, 0x00, 0x00, 0x00}, {0x6C, 0x00, 0x00, 0x00}, {0x70, 0x00, 0x00, 0x00},
	{0x74, 0x00, 0x00, 0x00}, {0x78, 0x00, 0x00, 0x00}, {0x7C, 0x00, 0x00, 0x00}
};

static char vbat_lim_min_thd[27][4] = {
	{0x14, 0x00, 0x00, 0x00}, {0x18, 0x00, 0x00, 0x00}, {0x1C, 0x00, 0x00, 0x00},
	{0x20, 0x00, 0x00, 0x00}, {0x24, 0x00, 0x00, 0x00}, {0x28, 0x00, 0x00, 0x00},
	{0x2C, 0x00, 0x00, 0x00}, {0x30, 0x00, 0x00, 0x00}, {0x34, 0x00, 0x00, 0x00},
	{0x38, 0x00, 0x00, 0x00}, {0x3C, 0x00, 0x00, 0x00}, {0x40, 0x00, 0x00, 0x00},
	{0x44, 0x00, 0x00, 0x00}, {0x48, 0x00, 0x00, 0x00}, {0x4C, 0x00, 0x00, 0x00},
	{0x50, 0x00, 0x00, 0x00}, {0x54, 0x00, 0x00, 0x00}, {0x58, 0x00, 0x00, 0x00},
	{0x5C, 0x00, 0x00, 0x00}, {0x60, 0x00, 0x00, 0x00}, {0x64, 0x00, 0x00, 0x00},
	{0x68, 0x00, 0x00, 0x00}, {0x6C, 0x00, 0x00, 0x00}, {0x70, 0x00, 0x00, 0x00},
	{0x74, 0x00, 0x00, 0x00}, {0x78, 0x00, 0x00, 0x00}, {0x7C, 0x00, 0x00, 0x00}
};

static char vbat_lim_infl_pt[41][4] = {
	{0x20, 0x00, 0x00, 0x00}, {0x21, 0x99, 0x99, 0x99}, {0x23, 0x33, 0x33, 0x33},
	{0x24, 0xCC, 0xCC, 0xCC}, {0x26, 0x66, 0x66, 0x66}, {0x28, 0x00, 0x00, 0x00},
	{0x29, 0x99, 0x99, 0x99}, {0x2B, 0x33, 0x33, 0x33}, {0x2C, 0xCC, 0xCC, 0xCC},
	{0x2E, 0x66, 0x66, 0x66}, {0x30, 0x00, 0x00, 0x00}, {0x31, 0x99, 0x99, 0x99},
	{0x33, 0x33, 0x33, 0x33}, {0x34, 0xCC, 0xCC, 0xCC}, {0x36, 0x66, 0x66, 0x66},
	{0x38, 0x00, 0x00, 0x00}, {0x39, 0x99, 0x99, 0x99}, {0x3B, 0x33, 0x33, 0x33},
	{0x3C, 0xCC, 0xCC, 0xCC}, {0x3E, 0x66, 0x66, 0x66}, {0x40, 0x00, 0x00, 0x00},
	{0x41, 0x99, 0x99, 0x99}, {0x43, 0x33, 0x33, 0x33}, {0x44, 0xCC, 0xCC, 0xCC},
	{0x46, 0x66, 0x66, 0x66}, {0x48, 0x00, 0x00, 0x00}, {0x49, 0x99, 0x99, 0x99},
	{0x4B, 0x33, 0x33, 0x33}, {0x4C, 0xCC, 0xCC, 0xCC}, {0x4E, 0x66, 0x66, 0x66},
	{0x4F, 0xFF, 0xFF, 0xFF}, {0x51, 0x99, 0x99, 0x99}, {0x53, 0x33, 0x33, 0x33},
	{0x54, 0xCC, 0xCC, 0xCC}, {0x56, 0x66, 0x66, 0x66}, {0x57, 0xFF, 0xFF, 0xFF},
	{0x59, 0x99, 0x99, 0x99}, {0x5B, 0x33, 0x33, 0x33}, {0x5C, 0xCC, 0xCC, 0xCC},
	{0x5E, 0x66, 0x66, 0x66}, {0x5F, 0xFF, 0xFF, 0xFF}
};

static char vbat_lim_track_slope[7][4] = {
	{0x10, 0x00, 0x00, 0x00}, {0x18, 0x00, 0x00, 0x00}, {0x20, 0x00, 0x00, 0x00},
	{0x28, 0x00, 0x00, 0x00}, {0x30, 0x00, 0x00, 0x00}, {0x38, 0x00, 0x00, 0x00},
	{0x40, 0x00, 0x00, 0x00}
};

static char bop_thd[16][4] = {
	{0x28, 0x00, 0x00, 0x00}, {0x29, 0x99, 0x99, 0x99}, {0x2B, 0x33, 0x33, 0x33},
	{0x2C, 0xCC, 0xCC, 0xCC}, {0x2E, 0x66, 0x66, 0x66}, {0x30, 0x00, 0x00, 0x00},
	{0x31, 0x99, 0x99, 0x99}, {0x33, 0x33, 0x33, 0x33}, {0x34, 0xCC, 0xCC, 0xCC},
	{0x36, 0x66, 0x66, 0x66}, {0x38, 0x00, 0x00, 0x00}, {0x39, 0x99, 0x99, 0x99},
	{0x3B, 0x33, 0x33, 0x33}, {0x3C, 0xCC, 0xCC, 0xCC}, {0x3E, 0x66, 0x66, 0x66},
	{0x40, 0x00, 0x00, 0x00}
};

static char bop_always_on[16][4] = {
	{0x28, 0x00, 0x00, 0x00}, {0x29, 0x99, 0x99, 0x99}, {0x2F, 0x33, 0x33, 0x33},
	{0x2C, 0xCC, 0xCC, 0xCC}, {0x2E, 0x66, 0x66, 0x66}, {0x30, 0x00, 0x00, 0x00},
	{0x31, 0x99, 0x99, 0x99}, {0x33, 0x33, 0x33, 0x33}, {0x34, 0xCC, 0xCC, 0xCC},
	{0x36, 0x66, 0x66, 0x66}, {0x38, 0x00, 0x00, 0x00}, {0x39, 0x99, 0x99, 0x99},
	{0x3B, 0x33, 0x33, 0x33}, {0x3C, 0xCC, 0xCC, 0xCC}, {0x3E, 0x66, 0x66, 0x66},
	{0x40, 0x00, 0x00, 0x00}
};

static char bop_always_off[16][4] = {
	{0x28, 0x00, 0x00, 0x00}, {0x29, 0x99, 0x99, 0x99}, {0x2B, 0x33, 0x33, 0x33},
	{0x2C, 0xCC, 0xCC, 0xCC}, {0x2E, 0x66, 0x66, 0x66}, {0x30, 0x00, 0x00, 0x00},
	{0x31, 0x99, 0x99, 0x99}, {0x33, 0x33, 0x33, 0x33}, {0x34, 0xCC, 0xCC, 0xCC},
	{0x36, 0x66, 0x66, 0x66}, {0x38, 0x00, 0x00, 0x00}, {0x39, 0x99, 0x99, 0x99},
	{0x3B, 0x33, 0x33, 0x33}, {0x3C, 0xCC, 0xCC, 0xCC}, {0x3E, 0x66, 0x66, 0x66},
	{0x40, 0x00, 0x00, 0x00}
};

static char bsd_thd[16][4] = {
	{0x28, 0x00, 0x00, 0x00}, {0x29, 0x99, 0x99, 0x99}, {0x2B, 0x33, 0x33, 0x33},
	{0x2C, 0xCC, 0xCC, 0xCC}, {0x2E, 0x66, 0x66, 0x66}, {0x30, 0x00, 0x00, 0x00},
	{0x31, 0x99, 0x99, 0x99}, {0x33, 0x33, 0x33, 0x33}, {0x34, 0xCC, 0xCC, 0xCC},
	{0x36, 0x66, 0x66, 0x66}, {0x38, 0x00, 0x00, 0x00}, {0x39, 0x99, 0x99, 0x99},
	{0x3B, 0x33, 0x33, 0x33}, {0x3C, 0xCC, 0xCC, 0xCC}, {0x3E, 0x66, 0x66, 0x66},
	{0x40, 0x00, 0x00, 0x00}
};

static char classH_timer[23][4] = {
	{0x40, 0x0F, 0x01, 0xC0}, {0x50, 0x11, 0x01, 0xC0}, {0x50, 0x13, 0x01, 0xC0},
	{0x50, 0x15, 0x01, 0xC0}, {0x50, 0x17, 0x01, 0xC0}, {0x50, 0x19, 0x01, 0xC0},
	{0x50, 0x1B, 0x01, 0xC0}, {0x50, 0x1D, 0x01, 0xC0}, {0x50, 0x1F, 0x01, 0xC0},
	{0x60, 0x21, 0x01, 0xC0}, {0x60, 0x23, 0x01, 0xC0}, {0x60, 0x25, 0x01, 0xC0},
	{0x60, 0x27, 0x01, 0xC0}, {0x60, 0x29, 0x01, 0xC0}, {0x60, 0x2B, 0x01, 0xC0},
	{0x60, 0x2D, 0x01, 0xC0}, {0x60, 0x2F, 0x01, 0xC0}, {0x60, 0x31, 0x01, 0xC0},
	{0x60, 0x33, 0x01, 0xC0}, {0x60, 0x35, 0x01, 0xC0}, {0x60, 0x37, 0x01, 0xC0},
	{0x60, 0x39, 0x01, 0xC0}, {0x60, 0x3B, 0x01, 0xC0}
};

static char classH_slope[23] = {
	0x2E, 0x28, 0x24, 0x21, 0x1E, 0x1C, 0x1A, 0x18, 0x17, 0x15,
	0x14, 0x13, 0x12, 0x11, 0x11, 0x10, 0xF, 0xF, 0xE, 0xD, 0xD,
	0xD, 0xC
};

static unsigned int p_tas2562_emphasis_filter_48kHZ[] = {
	/* Program the first order filter as an FIR filter */
	TAS256X_REG(0x00, 0x2, 0x70), 0xc, 0x7f, 0xfc, 0xb9, 0x23, 0x80,
		0x89, 0x79, 0x1A, 0x00, 0x00, 0x00, 0x00,
	/* Set post processing gain = 7.33/30 */
	TAS256X_REG(0x00, 0x3, 0x50), 0x8, 0xfd, 0xf7, 0xbc, 0xc0, 0xfd,
		0xde, 0x65, 0x55,
	/* Program the biquad feedback coefficients with pole = 16Hz */
	TAS256X_REG(0x00, 0x4, 0x40), 0xc, 0x04, 0x00, 0x00, 0x00, 0xc0,
		0x01, 0xc0, 0x60, 0x1f, 0xfe, 0x48, 0xd1,
	/* Program the biquad feedforward coefficients */
	TAS256X_REG(0x00, 0x4, 0x4c), 0xc, 0x7b, 0xfb, 0xfe, 0x62, 0x84,
		0x04, 0x01, 0x9d, 0x00, 0x00, 0x00, 0x00,
	0xFFFFFFFF, 0xFFFFFFFF
};

static int tas256x_i2c_load_data(struct tas256x_priv *p_tas256x,
				enum channel chn,
				unsigned int *p_data)
{
	unsigned int n_register;
	unsigned int *n_data;
	unsigned char buf[128] = {0};
	unsigned int n_length = 0;
	unsigned int i = 0;
	unsigned int n_size = 0;
	int n_result = 0;

	do {
		n_register = p_data[n_length];
		n_size = p_data[n_length + 1];
		n_data = &p_data[n_length + 2];
		if (n_register == TAS256X_MSLEEP) {
			msleep(n_data[0]);
			pr_debug("%s %s, msleep = %d\n", LOG_TAG,
				__func__, n_data[0]);
		} else if (n_register == TAS256X_MDELAY) {
			msleep(n_data[0]);
			pr_debug("%s %s, msleep = %d\n", LOG_TAG,
				__func__, n_data[0]);
		} else {
			if (n_register != 0xFFFFFFFF) {
				if (n_size > 128) {
					pr_err("%s %s, Line=%d, invalid size, maximum is 128 bytes!\n",
					 LOG_TAG, __func__, __LINE__);
					break;
				}
				if (n_size > 1) {
					for (i = 0; i < n_size; i++)
						buf[i] = (unsigned char)n_data[i];
					n_result = p_tas256x->bulk_write(
						p_tas256x, chn,
						n_register, buf, n_size);
					if (n_result < 0)
						break;
				} else if (n_size == 1) {
					n_result = p_tas256x->write(p_tas256x,
					chn,
					n_register, n_data[0]);
					if (n_result < 0)
						break;
				} else {
					pr_err("%s %s, Line=%d,invalid size, minimum is 1 bytes!\n",
						LOG_TAG, __func__, __LINE__);
				}
			}
		}
		n_length = n_length + 2 + p_data[n_length + 1];
	} while (n_register != 0xFFFFFFFF);
	return n_result;
}

int tas256x_update_default_params(struct tas256x_priv *p_tas256x, int ch)
{
	int n_result = 0;

	/*Initialize to default values*/
	if (p_tas256x->devs[ch-1]->device_id == DEVICE_TAS2562) {
		p_tas256x->devs[ch-1]->dvc_pcm = 55; /*0dB*/
		p_tas256x->devs[ch-1]->lim_max_attn = 7; /*-9dB*/
		p_tas256x->devs[ch-1]->lim_thr_max = 13; /*9V*/
		p_tas256x->devs[ch-1]->lim_thr_min = 3; /*4V*/
		p_tas256x->devs[ch-1]->lim_infl_pt = 13; /*3.3V*/
		p_tas256x->devs[ch-1]->lim_trk_slp = 0; /*1 V/V*/
		p_tas256x->devs[ch-1]->bop_thd = 4; /*2.9 V*/
		p_tas256x->devs[ch-1]->bosd_thd = 2; /*2.7 V*/
		p_tas256x->devs[ch-1]->bst_vltg = 0xA; /*Mode = speaker*/
		p_tas256x->devs[ch-1]->bst_ilm = 0x36; /*0dB*/
		p_tas256x->devs[ch-1]->ampoutput_lvl = 0x10; /*15.5dBV: 8 + 0.5 * ampoutput_lvl */
		p_tas256x->devs[ch-1]->lim_switch = 0; /*-9dB*/
		p_tas256x->devs[ch-1]->lim_att_rate = 1; /*9V*/
		p_tas256x->devs[ch-1]->lim_att_stp_size = 1; /*4V*/
		p_tas256x->devs[ch-1]->lim_rel_rate = 6; /*3.3V*/
		p_tas256x->devs[ch-1]->lim_rel_stp_size = 1; /*1 V/V*/
		p_tas256x->devs[ch-1]->bop_enable = 1; /*2.9 V*/
		p_tas256x->devs[ch-1]->bop_mute = 0; /*2.7 V*/
		p_tas256x->devs[ch-1]->bosd_enable = 0; /*3.3V*/
		p_tas256x->devs[ch-1]->bop_att_rate = 1; /*1 V/V*/
		p_tas256x->devs[ch-1]->bop_att_stp_size = 1; /*2.9 V*/
		p_tas256x->devs[ch-1]->bop_hld_time = 6; /*2.7 V*/
		p_tas256x->devs[ch-1]->vbat_lpf = 2; /*2.7 V*/
		p_tas256x->devs[ch-1]->rx_cfg = 0;
		p_tas256x->devs[ch-1]->classh_timer = 5;
		p_tas256x->devs[ch-1]->receiver_enable = 0;
	} else if (p_tas256x->devs[ch-1]->device_id == DEVICE_TAS2564) {
		p_tas256x->devs[ch-1]->rx_mode = 0; /*Mode = speaker*/
		p_tas256x->devs[ch-1]->dvc_pcm = 55; /*0dB*/
		p_tas256x->devs[ch-1]->lim_max_attn = 7; /*-9dB*/
		p_tas256x->devs[ch-1]->lim_thr_max = 13; /*9V*/
		p_tas256x->devs[ch-1]->lim_thr_min = 3; /*4V*/
		p_tas256x->devs[ch-1]->lim_infl_pt = 13; /*3.3V*/
		p_tas256x->devs[ch-1]->lim_trk_slp = 0; /*1 V/V*/
		p_tas256x->devs[ch-1]->bop_thd = 4; /*2.9 V*/
		p_tas256x->devs[ch-1]->bosd_thd = 2; /*2.7 V*/
		p_tas256x->devs[ch-1]->bst_vltg = 13; /*Mode = speaker*/
		p_tas256x->devs[ch-1]->bst_ilm = 0x39; /*0dB*/
		p_tas256x->devs[ch-1]->ampoutput_lvl = 0xD; /*16dBV: 9.5 + 0.5 * ampoutput_lvl */
		p_tas256x->devs[ch-1]->lim_switch = 0; /*-9dB*/
		p_tas256x->devs[ch-1]->lim_att_rate = 1; /*9V*/
		p_tas256x->devs[ch-1]->lim_att_stp_size = 1; /*4V*/
		p_tas256x->devs[ch-1]->lim_rel_rate = 6; /*3.3V*/
		p_tas256x->devs[ch-1]->lim_rel_stp_size = 1; /*1 V/V*/
		p_tas256x->devs[ch-1]->bop_enable = 1; /*2.9 V*/
		p_tas256x->devs[ch-1]->bop_mute = 0; /*2.7 V*/
		p_tas256x->devs[ch-1]->bosd_enable = 0; /*3.3V*/
		p_tas256x->devs[ch-1]->bop_att_rate = 1; /*1 V/V*/
		p_tas256x->devs[ch-1]->bop_att_stp_size = 1; /*2.9 V*/
		p_tas256x->devs[ch-1]->bop_hld_time = 6; /*2.7 V*/
		p_tas256x->devs[ch-1]->vbat_lpf = 2; /*2.7 V*/
		p_tas256x->devs[ch-1]->rx_cfg = 0;
		p_tas256x->devs[ch-1]->classh_timer = 5;
		p_tas256x->devs[ch-1]->receiver_enable = 0;
	}
	p_tas256x->icn_sw = 0;
	p_tas256x->mn_rx_slot_map[0] = 0;
	p_tas256x->mn_rx_slot_map[1] = 1;
	return n_result;
}

int tas256x_set_power_up(struct tas256x_priv *p_tas256x,
	enum channel chn)
{
	int n_result = 0;

	n_result = p_tas256x->update_bits(p_tas256x,
		chn, TAS256X_POWERCONTROL,
		TAS256X_POWERCONTROL_OPERATIONALMODE10_MASK,
		TAS256X_POWERCONTROL_OPERATIONALMODE10_ACTIVE);

	/* Let Power on the device */
	msleep(20);

	return n_result;
}

int tas256x_enable_cm_hysterysis(struct tas256x_priv *p_tas256x, int enable,
	int chn)
{
	int n_result = 0;

	n_result = p_tas256x->update_bits(p_tas256x, chn,
		TAS256X_MISC_CLASSD,
		TAS256X_CMP_HYST_MASK,
		(enable << TAS256X_CMP_HYST_SHIFT));

	return n_result;
}

int tas256x_set_power_mute(struct tas256x_priv *p_tas256x,
	enum channel chn)
{
	int n_result = 0;

	n_result = p_tas256x->update_bits(p_tas256x, chn,
		TAS256X_POWERCONTROL,
		TAS256X_POWERCONTROL_OPERATIONALMODE10_MASK,
		TAS256X_POWERCONTROL_OPERATIONALMODE10_MUTE);

	return n_result;
}

int tas256x_set_power_shutdown(struct tas256x_priv *p_tas256x,
	enum channel chn)
{
	int n_result = 0;

	n_result = p_tas256x->update_bits(p_tas256x, chn,
		TAS256X_POWERCONTROL,
		TAS256X_POWERCONTROL_OPERATIONALMODE10_MASK,
		TAS256X_POWERCONTROL_OPERATIONALMODE10_SHUTDOWN);

	/*Device Shutdown need 20ms after shutdown writes are made*/
	msleep(20);

	return n_result;
}

int tas256x_power_check(struct tas256x_priv *p_tas256x, int *state, int ch)
{
	int n_result = 0;
	int status;

	n_result = p_tas256x->read(p_tas256x, ch,
		TAS256X_POWERCONTROL, &status);

	status &= TAS256X_POWERCONTROL_OPERATIONALMODE10_MASK;

	if ((status != TAS256X_POWERCONTROL_OPERATIONALMODE10_SHUTDOWN)
		&& (status != TAS256X_POWERCONTROL_OPERATIONALMODE10_MUTE))
		*state = 1;
	else
		*state = 0;

	return n_result;
}

/*
 *bool enable = 1; IV Sense Power UP = 0;
 *	IV Sense Power Down
 */
int tas256x_iv_sense_enable_set(struct tas256x_priv *p_tas256x, bool enable,
	int ch)
{
	int n_result = 0;

	if (enable) /*IV Sense Power Up*/
		n_result = p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_POWERCONTROL,
			TAS256X_POWERCONTROL_ISNSPOWER_MASK |
			TAS256X_POWERCONTROL_VSNSPOWER_MASK,
			TAS256X_POWERCONTROL_VSNSPOWER_ACTIVE |
			TAS256X_POWERCONTROL_ISNSPOWER_ACTIVE);
	else /*IV Sense Power Down*/
		n_result = p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_POWERCONTROL,
			TAS256X_POWERCONTROL_ISNSPOWER_MASK |
			TAS256X_POWERCONTROL_VSNSPOWER_MASK,
			TAS256X_POWERCONTROL_VSNSPOWER_POWEREDDOWN |
			TAS256X_POWERCONTROL_ISNSPOWER_POWEREDDOWN);

	p_tas256x->iv_enable = enable;
	return n_result;
}

bool tas256x_iv_sense_enable_get(struct tas256x_priv *p_tas256x, int ch)
{
	int n_result = 0;
	bool enable = 0;
	int value = 0;

	n_result = p_tas256x->read(p_tas256x, ch,
			TAS256X_POWERCONTROL, &value);
	if (n_result < 0)
		pr_err("%s can't get ivsensor state %s, L=%d\n",
			 LOG_TAG, __func__, __LINE__);
	else if (((value & TAS256X_POWERCONTROL_ISNSPOWER_MASK)
			== TAS256X_POWERCONTROL_ISNSPOWER_ACTIVE)
			&& ((value & TAS256X_POWERCONTROL_VSNSPOWER_MASK)
			== TAS256X_POWERCONTROL_VSNSPOWER_ACTIVE)) {
		enable = TAS256X_IVSENSER_ENABLE;
	} else {
		enable = TAS256X_IVSENSER_DISABLE;
	}
	return enable;
}
/*IVVBat sense slot disable*/
int tas256x_ivvbat_slot_disable(struct tas256x_priv *p_tas256x, int vbat,
	int ch)
{
	int n_result = 0;

	/*Disable V-Sense*/
	n_result |= p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_TDMCONFIGURATIONREG5,
			TAS256X_TDMCONFIGURATIONREG5_VSNSTX_MASK,
			TAS256X_TDMCONFIGURATIONREG5_VSNSTX_DISABLE);
	/*Disable I-Sense*/
	n_result |= p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_TDMCONFIGURATIONREG6,
			TAS256X_TDMCONFIGURATIONREG6_ISNSTX_MASK,
			TAS256X_TDMCONFIGURATIONREG6_ISNSTX_DISABLE);

	/*Disable VBat-Sense*/
	if (vbat)
		n_result |= p_tas256x->update_bits(p_tas256x, ch,
				TAS256X_TDMCONFIGURATIONREG7,
				TAS256X_TDMCONFIGURATIONREG7_VBATTX_MASK,
				TAS256X_TDMCONFIGURATIONREG7_VBATTX_DISABLE);

	return n_result;
}

/*No need channel argument*/
int tas256x_set_iv_slot(struct tas256x_priv *p_tas256x, int ch, int vslot,
	int islot)
{
	int n_result = 0;

	vslot |= TAS256X_TDMCONFIGURATIONREG5_VSNSTX_ENABLE;
	islot |= TAS256X_TDMCONFIGURATIONREG6_ISNSTX_ENABLE;
	n_result |= p_tas256x->write(p_tas256x, ch,
					TAS256X_TDMCONFIGURATIONREG5, vslot);

	n_result |= p_tas256x->write(p_tas256x, ch,
					TAS256X_TDMCONFIGURATIONREG6, islot);

	return n_result;
}

int tas256x_set_vbat_slot(struct tas256x_priv *p_tas256x, int ch, int slot)
{
	int n_result = 0;

	slot |= TAS256X_TDMCONFIGURATIONREG7_VBATTX_ENABLE;
	n_result |= p_tas256x->write(p_tas256x, ch,
			TAS256X_TDMCONFIGURATIONREG7,
			slot);

	return n_result;
}

/*No need channel argument*/
int tas256x_iv_bitwidth_config(struct tas256x_priv *p_tas256x, int bitwidth,
	int ch)
{
	int n_result = 0;

	if (bitwidth == 8)
		n_result = p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_TDMCONFIGURATIONREG2,
			TAS256X_TDMCONFIGURATIONREG2_IVMONLEN76_MASK,
			TAS256X_TDMCONFIGURATIONREG2_IVMONLEN76_8BITS);
	else if (bitwidth == 12)
		n_result = p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_TDMCONFIGURATIONREG2,
			TAS256X_TDMCONFIGURATIONREG2_IVLEN76_MASK,
			TAS256X_TDMCONFIGURATIONREG2_IVLENCFG76_12BITS);
	else /*bitwidth == 16*/
		n_result = p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_TDMCONFIGURATIONREG2,
			TAS256X_TDMCONFIGURATIONREG2_IVLEN76_MASK,
			TAS256X_TDMCONFIGURATIONREG2_IVLENCFG76_16BITS);

	return n_result;
}

int tas256x_set_auto_detect_clock(struct tas256x_priv *p_tas256x,
			int value, int ch)
{
	int n_result = 0;

	if (value == 0)
		value = TAS256X_TDMCONFIGURATIONREG0_DETECTSAMPRATE_DISABLED;
	else
		value = TAS256X_TDMCONFIGURATIONREG0_DETECTSAMPRATE_ENABLED;

	/*Set Sample Rate for auto detection only in case of TDM*/
	n_result = p_tas256x->update_bits(p_tas256x, channel_both,
			TAS256X_TDMCONFIGURATIONREG0,
			TAS256X_TDMCONFIGURATIONREG0_DETECTSAMPRATE_MASK,
			value);

	return n_result;
}

/**/
int tas256x_set_samplerate(struct tas256x_priv *p_tas256x,
			int samplerate, int ch)
{
	int n_result = 0;

	switch (samplerate) {
	case 48000:
		n_result = p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_TDMCONFIGURATIONREG0,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATERAMP_MASK,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATERAMP_48KHZ);
		n_result |= p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_TDMCONFIGURATIONREG0,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATE31_MASK,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATE31_44_1_48KHZ);
		break;
	case 44100:
		n_result = p_tas256x->update_bits(p_tas256x, channel_both,
			TAS256X_TDMCONFIGURATIONREG0,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATERAMP_MASK,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATERAMP_44_1KHZ);
		n_result |= p_tas256x->update_bits(p_tas256x, channel_both,
			TAS256X_TDMCONFIGURATIONREG0,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATE31_MASK,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATE31_44_1_48KHZ);
		break;
	case 96000:
		n_result = p_tas256x->update_bits(p_tas256x, channel_both,
			TAS256X_TDMCONFIGURATIONREG0,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATERAMP_MASK,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATERAMP_48KHZ);
		n_result |= p_tas256x->update_bits(p_tas256x, channel_both,
			TAS256X_TDMCONFIGURATIONREG0,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATE31_MASK,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATE31_88_2_96KHZ);
		break;
	case 88200:
		n_result = p_tas256x->update_bits(p_tas256x, channel_both,
			TAS256X_TDMCONFIGURATIONREG0,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATERAMP_MASK,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATERAMP_44_1KHZ);
		n_result |= p_tas256x->update_bits(p_tas256x, channel_both,
			TAS256X_TDMCONFIGURATIONREG0,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATE31_MASK,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATE31_88_2_96KHZ);
		break;
	case 19200:
		n_result = p_tas256x->update_bits(p_tas256x, channel_both,
			TAS256X_TDMCONFIGURATIONREG0,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATERAMP_MASK,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATERAMP_48KHZ);
		n_result |= p_tas256x->update_bits(p_tas256x, channel_both,
			TAS256X_TDMCONFIGURATIONREG0,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATE31_MASK,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATE31_176_4_192KHZ);
		break;
	case 17640:
		n_result = p_tas256x->update_bits(p_tas256x, channel_both,
			TAS256X_TDMCONFIGURATIONREG0,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATERAMP_MASK,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATERAMP_44_1KHZ);
		n_result |= p_tas256x->update_bits(p_tas256x, channel_both,
			TAS256X_TDMCONFIGURATIONREG0,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATE31_MASK,
			TAS256X_TDMCONFIGURATIONREG0_SAMPRATE31_176_4_192KHZ);
		break;
	default:
		n_result = -1;
		pr_info("%s %s, unsupported sample rate, %d\n",
			 LOG_TAG, __func__, samplerate);
	}

	if (n_result == 0)
		p_tas256x->mn_sampling_rate = samplerate;

	return n_result;
}

/*
 *rx_edge = 0; Rising
 *= 1; Falling
 */
int tas256x_tx_set_edge(struct tas256x_priv *p_tas256x,
	unsigned int tx_edge, int ch)
{
	int n_result = 0;

	n_result |= p_tas256x->update_bits(p_tas256x, ch,
		TAS256X_TDMConfigurationReg4,
		TAS256X_TDMCONFIGURATIONREG4_TXEDGE_MASK,
		(tx_edge << TAS256X_TDMCONFIGURATIONREG4_TXEDGE_SHIFT));

	if (n_result == 0)
		p_tas256x->mn_tx_edge = tx_edge;

	return n_result;
}

/*
 *rx_edge = 0; Rising
 *= 1; Falling
 */
int tas256x_rx_set_edge(struct tas256x_priv *p_tas256x,
	unsigned int rx_edge, int ch)
{
	int n_result = 0;

	n_result |= p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_TDMCONFIGURATIONREG1,
			TAS256X_TDMCONFIGURATIONREG1_RXEDGE_MASK,
			(rx_edge << TAS256X_TDMCONFIGURATIONREG1_RXEDGE_SHIFT));

	if (n_result == 0)
		p_tas256x->mn_rx_edge = rx_edge;

	return n_result;
}
int tas256x_rx_set_start_slot(struct tas256x_priv *p_tas256x,
	unsigned int rx_start_slot, int ch)
{
	int n_result = 0;

	n_result |= p_tas256x->update_bits(p_tas256x, ch,
		TAS256X_TDMCONFIGURATIONREG1,
		TAS256X_TDMCONFIGURATIONREG1_RXOFFSET51_MASK,
		(rx_start_slot <<
		TAS256X_TDMCONFIGURATIONREG1_RXOFFSET51_SHIFT));

	if (n_result == 0)
		p_tas256x->mn_rx_offset = rx_start_slot;

	return n_result;
}

int tas256x_tx_set_start_slot(struct tas256x_priv *p_tas256x,
	unsigned int tx_start_slot, int ch)
{
	int n_result = 0;

	n_result |= p_tas256x->update_bits(p_tas256x, ch,
		TAS256X_TDMConfigurationReg4,
		TAS256X_TDMCONFIGURATIONREG4_TXOFFSET31_MASK,
		(tx_start_slot <<
		TAS256X_TDMCONFIGURATIONREG4_TXOFFSET31_SHIFT));

	if (n_result == 0)
		p_tas256x->mn_tx_offset = tx_start_slot;

	return n_result;
}

int tas256x_rx_set_frame_start(struct tas256x_priv *p_tas256x,
	unsigned int frame_start, int ch)
{
	int n_result = 0;

	p_tas256x->update_bits(p_tas256x, ch,
		TAS256X_TDMCONFIGURATIONREG0,
		TAS256X_TDMCONFIGURATIONREG0_FRAMESTART_MASK,
		frame_start);

	if (n_result == 0)
		p_tas256x->mn_frame_start = frame_start;

	return n_result;
}

int tas256x_rx_set_slot_len(struct tas256x_priv *p_tas256x,
	int slot_width, int ch)
{
	int n_result = -1;

	switch (slot_width) {
	case 16:
	n_result = p_tas256x->update_bits(p_tas256x, ch,
		TAS256X_TDMCONFIGURATIONREG2,
		TAS256X_TDMCONFIGURATIONREG2_RXSLEN10_MASK,
		TAS256X_TDMCONFIGURATIONREG2_RXSLEN10_16BITS);
	break;
	case 24:
	case 32:
	n_result = p_tas256x->update_bits(p_tas256x, ch,
		TAS256X_TDMCONFIGURATIONREG2,
		TAS256X_TDMCONFIGURATIONREG2_RXSLEN10_MASK,
		TAS256X_TDMCONFIGURATIONREG2_RXSLEN10_32BITS);
	break;
	case 0:
	/* Do not change slot width */
	break;
	}

	if (n_result == 0)
		p_tas256x->mn_rx_slot_width = slot_width;

	return n_result;
}

int tas256x_rx_set_slot(struct tas256x_priv *p_tas256x,
	int slot, int ch)
{
	int n_result = -1;

	if (ch == channel_right)
		n_result = p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_TDMCONFIGURATIONREG3,
			TAS256X_TDMCONFIGURATIONREG3_RXSLOTRIGHT74_MASK,
			(slot << TAS256X_TDMCONFIGURATIONREG3_RXSLOTRIGHT74_SHIFT));
	else/*Assumed Left*/
		n_result = p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_TDMCONFIGURATIONREG3,
			TAS256X_TDMCONFIGURATIONREG3_RXSLOTLeft30_Mask,
			(slot << TAS256X_TDMCONFIGURATIONREG3_RXSLOTLeft30_SHIFT));

	if (n_result == 0)
		p_tas256x->mn_rx_slot_map[ch-1] = slot;
	return n_result;
}

int tas256x_rx_set_bitwidth(struct tas256x_priv *p_tas256x,
	int bitwidth, int ch)
{
	int n_result = -1;

	switch (bitwidth) {
	case 16:
		n_result = p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_TDMCONFIGURATIONREG2,
			TAS256X_TDMCONFIGURATIONREG2_RXWLEN32_MASK,
			TAS256X_TDMCONFIGURATIONREG2_RXWLEN32_16BITS);
		break;
	case 24:
		n_result = p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_TDMCONFIGURATIONREG2,
			TAS256X_TDMCONFIGURATIONREG2_RXWLEN32_MASK,
			TAS256X_TDMCONFIGURATIONREG2_RXWLEN32_24BITS);
		break;
	case 32:
		n_result = p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_TDMCONFIGURATIONREG2,
			TAS256X_TDMCONFIGURATIONREG2_RXWLEN32_MASK,
			TAS256X_TDMCONFIGURATIONREG2_RXWLEN32_32BITS);
		break;
	default:
		pr_info("%s Not supported params format\n",  LOG_TAG);
		break;
	}

	if (n_result == 0)
		p_tas256x->mn_rx_width = bitwidth;

	return n_result;
}

int tas256x_interrupt_clear(struct tas256x_priv *p_tas256x, int ch)
{
	int n_result = 0;

	p_tas256x->update_bits(p_tas256x, ch,
		TAS256X_INTERRUPTCONFIGURATION,
		TAS256X_INTERRUPTCONFIGURATION_LTCHINTCLEAR_MASK,
		TAS256X_INTERRUPTCONFIGURATION_LTCHINTCLEAR);

	return n_result;
}

int tas256x_interrupt_enable(struct tas256x_priv *p_tas256x,
	int val, int ch)
{
	int n_result = 0;

	if (val) {
		/*Enable Interrupts*/
		n_result = p_tas256x->write(p_tas256x, ch,
			TAS256X_INTERRUPTMASKREG0, 0xf8);
		n_result |= p_tas256x->write(p_tas256x, ch,
			TAS256X_INTERRUPTMASKREG1, 0xb1);
	} else {
		/*Disable Interrupts*/
		n_result = p_tas256x->write(p_tas256x, ch,
			TAS256X_INTERRUPTMASKREG0,
			TAS256X_INTERRUPTMASKREG0_DISABLE);
		n_result |= p_tas256x->write(p_tas256x, ch,
			TAS256X_INTERRUPTMASKREG1,
			TAS256X_INTERRUPTMASKREG1_DISABLE);
	}

	return n_result;
}

int tas256x_interrupt_read(struct tas256x_priv *p_tas256x,
	int *intr1, int *intr2, int ch)
{
	int n_result = 0;

	n_result = p_tas256x->read(p_tas256x, ch,
		TAS256X_LATCHEDINTERRUPTREG0, intr1);
	n_result |= p_tas256x->read(p_tas256x, ch,
		TAS256X_LATCHEDINTERRUPTREG1, intr2);

	return n_result;
}

int tas256x_icn_disable(struct tas256x_priv *p_tas256x, int disable, int ch)
{
	int n_result = 0;

	if (disable) { /*Disable*/
		n_result = p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_ICN_SW_REG,
			TAS256X_ICN_SW_MASK,
			TAS256X_ICN_SW_DISABLE);
		pr_info("%s %s: ICN Disable!\n",  LOG_TAG, __func__);
	} else { /*Enable*/
		n_result = p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_ICN_SW_REG,
			TAS256X_ICN_SW_MASK,
			TAS256X_ICN_SW_ENABLE);
		pr_info("%s %s: ICN Enable!\n",  LOG_TAG, __func__);
	}

	if (n_result == 0)
		p_tas256x->icn_sw = disable;

	return n_result;
}

/* icn_hysteresis & p_icn_thresholds*/
int tas256x_icn_data(struct tas256x_priv *p_tas256x, int ch)
{
	int n_result = 0;

	n_result = p_tas256x->bulk_write(p_tas256x, ch,
		TAS256X_ICN_THRESHOLD_REG,
		p_icn_threshold,
		sizeof(p_icn_threshold));
	n_result |= p_tas256x->bulk_write(p_tas256x, ch,
		TAS256X_ICN_HYSTERESIS_REG,
		p_icn_hysteresis,
		sizeof(p_icn_hysteresis));

	return n_result;
}

int tas256x_icn_config(struct tas256x_priv *p_tas256x, int value, int ch)
{
	int n_result = 0;

	n_result = p_tas256x->write(p_tas256x, ch,
		TAS256X_REG(0, 253, 13), 0x0d);

	n_result |= p_tas256x->write(p_tas256x, ch,
		TAS256X_REG(0, 253, 25), 0x80);

	return n_result;
}

int tas256x_boost_volt_update(struct tas256x_priv *p_tas256x, int value,
	int ch)
{
	int n_result = 0;

	if (value == DEVICE_TAS2558) {
		/* Max voltage to 9V */
		/*TODO: Need to be fixed*/
		n_result = p_tas256x->update_bits(p_tas256x, ch,
			TAS2562_BOOSTCONFIGURATION2,
			TAS2562_BOOSTCONFIGURATION2_BOOSTMAXVOLTAGE_MASK,
			0x7);
		n_result |= p_tas256x->update_bits(p_tas256x, ch,
			TAS2562_PLAYBACKCONFIGURATIONREG0,
			TAS2562_PLAYBACKCONFIGURATIONREG0_AMPLIFIERLEVEL51_MASK,
			0xd << 1);
	} else if (value == DEVICE_TAS2562) {
		/*TODO: ??*/
	} else if (value == DEVICE_TAS2564) {
		/*Update Channel Gain to 16dBV*/
		n_result = p_tas256x->update_bits(p_tas256x, ch,
			TAS2564_PLAYBACKCONFIGURATIONREG0,
			TAS2564_PLAYBACKCONFIGURATION_AMP_LEVEL_MASK,
			TAS2564_AMP_LEVEL_16dBV);
	}
	return n_result;
}

int tas256x_set_misc_config(struct tas256x_priv *p_tas256x, int value, int ch)
{
	int n_result = 0;

	n_result = p_tas256x->write(p_tas256x, ch,
		TAS256X_MISCCONFIGURATIONREG0, 0xcf);

	return n_result;

}

int tas256x_set_tx_config(struct tas256x_priv *p_tas256x, int value, int ch)
{
	int n_result = 0;

#if IS_ENABLED(CONFIG_PLATFORM_EXYNOS)
	if (p_tas256x->mn_channels == 2) {
		n_result = p_tas256x->write(p_tas256x, channel_left,
				TAS256X_TDMConfigurationReg4, 0xf3);
		if (n_result < 0)
			return n_result;
		n_result = p_tas256x->write(p_tas256x, channel_right,
				TAS256X_TDMConfigurationReg4, 0x13);
		if (n_result < 0)
			return n_result;
	} else {
		n_result = p_tas256x->write(p_tas256x, channel_both,
				TAS256X_TDMConfigurationReg4, 0x03);
		if (n_result < 0)
			return n_result;
	}
#else
	if (p_tas256x->mn_channels == 2) {
		n_result = p_tas256x->write(p_tas256x, channel_left,
			TAS256X_TDMConfigurationReg4, 0xf1);
		if (n_result < 0)
			return n_result;
		n_result = p_tas256x->write(p_tas256x, channel_right,
				TAS256X_TDMConfigurationReg4, 0x11);
		if (n_result < 0)
			return n_result;
	} else {
		n_result = p_tas256x->write(p_tas256x, channel_both,
				TAS256X_TDMConfigurationReg4, 0x01);
		if (n_result < 0)
			return n_result;
	}
#endif

	return n_result;
}

int tas256x_set_clock_config(struct tas256x_priv *p_tas256x, int value, int ch)
{
	int n_result = 0;

	if (p_tas256x->mn_fmt_mode == 2) {/*TDM Mode*/
		n_result = p_tas256x->write(p_tas256x, ch,
			TAS256X_CLOCKCONFIGURATION, 0x19);
	} else {
		n_result = p_tas256x->write(p_tas256x, ch,
			TAS256X_CLOCKCONFIGURATION, 0x0c);
	}
	/* Increase the clock halt timer to 838ms to avoid
	 * TDM Clock errors during playback start/stop
	 */
	n_result |= p_tas256x->update_bits(p_tas256x, ch,
		TAS256X_INTERRUPTCONFIGURATION,
		TAS256X_CLOCK_HALT_TIMER_MASK,
		TAS256X_CLOCK_HALT_838MS);

	return n_result;
}

int tas256x_set_classH_config(struct tas256x_priv *p_tas256x, int value,
	int ch)
{
	int n_result = 0;

	n_result = tas256x_i2c_load_data(p_tas256x, ch,
			p_tas256x_classh_d_data);

	return n_result;
}

int tas256x_HPF_FF_Bypass(struct tas256x_priv *p_tas256x, int value, int ch)
{
	int n_result = 0;

	n_result = p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_PLAYBACKCONFIGURATIONREG0,
			TAS256X_PLAYBACKCONFIGURATIONREG0_DCBLOCKER_MASK,
			TAS256X_PLAYBACKCONFIGURATIONREG0_DCBLOCKER_DISABLED);

	return n_result;
}

int tas256x_HPF_FB_Bypass(struct tas256x_priv *p_tas256x, int value, int ch)
{
	int n_result = 0;

	n_result = p_tas256x->bulk_write(p_tas256x, ch,
		TAS256X_HPF, HPF_reverse_path,
		ARRAY_SIZE(HPF_reverse_path));

	return n_result;

}

int tas56x_software_reset(struct tas256x_priv *p_tas256x, int ch)
{
	int n_result = 0;

	n_result = p_tas256x->write(p_tas256x, ch, TAS256X_SOFTWARERESET,
		TAS256X_SOFTWARERESET_SOFTWARERESET_RESET);

	msleep(20);

	return n_result;
}

int tas256x_zero_sar_delay(struct tas256x_priv *p_tas256x, int ch)
{
	int n_result = 0;

	n_result = p_tas256x->write(p_tas256x, ch, TAS256X_SAR_CFG_REG,
		0x28);
	return n_result;
}

int tas256x_update_dmin(struct tas256x_priv *p_tas256x, int ch)
{
	int n_result = 0;

	/* Update DMIN Sweep setting */
	n_result = p_tas256x->write(p_tas256x, ch, TAS256X_DMIN_SWEEP,
		0x04);
	/* Unlock Test Page */
	n_result |= p_tas256x->write(p_tas256x, ch, TAS256X_TESTPAGECONFIGURATION,
		0xd);
	/* Update DMIN Value */
	n_result |= p_tas256x->write(p_tas256x, ch, TAS256X_DMIN_VALUE,
		0x20);
	return n_result;
}

int tas256x_interrupt_determine(struct tas256x_priv *p_tas256x, int ch,
	int int1status, int int2status)
{
	int mn_err_code = 0;

	if (((int1status & 0x7) != 0)
		|| ((int2status & 0x0f) != 0)) {

		if (int1status &
			TAS256X_LATCHEDINTERRUPTREG0_TDMCLOCKERRORSTICKY_INTERRUPT) {
			mn_err_code |= ERROR_CLOCK;
			pr_err("%s TDM clock error!\n", LOG_TAG);
		}

		if (int1status &
			TAS256X_LATCHEDINTERRUPTREG0_OCEFLAGSTICKY_INTERRUPT) {
			mn_err_code |= ERROR_OVER_CURRENT;
			pr_err("%s SPK over current!\n", LOG_TAG);
		}

		if (int1status &
			TAS256X_LATCHEDINTERRUPTREG0_OTEFLAGSTICKY_INTERRUPT) {
			mn_err_code |= ERROR_DIE_OVERTEMP;
			pr_err("%s die over temperature!\n", LOG_TAG);
		}

		if (int2status &
			TAS256X_LATCHEDINTERRUPTREG1_VBATOVLOSTICKY_INTERRUPT) {
			mn_err_code |= ERROR_OVER_VOLTAGE;
			pr_err("%s SPK over voltage!\n", LOG_TAG);
		}

		if (int2status &
			TAS256X_LATCHEDINTERRUPTREG1_VBATUVLOSTICKY_INTERRUPT) {
			mn_err_code |= ERROR_UNDER_VOLTAGE;
			pr_err("%s SPK under voltage!\n", LOG_TAG);
		}

		if (int2status &
			TAS256X_LATCHEDINTERRUPTREG1_BROWNOUTFLAGSTICKY_INTERRUPT) {
			mn_err_code |= ERROR_BROWNOUT;
			pr_err("%s brownout!\n", LOG_TAG);
		}
	} else {
		return 0;
	}

	return mn_err_code;
}

int tas56x_get_chipid(struct tas256x_priv *p_tas256x, int *chipid, int ch)
{
	int n_result = 0;

	n_result = p_tas256x->read(p_tas256x, ch,
		TAS256X_CHIPID, chipid);

	return n_result;
}

int tas2564_rx_mode_update(struct tas256x_priv *p_tas256x, int rx_mode, int ch)
{
	int n_result = 0;

	if (rx_mode)
		n_result = p_tas256x->update_bits(p_tas256x,
			ch, TAS2564_PLAYBACKCONFIGURATIONREG0,
			TAS2564_PLAYBACKCONFIGURATIONREG_RX_SPKR_MODE_MASK,
			TAS2564_PLAYBACKCONFIGURATIONREG_RX_MODE);
	else
		n_result = p_tas256x->update_bits(p_tas256x,
			ch, TAS2564_PLAYBACKCONFIGURATIONREG0,
			TAS2564_PLAYBACKCONFIGURATIONREG_RX_SPKR_MODE_MASK,
			TAS2564_PLAYBACKCONFIGURATIONREG_SPKR_MODE);

	if (n_result == 0)
		p_tas256x->devs[ch-1]->rx_mode = rx_mode;

	return n_result;
}

int tas256x_update_playback_volume(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 57)) {
		n_result = p_tas256x->bulk_write(p_tas256x, ch,
			TAS256X_DVC_PCM,
			(char *)&(dvc_pcm[value][0]), sizeof(int));
		if (n_result == 0)
			p_tas256x->devs[ch-1]->dvc_pcm = value;
	}

	return n_result;
}

int tas256x_update_lim_max_attenuation(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 16)) {
		n_result = p_tas256x->bulk_write(p_tas256x, ch,
			TAS256X_LIM_MAX_ATN,
			(char *)&(lim_max_attn[value][0]), sizeof(int));
		if (n_result == 0)
			p_tas256x->devs[ch-1]->lim_max_attn = value;
	}

	return n_result;
}

int tas256x_update_lim_max_thr(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 27)) {
		n_result = p_tas256x->bulk_write(p_tas256x, ch,
			TAS256X_LIMB_TH_MAX,
			(char *)&(vbat_lim_max_thd[value][0]), sizeof(int));
		if (n_result == 0)
			p_tas256x->devs[ch-1]->lim_thr_max = value;
	}

	return n_result;
}

int tas256x_update_lim_min_thr(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 27)) {
		n_result = p_tas256x->bulk_write(p_tas256x, ch,
			TAS256X_LIMB_TH_MIN,
			(char *)&(vbat_lim_min_thd[value][0]), sizeof(int));
		if (n_result == 0)
			p_tas256x->devs[ch-1]->lim_thr_min = value;
	}

	return n_result;
}

int tas256x_update_lim_inflection_point(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 41)) {
		n_result = p_tas256x->bulk_write(p_tas256x, ch,
			TAS256X_LIMB_INF_PT,
			(char *)&(vbat_lim_infl_pt[value][0]), sizeof(int));
		if (n_result == 0)
			p_tas256x->devs[ch-1]->lim_infl_pt = value;
	}

	return n_result;
}

int tas256x_update_lim_slope(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 7)) {
		n_result = p_tas256x->bulk_write(p_tas256x, ch,
			TAS256X_LIMB_SLOPE,
			(char *)&(vbat_lim_track_slope[value][0]), sizeof(int));
		if (n_result == 0)
			p_tas256x->devs[ch-1]->lim_trk_slp = value;
	}

	return n_result;
}

int tas256x_update_bop_thr(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 16)) {
		n_result = p_tas256x->bulk_write(p_tas256x, ch,
			TAS256X_BOP_TH,
			(char *)&(bop_thd[value][0]), sizeof(int));
		if (n_result == 0)
			p_tas256x->devs[ch-1]->bop_thd = value;
	}
	return n_result;
}

int tas256x_update_bosd_thr(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 16)) {
		n_result = p_tas256x->bulk_write(p_tas256x, ch,
			TAS256X_BOSD_TH,
			(char *)&(bsd_thd[value][0]), sizeof(int));
		if (n_result == 0)
			p_tas256x->devs[ch-1]->bosd_thd = value;
	}

	return n_result;
}

int tas256x_update_boost_always_on(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 16)) {
		n_result = p_tas256x->bulk_write(p_tas256x, ch,
			TAS256X_BOOST_ALWAYS_ON,
			(char *)&(bop_always_on[value][0]), sizeof(int));
	}
	return n_result;
}

int tas256x_update_boost_always_off(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 16)) {
		n_result = p_tas256x->bulk_write(p_tas256x, ch,
			TAS256X_BOOST_ALWAYS_OFF,
			(char *)&(bop_always_off[value][0]), sizeof(int));
	}
	return n_result;
}
int tas256x_update_boost_voltage(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if (p_tas256x->devs[ch-1]->device_id == DEVICE_TAS2562) {
		if ((value >= 0) && (value < 16))
			n_result = p_tas256x->update_bits(p_tas256x,
				ch, TAS2562_BOOSTCONFIGURATION2,
				TAS2562_BOOSTCONFIGURATION2_BOOSTMAXVOLTAGE_MASK,
				(value+1) << TAS2562_BOOSTCONFIGURATION2_BOOSTMAXVOLTAGE_SHIFT);
	} else if (p_tas256x->devs[ch-1]->device_id == DEVICE_TAS2564) {
		if ((value >= 0) && (value < 32))
			n_result = p_tas256x->update_bits(p_tas256x,
				ch, TAS2564_BOOSTCONFIGURATION2,
				TAS2564_BOOSTCONFIGURATION2_BOOSTMAXVOLTAGE_MASK,
				(value+7) << TAS2562_BOOSTCONFIGURATION2_BOOSTMAXVOLTAGE_SHIFT);
	}

	if (n_result == 0)
		p_tas256x->devs[ch-1]->bst_vltg = value;

	return n_result;
}

int tas256x_update_ampoutput_level(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	/* 131020 - Fix to support min(0x0) and max value(0x1c)*/
	if ((value >= 0) && (value <= 0x1C)) {
		if (p_tas256x->devs[ch-1]->device_id == DEVICE_TAS2562) {
			n_result = p_tas256x->update_bits(p_tas256x,
				ch, TAS2562_PLAYBACKCONFIGURATIONREG0,
				TAS2562_PLAYBACKCONFIGURATIONREG0_AMPLIFIERLEVEL51_MASK,
				value << TAS2562_PLAYBACKCONFIGURATIONREG0_AMPLIFIERLEVEL51_SHIFT);
		} else if (p_tas256x->devs[ch-1]->device_id == DEVICE_TAS2564) {
				n_result = p_tas256x->update_bits(p_tas256x,
					ch, TAS2564_PLAYBACKCONFIGURATIONREG0,
					TAS2562_PLAYBACKCONFIGURATIONREG0_AMPLIFIERLEVEL51_MASK,
					value << TAS2562_PLAYBACKCONFIGURATIONREG0_AMPLIFIERLEVEL51_SHIFT);
		}
	}

	if (n_result == 0)
		p_tas256x->devs[ch-1]->ampoutput_lvl = value;

	return n_result;
}

int tas256x_update_current_limit(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 64))
		n_result = p_tas256x->update_bits(p_tas256x,
			ch, TAS256X_BOOSTCONFIGURATION4,
			TAS256X_BOOSTCONFIGURATION4_BOOSTCURRENTLIMIT_MASK,
			(value) << TAS256X_BOOSTCONFIGURATION4_BOOSTCURRENTLIMIT_SHIFT);

	if (n_result == 0)
		p_tas256x->devs[ch-1]->bst_ilm = value;

	return n_result;
}

int tas256x_update_limiter_enable(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 2))
		n_result = p_tas256x->update_bits(p_tas256x,
			ch, TAS256X_LIMITERCONFIGURATION0,
			TAS256X_LIMITER_ENABLE_MASK,
			(value) << TAS256X_LIMITER_ENABLE_SHIFT);

	if (n_result == 0)
		p_tas256x->devs[ch-1]->lim_switch = value;

	return n_result;
}

int tas256x_update_limiter_attack_rate(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 8))
		n_result = p_tas256x->update_bits(p_tas256x,
			ch, TAS256X_LIMITERCONFIGURATION0,
			TAS256X_LIMITER_ATTACKRATE_MASK,
			(value) << TAS256X_LIMITER_ATTACKRATE_SHIFT);

	if (n_result == 0)
		p_tas256x->devs[ch-1]->lim_att_rate = value;

	return n_result;
}

int tas256x_update_limiter_attack_step_size(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 4))
		n_result = p_tas256x->update_bits(p_tas256x,
			ch, TAS256X_LIMITERCONFIGURATION0,
			TAS256X_LIMITER_ATTACKSTEPSIZE_MASK,
			(value) << TAS256X_LIMITER_ATTACKSTEPSIZE_SHIFT);

	if (n_result == 0)
		p_tas256x->devs[ch-1]->lim_att_stp_size = value;

	return n_result;
}

int tas256x_update_limiter_release_rate(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 8))
		n_result = p_tas256x->update_bits(p_tas256x,
			ch, TAS256X_LIMITERCONFIGURATION1,
			TAS256X_LIMITER_RELEASERATE_MASK,
			(value) << TAS256X_LIMITER_RELEASERATE_SHIFT);

	if (n_result == 0)
		p_tas256x->devs[ch-1]->lim_rel_rate = value;

	return n_result;
}

int tas256x_update_limiter_release_step_size(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 4))
		n_result = p_tas256x->update_bits(p_tas256x,
			ch, TAS256X_LIMITERCONFIGURATION1,
			TAS256X_LIMITER_RELEASESTEPSIZE_MASK,
			(value) << TAS256X_LIMITER_RELEASESTEPSIZE_SHIFT);

	if (n_result == 0)
		p_tas256x->devs[ch-1]->lim_rel_stp_size = value;

	return n_result;
}

int tas256x_update_bop_enable(struct tas256x_priv *p_tas256x, int value,
	int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 2))
		n_result = p_tas256x->update_bits(p_tas256x,
			ch, TAS256X_BOPCONFIGURATION0,
			TAS256X_BOP_ENABLE_MASK,
			(value) << TAS256X_BOP_ENABLE_SHIFT);

	if (n_result == 0)
		p_tas256x->devs[ch-1]->bop_enable = value;

	return n_result;
}

int tas256x_update_bop_mute(struct tas256x_priv *p_tas256x, int value,
	int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 2))
		n_result = p_tas256x->update_bits(p_tas256x,
			ch, TAS256X_BOPCONFIGURATION0,
			TAS256X_BOP_MUTE_MASK,
			(value) << TAS256X_BOP_MUTE_SHIFT);

	if (n_result == 0)
		p_tas256x->devs[ch-1]->bop_mute = value;

	return n_result;
}

int tas256x_update_bop_shutdown_enable(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 2))
		n_result = p_tas256x->update_bits(p_tas256x,
			ch, TAS256X_BOPCONFIGURATION0,
			TAS256X_BOP_SHUTDOWN_ENABLE_MASK,
			(value) << TAS256X_BOP_SHUTDOWN_ENABLE_SHIFT);

	if (n_result == 0)
		p_tas256x->devs[ch-1]->bosd_enable = value;

	return n_result;
}

int tas256x_update_bop_attack_rate(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 8))
		n_result = p_tas256x->update_bits(p_tas256x,
			ch, TAS256X_BOPCONFIGURATION1,
			TAS256X_BOP_ATTACKRATE_MASK,
			(value) << TAS256X_BOP_ATTACKRATE_SHIFT);

	if (n_result == 0)
		p_tas256x->devs[ch-1]->bop_att_rate = value;

	return n_result;
}

int tas256x_update_bop_attack_step_size(struct tas256x_priv *p_tas256x,
	int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 4))
		n_result = p_tas256x->update_bits(p_tas256x,
			ch, TAS256X_BOPCONFIGURATION1,
			TAS256X_BOP_ATTACKSTEPSIZE_MASK,
			(value) << TAS256X_BOP_ATTACKSTEPSIZE_SHIFT);

	if (n_result == 0)
		p_tas256x->devs[ch-1]->bop_att_stp_size = value;

	return n_result;
}

int tas256x_update_bop_hold_time(struct tas256x_priv *p_tas256x, int value,
	int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 8))
		n_result = p_tas256x->update_bits(p_tas256x,
			ch, TAS256X_BOPCONFIGURATION1,
			TAS256X_BOP_HOLDTIME_MASK,
			(value) << TAS256X_BOP_HOLDTIME_SHIFT);

	if (n_result == 0)
		p_tas256x->devs[ch-1]->bop_hld_time = value;

	return n_result;
}

int tas256x_update_vbat_lpf(struct tas256x_priv *p_tas256x, int value, int ch)
{
	int n_result = -1;

	if ((value >= 0) && (value < 4)) {
		n_result = p_tas256x->update_bits(p_tas256x,
			ch, TAS256X_VBATFILTER,
			TAS256X_VBAT_LPF_MASK,
			(value) << TAS256X_VBAT_LPF_SHIFT);
		if (n_result == 0)
			p_tas256x->devs[ch-1]->vbat_lpf = value;
	}

	return n_result;
}

int tas256x_update_rx_cfg(struct tas256x_priv *p_tas256x, int value, int ch)
{
	int n_result = -1;
	int data = -1;

	switch (value) {
	case 0:
		data = TAS256X_TDMCONFIGURATIONREG2_RXSCFG54_MONO_I2C;
		break;
	case 1:
		data = TAS256X_TDMCONFIGURATIONREG2_RXSCFG54_MONO_LEFT;
		break;
	case 2:
		data = TAS256X_TDMCONFIGURATIONREG2_RXSCFG54_MONO_RIGHT;
		break;
	case 3:
		data = TAS256X_TDMCONFIGURATIONREG2_RXSCFG54_STEREO_DOWNMIX;
		break;
	}

	if (data >= 0)
		n_result = p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_TDMCONFIGURATIONREG2,
			TAS256X_TDMCONFIGURATIONREG2_RXSCFG54_MASK,
			data);

	if (n_result == 0)
		p_tas256x->devs[ch-1]->rx_cfg = value;

	return n_result;
}

int tas256x_update_classh_timer(struct tas256x_priv *p_tas256x, int value,
	int ch)
{
	int n_result = -1;

	/*value = 255/(0.07*value)*/
	if (p_tas256x->devs[ch-1]->device_id == DEVICE_TAS2564) {
		n_result =
			p_tas256x->write(p_tas256x, ch, TAS2564_CLASSH_SLOPE,
			classH_slope[value]);
	}

	n_result = p_tas256x->bulk_write(p_tas256x, ch,
			TAS256X_CLASSH_TIMER,
			(char *)&(classH_timer[value][0]), sizeof(int));

	value += 8; /*Offset*/

	n_result |= p_tas256x->update_bits(p_tas256x, ch,
		TAS256X_CLASSH_TIMER_RELEASE,
		TAS256X_CLASSH_TIMER_RELEASE_MASK,
		value+1);

	value -= 8;

	if (n_result == 0)
		p_tas256x->devs[ch-1]->classh_timer = value;

	return n_result;
}

int tas256x_enable_receiver_mode(struct tas256x_priv *p_tas256x, int enable,
	int ch)
{
	int n_result = -1;

	if (enable) {
		/*Unlock test page*/
		n_result =
			p_tas256x->write(p_tas256x, ch, TAS256X_TEST_PAGE_LOCK,
				0xd);
		/*Keep DAC modulator dither to minimum*/
		n_result |=
			p_tas256x->write(p_tas256x, ch, TAS256X_DAC_MODULATOR,
				0xc0);
		/*Force DAC unmute*/
		n_result |=
			p_tas256x->write(p_tas256x, ch, TAS256X_DAC_MUTE,
				0x04);
		/*Force the device in idle channel mode*/
		n_result |=
			p_tas256x->write(p_tas256x, ch, TAS256X_ICN_MODE,
				0x14);
		/* ICN improvement*/
		n_result |=
			p_tas256x->write(p_tas256x, ch, TAS256X_ICN_IMPROVE,
				0x1f);

		/* Disable MSB DEM*/
		n_result |=
			p_tas256x->write(p_tas256x, ch, TAS256X_MSB_DEM,
				0x01);

		/* Disable Dither*/
		n_result |=
			p_tas256x->write(p_tas256x, ch, TAS256X_DITHER,
				0x00);

		/* Disable LSB DEM*/
		n_result |=
			p_tas256x->write(p_tas256x, ch, TAS256X_LSB_DEM,
				0xfc);

		/*Enable IRQ pull-up, disable SSM*/
		n_result |=
			p_tas256x->write(p_tas256x, ch,
				TAS256X_MISCCONFIGURATIONREG0,
				0xca);
		/*Shutdown IV sense ADC as speaker protection algorithm
		 *is bypassed when in Receiver Mode
		 */
		n_result |=
			p_tas256x->write(p_tas256x, ch, TAS256X_ICN_SW_REG,
				0x00);
	} else {
		/*Unlock test page*/
		n_result =
			p_tas256x->write(p_tas256x, ch, TAS256X_TEST_PAGE_LOCK,
				0xd);
		/*Keep DAC modulator dither to minimum*/
		n_result |=
			p_tas256x->write(p_tas256x, ch, TAS256X_DAC_MODULATOR,
				0x00);
		/*Force DAC unmute*/
		n_result |=
			p_tas256x->write(p_tas256x, ch, TAS256X_DAC_MUTE,
				0x00);
		/*Force the device in idle channel mode*/
		n_result |=
			p_tas256x->write(p_tas256x, ch, TAS256X_ICN_MODE,
				0x10);
		/* ICN improvement*/
		n_result |=
			p_tas256x->write(p_tas256x, ch, TAS256X_ICN_IMPROVE,
				0x1d);

		/* Enable MSB DEM to Default value*/
		n_result |=
			p_tas256x->write(p_tas256x, ch, TAS256X_MSB_DEM,
				0x21);

		/* Enable Dither*/
		n_result |=
			p_tas256x->write(p_tas256x, ch, TAS256X_DITHER,
				0x01);

		/* Enable LSB DEM*/
		n_result |=
			p_tas256x->write(p_tas256x, ch, TAS256X_LSB_DEM,
				0xdc);

		/*Enable IRQ pull-up, disable SSM*/
		n_result |=
			p_tas256x->write(p_tas256x, ch,
				TAS256X_MISCCONFIGURATIONREG0,
				0xcf);
		/*Mask reaction of idle channel detect on IV sense*/
		n_result |= p_tas256x->update_bits(p_tas256x, ch,
			TAS256X_ICN_SW_REG,
			TAS256X_ICN_IVSENSE_MASK,
			0x00);
		n_result |=
			tas256x_icn_disable(p_tas256x, p_tas256x->icn_sw, ch);
	}

	if (n_result == 0)
		p_tas256x->devs[ch-1]->receiver_enable = enable;

	return n_result;
}

int tas256x_enable_emphasis_filter(struct tas256x_priv *p_tas256x, int ch,
	int samplerate)
{
	int n_result = 0;
	int i = 0;

	if (ch == channel_both) {
		for (i = 0; i < p_tas256x->mn_channels; i++) {
			if ((p_tas256x->devs[i]->device_id == DEVICE_TAS2562)
				&& (samplerate == 48000)) {
				n_result = tas256x_i2c_load_data(p_tas256x, i+1,
					p_tas2562_emphasis_filter_48kHZ);
			}
		}
	} else {
		if ((p_tas256x->devs[ch-1]->device_id == DEVICE_TAS2562)
			&& (samplerate == 48000)) {
			n_result = tas256x_i2c_load_data(p_tas256x, ch,
				p_tas2562_emphasis_filter_48kHZ);
		}
	}

	return n_result;
}
