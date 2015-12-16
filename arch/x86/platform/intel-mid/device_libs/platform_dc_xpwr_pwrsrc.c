
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <asm/dc_xpwr_pwrsrc.h>

static struct dc_xpwr_pwrsrc_pdata pdata;

static void *get_platform_data(void)
{
	/*
	 * set en_chrg_det to true if the
	 * D+/D- lines are connected to
	 * PMIC itself.
	 */
	pdata.en_chrg_det = false;
	return &pdata;
}

void *dc_xpwr_pwrsrc_pdata(void *info)
{
	return get_platform_data();
}
