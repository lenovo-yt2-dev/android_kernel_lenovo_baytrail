/*
 * Crystal cove page0  -- Device access for Crystal cove
 * PMIC page0 register or memory map.
 *
 * Copyright (c) 2013, Intel Corporation.
 *
 * Author: Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <asm/uaccess.h>

#include <linux/debugfs.h>
#include <linux/mfd/intel_mid_pmic.h>

/* No of times we should retry on -EAGAIN error */
#define NR_RETRY_CNT		3

static struct i2c_client *ccpage0;

static int ccpage0_write_reg8(struct i2c_client *client, u8 reg, u8 value)
{
	int ret, i;

	for (i = 0; i < NR_RETRY_CNT; i++) {
		ret = i2c_smbus_write_byte_data(client, reg, value);
		if (ret < 0)
			continue;
		else
			break;
	}

	if (ret < 0)
		dev_err(&client->dev, "I2C SMbus Write error:%d\n", ret);

	return ret;
}

static int ccpage0_read_reg8(struct i2c_client *client, u8 reg)
{
	int ret, i;

	for (i = 0; i < NR_RETRY_CNT; i++) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0)
			continue;
		else
			break;
	}

	if (ret < 0)
		dev_err(&client->dev, "I2C SMbus Read error:%d\n", ret);

	return ret;
}
/*
* add by axs for pmic hardware verification
*/
#ifdef CONFIG_DEBUG_FS

u32 gpmic_reg_addr, gpmic_reg_value;
u32 gpmic_rom_addr, gpmic_rom_value;

#define PMIC_5E_I2C_ACTL 	0xC2
#define PMIC_5E_I2C_CFG 	0xC3
#define PMIC_5E_I2C_ADDR_H 	0xC4
#define PMIC_5E_I2C_ADDR_L 	0xC5
#define PMIC_5E_I2C_DATA 	0xC6

#define MAX_EEPROM_IMAGE_SIZE 	32

static ssize_t pmic_register_write(struct file *file, const char __user *ubuf,
						size_t count, loff_t *ppos)
{
	char buf[32];
	long ret;
	int i = 0;
	long reg, val;
	
	memset(buf, 0, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	buf[min_t(size_t, sizeof(buf) - 1, count)] = 0;
	
	for(i = 0; i < strlen(buf); i++){
		if(buf[i] == 'r' || buf[i] == 'w')
			break;
	}

	if(buf[i] == 'r'){
		for(;i < count; i++){
			if(buf[i] == '0'){
				if(buf[i+1] == 'x' || buf[i+1] == 'X')
					break;
			}
		}
		i += 2;
		
		ret = strict_strtol(buf + i, 16, &reg);
		if (ret)
			return ret;
		
		val = ccpage0_read_reg8(ccpage0, reg);
	
		pr_info("PMIC_REG(5E): R REG[0x%02X] = 0x%02X\n", (u32)reg, (u32)val);
		gpmic_reg_addr = reg;
		gpmic_reg_value = val;
	}
	else if(buf[i] == 'w'){
		int start = 0;

		for(;i < count; i++){
			if(buf[i] == '0'){
				if(buf[i+1] == 'x' || buf[i+1] == 'X')
					break;
			}
		}
		i += 2;
		start = i;

		for(;i < count; i++){
			if(buf[i] == '='){
				buf[i] = 0;
				break;
			}
		}
		
		ret = strict_strtol(buf + start, 16, &reg);
		if (ret){
			pr_info("PMIC_REG(5E): fail get register\n");
			return ret;
		}
		
		for(;i < count; i++){
			if(buf[i] == '0'){
				if(buf[i+1] == 'x' || buf[i+1] == 'X')
					break;
			}
		}
		i += 2;
				
		ret = strict_strtol(buf + i, 16, &val);
		if (ret){
			pr_info("PMIC_REG(5E): fail get value\n");
			return ret;
		}
		
		pr_info("PMIC_REG(5E): W REG[0x%02X] = 0x%02x\n", (u32)reg, (u32)val);

		ret = ccpage0_write_reg8(ccpage0, reg, val);
		if (ret){
			pr_info("PMIC_REG(5E): write failed ret = %d\n", (u32)ret);
			return ret;
		}
		
		gpmic_reg_addr = reg;
		gpmic_reg_value = val;
	}
	else{
		pr_info("Usage:\n");
		pr_info("echo \"r 0xREG\" > d/intel_pmic/pmic_5e \n");
		pr_info("echo \"w 0xREG=0xVAL\" > d/intel_pmic/pmic_5e \n");
	}
	return count;
}

static ssize_t pmic_register_read(struct file *file, char __user *userbuf, size_t bytes, loff_t *off){
	char buf[32] = { 0 };

	sprintf(buf, "0x%02X=0x%02X\n", (u8)gpmic_reg_addr, (u8)gpmic_reg_value);
	pr_info("PMIC_REG_READ: %s\n", buf);
	
	if (copy_to_user(userbuf, buf, strlen(buf)));
		return -EFAULT;
		
	return strlen(buf);
}

static ssize_t pmic_eeprom_write(struct file *file, const char __user *ubuf, size_t count, loff_t *ppos)
{
	char buf[128] = {0};
	long ret;
	int i = 0;
	long eeprom_addr, val;
	u8 retry = 0;

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	buf[min_t(size_t, sizeof(buf) - 1, count)] = 0;
	
	for(i = 0; i < strlen(buf); i++){
		if(buf[i] == 'r' || buf[i] == 'w')
			break;
	}

	if(buf[i] == 'r'){
		
		for(;i < count; i++){
			if(buf[i] == '0'){
				if(buf[i+1] == 'x' || buf[i+1] == 'X')
					break;
			}
		}	
		i += 2;
		
		ret = strict_strtol(buf + i, 16, &eeprom_addr);
		if (ret)
			return ret;
		
		//configure EEPROM slave address
		ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_CFG, 0xD0);
			
		//Clear read / write error, and set clock to 400KHz
		ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_ACTL, 0x0E);

		//configure EEPROM operation address
		ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_ADDR_H, (eeprom_addr >> 8) & 0xff);
		ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_ADDR_L, (eeprom_addr) & 0xff);
		msleep(100);
		
		val = ccpage0_read_reg8(ccpage0, PMIC_5E_I2C_ACTL);
		
		//read request
		ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_ACTL, val | 0x01); 

		//wait read finish
		do{
			val = ccpage0_read_reg8(ccpage0, PMIC_5E_I2C_ACTL);
			retry ++;
			msleep(10);
			if(retry > 20){
				pr_info("PMIC_EEPROM: Fail detect read complete (ACTL = 0x%02X)\n", (u32)val);
				break;
			}	
		}while(val & 0x01);

		val = ccpage0_read_reg8(ccpage0, PMIC_5E_I2C_DATA);
		gpmic_rom_addr = eeprom_addr;
		gpmic_rom_value = val;

		pr_info("PMIC_EEPROM: R ADDR[0x%04X] = 0x%02x\n", (u32)eeprom_addr, (u32)val);
		
		pr_info("PMIC_EEPROM: SUCCESS\n");

	}
	else if(buf[i] == 'w'){
		int start = 0;
		
		for(;i < count; i++){
			if(buf[i] == '0'){
				if(buf[i+1] == 'x' || buf[i+1] == 'X')
					break;
			}
		}	
		i += 2;
		start = i;

		for(;i < count; i++){
			if(buf[i] == '='){
				buf[i] = 0;
				break;
			}
		}	
		
		ret = strict_strtol(buf + start, 16, &eeprom_addr);
		if (ret){
			pr_info("PMIC_EEPROM: Fail get register\n");
			return ret;
		}
		
		for(;i < count; i++){
			if(buf[i] == '0'){
				if(buf[i+1] == 'x' || buf[i+1] == 'X')
					break;
			}
		}	
		i += 2;
		
		ret = strict_strtol(buf + i, 16, &val);
		if (ret){
			pr_info("PMIC_EEPROM: Fail get value\n");
			return ret;
		}
		
		pr_info("PMIC_EEPROM: W ADDR[0x%04X] = 0x%02X\n", (u32)eeprom_addr, (u32)val);

		ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_CFG, 0xD0);
		ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_ACTL, 0x0E);
		ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_ADDR_H, (eeprom_addr >> 8) & 0xff);
		ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_ADDR_L, (eeprom_addr) & 0xff);
		ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_DATA, val);
		
		//wait write finish
		do{
			val = ccpage0_read_reg8(ccpage0, PMIC_5E_I2C_ACTL);
			retry ++;
			if(retry > 20){
				pr_info("PMIC_EEPROM: Fail detect read complete (ACTL = 0x%02X) retry %d\n", 
										(u32)val, retry);
				return count;
			}	
		}while(val & 0x40);

		gpmic_rom_addr = eeprom_addr;
		gpmic_rom_value = val;

		pr_info("PMIC_EEPROM: SUCCESS\n");

	}
	else{
		pr_info("Usage:\n");
		pr_info("echo \"r 0xADDR\" > d/intel_pmic/pmic_eeprom \n");
		pr_info("echo \"w 0xADDR=0xVAL\" > d/intel_pmic/pmic_eeprom \n");
	}
	
_exit:
	return count;
}

static ssize_t pmic_eeprom_read(struct file *file, char __user *userbuf, size_t bytes, loff_t *off){
	char buf[32] = { 0 };

	sprintf(buf, "0x%04X=0x%02X\n", (u16)gpmic_rom_addr, (u8)gpmic_rom_value);
	pr_info("PMIC_ROM_READ: %s\n", buf);	
	if (copy_to_user(userbuf, buf, strlen(buf)));
		return -EFAULT;
		
	return strlen(buf);
}

static const struct file_operations pmic_register_ops = {
	.owner		= THIS_MODULE,
	.open		= simple_open,
	.write		= pmic_register_write,
	.read		= pmic_register_read,
};

static const struct file_operations pmic_eeprom_ops = {
	.owner		= THIS_MODULE,
	.open		= simple_open,
	.write		= pmic_eeprom_write,
	.read		= pmic_eeprom_read,
};

#endif





static void init_page0_registers(void)
{
	ccpage0_write_reg8(ccpage0, 0xf9, 0xad);
	ccpage0_write_reg8(ccpage0, 0xf9, 0xeb);
}

#define PMIC_5E_I2C_ACTL   0xC2
#define PMIC_5E_I2C_CFG    0xC3
#define PMIC_5E_I2C_ADDR_H 0xC4
#define PMIC_5E_I2C_ADDR_L 0xC5
#define PMIC_5E_I2C_DATA   0xC6
struct eeprom_config{
	u16 addr;
	u8 value;
};

struct eeprom_config eeprom_cfg [] = {
	{ 0x00FF, 0x5B },

	{ 0x0000, 0x59 },
	{ 0x0001, 0x34 },

	{ 0x0002, 0xb0 },
	{ 0x0003, 0x24 },//LOWBAT1
	{ 0x0004, 0x86 },//value

	{ 0x0005, 0xb0 },
	{ 0x0006, 0x23 },//LOWBAT0
	{ 0x0007, 0xcb },//value

	{ 0x0008, 0xb8 },
	{ 0x0009, 0xdb },
	{ 0x000a, 0x02 },

	{ 0x000b, 0xf8 },
	{ 0x000c, 0x5d },
	{ 0x000d, 0x23 },
	{ 0x000e, 0x01 },

	{ 0x5A5A, 0x5B }
};
/*
* EEPROM Write Protect set function.
* wp_stat = true; WP enable
* wp_stat = false; WP disable
*/
static void eeprom_wp_check(bool wp_stat)
{
	int ret;

	if(wp_stat){
		intel_mid_pmic_writeb(0x41,0x35);
		mdelay(5);
		ret = intel_mid_pmic_readb(0x41);
		printk("%s: WP status new is 0x%x\n",__func__,ret);
	}else{
		intel_mid_pmic_writeb(0x41,0x34);
		mdelay(5);
		ret = intel_mid_pmic_readb(0x41);
		printk("%s: WP status new is 0x%x\n",__func__,ret);
	}
}

static void init_pmic_eeprom(void)
{

	int i = 0;
	int retry = 0, ret = 0;;
	pr_info("PMIC_EEPROM: init start\n");
	eeprom_wp_check(false);

	ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_CFG, 0xD0);
	ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_ACTL, 0x0E);

	ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_ADDR_H, (eeprom_cfg[0].addr >> 8) & 0xff);
	ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_ADDR_L, (eeprom_cfg[0].addr) & 0xff);
	ret = ccpage0_read_reg8(ccpage0, PMIC_5E_I2C_ACTL);

	//request reading
	ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_ACTL, ret | 0x01);
	do{
		ret = ccpage0_read_reg8(ccpage0, PMIC_5E_I2C_ACTL);
		retry ++;
		msleep(100);
		if(retry > 20){
			pr_info("PMIC_EEPROM: read fail (%02X)\n", ret);
			return;
		}
	}while(ret & 0x01);
	
	ret = ccpage0_read_reg8(ccpage0, PMIC_5E_I2C_DATA);

	printk("%s: addr 0xff , value 0x%x\n",__func__,ret);
	if(ret == eeprom_cfg[0].value){
		pr_info("PMIC_EEPROM: already initialized (0x%04X = 0x%02X)\n", eeprom_cfg[0].addr, eeprom_cfg[0].value);
		return;
	}
	for(i = 0; i < sizeof(eeprom_cfg); i++ ){

		if(eeprom_cfg[i].addr == 0x5A5A && eeprom_cfg[i].value == 0x5B)
			break;
		
		pr_info("PMIC_EEPROM: 0x%04X = 0x%02X\n", eeprom_cfg[i].addr, eeprom_cfg[i].value);

		ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_ADDR_H, (eeprom_cfg[i].addr >> 8) & 0xff);
		ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_ADDR_L, (eeprom_cfg[i].addr) & 0xff);
		ccpage0_write_reg8(ccpage0, PMIC_5E_I2C_DATA, eeprom_cfg[i].value);
		
		do{
			ret = ccpage0_read_reg8(ccpage0, PMIC_5E_I2C_ACTL);
			retry ++;
			if(retry > 20){
				pr_info("PMIC_EEPROM: write fail (%02X)\n", (u32)ret);
				return;
			}	
			msleep(10);
		}while(ret & 0x40);

	}
	pr_info("PMIC_EEPROM: init done\n");

}

static int ccpage0_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev,
				"SM bus doesn't support BYTE transactions\n");
		return -EIO;
	}

	ccpage0 = client;
	init_page0_registers();

	eeprom_wp_check(true);

	init_pmic_eeprom();

	eeprom_wp_check(true);
//	pmic_create_debug_fs("pmic_5e", &pmic_register_ops);
//	pmic_create_debug_fs("pmic_eeprom", &pmic_eeprom_ops);
	return 0;
}

static int ccpage0_i2c_remove(struct i2c_client *i2c)
{
	return 0;
}

static int ccpage0_suspend(struct device *dev)
{
	return 0;
}

static int ccpage0_resume(struct device *dev)
{
	return 0;
}

static int ccpage0_runtime_suspend(struct device *dev)
{
	return 0;
}

static int ccpage0_runtime_resume(struct device *dev)
{
	return 0;
}

static int ccpage0_runtime_idle(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops ccpage0_pm_ops = {
		SET_SYSTEM_SLEEP_PM_OPS(ccpage0_suspend,
				ccpage0_resume)
		SET_RUNTIME_PM_OPS(ccpage0_runtime_suspend,
				ccpage0_runtime_resume,
				ccpage0_runtime_idle)
};

static const struct i2c_device_id ccpage0_i2c_id[] = {
	{ "crystalcove_page0", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ccpage0_i2c_id);

static struct acpi_device_id pmic_acpi_match[] = {
	{ "TEST0002", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, pmic_acpi_match);

static struct i2c_driver ccpage0_i2c_driver = {
	.driver = {
		.name = "intel_crystalcove_page0",
		.owner = THIS_MODULE,
		.pm = &ccpage0_pm_ops,
		.acpi_match_table = ACPI_PTR(pmic_acpi_match),
	},
	.probe = ccpage0_i2c_probe,
	.remove = ccpage0_i2c_remove,
	.id_table = ccpage0_i2c_id,
};

static int __init ccpage0_i2c_init(void)
{
	int ret;

	ret = i2c_add_driver(&ccpage0_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register pmic I2C driver: %d\n", ret);

	return ret;
}
late_initcall(ccpage0_i2c_init);

static void __exit ccpage0_i2c_exit(void)
{
	i2c_del_driver(&ccpage0_i2c_driver);
}
module_exit(ccpage0_i2c_exit);

MODULE_DESCRIPTION("Crystal Cove page0 device");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com");
