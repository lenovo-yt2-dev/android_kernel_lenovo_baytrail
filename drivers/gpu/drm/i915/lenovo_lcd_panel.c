#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include "lenovo_lcd_panel.h"
#include <linux/gpio.h>
#include "linux/mfd/intel_mid_pmic.h"

#define NAME_SIZE 25
#ifndef CONFIG_BLADE2_13
static struct lcd_panel lenovo_lcd_panel;
#else
#define GPIO1P5 381
#define LCD_ID GPIO1P5
#define PMIC_GPIO1P0_BASE_ADDRESS 0x3b
#define PMIC_GPIO1P5_OFFSET 0x5
#define PMIC_GPIO1P5_ADDRESS PMIC_GPIO1P0_BASE_ADDRESS+PMIC_GPIO1P5_OFFSET
#define GPIO_INPUT_NO_DRV 0x0
#endif
extern int i915_dpst_switch(bool on);
extern bool i915_get_dpst_status(void);
struct mutex lcd_mutex;
#ifdef CONFIG_BLADE2_13
char enable_ce_command[1]={0x08};
char disable_ce_command[1]={0x00};
char enable_cabc_command[1] = {0x10};
char disable_cabc_command[1] = {0x00};
extern int intel_dp_aux_extern_write( uint16_t address, uint8_t *send, int send_bytes);
extern int intel_dp_aux_extern_read( uint16_t address, uint8_t *recv, int recv_bytes);
extern struct intel_dp *g_intel_dp;
#endif
ssize_t lenovo_lcd_get_cabc(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
#ifndef CONFIG_BLADE2_13
	int index = 1;
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;
	struct hal_panel_ctrl_data ctrl;

    printk("[LCD]: %s: ==jinjt== line=%d\n",__func__,__LINE__);
	if(lcd_panel == NULL)
		return ret;

	if(lcd_panel->get_effect_index_by_name){
		index = lcd_panel->get_effect_index_by_name("cabc");
		if(index < 0){
			printk("[LCD]: %s: Not support cabc function\n",__func__);
			return ret;
			}
	}

	ctrl.index = index;

	if(lcd_panel->get_current_level)
		ret = lcd_panel->get_current_level(&ctrl);

	if(ctrl.panel_data.effect[index].level == 1)
		/*sprintf(buf, "on\n");*/
		sprintf(buf, "1\n");
	else
		/*sprintf(buf, "off\n");*/
		sprintf(buf, "0\n");

	ret = strlen(buf)+1;
    /*printk("[LCD]: %s: ==jinjt==line=%d ret=%d\n",__func__,__LINE__,ret);*/
#endif 
	return ret;
}

ssize_t lenovo_lcd_set_cabc(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef CONFIG_BLADE2_13
//	char tmp[10];

	if (strncmp(buf, "on", 2) == 0)
	{
		//intel_dp_aux_extern_write(0x0405,enable_cabc_command,1);
		intel_dp_aux_extern_write(0x0721,enable_cabc_command,1);
		printk("enable cabc \n");
	}
	else if ((strncmp(buf, "off", 3) == 0))
	{
	//	intel_dp_aux_extern_write(0x0405,disable_cabc_command,1);
		intel_dp_aux_extern_write(0x0721,disable_cabc_command,1);
		printk("disable cabc \n");
	}


#else
	int index = 0, ret = 0;
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;
	struct intel_dsi *dsi = lcd_panel->dsi;
	struct hal_panel_ctrl_data ctrl;

    	printk("[LCD]: %s: ===jinjt===%s\n",__func__, buf);
	if(lcd_panel == NULL || lcd_panel->status != ON)
		return ret;

	/*if (strncmp(buf, "on", 2) == 0)*/
	if (strncmp(buf, "1", 1) == 0)
		ctrl.level = 1;
	else
		ctrl.level = 0;
	if(lcd_panel->get_effect_index_by_name){
		index = lcd_panel->get_effect_index_by_name("cabc");
		if(index < 0 ){
			printk("[LCD]: %s: Not support cabc function\n",__func__);
			return ret;
			}
	}

	ctrl.index = index;

    printk("[LCD]: %s: ==jinjt==line=%d ctrl.level=%d\n",__func__,__LINE__,ctrl.level);
	if(lcd_panel->set_effect(&ctrl, dsi)!= 0){
		printk("[LCD]: errored from set effect\n");
		return ret;
	}

    /*printk("[LCD]: %s: ==jinjt==line=%d\n",__func__,__LINE__);*/
#endif 
	return count;
}

ssize_t lenovo_lcd_get_ce(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

#ifndef CONFIG_BLADE2_13
	int index = 1;
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;
	struct hal_panel_ctrl_data ctrl;

    printk("[LCD]: %s: ==jinjt==line=%d\n",__func__,__LINE__);
	if(lcd_panel == NULL || lcd_panel->status != ON)
		return ret;

	if(lcd_panel->get_effect_index_by_name){
		index = lcd_panel->get_effect_index_by_name("ce");
		if(index < 0){
			printk("[LCD]: %s: Not support ce function\n",__func__);
			return ret;
		}
	}

	ctrl.index = index;

	if(lcd_panel->get_current_level)
		ret = lcd_panel->get_current_level(&ctrl);

	if(ctrl.panel_data.effect[index].level == 1)
		/*sprintf(buf, "on\n");*/
		sprintf(buf, "1\n");
	else
		/*sprintf(buf, "off\n");*/
		sprintf(buf, "0\n");

	ret = strlen(buf)+1;

#endif 
	return ret;
}

ssize_t lenovo_lcd_set_ce(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef CONFIG_BLADE2_13
	if (strncmp(buf, "1", 1) == 0)
	{
		intel_dp_aux_extern_write(0x720,enable_ce_command,1);
		printk("enable ce \n");
	}
	else if ((strncmp(buf, "0", 1) == 0))
	{
		intel_dp_aux_extern_write(0x720,disable_ce_command,1);
		printk("disable ce \n");
	}
#else
	int index = 0, ret = 0;
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;
	struct intel_dsi *dsi = lcd_panel->dsi;
	struct hal_panel_ctrl_data ctrl;

    	printk("[LCD]: %s: ===jinjt===%s\n",__func__, buf);
	if(lcd_panel == NULL || lcd_panel->status != ON)
		return ret;

	/*if (strncmp(buf, "on", 2) == 0)*/
	if (strncmp(buf, "1", 1) == 0)
		ctrl.level = 1;
	else
		ctrl.level = 0;
	if(lcd_panel->get_effect_index_by_name){
		index = lcd_panel->get_effect_index_by_name("ce");
		if(index < 0){
			printk("[LCD]: %s: Not support ce function\n",__func__);
			return ret;
		}
	}

	ctrl.index = index;

	if(lcd_panel->set_effect(&ctrl, dsi)!= 0)
		printk("[LCD]: errored from set effect\n");

    printk("[LCD]: %s: ==jinjt==line=%d ctrl.level=%d\n",__func__,__LINE__,ctrl.level);
#endif 
	return count;
}

ssize_t lenovo_lcd_get_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
#ifndef CONFIG_BLADE2_13
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;

    /*printk("[LCD]: %s: ==jinjt==line=%d\n",__func__,__LINE__);*/
	if(lcd_panel == NULL){
		printk("[LCD]: %s: Not have registed lcd panel\n",__func__);
		return ret;
	}

	sprintf(buf, "%s\n", lcd_panel->name);
#else
	int lcd_id = 0 ;
	ret = gpio_request(LCD_ID, "lcd_id");
        if (ret < 0)
                 printk("[lcd_id] request fail\n");
        ret = gpio_direction_input(LCD_ID);
        if(ret< 0)
        	printk("[lcd_id]: Failed to config gpio lcd_id\n");
 
         //set gpio1p2 function state
        intel_mid_pmic_writeb(PMIC_GPIO1P5_ADDRESS, GPIO_INPUT_NO_DRV);
 
        lcd_id = gpio_get_value_cansleep(LCD_ID);
        printk("[lcd_id]:%s,gpio1P5 value: 0x%x\n",__func__,lcd_id);
       
    if(lcd_id==0)
    	{
    	 sprintf(buf, "%s\n", "BOE_2560x1440_panel" );
    	}
	else
	{
	 sprintf(buf, "%s\n", "INL_2560x1440_panel" );
	}

	//sprintf(buf, "lcd_type: %d\n", lcd_id);
#endif
	ret = strlen(buf) + 1;

    /*printk("[LCD]: %s: ==jinjt==name=%s   line=%d\n",__func__,buf,__LINE__);*/
	return ret;
}
ssize_t lenovo_lcd_get_dpst(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
    bool status;
#ifndef CONFIG_BLADE2_13
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;

    printk("[LCD]: %s: ==jinjt== line=%d\n",__func__,__LINE__);
	if(lcd_panel == NULL)
		return ret;
#endif
	//status = i915_get_dpst_status();
    status = 0;
    if(status == true)
        sprintf(buf, "1\n");
    else
        sprintf(buf, "0\n");
    ret = strlen(buf)+1;
	return ret;
}

ssize_t lenovo_lcd_set_dpst(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = 0;


    return ret;

}
static DEVICE_ATTR(cabc_onoff, S_IRUGO | S_IWUSR | S_IWGRP, lenovo_lcd_get_cabc, lenovo_lcd_set_cabc);
static DEVICE_ATTR(ce_onoff, S_IRUGO | S_IWUSR | S_IWGRP, lenovo_lcd_get_ce, lenovo_lcd_set_ce);
static DEVICE_ATTR(lcd_name, S_IRUGO | S_IWUSR | S_IWGRP, lenovo_lcd_get_name, NULL);
static DEVICE_ATTR(dpst_onoff, S_IRUGO | S_IWUSR | S_IWGRP, lenovo_lcd_get_dpst, lenovo_lcd_set_dpst);

static struct attribute *lenovo_lcd_attrs[] = {
	&dev_attr_cabc_onoff.attr,
	&dev_attr_ce_onoff.attr,
	&dev_attr_lcd_name.attr,
	&dev_attr_dpst_onoff.attr,
	NULL,
};

static struct attribute_group lenovo_lcd_attr_group = {
	.attrs = lenovo_lcd_attrs,
};
#ifndef CONFIG_BLADE2_13
static int lenovo_lcd_panel_open(struct inode *inode, struct file *filp)
{
	int ret = -1;
	struct lcd_panel_dev *lcd_panel =  lenovo_lcd_panel.lcd_device;

    printk("[LCD]: %s: ==jinjt==line=%d\n",__func__,__LINE__);
	if(lcd_panel == NULL)
		return ret;

	if(lcd_panel->status != ON)
	{
		printk("[LCD]: panel have powered down\n");
		return ret;
	}

	return 0;
}

static long lenovo_lcd_panel_ioctl(struct file *filp, unsigned int cmd, unsigned long argp)
{
	int ret = 0;
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;
	struct intel_dsi *dsi = lcd_panel->dsi;
	struct hal_panel_ctrl_data *hal_panel_data = (struct hal_panel_ctrl_data *)argp;
#ifndef CONFIG_BLADE2_13	
    struct drm_device *dev = dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
#endif
/*printk("[LCD]: %s: ==jinjt==line=%d LCD_IOCTL_GET_SUPPORTED_EFFECT=%d\n",__func__,__LINE__,LCD_IOCTL_GET_SUPPORTED_EFFECT);*/
/*printk("[LCD]: %s: ==jinjt==line=%d LCD_IOCTL_GET_EFFECT_LEVELS=%d\n",__func__,__LINE__,LCD_IOCTL_GET_EFFECT_LEVELS);*/
/*printk("[LCD]: %s: ==jinjt==line=%d LCD_IOCTL_GET_SUPPORTED_MODE=%d\n",__func__,__LINE__,LCD_IOCTL_GET_SUPPORTED_MODE);*/
/*printk("[LCD]: %s: ==jinjt==line=%d LCD_IOCTL_SET_EFFECT=%d\n",__func__,__LINE__,LCD_IOCTL_SET_EFFECT);*/
/*printk("[LCD]: %s: ==jinjt==line=%d LCD_IOCTL_SET_MODE=%d\n",__func__,__LINE__,LCD_IOCTL_SET_MODE);*/
/*printk("[LCD]: %s: ==jinjt==line=%d LCD_IOCTL_GET_CURRENT_LEVEL=%d\n",__func__,__LINE__,LCD_IOCTL_GET_CURRENT_LEVEL);*/
/*#define LCD_IOCTL_GET_SUPPORTED_EFFECT     _IOW(LCD_IOCTL_MAGIC, 1, struct hal_panel_ctrl_data)*/
/*#define LCD_IOCTL_GET_EFFECT_LEVELS        _IOW(LCD_IOCTL_MAGIC, 2, struct hal_panel_ctrl_data)*/
/*#define LCD_IOCTL_GET_SUPPORTED_MODE       _IOW(LCD_IOCTL_MAGIC, 3, struct hal_panel_ctrl_data)*/
/*#define LCD_IOCTL_SET_EFFECT               _IOW(LCD_IOCTL_MAGIC, 4, struct hal_panel_ctrl_data)*/
/*#define LCD_IOCTL_SET_MODE                 _IOW(LCD_IOCTL_MAGIC, 5, struct hal_panel_ctrl_data)*/
/*#define LCD_IOCTL_GET_CURRENT_LEVEL        _IOW(LCD_IOCTL_MAGIC, 6, struct hal_panel_ctrl_data)*/
	if(lcd_panel == NULL)
	{
		printk("[LCD]: %s: ==jinjt==lcd_panel is NULL\n",__func__);
		return ret;
	}

	if(!dev_priv->backlight.enabled || lcd_panel-> status !=  ON){
		printk("[LCD]: ==jinjt==DSI already powered down  %d   %d\n", dev_priv->backlight.enabled, lcd_panel-> status);
		return ret;
	}

	switch (cmd){
		case LCD_IOCTL_GET_SUPPORTED_EFFECT:
			printk("[LCD]: %s: ==jinjt=LCD_IOCTL_GET_SUPPORTED_EFFECT=cmd=%d  line=%d\n",__func__,cmd,__LINE__);
			if(lcd_panel->get_supported_effect != NULL)
			{
				ret = lcd_panel->get_supported_effect(hal_panel_data);
				if (ret < 0 ){
					printk("[lcd]: Error from get_supported_effect\n");
					return ret;
				}
			}else{
				printk("[LCD]: get_supported_effect have not implemented\n");
				return ret;
			}
			break;
		case LCD_IOCTL_GET_EFFECT_LEVELS:
			printk("[LCD]: %s: ==jinjt=LCD_IOCTL_GET_EFFECT_LEVELS=cmd=%d  line=%d\n",__func__,cmd,__LINE__);
			if(lcd_panel->get_effect_levels != NULL)
			{
				ret = lcd_panel->get_effect_levels(hal_panel_data);
				if(ret < 0){
					printk("[LCD]: Error from get_effect_levels\n");
					return ret;
				}
			}else{
				printk("[LCD]: get_effect_levels have not implemented\n");
				return ret;
			}
			break;
#if 1 
        	case LCD_IOCTL_GET_CURRENT_LEVEL:
			printk("[LCD]: %s: ==jinjt=LCD_IOCTL_GET_CURRENT_LEVEL=cmd=%d  line=%d\n",__func__,cmd,__LINE__);
			if(lcd_panel->get_current_level != NULL)
			{
				ret = lcd_panel->get_current_level(hal_panel_data);
				if(ret < 0){
					printk("[LCD]: Error from get_supported_level\n");
					return ret;
				}
			}else{
				printk("[LCD]: get_supported_level have not implemented\n");
				return ret;
			}
			break;
#endif
		case LCD_IOCTL_GET_SUPPORTED_MODE:
			printk("[LCD]: %s: ==jinjt=LCD_IOCTL_GET_SUPPORTED_MODE=cmd=%d  line=%d\n",__func__,cmd,__LINE__);
			if(lcd_panel->get_supported_mode != NULL)
			{
				ret = lcd_panel->get_supported_mode(hal_panel_data);
				if(ret < 0){
					printk("[LCD]:Error from get_supported_mode\n");
					return ret;
				}
			}else{
				printk("[LCD]: get_supported_mode have not implemented\n");
				return ret;
			}
			break;
		case LCD_IOCTL_SET_EFFECT:
			printk("[LCD]: %s: ==jinjt=LCD_IOCTL_SET_EFFECT=cmd=%d  line=%d\n",__func__,cmd,__LINE__);
			if(lcd_panel->set_effect != NULL)
			{
				ret = lcd_panel->set_effect(hal_panel_data, dsi);
				if(ret < 0){
					printk("[LCD]:Error from set_effect\n");
					return ret;
				}
			}else{
				printk("[LCD]: set_effect have not implemented\n");
				return ret;
			}
			break;
		case LCD_IOCTL_SET_MODE:
			printk("[LCD]: %s: ==jinjt=LCD_IOCTL_SET_MODE=cmd=%d  line=%d\n",__func__,cmd,__LINE__);
			if(lcd_panel->set_mode!= NULL)
			{
				ret = lcd_panel->set_mode(hal_panel_data, dsi);
				if(ret < 0){
					printk("[LCD]: Error from set_mode\n");
					return ret;
				}
			}else{
				printk("[LCD]:set_mode have not implemented\n");
				return ret;
			}
			break;
		default: 
			printk("[LCD]: %s: ==jinjt=unknow default=cmd=%d  line=%d\n",__func__,cmd,__LINE__);
			break;
	}

	printk("[LCD]: %s: ==jinjt==exit\n",__func__);
	return ret;
}

int lenovo_lcd_panel_register(struct lcd_panel_dev *lcd_panel_device)
{
	int ret = 0;

	if(lcd_panel_device->name)
		printk("[LCD]:@@@@@@%s: @@@@@@%s \n",__func__, lcd_panel_device->name);
	if(lcd_panel_device == NULL)
	{
		printk("[LCD]:%s, invalid parameter \n",__func__);
		return ret;
	}

	lenovo_lcd_panel.lcd_device = lcd_panel_device;

	return ret;

}
struct file_operations lcd_panel_fops = {
	.owner = THIS_MODULE,
	.open = lenovo_lcd_panel_open,
	.compat_ioctl = lenovo_lcd_panel_ioctl,
};

static int lenovo_lcd_panel_dev_init(struct cdev *cdev, dev_t *devno)
{
	int ret = 0;
	dev_t dev_no; 

	ret = alloc_chrdev_region(devno, 0 , 1, "lenovo_lcd_panel");
	if(ret < 0){
		printk("[LCD]: Failed to alloc chrdev no. \n");
		return ret;
	}
	
	dev_no = *devno;
	cdev_init(cdev, &lcd_panel_fops);
	cdev->owner = THIS_MODULE;
	cdev_add(cdev, dev_no, 1);

	return ret;
}
#endif 
#ifdef CONFIG_BLADE2_13
int lenovo_lcd_panel_register(struct lcd_panel_dev *lcd_panel_device)
{
	return 1;
}
#endif
static int __init lcd_panel_init(void)
{

	int ret = 0;
	struct kobject *lcd_kobject;
#ifndef CONFIG_BLADE2_13
	
	struct cdev *lcd_cdev;
	dev_t *lcd_devno;
	

	lcd_cdev  = &lenovo_lcd_panel.lcd_panel_cdev;
	lcd_devno = &lenovo_lcd_panel.lcd_panel_devno;
    mutex_init(&lcd_mutex);


	ret = lenovo_lcd_panel_dev_init(lcd_cdev, lcd_devno);
	if(ret < 0)
		return ret;

	/*create sys class for lcd panel*/
    lenovo_lcd_panel.lcd_panel_class = class_create(THIS_MODULE, "lcd_class");
	if(IS_ERR(lenovo_lcd_panel.lcd_panel_class))
	{
		printk("[LCD]: failed to create lcd panel class");
		return -1;
		}

	/*create dev file */
    device_create(lenovo_lcd_panel.lcd_panel_class, NULL, *lcd_devno,NULL,"lcd%d",0);
#endif 	
	//add kobject to sys filesystem
    	lcd_kobject = kobject_create_and_add("lcd_panel",NULL);
		
	if(lcd_kobject)
	{   
#ifndef CONFIG_BLADE2_13
                lenovo_lcd_panel.lcd_kobj = lcd_kobject;
#endif
		ret = sysfs_create_group(lcd_kobject, &lenovo_lcd_attr_group);
	}
	if(ret){
		printk("[LCD]: failed to create group\n");
		kobject_put(lcd_kobject);
		return ret;
	}
	return ret;
} 

static void __exit lcd_panel_exit(void)
{
#ifndef CONFIG_BLADE2_13
	struct cdev *lcd_cdev = &lenovo_lcd_panel.lcd_panel_cdev;
	struct class *lcd_class = lenovo_lcd_panel.lcd_panel_class;
	dev_t lcd_devno =  lenovo_lcd_panel.lcd_panel_devno;
	struct kobject *lcd_kobject = lenovo_lcd_panel.lcd_kobj;

	cdev_del(lcd_cdev);

	device_destroy(lcd_class, lcd_devno); //delete the dev node under /dev

	class_destroy(lcd_class);

	unregister_chrdev_region(lcd_devno, 1);

	kobject_put(lcd_kobject);
#endif 
	return;

}

module_init(lcd_panel_init);
module_exit(lcd_panel_exit);
