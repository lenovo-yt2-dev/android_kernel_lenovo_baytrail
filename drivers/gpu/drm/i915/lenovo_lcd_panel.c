#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include "lenovo_lcd_panel.h"

#define NAME_SIZE 25

static struct lcd_panel lenovo_lcd_panel;
extern int i915_dpst_switch(bool on);
extern bool i915_get_dpst_status(void);

ssize_t lenovo_lcd_get_cabc(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
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

	return ret;
}

ssize_t lenovo_lcd_set_cabc(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
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
	return count;
}

ssize_t lenovo_lcd_get_ce(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
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

    printk("[LCD]: %s: ==jinjt==buf=%s  ret=%d  line=%d\n",__func__,buf,ret,__LINE__);
	return ret;
}

ssize_t lenovo_lcd_set_ce(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
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
	return count;
}

ssize_t lenovo_lcd_get_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;

    /*printk("[LCD]: %s: ==jinjt==line=%d\n",__func__,__LINE__);*/
	if(lcd_panel == NULL){
		printk("[LCD]: %s: Not have registed lcd panel\n",__func__);
		return ret;
	}

	sprintf(buf, "%s\n", lcd_panel->name);

	ret = strlen(buf) + 1;

    /*printk("[LCD]: %s: ==jinjt==name=%s   line=%d\n",__func__,buf,__LINE__);*/
	return ret;
}
ssize_t lenovo_lcd_get_dpst(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
    bool status;
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;

    printk("[LCD]: %s: ==jinjt== line=%d\n",__func__,__LINE__);
	if(lcd_panel == NULL)
		return ret;
    status = i915_get_dpst_status();
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
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;
    struct hal_panel_ctrl_data ctrl;
	int index = 0;
	struct intel_dsi *dsi = lcd_panel->dsi;
	struct drm_device *drmdev = dsi->base.base.dev;
	struct drm_i915_private *dev_priv = drmdev->dev_private;

    //printk("[LCD]: %s: ===jinjt===%s\n",__func__, buf);

	if(lcd_panel == NULL )
	{
		printk("[LCD]: %s: ==jinjt==lcd_panel is NULL\n",__func__);
		return ret;
	}

	if(!dev_priv->backlight.enabled || lcd_panel-> status !=  ON){
		printk("[LCD]: %s=jinjt=lcd already powered down  %d   %d\n", __func__, dev_priv->backlight.enabled, lcd_panel-> status);
		return ret;
	}
	
	if(lcd_panel->get_effect_index_by_name){
		index = lcd_panel->get_effect_index_by_name("cabc");
		if(index < 0 ){
			printk("[LCD]: %s: Not support cabc function\n",__func__);
			return ret;
			}
	}

	ctrl.index = index;
	if (strncmp(buf, "1", 1) == 0)
    {
        ctrl.level = 0;
        if(lcd_panel->set_effect(&ctrl, dsi)!= 0){
            printk("[LCD]: errored from set effect\n");
            return ret;
        }
        i915_dpst_switch(true);
    }
	else
    {
        ctrl.level = 1;
        if(lcd_panel->set_effect(&ctrl, dsi)!= 0){
            printk("[LCD]: errored from set effect\n");
            return ret;
        }
        i915_dpst_switch(false);
    }
    printk("[LCD]: %s: ==jinjt==line=%d ctrl.level=%d\n",__func__,__LINE__,ctrl.level);
	return count;
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

static int lenovo_lcd_panel_open(struct inode *inode, struct file *filp)
{
	int ret = -1;
	struct lcd_panel_dev *lcd_panel =  lenovo_lcd_panel.lcd_device;

    /*printk("[LCD]: %s: ==jinjt==line=%d\n",__func__,__LINE__);*/
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
	struct drm_device *dev = dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

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
	.unlocked_ioctl = lenovo_lcd_panel_ioctl,
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

static int __init lcd_panel_init(void)
{
	int ret = 0;
	struct cdev *lcd_cdev;
	dev_t *lcd_devno;
	struct kobject *lcd_kobject;

	lcd_cdev  = &lenovo_lcd_panel.lcd_panel_cdev;
	lcd_devno = &lenovo_lcd_panel.lcd_panel_devno;


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
	
	//add kobject to sys filesystem
    lcd_kobject = kobject_create_and_add("lcd_panel",NULL);
		if(ret){
			printk("[LCD]: failed to add kobject\n");
			return ret;
		}else
			lenovo_lcd_panel.lcd_kobj = lcd_kobject;

	ret = sysfs_create_group(lcd_kobject, &lenovo_lcd_attr_group);
	if(ret){
		printk("[LCD]: failed to create group\n");
		kobject_put(lcd_kobject);
		return ret;
	}
	return ret;
} 

static void __exit lcd_panel_exit(void)
{

	struct cdev *lcd_cdev = &lenovo_lcd_panel.lcd_panel_cdev;
	struct class *lcd_class = lenovo_lcd_panel.lcd_panel_class;
	dev_t lcd_devno =  lenovo_lcd_panel.lcd_panel_devno;
	struct kobject *lcd_kobject = lenovo_lcd_panel.lcd_kobj;

	cdev_del(lcd_cdev);

	device_destroy(lcd_class, lcd_devno); //delete the dev node under /dev

	class_destroy(lcd_class);

	unregister_chrdev_region(lcd_devno, 1);

	kobject_put(lcd_kobject);

	return;

}

module_init(lcd_panel_init);
module_exit(lcd_panel_exit);
