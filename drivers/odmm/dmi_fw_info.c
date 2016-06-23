
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>

#include <linux/dmi.h>


#define bios_attr(_name) \
static struct kobj_attribute bios_##_name##_attr = {	\
	.attr   = {				\
		.name = __stringify(_name),	\
		.mode = 0440,			\
	},					\
	.show   = bios_##_name##_show,			\
}

static ssize_t bios_version_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%s\n", dmi_get_system_info(DMI_BIOS_VERSION));
}

bios_attr(version);

static ssize_t bios_vendor_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", dmi_get_system_info(DMI_BIOS_VENDOR));
}
bios_attr(vendor);

static ssize_t bios_date_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%s\n", dmi_get_system_info(DMI_BIOS_DATE));
}

bios_attr(date);

static struct attribute *bios_attrs[] = {
	&bios_version_attr.attr,
	&bios_vendor_attr.attr,
	&bios_date_attr.attr,
	NULL,
};

static struct attribute_group bios_attr_group = {
	.attrs = bios_attrs,
};


struct kobject *odmm_kobj;
EXPORT_SYMBOL(odmm_kobj);
static int __init odmm_fw_info_init(void){
    int ret = 0;
    struct kobject *bios_kobj;
    ret = -ENOMEM;
    odmm_kobj = kobject_create_and_add("odmm", NULL);
    if (!odmm_kobj)
        goto odmm_fail;
    bios_kobj = kobject_create_and_add("bios", odmm_kobj);
    if (!bios_kobj) 
        goto bios_fail;
    
    ret = sysfs_create_group(bios_kobj, &bios_attr_group);
    if (ret) 
        goto bios_fail;
    return 0;
bios_fail:
    kobject_put(odmm_kobj);
odmm_fail:
    return ret;
        
}

arch_initcall(odmm_fw_info_init);

