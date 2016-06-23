

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/data_pt_check.h>
#include <linux/gfp.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

static struct proc_dir_entry *odmm_dir = NULL;
static ssize_t data_pt_size_write(struct file *filp, const char *ubuf,
	   size_t cnt, loff_t *data)
{

    int err;
    u64 input;
    char *kbuf;
    if(cnt <= 0 || cnt >= PAGE_SIZE){
        return -EINVAL;
    }
    err = -ENOMEM;
    kbuf = kmalloc(cnt + 1, GFP_KERNEL);
    if (!kbuf)
        goto nomem;
    err = -EFAULT;
    if(copy_from_user(kbuf, ubuf, cnt) != 0)
        goto infault;
    kbuf[cnt] = 0;
    err = kstrtou64(kbuf, 10, &input);
    if (err)
        goto infault;
    vfs_set_data_pt_size(input);
infault:
	kfree(kbuf);
nomem:
    return cnt;
    
}


static ssize_t data_pt_limit_size_write(struct file *filp, const char *ubuf,
	   size_t cnt, loff_t *data)
{
    int err;
    u64 input;
    char *kbuf;
    if(cnt <= 0 || cnt >= PAGE_SIZE){
        return -EINVAL;
    }
    err = -ENOMEM;
    kbuf = kmalloc(cnt + 1, GFP_KERNEL);
    if (!kbuf)
        goto nomem;
    err = -EFAULT;
    if(copy_from_user(kbuf, ubuf, cnt) != 0)
        goto infault;
    kbuf[cnt] = 0;
    err = kstrtou64(kbuf, 10, &input);
    if (err)
        goto infault;
    vfs_set_data_pt_limit_size(input);
infault:
	kfree(kbuf);
nomem:
    return cnt;

}


static ssize_t data_pt_check_write(struct file *filp, const char *ubuf,
	   size_t cnt, loff_t *data)
{
    int err;
    int input;
    char *kbuf;
    if(cnt <= 0 || cnt >= PAGE_SIZE){
        return -EINVAL;
    }
    err = -ENOMEM;
    kbuf = kmalloc(cnt + 1, GFP_KERNEL);
    if (!kbuf)
        goto nomem;
    err = -EFAULT;
    if(copy_from_user(kbuf, ubuf, cnt) != 0)
        goto infault;
    kbuf[cnt] = 0;
    err = kstrtoint(kbuf, 10, &input);
    if (err)
        goto infault;
    vfs_set_data_pt_check(input);
infault:
	kfree(kbuf);
nomem:
    return cnt;
}

#ifdef CONFIG_ODM_SDCARD_PT_CHECK
static ssize_t sdcard_pt_check_write(struct file *filp, const char *ubuf,
	   size_t cnt, loff_t *data)
{
    int err;
    int input;
    char *kbuf;
    if(cnt <= 0 || cnt >= PAGE_SIZE){
        return -EINVAL;
    }
    err = -ENOMEM;
    kbuf = kmalloc(cnt + 1, GFP_KERNEL);
    if (!kbuf)
        goto nomem;
    err = -EFAULT;
    if(copy_from_user(kbuf, ubuf, cnt) != 0)
        goto infault;
    kbuf[cnt] = 0;
    err = kstrtoint(kbuf, 10, &input);
    if (err)
        goto infault;
    vfs_set_sdcard_pt_check(input);
infault:
	kfree(kbuf);
nomem:
    return cnt;
}

static ssize_t sdcard_pt_limit_size_write(struct file *filp, const char *ubuf,
	   size_t cnt, loff_t *data)
{
    int err;
    u64 input;
    char *kbuf;
    if(cnt <= 0 || cnt >= PAGE_SIZE){
        return -EINVAL;
    }
    err = -ENOMEM;
    kbuf = kmalloc(cnt + 1, GFP_KERNEL);
    if (!kbuf)
        goto nomem;
    err = -EFAULT;
    if(copy_from_user(kbuf, ubuf, cnt) != 0)
        goto infault;
    kbuf[cnt] = 0;
    err = kstrtou64(kbuf, 10, &input);
    if (err)
        goto infault;
    vfs_set_sdcard_pt_limit_size(input);
infault:
	kfree(kbuf);
nomem:
    return cnt;

}

#endif

static int data_pt_size_show(struct seq_file *m, void *v)
{
    seq_printf(m, "%llu\n",vfs_get_data_pt_size());
    return 0;
}

static int data_pt_limit_size_show(struct seq_file *m, void *v)
{
    seq_printf(m, "%llu\n",vfs_get_data_pt_limit_size());
    return 0;
}

static int data_pt_check_show(struct seq_file *m, void *v)
{
    seq_printf(m, "%d\n",vfs_get_data_pt_check());
    return 0;
}

#ifdef CONFIG_ODM_SDCARD_PT_CHECK
static int sdcard_pt_check_show(struct seq_file *m, void *v)
{
    seq_printf(m, "%d\n",vfs_get_sdcard_pt_check());
    return 0;
}

static int sdcard_pt_limit_size_show(struct seq_file *m, void *v)
{
    seq_printf(m, "%llu\n",vfs_get_sdcard_pt_limit_size());
    return 0;
}
#endif
static int data_pt_size_open(struct inode *inode, struct file *file) 
{ 
    return single_open(file, data_pt_size_show, inode->i_private); 
} 

static int data_pt_limit_size_open(struct inode *inode, struct file *file) 
{ 
    return single_open(file, data_pt_limit_size_show, inode->i_private); 
} 

static int data_pt_check_open(struct inode *inode, struct file *file) 
{ 
    return single_open(file, data_pt_check_show, inode->i_private); 
} 

#ifdef CONFIG_ODM_SDCARD_PT_CHECK
static int sdcard_pt_check_open(struct inode *inode, struct file *file) 
{ 
    return single_open(file, sdcard_pt_check_show, inode->i_private); 
} 

static int sdcard_pt_limit_size_open(struct inode *inode, struct file *file) 
{ 
    return single_open(file, sdcard_pt_limit_size_show, inode->i_private); 
} 
#endif

static const struct file_operations data_pt_size_fops = { 
    .open = data_pt_size_open, 
    .write = data_pt_size_write,
    .read = seq_read, 
    .llseek = seq_lseek, 
    .release = single_release, 
};

static const struct file_operations  data_pt_limit_size_fops = { 
    .open =  data_pt_limit_size_open, 
    .write =  data_pt_limit_size_write,
    .read = seq_read, 
    .llseek = seq_lseek, 
    .release = single_release, 
};

static const struct file_operations  data_pt_check_fops = { 
    .open =  data_pt_check_open, 
    .write =  data_pt_check_write,
    .read = seq_read, 
    .llseek = seq_lseek, 
    .release = single_release, 
};

#ifdef CONFIG_ODM_SDCARD_PT_CHECK
static const struct file_operations  sdcard_pt_check_fops = { 
    .open =  sdcard_pt_check_open, 
    .write =  sdcard_pt_check_write,
    .read = seq_read, 
    .llseek = seq_lseek, 
    .release = single_release, 
};

static const struct file_operations  sdcard_pt_limit_size_fops = { 
    .open =  sdcard_pt_limit_size_open, 
    .write =  sdcard_pt_limit_size_write,
    .read = seq_read, 
    .llseek = seq_lseek, 
    .release = single_release, 
};
#endif
static int __init init_data_pt_check(void)
{

    odmm_dir = proc_mkdir("odmm", NULL);

    if (!odmm_dir) {
        printk(KERN_INFO "Failed to create /proc/odmm/\n ");
        return 0;
    }
    proc_create("data_pt_size", 0664, odmm_dir, &data_pt_size_fops);
    proc_create("data_pt_limit_size", 0664, odmm_dir, &data_pt_limit_size_fops);
    proc_create("data_pt_check", 0664, odmm_dir, &data_pt_check_fops);
#ifdef CONFIG_ODM_SDCARD_PT_CHECK
    proc_create("sdcard_pt_check", 0664, odmm_dir, &sdcard_pt_check_fops);
    proc_create("sdcard_pt_limit_size", 0664, odmm_dir, &sdcard_pt_limit_size_fops);

#endif
    return 0;
}
module_init(init_data_pt_check);
