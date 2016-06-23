/*
 *  linux/fs/data_pt_check.c
 *
 */

#include <linux/module.h>
#include <linux/syscalls.h>
#include <asm/atomic.h>
#include <linux/data_pt_check.h>
#include <linux/mount.h>
#include "mount.h"

static atomic_t g_data_pt_check = ATOMIC_INIT(0);
#ifdef CONFIG_ODM_SDCARD_PT_CHECK
static atomic_t g_sdcard_pt_check = ATOMIC_INIT(0);
static atomic64_t g_sdcard_pt_limit_size = ATOMIC64_INIT(0x3200000);
#endif
static atomic64_t g_data_pt_size = ATOMIC64_INIT(0x0FFFFFFF);
static atomic64_t g_data_pt_limit_size = ATOMIC64_INIT(0x500000);

static int __do_check_data_mountpoint(const struct path *path);


static const char g_whitelist_table[][TASK_COMM_LEN] = {
	"d.process.acore",//android.process.acore
	"ndroid.systemui",//com.android.systemui
	"m.android.phone",//com.android.phone
	"android.setting",//com.android.setting
	"novo.lsf.device",//com.lenovo.lsf.device
	"android.qualcmm",//com.android.qualcmm
	"ndroid.launcher",//com.android.launcher
	"system_process",//system_process
	"d.process.media",//android.process.media
	"system_server",//system_server
	"servicemanager",
	"zygote"
};

__inline__ void vfs_set_data_pt_check(int flag)
{
	atomic_set(&g_data_pt_check, flag);
}
__inline__ int vfs_get_data_pt_check(void)
{
	return atomic_read(&g_data_pt_check);
}

#ifdef CONFIG_ODM_SDCARD_PT_CHECK
__inline__ void vfs_set_sdcard_pt_check(int flag)
{
	atomic_set(&g_sdcard_pt_check, flag);
}
__inline__ int vfs_get_sdcard_pt_check(void)
{
	return atomic_read(&g_sdcard_pt_check);
}

__inline__ void vfs_set_sdcard_pt_limit_size(u64 size)
{
	atomic64_set(&g_sdcard_pt_limit_size, size);
}
__inline__ u64 vfs_get_sdcard_pt_limit_size(void)
{
	return atomic64_read(&g_sdcard_pt_limit_size);
}

#endif

__inline__ void vfs_set_data_pt_size(u64 size)
{
	atomic64_set(&g_data_pt_size, size);
}
__inline__ u64 vfs_get_data_pt_size(void)
{
	return atomic64_read(&g_data_pt_size);
}

__inline__ void vfs_set_data_pt_limit_size(u64 size)
{
	atomic64_set(&g_data_pt_limit_size, size);
}
__inline__ u64 vfs_get_data_pt_limit_size(void)
{
	return atomic64_read(&g_data_pt_limit_size);
}

#if 0
static void debug_print_file_name(struct file *file){
	struct path *path = &file->f_path;
	struct inode *inode = path->dentry->d_inode;
	struct dentry * d_dentry_current = NULL;
	struct dentry * d_dentry_parent = NULL;
	struct vfsmount *d_mount_current = NULL;
	struct vfsmount *d_mount_parent = NULL;
	int first = 1;
	char mount_full_path[256];
	char file_full_path[256];
	char temp_buf[128];
	printk("\nLRH: BEGIN##########debug_print_file_name#########\n");
	d_mount_current = file->f_vfsmnt;
	memset(temp_buf, 0, sizeof(temp_buf));
	//printk("LRH: File dentry 0x%x\n", (int)file->f_dentry);
	if(d_mount_current != NULL){
		do{
			if(d_mount_current->mnt_mountpoint->d_name.name != NULL){
				//printk("LRH: d_mount_current->mnt_mountpoint->d_name: %s\n", d_mount_current->mnt_mountpoint->d_name.name);
				if(strncmp(d_mount_current->mnt_mountpoint->d_name.name, "/", 1) != 0){
					sprintf(mount_full_path, "%s/%s", d_mount_current->mnt_mountpoint->d_name.name, temp_buf);
				} else {
					sprintf(mount_full_path, "%s%s", d_mount_current->mnt_mountpoint->d_name.name, temp_buf);
				}
				sprintf(temp_buf, "%s", mount_full_path);
			}
			//printk("LRH: d_mount_current 0x%x\n", (int)d_mount_current);
			//printk("LRH: dentry mount 0x%x\n", (int)d_mount_current->mnt_mountpoint);
			//printk("LRH: dentry root mount 0x%x\n", (int)d_mount_current->mnt_root);
			
			d_mount_parent = d_mount_current->mnt_parent;
			if(d_mount_parent == d_mount_current){
				d_mount_current = NULL;
			} else {
				d_mount_current = d_mount_parent;	
			}
		}while(d_mount_current != NULL);
	}
	d_dentry_current = path->dentry;
	memset(temp_buf, 0, sizeof(temp_buf));
	if(d_dentry_current != NULL){
		do{
			if(d_dentry_current->d_name.name != NULL){
				if(strncmp(d_dentry_current->d_name.name, "/", 1) != 0){
					if(first == 1){
						sprintf(file_full_path, "%s", d_dentry_current->d_name.name);
						first = 0;
					} else {
						sprintf(file_full_path, "%s/%s", d_dentry_current->d_name.name, temp_buf);
					}
				} 
				//printk("LRH: d_dentry_current->d_name.name: %s\n", d_dentry_current->d_name.name);
				sprintf(temp_buf, "%s", file_full_path);
			}
			//printk("LRH: dentry 0x%x\n", (int)d_dentry_current);
			d_dentry_parent = d_dentry_current->d_parent;
			if(d_dentry_parent == d_dentry_current){
				d_dentry_current = NULL;
			} else {
				d_dentry_current = d_dentry_parent;	
			}
		}while(d_dentry_current != NULL);
	}
	if(S_ISDIR(inode->i_mode)){
		printk("LRH: current file is dir\n");
		printk("LRH: current open file is %s%s/\n", mount_full_path, file_full_path);
	} else {
		printk("LRH: current open file is %s%s\n", mount_full_path, file_full_path);
	}
	printk("\nLRH: END##########debug_print_file_name#########\n");

}
#endif


static int __do_check_sdcard_path(const struct path *path){

	int ret = 0;
	struct dentry * d_dentry_current = path->dentry;
	struct dentry * d_dentry_prev = NULL;
	while(d_dentry_current != d_dentry_current->d_parent){
		d_dentry_prev = d_dentry_current;
		d_dentry_current =  d_dentry_current->d_parent;
	}
	if((d_dentry_prev != NULL) &&
		(d_dentry_current != NULL) &&
		(d_dentry_current->d_name.name != NULL) &&
		(d_dentry_prev->d_name.name != NULL)){
		if((strncmp(d_dentry_prev->d_name.name, "media", 5) == 0)&& 
			(strncmp(d_dentry_current->d_name.name, "/", 1) == 0)){
			//printk("VFS_CHECK: __do_check_sdcard_path FIND!!!!\n");
			ret = 1;
		}
	}
	return ret;
}
static int check_whitelist(void)
{
	int ret = 1;
	int i = 0;
	int table_size = sizeof(g_whitelist_table) / sizeof(g_whitelist_table[0]);
	int len = 0;
	char name[TASK_COMM_LEN]={0};
	struct task_struct *task_ptr = NULL;
	task_ptr = current;
	if(current->group_leader != NULL){
		task_ptr = current->group_leader;
	}
	memcpy(name,task_ptr->comm,TASK_COMM_LEN);
	name[TASK_COMM_LEN - 1] = 0;
	//printk("VFS_CHECK: check_whitelist current->comm %s\n",name);
	len = strlen(name);
	for(i = 0; i < table_size; i++){
		if(strncmp(name, g_whitelist_table[i], len) == 0){
			ret = 0;
			//printk("VFS_CHECK: Find one!!!!\n");
			break;
		}
	}
	return ret;
}

int vfs_check_current_path(const struct file *file, u64 size)
{
	int ret = 0;
	do{
	#ifdef CONFIG_ODM_SDCARD_PT_CHECK
		if((ret  =  vfs_get_sdcard_pt_check()) == 0){
			if(size > vfs_get_data_pt_size()){
				ret = __do_check_data_mountpoint(&file->f_path);
				break;
			} else if(vfs_get_sdcard_pt_limit_size() <= (vfs_get_data_pt_size() - size)){
				break;
			} else  if ((ret = __do_check_data_mountpoint(&file->f_path)) == 0){
				break;
			} else if((ret = __do_check_sdcard_path(&file->f_path)) == 0){
				if((ret  =  vfs_get_data_pt_check()) == 0) {
					if(vfs_get_data_pt_limit_size() <= (vfs_get_data_pt_size() - size)){
						break;
					} else if((ret  =  check_whitelist()) == 0){
						break;
					}
				} else if((ret  =  check_whitelist()) == 0){
					break;
				}
			}
		} else  if ((ret = __do_check_data_mountpoint(&file->f_path)) == 0){
			break;
		} else if((ret = __do_check_sdcard_path(&file->f_path)) == 0){
			if((ret  =  vfs_get_data_pt_check()) == 0) {
				if(size > vfs_get_data_pt_size()){
					ret = 1;
					break;
				} else if(vfs_get_data_pt_limit_size() <= (vfs_get_data_pt_size() - size)){
					break;
				} else if((ret  =  check_whitelist()) == 0){
					break;
				}
			} else if((ret  =  check_whitelist()) == 0){
				break;
			}
		}
	#else
		if((ret  =  vfs_get_data_pt_check()) == 0){
			if(size > vfs_get_data_pt_size()){
				ret = __do_check_data_mountpoint(&file->f_path);
				break;
			} else if(vfs_get_data_pt_limit_size() <= (vfs_get_data_pt_size() - size)){
				break;
			} else if((ret = __do_check_data_mountpoint(&file->f_path)) == 0){
				break;
			} else if((ret  =  check_whitelist()) == 0){
				break;
			}
		} else if((ret = __do_check_data_mountpoint(&file->f_path)) == 0){
			break;
		} else if((ret  =  check_whitelist()) == 0){
			break;
		}
	#endif
		//printk("VFS_CHECK: vfs_check_current_path ####NOT ALLOWED to write#### ret = %d\n", ret);
	}while(0);
	return ret;
}

static int __do_check_data_mountpoint(const struct path *path){
	int data_find_flag = 0;
	int ret = 0;
	struct mount *mount_current = NULL;
	struct mount *mount_parent = NULL;
	mount_current = real_mount(path->mnt);
	//printk("VFS_CHECK: check_current_path IN\n");
	if(mount_current != NULL){
		do{
			if(mount_current->mnt_mountpoint->d_name.name != NULL){
				//printk("VFS_CHECK: mount_current->mnt_mountpoint->d_name: %s\n", mount_current->mnt_mountpoint->d_name.name);
				if(data_find_flag == 0){
					if(strncmp(mount_current->mnt_mountpoint->d_name.name, "data", 4) == 0){
						//printk("VFS_CHECK: mount_current->mnt_mountpoint->d_name: %s\n", mount_current->mnt_mountpoint->d_name.name);
						data_find_flag = 1;
					}
					if(strncmp(mount_current->mnt_mountpoint->d_name.name, "/", 1) == 0){
						break;
					}
				}  else {
					if(strncmp(mount_current->mnt_mountpoint->d_name.name, "/", 1) == 0){
						//printk("VFS_CHECK: MATCH!!!!!\n");
						ret = 1;
						break;
					}
					break;
				}
				
			} else {
				//printk("VFS_CHECK: mount_current->mnt_mountpoint->d_name: NULL\n");
				break;
			}
			mount_parent = mount_current->mnt_parent;
			if(mount_parent == mount_current){
				//printk("VFS_CHECK: mount_parent == mount_current\n");
				mount_current = NULL;
			} else {
				mount_current = mount_parent;	
			}
		}while(mount_current != NULL);
	}
	
	//printk("VFS_CHECK: check_current_path OUT\n");
	return ret;
}


int vfs_check_current(const struct path *path){
	int ret = 0;
	//printk("VFS_CHECK: ####vfs_check_current IN####\n");
	do{
	#ifdef CONFIG_ODM_SDCARD_PT_CHECK
		if((ret  =  vfs_get_sdcard_pt_check()) == 0){
			//printk("VFS_CHECK: vfs_get_sdcard_pt_check allowed to write!!\n");
			break;
		} else if ((ret = __do_check_data_mountpoint(path)) == 0){
			//printk("VFS_CHECK: __do_check_data_mountpoint allowed to write!!\n");
			break;
		} else if((ret = __do_check_sdcard_path(path)) == 0){
			if((ret  =  vfs_get_data_pt_check()) == 0) {
				//printk("VFS_CHECK: vfs_get_data_pt_check allowed to write!!\n");
				break;
			} else if((ret  =  check_whitelist()) == 0){
				//printk("VFS_CHECK: check_whitelist allowed to write!!\n");
				break;
			}
		}
	#else
		if((ret  =  vfs_get_data_pt_check()) == 0){
			//printk("VFS_CHECK: vfs_get_data_pt_check allowed to write!!\n");
			break;
		} else if((ret = __do_check_data_mountpoint(path)) == 0){
			//printk("VFS_CHECK: __do_check_data_mountpoint allowed to write!!\n");
			break;
		} else if((ret  =  check_whitelist()) == 0){
			//printk("VFS_CHECK: check_whitelist allowed to write!!\n");
			break;
		}
	#endif
		//printk("VFS_CHECK: vfs_check_current####NOT ALLOWED to write####ret = %d\n",ret );
	}while(0);
	//printk("VFS_CHECK: ####vfs_check_current OUT####\n");
	return ret;
}


EXPORT_SYMBOL(vfs_set_data_pt_check);
EXPORT_SYMBOL(vfs_get_data_pt_check);
#ifdef CONFIG_ODM_SDCARD_PT_CHECK
EXPORT_SYMBOL(vfs_set_sdcard_pt_check);
EXPORT_SYMBOL(vfs_get_sdcard_pt_check);
#endif
EXPORT_SYMBOL(vfs_set_data_pt_size);
EXPORT_SYMBOL(vfs_get_data_pt_size);
EXPORT_SYMBOL(vfs_set_data_pt_limit_size);
EXPORT_SYMBOL(vfs_get_data_pt_limit_size);
EXPORT_SYMBOL(vfs_check_current_path);
EXPORT_SYMBOL(vfs_check_current);


