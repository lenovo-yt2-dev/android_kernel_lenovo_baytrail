#ifndef _LINUX_DATA_PT_CHECK_H
#define _LINUX_DATA_PT_CHECK_H

/*
 * This file has definitions for some important file table
 * structures etc.
 */

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mount.h>
extern __inline__ void vfs_set_data_pt_check(int flag);
extern __inline__ int vfs_get_data_pt_check(void);
#ifdef CONFIG_ODM_SDCARD_PT_CHECK
extern __inline__ void vfs_set_sdcard_pt_check(int flag);
extern __inline__ int vfs_get_sdcard_pt_check(void);
extern __inline__ void vfs_set_sdcard_pt_limit_size(u64 size);
extern __inline__ u64 vfs_get_sdcard_pt_limit_size(void);
#endif
extern __inline__ void vfs_set_data_pt_size(u64 size);
extern __inline__ u64 vfs_get_data_pt_size(void);
extern __inline__ void vfs_set_data_pt_limit_size(u64 size);
extern __inline__ u64 vfs_get_data_pt_limit_size(void);
extern int vfs_check_current_path(const struct file *file, u64 size);
extern int vfs_check_current(const struct path *path);

#endif /* _LINUX_DATA_PT_CHECK_H */
