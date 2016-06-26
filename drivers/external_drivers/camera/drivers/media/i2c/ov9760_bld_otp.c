/*
 * Support for OV9760 NVM OTP in Blade II project.
 *
 * Copyright (c) 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
 
#include "ov9760_bld_otp.h"

static const u8 std_cie_xy[] = {0x55, 0xb8, 0x77, 0x97};

#ifdef __KERNEL__
static u32 ov9760_otp_save(const u8 *databuf, u32 data_size, const u8 *filp_name)
{
	struct file *fp=NULL;
	mm_segment_t fs;
	loff_t pos;

	fp=filp_open(filp_name,O_CREAT|O_RDWR,0644);
	if(IS_ERR(fp))
		return -1;

	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_write(fp, databuf, data_size, &pos);
	set_fs(fs);

	filp_close(fp,NULL);

	return 0;
}

static int
ov9760_read_otp(struct i2c_client *client, u16 len, u16 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u16 data[OV9760_SHORT_MAX];
	int err;

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}


	memset(msg, 0, sizeof(msg));
	memset(data, 0, sizeof(data));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = (u8 *)data;
	/* high byte goes first */
	data[0] = cpu_to_be16(reg);

	msg[1].addr = client->addr;
	msg[1].len = len;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = (u8 *)val;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err < 0)
		goto error;

	return 0;

error:
	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}


static u32 ov9760_otp_read(struct i2c_client *client, u8 *ov9760_data_ptr, u32 *ov9760_size)
{
	u16 bank,index=0;
	int ret;

	for(index=0;index<OV9760_OTP_BANK_NUM;index++)
	{
		/* 0x3D84:Bit[5:0] Memory select;Bit[6] mode_select;Bit[7]program_dis */
		/* 0xc0:Disable program_dis;Manual mode;select Bank 0 */
		bank = OV9760_OTP_MODE_CTRL_BANK_0|index;
		/* select bank number */
		ret = ov9760_write_reg(client, OV9760_8BIT,
			OV9760_OTP_MODE_CTRL, bank);
		if(ret)
			return ret;
		/* enable otp read */
		ret = ov9760_write_reg(client, OV9760_8BIT,
			OV9760_OTP_LOAD_CTRL, OV9760_OTP_LOAD_CTRL_ENABLE);
		if(ret)
			return ret;
		usleep_range(5000, 5100); /*wait OTP load ready*/

		ret = ov9760_read_otp(client, OV9760_OTP_BANK_SIZE, OV9760_OTP_BANK_START, ov9760_data_ptr + index*OV9760_OTP_BANK_SIZE);
		if (ret) {
			dev_err(&client->dev, "read from bank:%d failed", index);
			return ret;
		}
		/* disable otp read */
		ret = ov9760_write_reg(client, OV9760_8BIT,
			OV9760_OTP_LOAD_CTRL, OV9760_OTP_LOAD_CTRL_DISABLE);
		if(ret)
			return ret;
	}

	*ov9760_size = OV9760_OTP_BANK_NUM * OV9760_OTP_BANK_SIZE;

	return 0;
}
#endif

u32 ov9760_otp_trans(const u8 *ov9760_data_ptr, const u32 ov9760_size, u8 *otp_data_ptr, u32 *otp_size)
{
	u32 ret = 0;
	if(ov9760_data_ptr == NULL || ov9760_size == 0)
		return ENULL_OV9760;

	if(otp_data_ptr == NULL || otp_size == 0)
		return ENULL_OTP;

	/* initialize ov9760_data_group_pos */
	ov9760_data_group_pos.module_info = 0;
	ov9760_data_group_pos.lsc_light_source_one = 0;
	ov9760_data_group_pos.lsc_light_source_one_valid = 0;
	ov9760_data_group_pos.lsc_light_source_two = 0;
	ov9760_data_group_pos.lsc_light_source_two_valid = 0;
	ov9760_data_group_pos.awb_light_source_one = 0;
	ov9760_data_group_pos.awb_light_source_two = 0;

	/* check the whole ov9760 data whether is correction */
	ret = ov9760_data_check(ov9760_data_ptr,ov9760_size);
	if(ret) {
		OTP_LOG("%s %d data check failed\n", __func__, __LINE__);
		return ret;
	}

	OTP_LOG("\noffset--module info:%x ls1:%x lsc2:%x awb1:%x awb2:%x\n\n", ov9760_data_group_pos.module_info,
				ov9760_data_group_pos.lsc_light_source_one,
				ov9760_data_group_pos.lsc_light_source_two,
				ov9760_data_group_pos.awb_light_source_one,
				ov9760_data_group_pos.awb_light_source_two);
	/* initialize otp_data_ptr to ready for copy */
	memset(otp_data_ptr, 0, *otp_size);
	ret = otp_data_copy(ov9760_data_ptr,ov9760_size, otp_data_ptr, otp_size);
	if(ret) {
		OTP_LOG("%s %d data copy failed\n", __func__, __LINE__);
		return ret;
	}

	return ret;
}

static u32 ov9760_data_check(const u8 *ov9760_data_ptr, u32 ov9760_size)
{
	u32 offset = 0;
	u32 ret = 0;

	/* check ov9760 module information table */
	offset = OV9760_MODULE_INFORMATION_OFFSET;
	ret = check_ov9760_Module_Information(ov9760_data_ptr,offset);
	if(ret == -1) {
		OTP_LOG("%s %d data check failed\n", __func__, __LINE__);
		return ret;
	}
	else
		ov9760_data_group_pos.module_info = ret;/* record the valid position */

	dump_ov9760_module_info(ov9760_data_ptr);
	/* check ov9760 AWB of light source 1 table */
	offset = OV9760_AWB_LIGHT_SOURCE_ONE_OFFSET;
	ret = check_ov9760_AWB_light_source(ov9760_data_ptr,offset);
	if(ret == -1) {
		OTP_LOG("%s %d data check failed\n", __func__, __LINE__);
		return ret;
	}
	else
		ov9760_data_group_pos.awb_light_source_one = ret;

	/* check ov9760 AWB of light source 2 table */
	offset = OV9760_AWB_LIGHT_SOURCE_TWO_OFFSET;
	ret = check_ov9760_AWB_light_source(ov9760_data_ptr,offset);
	if(ret == -1) {
		OTP_LOG("%s %d data check failed\n", __func__, __LINE__);
		return ret;
	}
	else
		ov9760_data_group_pos.awb_light_source_two = ret;

	/* check ov9760 LSC of light source 1 table */
	offset = OV9760_LSC_LIGHT_SOURCE_ONE_OFFSET;
	ret = check_ov9760_LSC_light_source(ov9760_data_ptr,offset);
	if(ret == -1) {
		OTP_LOG("%s %d data check failed\n", __func__, __LINE__);
		ov9760_data_group_pos.lsc_light_source_one = 0;
		ov9760_data_group_pos.lsc_light_source_one_valid = 0;
	}
	else {
		ov9760_data_group_pos.lsc_light_source_one = ret;
		ov9760_data_group_pos.lsc_light_source_one_valid = 1;
	}

	/* check ov9760 LSC of light source 2 table */
	offset = OV9760_LSC_LIGHT_SOURCE_TWO_OFFSET;
	ret = check_ov9760_LSC_light_source(ov9760_data_ptr,offset);
	if(ret == -1) {
		OTP_LOG("%s %d data check failed\n", __func__, __LINE__);
		ov9760_data_group_pos.lsc_light_source_two = 0;
		ov9760_data_group_pos.lsc_light_source_two_valid = 0;
	}
	else {
		ov9760_data_group_pos.lsc_light_source_two = ret;
		ov9760_data_group_pos.lsc_light_source_two_valid = 1;
	}

	if (ov9760_data_group_pos.lsc_light_source_two_valid &&
		ov9760_data_group_pos.lsc_light_source_one_valid) {
		if(!check_ov9760_first_mp_module(ov9760_data_ptr)) {
			/* all data is valid, will check overall sum*/
			ret = check_all_calibration_data(ov9760_data_ptr,
					ov9760_data_group_pos.awb_light_source_two,
					awb_light_src_two_group_addr[0].len);
			if(ret) {
				OTP_LOG("overall check failed\n");
				return -1;
			}
		}
	}

	return 0;
}

/*
 *As the fist mass production module has some proble
 * we have to check it and workaround;
 */
static void dump_ov9760_module_info(const u8 *ov9760_data_ptr) {
	module_info_t *info;

	info = (module_info_t *) &ov9760_data_ptr[ov9760_data_group_pos.module_info];

	OTP_LOG("module information:\n");
	OTP_LOG("mid:%d calibration vsersion:%d \n", info->mid, info->calibration_version);
	OTP_LOG("year:%d month:%d day:%d\n", info->year, info->month, info->day);
	OTP_LOG("IDs of sensor:%d lens:%d vcm:%d ic:%d IR/BG:%d\n", info->sensor_id,
			info->lens_id, info->vcm_id, info->driver_ic_id, info->IR_BG_id);
	OTP_LOG("color temp:%d AF/FF:%d light:%d\n", info->color_ctc, info->af_flag, info->light_type);
}
static u32 check_ov9760_first_mp_module(const u8 *ov9760_data_ptr)
{

	module_info_t *info;

	info = (module_info_t *) &ov9760_data_ptr[ov9760_data_group_pos.module_info];

	/* the first mass production is befor 10/4/2014*/
	if ((info->year == 14) && (info->month == 4) && (info->day < 10)) {
		OTP_LOG("CAUTION!!! this a module from fist production befor April 10th, 2014\n");
		return 1;
	}

	return 0;
}

static u32 check_ov9760_Module_Information(const u8 *ov9760_data_ptr, u32 offset)
{
	u8 flag = ov9760_data_ptr[offset];
	int i;
	int ret;

	OTP_LOG("%s %d group check offset:%x flag:%x\n", __func__, __LINE__, offset, flag);
	for (i = 2; i >= 0; i--) {
		if ((flag >> ((3 - i) << 1)) & 0x01) break;
	}

	if (i < 0) {
		OTP_LOG("no valid module info found\n");
		return -1;
	}
	OTP_LOG("Module info found:%d start:0x%x len:0x%x\n", i,
				module_info_group_addr[i].start,
				module_info_group_addr[i].len);

	ret = check_ov9760_Group(ov9760_data_ptr, module_info_group_addr[i].start, module_info_group_addr[i].len);
	if (ret) {
		OTP_LOG("module info sum check failed\n");
		return -1;
	}

	return module_info_group_addr[i].start;
}

static u32 check_ov9760_LSC_light_source(const u8 *ov9760_data_ptr, u32 offset)
{
	u8 flag = ov9760_data_ptr[offset];
	ov9760_group_address_t temp_group_addr;
	int ret;

	OTP_LOG("%s %d group check offset:%x flag:%x \n", __func__, __LINE__, offset, flag);
	if (offset == OV9760_LSC_LIGHT_SOURCE_TWO_OFFSET) {
		temp_group_addr = lsc_light_src_two_group_addr;
	} else {
		temp_group_addr = lsc_light_src_one_group_addr;
	}

	if (!(flag & (0x01 << 6))) {
		OTP_LOG("no valid lsc group found\n");
		return -1;
	}

	OTP_LOG("lsc found start:0x%x len:0x%x\n",  temp_group_addr.start,
				temp_group_addr.len);

	ret = check_ov9760_Group(ov9760_data_ptr, temp_group_addr.start, temp_group_addr.len);
	if (ret) {
		OTP_LOG("module info sum check failed\n");
		return -1;
	}

	return temp_group_addr.start;
}


static u32 check_ov9760_AWB_light_source(const u8 *ov9760_data_ptr, u32 offset)
{
	u8 flag = ov9760_data_ptr[offset];
	ov9760_group_address_t *group_addr;
	int i;
	int ret;

	OTP_LOG("%s %d offset:%x flag:%x\n", __func__, __LINE__, offset, flag);

	if (offset == OV9760_AWB_LIGHT_SOURCE_TWO_OFFSET) {
		group_addr = awb_light_src_two_group_addr;
	} else {
		group_addr = awb_light_src_one_group_addr;
	}

	for (i = 2; i >= 0; i--) {
		if ((flag >> ((3 - i) << 1)) & 0x01) break;
	}

	if (i < 0) {
		OTP_LOG("no valid awb info found\n");
		return -1;
	}

	OTP_LOG("awb found:%d start:0x%x len:0x%x\n", i,
				group_addr[i].start,
				group_addr[i].len);
	ret = check_ov9760_Group(ov9760_data_ptr, group_addr[i].start, group_addr[i].len);
	if (ret) {
		OTP_LOG("module info sum check failed\n");
		return -1;
	}

	return group_addr[i].start;
}

/*
 * 1.5000K LSC bank 3: 0x3D00~ bank 11:0x3D0E;
 * 2.one effective group of AWB light one 1)bank 12:0x3D00~0x3D0C 2)bank 13:0x3D00~0x3D0C 3)bank 14:0x3D00~0x3D0C
 * 3.3000K LSC bank 15: 0x3D04~ bank 24:0x3D00;
 * 4.one effective group of AWB light two 1)bank 24:0x3D05~0x3D0E 2)bank 25:0x3D05~0x3D0E 3)bank 26:0x3D05~0x3D0E
 */
static u32 check_all_calibration_data(const u8 *ov9760_data_ptr, u32 offset, u32 len)
{
	u32 i = 0;
	u32 sum = 0;
	u32 sum_read = 0;
	u8 tmp[DATA_BUF_SIZE];
	u32 tmp_len=0;

	memset(tmp,0,DATA_BUF_SIZE);
	/* LSC information of light source 1 */
	memcpy(tmp+tmp_len, ov9760_data_ptr+ov9760_data_group_pos.lsc_light_source_one,
		OV9760_LSC_LIGHT_SOURCE_GROUP_LEN-1);
	tmp_len += OV9760_LSC_LIGHT_SOURCE_GROUP_LEN-1;

	/* LSC information of light source 2 */
	memcpy(tmp+tmp_len, ov9760_data_ptr+ov9760_data_group_pos.lsc_light_source_two+2,
		OV9760_LSC_LIGHT_SOURCE_GROUP_LEN-3);
	tmp_len += OV9760_LSC_LIGHT_SOURCE_GROUP_LEN-3;

	/* AWB information of light source 1 */
	memcpy(tmp+tmp_len, ov9760_data_ptr+ov9760_data_group_pos.awb_light_source_one,
		OV9760_AWB_LIGHT_SOURCE_GROUP_LEN-3);
	tmp_len += OV9760_AWB_LIGHT_SOURCE_GROUP_LEN-3;

	/* AWB information of light source 2 */
	memcpy(tmp+tmp_len, ov9760_data_ptr+ov9760_data_group_pos.awb_light_source_two+3,
		OV9760_AWB_LIGHT_SOURCE_GROUP_LEN-6);
	tmp_len += OV9760_AWB_LIGHT_SOURCE_GROUP_LEN-6;

	/* CRC_16 checksum */
	for(i=0,sum=0;i<tmp_len;i++)  {
		sum += tmp[i];
		/* OTP_LOG("data: %2x sum:%4x NO:%3d \n", tmp[i], sum, i); */
	}

	sum = sum & 0xffff;
	sum_read = (ov9760_data_ptr[offset+len-2] << 8) | ov9760_data_ptr[offset+len-3];
	OTP_LOG("sum:%x sum_read:%x sum_read1:%x sum_read2:%x\n", sum, sum_read, ov9760_data_ptr[offset+len-2], ov9760_data_ptr[offset+len-3]);
	if(sum != sum_read) {
 		OTP_LOG("%s %d overall checksum failed\n", __func__, __LINE__);
 		return -1;
	}

	return 0;
}
static u32 check_ov9760_Group(const u8 *ov9760_data_ptr, u32 offset, u32 len)
{
	u32 i = 0;
	u32 sum = 0;
	/* calculated(sum) from all bytes before checksum */
	for(i=0;i<len-1;i++) {
		sum += ov9760_data_ptr[offset+i];
		//OTP_LOG("%x sum:%x\n", ov9760_data_ptr[offset+i], sum);
	}

	sum = sum % 255 + 1;
	if(sum != ov9760_data_ptr[offset+i]) {
		OTP_LOG("%s %d offset:%d len:%d sum:%x sum_read:%x checksum failed\n", __func__, __LINE__, offset, len, sum, ov9760_data_ptr[offset+i]);
		return -1;
	}
	return 0;
}

static u16 get_nvm_checksum(const u8 *a_data_ptr, u32 a_size)
{
	u16 crc = 0;
	u32 i;

	for (i = 0; i < a_size; i++)
	{
		u8 index = crc ^ a_data_ptr[i];
		crc = (crc >> 8) ^ crc16table[index];
	}

	return crc;
}

static u32 otp_data_copy(const u8 *ov9760_data_ptr, const u32 ov9760_size, u8 *otp_data_ptr, u32 *otp_size)
{
	u32 ret = 0;
	module_info_t *info;

	*otp_size = 0;
	ret = otp_Module_Information_copy(ov9760_data_ptr,ov9760_size,otp_data_ptr,otp_size); 
	if(ret == -1) {
		OTP_LOG("%s %d module info copy failed\n", __func__, __LINE__);
		return ret;
	}

	ret = otp_AWB_LSC_light_source_copy(ov9760_data_ptr,ov9760_size,otp_data_ptr,otp_size);
	if(ret == -1) {
		OTP_LOG("%s %d light source copy failed\n", __func__, __LINE__);
		return ret;
	}

	info = (module_info_t *) &ov9760_data_ptr[ov9760_data_group_pos.module_info];
	if (info->mid == MID_OFILM) {
		memcpy(otp_data_ptr + OFFSET_CIEXY, std_cie_xy, sizeof(std_cie_xy));
		printk("second source found, replace xy to first source\r\n");
	} else {
		printk("first source found, keep xy in OTP\r\n");
	}

	ret = otp_crc(otp_data_ptr,otp_size);
	if(ret == -1) {
		OTP_LOG("%s %d otp_crc failed\n", __func__, __LINE__);
		return ret;
	}

	return 0;
}

static u32 otp_AWB_LSC_light_source_copy(const u8 *ov9760_data_ptr, const u32 ov9760_size, u8 *otp_data_ptr, u32 *otp_size)
{
	u32 otp_offset = 0;
	u32 ov9760_offset = 0;
	u8 fake_af[] = {0x01, 0x00, 0x0a, 0x8a, 0x02, 0x2C, 0x01, 0x64, 0x00, 0x84, 0x03};

	/* otp_data[0]:major_version and; otp_data[1]:mino_version; otp_data[2]:n_lights */
	otp_offset = 0;
	ov9760_offset = ov9760_data_group_pos.awb_light_source_one;
	memcpy(otp_data_ptr+otp_offset,ov9760_data_ptr+ov9760_offset,2);

    /* otp_data[2]:n_lights */
	otp_offset += 2;
	memcpy(otp_data_ptr+otp_offset,fake_af, sizeof(fake_af));

	/* otp_data[2]:n_lights */
	otp_offset += sizeof(fake_af);
	ov9760_offset += 2;
	memcpy(otp_data_ptr+otp_offset,ov9760_data_ptr+ov9760_offset,1);
	/* otp_data[3]:cie_x1 of light source 1; otp_data[4]:cie_x1 of light source 2*/
	/* for hack*/
	otp_offset += 1;
	ov9760_offset += 1;
	memcpy(otp_data_ptr+otp_offset,ov9760_data_ptr+ov9760_offset,1);
	if (check_ov9760_first_mp_module(ov9760_data_ptr)) memset(otp_data_ptr+otp_offset, 0x55, 1);

	otp_offset += 1;
	ov9760_offset = ov9760_data_group_pos.awb_light_source_two+3;
	memcpy(otp_data_ptr+otp_offset,ov9760_data_ptr+ov9760_offset,1);
	if (check_ov9760_first_mp_module(ov9760_data_ptr)) memset(otp_data_ptr+otp_offset, 0xB8, 1);

	/* otp_data[5]:cie_y1 of light source 1; otp_data[6]:cie_y1 of light source 2*/
	otp_offset += 1;
	ov9760_offset = ov9760_data_group_pos.awb_light_source_one+4;
	memcpy(otp_data_ptr+otp_offset,ov9760_data_ptr+ov9760_offset,1);
	if (check_ov9760_first_mp_module(ov9760_data_ptr)) memset(otp_data_ptr+otp_offset, 0x77, 1);

	otp_offset += 1;
	ov9760_offset = ov9760_data_group_pos.awb_light_source_two+4;
	memcpy(otp_data_ptr+otp_offset,ov9760_data_ptr+ov9760_offset,1);
	if (check_ov9760_first_mp_module(ov9760_data_ptr)) memset(otp_data_ptr+otp_offset, 0x97, 1);

	/* otp_data[7]:LSC_width; otp_data[8]: LSC_height */
	/* LSC of light source 1:lsc_fraq_bits; LSCgrid */
	otp_offset += 1;
	ov9760_offset = ov9760_data_group_pos.lsc_light_source_one;
	memcpy(otp_data_ptr+otp_offset,ov9760_data_ptr+ov9760_offset,OV9760_LSC_LIGHT_SOURCE_GROUP_LEN-1);

	if (ov9760_data_group_pos.lsc_light_source_one_valid == 0) {
		/* the lsc1 data in otp have problem, replace with default*/
		memcpy(otp_data_ptr+otp_offset,lsc1_default,OV9760_LSC_LIGHT_SOURCE_GROUP_LEN-1);
	}

	/* LSC of light source 2:lsc_fraq_bits; LSCgrid */
	otp_offset += OV9760_LSC_LIGHT_SOURCE_GROUP_LEN-1;
	ov9760_offset = ov9760_data_group_pos.lsc_light_source_two+2;
	memcpy(otp_data_ptr+otp_offset,ov9760_data_ptr+ov9760_offset,OV9760_LSC_LIGHT_SOURCE_GROUP_LEN-3);

	if(ov9760_data_group_pos.lsc_light_source_two_valid == 0) {
		/* the lsc1 data in otp have problem, replace with default*/
		memcpy(otp_data_ptr+otp_offset, lsc2_default, OV9760_LSC_LIGHT_SOURCE_GROUP_LEN-3);
	}

	/* AWB of light source 1&2 */
	/* AWB_ch1_source_1 */
	otp_offset += OV9760_LSC_LIGHT_SOURCE_GROUP_LEN-3;
	ov9760_offset = ov9760_data_group_pos.awb_light_source_one+5;
	memcpy(otp_data_ptr+otp_offset,ov9760_data_ptr+ov9760_offset,2);

	/* AWB_ch1_source_2 */
	otp_offset += 2;
	ov9760_offset = ov9760_data_group_pos.awb_light_source_two+5;
	memcpy(otp_data_ptr+otp_offset,ov9760_data_ptr+ov9760_offset,2);

	/* AWB_ch2_source_1 */
	otp_offset += 2;
	ov9760_offset = ov9760_data_group_pos.awb_light_source_one+7;
	memcpy(otp_data_ptr+otp_offset,ov9760_data_ptr+ov9760_offset,2);

	/* AWB_ch2_source_2 */
	otp_offset += 2;
	ov9760_offset = ov9760_data_group_pos.awb_light_source_two+7;
	memcpy(otp_data_ptr+otp_offset,ov9760_data_ptr+ov9760_offset,2);

	/* AWB_ch3_source_1 */
	otp_offset += 2;
	ov9760_offset = ov9760_data_group_pos.awb_light_source_one+9;
	memcpy(otp_data_ptr+otp_offset,ov9760_data_ptr+ov9760_offset,2);

	/* AWB_ch3_source_2 */
	otp_offset += 2;
	ov9760_offset = ov9760_data_group_pos.awb_light_source_two+9;
	memcpy(otp_data_ptr+otp_offset,ov9760_data_ptr+ov9760_offset,2);

	/* AWB_ch4_source_1 */
	otp_offset += 2;
	ov9760_offset = ov9760_data_group_pos.awb_light_source_one+0x0B;
	memcpy(otp_data_ptr+otp_offset,ov9760_data_ptr+ov9760_offset,2);

	/* AWB_ch4_source_2 */
	otp_offset += 2;
	ov9760_offset = ov9760_data_group_pos.awb_light_source_two+0x0B;
	memcpy(otp_data_ptr+otp_offset,ov9760_data_ptr+ov9760_offset,2);

	*otp_size = otp_offset + 2;

	return 0;
}

static u32 otp_crc(u8 *otp_data_ptr, u32 *otp_size)
{
	u8 crc_buf[2];
	u16 crc;

	if((otp_data_ptr == NULL) || (*otp_size == 0))
		return -1;
	crc = get_nvm_checksum((u8*)otp_data_ptr,*otp_size);

	memset(crc_buf, 0, 2);
	/* Little endian */
	crc_buf[0] = (u8)crc;
	crc_buf[1] = (u8)(crc>>8);
	memcpy(otp_data_ptr+(*otp_size),crc_buf,2);
	*otp_size += 2;

	return 0;
}

static u32 otp_Module_Information_copy(const u8 *ov9760_data_ptr, const u32 ov9760_size, u8 *otp_data_ptr, u32 *otp_size)
{
	/* There's nothing to copy */
	return 0;
}
