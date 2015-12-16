/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2014 Intel Corporation. All Rights Reserved.
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

#include "ia_css_frame.h"
#include "ia_css_types.h"
#include "sh_css_defs.h"
#include "ia_css_debug.h"
#include "assert_support.h"
#define IA_CSS_INCLUDE_CONFIGURATIONS
#include "ia_css_isp_configs.h"
#include "isp.h"

#include "ia_css_raw.host.h"

void
ia_css_raw_encode(
	struct sh_css_isp_raw_params *to,
	const struct ia_css_aa_config *from)
{
	to->baf_strength = from->strength;
}

void
ia_css_raw_dump(
	const struct sh_css_isp_raw_params *raw,
	unsigned level)
{
	(void)raw;
	(void)level;
}

static inline unsigned
sh_css_elems_bytes_from_info (unsigned raw_bit_depth)
{
	return CEIL_DIV(raw_bit_depth,8);
}

static inline unsigned
sh_css_stride_from_info (
	enum ia_css_frame_format format,
	unsigned stride_b,
	unsigned raw_bit_depth)
{
	/* padded_width is in terms of elements */
	unsigned stride;
	if (format == IA_CSS_FRAME_FORMAT_RAW_PACKED) {
		stride = ((unsigned)HIVE_ISP_DDR_WORD_BYTES) *
				CEIL_DIV(stride_b,
				(unsigned char)(HIVE_ISP_DDR_WORD_BITS /
					raw_bit_depth));
	} else {
		stride = stride_b * sh_css_elems_bytes_from_info(raw_bit_depth);
	}
	return stride;
}

void
ia_css_raw_config(
	struct sh_css_isp_raw_isp_config *to,
	const struct ia_css_raw_configuration  *from)
{
	unsigned elems_a = ISP_VEC_NELEMS;
	const struct ia_css_frame_info *in_info = from->in_info;
	const struct ia_css_frame_info *internal_info = from->internal_info;
#if !defined(USE_INPUT_SYSTEM_VERSION_2401)
	/* 2401 input system uses input width width */
	in_info = internal_info;
#else
	(void)internal_info;
#endif
	ia_css_dma_configure_from_info(&to->port_b, in_info);
	to->width_a_over_b = elems_a / to->port_b.elems;
	to->port_b.stride /= 2; /* Half BQ lines */

	to->port_b.stride = sh_css_stride_from_info(in_info->format, to->port_b.stride, in_info->raw_bit_depth);

	/* Assume divisiblity here, may need to generalize to fixed point. */
	assert (elems_a % to->port_b.elems == 0);

	to->inout_port_config       = from->pipe->inout_port_config;
	to->format = in_info->format;
	to->required_bds_factor = from->pipe->required_bds_factor;
}

void
ia_css_raw_configure(
	const struct sh_css_sp_pipeline *pipe,
	const struct ia_css_binary      *binary,
	const struct ia_css_frame_info  *in_info,
	const struct ia_css_frame_info  *internal_info)
{
	const struct ia_css_raw_configuration config =
		{ pipe, in_info, internal_info };
	ia_css_configure_raw(binary, &config);
}
