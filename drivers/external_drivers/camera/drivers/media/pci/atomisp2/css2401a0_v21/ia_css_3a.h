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

#ifndef __IA_CSS_3A_H
#define __IA_CSS_3A_H

#include <type_support.h>
#include "ia_css_types.h"

enum ia_css_3a_tables {
	IA_CSS_S3A_TBL_HI,
	IA_CSS_S3A_TBL_LO,
	IA_CSS_RGBY_TBL,
	IA_CSS_NUM_3A_TABLES
};

/** Structure that holds 3A statistics in the ISP internal
 * format. Use ia_css_get_3a_statistics() to translate
 * this to the format used on the host (3A library).
 * */
struct ia_css_isp_3a_statistics {
	union {
		struct {
			ia_css_ptr s3a_tbl;
		} dmem;
		struct {
			ia_css_ptr s3a_tbl_hi;
			ia_css_ptr s3a_tbl_lo;
		} vmem;
	} data;
	struct {
		ia_css_ptr rgby_tbl;
	} data_hmem;
	uint32_t exp_id;
};
#define SIZE_OF_DMEM_STRUCT						\
	(SIZE_OF_IA_CSS_PTR)

#define SIZE_OF_VMEM_STRUCT						\
	(2 * SIZE_OF_IA_CSS_PTR)

#define SIZE_OF_DATA_UNION						\
	(MAX(SIZE_OF_DMEM_STRUCT, SIZE_OF_VMEM_STRUCT))

#define SIZE_OF_DATA_HMEM_STRUCT					\
	(SIZE_OF_IA_CSS_PTR)

#define SIZE_OF_IA_CSS_ISP_3A_STATISTICS_STRUCT				\
	(SIZE_OF_DATA_UNION +						\
	SIZE_OF_DATA_HMEM_STRUCT +					\
	sizeof(uint32_t))

#if defined(IS_ISP_2500_SYSTEM)
/** @brief Copy 4A statistics from an ISP/ACC buffer to a host buffer.
 * @param[in]	host_stats Host buffer.
 * @param[in]	isp_stats ISP buffer.
 * @return		None
 */
struct ia_css_4a_statistics;
void ia_css_get_4a_statistics(struct ia_css_4a_statistics           *host_stats,
		const struct ia_css_isp_3a_statistics *isp_stats);
#endif

/** @brief Copy 3A statistics from an ISP/ACC buffer to a host buffer.
 * @param[in]	host_stats Host buffer.
 * @param[in]	isp_stats ISP buffer.
 * @return		None
 *
 * This may include a translation step as well depending
 * on the ISP version.
 * Always use this function, never copy the buffer directly.
 */
void
ia_css_get_3a_statistics(struct ia_css_3a_statistics           *host_stats,
			 const struct ia_css_isp_3a_statistics *isp_stats);

/* Convenience functions for alloc/free of certain datatypes */

/** @brief Allocate memory for the 3a statistics on the ISP
 * @param[in]	grid The grid.
 * @return		Pointer to the allocated 3a statistics buffer on the ISP
*/
struct ia_css_isp_3a_statistics *
ia_css_isp_3a_statistics_allocate(const struct ia_css_3a_grid_info *grid);

/** @brief Free the 3a statistics memory on the isp
 * @param[in]	me Pointer to the 3a statistics buffer on the ISP.
 * @return		None
*/
void
ia_css_isp_3a_statistics_free(struct ia_css_isp_3a_statistics *me);

/** @brief Allocate memory for the 3a statistics on the host
 * @param[in]	grid The grid.
 * @return		Pointer to the allocated 3a statistics buffer on the host
*/
struct ia_css_3a_statistics *
ia_css_3a_statistics_allocate(const struct ia_css_3a_grid_info *grid);

/** @brief Free the 3a statistics memory on the host
 * @param[in]	me Pointer to the 3a statistics buffer on the host.
 * @return		None
 */
void
ia_css_3a_statistics_free(struct ia_css_3a_statistics *me);

#endif /* __IA_CSS_3A_H */
