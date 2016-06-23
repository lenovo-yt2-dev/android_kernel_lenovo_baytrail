/* Release Version: irci_master_20140212_0006 */
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

/*! \file */

#ifndef _IA_CSS_H_
#define _IA_CSS_H_

#include <type_support.h>
#include <stdarg.h> /* va_list */
#include "ia_css_types.h"
#include "ia_css_3a.h"
#include "ia_css_acc_types.h"
#include "ia_css_buffer.h"
#include "ia_css_control.h"
#include "ia_css_dvs.h"
#include "ia_css_env.h"
#include "ia_css_err.h"
#include "ia_css_event_public.h"
#include "ia_css_firmware.h"
#include "ia_css_frame_public.h"
#include "ia_css_input_port.h"
#include "ia_css_irq.h"
#include "ia_css_metadata.h"
#include "ia_css_pipe_public.h"
#include "ia_css_prbs.h"
#include "ia_css_stream_format.h"
#include "ia_css_stream_public.h"
#include "ia_css_tpg.h"

/* a common size for the version arrays */
#define MAX_VERSION_SIZE 	500

struct ia_css_properties {
	int  gdc_coord_one;
	bool l1_base_is_index; /**< Indicate whether the L1 page base
				    is a page index or a byte address. */
	enum ia_css_vamem_type vamem_type;
};

/* ===== GENERIC ===== */

/** @brief Retrieves the current CSS version
 * @param[out]	version		A pointer to a buffer where to put the generated
 *				version string. NULL is ignored.
 *
 * This function generates and returns the version string. If FW is loaded, it
 * attaches the FW version.
 */
enum ia_css_err
ia_css_get_version(char *version, int max_size);


/** @brief Get hardware properties
 * @param[in,out]	properties The hardware properties
 * @return	None
 *
 * This function returns a number of hardware properties.
 */
void
ia_css_get_properties(struct ia_css_properties *properties);

/** @brief Invalidate the MMU internal cache.
 * @return	None
 *
 * This function triggers an invalidation of the translate-look-aside
 * buffer (TLB) that's inside the CSS MMU. This function should be called
 * every time the page tables used by the MMU change.
 */
void
ia_css_mmu_invalidate_cache(void);

/* Convenience functions for alloc/free of certain datatypes */

/** @brief Morphing table
 * @param[in]	width Width of the morphing table.
 * @param[in]	height Height of the morphing table.
 * @return		Pointer to the morphing table
*/
struct ia_css_morph_table *
ia_css_morph_table_allocate(unsigned int width, unsigned int height);

/** @brief Free the morph table
 * @param[in]	me Pointer to the morph table.
 * @return		None
*/
void
ia_css_morph_table_free(struct ia_css_morph_table *me);

/** @brief Shading table
 * @param[in]	width Width of the shading table.
 * @param[in]	height Height of the shading table.
 * @return		Pointer to the shading table
*/
struct ia_css_shading_table *
ia_css_shading_table_alloc(unsigned int width,
			   unsigned int height);

/** @brief Free shading table
 * @param[in]	table Pointer to the shading table.
 * @return		None
*/
void
ia_css_shading_table_free(struct ia_css_shading_table *table);

/** Backward compatible for CSS API 2.0 only
 * TO BE REMOVED when all drivers move to CSS API 2.1.
 */
/** @brief Specify a CSS MIPI frame buffer.
 *
 * @param[in]	size_mem_words	The frame size in memory words (32B).
 * @param[in]	contiguous	Allocate memory physically contiguously or not.
 * @return		The error code.
 *
 * Specifies a CSS MIPI frame buffer: size in memory words (32B).
 */
enum ia_css_err
ia_css_mipi_frame_specify(const unsigned int	size_mem_words,
				const bool contiguous);

#if !defined(HAS_NO_INPUT_SYSTEM)
/** @brief Register size of a CSS MIPI frame for check during capturing.
 *
 * @param[in]	port	CSI-2 port this check is registered.
 * @param[in]	size_mem_words	The frame size in memory words (32B).
 * @return		Return the error in case of failure. E.g. MAX_NOF_ENTRIES REACHED
 *
 * Register size of a CSS MIPI frame to check during capturing. Up to
 *		IA_CSS_MIPI_SIZE_CHECK_MAX_NOF_ENTRIES entries per port allowed. Entries are reset
 *		when stream is stopped.
 *
 *
 */
enum ia_css_err
ia_css_mipi_frame_enable_check_on_size(const enum ia_css_csi2_port port,
				const unsigned int	size_mem_words);
#endif

#endif /* _IA_CSS_H_ */
