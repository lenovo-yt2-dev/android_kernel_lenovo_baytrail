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

#ifndef __ISP_OP2W_H_INCLUDED__
#define __ISP_OP2W_H_INCLUDED__

/*
 * This file is part of the Multi-precision vector operations exstension package.
 */

/*
 * Single-precision vector operations
 */

/*
 * Prerequisites:
 *
 */

#ifndef STORAGE_CLASS_ISP_OP2W_H
#define STORAGE_CLASS_ISP_OP2W_H extern
#endif

/*
 * Single-precision data type specification
 */

#include "isp_op2w_types.h"

/*
 * Single-precision prototype specification
 */

/* Arithmetic */

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_and(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_or(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_xor(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_inv(
    const tvector2w     _a);

/* Additive */

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_add(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_sub(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_addsat(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_subsat(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_subasr1(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_abs(
    const tvector2w     _a);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_subabs(
    const tvector2w     _a,
    const tvector2w     _b);

/* Multiplicative */

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_mul(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_qmul(
    const tvector2w     _a,
    const tvector2w     _b);

/* Comparative */

STORAGE_CLASS_ISP_OP2W_H tflags OP_2w_eq(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tflags OP_2w_ne(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tflags OP_2w_le(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tflags OP_2w_lt(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tflags OP_2w_ge(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tflags OP_2w_gt(
    const tvector2w     _a,
    const tvector2w     _b);

/* Shift */

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_asr(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_asl(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_asrrnd(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_lsl(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_lslsat(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_lsr(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_lsrrnd(
    const tvector2w     _a,
    const tvector2w     _b);

/* clipping */

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_clip_asym(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_clipz(
    const tvector2w     _a,
    const tvector2w     _b);

/* division */

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_div(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_mod(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_sqrt(
    const tvector2w     _a,
    const tvector2w     _b);

/* Miscellaneous */

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_mux(
    const tvector2w     _a,
    const tvector2w     _b,
    const tflags           _c);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_avgrnd(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_min(
    const tvector2w     _a,
    const tvector2w     _b);

STORAGE_CLASS_ISP_OP2W_H tvector2w OP_2w_max(
    const tvector2w     _a,
    const tvector2w     _b);

#endif /* __ISP_OP2W_H_INCLUDED__ */
