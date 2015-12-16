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

#ifndef __ISP_OP1W_H_INCLUDED__
#define __ISP_OP1W_H_INCLUDED__

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

#ifndef STORAGE_CLASS_ISP_OP1W_H
#define STORAGE_CLASS_ISP_OP1W_H extern
#endif

/*
 * Single-precision data type specification
 */

#include "isp_op1w_types.h"
#include "isp_op2w_types.h" // for doubling operations.

/*
 * Single-precision prototype specification
 */

/* Arithmetic */

/** @brief bitwise AND
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		bitwise and of both input arguments
 *
 * This function will calculate the bitwise and.
 * result = _a & _b
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_and(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief bitwise OR
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		bitwise or of both input arguments
 *
 * This function will calculate the bitwise or.
 * result = _a | _b
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_or(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief bitwise XOR
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		bitwise xor of both input arguments
 *
 * This function will calculate the bitwise xor.
 * result = _a ^ _b
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_xor(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief bitwise inverse
 *
 * @param[in] _a	first argument
 *
 * @return		bitwise inverse of both input arguments
 *
 * This function will calculate the bitwise inverse.
 * result = ~_a
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_inv(
    const tvector1w     _a);

/* Additive */

/** @brief addition
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		sum of both input arguments
 *
 * This function will calculate the sum of the input arguments.
 * in case of overflow it will wrap around.
 * result = _a + _b
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_add(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief substraction
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_b substracted from _a.
 *
 * This function will substract _b from _a.
 * in case of overflow it will wrap around.
 * result = _a - _b
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_sub(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief saturated addition
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		saturated sum of both input arguments
 *
 * This function will calculate the sum of the input arguments.
 * in case of overflow it will saturate
 * result = CLIP(_a + _b, MIN_RANGE, MAX_RANGE);
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_addsat(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief saturated substraction
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		saturated substraction of both input arguments
 *
 * This function will substract _b from _a.
 * in case of overflow it will saturate
 * result = CLIP(_a - _b, MIN_RANGE, MAX_RANGE);
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_subsat(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief substraction with shift right
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		(a - b) >> 1
 *
 * This function will substract _b from _a.
 * and right shift the result with 1 bit.
 * result = (_a - _b) >> 1
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_subasr1(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief saturated substraction
 *
 * @param[in] _a	input
 *
 * @return		absolute value of the input
 *
 * This function will calculate the absolute value of the input
 * if (_a > 0) return _a;<br>
 * else return -_a;<br>
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_abs(
    const tvector1w     _a);

/** @brief subabs
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		abs(a-b);
 *
 * This function will calculate the absolute value
 * of the difference of both inputs
 * result = abs(_a - _b);
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_subabs(
    const tvector1w     _a,
    const tvector1w     _b);

/* Multiplicative */

/** @brief doubling multiply
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		product of _a and _b
 *
 * This function will calculate the product
 * of the input arguments and returns a double
 * precision result
 * result = _a * _b;
 */
STORAGE_CLASS_ISP_OP1W_H tvector2w OP_1w_muld(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief integer multiply
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		product of _a and _b
 *
 * This function will calculate the product
 * of the input arguments and returns the LSB
 * aligned single precison result.
 * result = _a * _b;
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_mul(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief fractional multiply
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		product of _a and _b
 *
 * This function will calculate the product
 * of the input arguments and returns the MSB
 * aligned single precison result.
 * result = _a * _b >> NUM_BITS;
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_qmul(
    const tvector1w     _a,
    const tvector1w     _b);

/* Comparative */

/** @brief equal
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_a == _b
 *
 * This function will return true if both inputs
 * are equal, and false if not equal.
 */
STORAGE_CLASS_ISP_OP1W_H tflags OP_1w_eq(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief not equal
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_a != _b
 *
 * This function will return false if both inputs
 * are equal, and true if not equal.
 */
STORAGE_CLASS_ISP_OP1W_H tflags OP_1w_ne(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief less or equal
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_a <= _b
 *
 * This function will return true if _a is smaller
 * or equal than _b.
 */
STORAGE_CLASS_ISP_OP1W_H tflags OP_1w_le(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief less then
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_a < _b
 *
 * This function will return true if _a is smaller
 * than _b.
 */
STORAGE_CLASS_ISP_OP1W_H tflags OP_1w_lt(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief greater or equal
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_a >= _b
 *
 * This function will return true if _a is greater
 * or equal than _b.
 */
STORAGE_CLASS_ISP_OP1W_H tflags OP_1w_ge(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief greater than
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_a > _b
 *
 * This function will return true if _a is greater
 * than _b.
 */
STORAGE_CLASS_ISP_OP1W_H tflags OP_1w_gt(
    const tvector1w     _a,
    const tvector1w     _b);

/* Shift */

/** @brief aritmetic shift right
 *
 * @param[in] _a	input
 * @param[in] _b	shift amount
 *
 * @return		_a >> _b
 *
 * This function will shift _a with _b bits to the right.
 * preserving the sign bit.
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_asr(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief aritmetic shift left
 *
 * @param[in] _a	input
 * @param[in] _b	shift amount
 *
 * @return		_a << _b
 *
 * This function will shift _a with _b bits to the left.
 *
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_asl(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief aritmetic shift right with rounding
 *
 * @param[in] _a	input
 * @param[in] _b	shift amount
 *
 * @return		_a >> _b
 *
 * This function will shift _a with _b bits to the right.
 * and depending on the rounding mode of the core
 * it will round to nearest or to nearest even.
 *
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_asrrnd(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief logical shift left
 *
 * @param[in] _a	input
 * @param[in] _b	shift amount
 *
 * @return		_a << _b
 *
 * This function will shift _a with _b bits to the left.
 * It will insert zero's on the right.
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_lsl(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief saturated logical shift left
 *
 * @param[in] _a	input
 * @param[in] _b	shift amount
 *
 * @return		_a << _b
 *
 * This function will shift _a with _b bits to the left.
 * It will insert zero's on the right, and clip the result.
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_lslsat(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief logical shift right
 *
 * @param[in] _a	input
 * @param[in] _b	shift amount
 *
 * @return		_a >> _b
 *
 * This function will shift _a with _b bits to the right.
 * It will insert zero's on the left
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_lsr(
    const tvector1w     _a,
    const tvector1w     _b);

/** @brief logical shift right with rounding
 *
 * @param[in] _a	input
 * @param[in] _b	shift amount
 *
 * @return		_a >> _b
 *
 * This function will shift _a with _b bits to the right.
 * It will insert zero's on the left
 * and depending on the rounding mode of the core
 * it will round to nearest or to nearest even.
 *
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_lsrrnd(
    const tvector1w     _a,
    const tvector1w     _b);


/* Cast */

/** @brief Cast from int to 1w
 *
 * @param[in] _a	input
 *
 * @return		_a
 *
 * This function cast the input from integer type to
 * single precision. It will also assert on ranges
 *
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_int_cast_to_1w (
    const int           _a);

/** @brief Cast from 1w to int
 *
 * @param[in] _a	input
 *
 * @return		_a
 *
 * This function cast the input from single precision type to
 * integer.
 *
 */
STORAGE_CLASS_ISP_OP1W_H int OP_1w_cast_to_int (
    const tvector1w      _a);

/** @brief Cast from int to 1w
 *
 * @param[in] _a	input
 *
 * @return		_a
 *
 * This function cast the input from single precision type to
 * double precision. lsb aligned.
 *
 */
STORAGE_CLASS_ISP_OP1W_H tvector2w OP_1w_cast_to_2w (
    const tvector1w     _a);

/** @brief Cast from int to 1w
 *
 * @param[in] _a	input
 *
 * @return		_a
 *
 * This function cast the input from double precision type to
 * single precision. MSB's will be truncated.
 *
 */
STORAGE_CLASS_ISP_OP1W_H tvector1w OP_2w_cast_to_1w (
    const tvector2w    _a);

/* clipping */

STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_clip_asym(
    const tvector1w     _a,
    const tvector1w     _b);

STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_clipz(
    const tvector1w     _a,
    const tvector1w     _b);

/* division */

STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_div(
    const tvector1w     _a,
    const tvector1w     _b);

STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_mod(
    const tvector1w     _a,
    const tvector1w     _b);

STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_sqrt(
    const tvector1w     _a,
    const tvector1w     _b);

/* Miscellaneous */

STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_mux(
    const tvector1w     _a,
    const tvector1w     _b,
    const tflags           _c);

STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_avgrnd(
    const tvector1w     _a,
    const tvector1w     _b);

STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_min(
    const tvector1w     _a,
    const tvector1w     _b);

STORAGE_CLASS_ISP_OP1W_H tvector1w OP_1w_max(
    const tvector1w     _a,
    const tvector1w     _b);

#endif /* __ISP_OP1W_H_INCLUDED__ */
