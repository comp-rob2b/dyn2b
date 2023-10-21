// SPDX-License-Identifier: LGPL-3.0
#ifndef DYN2B_FUNCTIONS_DYN2B_H
#define DYN2B_FUNCTIONS_DYN2B_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file array.h
 *
 * This file contains component-wise operations on arrays.
 */


/**
 * Copy an array.
 *
 * `out[i] = in[i]`
 *
 * @param[in] n Number of array entries.
 * @param[in] in The source array with \f$n\f$ entries.
 * @param[out] out The destination array with \f$n\f$ entries.
 */
void dyn2b_cpy_arr(
        int n,
        const double *restrict in,
        double *restrict out);


/**
 * Add the entries from two arrays.
 *
 * `out[offset + i] = in1[i] + in2[i]`
 *
 * @param[in] n Number of array entries.
 * @param[in] offset The offset (number of elements) at which to start writing
 *                   the sum into the destination array.
 * @param[in] in1 The first source array with \f$n\f$ entries.
 * @param[in] in2 The second source array with \f$n\f$ entries.
 * @param[out] out The destination array with \f$offset + n\f$ entries.
 */
void dyn2b_add_arr(
        int n,
        int offset,
        const double *restrict in1,
        const double *restrict in2,
        double *restrict out);


/**
 * Accumulate the entries from the source array to the destination array
 * (in-place addition).
 *
 * `out[offset + i] += in[i]`
 *
 * @param[in] n Number of array entries.
 * @param[in] offset The offset (number of elements) at which to start writing
 *                   the sum into the destination array.
 * @param[in] in The first source array with \f$n\f$ entries.
 * @param[out] out The destination array with \f$offset + n\f$ entries.
 */
void dyn2b_add_arr_i(
        int n,
        int offset,
        const double *restrict in,  // [n]
        double *restrict out);      // [n]


/**
 * Subtract the entries in the second array from the arrayes in the first array.
 *
 * `out[i] = in1[i] - in2[i]`
 *
 * @param[in] n Number of array entries.
 * @param[in] in1 The first source array ("minuend") with \f$n\f$ entries.
 * @param[in] in2 The second source array ("subtrahend") with \f$n\f$ entries.
 * @param[out] out The destination array ("difference") with \f$n\f$ entries.
 */
void dyn2b_sub_arr(
        int n,
        const double *restrict in1,
        const double *restrict in2,
        double *restrict out);


/**
 * Scale the entries in an array.
 *
 * `out[i] = a * in1[i]`
 *
 * @param[in] n Number of array entries.
 * @param[in] a Pointer to the scalar (single array entry).
 * @param[in] in The source array with \f$n\f$ entries.
 * @param[out] out The destination array with \f$n\f$ entries.
 */
void dyn2b_scl_arr(
        int n,
        const double *restrict a,
        const double *restrict in,
        double *restrict out);


/**
 * Invert the entries in an array.
 *
 * `out[i] = -in1[i]`
 *
 * @param[in] n Number of array entries.
 * @param[in] in The source array with \f$n\f$ entries.
 * @param[out] out The destination array with \f$n\f$ entries.
 */
void dyn2b_inv_arr(
        int n,
        const double *restrict in,
        double *restrict out);


#ifdef __cplusplus
}
#endif

#endif
