// SPDX-License-Identifier: LGPL-3.0
#ifndef DYN2B_FUNCTIONS_MATRIX_H
#define DYN2B_FUNCTIONS_MATRIX_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file matrix.h
 *
 * This file contains operations on matrices.
 */


/**
 * Copy a matrix.
 *
 * `dst[i, j] = src[i, j]`
 *
 * @param[in] m Number of rows to copy.
 * @param[in] n Number of columns to copy.
 * @param[in] src The source matrix with \f$m \times n\f$ entries.
 * @param[in] lds The leading dimension or stride of the source matrix, i.e. the
 *                number of matrix entries between two rows (\f$lds \ge n\f$).
 * @param[out] dst The destination array with \f$m \times n\f$ entries.
 * @param[in] ldd The leading dimension or stride of the destination matrix,
 *                i.e. the number of matrix entries between two rows
 *                (\f$ldd \ge n\f$).
 */
void dyn2b_cpy_mat(
        int m,
        int n,
        const double *restrict src,
        int lds,
        double *restrict dst,
        int ldd);


/**
 * Scalar-multiply a matrix then add a second matrix ("multiply-add").
 *
 * `out[i, j] = alpha * in1[i, j] + in2[i, j]`
 *
 * @param[in] m Number of rows to copy.
 * @param[in] n Number of columns to copy.
 * @param[in] alpha The scalar to be applied to the first matrix.
 * @param[in] in1 The first source matrix with \f$m \times n\f$ entries.
 * @param[in] ld1 The leading dimension or stride of the first source matrix,
 *                i.e. the number of matrix entries between two rows
 *                (\f$ld1 \ge n\f$).
 * @param[in] in2 The second source matrix with \f$m \times n\f$ entries.
 * @param[in] ld2 The leading dimension or stride of the second source matrix,
 *                i.e. the number of matrix entries between two rows
 *                (\f$ld2 \ge n\f$).
 * @param[out] out The destination array with \f$m \times n\f$ entries.
 * @param[in] ldo The leading dimension or stride of the destination matrix,
 *                i.e. the number of matrix entries between two rows
 *                (\f$ldo \ge n\f$).
 */
void dyn2b_mad_mat(
        int m,
        int n,
        double alpha,
        const double *restrict in1,
        int ld1,
        const double *restrict in2,
        int ld2,
        double *restrict out,
        int ldo);

#ifdef __cplusplus
}
#endif

#endif
