// SPDX-License-Identifier: LGPL-3.0
#ifndef DYN2B_FUNCTIONS_VECTOR3_H
#define DYN2B_FUNCTIONS_VECTOR3_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file vector3.h
 *
 * This file contains operations on 3D vectors.
 */


/**
 * Compute the cross product between two arrays of 3D vectors.
 *
 * `out[i] = in1[i] x in2[i]`
 *
 * @param[in] n Number of 3D vectors in the arrays.
 * @param[in] in1 The first source array with \f$n\f$ 3D vectors.
 * @param[in] ld1 The leading dimension or stride of the first source array,
 *                i.e. the number of elements between two 3D vectors
 *                (\f$ld1 \ge n\f$).
 * @param[in] in2 The second source array with \f$n\f$ 3D vectors.
 * @param[in] ld2 The leading dimension or stride of the second source array,
 *                i.e. the number of elements between two 3D vectors
 *                (\f$ld2 \ge n\f$).
 * @param[out] out The destination array with \f$n\f$ 3D vectors.
 * @param[in] ldo The leading dimension or stride of the destination array,
 *                i.e. the number of elements between two 3D vectors
 *                (\f$ldo \ge n\f$).
 */
void dyn2b_crs_vec3(
        int n,
        const double *restrict in1,
        int ld1,
        const double *restrict in2,
        int ld2,
        double *restrict out,
        int ldo);


/**
 * Compute the cross product between two arrays of 3D vectors and add a third
 * array of 3D vectors ("cross-add").
 *
 * `out[i] = in1[i] + in2[i] x in3[i]`
 *
 * @param[in] n Number of 3D vectors in the arrays.
 * @param[in] in1 The first source array with \f$n\f$ 3D vectors.
 * @param[in] ld1 The leading dimension or stride of the first source array,
 *                i.e. the number of elements between two 3D vectors
 *                (\f$ld1 \ge n\f$).
 * @param[in] in2 The second source array with \f$n\f$ 3D vectors.
 * @param[in] ld2 The leading dimension or stride of the second source array,
 *                i.e. the number of elements between two 3D vectors
 *                (\f$ld2 \ge n\f$).
 * @param[in] in3 The third source array with \f$n\f$ 3D vectors.
 * @param[in] ld3 The leading dimension or stride of the third source array,
 *                i.e. the number of elements between two 3D vectors
 *                (\f$ld3 \ge n\f$).
 * @param[out] out The destination array with \f$n\f$ 3D vectors.
 * @param[in] ldo The leading dimension or stride of the destination array,
 *                i.e. the number of elements between two 3D vectors
 *                (\f$ldo \ge n\f$).
 */
void dyn2b_cad_vec3(
        int n,
        const double *restrict in1, // [3xn]
        int ld1,
        const double *restrict in2, // [3xn]
        int ld2,
        const double *restrict in3, // [3xn]
        int ld3,
        double *restrict out,       // [3xn]
        int ldo);


/**
 * Map a 3D vector to a skew-symmetric matrix.
 *
 * `out = [in]_x`
 *
 * @param[in] in The 3D input vector.
 * @param[out] out The \f$3 \times 3\f$ output matrix.
 */
void dyn2b_skw_vec3(
        const double *restrict in,
        double *restrict out);

#ifdef __cplusplus
}
#endif

#endif
