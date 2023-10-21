// SPDX-License-Identifier: LGPL-3.0
#include <dyn2b/functions/array.h>


void dyn2b_cpy_arr(
        int n,
        const double *restrict in,
        double *restrict out)
{
    for (int i = 0; i < n; i++) {
        out[i] = in[i];
    }
}


void dyn2b_add_arr(
        int n,
        int offset,
        const double *restrict in1,
        const double *restrict in2,
        double *restrict out)
{
    for (int i = 0; i < n; i++) {
        out[offset + i] = in1[i] + in2[i];
    }
}


void dyn2b_add_arr_i(
        int n,
        int offset,
        const double *restrict in,
        double *restrict out)
{
    for (int i = 0; i < n; i++) {
        out[offset + i] += in[i];
    }
}


void dyn2b_sub_arr(
        int n,
        const double *restrict in1,
        const double *restrict in2,
        double *restrict out)
{
    for (int i = 0; i < n; i++) {
        out[i] = in1[i] - in2[i];
    }
}


void dyn2b_scl_arr(
        int n,
        const double *restrict a,
        const double *restrict in,
        double *restrict out)
{
    for (int i = 0; i < n; i++) {
        out[i] = *a * in[i];
    }
}


void dyn2b_inv_arr(
        int n,
        const double *restrict in,
        double *restrict out)
{
    for (int i = 0; i < n; i++) {
        out[i] = -in[i];
    }
}
