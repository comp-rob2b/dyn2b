// SPDX-License-Identifier: LGPL-3.0
#include <dyn2b/functions/matrix.h>


void dyn2b_cpy_mat(
        int m,
        int n,
        const double *restrict src,
        int lds,
        double *restrict dst,
        int ldd)
{
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            dst[(i * ldd) + j] = src[(i * lds) + j];
        }
    }
}


void dyn2b_mad_mat(
        int m,
        int n,
        double alpha,
        const double *restrict in1,
        int ld1,
        const double *restrict in2,
        int ld2,
        double *restrict out,
        int ldo)
{
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            const int I1 = (i * ld1) + j;
            const int I2 = (i * ld2) + j;
            const int O1 = (i * ldo) + j;

            out[O1] = alpha * in1[I1] + in2[I2];
        }
    }
}
