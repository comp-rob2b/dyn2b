// SPDX-License-Identifier: LGPL-3.0
#include <dyn2b/functions/vector3.h>
#include <dyn2b/functions/array.h>


void dyn2b_crs_vec3(
        int n,
        const double *restrict in1,
        int ld1,
        const double *restrict in2,
        int ld2,
        double *restrict out,
        int ldo)
{
    for (int i = 0; i < n; i++) {
        const int I1 = i * ld1;
        const int I2 = i * ld2;
        const int O1 = i * ldo;

        out[O1 + 0] = in1[I1 + 1] * in2[I2 + 2] - in1[I1 + 2] * in2[I2 + 1];
        out[O1 + 1] = in1[I1 + 2] * in2[I2 + 0] - in1[I1 + 0] * in2[I2 + 2];
        out[O1 + 2] = in1[I1 + 0] * in2[I2 + 1] - in1[I1 + 1] * in2[I2 + 0];
    }
}


void dyn2b_cad_vec3(
        int n,
        const double *restrict in1,
        int ld1,
        const double *restrict in2,
        int ld2,
        const double *restrict in3,
        int ld3,
        double *restrict out,
        int ldo)
{
    for (int i = 0; i < n; i++) {
        const int I1 = i * ld1;
        const int I2 = i * ld2;
        const int I3 = i * ld3;
        const int O1 = i * ldo;

        out[O1 + 0] = in1[I1 + 0] + in2[I2 + 1] * in3[I3 + 2] - in2[I2 + 2] * in3[I3 + 1];
        out[O1 + 1] = in1[I1 + 1] + in2[I2 + 2] * in3[I3 + 0] - in2[I2 + 0] * in3[I3 + 2];
        out[O1 + 2] = in1[I1 + 2] + in2[I2 + 0] * in3[I3 + 1] - in2[I2 + 1] * in3[I3 + 0];
    }
}


void dyn2b_skw_vec3(
        const double *restrict in,
        double *restrict out)
{
    // Column-major layout
    out[0] =  0.0;   out[1] =  in[2]; out[2] = -in[1];
    out[3] = -in[2]; out[4] =  0.0;   out[5] =  in[0];
    out[6] =  in[1]; out[7] = -in[0]; out[8] =  0.0;
}
