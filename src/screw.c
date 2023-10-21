// SPDX-License-Identifier: LGPL-3.0
#include <dyn2b/functions/screw.h>
#include <dyn2b/functions/vector3.h>
#include <dyn2b/functions/array.h>
#include <dyn2b/types/screw.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <cblas.h>


void dyn2b_cmp_pose3(
        const double *restrict x_prox,
        const double *restrict x_dist,
        double *restrict x_comp)
{
    assert(x_prox);
    assert(x_dist);
    assert(x_comp);

    // R_p R_d
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3,
            1.0, &x_prox[DYN2B_POSE3_ANG_OFFSET], DYN2B_POSE3_ANG_LD,
            &x_dist[DYN2B_POSE3_ANG_OFFSET], DYN2B_POSE3_ANG_LD,
            0.0, &x_comp[DYN2B_POSE3_ANG_OFFSET], DYN2B_POSE3_ANG_LD);

    // r_p + R_p r_d
    memcpy(&x_comp[DYN2B_POSE3_LIN_OFFSET], &x_prox[DYN2B_POSE3_LIN_OFFSET],
            DYN2B_POSE3_LIN_SIZE * sizeof(double));
    cblas_dgemv(CblasColMajor, CblasNoTrans, 3, 3,
            1.0, &x_prox[DYN2B_POSE3_ANG_OFFSET], DYN2B_POSE3_ANG_LD,
            &x_dist[DYN2B_POSE3_LIN_OFFSET], 1,
            1.0, &x_comp[DYN2B_POSE3_LIN_OFFSET], 1);
}


void dyn2b_dot_screw3(
        int m,
        int n,
        const double *s_dual,
        const double *s,
        double *out)
{
    assert(m >= 1);
    assert(n >= 1);
    assert(s_dual);
    assert(s);
    assert(out);

    // out[i, j] = mom_dual[i]^T dir[j]
    cblas_dgemm(CblasColMajor, CblasTrans, CblasNoTrans,
            m, n, DYN2B_SCREW3_DIR_SIZE,
            1.0, &s_dual[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE,
            &s[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE,
            0.0, out, m);

    // out[i, j] += dir_dual[i]^T mom[j]
    cblas_dgemm(CblasColMajor, CblasTrans, CblasNoTrans,
            m, n, DYN2B_SCREW3_MOM_SIZE,
            1.0, &s_dual[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE,
            &s[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE,
            1.0, out, m);
}


void dyn2b_crs_screw3(
        const double *restrict s1,
        const double *restrict s2,
        double *restrict out)
{
    assert(s1);
    assert(s2);
    assert(out);

    // dir_out = dir_1 x dir_2
    dyn2b_crs_vec3(1,
            &s1[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE,
            &s2[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE,
            &out[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE);

    // mom_out = dir_1 x mom_2 + mom_1 x dir_2
    double tmp[DYN2B_SCREW3_SIZE];
    dyn2b_crs_vec3(1,
            &s1[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE,
            &s2[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE,
            &tmp[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE);
    dyn2b_cad_vec3(1,
            &tmp[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE,
            &s1[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE,
            &s2[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE,
            &out[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE);
}


void dyn2b_cad_screw3(
        const double *restrict s1,
        const double *restrict s2,
        const double *restrict s3,
        double *restrict out)
{
    assert(s1);
    assert(s2);
    assert(s3);
    assert(out);

    // dir_out = dir_1 + dir_2 x dir_3
    dyn2b_cad_vec3(1,
            &s1[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE,
            &s2[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE,
            &s3[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE,
            &out[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE);

    // mom_out = mom_1 + dir_2 x mom_3 + mom_2 x dir_3
    double tmp[DYN2B_SCREW3_SIZE];
    dyn2b_cad_vec3(1,
            &s1[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE,
            &s2[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE,
            &s3[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE,
            &tmp[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE);
    dyn2b_cad_vec3(1,
            &tmp[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE,
            &s2[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE,
            &s3[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE,
            &out[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE);
}


void dyn2b_tf_dist_screw3(
        int n,
        const double *restrict x,
        const double *restrict s_prox,
        double *restrict s_dist)
{
    assert(n >= 1);
    assert(x);
    assert(s_prox);
    assert(s_dist);

    // dir_dist[i] = R^T * dir_prox[i]
    cblas_dgemm(CblasColMajor, CblasTrans, CblasNoTrans, 3, n, 3,
            1.0, &x[DYN2B_POSE3_ANG_OFFSET], DYN2B_POSE3_ANG_LD,
            &s_prox[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE,
            0.0, &s_dist[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE);

    // mom_dist[i] = R^T * (mom_prox[i] - r x dir_prox[i])
    //             = R^T * (mom_prox[i] + dir_prox[i] x r)
    double tmp[3 * n];
    dyn2b_cad_vec3(n,
            &s_prox[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE,
            &s_prox[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE,
            &x[DYN2B_POSE3_LIN_OFFSET], 0,
            tmp, 3);
    cblas_dgemm(CblasColMajor, CblasTrans, CblasNoTrans, 3, n, 3,
            1.0, &x[DYN2B_POSE3_ANG_OFFSET], DYN2B_POSE3_ANG_LD,
            tmp, 3,
            0.0, &s_dist[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE);
}


void dyn2b_tf_prox_screw3(
        int n,
        const double *restrict x,
        const double *restrict s_dist,
        double *restrict s_prox)
{
    assert(n >= 1);
    assert(x);
    assert(s_dist);
    assert(s_prox);

    // dir_prox[i] = R * dir_dist[i]
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, 3, n, 3,
            1.0, &x[DYN2B_POSE3_ANG_OFFSET], DYN2B_POSE3_ANG_LD,
            &s_dist[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE,
            0.0, &s_prox[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE);

    // mom_prox[i] = R * mom_dist[i] + r x R * dir_dist[i]
    //             = R * mom_dist[i] + r x dir_prox[i]
    dyn2b_crs_vec3(n,
            &x[DYN2B_POSE3_LIN_OFFSET], 0,
            &s_prox[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE,
            &s_prox[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE);
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, 3, n, 3,
            1.0, &x[DYN2B_POSE3_ANG_OFFSET], DYN2B_POSE3_ANG_LD,
            &s_dist[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE,
            1.0, &s_prox[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE);
}
