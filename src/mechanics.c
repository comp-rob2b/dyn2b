// SPDX-License-Identifier: LGPL-3.0
#include <dyn2b/functions/mechanics.h>
#include <dyn2b/functions/screw.h>
#include <dyn2b/functions/vector3.h>
#include <dyn2b/types/mechanics.h>
#include <dyn2b/types/screw.h>
#include <cblas.h>
#include <string.h>
#include <assert.h>


void dyn2b_tf_dist_acc3(
        const double *restrict x,
        const double *restrict xd_abs,
        const double *restrict xd_rel,
        const double *restrict xdd_prox,
        double *restrict xdd_dist)
{
    assert(x);
    assert(xd_abs);
    assert(xd_rel);
    assert(xdd_prox);
    assert(xdd_dist);

    // X_{i,i+1} xdd_{0,i} + xd_{0,i+1} x xd_{i,i+1}
    double tmp[DYN2B_SCREW3_SIZE];
    dyn2b_tf_dist_screw3(1, x, xdd_prox, tmp);
    dyn2b_cad_screw3(tmp, xd_abs, xd_rel, xdd_dist);
}


void dyn2b_rbi_to_wrench3(
        const double *restrict rbi,
        const double *restrict xdd,
        double *restrict w)
{
    assert(rbi);
    assert(xdd);
    assert(w);

    // n = I w + h x v
    dyn2b_crs_vec3(1,
            &rbi[DYN2B_RBI3_H_OFFSET], 1,
            &xdd[DYN2B_TWIST3_LIN_OFFSET], 1,
            &w[DYN2B_WRENCH3_ANG_OFFSET], 1);
    cblas_dgemv(CblasColMajor, CblasNoTrans, 3, 3,
            1.0, &rbi[DYN2B_RBI3_I_OFFSET], DYN2B_RBI3_I_LD,
            &xdd[DYN2B_TWIST3_ANG_OFFSET], 1,
            1.0, &w[DYN2B_WRENCH3_ANG_OFFSET], 1);

    // f = m v - h x w
    //   = m v + w x h
    dyn2b_crs_vec3(1,
            &xdd[DYN2B_TWIST3_ANG_OFFSET], 1,
            &rbi[DYN2B_RBI3_H_OFFSET], 1,
            &w[DYN2B_WRENCH3_LIN_OFFSET], 1);
    cblas_daxpy(3,
            rbi[DYN2B_RBI3_M_OFFSET], &xdd[DYN2B_TWIST3_LIN_OFFSET], 1,
            &w[DYN2B_WRENCH3_LIN_OFFSET], 1);
}


void dyn2b_nrt_wrench3(
        const double *restrict rbi,
        const double *restrict xd,
        double *restrict w)
{
    assert(rbi);
    assert(xd);
    assert(w);

    double p[DYN2B_SCREW3_SIZE];
    dyn2b_rbi_to_wrench3(rbi, xd, p);
    dyn2b_crs_screw3(xd, p, w);
}
