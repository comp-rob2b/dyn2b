// SPDX-License-Identifier: LGPL-3.0
#include <dyn2b/functions/joint.h>
#include <dyn2b/functions/vector3.h>
#include <dyn2b/functions/mechanics.h>
#include <dyn2b/types/vector3.h>
#include <dyn2b/types/screw.h>
#include <dyn2b/types/mechanics.h>
#include <dyn2b/types/joint.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <cblas.h>
#include <lapacke.h>


//
// Operations on joints
//


void dyn2b_rev_x_to_pose3(
        const double *restrict jnt,
        double *restrict cart)
{
    assert(jnt);
    assert(cart);

    double cq = cos(*jnt);
    double sq = sin(*jnt);

    // Column-major layout
    cart[0] = 1.0; cart[ 1] = 0.0; cart[ 2] = 0.0;
    cart[3] = 0.0; cart[ 4] =  cq; cart[ 5] =  sq;
    cart[6] = 0.0; cart[ 7] = -sq; cart[ 8] =  cq;
    cart[9] = 0.0; cart[10] = 0.0; cart[11] = 0.0;
}


void dyn2b_rev_y_to_pose3(
        const double *restrict jnt,
        double *restrict cart)
{
    assert(jnt);
    assert(cart);

    double cq = cos(*jnt);
    double sq = sin(*jnt);

    // Column-major layout
    cart[0] =  cq; cart[ 1] = 0.0; cart[ 2] = -sq;
    cart[3] = 0.0; cart[ 4] = 1.0; cart[ 5] = 0.0;
    cart[6] =  sq; cart[ 7] = 0.0; cart[ 8] =  cq;
    cart[9] = 0.0; cart[10] = 0.0; cart[11] = 0.0;
}


void dyn2b_rev_z_to_pose3(
        const double *restrict jnt,
        double *restrict cart)
{
    assert(jnt);
    assert(cart);

    double cq = cos(*jnt);
    double sq = sin(*jnt);

    // Column-major layout
    cart[0] =  cq; cart[ 1] =  sq; cart[ 2] = 0.0;
    cart[3] = -sq; cart[ 4] =  cq; cart[ 5] = 0.0;
    cart[6] = 0.0; cart[ 7] = 0.0; cart[ 8] = 1.0;
    cart[9] = 0.0; cart[10] = 0.0; cart[11] = 0.0;
}


void dyn2b_trans_x_to_pose3(
        const double *restrict jnt,
        double *restrict cart)
{
    assert(jnt);
    assert(cart);

    // Column-major layout
    cart[0] =  1.0; cart[ 1] = 0.0; cart[ 2] = 0.0;
    cart[3] =  0.0; cart[ 4] = 1.0; cart[ 5] = 0.0;
    cart[6] =  0.0; cart[ 7] = 0.0; cart[ 8] = 1.0;
    cart[9] = *jnt; cart[10] = 0.0; cart[11] = 0.0;
}


void dyn2b_trans_y_to_pose3(
        const double *restrict jnt,
        double *restrict cart)
{
    assert(jnt);
    assert(cart);

    // Column-major layout
    cart[0] = 1.0; cart[ 1] =  0.0; cart[ 2] = 0.0;
    cart[3] = 0.0; cart[ 4] =  1.0; cart[ 5] = 0.0;
    cart[6] = 0.0; cart[ 7] =  0.0; cart[ 8] = 1.0;
    cart[9] = 0.0; cart[10] = *jnt; cart[11] = 0.0;
}


void dyn2b_trans_z_to_pose3(
        const double *restrict jnt,
        double *restrict cart)
{
    assert(jnt);
    assert(cart);

    // Column-major layout
    cart[0] = 1.0; cart[ 1] = 0.0; cart[ 2] =  0.0;
    cart[3] = 0.0; cart[ 4] = 1.0; cart[ 5] =  0.0;
    cart[6] = 0.0; cart[ 7] = 0.0; cart[ 8] =  1.0;
    cart[9] = 0.0; cart[10] = 0.0; cart[11] = *jnt;
}


void dyn2b_rev_x_to_twist3(
        const double *restrict jnt,
        double *restrict cart)
{
    assert(jnt);
    assert(cart);

    // Angular-before-linear order
    cart[0] = *jnt; cart[1] =  0.0; cart[2] = 0.0;
    cart[3] =  0.0; cart[4] =  0.0; cart[5] = 0.0;
}


void dyn2b_rev_y_to_twist3(
        const double *restrict jnt,
        double *restrict cart)
{
    assert(jnt);
    assert(cart);

    // Angular-before-linear order
    cart[0] =  0.0; cart[1] = *jnt; cart[2] = 0.0;
    cart[3] =  0.0; cart[4] =  0.0; cart[5] = 0.0;
}


void dyn2b_rev_z_to_twist3(
        const double *restrict jnt,
        double *restrict cart)
{
    assert(jnt);
    assert(cart);

    // Angular-before-linear order
    cart[0] =  0.0; cart[1] =  0.0; cart[2] = *jnt;
    cart[3] =  0.0; cart[4] =  0.0; cart[5] =  0.0;
}


void dyn2b_trans_x_to_twist3(
        const double *restrict jnt,
        double *restrict cart)
{
    assert(jnt);
    assert(cart);

    // Angular-before-linear order
    cart[0] =  0.0; cart[1] = 0.0; cart[2] = 0.0;
    cart[3] = *jnt; cart[4] = 0.0; cart[5] = 0.0;
}


void dyn2b_trans_y_to_twist3(
        const double *restrict jnt,
        double *restrict cart)
{
    assert(jnt);
    assert(cart);

    // Angular-before-linear order
    cart[0] = 0.0; cart[1] =  0.0; cart[2] = 0.0;
    cart[3] = 0.0; cart[4] = *jnt; cart[5] = 0.0;
}


void dyn2b_trans_z_to_twist3(
        const double *restrict jnt,
        double *restrict cart)
{
    assert(jnt);
    assert(cart);

    // Angular-before-linear order
    cart[0] = 0.0; cart[1] = 0.0; cart[2] =  0.0;
    cart[3] = 0.0; cart[4] = 0.0; cart[5] = *jnt;
}


void dyn2b_rev_x_from_wrench3(
        int n,
        const double *restrict cart,
        double *restrict jnt)
{
    assert(n >= 0);
    assert(jnt);
    assert(cart);

    for (int i = 0; i < n; i++) {
        // Linear-before-angular order
        int idx = (i * DYN2B_SCREW3_SIZE)
                  + DYN2B_WRENCH3_ANG_OFFSET + DYN2B_X_OFFSET;
        jnt[i] = cart[idx];
    }
}


void dyn2b_rev_y_from_wrench3(
        int n,
        const double *restrict cart,
        double *restrict jnt)
{
    assert(n >= 0);
    assert(jnt);
    assert(cart);

    for (int i = 0; i < n; i++) {
        // Linear-before-angular order
        int idx = (i * DYN2B_SCREW3_SIZE)
                  + DYN2B_WRENCH3_ANG_OFFSET + DYN2B_Y_OFFSET;
        jnt[i] = cart[idx];
    }
}


void dyn2b_rev_z_from_wrench3(
        int n,
        const double *restrict cart,
        double *restrict jnt)
{
    assert(n >= 0);
    assert(jnt);
    assert(cart);

    for (int i = 0; i < n; i++) {
        // Linear-before-angular order
        int idx = (i * DYN2B_SCREW3_SIZE)
                  + DYN2B_WRENCH3_ANG_OFFSET + DYN2B_Z_OFFSET;
        jnt[i] = cart[idx];
    }
}


void dyn2b_trans_x_from_wrench3(
        int n,
        const double *restrict cart,
        double *restrict jnt)
{
    assert(n >= 0);
    assert(jnt);
    assert(cart);

    for (int i = 0; i < n; i++) {
        // Linear-before-angular order
        int idx = (i * DYN2B_SCREW3_SIZE)
                  + DYN2B_WRENCH3_LIN_OFFSET + DYN2B_X_OFFSET;
        jnt[i] = cart[idx];
    }
}


void dyn2b_trans_y_from_wrench3(
        int n,
        const double *restrict cart,
        double *restrict jnt)
{
    assert(n >= 0);
    assert(jnt);
    assert(cart);

    for (int i = 0; i < n; i++) {
        // Linear-before-angular order
        int idx = (i * DYN2B_SCREW3_SIZE)
                  + DYN2B_WRENCH3_LIN_OFFSET + DYN2B_Y_OFFSET;
        jnt[i] = cart[idx];
    }
}


void dyn2b_trans_z_from_wrench3(
        int n,
        const double *restrict cart,
        double *restrict jnt)
{
    assert(n >= 0);
    assert(jnt);
    assert(cart);

    for (int i = 0; i < n; i++) {
        // Linear-before-angular order
        int idx = (i * DYN2B_SCREW3_SIZE)
                  + DYN2B_WRENCH3_LIN_OFFSET + DYN2B_Z_OFFSET;
        jnt[i] = cart[idx];
    }
}


void dyn2b_to_abi3(
        const double *restrict in,
        double *restrict out)
{
    assert(in);
    assert(out);

    memcpy(&out[DYN2B_ABI3_I_OFFSET], &in[DYN2B_RBI3_I_OFFSET],
            DYN2B_ABI3_I_SIZE * sizeof(double));

    dyn2b_skw_vec3(&in[DYN2B_RBI3_H_OFFSET], &out[DYN2B_ABI3_H_OFFSET]);

    const int MA = DYN2B_ABI3_M_OFFSET;
    const int MR = DYN2B_RBI3_M_OFFSET;
    out[MA + 0] = in[MR]; out[MA + 1] =   0.0 ; out[MA + 2] =   0.0 ;
    out[MA + 3] =   0.0 ; out[MA + 4] = in[MR]; out[MA + 5] =   0.0 ;
    out[MA + 6] =   0.0 ; out[MA + 7] =   0.0 ; out[MA + 8] = in[MR];
}


void dyn2b_tf_prox_abi3(
        const double *restrict tf,
        const double *restrict in,
        double *restrict out)
{
    assert(tf);
    assert(in);
    assert(out);

    // This is the same as [Featherstone2008]'s formula X^T I^A X ...
    // ... just with R = E^T

    // M' = R M R^T
    //
    double mrt[DYN2B_ABI3_M_SIZE];

    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasTrans, 3, 3, 3,
            1.0, &in[DYN2B_ABI3_M_OFFSET], DYN2B_ABI3_M_LD,
            &tf[DYN2B_POSE3_ANG_OFFSET], DYN2B_POSE3_ANG_LD,
            0.0, mrt, DYN2B_ABI3_M_LD);
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3,
            1.0, &tf[DYN2B_POSE3_ANG_OFFSET], DYN2B_POSE3_ANG_LD,
            mrt, DYN2B_ABI3_M_LD,
            0.0, &out[DYN2B_ABI3_M_OFFSET], DYN2B_ABI3_M_LD);


    // H'' = H' + rxM'
    // H'  = R H R^T
    //
    double hrt[DYN2B_ABI3_H_SIZE];
    double rhrt[DYN2B_ABI3_H_SIZE];

    // R H R^T
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasTrans, 3, 3, 3,
            1.0, &in[DYN2B_ABI3_H_OFFSET], DYN2B_ABI3_H_LD,
            &tf[DYN2B_POSE3_ANG_OFFSET], DYN2B_POSE3_ANG_LD,
            0.0, hrt, DYN2B_ABI3_H_LD);
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3,
            1.0, &tf[DYN2B_POSE3_ANG_OFFSET], DYN2B_POSE3_ANG_LD,
            hrt, DYN2B_ABI3_H_LD,
            0.0, rhrt, DYN2B_ABI3_H_LD);

    // + rxM'
    dyn2b_cad_vec3(3,
            rhrt, DYN2B_ABI3_H_LD,
            &tf[DYN2B_POSE3_LIN_OFFSET], 0,
            &out[DYN2B_ABI3_M_OFFSET], DYN2B_ABI3_M_LD,
            &out[DYN2B_ABI3_H_OFFSET], DYN2B_ABI3_H_LD);


    // I' = R I R^T + rx(H'^T)      - (H' + rxM')rx
    //    = R I R^T + rx(R H R^T)^T - H''rx
    //
    double irt[DYN2B_ABI3_I_SIZE];
    double rx[9];

    // R I R^T ...
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasTrans, 3, 3, 3,
            1.0, &in[DYN2B_ABI3_I_OFFSET], DYN2B_ABI3_I_LD,
            &tf[DYN2B_POSE3_ANG_OFFSET], DYN2B_POSE3_ANG_LD,
            0.0, irt, DYN2B_ABI3_I_LD);
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3,
            1.0, &tf[DYN2B_POSE3_ANG_OFFSET], DYN2B_POSE3_ANG_LD,
            irt, DYN2B_ABI3_I_LD,
            0.0, &out[DYN2B_ABI3_I_OFFSET], DYN2B_ABI3_I_LD);

    // ... + rx(R H R^T)^T ...
    dyn2b_skw_vec3(&tf[DYN2B_POSE3_LIN_OFFSET], rx);
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasTrans, 3, 3, 3,
            1.0, rx, 3,
            rhrt, DYN2B_ABI3_H_LD,
            1.0, &out[DYN2B_ABI3_I_OFFSET], DYN2B_ABI3_I_LD);

    // ... - H''rx
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3,
            -1.0, &out[DYN2B_ABI3_H_OFFSET], DYN2B_ABI3_H_LD,
            rx, 3,
            1.0, &out[DYN2B_ABI3_I_OFFSET], DYN2B_ABI3_I_LD);
}


void dyn2b_abi_to_wrench3(
        int n,
        const double *restrict abi,
        const double *restrict xdd,
        double *restrict w)
{
    assert(n >= 0);
    assert(abi);
    assert(xdd);
    assert(w);

    // n = I w + H v
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, 3, n, 3,
            1.0, &abi[DYN2B_ABI3_I_OFFSET], DYN2B_ABI3_I_LD,
            &xdd[DYN2B_TWIST3_ANG_OFFSET], DYN2B_TWIST3_SIZE,
            0.0, &w[DYN2B_WRENCH3_ANG_OFFSET], DYN2B_WRENCH3_SIZE);
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, 3, n, 3,
            1.0, &abi[DYN2B_ABI3_H_OFFSET], DYN2B_ABI3_H_LD,
            &xdd[DYN2B_TWIST3_LIN_OFFSET], DYN2B_TWIST3_SIZE,
            1.0, &w[DYN2B_WRENCH3_ANG_OFFSET], DYN2B_WRENCH3_SIZE);

    // f = M v + H^T w
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, 3, n, 3,
            1.0, &abi[DYN2B_ABI3_M_OFFSET], DYN2B_ABI3_M_LD,
            &xdd[DYN2B_TWIST3_LIN_OFFSET], DYN2B_TWIST3_SIZE,
            0.0, &w[DYN2B_WRENCH3_LIN_OFFSET], DYN2B_WRENCH3_SIZE);
    cblas_dgemm(CblasColMajor, CblasTrans, CblasNoTrans, 3, n, 3,
            1.0, &abi[DYN2B_ABI3_H_OFFSET], DYN2B_ABI3_H_LD,
            &xdd[DYN2B_TWIST3_ANG_OFFSET], DYN2B_TWIST3_SIZE,
            1.0, &w[DYN2B_WRENCH3_LIN_OFFSET], DYN2B_WRENCH3_SIZE);
}


void dyn2b_rev_x_proj_abi3(
        const double *restrict d,
        const double *restrict m_in,
        double *restrict m_out)
{
    assert(d);
    assert(m_in);
    assert(m_out);

    const int k = DYN2B_X_OFFSET;

    // S^T M S
    double i_kk = m_in[DYN2B_ABI3_I_OFFSET + (k * DYN2B_ABI3_I_LD) + k];
    // d + S^T M S
    double dstms = *d + i_kk;

    for (int c = 0; c < 3; c++) {
        for (int r = 0; r < 3; r++) {
            // I - "2nd moment of mass matrix"
            double i_rc = m_in[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * c) + r];
            double i_rk = m_in[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * k) + r];
            double i_kc = m_in[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * c) + k];
            m_out[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * c) + r]
                    = i_rc - (i_rk * i_kc) / dstms;

            // H - "1st moment of mass matrix"
            double h_rc = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + r];
            double h_kc = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + k];
            double h_rk = m_in[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * k) + r];
            m_out[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + r]
                    = h_rc - (h_kc * h_rk) / dstms;

            // M - "0th moment of mass matrix"
            double m_rc = m_in[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + r];
            double m_kc = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + k];
            double m_kr = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * r) + k];
            m_out[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + r]
                    = m_rc - (m_kc * m_kr) / dstms;
        }
    }
}


void dyn2b_rev_y_proj_abi3(
        const double *restrict d,
        const double *restrict m_in,
        double *restrict m_out)
{
    assert(d);
    assert(m_in);
    assert(m_out);

    const int k = DYN2B_Y_OFFSET;

    // S^T M S
    double i_kk = m_in[DYN2B_ABI3_I_OFFSET + (k * DYN2B_ABI3_I_LD) + k];
    // d + S^T M S
    double dstms = *d + i_kk;

    for (int c = 0; c < 3; c++) {
        for (int r = 0; r < 3; r++) {
            // I - "2nd moment of mass matrix"
            double i_rc = m_in[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * c) + r];
            double i_rk = m_in[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * k) + r];
            double i_kc = m_in[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * c) + k];
            m_out[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * c) + r]
                    = i_rc - (i_rk * i_kc) / dstms;

            // H - "1st moment of mass matrix"
            double h_rc = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + r];
            double h_kc = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + k];
            double h_rk = m_in[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * k) + r];
            m_out[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + r]
                    = h_rc - (h_kc * h_rk) / dstms;

            // M - "0th moment of mass matrix"
            double m_rc = m_in[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + r];
            double m_kc = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + k];
            double m_kr = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * r) + k];
            m_out[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + r]
                    = m_rc - (m_kc * m_kr) / dstms;
        }
    }
}


void dyn2b_rev_z_proj_abi3(
        const double *restrict d,
        const double *restrict m_in,
        double *restrict m_out)
{
    assert(d);
    assert(m_in);
    assert(m_out);

    const int k = DYN2B_Z_OFFSET;

    // S^T M S
    double i_kk = m_in[DYN2B_ABI3_I_OFFSET + (k * DYN2B_ABI3_I_LD) + k];
    // d + S^T M S
    double dstms = *d + i_kk;

    for (int c = 0; c < 3; c++) {
        for (int r = 0; r < 3; r++) {
            // I - "2nd moment of mass matrix"
            double i_rc = m_in[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * c) + r];
            double i_rk = m_in[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * k) + r];
            double i_kc = m_in[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * c) + k];
            m_out[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * c) + r]
                    = i_rc - (i_rk * i_kc) / dstms;

            // H - "1st moment of mass matrix"
            double h_rc = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + r];
            double h_kc = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + k];
            double h_rk = m_in[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * k) + r];
            m_out[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + r]
                    = h_rc - (h_kc * h_rk) / dstms;

            // M - "0th moment of mass matrix"
            double m_rc = m_in[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + r];
            double m_kc = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + k];
            double m_kr = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * r) + k];
            m_out[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + r]
                    = m_rc - (m_kc * m_kr) / dstms;
        }
    }
}


void dyn2b_trans_x_proj_abi3(
        const double *restrict d,
        const double *restrict m_in,
        double *restrict m_out)
{
    assert(d);
    assert(m_in);
    assert(m_out);

    const int k = DYN2B_X_OFFSET;

    // S^T M S
    double m_kk = m_in[DYN2B_ABI3_M_OFFSET + (k * DYN2B_ABI3_M_LD) + k];
    // d + S^T M S
    double dstms = *d + m_kk;

    for (int c = 0; c < 3; c++) {
        for (int r = 0; r < 3; r++) {
            // I - "2nd moment of mass matrix"
            double i_rc = m_in[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * c) + r];
            double i_rk = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * k) + r];
            double i_ck = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * k) + c];
            m_out[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * c) + r]
                    = i_rc - (i_rk * i_ck) / dstms;

            // H - "1st moment of mass matrix"
            double h_rc = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + r];
            double h_rk = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * k) + r];
            double h_kc = m_in[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + k];
            m_out[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + r]
                    = h_rc - (h_rk * h_kc) / dstms;

            // M - "0th moment of mass matrix"
            double m_rc = m_in[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + r];
            double m_rk = m_in[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * k) + r];
            double m_kc = m_in[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + k];
            m_out[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + r]
                    = m_rc - (m_rk * m_kc) / dstms;
        }
    }
}


void dyn2b_trans_y_proj_abi3(
        const double *restrict d,
        const double *restrict m_in,
        double *restrict m_out)
{
    assert(d);
    assert(m_in);
    assert(m_out);

    const int k = DYN2B_Y_OFFSET;

    // S^T M S
    double m_kk = m_in[DYN2B_ABI3_M_OFFSET + (k * DYN2B_ABI3_M_LD) + k];
    // d + S^T M S
    double dstms = *d + m_kk;

    for (int c = 0; c < 3; c++) {
        for (int r = 0; r < 3; r++) {
            // I - "2nd moment of mass matrix"
            double i_rc = m_in[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * c) + r];
            double i_rk = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * k) + r];
            double i_ck = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * k) + c];
            m_out[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * c) + r]
                    = i_rc - (i_rk * i_ck) / dstms;

            // H - "1st moment of mass matrix"
            double h_rc = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + r];
            double h_rk = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * k) + r];
            double h_kc = m_in[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + k];
            m_out[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + r]
                    = h_rc - (h_rk * h_kc) / dstms;

            // M - "0th moment of mass matrix"
            double m_rc = m_in[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + r];
            double m_rk = m_in[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * k) + r];
            double m_kc = m_in[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + k];
            m_out[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + r]
                    = m_rc - (m_rk * m_kc) / dstms;
        }
    }
}


void dyn2b_trans_z_proj_abi3(
        const double *restrict d,
        const double *restrict m_in,
        double *restrict m_out)
{
    assert(d);
    assert(m_in);
    assert(m_out);

    const int k = DYN2B_Z_OFFSET;

    // S^T M S
    double m_kk = m_in[DYN2B_ABI3_M_OFFSET + (k * DYN2B_ABI3_M_LD) + k];
    // d + S^T M S
    double dstms = *d + m_kk;

    for (int c = 0; c < 3; c++) {
        for (int r = 0; r < 3; r++) {
            // I - "2nd moment of mass matrix"
            double i_rc = m_in[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * c) + r];
            double i_rk = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * k) + r];
            double i_ck = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * k) + c];
            m_out[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * c) + r]
                    = i_rc - (i_rk * i_ck) / dstms;

            // H - "1st moment of mass matrix"
            double h_rc = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + r];
            double h_rk = m_in[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * k) + r];
            double h_kc = m_in[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + k];
            m_out[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * c) + r]
                    = h_rc - (h_rk * h_kc) / dstms;

            // M - "0th moment of mass matrix"
            double m_rc = m_in[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + r];
            double m_rk = m_in[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * k) + r];
            double m_kc = m_in[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + k];
            m_out[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * c) + r]
                    = m_rc - (m_rk * m_kc) / dstms;
        }
    }
}


void dyn2b_rev_x_proj_wrench3(
        int n,
        const double *restrict d,
        const double *restrict m,
        const double *restrict f_in,
        double *restrict f_out)
{
    assert(n >= 0);
    assert(d);
    assert(m);
    assert(f_in);
    assert(f_out);

    const int JNT_IDX = DYN2B_X_OFFSET;

    // | I  H|
    // |H^T M|
    //
    // [ i00,  - ,  - , h00, h01, h02],
    // [ i10,  - ,  - ,  - ,  - ,  - ],
    // [ i20,  - ,  - ,  - ,  - ,  - ],
    // [  - ,  - ,  - ,  - ,  - ,  - ],
    // [  - ,  - ,  - ,  - ,  - ,  - ],
    // [  - ,  - ,  - ,  - ,  - ,  - ]])

    double i_kk = m[DYN2B_ABI3_I_OFFSET
                    + (JNT_IDX * DYN2B_ABI3_I_LD) + JNT_IDX];
    double dstms = *d + i_kk;  // d + S^T M S

    for (int i = 0; i < DYN2B_SCREW3_SIZE / 2; i++) {
        double i_ik = m[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * JNT_IDX) + i];
        double h_ki = m[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * i) + JNT_IDX];

        for (int j = 0; j < n; j++) {
            int idx_k = (j * DYN2B_SCREW3_SIZE)
                        + (DYN2B_WRENCH3_ANG_OFFSET + JNT_IDX);
            double f_k = f_in[idx_k];

            int idx_ang_io = (j * DYN2B_SCREW3_SIZE)
                             + (DYN2B_WRENCH3_ANG_OFFSET + i);
            f_out[idx_ang_io] = f_in[idx_ang_io] - f_k * i_ik / dstms;

            int idx_lin_io = (j * DYN2B_SCREW3_SIZE)
                             + (DYN2B_WRENCH3_LIN_OFFSET + i);
            f_out[idx_lin_io] = f_in[idx_lin_io] - f_k * h_ki / dstms;
        }
    }
}


void dyn2b_rev_y_proj_wrench3(
        int n,
        const double *restrict d,
        const double *restrict m,
        const double *restrict f_in,
        double *restrict f_out)
{
    assert(n >= 0);
    assert(d);
    assert(m);
    assert(f_in);
    assert(f_out);

    const int JNT_IDX = DYN2B_Y_OFFSET;

    // | I  H|
    // |H^T M|
    //
    // [  - , i01,  - ,  - ,  - ,  - ],
    // [  - , i11,  - , h10, h11, h12],
    // [  - , i21,  - ,  - ,  - ,  - ],
    // [  - ,  - ,  - ,  - ,  - ,  - ],
    // [  - ,  - ,  - ,  - ,  - ,  - ],
    // [  - ,  - ,  - ,  - ,  - ,  - ]])

    double i_kk = m[DYN2B_ABI3_I_OFFSET
                    + (JNT_IDX * DYN2B_ABI3_I_LD) + JNT_IDX];
    double dstms = *d + i_kk;  // d + S^T M S

    for (int i = 0; i < DYN2B_SCREW3_SIZE / 2; i++) {
        double i_ik = m[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * JNT_IDX) + i];
        double h_ki = m[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * i) + JNT_IDX];

        for (int j = 0; j < n; j++) {
            int idx_k = (j * DYN2B_SCREW3_SIZE)
                        + (DYN2B_WRENCH3_ANG_OFFSET + JNT_IDX);
            double f_k = f_in[idx_k];

            int idx_ang_io = (j * DYN2B_SCREW3_SIZE)
                             + (DYN2B_WRENCH3_ANG_OFFSET + i);
            f_out[idx_ang_io] = f_in[idx_ang_io] - f_k * i_ik / dstms;

            int idx_lin_io = (j * DYN2B_SCREW3_SIZE)
                             + (DYN2B_WRENCH3_LIN_OFFSET + i);
            f_out[idx_lin_io] = f_in[idx_lin_io] - f_k * h_ki / dstms;
        }
    }
}


void dyn2b_rev_z_proj_wrench3(
        int n,
        const double *restrict d,
        const double *restrict m,
        const double *restrict f_in,
        double *restrict f_out)
{
    assert(n >= 0);
    assert(d);
    assert(m);
    assert(f_in);
    assert(f_out);

    const int JNT_IDX = DYN2B_Z_OFFSET;

    // | I  H|
    // |H^T M|
    //
    // [  - ,  - , i02,  - ,  - ,  - ],
    // [  - ,  - , i12,  - ,  - ,  - ],
    // [  - ,  - , i22, h20, h21, h22],
    // [  - ,  - ,  - ,  - ,  - ,  - ],
    // [  - ,  - ,  - ,  - ,  - ,  - ],
    // [  - ,  - ,  - ,  - ,  - ,  - ]])

    double i_kk = m[DYN2B_ABI3_I_OFFSET
                    + (JNT_IDX * DYN2B_ABI3_I_LD) + JNT_IDX];
    double dstms = *d + i_kk;  // d + S^T M S

    for (int i = 0; i < DYN2B_SCREW3_SIZE / 2; i++) {
        double i_ik = m[DYN2B_ABI3_I_OFFSET + (DYN2B_ABI3_I_LD * JNT_IDX) + i];
        double h_ki = m[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * i) + JNT_IDX];

        for (int j = 0; j < n; j++) {
            int idx_k = (j * DYN2B_SCREW3_SIZE)
                        + (DYN2B_WRENCH3_ANG_OFFSET + JNT_IDX);
            double f_k = f_in[idx_k];

            int idx_ang_io = (j * DYN2B_SCREW3_SIZE)
                             + (DYN2B_WRENCH3_ANG_OFFSET + i);
            f_out[idx_ang_io] = f_in[idx_ang_io] - f_k * i_ik / dstms;

            int idx_lin_io = (j * DYN2B_SCREW3_SIZE)
                             + (DYN2B_WRENCH3_LIN_OFFSET + i);
            f_out[idx_lin_io] = f_in[idx_lin_io] - f_k * h_ki / dstms;
        }
    }
}


void dyn2b_trans_x_proj_wrench3(
        int n,
        const double *restrict d,
        const double *restrict m,
        const double *restrict f_in,
        double *restrict f_out)
{
    assert(n >= 0);
    assert(d);
    assert(m);
    assert(f_in);
    assert(f_out);

    const int JNT_IDX = DYN2B_X_OFFSET;

    // | I  H|
    // |H^T M|
    //
    // [  - ,  - ,  - , h00,  - ,  - ],
    // [  - ,  - ,  - , h10,  - ,  - ],
    // [  - ,  - ,  - , h20,  - ,  - ],
    // [  - ,  - ,  - , m00,  - ,  - ],
    // [  - ,  - ,  - , m10,  - ,  - ],
    // [  - ,  - ,  - , m20,  - ,  - ]])

    double m_kk = m[DYN2B_ABI3_M_OFFSET
                    + (JNT_IDX * DYN2B_ABI3_M_LD) + JNT_IDX];
    double dstms = *d + m_kk;  // d + S^T M S

    for (int i = 0; i < DYN2B_SCREW3_SIZE / 2; i++) {
        double m_ik = m[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * JNT_IDX) + i];
        double h_ik = m[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * JNT_IDX) + i];

        for (int j = 0; j < n; j++) {
            int idx_k = (j * DYN2B_SCREW3_SIZE)
                        + (DYN2B_WRENCH3_LIN_OFFSET + JNT_IDX);
            double f_k = f_in[idx_k];

            int idx_lin_io = (j * DYN2B_SCREW3_SIZE)
                             + (DYN2B_WRENCH3_LIN_OFFSET + i);
            f_out[idx_lin_io] = f_in[idx_lin_io] - f_k * m_ik / dstms;

            int idx_ang_io = (j * DYN2B_SCREW3_SIZE)
                             + (DYN2B_WRENCH3_ANG_OFFSET + i);
            f_out[idx_ang_io] = f_in[idx_ang_io] - f_k * h_ik / dstms;
        }
    }
}


void dyn2b_trans_y_proj_wrench3(
        int n,
        const double *restrict d,
        const double *restrict m,
        const double *restrict f_in,
        double *restrict f_out)
{
    assert(n >= 0);
    assert(d);
    assert(m);
    assert(f_in);
    assert(f_out);

    const int JNT_IDX = DYN2B_Y_OFFSET;

    // | I  H|
    // |H^T M|
    //
    // [  - ,  - ,  - ,  - , h01,  - ],
    // [  - ,  - ,  - ,  - , h11,  - ],
    // [  - ,  - ,  - ,  - , h21,  - ],
    // [  - ,  - ,  - ,  - , m01,  - ],
    // [  - ,  - ,  - ,  - , m11,  - ],
    // [  - ,  - ,  - ,  - , m21,  - ]])

    double m_kk = m[DYN2B_ABI3_M_OFFSET
                    + (JNT_IDX * DYN2B_ABI3_M_LD) + JNT_IDX];
    double dstms = *d + m_kk;  // d + S^T M S

    for (int i = 0; i < DYN2B_SCREW3_SIZE / 2; i++) {
        double m_ik = m[DYN2B_ABI3_M_OFFSET + (DYN2B_ABI3_M_LD * JNT_IDX) + i];
        double h_ik = m[DYN2B_ABI3_H_OFFSET + (DYN2B_ABI3_H_LD * JNT_IDX) + i];

        for (int j = 0; j < n; j++) {
            int idx_k = (j * DYN2B_SCREW3_SIZE)
                        + (DYN2B_WRENCH3_LIN_OFFSET + JNT_IDX);
            double f_k = f_in[idx_k];

            int idx_lin_io = (j * DYN2B_SCREW3_SIZE)
                             + (DYN2B_WRENCH3_LIN_OFFSET + i);
            f_out[idx_lin_io] = f_in[idx_lin_io] - f_k * m_ik / dstms;

            int idx_ang_io = (j * DYN2B_SCREW3_SIZE)
                             + (DYN2B_WRENCH3_ANG_OFFSET + i);
            f_out[idx_ang_io] = f_in[idx_ang_io] - f_k * h_ik / dstms;
        }
    }
}


void dyn2b_trans_z_proj_wrench3(
        int n,
        const double *restrict d,
        const double *restrict m,
        const double *restrict f_in,
        double *restrict f_out)
{
    assert(n >= 0);
    assert(d);
    assert(m);
    assert(f_in);
    assert(f_out);

    const int JNT_IDX = DYN2B_Z_OFFSET;

    // | I  H|
    // |H^T M|
    //
    // [  - ,  - ,  - ,  - ,  - , h02],
    // [  - ,  - ,  - ,  - ,  - , h12],
    // [  - ,  - ,  - ,  - ,  - , h22],
    // [  - ,  - ,  - ,  - ,  - , m02],
    // [  - ,  - ,  - ,  - ,  - , m12],
    // [  - ,  - ,  - ,  - ,  - , m22]])

    double m_kk = m[DYN2B_ABI3_M_OFFSET
                    + (JNT_IDX * DYN2B_ABI3_M_LD) + JNT_IDX];
    double dstms = *d + m_kk;  // d + S^T M S

    for (int i = 0; i < DYN2B_SCREW3_SIZE / 2; i++) {
        double m_ik = m[DYN2B_ABI3_M_OFFSET
                        + (DYN2B_ABI3_M_LD * JNT_IDX) + i];
        double h_ik = m[DYN2B_ABI3_H_OFFSET
                        + (DYN2B_ABI3_H_LD * JNT_IDX) + i];

        for (int j = 0; j < n; j++) {
            int idx_k = (j * DYN2B_SCREW3_SIZE)
                        + (DYN2B_WRENCH3_LIN_OFFSET + JNT_IDX);
            double f_k = f_in[idx_k];

            int idx_lin_io = (j * DYN2B_SCREW3_SIZE)
                             + (DYN2B_WRENCH3_LIN_OFFSET + i);
            f_out[idx_lin_io] = f_in[idx_lin_io] - f_k * m_ik / dstms;

            int idx_ang_io = (j * DYN2B_SCREW3_SIZE)
                             + (DYN2B_WRENCH3_ANG_OFFSET + i);
            f_out[idx_ang_io] = f_in[idx_ang_io] - f_k * h_ik / dstms;
        }
    }
}


void dyn2b_to_mat_abi3(
        const double *restrict tup,
        double *restrict mat)
{
    assert(tup);
    assert(mat);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int ii  =  0 + (i * DYN2B_SCREW3_SIZE) + j;
            int iht =  3 + (i * DYN2B_SCREW3_SIZE) + j;
            int ih  = 18 + (i * DYN2B_SCREW3_SIZE) + j;
            int im  = 21 + (i * DYN2B_SCREW3_SIZE) + j;
            mat[ii ] = tup[DYN2B_ABI3_I_OFFSET + (i * DYN2B_ABI3_I_LD) + j];
            mat[iht] = tup[DYN2B_ABI3_H_OFFSET + (j * DYN2B_ABI3_H_LD) + i];
            mat[ih ] = tup[DYN2B_ABI3_H_OFFSET + (i * DYN2B_ABI3_H_LD) + j];
            mat[im ] = tup[DYN2B_ABI3_M_OFFSET + (i * DYN2B_ABI3_M_LD) + j];
        }
    }
}


void dyn2b_to_tup_abi3(
        const double *restrict mat,
        double *restrict tup)
{
    assert(mat);
    assert(tup);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int ii = DYN2B_ABI3_I_OFFSET + (i * DYN2B_ABI3_I_LD) + j;
            int ih = DYN2B_ABI3_H_OFFSET + (i * DYN2B_ABI3_H_LD) + j;
            int im = DYN2B_ABI3_M_OFFSET + (i * DYN2B_ABI3_M_LD) + j;
            tup[ii] = mat[ 0 + (i * DYN2B_SCREW3_SIZE) + j];
            tup[ih] = mat[18 + (i * DYN2B_SCREW3_SIZE) + j];
            tup[im] = mat[21 + (i * DYN2B_SCREW3_SIZE) + j];
        }
    }
}


void dyn2b_jnt_inv_abi3(
        int dof,
        const double *restrict jac,
        const double *restrict m,
        const double *restrict d,
        double *restrict dstms)
{
    assert(dof >= 1);
    assert(dof <= 6);
    assert(jac);
    assert(m);
    assert(d);
    assert(dstms);

    // D = d + S^T M^A S
    //
    double ms[DYN2B_SCREW3_SIZE * dof];

    // M^A S
    cblas_dsymm(CblasColMajor, CblasLeft, CblasUpper, DYN2B_SCREW3_SIZE, dof,
            1.0, m, DYN2B_SCREW3_SIZE,
            jac, DYN2B_SCREW3_SIZE,
            0.0, ms, DYN2B_SCREW3_SIZE);

    // d + S^T (M^A S)
    memcpy(dstms, d, dof * dof * sizeof(double));
    cblas_dgemm(CblasColMajor, CblasTrans, CblasNoTrans,
            dof, dof, DYN2B_SCREW3_SIZE,
            1.0, jac, DYN2B_SCREW3_SIZE,
            ms, DYN2B_SCREW3_SIZE,
            1.0, dstms, dof);

    // D^{-1}
    if (dof == 1) {
        dstms[0] = 1.0 / dstms[0];
    } else {
        int piv[dof];
        // LU decomposition
        //LAPACKE_dgetrf(LAPACK_COL_MAJOR, dof, dof, dstms, dof, piv);
        //LAPACKE_dgetri(LAPACK_COL_MAJOR, dof, dstms, dof, piv);

        // Cholesky decomposition: more efficient than LU decomposition but only
        // fills out upper triangular matrix ...
        LAPACKE_dsytrf(LAPACK_COL_MAJOR, 'U', dof, dstms, dof, piv);
        LAPACKE_dsytri(LAPACK_COL_MAJOR, 'U', dof, dstms, dof, piv);

        // ... copy upper triangular matrix to lower triangular matrix
        for (int i = 0; i < dof; i++) {
            for (int j = i; j < dof; j++) {
                dstms[i * dof + j] = dstms[i + j * dof];
            }
        }
    }
}


void dyn2b_jnt_to_proj3(
        int dof,
        const double *restrict jac,
        const double *restrict d,
        const double *restrict m,
        double *restrict proj)
{
    assert(dof >= 1);
    assert(dof <= 6);
    assert(jac);
    assert(d);
    assert(m);
    assert(proj);

    // Construct inertia matrix to simplify the code below (at the expense of
    // slightly more computations)
    double m_mat[DYN2B_SCREW3_SIZE * DYN2B_SCREW3_SIZE];
    dyn2b_to_mat_abi3(m, m_mat);

    // D^{-1} = (d + S^T M^A S)^{-1}
    double d_inv[dof * dof];
    dyn2b_jnt_inv_abi3(dof, jac, m_mat, d, d_inv);

    // S D^{-1} S^T
    double sdi[DYN2B_SCREW3_SIZE * dof];
    double sdist[DYN2B_SCREW3_SIZE * DYN2B_SCREW3_SIZE];

    cblas_dsymm(CblasColMajor, CblasRight, CblasUpper,
            DYN2B_SCREW3_SIZE, dof,
            1.0, d_inv, dof,
            jac, DYN2B_SCREW3_SIZE,
            0.0, sdi, DYN2B_SCREW3_SIZE);
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasTrans,
            DYN2B_SCREW3_SIZE, DYN2B_SCREW3_SIZE, dof,
            1.0, sdi, DYN2B_SCREW3_SIZE,
            jac, DYN2B_SCREW3_SIZE,
            0.0, sdist, DYN2B_SCREW3_SIZE);


    // I - M^A S D^{-1} S^T
    memset(proj, 0, DYN2B_SCREW3_SIZE * DYN2B_SCREW3_SIZE * sizeof(double));
    for (int i = 0; i < DYN2B_SCREW3_SIZE; i++) {
        proj[(DYN2B_SCREW3_SIZE * i) + i] = 1.0;
    }

   cblas_dsymm(CblasColMajor, CblasLeft, CblasUpper,
        DYN2B_SCREW3_SIZE, DYN2B_SCREW3_SIZE,
        -1.0, m_mat, DYN2B_SCREW3_SIZE,
        sdist, DYN2B_SCREW3_SIZE,
        1.0, proj, DYN2B_SCREW3_SIZE);
}


void dyn2b_jnt_proj_abi3(
        int dof,
        const double *restrict jac,
        const double *restrict d,
        const double *restrict m_in,
        double *restrict m_out)
{
    assert(dof >= 1);
    assert(jac);
    assert(d);
    assert(m_in);
    assert(m_out);

    double proj[DYN2B_SCREW3_SIZE * DYN2B_SCREW3_SIZE];
    dyn2b_jnt_to_proj3(dof, jac, d, m_in, proj);

    // TODO: Remove duplicated computations. The previous function already
    //       constructs a dense representation of the ABI internally.
    double m_mat[DYN2B_SCREW3_SIZE * DYN2B_SCREW3_SIZE];
    dyn2b_to_mat_abi3(m_in, m_mat);

    double m_proj[DYN2B_SCREW3_SIZE * DYN2B_SCREW3_SIZE];
    cblas_dsymm(CblasColMajor, CblasRight, CblasUpper,
            DYN2B_SCREW3_SIZE, DYN2B_SCREW3_SIZE,
            1.0, m_mat, DYN2B_SCREW3_SIZE,
            proj, DYN2B_SCREW3_SIZE,
            0.0, m_proj, DYN2B_SCREW3_SIZE);

    dyn2b_to_tup_abi3(m_proj, m_out);
}


void dyn2b_jnt_proj_wrench3(
        int n,
        int dof,
        const double *restrict jac,
        const double *restrict d,
        const double *restrict m,
        const double *restrict f_in,
        double *restrict f_out)
{
    assert(n >= 0);
    assert(dof >= 0);
    assert(dof <= 6);
    assert(jac);
    assert(d);
    assert(m);
    assert(f_in);
    assert(f_out);

    // Construct inertia matrix to simplify the code below (at the expense of
    // slightly more computations)
    double m_mat[DYN2B_SCREW3_SIZE * DYN2B_SCREW3_SIZE];
    dyn2b_to_mat_abi3(m, m_mat);

    // D^{-1}
    double d_inv[dof * dof];
    dyn2b_jnt_inv_abi3(dof, jac, m_mat, d, d_inv);


    // S^T F
    // Note: S (angular-before-linear) vs. F (linear-before-angular)
    double stf[dof * n];
    cblas_dgemm(CblasColMajor, CblasTrans, CblasNoTrans,
            dof, n, DYN2B_SCREW3_DIR_SIZE,
            1.0, &jac[DYN2B_TWIST3_ANG_OFFSET], DYN2B_TWIST3_SIZE,
            &f_in[DYN2B_WRENCH3_ANG_OFFSET], DYN2B_WRENCH3_SIZE,
            0.0, stf, dof);
    cblas_dgemm(CblasColMajor, CblasTrans, CblasNoTrans,
            dof, n, DYN2B_SCREW3_MOM_SIZE,
            1.0, &jac[DYN2B_TWIST3_LIN_OFFSET], DYN2B_TWIST3_SIZE,
            &f_in[DYN2B_WRENCH3_LIN_OFFSET], DYN2B_WRENCH3_SIZE,
            1.0, stf, dof);

    // D^{-1} (S^T F)
    double distf[dof * n];
    cblas_dsymm(CblasColMajor, CblasLeft, CblasUpper, dof, n,
            1.0, d_inv, n,
            stf, dof,
            0.0, distf, dof);

    // S (D^{-1} S^T F)
    double sdistf[DYN2B_TWIST3_SIZE * n];
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans,
            DYN2B_TWIST3_SIZE, n, dof,
            1.0, jac, DYN2B_TWIST3_SIZE,
            distf, dof,
            0.0, sdistf, DYN2B_TWIST3_SIZE);

    // F - (M^A S D^{-1} S^T F)
    // Note: S (angular-before-linear) vs. F (linear-before-angular)
    memcpy(f_out, f_in, DYN2B_WRENCH3_SIZE * n * sizeof(double));

    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans,
            DYN2B_SCREW3_DIR_SIZE, n, DYN2B_SCREW3_SIZE,
            -1.0, &m_mat[DYN2B_SCREW3_DIR_OFFSET], DYN2B_SCREW3_SIZE,
            sdistf, DYN2B_SCREW3_SIZE,
            1.0, &f_out[DYN2B_WRENCH3_ANG_OFFSET], DYN2B_SCREW3_SIZE);
    cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans,
            DYN2B_SCREW3_MOM_SIZE, n, DYN2B_SCREW3_SIZE,
            -1.0, &m_mat[DYN2B_SCREW3_MOM_OFFSET], DYN2B_SCREW3_SIZE,
            sdistf, DYN2B_SCREW3_SIZE,
            1.0, &f_out[DYN2B_WRENCH3_LIN_OFFSET], DYN2B_SCREW3_SIZE);
}
