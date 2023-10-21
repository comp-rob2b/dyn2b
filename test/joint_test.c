// SPDX-License-Identifier: LGPL-3.0
#include <dyn2b/functions/joint.h>
#include <dyn2b/functions/mechanics.h>
#include <dyn2b/types/screw.h>
#include <dyn2b/types/mechanics.h>
#include <dyn2b/types/joint.h>
#include <check.h>
#include <math.h>

#include "common.h"


#define N 2

// Wrench
static double w[DYN2B_SCREW3_SIZE * N] = {
    2.0,  3.0,  4.0, 1.0, 2.0, 3.0,
    8.0, 10.0, 12.0, 2.0, 4.0, 6.0
};

// Articulated-body inertia
static const double m[27] = {
    // I
    1.0, 2.0, 3.0,
    2.0, 2.0, 3.0,
    3.0, 3.0, 3.0,
    // H
    2.0, 3.0, 4.0,
    3.0, 3.0, 4.0,
    4.0, 4.0, 4.0,
    // M
    4.0, 5.0, 6.0,
    5.0, 5.0, 6.0,
    6.0, 6.0, 6.0
};

// Actuator inertia
static const double d[1] = { 3.0 };

// Jacobian of a revolute-z joint
static double s[DYN2B_SCREW3_SIZE] = {
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0
};


START_TEST(test_rev_x_to_pose3)
{
    double in[1] = { M_PI_2 };
    double out[DYN2B_POSE3_SIZE];


    double res[DYN2B_POSE3_SIZE] = { // column-major layout
        1.0,  0.0, 0.0,
        0.0,  0.0, 1.0,
        0.0, -1.0, 0.0,
        0.0,  0.0, 0.0
    };

    dyn2b_rev_x_to_pose3(in, out);
    for (int i = 0; i < DYN2B_POSE3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_rev_y_to_pose3)
{
    double in[1] = { M_PI_2 };
    double out[DYN2B_POSE3_SIZE];

    double res[DYN2B_POSE3_SIZE] = { // column-major layout
        0.0, 0.0, -1.0,
        0.0, 1.0,  0.0,
        1.0, 0.0,  0.0,
        0.0, 0.0,  0.0
    };

    dyn2b_rev_y_to_pose3(in, out);
    for (int i = 0; i < DYN2B_POSE3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_rev_z_to_pose3)
{
    double in[1] = { M_PI_2 };
    double out[DYN2B_POSE3_SIZE];

    double res[DYN2B_POSE3_SIZE] = { // column-major layout
         0.0, 1.0, 0.0,
        -1.0, 0.0, 0.0,
         0.0, 0.0, 1.0,
         0.0, 0.0, 0.0
    };

    dyn2b_rev_z_to_pose3(in, out);
    for (int i = 0; i < DYN2B_POSE3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_trans_x_to_pose3)
{
    double in[1] = { 1.0 };
    double out[DYN2B_POSE3_SIZE];

    double res[DYN2B_POSE3_SIZE] = { // column-major layout
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        1.0, 0.0, 0.0
    };

    dyn2b_trans_x_to_pose3(in, out);
    for (int i = 0; i < DYN2B_POSE3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_trans_y_to_pose3)
{
    double in[1] = { 1.0 };
    double out[DYN2B_POSE3_SIZE];

    double res[DYN2B_POSE3_SIZE] = { // column-major layout
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        0.0, 1.0, 0.0
    };

    dyn2b_trans_y_to_pose3(in, out);
    for (int i = 0; i < DYN2B_POSE3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_trans_z_to_pose3)
{
    double in[1] = { 1.0 };
    double out[DYN2B_POSE3_SIZE];

    double res[DYN2B_POSE3_SIZE] = { // column-major layout
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        0.0, 0.0, 1.0
    };

    dyn2b_trans_z_to_pose3(in, out);
    for (int i = 0; i < DYN2B_POSE3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_rev_x_to_twist3)
{
    double in[1] = { 1.0 };
    double out[DYN2B_SCREW3_SIZE];

    double res[DYN2B_SCREW3_SIZE] = {
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };

    dyn2b_rev_x_to_twist3(in, out);
    for (int i = 0; i < DYN2B_SCREW3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_rev_y_to_twist3)
{
    double in[1] = { 1.0 };
    double out[DYN2B_SCREW3_SIZE];

    double res[DYN2B_SCREW3_SIZE] = {
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0
    };

    dyn2b_rev_y_to_twist3(in, out);
    for (int i = 0; i < DYN2B_SCREW3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_rev_z_to_twist3)
{
    double in[1] = { 1.0 };
    double out[DYN2B_SCREW3_SIZE];

    double res[DYN2B_SCREW3_SIZE] = {
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0
    };

    dyn2b_rev_z_to_twist3(in, out);
    for (int i = 0; i < DYN2B_SCREW3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_trans_x_to_twist3)
{
    double in[1] = { 1.0 };
    double out[DYN2B_SCREW3_SIZE];

    double res[DYN2B_SCREW3_SIZE] = {
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0
    };

    dyn2b_trans_x_to_twist3(in, out);
    for (int i = 0; i < DYN2B_SCREW3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_trans_y_to_twist3)
{
    double in[1] = { 1.0 };
    double out[DYN2B_SCREW3_SIZE];

    double res[DYN2B_SCREW3_SIZE] = {
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0
    };

    dyn2b_trans_y_to_twist3(in, out);
    for (int i = 0; i < DYN2B_SCREW3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_trans_z_to_twist3)
{
    double in[1] = { 1.0 };
    double out[DYN2B_SCREW3_SIZE];

    double res[DYN2B_SCREW3_SIZE] = {
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0
    };

    dyn2b_trans_z_to_twist3(in, out);
    for (int i = 0; i < DYN2B_SCREW3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_rev_x_from_wrench3)
{
    double out[N];

    double res[N] = {
        1.0,
        2.0
    };

    dyn2b_rev_x_from_wrench3(N, w, out);
    for (int i = 0; i < N; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_rev_y_from_wrench3)
{
    double out[N];

    double res[N] = {
        2.0,
        4.0
    };

    dyn2b_rev_y_from_wrench3(N, w, out);
    for (int i = 0; i < N; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_rev_z_from_wrench3)
{
    double out[N];

    double res[N] = {
        3.0,
        6.0
    };

    dyn2b_rev_z_from_wrench3(N, w, out);
    for (int i = 0; i < N; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_trans_x_from_wrench3)
{
    double out[N];

    double res[N] = {
        2.0,
        8.0
    };

    dyn2b_trans_x_from_wrench3(N, w, out);
    for (int i = 0; i < N; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_trans_y_from_wrench3)
{
    double out[N];

    double res[N] = {
         3.0,
        10.0
    };

    dyn2b_trans_y_from_wrench3(N, w, out);
    for (int i = 0; i < N; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_trans_z_from_wrench3)
{
    double out[N];

    double res[N] = {
         4.0,
        12.0
    };

    dyn2b_trans_z_from_wrench3(N, w, out);
    for (int i = 0; i < N; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_to_abi3)
{
    double in[DYN2B_RBI3_SIZE] = {
        // I
        3.0, 4.0, 5.0,
        4.0, 6.0, 7.0,
        5.0, 7.0, 8.0,
        // h
        4.0, 6.0, 8.0,
        // m
        2.0
    };
    double out[DYN2B_ABI3_SIZE];

    double res[DYN2B_ABI3_SIZE] = {
        // I
         3.0, 4.0, 5.0,
         4.0, 6.0, 7.0,
         5.0, 7.0, 8.0,
        // H
         0.0,  8.0, -6.0,
        -8.0,  0.0,  4.0,
         6.0, -4.0,  0.0,
        // M
         2.0, 0.0, 0.0,
         0.0, 2.0, 0.0,
         0.0, 0.0, 2.0
    };
    dyn2b_to_abi3(in, out);
    for (int i = 0; i < DYN2B_ABI3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_tf_prox_abi3)
{
    double tf[DYN2B_POSE3_SIZE] = {
        0.0, 0.0, 1.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        1.0, 2.0, 3.0
    };
    double in[DYN2B_ABI3_SIZE] = {
        // I
        3.0, 4.0, 5.0,
        4.0, 6.0, 7.0,
        5.0, 7.0, 8.0,
        // H
        4.0, 3.0, 2.0,
        5.0, 4.0, 3.0,
        6.0, 5.0, 4.0,
        // M
        2.0, 3.0, 4.0,
        3.0, 4.0, 5.0,
        4.0, 5.0, 6.0
    };
    double out[DYN2B_ABI3_SIZE];

    double res[DYN2B_ABI3_SIZE] = {
        // I
          2.0, -11.0, - 1.0,
        -11.0,  42.0,   7.0,
        - 1.0,   7.0, - 3.0,
        // H
        -5.0, 12.0, 2.0,
        -5.0, 15.0, 2.0,
        -5.0,  9.0, 2.0,
        // M
        4.0, 5.0, 3.0,
        5.0, 6.0, 4.0,
        3.0, 4.0, 2.0
    };

    dyn2b_tf_prox_abi3(tf, in, out);
    for (int i = 0; i < DYN2B_ABI3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_abi_to_wrench3)
{
    double m[DYN2B_ABI3_SIZE] = {
        // I
        3.0, 4.0, 5.0,
        4.0, 6.0, 7.0,
        5.0, 7.0, 8.0,
        // H
        4.0, 5.0, 6.0,
        5.0, 6.0, 7.0,
        6.0, 7.0, 8.0,
        // M
        1.0, 2.0, 3.0,
        2.0, 3.0, 4.0,
        3.0, 4.0, 5.0
    };
    double in[DYN2B_SCREW3_SIZE * N] = {
        1.0, 2.0, 3.0, 3.0, 4.0, 5.0,
        1.0, 2.0, 3.0, 3.0, 4.0, 5.0
    };
    double out[DYN2B_SCREW3_SIZE * N];

    double res[DYN2B_SCREW3_SIZE * N] = {
        58.0, 76.0, 94.0, 88.0, 111.0, 129.0,
        58.0, 76.0, 94.0, 88.0, 111.0, 129.0
    };
    dyn2b_abi_to_wrench3(N, m, in, out);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < DYN2B_SCREW3_SIZE; j++) {
            ck_assert_flt_eq(
                    out[(i * DYN2B_SCREW3_SIZE) + j],
                    res[(i * DYN2B_SCREW3_SIZE) + j]);
        }
    }
}
END_TEST


START_TEST(test_rev_x_proj_abi3)
{
    double m_out[27];

    double res[27] = {
        // I
        0.75, 1.5, 2.25,
        1.5 , 1.0, 1.5,
        2.25, 1.5, 0.75,
        // H
        1.5 , 2.0, 2.5,
        2.25, 1.5, 1.75,
        3.0 , 2.0, 1.0,
        // M
        3.0, 3.5 , 4.0,
        3.5, 2.75, 3.0,
        4.0, 3.0 , 2.0
    };

    dyn2b_rev_x_proj_abi3(d, m, m_out);
    for (int i = 0; i < 27; i++) {
        ck_assert_flt_eq(m_out[i], res[i]);
    }
}
END_TEST


START_TEST(test_rev_y_proj_abi3)
{
    double m_out[27];

    double res[27] = {
        // I
        0.2, 1.2, 1.8,
        1.2, 1.2, 1.8,
        1.8, 1.8, 1.2,
        // H
        0.8, 1.8, 2.2,
        1.8, 1.8, 2.2,
        2.4, 2.4, 1.6,
        // M
        2.2, 3.2, 3.6,
        3.2, 3.2, 3.6,
        3.6, 3.6, 2.8
    };

    dyn2b_rev_y_proj_abi3(d, m, m_out);
    for (int i = 0; i < 27; i++) {
        ck_assert_flt_eq(m_out[i], res[i]);
    }
}
END_TEST


START_TEST(test_rev_z_proj_abi3)
{
    double m_out[27];

    double res[27] = {
        // I
        -0.5, 0.5, 1.5,
         0.5, 0.5, 1.5,
         1.5, 1.5, 1.5,
        // H
        0.0, 1.0, 2.0,
        1.0, 1.0, 2.0,
        2.0, 2.0, 2.0,
        // M
         4.0 / 3.0,  7.0 / 3.0, 10.0 / 3.0,
         7.0 / 3.0,  7.0 / 3.0, 10.0 / 3.0,
        10.0 / 3.0, 10.0 / 3.0, 10.0 / 3.0
    };

    dyn2b_rev_z_proj_abi3(d, m, m_out);
    for (int i = 0; i < 27; i++) {
        ck_assert_flt_eq(m_out[i], res[i]);
    }
}
END_TEST


START_TEST(test_trans_x_proj_abi3)
{
    double m_out[27];

    double res[27] = {
        // I
        0.4286, 1.1429, 1.8571,
        1.1429, 0.7143, 1.2857,
        1.8571, 1.2857, 0.7143,
        // H
        0.8571, 1.2857, 1.7143,
        1.5714, 0.8571, 1.1429,
        2.2857, 1.4286, 0.5714,
        // M
        1.7143, 2.1429, 2.5714,
        2.1429, 1.4286, 1.7143,
        2.5714, 1.7143, 0.8571
    };

    dyn2b_trans_x_proj_abi3(d, m, m_out);
    for (int i = 0; i < 27; i++) {
        ck_assert_flt_eq(m_out[i], res[i]);
    }
}
END_TEST


START_TEST(test_trans_y_proj_abi3)
{
    double m_out[27];

    double res[27] = {
        // I
        -0.125, 0.875, 1.5,
         0.875, 0.875, 1.5,
         1.5  , 1.5  , 1.0,
        // H
        0.125, 1.125, 1.5,
        1.125, 1.125, 1.5,
        1.75 , 1.75 , 1.0,
        // M
        0.875, 1.875, 2.25,
        1.875, 1.875, 2.25,
        2.25 , 2.25 , 1.5
    };

    dyn2b_trans_y_proj_abi3(d, m, m_out);
    for (int i = 0; i < 27; i++) {
        ck_assert_flt_eq(m_out[i], res[i]);
    }
}
END_TEST


START_TEST(test_trans_z_proj_abi3)
{
    double m_out[27];

    double res[27] = {
        // I
        -0.7778, 0.2222, 1.2222,
         0.2222, 0.2222, 1.2222,
         1.2222, 1.2222, 1.2222,
        // H
        -0.6667, 0.3333, 1.3333,
         0.3333, 0.3333, 1.3333,
         1.3333, 1.3333, 1.3333,
        // M
        0.0, 1.0, 2.0,
        1.0, 1.0, 2.0,
        2.0, 2.0, 2.0
    };

    dyn2b_trans_z_proj_abi3(d, m, m_out);
    for (int i = 0; i < 27; i++) {
        ck_assert_flt_eq(m_out[i], res[i]);
    }
}
END_TEST


START_TEST(test_rev_x_proj_wrench3)
{
    double out[DYN2B_SCREW3_SIZE * N];

    double res[DYN2B_SCREW3_SIZE * N] = {
        1.5, 2.25,  3.0, 0.75, 1.5, 2.25,
        7.0, 8.5 , 10.0, 1.5 , 3.0, 4.5
    };

    dyn2b_rev_x_proj_wrench3(N, d, m, w, out);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < DYN2B_SCREW3_SIZE; j++) {
            ck_assert_flt_eq(
                    out[(i * DYN2B_SCREW3_SIZE) + j],
                    res[(i * DYN2B_SCREW3_SIZE) + j]);
        }
    }
}
END_TEST


START_TEST(test_rev_y_proj_wrench3)
{
    double out[DYN2B_SCREW3_SIZE * N];

    double res[DYN2B_SCREW3_SIZE * N] = {
        0.8, 1.8, 2.4, 0.2, 1.2, 1.8,
        5.6, 7.6, 8.8, 0.4, 2.4, 3.6
    };

    dyn2b_rev_y_proj_wrench3(N, d, m, w, out);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < DYN2B_SCREW3_SIZE; j++) {
            ck_assert_flt_eq(
                    out[(i * DYN2B_SCREW3_SIZE) + j],
                    res[(i * DYN2B_SCREW3_SIZE) + j]);
        }
    }
}
END_TEST


START_TEST(test_rev_z_proj_wrench3)
{
    double out[DYN2B_SCREW3_SIZE * N];

    double res[DYN2B_SCREW3_SIZE * N] = {
        0.0, 1.0, 2.0, -0.5, 0.5, 1.5,
        4.0, 6.0, 8.0, -1.0, 1.0, 3.0
    };

    dyn2b_rev_z_proj_wrench3(N, d, m, w, out);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < DYN2B_SCREW3_SIZE; j++) {
            ck_assert_flt_eq(
                    out[(i * DYN2B_SCREW3_SIZE) + j],
                    res[(i * DYN2B_SCREW3_SIZE) + j]);
        }
    }
}
END_TEST


START_TEST(test_trans_x_proj_wrench3)
{
    double out[DYN2B_SCREW3_SIZE * N];

    double res[DYN2B_SCREW3_SIZE * N] = {
        0.8571, 1.5714, 2.2857,  0.42857, 1.1429, 1.8571,
        3.4286, 4.2857, 5.1429, -0.28571, 0.5714, 1.4286
    };

    dyn2b_trans_x_proj_wrench3(N, d, m, w, out);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < DYN2B_SCREW3_SIZE; j++) {
            ck_assert_flt_eq(
                    out[(i * DYN2B_SCREW3_SIZE) + j],
                    res[(i * DYN2B_SCREW3_SIZE) + j]);
        }
    }
}
END_TEST


START_TEST(test_trans_y_proj_wrench3)
{
    double out[DYN2B_SCREW3_SIZE * N];

    double res[DYN2B_SCREW3_SIZE * N] = {
        0.125, 1.125, 1.75, -0.125, 0.875, 1.5,
        1.75 , 3.75 , 4.5 , -1.75 , 0.25 , 1.0
    };

    dyn2b_trans_y_proj_wrench3(N, d, m, w, out);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < DYN2B_SCREW3_SIZE; j++) {
            ck_assert_flt_eq(
                    out[(i * DYN2B_SCREW3_SIZE) + j],
                    res[(i * DYN2B_SCREW3_SIZE) + j]);
        }
    }
}
END_TEST


START_TEST(test_trans_z_proj_wrench3)
{
    double out[DYN2B_SCREW3_SIZE * N];

    double res[DYN2B_SCREW3_SIZE * N] = {
        -0.6667, 0.3333, 1.3333, -0.7778,  0.2222, 1.2222,
         0.0   , 2.0   , 4.0   , -3.3333, -1.3333, 0.6667
    };

    dyn2b_trans_z_proj_wrench3(N, d, m, w, out);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < DYN2B_SCREW3_SIZE; j++) {
            ck_assert_flt_eq(
                    out[(i * DYN2B_SCREW3_SIZE) + j],
                    res[(i * DYN2B_SCREW3_SIZE) + j]);
        }
    }
}
END_TEST


START_TEST(test_to_mat_abi3)
{
    double ms[DYN2B_ABI3_SIZE] = {
        // I
        3.0, 4.0, 5.0,
        4.0, 6.0, 7.0,
        5.0, 7.0, 8.0,
        // H
        4.0, 5.0, 6.0,
        6.0, 7.0, 8.0,
        7.0, 8.0, 9.0,
        // M
        1.0, 2.0, 3.0,
        2.0, 3.0, 4.0,
        3.0, 4.0, 5.0
    };
    double out[DYN2B_SCREW3_SIZE * DYN2B_SCREW3_SIZE];

    double res[DYN2B_SCREW3_SIZE * DYN2B_SCREW3_SIZE] = {
        3.0, 4.0, 5.0, 4.0, 6.0, 7.0,
        4.0, 6.0, 7.0, 5.0, 7.0, 8.0,
        5.0, 7.0, 8.0, 6.0, 8.0, 9.0,
        4.0, 5.0, 6.0, 1.0, 2.0, 3.0,
        6.0, 7.0, 8.0, 2.0, 3.0, 4.0,
        7.0, 8.0, 9.0, 3.0, 4.0, 5.0
    };
    dyn2b_to_mat_abi3(ms, out);
    for (int i = 0; i < DYN2B_SCREW3_SIZE * DYN2B_SCREW3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_to_tup_abi3)
{
    double md[DYN2B_SCREW3_SIZE * DYN2B_SCREW3_SIZE] = {
        3.0, 4.0, 5.0, 4.0, 6.0, 7.0,
        4.0, 6.0, 7.0, 5.0, 7.0, 8.0,
        5.0, 7.0, 8.0, 6.0, 8.0, 9.0,
        4.0, 5.0, 6.0, 1.0, 2.0, 3.0,
        6.0, 7.0, 8.0, 2.0, 3.0, 4.0,
        7.0, 8.0, 9.0, 3.0, 4.0, 5.0
    };
    double out[DYN2B_ABI3_SIZE];

    double res[DYN2B_ABI3_SIZE] = {
        // I
        3.0, 4.0, 5.0,
        4.0, 6.0, 7.0,
        5.0, 7.0, 8.0,
        // H
        4.0, 5.0, 6.0,
        6.0, 7.0, 8.0,
        7.0, 8.0, 9.0,
        // M
        1.0, 2.0, 3.0,
        2.0, 3.0, 4.0,
        3.0, 4.0, 5.0
    };
    dyn2b_to_tup_abi3(md, out);
    for (int i = 0; i < DYN2B_ABI3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_jnt_inv_abi3)
{
    double s2[DYN2B_SCREW3_SIZE * 2] = {
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0
    };
    double d2[2 * 2] = {
        3.0, 4.0,
        4.0, 5.0
    };
    double out1[1 * 1];
    double out2[2 * 2];

    double m_mat[DYN2B_SCREW3_SIZE * DYN2B_SCREW3_SIZE];
    dyn2b_to_mat_abi3(m, m_mat);

    double res1[1 * 1] = {
         0.16667
    };
    dyn2b_jnt_inv_abi3(1, s, m_mat, d, out1);
    for (int i = 0; i < 1 * 1; i++) {
        ck_assert_flt_eq(out1[i], res1[i]);
    }

    double res2[2 * 2] = {
        -0.8889,  0.7778,
         0.7778, -0.5556
    };
    dyn2b_jnt_inv_abi3(2, s2, m_mat, d2, out2);
    for (int i = 0; i < 2 * 2; i++) {
        ck_assert_flt_eq(out2[i], res2[i]);
    }
}
END_TEST


START_TEST(test_jnt_to_proj3)
{
    double s2[DYN2B_SCREW3_SIZE * 2] = {
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0
    };
    double d2[2 * 2] = {
        3.0, 4.0,
        4.0, 5.0
    };
    double out[DYN2B_SCREW3_SIZE * DYN2B_SCREW3_SIZE];

    double res1[DYN2B_SCREW3_SIZE * DYN2B_SCREW3_SIZE] = {
         1.0,  0.0, 0.0,  0.0   ,  0.0   ,  0.0   ,
         0.0,  1.0, 0.0,  0.0   ,  0.0   ,  0.0   ,
        -0.5, -0.5, 0.5, -0.6667, -0.6667, -0.6667,
         0.0,  0.0, 0.0,  1.0   ,  0.0   ,  0.0   ,
         0.0,  0.0, 0.0,  0.0   ,  1.0   ,  0.0   ,
         0.0,  0.0, 0.0,  0.0   ,  0.0   ,  1.0
    };
    dyn2b_jnt_to_proj3(1, s, d, m, out);
    for (int i = 0; i < DYN2B_SCREW3_SIZE; i++) {
        for (int j = 0; j < DYN2B_SCREW3_SIZE; j++) {
            ck_assert_flt_eq(
                    out[(i * DYN2B_SCREW3_SIZE) + j],
                    res1[(i * DYN2B_SCREW3_SIZE) + j]);
        }
    }

    double res2[DYN2B_SCREW3_SIZE * DYN2B_SCREW3_SIZE] = {
         1.0   , 0.0   , 0.0   ,  0.0   ,  0.0   ,  0.0   ,
        -0.5556, 0.4444, 0.3333, -0.4444, -0.4444,  0.4444,
         0.1111, 0.1111, 0.3333, -0.1111, -0.1111, -0.8889,
         0.0   , 0.0   , 0.0   ,  1.0   ,  0.0   ,  0.0   ,
         0.0   , 0.0   , 0.0   ,  0.0   ,  1.0   ,  0.0   ,
         0.0   , 0.0   , 0.0   ,  0.0   ,  0.0   ,  1.0
    };
    dyn2b_jnt_to_proj3(2, s2, d2, m, out);
    for (int i = 0; i < DYN2B_SCREW3_SIZE; i++) {
        for (int j = 0; j < DYN2B_SCREW3_SIZE; j++) {
            ck_assert_flt_eq(
                    out[(i * DYN2B_SCREW3_SIZE) + j],
                    res2[(i * DYN2B_SCREW3_SIZE) + j]);
        }
    }
}
END_TEST


START_TEST(test_jnt_proj_abi3)
{
    double s2[DYN2B_SCREW3_SIZE * 2] = {
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0
    };
    double d2[2 * 2] = {
        3.0, 4.0,
        4.0, 5.0
    };
    double out[27];

    double res1[27] = {
        // I
        -0.5, 0.5, 1.5,
         0.5, 0.5, 1.5,
         1.5, 1.5, 1.5,
        // H
        0.0, 1.0, 2.0,
        1.0, 1.0, 2.0,
        2.0, 2.0, 2.0,
        // M
         4.0 / 3.0,  7.0 / 3.0, 10.0 / 3.0,
         7.0 / 3.0,  7.0 / 3.0, 10.0 / 3.0,
        10.0 / 3.0, 10.0 / 3.0, 10.0 / 3.0
    };
    dyn2b_jnt_proj_abi3(1, s, d, m, out);
    for (int i = 0; i < 27; i++) {
        ck_assert_flt_eq(out[i], res1[i]);
    }

    double res2[27] = {
        // I
        0.2222, 1.2222, 1.6667,
        1.2222, 1.2222, 1.6667,
        1.6667, 1.6667, 2.0,
        // H
        0.7778, 1.7778, 2.3333,
        1.7778, 1.7778, 2.3333,
        2.2222, 2.2222, 2.6667,
        // M
        2.2222, 3.2222, 3.7778,
        3.2222, 3.2222, 3.7778,
        3.7778, 3.7778, 4.2222
    };
    dyn2b_jnt_proj_abi3(2, s2, d2, m, out);
    for (int i = 0; i < 27; i++) {
        ck_assert_flt_eq(out[i], res2[i]);
    }
}
END_TEST


START_TEST(test_jnt_proj_wrench3)
{
    double s2[DYN2B_SCREW3_SIZE * 2] = {
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0
    };
    double d2[2 * 2] = {
        3.0, 4.0,
        4.0, 5.0
    };
    double out[DYN2B_SCREW3_SIZE * N];

    double res1[DYN2B_SCREW3_SIZE * N] = {
        0.0, 1.0, 2.0, -0.5, 0.5, 1.5,
        4.0, 6.0, 8.0, -1.0, 1.0, 3.0
    };
    dyn2b_jnt_proj_wrench3(N, 1, s, d, m, w, out);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < DYN2B_SCREW3_SIZE; j++) {
            ck_assert_flt_eq(
                    out[(i * DYN2B_SCREW3_SIZE) + j],
                    res1[(i * DYN2B_SCREW3_SIZE) + j]);
        }
    }

    double res2[DYN2B_SCREW3_SIZE * DYN2B_SCREW3_SIZE] = {
        0.7778, 1.7778, 2.2222, 0.2222, 1.2222, 1.6667,
        5.5556, 7.5556, 8.4444, 0.4444, 2.4444, 3.3333
    };
    dyn2b_jnt_proj_wrench3(N, 2, s2, d2, m, w, out);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < DYN2B_SCREW3_SIZE; j++) {
            ck_assert_flt_eq(
                    out[(i * DYN2B_SCREW3_SIZE) + j],
                    res2[(i * DYN2B_SCREW3_SIZE) + j]);
        }
    }
}
END_TEST


TCase *joint_test()
{
    TCase *tc = tcase_create("Joint");

    tcase_add_test(tc, test_rev_x_to_pose3);
    tcase_add_test(tc, test_rev_y_to_pose3);
    tcase_add_test(tc, test_rev_z_to_pose3);
    tcase_add_test(tc, test_trans_x_to_pose3);
    tcase_add_test(tc, test_trans_y_to_pose3);
    tcase_add_test(tc, test_trans_z_to_pose3);
    tcase_add_test(tc, test_rev_x_to_twist3);
    tcase_add_test(tc, test_rev_y_to_twist3);
    tcase_add_test(tc, test_rev_z_to_twist3);
    tcase_add_test(tc, test_trans_x_to_twist3);
    tcase_add_test(tc, test_trans_y_to_twist3);
    tcase_add_test(tc, test_trans_z_to_twist3);
    tcase_add_test(tc, test_rev_x_from_wrench3);
    tcase_add_test(tc, test_rev_y_from_wrench3);
    tcase_add_test(tc, test_rev_z_from_wrench3);
    tcase_add_test(tc, test_trans_x_from_wrench3);
    tcase_add_test(tc, test_trans_y_from_wrench3);
    tcase_add_test(tc, test_trans_z_from_wrench3);
    tcase_add_test(tc, test_to_abi3);
    tcase_add_test(tc, test_tf_prox_abi3);
    tcase_add_test(tc, test_abi_to_wrench3);
    tcase_add_test(tc, test_rev_x_proj_abi3);
    tcase_add_test(tc, test_rev_y_proj_abi3);
    tcase_add_test(tc, test_rev_z_proj_abi3);
    tcase_add_test(tc, test_trans_x_proj_abi3);
    tcase_add_test(tc, test_trans_y_proj_abi3);
    tcase_add_test(tc, test_trans_z_proj_abi3);
    tcase_add_test(tc, test_rev_x_proj_wrench3);
    tcase_add_test(tc, test_rev_y_proj_wrench3);
    tcase_add_test(tc, test_rev_z_proj_wrench3);
    tcase_add_test(tc, test_trans_x_proj_wrench3);
    tcase_add_test(tc, test_trans_y_proj_wrench3);
    tcase_add_test(tc, test_trans_z_proj_wrench3);
    tcase_add_test(tc, test_to_mat_abi3);
    tcase_add_test(tc, test_to_tup_abi3);
    tcase_add_test(tc, test_jnt_inv_abi3);
    tcase_add_test(tc, test_jnt_to_proj3);
    tcase_add_test(tc, test_jnt_proj_abi3);
    tcase_add_test(tc, test_jnt_proj_wrench3);

    return tc;
}
