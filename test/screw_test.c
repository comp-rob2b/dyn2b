// SPDX-License-Identifier: LGPL-3.0
#include <dyn2b/functions/screw.h>
#include <dyn2b/types/screw.h>
#include <math.h>
#include <check.h>
#include <stdbool.h>

#include "common.h"


#define N 2


START_TEST(test_cmp_pose3)
{
    double a[DYN2B_POSE3_SIZE] = {
        0.0, 0.0, 1.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        1.0, 2.0, 3.0
    };
    double b[DYN2B_POSE3_SIZE] = {
        cos(M_PI_4), 0.0, -sin(M_PI_4),
            0.0    , 1.0,      0.0    ,
        sin(M_PI_4), 0.0,  cos(M_PI_4),
            3.0    , 2.0,      1.0
    };
    double out[DYN2B_POSE3_SIZE];

    double res1[DYN2B_POSE3_SIZE] = {
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        1.0, 0.0, 0.0,
        3.0, 5.0, 4.0
    };
    double res2[DYN2B_POSE3_SIZE] = {
        0.0, -sin(M_PI_4), cos(M_PI_4),
        1.0,      0.0    ,     0.0,
        0.0,  cos(M_PI_4), sin(M_PI_4),
        3.0,      3.0    ,     6.0
    };

    dyn2b_cmp_pose3(a, a, out);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            ck_assert_flt_eq(out[(i * 3) + j], res1[(i * 3) + j]);
        }
    }

    dyn2b_cmp_pose3(a, b, out);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            ck_assert_flt_eq(out[(i * 3) + j], res2[(i * 3) + j]);
        }
    }
}
END_TEST


START_TEST(test_dot_screw3)
{
    // wrench, linear-before-angular
    double in1[DYN2B_SCREW3_SIZE * N] = {
        1.0, 2.0, 3.0, 2.0,  3.0,  4.0,
        2.0, 4.0, 6.0, 8.0, 10.0, 12.0
    };
    // twist, angular-before-linear
    double in2[DYN2B_SCREW3_SIZE * N] = {
        1.0, 2.0, 3.0, 3.0, 4.0, 5.0,
        5.0, 6.0, 7.0, 7.0, 8.0, 9.0
    };
    double out[DYN2B_SCREW3_SIZE * N];

    double res[N * N] = {
         46.0, 116.0,
        106.0, 284.0
    };
    dyn2b_dot_screw3(N, N, in1, in2, out);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            ck_assert_flt_eq(out[(i * N) + j], res[(i * N) + j]);
        }
    }
}
END_TEST


START_TEST(test_crs_screw3)
{
    double in1[DYN2B_SCREW3_SIZE] = {
        1.0, 2.0, 3.0, 2.0, 3.0, 4.0
    };
    double in2[DYN2B_SCREW3_SIZE] = {
        2.0, 3.0, 4.0, 3.0, 4.0, 5.0
    };
    double out[DYN2B_SCREW3_SIZE];

    double res1[DYN2B_SCREW3_SIZE] = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };
    dyn2b_crs_screw3(in1, in1, out);
    for (int i = 0; i < DYN2B_SCREW3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res1[i]);
    }

    double res2[DYN2B_SCREW3_SIZE] = {
        -1.0, 2.0, -1.0, -2.0, 4.0, -2.0
    };
    dyn2b_crs_screw3(in1, in2, out);
    for (int i = 0; i < DYN2B_SCREW3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res2[i]);
    }
}
END_TEST


START_TEST(test_cad_screw3)
{
    double in1[DYN2B_SCREW3_SIZE] = {
        1.0, 2.0, 3.0, 2.0, 3.0, 4.0
    };
    double in2[DYN2B_SCREW3_SIZE] = {
        2.0, 3.0, 4.0, 3.0, 4.0, 5.0
    };
    double in3[DYN2B_SCREW3_SIZE] = {
        2.0, 3.0, 4.0, 3.0, 4.0, 5.0
    };
    double out[DYN2B_SCREW3_SIZE];

    double res1[DYN2B_SCREW3_SIZE] = {
        2.0, 3.0, 4.0, 3.0, 4.0, 5.0
    };
    dyn2b_cad_screw3(in3, in1, in1, out);
    for (int i = 0; i < DYN2B_SCREW3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res1[i]);
    }

    double res2[DYN2B_SCREW3_SIZE] = {
        1.0, 5.0, 3.0, 1.0, 8.0, 3.0
    };
    dyn2b_cad_screw3(in3, in1, in3, out);
    for (int i = 0; i < DYN2B_SCREW3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res2[i]);
    }
}
END_TEST


START_TEST(test_tf_dist_screw3)
{
    double tf1[DYN2B_POSE3_SIZE] = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        2.0, 3.0, 4.0
    };
    double tf2[DYN2B_POSE3_SIZE] = {
        0.0, 0.0, 1.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        1.0, 2.0, 3.0
    };
    double in[DYN2B_SCREW3_SIZE * N] = {    // direction-before-moment
        1.0, 2.0, 3.0, 2.0, 3.0, 4.0,
        1.0, 2.0, 3.0, 2.0, 3.0, 4.0
    };
    double out[DYN2B_SCREW3_SIZE * N];

    double res1[DYN2B_SCREW3_SIZE * N] = {
        1.0, 2.0, 3.0, 1.0, 5.0, 3.0,
        1.0, 2.0, 3.0, 1.0, 5.0, 3.0
    };
    double res2[DYN2B_SCREW3_SIZE * N] = {
        3.0, 1.0, 2.0, 4.0, 2.0, 3.0,
        3.0, 1.0, 2.0, 4.0, 2.0, 3.0
    };

    dyn2b_tf_dist_screw3(N, tf1, in, out);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < DYN2B_SCREW3_SIZE; j++) {
            ck_assert_flt_eq(out[(i * N) + j], res1[(i * N) + j]);
        }
    }

    dyn2b_tf_dist_screw3(N, tf2, in, out);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < DYN2B_SCREW3_SIZE; j++) {
            ck_assert_flt_eq(out[(i * N) + j], res2[(i * N) + j]);
        }
    }
}
END_TEST


START_TEST(test_tf_prox_screw3)
{
    double tf1[DYN2B_POSE3_SIZE] = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        2.0, 3.0, 4.0
    };
    double tf2[DYN2B_POSE3_SIZE] = {
        0.0, 0.0, 1.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        1.0, 2.0, 3.0
    };
    double in[DYN2B_SCREW3_SIZE * N] = {    // direction-before-moment
        1.0, 2.0, 3.0, 2.0, 3.0, 4.0,
        1.0, 2.0, 3.0, 2.0, 3.0, 4.0
    };
    double out[DYN2B_SCREW3_SIZE * N];

    double res1[DYN2B_SCREW3_SIZE * N] = {
        1.0, 2.0, 3.0, 3.0, 1.0, 5.0,
        1.0, 2.0, 3.0, 3.0, 1.0, 5.0
    };
    double res2[DYN2B_SCREW3_SIZE * N] = {
        2.0, 3.0, 1.0, -4.0, 9.0, 1.0,
        2.0, 3.0, 1.0, -4.0, 9.0, 1.0
    };

    dyn2b_tf_prox_screw3(N, tf1, in, out);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < DYN2B_SCREW3_SIZE; j++) {
            ck_assert_flt_eq(out[(i * N) + j], res1[(i * N) + j]);
        }
    }

    dyn2b_tf_prox_screw3(N, tf2, in, out);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < DYN2B_SCREW3_SIZE; j++) {
            ck_assert_flt_eq(out[(i * N) + j], res2[(i * N) + j]);
        }
    }
}
END_TEST


TCase *screw_test()
{
    TCase *tc = tcase_create("Screw");

    tcase_add_test(tc, test_cmp_pose3);
    tcase_add_test(tc, test_dot_screw3);
    tcase_add_test(tc, test_crs_screw3);
    tcase_add_test(tc, test_cad_screw3);
    tcase_add_test(tc, test_tf_dist_screw3);
    tcase_add_test(tc, test_tf_prox_screw3);

    return tc;
}