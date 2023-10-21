// SPDX-License-Identifier: LGPL-3.0
#include <dyn2b/functions/mechanics.h>
#include <dyn2b/types/mechanics.h>
#include <dyn2b/types/screw.h>
#include <math.h>
#include <check.h>

#include "common.h"


START_TEST(test_tf_dist_acc3)
{
    double tf[DYN2B_POSE3_SIZE] = {
        0.0, 0.0, 1.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        1.0, 2.0, 3.0
    };
    double v_abs[DYN2B_SCREW3_SIZE] = {
        1.0, 2.0, 3.0, 2.0, 3.0, 4.0
    };
    double v_rel[DYN2B_SCREW3_SIZE] = {
        2.0, 3.0, 4.0, 3.0, 4.0, 5.0
    };
    double in[DYN2B_SCREW3_SIZE] = {    // angular-before-linear
        1.0, 2.0, 3.0, 2.0, 3.0, 4.0,
    };
    double out[DYN2B_SCREW3_SIZE];

    double res[DYN2B_SCREW3_SIZE] = {
        2.0, 3.0, 1.0, 2.0, 6.0, 1.0
    };

    dyn2b_tf_dist_acc3(tf, v_abs, v_rel, in, out);
    for (int i = 0; i < DYN2B_SCREW3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_rbi_to_wrench3)
{
    double m[DYN2B_RBI3_SIZE] = {
        // I
        3.0, 4.0, 5.0,
        4.0, 6.0, 7.0,
        5.0, 7.0, 8.0,
        // h
        4.0, 6.0, 8.0,
        // m
        2.0
    };
    double in[DYN2B_SCREW3_SIZE] = {
        1.0, 2.0, 3.0, 3.0, 4.0, 5.0
    };
    double out[DYN2B_SCREW3_SIZE] = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };

    double res[DYN2B_SCREW3_SIZE] = {
        4.0, 12.0, 8.0, 24.0, 41.0, 41.0
    };
    dyn2b_rbi_to_wrench3(m, in, out);
    for (int i = 0; i < DYN2B_SCREW3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


START_TEST(test_to_nrt_wrench3)
{
    double m[DYN2B_RBI3_SIZE] = {
        // I
        3.0, 4.0, 5.0,
        4.0, 6.0, 7.0,
        5.0, 7.0, 8.0,
        // h
        4.0, 6.0, 8.0,
        // m
        2.0
    };
    double v[DYN2B_SCREW3_SIZE] = {
        1.0, 2.0, 3.0, 3.0, 4.0, 5.0
    };
    double out[DYN2B_SCREW3_SIZE];

    double res[DYN2B_SCREW3_SIZE] = {
        -20.0, 4.0, 4.0, -69.0, 27.0, 13.0
    };
    dyn2b_nrt_wrench3(m, v, out);
    for (int i = 0; i < DYN2B_SCREW3_SIZE; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


TCase *mechanics_test()
{
    TCase *tc = tcase_create("Mechanics");

    tcase_add_test(tc, test_tf_dist_acc3);
    tcase_add_test(tc, test_rbi_to_wrench3);
    tcase_add_test(tc, test_to_nrt_wrench3);

    return tc;
}
