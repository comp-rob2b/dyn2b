// SPDX-License-Identifier: LGPL-3.0
#include <dyn2b/functions/vector3.h>
#include <check.h>
#include <math.h>

#include "common.h"


START_TEST(test_crs_vec3)
{
    double a[6 * 2] = {
        1.0, 2.0, 3.0, 0.0, 0.0, 0.0,
        4.0, 5.0, 6.0, 0.0, 0.0, 0.0
    };
    double b[6 * 2] = {
        2.0, 3.0, 4.0, 0.0, 0.0, 0.0,
        6.0, 7.0, 8.0, 0.0, 0.0, 0.0
    };
    double out[4 * 2];
    memset(out, 0, 4 * 2 * sizeof(double));

    double res1[4 * 2] = {
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0
    };
    dyn2b_crs_vec3(2, a, 6, a, 6, out, 4);
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            int idx = (i * 4) + j;
            ck_assert_flt_eq(out[idx], res1[idx]);
        }
    }

    double res2[4 * 2] = {
        -1.0, 2.0, -1.0, 0.0,
        -2.0, 4.0, -2.0, 0.0
    };
    dyn2b_crs_vec3(2, a, 6, b, 6, out, 4);
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            int idx = (i * 4) + j;
            ck_assert_flt_eq(out[idx], res2[idx]);
        }
    }
}
END_TEST


START_TEST(test_cad_vec3)
{
    double a[4 * 2] = {
        1.0, 2.0, 3.0, 0.0,
        4.0, 5.0, 6.0, 0.0
    };
    double b[6 * 2] = {
        1.0, 2.0, 3.0, 0.0, 0.0, 0.0,
        4.0, 5.0, 6.0, 0.0, 0.0, 0.0
    };
    double c[6 * 2] = {
        2.0, 3.0, 4.0, 0.0, 0.0, 0.0,
        6.0, 7.0, 8.0, 0.0, 0.0, 0.0
    };
    double out[4 * 2];
    memset(out, 0, 4 * 2 * sizeof(double));

    double res[4 * 2] = {
        0.0, 4.0, 2.0, 0.0,
        2.0, 9.0, 4.0, 0.0
    };
    dyn2b_cad_vec3(2, a, 4, b, 6, c, 6, out, 4);
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            int idx = (i * 4) + j;
            ck_assert_flt_eq(out[idx], res[idx]);
        }
    }
}
END_TEST


START_TEST(test_skw_vec3)
{
    double a[3] = { 1.0, 2.0, 3.0 };
    double out[3 * 3];

    double res[3 * 3] = {
         0.0,  3.0, -2.0,
        -3.0,  0.0,  1.0,
         2.0, -1.0,  0.0
    };

    dyn2b_skw_vec3(a, out);
    for (int i = 0; i < 9; i++) {
        ck_assert_flt_eq(out[i], res[i]);
    }
}
END_TEST


TCase *vector3_test()
{
    TCase *tc = tcase_create("Vector3");

    tcase_add_test(tc, test_crs_vec3);
    tcase_add_test(tc, test_cad_vec3);
    tcase_add_test(tc, test_skw_vec3);

    return tc;
}