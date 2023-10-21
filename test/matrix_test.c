// SPDX-License-Identifier: LGPL-3.0
#include <dyn2b/functions/matrix.h>
#include <check.h>

#include "common.h"


START_TEST(test_cpy_mat)
{
    double a[2 * 3] = {
        2.0, 4.0, 6.0,
        4.0, 6.0, 8.0
    };
    double out[3 * 8] = {
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0,
        3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };

    double res[3 * 8] = {
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0,
        3.0, 0.0, 2.0, 4.0, 6.0, 0.0, 0.0, 0.0,
        5.0, 0.0, 4.0, 6.0, 8.0, 0.0, 0.0, 0.0
    };
    dyn2b_cpy_mat(2, 3,
            a, 3,
            &out[(8 * 1) + 2], 8);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 8; j++) {
            int idx = (i * 8) + j;
            ck_assert_flt_eq(out[idx], res[idx]);
        }
    }
}
END_TEST


START_TEST(test_mad_mat)
{
    double a[6 * 2] = {
        1.0, 2.0, 3.0, 0.0, 0.0, 0.0,
        4.0, 5.0, 6.0, 0.0, 0.0, 0.0
    };
    double b[6 * 2] = {
        2.0, 3.0, 4.0, 0.0, 0.0, 0.0,
        6.0, 7.0, 8.0, 0.0, 0.0, 0.0
    };
    double out[8 * 3] = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };

    double res[8 * 3] = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 2.0, 2.0, 2.0, 0.0, 0.0, 0.0
    };
    dyn2b_mad_mat(2, 3,
            -1.0, a, 6,
            b, 6,
            &out[(8 * 1) + 2], 8);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 8; j++) {
            int idx = (i * 8) + j;
            ck_assert_flt_eq(out[idx], res[idx]);
        }
    }
}
END_TEST


TCase *matrix_test()
{
    TCase *tc = tcase_create("Matrix");

    tcase_add_test(tc, test_cpy_mat);
    tcase_add_test(tc, test_mad_mat);

    return tc;
}