// SPDX-License-Identifier: LGPL-3.0
#include <check.h>

extern TCase *vector3_test();
extern TCase *matrix_test();
extern TCase *screw_test();


int main(int argc, char **argv)
{
    Suite *s = suite_create("Core");
    suite_add_tcase(s, vector3_test());
    suite_add_tcase(s, matrix_test());
    suite_add_tcase(s, screw_test());

    SRunner *sr = srunner_create(s);

    srunner_run_all(sr, CK_ENV);
    int nf = srunner_ntests_failed(sr);
    srunner_free(sr);

    return nf == 0 ? 0 : 1;
}
