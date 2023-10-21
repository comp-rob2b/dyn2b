// SPDX-License-Identifier: LGPL-3.0
#ifndef DYN2B_TYPES_MECHANICS_H
#define DYN2B_TYPES_MECHANICS_H

#ifdef __cplusplus
extern "C" {
#endif


// Twist: angular-before-linear
#define DYN2B_TWIST3_ANG_OFFSET DYN2B_SCREW3_DIR_OFFSET
#define DYN2B_TWIST3_ANG_SIZE   DYN3B_SCREW3_DIR_SIZE
#define DYN2B_TWIST3_LIN_OFFSET DYN2B_SCREW3_MOM_OFFSET
#define DYN2B_TWIST3_LIN_SIZE   DYN3B_SCREW3_MOM_SIZE
#define DYN2B_TWIST3_SIZE       DYN2B_SCREW3_SIZE

// Wrench: linear-before-angular
#define DYN2B_WRENCH3_ANG_OFFSET DYN2B_SCREW3_MOM_OFFSET
#define DYN2B_WRENCH3_ANG_SIZE   DYN2B_SCREW3_MOM_SIZE
#define DYN2B_WRENCH3_LIN_OFFSET DYN2B_SCREW3_DIR_OFFSET
#define DYN2B_WRENCH3_LIN_SIZE   DYN2B_SCREW3_DIR_SIZE
#define DYN2B_WRENCH3_SIZE       DYN2B_SCREW3_SIZE

// Rigid-body inertia: [I, h, m]
// I: 3x3, symmetric
// h: 3x1
// m: 1
#define DYN2B_RBI3_I_LD      3
#define DYN2B_RBI3_I_OFFSET  0
#define DYN2B_RBI3_I_SIZE    9
#define DYN2B_RBI3_H_OFFSET  9
#define DYN2B_RBI3_H_SIZE    3
#define DYN2B_RBI3_M_OFFSET 12
#define DYN2B_RBI3_M_SIZE    1
#define DYN2B_RBI3_SIZE      (DYN2B_RBI3_I_SIZE \
                              + DYN2B_RBI3_H_SIZE \
                              + DYN2B_RBI3_M_SIZE)


#ifdef __cplusplus
}
#endif

#endif
