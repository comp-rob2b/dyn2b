// SPDX-License-Identifier: LGPL-3.0
#ifndef DYN2B_TYPES_JOINT_H
#define DYN2B_TYPES_JOINT_H

#ifdef __cplusplus
extern "C" {
#endif


// Articulated-body inertia: [I, H, M]
// I: 3x3, symmetric
// H: 3x3
// M: 3x3, symmetric
#define DYN2B_ABI3_I_LD      3
#define DYN2B_ABI3_I_OFFSET  0
#define DYN2B_ABI3_I_SIZE    9
#define DYN2B_ABI3_H_LD      3
#define DYN2B_ABI3_H_OFFSET  9
#define DYN2B_ABI3_H_SIZE    9
#define DYN2B_ABI3_M_LD      3
#define DYN2B_ABI3_M_OFFSET 18
#define DYN2B_ABI3_M_SIZE    9
#define DYN2B_ABI3_SIZE      (DYN2B_ABI3_I_SIZE \
                                + DYN2B_ABI3_H_SIZE \
                                + DYN2B_ABI3_M_SIZE)


#ifdef __cplusplus
}
#endif

#endif
