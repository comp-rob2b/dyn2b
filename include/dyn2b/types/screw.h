// SPDX-License-Identifier: LGPL-3.0
#ifndef DYN2B_TYPES_SCREW_H
#define DYN2B_TYPES_SCREW_H

#ifdef __cplusplus
extern "C" {
#endif


// Pose: angular-before-linear
#define DYN2B_POSE3_ANG_LD      3
#define DYN2B_POSE3_ANG_OFFSET  0
#define DYN2B_POSE3_ANG_SIZE    9
#define DYN2B_POSE3_LIN_OFFSET  9
#define DYN2B_POSE3_LIN_SIZE    3
#define DYN2B_POSE3_SIZE        (DYN2B_POSE3_ANG_SIZE + DYN2B_POSE3_LIN_SIZE)

// Screw: moment-before-direction
#define DYN2B_SCREW3_DIR_OFFSET 0
#define DYN2B_SCREW3_DIR_SIZE   3
#define DYN2B_SCREW3_MOM_OFFSET 3
#define DYN2B_SCREW3_MOM_SIZE   3
#define DYN2B_SCREW3_SIZE       (DYN2B_SCREW3_DIR_SIZE + DYN2B_SCREW3_MOM_SIZE)


#ifdef __cplusplus
}
#endif

#endif
