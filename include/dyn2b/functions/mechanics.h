// SPDX-License-Identifier: LGPL-3.0
#ifndef DYN2B_FUNCTIONS_MECHANICS_H
#define DYN2B_FUNCTIONS_MECHANICS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file mechanics.h
 *
 * This file contains operations that involve:
 * - Rigid-body inertia: the mapping between between the motion space (velocity
 *   and acceleration) and the force space for a single, unconstrained body.
 * - Acceleration: in particular its transformation which differs from the
 *   screw transformation due the velocity-dependency.
 */


/**
 * Transform a screw acceleration twist from a proximal frame \f$P\f$ to a
 * distal frame \f$D\f$. This transformation depends on the velocities of and
 * between the frames.
 *
 * \f[
 * {}^D\ddot{\boldsymbol{x}}_{\mathcal{W},\mathcal{P}}
 * = {}^P\boldsymbol{X}_D^{-1}~
 *     {}^P\boldsymbol{\ddot{x}}_{\mathcal{W},\mathcal{P}}
 *   + {}^D\boldsymbol{\dot{x}}_{\mathcal{W},\mathcal{P}}
 *     \times {}^D\boldsymbol{\dot{x}}_{\mathcal{P},\mathcal{D}}
 * \f]
 *
 * @param[in] x Pose \f${}^P\boldsymbol{X}_D\f$ of proximal frame \f$\{P\}\f$
 *              with respect to distal frame \f$\{D\}\f$.
 *              Size: \f$[3 \times 3 + 3 \times 1]\f$.
 * @param[in] xd_abs Screw velocity twist
 *                   \f${}^D\dot{\boldsymbol{x}}_{\mathcal{W},\mathcal{P}}\f$ as
 *                   seen by distal frame \f$\{D\}\f$.
 *                   Size: \f$[6 \times 1]\f$.
 * @param[in] xd_rel Screw velocity twist
 *                   \f${}^D\dot{\boldsymbol{x}}_{\mathcal{P},\mathcal{D}}\f$ as
 *                   seen by distal frame \f$\{D\}\f$.
 *                   Size: \f$[6 \times 1]\f$.
 * @param[in] xdd_prox Screw acceleration twist
 *                     \f${}^P\ddot{\boldsymbol{x}}_{\mathcal{W},\mathcal{P}}\f$
 *                     as seen by proximal frame \f$\{P\}\f$.
 *                     Size: \f$[6 \times 1]\f$.
 * @param[out] xdd_dist Screw acceleration twist
 *                      \f${}^D\ddot{\boldsymbol{x}}_{\mathcal{W},\mathcal{P}}\f$
 *                      as seen by distal frame \f$\{D\}\f$.
 *                      Size: \f$[6 \times 1]\f$.
 */
void dyn2b_tf_dist_acc3(
        const double *restrict x,
        const double *restrict xd_abs,
        const double *restrict xd_rel,
        const double *restrict xdd_prox,
        double *restrict xdd_dist);


/**
 * Map a screw acceleration twist into a wrench with a rigid-body inertia. In
 * the equations of motion
 *
 * \f[
 * {}^D\boldsymbol{w}
 * = {}^D\boldsymbol{I}_\mathcal{D}~
 *     {}^D\ddot{\boldsymbol{x}}_{\mathcal{W},\mathcal{D}}
 *   + [{}^D\dot{\boldsymbol{x}}_{\mathcal{W},\mathcal{D}}]_\times~
 *     {}^D\boldsymbol{I}_\mathcal{D}~
 *     {}^D\dot{\boldsymbol{x}}_{\mathcal{W},\mathcal{D}}
 * \f]
 *
 * this represents the first term on the right-hand side.
 *
 * @param[in] rbi Rigid-body inertia \f${}^D\boldsymbol{I}_\mathcal{D}\f$.
 *                Size: \f$[3 \times 3 + 3 \times 1 + 1]\f$.
 * @param[in] xdd Screw acceleration twist
 *                \f${}^D\ddot{\boldsymbol{x}}_{\mathcal{W},\mathcal{D}}\f$
 *                as seen by frame \f$\{D\}\f$.
 *                Size: \f$[6 \times 1]\f$.
 * @param[out] w Wrench \f${}^D\boldsymbol{w}\f$ as seen by frame \f$\{D\}\f$.
 *               Size: \f$[6 \times 1]\f$.
 */
void dyn2b_rbi_to_wrench3(
        const double *restrict rbi,
        const double *restrict xdd,
        double *restrict w);


/**
 * Compute the velocity-dependent, bias force which originates from a change in
 * the inertia distribution due to the motion of the body. In the equations of
 * motion
 *
 * \f[
 * {}^D\boldsymbol{w}
 * = {}^D\boldsymbol{I}_\mathcal{D}~
 *     {}^D\ddot{\boldsymbol{x}}_{\mathcal{W},\mathcal{D}}
 *   + [{}^D\dot{\boldsymbol{x}}_{\mathcal{W},\mathcal{D}}]_\times~
 *     {}^D\boldsymbol{I}_\mathcal{D}~
 *     {}^D\dot{\boldsymbol{x}}_{\mathcal{W},\mathcal{D}}
 * \f]
 *
 * this represents the second term on the right-hand side.
 *
 * @param[in] rbi Rigid-body inertia \f${}^D\boldsymbol{I}_\mathcal{D}\f$.
 *                Size: \f$[3 \times 3 + 3 \times 1 + 1]\f$.
 * @param[in] xd Screw velocity twist
 *               \f${}^D\dot{\boldsymbol{x}}_{\mathcal{W},\mathcal{D}}\f$ as
 *               seen by frame \f$\{D\}\f$.
 *               Size: \f$[6 \times 1]\f$.
 * @param[out] w Wrench \f${}^D\boldsymbol{w}\f$ as seen by frame \f$\{D\}\f$.
 *               Size: \f$[6 \times 1]\f$.
 */
void dyn2b_nrt_wrench3(
        const double *restrict rbi,
        const double *restrict xd,
        double *restrict w);


#ifdef __cplusplus
}
#endif

#endif
