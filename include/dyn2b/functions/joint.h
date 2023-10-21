// SPDX-License-Identifier: LGPL-3.0
#ifndef DYN2B_FUNCTIONS_JOINT_H
#define DYN2B_FUNCTIONS_JOINT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file joint.h
 *
 * This file contains operations that involve motion constraints (or joints)
 * between two or more bodies:
 * - Forward position/velocity/acceleration kinematics
 * - Inverse force kinematics
 * - Articulated-body inertia
 * - Projection of wrench/ABI over a joint (the "dynamic" joint in forward
 *   dynamics problems as opposed to the "quasi-static" joint in the Recursive
 *   Newton-Euler inverse dynamics)
 */



/**
 * Compute the forward position kinematics of a revolute-x joint.
 *
 * `cart = fpk(jnt)`
 *
 * @param[in] jnt The joint position measured in radians.
 *                Size: \f$[1 \times 1]\f$
 * @param[out] cart The pose of the joint's distal frame \f$\{D\}\f$ with
 *                  respect to the joint's proximal frame \f$\{P\}\f$.
 *                  Size: \f$[3 \times 3 + 3 \times 1]\f$
 */
void dyn2b_rev_x_to_pose3(
        const double *restrict jnt,
        double *restrict cart);


/**
 * Compute the forward position kinematics of a revolute-y joint.
 *
 * `cart = fpk(jnt)`
 *
 * @param[in] jnt The joint position measured in radians.
 *                Size: \f$[1 \times 1]\f$
 * @param[out] cart The pose of the joint's distal frame \f$\{D\}\f$ with
 *                  respect to the joint's proximal frame \f$\{P\}\f$.
 *                  Size: \f$[3 \times 3 + 3 \times 1]\f$
 */
void dyn2b_rev_y_to_pose3(
        const double *restrict jnt,
        double *restrict cart);


/**
 * Compute the forward position kinematics of a revolute-z joint.
 *
 * `cart = fpk(jnt)`
 *
 * @param[in] jnt The joint position measured in radians.
 *                Size: \f$[1 \times 1]\f$
 * @param[out] cart The pose of the joint's distal frame \f$\{D\}\f$ with
 *                  respect to the joint's proximal frame \f$\{P\}\f$.
 *                  Size: \f$[3 \times 3 + 3 \times 1]\f$
 */
void dyn2b_rev_z_to_pose3(
        const double *restrict jnt,
        double *restrict cart);


/**
 * Compute the forward position kinematics of a prismatic-x joint.
 *
 * `cart = fpk(jnt)`
 *
 * @param[in] jnt The joint position measured in radians.
 *                Size: \f$[1 \times 1]\f$
 * @param[out] cart The pose of the joint's distal frame \f$\{D\}\f$ with
 *                  respect to the joint's proximal frame \f$\{P\}\f$.
 *                  Size: \f$[3 \times 3 + 3 \times 1]\f$
 */
void dyn2b_trans_x_to_pose3(
        const double *restrict jnt,
        double *restrict cart);


/**
 * Compute the forward position kinematics of a prismatic-y joint.
 *
 * `cart = fpk(jnt)`
 *
 * @param[in] jnt The joint position measured in radians.
 *                Size: \f$[1 \times 1]\f$
 * @param[out] cart The pose of the joint's distal frame \f$\{D\}\f$ with
 *                  respect to the joint's proximal frame \f$\{P\}\f$.
 *                  Size: \f$[3 \times 3 + 3 \times 1]\f$
 */
void dyn2b_trans_y_to_pose3(
        const double *restrict jnt,
        double *restrict cart);


/**
 * Compute the forward position kinematics of a prismatic-z joint.
 *
 * `cart = fpk(jnt)`
 *
 * @param[in] jnt The joint position measured in radians.
 *                Size: \f$[1 \times 1]\f$
 * @param[out] cart The pose of the joint's distal frame \f$\{D\}\f$ with
 *                  respect to the joint's proximal frame \f$\{P\}\f$.
 *                  Size: \f$[3 \times 3 + 3 \times 1]\f$
 */
void dyn2b_trans_z_to_pose3(
        const double *restrict jnt,
        double *restrict cart);


/**
 * Compute the velocity or acceleration twist for a revolute-x joint.
 *
 * `cart = fvk(jnt)` or `cart = fak(jnt)`
 *
 * @param[in] jnt The joint velocity or acceleration.
 *                Size: \f$[1 \times 1]\f$
 * @param[out] cart The velocity or acceleration twist of the joint's distal
 *                  body \f$\mathcal{D}\f$ with respect to the joint's proximal
 *                  body \f$\mathcal{P}\f$ as seen by the joint's distal frame
 *                  \f$\{D\}\f$ and that frame's origin \f$d\f$ as the reference
 *                  point.
 *                  Size: \f$[6 \times 1]\f$
 */
void dyn2b_rev_x_to_twist3(
        const double *restrict jnt,
        double *restrict cart);


/**
 * Compute the velocity or acceleration twist for a revolute-y joint.
 *
 * `cart = fvk(jnt)` or `cart = fak(jnt)`
 *
 * @param[in] jnt The joint velocity or acceleration.
 *                Size: \f$[1 \times 1]\f$
 * @param[out] cart The velocity or acceleration twist of the joint's distal
 *                  body \f$\mathcal{D}\f$ with respect to the joint's proximal
 *                  body \f$\mathcal{P}\f$ as seen by the joint's distal frame
 *                  \f$\{D\}\f$ and that frame's origin \f$d\f$ as the reference
 *                  point.
 *                  Size: \f$[6 \times 1]\f$
 */
void dyn2b_rev_y_to_twist3(
        const double *restrict jnt,
        double *restrict cart);


/**
 * Compute the velocity or acceleration twist for a revolute-z joint.
 *
 * `cart = fvk(jnt)` or `cart = fak(jnt)`
 *
 * @param[in] jnt The joint velocity or acceleration.
 *                Size: \f$[1 \times 1]\f$
 * @param[out] cart The velocity or acceleration twist of the joint's distal
 *                  body \f$\mathcal{D}\f$ with respect to the joint's proximal
 *                  body \f$\mathcal{P}\f$ as seen by the joint's distal frame
 *                  \f$\{D\}\f$ and that frame's origin \f$d\f$ as the reference
 *                  point.
 *                  Size: \f$[6 \times 1]\f$
 */
void dyn2b_rev_z_to_twist3(
        const double *restrict jnt,
        double *restrict cart);


/**
 * Compute the velocity or acceleration twist for a prismatic-x joint.
 *
 * `cart = fvk(jnt)` or `cart = fak(jnt)`
 *
 * @param[in] jnt The joint velocity or acceleration.
 *                Size: \f$[1 \times 1]\f$
 * @param[out] cart The velocity or acceleration twist of the joint's distal
 *                  body \f$\mathcal{D}\f$ with respect to the joint's proximal
 *                  body \f$\mathcal{P}\f$ as seen by the joint's distal frame
 *                  \f$\{D\}\f$ and that frame's origin \f$d\f$ as the reference
 *                  point.
 *                  Size: \f$[6 \times 1]\f$
 */
void dyn2b_trans_x_to_twist3(
        const double *restrict jnt,
        double *restrict cart);


/**
 * Compute the velocity or acceleration twist for a prismatic-y joint.
 *
 * `cart = fvk(jnt)` or `cart = fak(jnt)`
 *
 * @param[in] jnt The joint velocity or acceleration.
 *                Size: \f$[1 \times 1]\f$
 * @param[out] cart The velocity or acceleration twist of the joint's distal
 *                  body \f$\mathcal{D}\f$ with respect to the joint's proximal
 *                  body \f$\mathcal{P}\f$ as seen by the joint's distal frame
 *                  \f$\{D\}\f$ and that frame's origin \f$d\f$ as the reference
 *                  point.
 *                  Size: \f$[6 \times 1]\f$
 */
void dyn2b_trans_y_to_twist3(
        const double *restrict jnt,
        double *restrict cart);


/**
 * Compute the velocity or acceleration twist for a prismatic-z joint.
 *
 * `cart = fvk(jnt)` or `cart = fak(jnt)`
 *
 * @param[in] jnt The joint velocity or acceleration.
 *                Size: \f$[1 \times 1]\f$
 * @param[out] cart The velocity or acceleration twist of the joint's distal
 *                  body \f$\mathcal{D}\f$ with respect to the joint's proximal
 *                  body \f$\mathcal{P}\f$ as seen by the joint's distal frame
 *                  \f$\{D\}\f$ and that frame's origin \f$d\f$ as the reference
 *                  point.
 *                  Size: \f$[6 \times 1]\f$
 */
void dyn2b_trans_z_to_twist3(
        const double *restrict jnt,
        double *restrict cart);


/**
 * Compute joint torques from a collection of wrenches for a revolute-x joint.
 *
 * `jnt = ifk(cart)`
 *
 * @param[in] n Number of wrenches to transform.
 * @param[in] cart The wrench applied to the joint's distal body
 *                 \f$\mathcal{D}\f$ as seen by the joint's distal frame
 *                 \f$\{D\}\f$ and that frame's origin \f$d\f$ as the reference
 *                 point.
 *                 Size: \f$[6 \times n]\f$
 * @param[out] jnt The joint force.
 *                 Size: \f$[1 \times n]\f$
 */
void dyn2b_rev_x_from_wrench3(
        int n,
        const double *restrict cart,
        double *restrict jnt);


/**
 * Compute joint torques from a collection of wrenches for a revolute-y joint.
 *
 * `jnt = ifk(cart)`
 *
 * @param[in] n Number of wrenches to transform.
 * @param[in] cart The wrench applied to the joint's distal body
 *                 \f$\mathcal{D}\f$ as seen by the joint's distal frame
 *                 \f$\{D\}\f$ and that frame's origin \f$d\f$ as the reference
 *                 point.
 *                 Size: \f$[6 \times n]\f$
 * @param[out] jnt The joint force.
 *                 Size: \f$[1 \times n]\f$
 */
void dyn2b_rev_y_from_wrench3(
        int n,
        const double *restrict cart,
        double *restrict jnt);


/**
 * Compute joint torques from a collection of wrenches for a revolute-z joint.
 *
 * `jnt = ifk(cart)`
 *
 * @param[in] n Number of wrenches to transform.
 * @param[in] cart The wrench applied to the joint's distal body
 *                 \f$\mathcal{D}\f$ as seen by the joint's distal frame
 *                 \f$\{D\}\f$ and that frame's origin \f$d\f$ as the reference
 *                 point.
 *                 Size: \f$[6 \times n]\f$
 * @param[out] jnt The joint force.
 *                 Size: \f$[1 \times n]\f$
 */
void dyn2b_rev_z_from_wrench3(
        int n,
        const double *restrict cart,
        double *restrict jnt);


/**
 * Compute joint torques from a collection of wrenches for a prismatic-x joint.
 *
 * `jnt = ifk(cart)`
 *
 * @param[in] n Number of wrenches to transform.
 * @param[in] cart The wrench applied to the joint's distal body
 *                 \f$\mathcal{D}\f$ as seen by the joint's distal frame
 *                 \f$\{D\}\f$ and that frame's origin \f$d\f$ as the reference
 *                 point.
 *                 Size: \f$[6 \times n]\f$
 * @param[out] jnt The joint force.
 *                 Size: \f$[1 \times n]\f$
 */
void dyn2b_trans_x_from_wrench3(
        int n,
        const double *restrict cart,
        double *restrict jnt);


/**
 * Compute joint torques from a collection of wrenches for a prismatic-y joint.
 *
 * `jnt = ifk(cart)`
 *
 * @param[in] n Number of wrenches to transform.
 * @param[in] cart The wrench applied to the joint's distal body
 *                 \f$\mathcal{D}\f$ as seen by the joint's distal frame
 *                 \f$\{D\}\f$ and that frame's origin \f$d\f$ as the reference
 *                 point.
 *                 Size: \f$[6 \times n]\f$
 * @param[out] jnt The joint force.
 *                 Size: \f$[1 \times n]\f$
 */
void dyn2b_trans_y_from_wrench3(
        int n,
        const double *restrict cart,
        double *restrict jnt);


/**
 * Compute joint torques from a collection of wrenches for a prismatic-z joint.
 *
 * `jnt = ifk(cart)`
 *
 * @param[in] n Number of wrenches to transform.
 * @param[in] cart The wrench applied to the joint's distal body
 *                 \f$\mathcal{D}\f$ as seen by the joint's distal frame
 *                 \f$\{D\}\f$ and that frame's origin \f$d\f$ as the reference
 *                 point.
 *                 Size: \f$[6 \times n]\f$
 * @param[out] jnt The joint force.
 *                 Size: \f$[1 \times n]\f$
 */
void dyn2b_trans_z_from_wrench3(
        int n,
        const double *restrict cart,
        double *restrict jnt);


/**
 * Initialize articulated-body inertia from rigid-body inertia.
 *
 * \f[
 * (\bar{\boldsymbol{I}}, \boldsymbol{h}, m)
 * \mapsto
 * (\bar{\boldsymbol{I}}, [\boldsymbol{h}]_\times, m\boldsymbol{I})
 * \f]
 *
 * @param[in] rbi Rigid-body inertia
 *                \f$(\bar{\boldsymbol{I}}, \boldsymbol{h}, m)\f$.
 *                Size: \f$[3 \times 3 + 3 \times 1 + 1]\f$.
 * @param[out] abi Articulated-body inertia
 *                 \f$(\bar{\boldsymbol{I}}, \boldsymbol{H}, \boldsymbol{M})\f$.
 *                 Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 */
void dyn2b_to_abi3(
        const double *restrict rbi,
        double *restrict abi);


/**
 * Transform articulated-body inertia from a distal frame \f$D\f$ to a proximal
 * frame \f$P\f$.
 *
 * \f[
 * {}^P\boldsymbol{I}^A
 * = {}^P\boldsymbol{X}_D~{}^D\boldsymbol{I}^A~{}^P\boldsymbol{X}_D^{-1}
 * \f]
 *
 * \f[
 * \begin{pmatrix}
 *   {}^P\bar{\boldsymbol{I}} & {}^P\boldsymbol{H} \\
 *   {}^P\boldsymbol{H}^T     & {}^P\boldsymbol{M}
 * \end{pmatrix}
 * =
 * \begin{pmatrix}
 *   {}^P\boldsymbol{R}_D                                              & \boldsymbol{0}       \\
 *   \left[{}^P\boldsymbol{r}^{p,d}\right]_\times~{}^P\boldsymbol{R}_D & {}^P\boldsymbol{R}_D
 * \end{pmatrix}
 * \begin{pmatrix}
 *   {}^D\bar{\boldsymbol{I}} & {}^D\boldsymbol{H} \\
 *   {}^D\boldsymbol{H}^T     & {}^D\boldsymbol{M}
 * \end{pmatrix}
 * \begin{pmatrix}
 *    {}^P\boldsymbol{R}_D^T                                              & \boldsymbol{0}         \\
 *   -{}^P\boldsymbol{R}_D^T~\left[{}^P\boldsymbol{r}^{p,d}\right]_\times & {}^P\boldsymbol{R}_D^T
 * \end{pmatrix}
 * \f]
 *
 * @param[in] x Screw transformation \f${}^D\boldsymbol{X}_P\f$ of distal frame
 *              \f$\{D\}\f$ with respect to proximal frame \f$\{P\}\f$.
 *              Size: \f$[3 \times 3 + 3 \times 1]\f$.
 * @param[in] abi_dist Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$ as
 *                     seen by distal frame \f$\{D\}\f$.
 *                     Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[out] abi_prox Articulated-body inertia \f${}^P\boldsymbol{I}^A\f$ as
 *                      seen by distal frame \f$\{P\}\f$.
 *                      Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 */
void dyn2b_tf_prox_abi3(
        const double *restrict x,
        const double *restrict abi_dist,
        double *restrict abi_prox);


/**
 * Map a collection of screw acceleration twists into a collection of wrenches
 * using an articulated-body inertia.
 *
 * \f[
 * {}^D\boldsymbol{w}
 * = {}^D\boldsymbol{I}^A~{}^D\ddot{\boldsymbol{x}}_{\mathcal{W},\mathcal{D}}
 * \f]
 *
 * @param[in] n The number screw acceleration twists to map.
 * @param[in] abi Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$.
 *                Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[in] xdd Screw acceleration twist
 *                \f${}^D\ddot{\boldsymbol{x}}_{\mathcal{W},\mathcal{D}}\f$
 *                as seen by frame \f$\{C\}\f$.
 *                Size: \f$[6 \times n]\f$.
 * @param[out] w Wrench \f${}^D\boldsymbol{w}\f$ as seen by frame \f$\{D\}\f$.
 *               Size: \f$[6 \times n]\f$.
 */
void dyn2b_abi_to_wrench3(
        int n,
        const double *restrict abi,
        const double *restrict xdd,
        double *restrict w);


/**
 * Project an articulated-body inertia over a revolute-x joint. This results in
 * the so-called apparent inertia: after the projection (on the other side of
 * the joint) the articulated-body inertia seems to reduce because of the joint.
 *
 * \f[
 * {}^D\boldsymbol{I}^a = \boldsymbol{P}^T~{}^D\boldsymbol{I}^A
 * \f]
 *
 * with
 *
 * \f[
 * \boldsymbol{P}^T
 * = \boldsymbol{1} - {}^D\boldsymbol{I}^A~
 *   {}^D\boldsymbol{S}~D^{-1}~{}^D\boldsymbol{S}^T
 * \f]
 *
 * and
 *
 * \f[
 * D = d + {}^D\boldsymbol{S}^T~{}^D\boldsymbol{I}^A~
 *         {}^D\boldsymbol{S}
 * \f]
 *
 * where \f$\boldsymbol{S}\f$ is the joint's Jacobian matrix (or motion
 * sub-space matrix) and \f$d\f$ is the inertia that the joint feels from the
 * actuator, possibly through a gearbox.
 *
 * @param[in] d The joint inertia \f$d\f$.
 *              Size: \f$[1 \times 1]\f$.
 * @param[in] m_in Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$ of the
 *                 joint's distal sub-tree as seen by the joint's distal frame
 *                 \f$\{D\}\f$.
 *                 Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[out] m_out Apparent inertia \f${}^D\boldsymbol{I}^a\f$ of the joint's
 *                   distal sub-tree as seen by the joint's distal frame
 *                   \f$\{D\}\f$.
 *                   Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 */
void dyn2b_rev_x_proj_abi3(
        const double *restrict d,
        const double *restrict m_in,
        double *restrict m_out);


/**
 * Project an articulated-body inertia over a revolute-y joint. This results in
 * the so-called apparent inertia: after the projection (on the other side of
 * the joint) the articulated-body inertia seems to reduce because of the joint.
 *
 * \f[
 * {}^D\boldsymbol{I}^a = \boldsymbol{P}^T~{}^D\boldsymbol{I}^A
 * \f]
 *
 * with
 *
 * \f[
 * \boldsymbol{P}^T
 * = \boldsymbol{1} - {}^D\boldsymbol{I}^A~
 *   {}^D\boldsymbol{S}~D^{-1}~{}^D\boldsymbol{S}^T
 * \f]
 *
 * and
 *
 * \f[
 * D = d + {}^D\boldsymbol{S}^T~{}^D\boldsymbol{I}^A~
 *         {}^D\boldsymbol{S}
 * \f]
 *
 * where \f$\boldsymbol{S}\f$ is the joint's Jacobian matrix (or motion
 * sub-space matrix) and \f$d\f$ is the inertia that the joint feels from the
 * actuator, possibly through a gearbox.
 *
 * @param[in] d The joint inertia \f$d\f$.
 *              Size: \f$[1 \times 1]\f$.
 * @param[in] m_in Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$ of the
 *                 joint's distal sub-tree as seen by the joint's distal frame
 *                 \f$\{D\}\f$.
 *                 Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[out] m_out Apparent inertia \f${}^D\boldsymbol{I}^a\f$ of the joint's
 *                   distal sub-tree as seen by the joint's distal frame
 *                   \f$\{D\}\f$.
 *                   Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 */
void dyn2b_rev_y_proj_abi3(
        const double *restrict d,
        const double *restrict m_in,
        double *restrict m_out);


/**
 * Project an articulated-body inertia over a revolute-z joint. This results in
 * the so-called apparent inertia: after the projection (on the other side of
 * the joint) the articulated-body inertia seems to reduce because of the joint.
 *
 * \f[
 * {}^D\boldsymbol{I}^a = \boldsymbol{P}^T~{}^D\boldsymbol{I}^A
 * \f]
 *
 * with
 *
 * \f[
 * \boldsymbol{P}^T
 * = \boldsymbol{1} - {}^D\boldsymbol{I}^A~
 *   {}^D\boldsymbol{S}~D^{-1}~{}^D\boldsymbol{S}^T
 * \f]
 *
 * and
 *
 * \f[
 * D = d + {}^D\boldsymbol{S}^T~{}^D\boldsymbol{I}^A~
 *         {}^D\boldsymbol{S}
 * \f]
 *
 * where \f$\boldsymbol{S}\f$ is the joint's Jacobian matrix (or motion
 * sub-space matrix) and \f$d\f$ is the inertia that the joint feels from the
 * actuator, possibly through a gearbox.
 *
 * @param[in] d The joint inertia \f$d\f$.
 *              Size: \f$[1 \times 1]\f$.
 * @param[in] m_in Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$ of the
 *                 joint's distal sub-tree as seen by the joint's distal frame
 *                 \f$\{D\}\f$.
 *                 Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[out] m_out Apparent inertia \f${}^D\boldsymbol{I}^a\f$ of the joint's
 *                   distal sub-tree as seen by the joint's distal frame
 *                   \f$\{D\}\f$.
 *                   Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 */
void dyn2b_rev_z_proj_abi3(
        const double *restrict d,
        const double *restrict m_in,
        double *restrict m_out);


/**
 * Project an articulated-body inertia over a prismatic-x joint. This results in
 * the so-called apparent inertia: after the projection (on the other side of
 * the joint) the articulated-body inertia seems to reduce because of the joint.
 *
 * \f[
 * {}^D\boldsymbol{I}^a = \boldsymbol{P}^T~{}^D\boldsymbol{I}^A
 * \f]
 *
 * with
 *
 * \f[
 * \boldsymbol{P}^T
 * = \boldsymbol{1} - {}^D\boldsymbol{I}^A~
 *   {}^D\boldsymbol{S}~D^{-1}~{}^D\boldsymbol{S}^T
 * \f]
 *
 * and
 *
 * \f[
 * D = d + {}^D\boldsymbol{S}^T~{}^D\boldsymbol{I}^A~
 *         {}^D\boldsymbol{S}
 * \f]
 *
 * where \f$\boldsymbol{S}\f$ is the joint's Jacobian matrix (or motion
 * sub-space matrix) and \f$d\f$ is the inertia that the joint feels from the
 * actuator, possibly through a gearbox.
 *
 * @param[in] d The joint inertia \f$d\f$.
 *              Size: \f$[1 \times 1]\f$.
 * @param[in] m_in Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$ of the
 *                 joint's distal sub-tree as seen by the joint's distal frame
 *                 \f$\{D\}\f$.
 *                 Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[out] m_out Apparent inertia \f${}^D\boldsymbol{I}^a\f$ of the joint's
 *                   distal sub-tree as seen by the joint's distal frame
 *                   \f$\{D\}\f$.
 *                   Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 */
void dyn2b_trans_x_proj_abi3(
        const double *restrict d,
        const double *restrict m_in,
        double *restrict m_out);


/**
 * Project an articulated-body inertia over a prismatic-y joint. This results in
 * the so-called apparent inertia: after the projection (on the other side of
 * the joint) the articulated-body inertia seems to reduce because of the joint.
 *
 * \f[
 * {}^D\boldsymbol{I}^a = \boldsymbol{P}^T~{}^D\boldsymbol{I}^A
 * \f]
 *
 * with
 *
 * \f[
 * \boldsymbol{P}^T
 * = \boldsymbol{1} - {}^D\boldsymbol{I}^A~
 *   {}^D\boldsymbol{S}~D^{-1}~{}^D\boldsymbol{S}^T
 * \f]
 *
 * and
 *
 * \f[
 * D = d + {}^D\boldsymbol{S}^T~{}^D\boldsymbol{I}^A~
 *         {}^D\boldsymbol{S}
 * \f]
 *
 * where \f$\boldsymbol{S}\f$ is the joint's Jacobian matrix (or motion
 * sub-space matrix) and \f$d\f$ is the inertia that the joint feels from the
 * actuator, possibly through a gearbox.
 *
 * @param[in] d The joint inertia \f$d\f$.
 *              Size: \f$[1 \times 1]\f$.
 * @param[in] m_in Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$ of the
 *                 joint's distal sub-tree as seen by the joint's distal frame
 *                 \f$\{D\}\f$.
 *                 Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[out] m_out Apparent inertia \f${}^D\boldsymbol{I}^a\f$ of the joint's
 *                   distal sub-tree as seen by the joint's distal frame
 *                   \f$\{D\}\f$.
 *                   Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 */
void dyn2b_trans_y_proj_abi3(
        const double *restrict d,
        const double *restrict m_in,
        double *restrict m_out);


/**
 * Project an articulated-body inertia over a prismatic-z joint. This results in
 * the so-called apparent inertia: after the projection (on the other side of
 * the joint) the articulated-body inertia seems to reduce because of the joint.
 *
 * \f[
 * {}^D\boldsymbol{I}^a = \boldsymbol{P}^T~{}^D\boldsymbol{I}^A
 * \f]
 *
 * with
 *
 * \f[
 * \boldsymbol{P}^T
 * = \boldsymbol{1} - {}^D\boldsymbol{I}^A~
 *   {}^D\boldsymbol{S}~D^{-1}~{}^D\boldsymbol{S}^T
 * \f]
 *
 * and
 *
 * \f[
 * D = d + {}^D\boldsymbol{S}^T~{}^D\boldsymbol{I}^A~{}^D\boldsymbol{S}
 * \f]
 *
 * where \f$\boldsymbol{S}\f$ is the joint's Jacobian matrix (or motion
 * sub-space matrix) and \f$d\f$ is the inertia that the joint feels from the
 * actuator, possibly through a gearbox.
 *
 * @param[in] d The joint inertia \f$d\f$.
 *              Size: \f$[1 \times 1]\f$.
 * @param[in] m_in Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$ of the
 *                 joint's distal sub-tree as seen by the joint's distal frame
 *                 \f$\{D\}\f$.
 *                 Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[out] m_out Apparent inertia \f${}^D\boldsymbol{I}^a\f$ of the joint's
 *                   distal sub-tree as seen by the joint's distal frame
 *                   \f$\{D\}\f$.
 *                   Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 */
void dyn2b_trans_z_proj_abi3(
        const double *restrict d,
        const double *restrict m_in,
        double *restrict m_out);


/**
 * Project a collection of articulated-body wrenches over a revolute-x joint.
 * This results in the so-called apparent wrench: the projection represents how
 * much force is used to accelerate the joint.
 *
 * \f[
 * {}^D\boldsymbol{w}^a = \boldsymbol{P}^T~{}^D\boldsymbol{w}^A
 * \f]
 *
 * with
 *
 * \f[
 * \boldsymbol{P}^T
 * = \boldsymbol{1} - {}^D\boldsymbol{I}^A~
 *   {}^D\boldsymbol{S}~D^{-1}~{}^D\boldsymbol{S}^T
 * \f]
 *
 * and
 *
 * \f[
 * D = d + {}^D\boldsymbol{S}^T~{}^D\boldsymbol{I}^A~{}^D\boldsymbol{S}
 * \f]
 *
 * where \f$\boldsymbol{S}\f$ is the joint's Jacobian matrix (or motion
 * sub-space matrix) and \f$d\f$ is the inertia that the joint feels from the
 * actuator, possibly through a gearbox.
 *
 * @param[in] n Number of wrenches to project.
 * @param[in] d The joint inertia \f$d\f$.
 *              Size: \f$[1 \times 1]\f$.
 * @param[in] m Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$ of the
 *              joint's distal sub-tree as seen by the joint's distal frame
 *              \f$\{D\}\f$.
 *              Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[in] f_in Wrench \f${}^D\boldsymbol{w}^A\f$ of the joint's distal
 *                 sub-tree as seen by the joint's distal frame \f$\{D\}\f$.
 *                 Size: \f$[6 \times n]\f$.
 * @param[out] f_out Apparent wrench \f${}^D\boldsymbol{w}^a\f$
 *                   of the joint's distal sub-tree as seen by the joint's
 *                   distal frame \f$\{D\}\f$.
 *                   Size: \f$[6 \times n]\f$.
 */
void dyn2b_rev_x_proj_wrench3(
        int n,
        const double *restrict d,
        const double *restrict m,
        const double *restrict f_in,
        double *restrict f_out);


/**
 * Project a collection of articulated-body wrenches over a revolute-y joint.
 * This results in the so-called apparent wrench: the projection represents how
 * much force is used to accelerate the joint.
 *
 * \f[
 * {}^D\boldsymbol{w}^a = \boldsymbol{P}^T~{}^D\boldsymbol{w}^A
 * \f]
 *
 * with
 *
 * \f[
 * \boldsymbol{P}^T
 * = \boldsymbol{1} - {}^D\boldsymbol{I}^A~
 *   {}^D\boldsymbol{S}~D^{-1}~{}^D\boldsymbol{S}^T
 * \f]
 *
 * and
 *
 * \f[
 * D = d + {}^D\boldsymbol{S}^T~{}^D\boldsymbol{I}^A~{}^D\boldsymbol{S}
 * \f]
 *
 * where \f$\boldsymbol{S}\f$ is the joint's Jacobian matrix (or motion
 * sub-space matrix) and \f$d\f$ is the inertia that the joint feels from the
 * actuator, possibly through a gearbox.
 *
 * @param[in] n Number of wrenches to project.
 * @param[in] d The joint inertia \f$d\f$.
 *              Size: \f$[1 \times 1]\f$.
 * @param[in] m Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$ of the
 *              joint's distal sub-tree as seen by the joint's distal frame
 *              \f$\{D\}\f$.
 *              Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[in] f_in Wrench \f${}^D\boldsymbol{w}^A\f$ of the joint's distal
 *                 sub-tree as seen by the joint's distal frame \f$\{D\}\f$.
 *                 Size: \f$[6 \times n]\f$.
 * @param[out] f_out Apparent wrench \f${}^D\boldsymbol{w}^a\f$
 *                   of the joint's distal sub-tree as seen by the joint's
 *                   distal frame \f$\{D\}\f$.
 *                   Size: \f$[6 \times n]\f$.
 */
void dyn2b_rev_y_proj_wrench3(
        int n,
        const double *restrict d,
        const double *restrict m,
        const double *restrict f_in,
        double *restrict f_out);


/**
 * Project a collection of articulated-body wrenches over a revolute-x joint.
 * This results in the so-called apparent wrench: the projection represents how
 * much force is used to accelerate the joint.
 *
 * \f[
 * {}^D\boldsymbol{w}^a = \boldsymbol{P}^T~{}^D\boldsymbol{w}^A
 * \f]
 *
 * with
 *
 * \f[
 * \boldsymbol{P}^T
 * = \boldsymbol{1} - {}^D\boldsymbol{I}^A~
 *   {}^D\boldsymbol{S}~D^{-1}~{}^D\boldsymbol{S}^T
 * \f]
 *
 * and
 *
 * \f[
 * D = d + {}^D\boldsymbol{S}^T~{}^D\boldsymbol{I}^A~{}^D\boldsymbol{S}
 * \f]
 *
 * where \f$\boldsymbol{S}\f$ is the joint's Jacobian matrix (or motion
 * sub-space matrix) and \f$d\f$ is the inertia that the joint feels from the
 * actuator, possibly through a gearbox.
 *
 * @param[in] n Number of wrenches to project.
 * @param[in] d The joint inertia \f$d\f$.
 *              Size: \f$[1 \times 1]\f$.
 * @param[in] m Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$ of the
 *              joint's distal sub-tree as seen by the joint's distal frame
 *              \f$\{D\}\f$.
 *              Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[in] f_in Wrench \f${}^D\boldsymbol{w}^A\f$ of the joint's distal
 *                 sub-tree as seen by the joint's distal frame \f$\{D\}\f$.
 *                 Size: \f$[6 \times n]\f$.
 * @param[out] f_out Apparent wrench \f${}^D\boldsymbol{w}^a\f$
 *                   of the joint's distal sub-tree as seen by the joint's
 *                   distal frame \f$\{D\}\f$.
 *                   Size: \f$[6 \times n]\f$.
 */
void dyn2b_rev_z_proj_wrench3(
        int n,
        const double *restrict d,
        const double *restrict m,
        const double *restrict f_in,
        double *restrict f_out);


/**
 * Project a collection of articulated-body wrenches over a prismatic-x joint.
 * This results in the so-called apparent wrench: the projection represents how
 * much force is used to accelerate the joint.
 *
 * \f[
 * {}^D\boldsymbol{w}^a = \boldsymbol{P}^T~{}^D\boldsymbol{w}^A
 * \f]
 *
 * with
 *
 * \f[
 * \boldsymbol{P}^T
 * = \boldsymbol{1} - {}^D\boldsymbol{I}^A~
 *   {}^D\boldsymbol{S}~D^{-1}~{}^D\boldsymbol{S}^T
 * \f]
 *
 * and
 *
 * \f[
 * D = d + {}^D\boldsymbol{S}^T~{}^D\boldsymbol{I}^A~{}^D\boldsymbol{S}
 * \f]
 *
 * where \f$\boldsymbol{S}\f$ is the joint's Jacobian matrix (or motion
 * sub-space matrix) and \f$d\f$ is the inertia that the joint feels from the
 * actuator, possibly through a gearbox.
 *
 * @param[in] n Number of wrenches to project.
 * @param[in] d The joint inertia \f$d\f$.
 *              Size: \f$[1 \times 1]\f$.
 * @param[in] m Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$ of the
 *              joint's distal sub-tree as seen by the joint's distal frame
 *              \f$\{D\}\f$.
 *              Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[in] f_in Wrench \f${}^D\boldsymbol{w}^A\f$ of the joint's distal
 *                 sub-tree as seen by the joint's distal frame \f$\{D\}\f$.
 *                 Size: \f$[6 \times n]\f$.
 * @param[out] f_out Apparent wrench \f${}^D\boldsymbol{w}^a\f$
 *                   of the joint's distal sub-tree as seen by the joint's
 *                   distal frame \f$\{D\}\f$.
 *                   Size: \f$[6 \times n]\f$.
 */
void dyn2b_trans_x_proj_wrench3(
        int n,
        const double *restrict d,
        const double *restrict m,
        const double *restrict f_in,
        double *restrict f_out);


/**
 * Project a collection of articulated-body wrenches over a prismatic-y joint.
 * This results in the so-called apparent wrench: the projection represents how
 * much force is used to accelerate the joint.
 *
 * \f[
 * {}^D\boldsymbol{w}^a = \boldsymbol{P}^T~{}^D\boldsymbol{w}^A
 * \f]
 *
 * with
 *
 * \f[
 * \boldsymbol{P}^T
 * = \boldsymbol{1} - {}^D\boldsymbol{I}^A~
 *   {}^D\boldsymbol{S}~D^{-1}~{}^D\boldsymbol{S}^T
 * \f]
 *
 * and
 *
 * \f[
 * D = d + {}^D\boldsymbol{S}^T~{}^D\boldsymbol{I}^A~{}^D\boldsymbol{S}
 * \f]
 *
 * where \f$\boldsymbol{S}\f$ is the joint's Jacobian matrix (or motion
 * sub-space matrix) and \f$d\f$ is the inertia that the joint feels from the
 * actuator, possibly through a gearbox.
 *
 * @param[in] n Number of wrenches to project.
 * @param[in] d The joint inertia \f$d\f$.
 *              Size: \f$[1 \times 1]\f$.
 * @param[in] m Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$ of the
 *              joint's distal sub-tree as seen by the joint's distal frame
 *              \f$\{D\}\f$.
 *              Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[in] f_in Wrench \f${}^D\boldsymbol{w}^A\f$ of the joint's distal
 *                 sub-tree as seen by the joint's distal frame \f$\{D\}\f$.
 *                 Size: \f$[6 \times n]\f$.
 * @param[out] f_out Apparent wrench \f${}^D\boldsymbol{w}^a\f$
 *                   of the joint's distal sub-tree as seen by the joint's
 *                   distal frame \f$\{D\}\f$.
 *                   Size: \f$[6 \times n]\f$.
 */
void dyn2b_trans_y_proj_wrench3(
        int n,
        const double *restrict d,
        const double *restrict m,
        const double *restrict f_in,
        double *restrict f_out);


/**
 * Project a collection of articulated-body wrenches over a prismatic-z joint.
 * This results in the so-called apparent wrench: the projection represents how
 * much force is used to accelerate the joint.
 *
 * \f[
 * {}^D\boldsymbol{w}^a = \boldsymbol{P}^T~{}^D\boldsymbol{w}^A
 * \f]
 *
 * with
 *
 * \f[
 * \boldsymbol{P}^T
 * = \boldsymbol{1} - {}^D\boldsymbol{I}^A~
 *   {}^D\boldsymbol{S}~D^{-1}~{}^D\boldsymbol{S}^T
 * \f]
 *
 * and
 *
 * \f[
 * D = d + {}^D\boldsymbol{S}^T~{}^D\boldsymbol{I}^A~{}^D\boldsymbol{S}
 * \f]
 *
 * where \f$\boldsymbol{S}\f$ is the joint's Jacobian matrix (or motion
 * sub-space matrix) and \f$d\f$ is the inertia that the joint feels from the
 * actuator, possibly through a gearbox.
 *
 * @param[in] n Number of wrenches to project.
 * @param[in] d The joint inertia \f$d\f$.
 *              Size: \f$[1 \times 1]\f$.
 * @param[in] m Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$ of the
 *              joint's distal sub-tree as seen by the joint's distal frame
 *              \f$\{D\}\f$.
 *              Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[in] f_in Wrench \f${}^D\boldsymbol{w}^A\f$ of the joint's distal
 *                 sub-tree as seen by the joint's distal frame \f$\{D\}\f$.
 *                 Size: \f$[6 \times n]\f$.
 * @param[out] f_out Apparent wrench \f${}^D\boldsymbol{w}^a\f$
 *                   of the joint's distal sub-tree as seen by the joint's
 *                   distal frame \f$\{D\}\f$.
 *                   Size: \f$[6 \times n]\f$.
 */
void dyn2b_trans_z_proj_wrench3(
        int n,
        const double *restrict d,
        const double *restrict m,
        const double *restrict f_in,
        double *restrict f_out);


/**
 * Transform a compact articulated-body inertia tuple to an articulated-body
 * inertia matrix.
 *
 * \f[
 * (\bar{\boldsymbol{I}}, \boldsymbol{H}, \boldsymbol{M})
 * \mapsto
 * \begin{pmatrix}
 *   \bar{\boldsymbol{I}} & \boldsymbol{H} \\
 *   \boldsymbol{H}^T     & \boldsymbol{M}
 * \end{pmatrix}
 * \f]
 *
 * @param[in] abi_tup Compact articulated-body inertia tuple
 *                    \f$(\bar{\boldsymbol{I}}, \boldsymbol{H},
 *                    \boldsymbol{M})\f$.
 *                    Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[out] abi_mat Articulated-body inertia matrix \f$I^A\f$.
 *                     Size: \f$[6 \times 6]\f$.
 */
void dyn2b_to_mat_abi3(
        const double *restrict abi_tup,
        double *restrict abi_mat);


/**
 * Transform an articulated-body inertia matrix to a compact articulated-body
 * inertia tuple.
 *
 * \f[
 * \begin{pmatrix}
 *   \bar{\boldsymbol{I}} & \boldsymbol{H} \\
 *   \boldsymbol{H}^T     & \boldsymbol{M}
 * \end{pmatrix}
 * \mapsto
 * (\bar{\boldsymbol{I}}, \boldsymbol{H}, \boldsymbol{M})
 * \f]
 *
 * @param[in] abi_mat Articulated-body inertia matrix \f$I^A\f$.
 *                    Size: \f$[6 \times 6]\f$.
 * @param[out] abi_tup Compact articulated-body inertia tuple
 *                     \f$(\bar{\boldsymbol{I}}, \boldsymbol{H},
 *                     \boldsymbol{M})\f$.
 *                     Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 */
void dyn2b_to_tup_abi3(
        const double *restrict abi_mat,
        double *restrict abi_tup);


/**
 * Explictly compute the inverse of a generic joint's (specified by the joint's
 * Jacobian matrix) inertia. This combines (i) the inertia felt from Cartesian
 * space due to the links; and (ii) the inertia felt from an actuator.
 *
 * \f[
 * D^{-1}
 * = (d + {}^D\boldsymbol{S}^T~{}^D\boldsymbol{I}^A~{}^D\boldsymbol{S})^{-1}
 * \f]
 *
 * where \f$\boldsymbol{S}\f$ is the joint's Jacobian matrix (or motion
 * sub-space matrix) and \f$d\f$ is the inertia that the joint feels from the
 * actuator, possibly through a gearbox.
 *
 * @param[in] dof Number of joint's motion degrees of freedom.
 * @param[in] jac The joint Jacobian (or motion subspace matrix)
 *                \f${}^D\boldsymbol{S}\f$ as seen by the joint's distal frame
 *                \f$\{D\}\f$.
 *                Size: \f$[6 \times \text{dof}]\f$.
 * @param[in] m Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$ of the
 *              joint's distal sub-tree as seen by the joint's distal frame
 *              \f$\{D\}\f$. This inertia is represented as a dense matrix.
 *              Size: \f$[6 \times 6]\f$.
 * @param[in] d The joint inertia \f$d\f$.
 *              Size: \f$[\text{dof} \times \text{dof}]\f$.
 * @param[out] dstms The inverse inertia \f$D^{-1}\f$.
 *                   Size: \f$[\text{dof} \times \text{dof}]\f$.
 */
void dyn2b_jnt_inv_abi3(
        int dof,
        const double *restrict jac,
        const double *restrict m,
        const double *restrict d,
        double *restrict dstms);


/**
 * Compute an explicit projection matrix for a generic joint (specified by the
 * joint's Jacobian matrix). This results in the so-called apparent inertia:
 * after the projection (on the other side of the joint) the articulated-body
 * inertia seems to reduce because of the joint.
 *
 * \f[
 * \boldsymbol{P}^T
 * = \boldsymbol{1} - {}^D\boldsymbol{I}^A~
 *   {}^D\boldsymbol{S}~D^{-1}~{}^D\boldsymbol{S}^T
 * \f]
 *
 * with
 *
 * \f[
 * D = d + {}^D\boldsymbol{S}^T~{}^D\boldsymbol{I}^A~{}^D\boldsymbol{S}
 * \f]
 *
 * where \f$\boldsymbol{S}\f$ is the joint's Jacobian matrix (or motion
 * sub-space matrix) and \f$d\f$ is the inertia that the joint feels from the
 * actuator, possibly through a gearbox.
 *
 * @param[in] dof Number of joint's motion degrees of freedom.
 * @param[in] jac The joint Jacobian (or motion subspace matrix)
 *                \f${}^D\boldsymbol{S}\f$ as seen by the joint's distal frame
 *                \f$\{D\}\f$.
 *                Size: \f$[6 \times \text{dof}]\f$.
 * @param[in] d The joint inertia \f$d\f$.
 *              Size: \f$[\text{dof} \times \text{dof}]\f$.
 * @param[in] m Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$ of the
 *              joint's distal sub-tree as seen by the joint's distal frame
 *              \f$\{D\}\f$.
 *              Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[out] proj The projection matrix \f$\boldsymbol{P}^T\f$.
 *                  Size: \f$[6 \times 6]\f$.
 */
void dyn2b_jnt_to_proj3(
        int dof,
        const double *restrict jac,
        const double *restrict d,
        const double *restrict m,
        double *restrict proj);


/**
 * Project an articulated-body inertia over a generic joint (specified by the
 * joint's Jacobian matrix). This results in the so-called apparent inertia:
 * after the projection (on the other side of the joint) the articulated-body
 * inertia seems to reduce because of the joint.
 *
 * \f[
 * {}^D\boldsymbol{I}^a = \boldsymbol{P}^T~{}^D\boldsymbol{I}^A
 * \f]
 *
 * with
 *
 * \f[
 * \boldsymbol{P}^T
 * = \boldsymbol{1} - {}^D\boldsymbol{I}^A~
 *   {}^D\boldsymbol{S}~D^{-1}~{}^D\boldsymbol{S}^T
 * \f]
 *
 * and
 *
 * \f[
 * D = d + {}^D\boldsymbol{S}^T~{}^D\boldsymbol{I}^A~{}^D\boldsymbol{S}
 * \f]
 *
 * where \f$\boldsymbol{S}\f$ is the joint's Jacobian matrix (or motion
 * sub-space matrix) and \f$d\f$ is the inertia that the joint feels from the
 * actuator, possibly through a gearbox.
 *
 * @param[in] dof Number of joint's motion degrees of freedom.
 * @param[in] jac The joint Jacobian (or motion subspace matrix)
 *                \f${}^D\boldsymbol{S}\f$ as seen by the joint's distal frame
 *                \f$\{D\}\f$.
 *                Size: \f$[6 \times \text{dof}]\f$.
 * @param[in] d The joint inertia \f$d\f$.
 *              Size: \f$[\text{dof} \times \text{dof}]\f$.
 * @param[in] m_in Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$ of the
 *                 joint's distal sub-tree as seen by the joint's distal frame
 *                 \f$\{D\}\f$.
 *                 Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[out] m_out Apparent inertia \f${}^D\boldsymbol{I}^a\f$ of the joint's
 *                   distal sub-tree as seen by the joint's distal frame
 *                   \f$\{D\}\f$.
 *                   Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 */
void dyn2b_jnt_proj_abi3(
        int dof,
        const double *restrict jac,
        const double *restrict d,
        const double *restrict m_in,
        double *restrict m_out);


/**
 * Project a collection of articulated-body wrenches over a generic joint
 * (specified by the joint's Jacobian matrix). This results in the so-called
 * apparent wrench: the projection represents how much force is used to
 * accelerate the joint.
 *
 * \f[
 * {}^D\boldsymbol{w}^a = \boldsymbol{P}^T~{}^D\boldsymbol{w}^A
 * \f]
 *
 * with
 *
 * \f[
 * \boldsymbol{P}^T
 * = \boldsymbol{1} - {}^D\boldsymbol{I}^A~
 *   {}^D\boldsymbol{S}~D^{-1}~{}^D\boldsymbol{S}^T
 * \f]
 *
 * and
 *
 * \f[
 * D = d + {}^D\boldsymbol{S}^T~{}^D\boldsymbol{I}^A~{}^D\boldsymbol{S}
 * \f]
 *
 * where \f$\boldsymbol{S}\f$ is the joint's Jacobian matrix (or motion
 * sub-space matrix) and \f$d\f$ is the inertia that the joint feels from the
 * actuator, possibly through a gearbox.
 *
 * @param[in] n Number of wrenches to project.
 * @param[in] dof Number of joint's motion degrees of freedom.
 * @param[in] jac The joint Jacobian (or motion subspace matrix)
 *                \f${}^D\boldsymbol{S}\f$ as seen by the joint's distal frame
 *                \f$\{D\}\f$.
 *                Size: \f$[6 \times \text{dof}]\f$.
 * @param[in] d The joint inertia \f$d\f$.
 *              Size: \f$[\text{dof} \times \text{dof}]\f$.
 * @param[in] m Articulated-body inertia \f${}^D\boldsymbol{I}^A\f$ of the
 *              joint's distal sub-tree as seen by the joint's distal frame
 *              \f$\{D\}\f$.
 *              Size: \f$[3 \times 3 + 3 \times 3 + 3 \times 3]\f$.
 * @param[in] f_in Wrench \f${}^D\boldsymbol{w}^A\f$ of the joint's distal
 *                 sub-tree as seen by the joint's distal frame \f$\{D\}\f$.
 *                 Size: \f$[6 \times n]\f$.
 * @param[out] f_out Apparent wrench \f${}^D\boldsymbol{w}^a\f$
 *                   of the joint's distal sub-tree as seen by the joint's
 *                   distal frame \f$\{D\}\f$.
 *                   Size: \f$[6 \times n]\f$.
 */
void dyn2b_jnt_proj_wrench3(
        int n,
        int dof,
        const double *restrict jac,
        const double *restrict d,
        const double *restrict m,
        const double *restrict f_in,
        double *restrict f_out);

#ifdef __cplusplus
}
#endif

#endif
