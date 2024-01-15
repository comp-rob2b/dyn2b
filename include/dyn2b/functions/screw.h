// SPDX-License-Identifier: LGPL-3.0
#ifndef DYN2B_FUNCTIONS_SCREW_H
#define DYN2B_FUNCTIONS_SCREW_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file screw.h
 *
 * This file contains operations that involve 3D screws:
 * - Rotations
 * - Transformations
 * - Screw dot product
 * - Screw cross product
 */


/**
 * Compose two 3D poses.
 *
 * \f[
 * {}^W\boldsymbol{X}_D = {}^W\boldsymbol{X}_P {}^P\boldsymbol{X}_D
 * \f]
 *
 * Due to the compact representation only the following orientation and position
 * is computed:
 *
 * \f{eqnarray*}{
 *   {}^W\boldsymbol{R}_D     &=& {}^W\boldsymbol{R}_P {}^P\boldsymbol{R}_D \\
 *   {}^W\boldsymbol{r}^{w,d} &=& {}^W\boldsymbol{r}^{w,p}
 *                              + {}^W\boldsymbol{R}_P {}^P\boldsymbol{r}^{p,d}
 * \f}
 *
 * @param[in] x_prox The proximal pose \f${}^W\boldsymbol{X}_P\f$.
 *                   Size: \f$[3 \times 3 + 3 \times 1]\f$.
 * @param[in] x_dist The distal pose \f${}^P\boldsymbol{X}_D\f$.
 *                   Size: \f$[3 \times 3 + 3 \times 1]\f$.
 * @param[out] x_comp The composite pose \f${}^W\boldsymbol{X}_D\f$.
 *                    Size: \f$[3 \times 3 + 3 \times 1]\f$.
*/
void dyn2b_cmp_pose3(
        const double *restrict x_prox,
        const double *restrict x_dist,
        double *restrict x_comp);


/**
 * Compute the dot product between two collections of 3D screws.
 *
 * \f[
 *   \boldsymbol{s}^* \cdot \boldsymbol{s} =
 *   \begin{pmatrix}
 *     \boldsymbol{m}^* \\
 *     \boldsymbol{d}^*
 *   \end{pmatrix}
 *   \cdot
 *   \begin{pmatrix}
 *     \boldsymbol{m} \\
 *     \boldsymbol{d}
 *   \end{pmatrix}
 *   =
 *   \begin{pmatrix}
 *     \boldsymbol{m}^* \cdot \boldsymbol{d} \\
 *     \boldsymbol{d}^* \cdot \boldsymbol{m}
 *   \end{pmatrix}
 * \f]
 *
 * This dot product is also known as (natural) pairing, spatial scalar product
 * or reciprocal product. One of the screws must originate from the motion space
 * (position, velocity or acceleration) whereas the other screw must originate
 * from the force space (momentum or force). The resulting scalars are known as:
 *
 * - Work: \f$W = \boldsymbol{w} \cdot d\boldsymbol{x}\f$
 * - Power: \f$P = \boldsymbol{w} \cdot \dot{\boldsymbol{x}}\f$
 * - Acceleration energy: \f$E_a = \boldsymbol{w} \cdot \ddot{\boldsymbol{x}}\f$
 *
 * If there is a single screw and a single dual screw (this is the most familiar
 * situation), the result is indeed a single scalar. However, this function
 * supports multiple instances of screws and dual screws. Hence, the result is a
 * matrix that represents how much work/power/acceleration energy each dual
 * screw (from the force space) produces against each screw (from the motion
 * space).
 *
 * @param[in] m Number of dual screws.
 * @param[in] n Number of screws.
 * @param[in] s_dual The dual screw \f${}^D\boldsymbol{s}^*\f$ as seen by frame
 *                   \f$\{D\}\f$.
 *                   Size: \f$[6 \times m]\f$.
 * @param[in] s The screw \f${}^D\boldsymbol{s}\f$ as seen by frame \f$\{D\}\f$.
 *              Size: \f$[6 \times n]\f$.
 * @param[out] out The resulting dot product.
 *                 Size: \f$[m \times n]\f$.
 */
void dyn2b_dot_screw3(
        int m,
        int n,
        const double *restrict s_dual,
        const double *restrict s,
        double *restrict out);


/**
 * Compute the cross product between two 3D screws.
 *
 * \f[
 *   \boldsymbol{s}_1 \times \boldsymbol{s}_2 =
 *   \begin{pmatrix}
 *     \boldsymbol{m}_1 \\
 *     \boldsymbol{d}_1
 *   \end{pmatrix}
 *   \times
 *   \begin{pmatrix}
 *     \boldsymbol{m}_2 \\
 *     \boldsymbol{d}_2
 *   \end{pmatrix}
 *   =
 *   \begin{pmatrix}
 *     \boldsymbol{m}_1 \times \boldsymbol{m}_2 \\
 *     \boldsymbol{m}_1 \times \boldsymbol{d}_2
 *       + \boldsymbol{d}_1 \times \boldsymbol{m}_2
 *   \end{pmatrix}
 * \f]
 *
 * This cross product is also known as spatial cross product or motor product.
 * The first screw should originate from the motion space whereas the latter
 * screw can either represent an element from the motion space or from the force
 * space. The result is known as the bias acceleration (if both screws represent
 * velocities) or bias force (if the first screw represents a velocity and the
 * second screw represents a momentum wrench)
 *
 * @param[in] s1 The screw \f${}^D\boldsymbol{s}_1\f$ as seen by frame
 *               \f$\{D\}\f$.
 *               Size: \f$[6 \times 1]\f$.
 * @param[in] s2 The screw \f${}^D\boldsymbol{s}_2\f$ as seen by frame
 *               \f$\{D\}\f$.
 *               Size: \f$[6 \times 1]\f$.
 * @param[out] out The resulting screw as seen by frame \f$\{D\}\f$.
 *                 Size: \f$[6 \times 1]\f$.
 */
void dyn2b_crs_screw3(
        const double *restrict s1,
        const double *restrict s2,
        double *restrict out);


/**
 * "Cross-add" operation for 3D screws. It computes the cross product of two
 * screws and adds the result to a third screw. The operation is performed
 * out-of-place, i.e. the output is stored in a different location than any of
 * the inputs.
 *
 * \f[
 *   \boldsymbol{s}_1 + \boldsymbol{s}_2 \times \boldsymbol{s}_3 =
 *   \begin{pmatrix}
 *     \boldsymbol{m}_1 \\
 *     \boldsymbol{d}_1
 *   \end{pmatrix}
 *   +
 *   \begin{pmatrix}
 *     \boldsymbol{m}_2 \\
 *     \boldsymbol{d}_2
 *   \end{pmatrix}
 *   \times
 *   \begin{pmatrix}
 *     \boldsymbol{m}_3 \\
 *     \boldsymbol{d}_3
 *   \end{pmatrix}
 *   =
 *   \begin{pmatrix}
 *     \boldsymbol{m}_1 \\
 *     \boldsymbol{d}_1
 *   \end{pmatrix}
 *   +
 *   \begin{pmatrix}
 *     \boldsymbol{m}_2 \times \boldsymbol{m}_3 \\
 *     \boldsymbol{m}_2 \times \boldsymbol{d}_3
 *       + \boldsymbol{d}_2 \times \boldsymbol{m}_3
 *   \end{pmatrix}
 * \f]
 *
 * @param[in] s1 The screw \f${}^D\boldsymbol{s}_1\f$ as seen by frame
 *               \f$\{D\}\f$.
 *               Size: \f$[6 \times 1]\f$.
 * @param[in] s2 The screw \f${}^D\boldsymbol{s}_2\f$ as seen by frame
 *               \f$\{D\}\f$.
 *               Size: \f$[6 \times 1]\f$.
 * @param[in] s3 The screw \f${}^D\boldsymbol{s}_3\f$ as seen by frame
 *               \f$\{D\}\f$.
 *               Size: \f$[6 \times 1]\f$.
 * @param[out] out The resulting screw as seen by frame \f$\{D\}\f$.
 *                 Size: \f$[6 \times 1]\f$.
 */
void dyn2b_cad_screw3(
        const double *restrict s1,
        const double *restrict s2,
        const double *restrict s3,
        double *restrict out);


/**
 * Rotate a collection of 3D screws from an orientation's proximal frame to the
 * orientation's distal frame.
 *
 * \f[
 * {}^D\boldsymbol{s} = {}^P\boldsymbol{R}_D^{-1}~{}^P\boldsymbol{s}
 * \f]
 * 
 * \f[
 * \begin{pmatrix}
 *   {}^D\boldsymbol{d} \\ {}^D\boldsymbol{m}
 * \end{pmatrix}
 * =
 * \begin{pmatrix}
 *   {}^P\boldsymbol{R}_D^T & \boldsymbol{0} \\
 *   \boldsymbol{0} & {}^P\boldsymbol{R}_D^T
 * \end{pmatrix}
 * \begin{pmatrix}
 *   {}^P\boldsymbol{d} \\ {}^P\boldsymbol{m}
 * \end{pmatrix}
 * \f]
 *
 * @param[in] n Number of screws to transform.
 * @param[in] r The orientation \f${}^P\boldsymbol{R}_D\f$ of proximal frame
 *              \f$\{P\}\f$ with respect to distal frame \f$\{D\}\f$.
 *              Size: \f$[3 \times 3]\f$.
 * @param[in] s_prox Screw \f${}^P\boldsymbol{s}\f$ as seen by proximal frame
 *                   \f$\{P\}\f$.
 *                   Size: \f$[6 \times n]\f$.
 * @param[out] s_dist Screw \f${}^D\boldsymbol{s}\f$ as seen by distal frame
 *                    \f$\{D\}\f$.
 *                    Size: \f$[6 \times n]\f$.
 */
void dyn2b_rot_dist_screw3(
        int n,
        const double *restrict r,
        const double *restrict s_prox,
        double *restrict s_dist);


/**
 * Transform a collection of 3D screws from a pose's proximal frame to the
 * pose's distal frame.
 *
 * \f[
 * {}^D\boldsymbol{s} = {}^P\boldsymbol{X}_D^{-1}~{}^P\boldsymbol{s}
 * \f]
 * 
 * \f[
 * \begin{pmatrix}
 *   {}^D\boldsymbol{d} \\ {}^D\boldsymbol{m}
 * \end{pmatrix}
 * =
 * \begin{pmatrix}
 *    {}^P\boldsymbol{R}_D^T
 *      & \boldsymbol{0} \\
 *   -{}^P\boldsymbol{R}_D^T~\left[{}^P\boldsymbol{r}^{p,d}\right]_\times
 *      & {}^P\boldsymbol{R}_D^T
 * \end{pmatrix}
 * \begin{pmatrix}
 *   {}^P\boldsymbol{d} \\ {}^P\boldsymbol{m}
 * \end{pmatrix}
 * \f]
 *
 * @param[in] n Number of screws to transform.
 * @param[in] x The pose \f${}^P\boldsymbol{X}_D\f$ of proximal frame
 *              \f$\{P\}\f$ with respect to distal frame \f$\{D\}\f$.
 *              Size: \f$[3 \times 3 + 3 \times 1]\f$.
 * @param[in] s_prox Screw \f${}^P\boldsymbol{s}\f$ as seen by proximal frame
 *                   \f$\{P\}\f$.
 *                   Size: \f$[6 \times n]\f$.
 * @param[out] s_dist Screw \f${}^D\boldsymbol{s}\f$ as seen by distal frame
 *                    \f$\{D\}\f$.
 *                    Size: \f$[6 \times n]\f$.
 */
void dyn2b_tf_dist_screw3(
        int n,
        const double *restrict x,
        const double *restrict s_prox,
        double *restrict s_dist);


/**
 * Rotate a collection of 3D screws from an orientation's distal frame to the
 * orientation's proximal frame.
 *
 * \f[
 * {}^P\boldsymbol{s} = {}^P\boldsymbol{R}_D~{}^D\boldsymbol{s}
 * \f]
 *
 * \f[
 * \begin{pmatrix}
 *   {}^P\boldsymbol{d} \\ {}^P\boldsymbol{m}
 * \end{pmatrix}
 * =
 * \begin{pmatrix}
 *   {}^P\boldsymbol{R}_D & \boldsymbol{0} \\
 *   \boldsymbol{0}_D & {}^P\boldsymbol{R}_D
 * \end{pmatrix}
 * \begin{pmatrix}
 *   {}^D\boldsymbol{d} \\ {}^D\boldsymbol{m}
 * \end{pmatrix}
 * \f]
 *
 * @param[in] n Number of screws to transform.
 * @param[in] r The pose \f${}^P\boldsymbol{X}_D\f$ of proximal frame
 *              \f$\{P\}\f$ with respect to distal frame \f$\{D\}\f$.
 *              Size: \f$[3 \times 3 + 3 \times 1]\f$.
 * @param[in] s_dist Screw \f${}^D\boldsymbol{s}\f$ as seen by distal frame
 *                   \f$\{D\}\f$.
 *                   Size: \f$[6 \times n]\f$.
 * @param[out] s_prox Screw \f${}^P\boldsymbol{s}\f$ as seen by proximal frame
 *                    \f$\{P\}\f$.
 *                    Size: \f$[6 \times n]\f$.
 */
void dyn2b_rot_prox_screw3(
        int n,
        const double *restrict r,
        const double *restrict s_dist,
        double *restrict s_prox);


/**
 * Transform a collection of 3D screws from a pose's distal frame to the pose's
 * proximal frame.
 *
 * \f[
 * {}^P\boldsymbol{s} = {}^P\boldsymbol{X}_D~{}^D\boldsymbol{s}
 * \f]
 *
 * \f[
 * \begin{pmatrix}
 *   {}^P\boldsymbol{d} \\ {}^P\boldsymbol{m}
 * \end{pmatrix}
 * =
 * \begin{pmatrix}
 *   {}^P\boldsymbol{R}_D
 *      & \boldsymbol{0} \\
 *   \left[{}^P\boldsymbol{r}^{p,d}\right]_\times~{}^P\boldsymbol{R}_D
 *      & {}^P\boldsymbol{R}_D
 * \end{pmatrix}
 * \begin{pmatrix}
 *   {}^D\boldsymbol{d} \\ {}^D\boldsymbol{m}
 * \end{pmatrix}
 * \f]
 *
 * @param[in] n Number of screws to transform.
 * @param[in] x The pose \f${}^P\boldsymbol{X}_D\f$ of proximal frame
 *              \f$\{P\}\f$ with respect to distal frame \f$\{D\}\f$.
 *              Size: \f$[3 \times 3 + 3 \times 1]\f$.
 * @param[in] s_dist Screw \f${}^D\boldsymbol{s}\f$ as seen by distal frame
 *                   \f$\{D\}\f$.
 *                   Size: \f$[6 \times n]\f$.
 * @param[out] s_prox Screw \f${}^P\boldsymbol{s}\f$ as seen by proximal frame
 *                    \f$\{P\}\f$.
 *                    Size: \f$[6 \times n]\f$.
 */
void dyn2b_tf_prox_screw3(
        int n,
        const double *restrict x,
        const double *restrict s_dist,
        double *restrict s_prox);


#ifdef __cplusplus
}
#endif

#endif
