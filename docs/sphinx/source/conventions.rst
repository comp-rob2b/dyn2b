Conventions and choices
***********************

Orientation representation
==========================

``dyn2b`` represents orientations :math:`\boldsymbol{R}` as direction cosine matrices where each *column* represents a basis vector. The orientation :math:`{}^P\boldsymbol{R}_D` of a distal frame :math:`\{D\}` with respect to a proximal frame :math:`\{P\}` describes how to rotate frame :math:`\{D\}` to make it coincide with frame :math:`\{P\}`. This is the same convention as used in [Craig2005]_ and the inverse of [Featherstone2008]_'s convention. Hence, the direction cosine matrices for a rotation of an agle :math:`\theta` around the :math:`x`, :math:`y` and :math:`z` axis are, respectively:

.. math::
   \boldsymbol{R}_x(\theta) =
   \begin{pmatrix}
     1 &       0      &        0      \\
     0 & \cos(\theta) & -\sin(\theta) \\
     0 & \sin(\theta) &  \cos(\theta)
   \end{pmatrix}

.. math::
   \boldsymbol{R}_y(\theta) =
   \begin{pmatrix}
      \cos(\theta) & 0 & \sin(\theta) \\
            0      & 1 &       0      \\
     -\sin(\theta) & 0 & \cos(\theta)
   \end{pmatrix}

.. math::
   \boldsymbol{R}_z(\theta) =
   \begin{pmatrix}
     \cos(\theta) & -\sin(\theta) & 0 \\
     \sin(\theta) &  \cos(\theta) & 0 \\
           0      &        0      & 1
   \end{pmatrix}


Reference points and coordinate frames
======================================

* For all quantities that depend on the choice of a reference point, ``dyn2b`` chooses the origin of the selected coordinate frame as the reference point. For motion vectors, the resulting quantities are also known by the non-standardized name of *screw twists* (as opposed to *pose twists*)
* For accelerations, ``dyn2b`` chooses the *Eulerian* or *ordinary* time derivative. That means, it keeps a reference point fixed in space and compares the velocities of (i) *different* particles at (ii) *different* instants in time at (iii) that *same* reference point, to compute the acceleration. This representation is also known by the non-standardizes names of *spatial acceleration* or *screw acceleration twist* (as opposed to *pose acceleration twists*, *Lagrangian derivatives* or *material derivatives*).
* For all functions that map between joint space and Cartesian space, ``dyn2b`` chooses the joint's more distal (or "successor") frame as the coordinate frame.


Order of arguments
==================

Many quantities in rigid-body dynamics are composed of angular and linear parts that can be arranged in an arbitrary order. In ``dyn2b`` we have made the following choices for coordinate vectors and the compact representation of matrices.


Coordinate vectors
------------------

* Screw: direction before moment

  - :math:`\boldsymbol{s} = [\boldsymbol{d}, \boldsymbol{m}]`

* Twist: angular before linear

  - :math:`\dot{\boldsymbol{x}} = [\boldsymbol{\omega}, \boldsymbol{v}]`
  - :math:`\ddot{\boldsymbol{x}} = [\dot{\boldsymbol{\omega}}, \dot{\boldsymbol{v}}]`

* Wrench: linear before angular

  - :math:`\boldsymbol{w} = [\boldsymbol{f}, \boldsymbol{\tau}]`


Compact representation of matrices as tuples
--------------------------------------------

.. note::
   ``dyn2b`` always uses the compact representation of matrices as tuples unless otherwise stated.

* Homogeneous transformation matrix: angular before linear

  - :math:`\boldsymbol{T} = [\boldsymbol{R}, \boldsymbol{r}]`

* Plücker transformation matrix: angular before linear

  - :math:`\boldsymbol{X} = [\boldsymbol{R}, \boldsymbol{r}]`

* Rigid-body inertia: angular before coupling and linear

  - :math:`\boldsymbol{I} = [\bar{\boldsymbol{I}}, \boldsymbol{h}, m]`

* Articulated-body inertia: angular before coupling and linear

  - :math:`\boldsymbol{I}^A = [\bar{\boldsymbol{I}}, \boldsymbol{H}, \boldsymbol{M}]`


Digital data representation
===========================

The digital data representation determines how data is arranged in a computer's memory. In ``dyn2b`` ...

* ... all physical quantities are represented via C's ``double`` type.
* ... matrices are stored in column-major order. This ensures that the vectors that represent the matrix (i.e. its columns) remain contiguous in memory.
* ... function parameters are assumed to *not* alias as indicated by the `restrict <https://en.cppreference.com/w/c/language/restrict>`_ keyword. This is meant to facilitate future performance improvements, especially via `auto vectorization <https://en.wikipedia.org/wiki/Automatic_vectorization>`_.


Agnostic about
==============

Here, "agnostic" means that the provided *input* data should be consistent. Hence, ``dyn2b`` does not care about the particular choice of:

* Handedness of frames, either right-handed or left-handed
* Physical units of quantities
* Choice of reference frame's location, either world frame (spatial quantities) or body-fixed frame (body quantities)
* Interpretation of transformations as being active or passive