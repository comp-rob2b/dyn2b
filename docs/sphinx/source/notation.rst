Symbols and notation
====================

Spatial entities
----------------

* Bodies: :math:`\mathcal{A}`, :math:`\mathcal{B}`, ...
* Frames (*are* bodies): :math:`\{A\}`, :math:`\{B\}`, ...
* Point: :math:`\boldsymbol{p}`

  - ... as seen by frame :math:`\{A\}`: :math:`{}^A\boldsymbol{p}`

* Direction vector: :math:`\boldsymbol{d}`

  - ... as seen by frame :math:`\{A\}`: :math:`{}^A\boldsymbol{d}`

* Moment (of some quantity about a point e.g. moment in a screw, linear velocity in a twist, force in a screw): :math:`\boldsymbol{m}`

  - ... measured about reference point :math:`a`: :math:`{}_a\boldsymbol{m}`
  - ... as seen by frame :math:`\{A\}`: :math:`{}_a^A\boldsymbol{m}`

* Screw: :math:`\boldsymbol{s}`

  - ... measured about reference point :math:`a`: :math:`{}_a\boldsymbol{s}`
  - ... as seen by frame :math:`\{A\}`: :math:`{}_a^A\boldsymbol{s}`

Position quantities
-------------------

* Orientation matrix: :math:`\boldsymbol{R}`

  - ... of frame :math:`\{B\}` with respect to frame :math:`\{A\}`: :math:`{}^A\boldsymbol{R}_B`

* (Linear) position vector: :math:`\boldsymbol{r}`

  - ... of point :math:`b` with respect to point :math:`a`: :math:`\boldsymbol{r}^{a,b}`
  - ... as seen by frame :math:`\{A\}`: :math:`{}^A\boldsymbol{r}^{a,b}`

* Homogeneous transformation matrix: :math:`\boldsymbol{T}`

  - ... of frame :math:`\{B\}` with respect to frame :math:`\{A\}`: :math:`{}^A\boldsymbol{T}_B`

* Screw/Plücker transformation matrix: :math:`\boldsymbol{X}`

  - ... of frame :math:`\{B\}` with respect to frame :math:`\{A\}`: :math:`{}^A\boldsymbol{X}_B`

* Joint position: :math:`\boldsymbol{q}`

Velocity quantities
-------------------

* Angular velocity vector: :math:`\boldsymbol{\omega}`

  - ... of body :math:`\mathcal{B}` with respect to body :math:`\mathcal{A}`: :math:`\boldsymbol{\omega}_{\mathcal{A},\mathcal{B}}`
  - ... as seen by frame :math:`\{C\}`: :math:`{}^C\boldsymbol{\omega}_{\mathcal{A},\mathcal{B}}`

* Linear velocity vector: :math:`\boldsymbol{v}`

  - ... of body :math:`\mathcal{B}` with respect to body :math:`\mathcal{A}`: :math:`\boldsymbol{v}_{\mathcal{A},\mathcal{B}}`
  - ... measured at reference point :math:`c`: :math:`{}_c\boldsymbol{v}_{\mathcal{A},\mathcal{B}}`
  - ... as seen by frame :math:`\{C\}`: :math:`{}_c^C\boldsymbol{v}_{\mathcal{A},\mathcal{B}}`

* Velocity twist vector: :math:`\dot{\boldsymbol{x}}`

  - ... of body :math:`\mathcal{B}` with respect to body :math:`\mathcal{A}`: :math:`\dot{\boldsymbol{x}}_{\mathcal{A},\mathcal{B}}`
  - ... measured at reference point :math:`c`: :math:`{}_c\dot{\boldsymbol{x}}_{\mathcal{A},\mathcal{B}}`
  - ... as seen by frame :math:`\{C\}`: :math:`{}_c^C\dot{\boldsymbol{x}}_{\mathcal{A},\mathcal{B}}`

* Joint velocity: :math:`\dot{\boldsymbol{q}}`

Acceleration quantities
-----------------------

* Angular acceleration vector: :math:`\dot{\boldsymbol{\omega}}`

  - ... of body :math:`\mathcal{B}` with respect to body :math:`\mathcal{A}`: :math:`\dot{\boldsymbol{\omega}}_{\mathcal{A},\mathcal{B}}`
  - ... as seen by frame :math:`\{C\}`: :math:`{}^C\dot{\boldsymbol{\omega}}_{\mathcal{A},\mathcal{B}}`

* Linear acceleration vector: :math:`\dot{\boldsymbol{v}}`

  - ... of body :math:`\mathcal{B}` with respect to body :math:`\mathcal{A}`: :math:`\dot{\boldsymbol{v}}_{\mathcal{A},\mathcal{B}}`
  - ... measured at reference point :math:`c`: :math:`{}_c\dot{\boldsymbol{v}}_{\mathcal{A},\mathcal{B}}`
  - ... as seen by frame :math:`\{C\}`: :math:`{}_c^C\dot{\boldsymbol{v}}_{\mathcal{A},\mathcal{B}}`

* Acceleration twist vector: :math:`\ddot{\boldsymbol{x}}`

  - ... of body :math:`\mathcal{B}` with respect to body :math:`\mathcal{A}`: :math:`\ddot{\boldsymbol{x}}_{\mathcal{A},\mathcal{B}}`
  - ... measured at reference point :math:`c`::math:`{}_c\ddot{\boldsymbol{x}}_{\mathcal{A},\mathcal{B}}`
  - ... as seen by frame :math:`\{C\}`: :math:`{}_c^C\ddot{\boldsymbol{x}}_{\mathcal{A},\mathcal{B}}`
* Special "types" of acceleration:

  - Spatial acceleration/screw acceleration twist vector: :math:`\ddot{\boldsymbol{x}}^s`
  - Material acceleration/pose acceleration twist vector: :math:`\ddot{\boldsymbol{x}}^m`
  - Bias acceleration: :math:`\ddot{\boldsymbol{x}}^b`
* Joint acceleration: :math:`\ddot{\boldsymbol{q}}`

Force quantities
----------------

* Moment of force/torque vector: :math:`\boldsymbol{m}`

  - ... applied at moment action point :math:`c`: :math:`{}_c\boldsymbol{m}`
  - ... as seen by frame :math:`\{C\}`: :math:`{}_c^C\boldsymbol{m}`

* (Linear) force vector: :math:`\boldsymbol{f}`

  - ... as seen by frame :math:`\{C\}`: :math:`{}^C\boldsymbol{f}`

* Wrench vector: :math:`\boldsymbol{w}`

  - ... applied at moment action point :math:`c`: :math:`{}_c\boldsymbol{w}`
  - ... as seen by frame :math:`\{C\}`: :math:`{}_c^C\boldsymbol{w}`

* Joint force: :math:`\boldsymbol{\tau}`

Inertia quantities
------------------

* (Articulated-body) rotational inertia matrix: :math:`\bar{\boldsymbol{I}}`

  - ... of body :math:`\mathcal{B}`: :math:`\bar{\boldsymbol{I}}_\mathcal{B}`
  - ... measured about reference point :math:`c`: :math:`{}_c\bar{\boldsymbol{I}}_\mathcal{B}`
  - ... as seen by frame :math:`\{C\}`: :math:`{}_c^C\bar{\boldsymbol{I}}_\mathcal{B}`

* Moment of mass vector: :math:`\boldsymbol{h} = m \cdot \boldsymbol{r}^{o,c}` (product of mass and vector from the origin of the body's reference frame :math:`o` to the centre of mass point :math:`c`)

  - ... of body :math:`\mathcal{B}`: :math:`\boldsymbol{h}_\mathcal{B}`
  - ... measured about reference point :math:`o`: :math:`{}_o\boldsymbol{h}_\mathcal{B}`
  - ... as seen by frame :math:`\{C\}`: :math:`{}_o^C\boldsymbol{h}_\mathcal{B}`

* Mass scalar: :math:`m`

  - ... of body :math:`\mathcal{B}`: :math:`m_\mathcal{B}`

* Rigid-body inertia matrix: :math:`\boldsymbol{I}`

  - ... of body :math:`\mathcal{B}`: :math:`\boldsymbol{I}_\mathcal{B}`
  - ... measured about reference point :math:`c`: :math:`{}_c\boldsymbol{I}_\mathcal{B}`
  - ... as seen by frame :math:`\{C\}`: :math:`{}_c^C\boldsymbol{I}_\mathcal{B}`

* Articulated-body moment of mass matrix: :math:`\boldsymbol{H}`

  - ... of body :math:`\mathcal{B}`: :math:`\boldsymbol{H}_\mathcal{B}`
  - ... measured about reference point :math:`c`: :math:`{}_c\boldsymbol{H}_\mathcal{B}`
  - ... as seen by frame :math:`\{C\}`: :math:`{}_c^C\boldsymbol{H}_\mathcal{B}`

* Articulated-body mass matrix: :math:`\boldsymbol{M}`

  - ... of body :math:`\mathcal{B}`: :math:`\boldsymbol{M}_\mathcal{B}`
  - ... measured about reference point :math:`c`: :math:`{}_c\boldsymbol{M}_\mathcal{B}`
  - ... as seen by frame :math:`\{C\}`: :math:`{}_c^C\boldsymbol{M}_\mathcal{B}`

* Articulated-body inertia matrix: :math:`\boldsymbol{I}^A`

  - ... of body :math:`\mathcal{B}`: :math:`\boldsymbol{I}_\mathcal{B}^A`
  - ... measured about reference point :math:`c`: :math:`{}_c\boldsymbol{I}_\mathcal{B}^A`
  - ... as seen by frame :math:`\{C\}`: :math:`{}_c^C\boldsymbol{I}_\mathcal{B}^A`

Operators
---------

.. note::
   Several of the above quantities *are* also operators, e.g. force or inertia.

* Polar vector, skew-symmetric matrix: :math:`[\cdot]_\times`
* Motion projection matrix: :math:`\boldsymbol{P}`
* Jacobian matrix: :math:`\boldsymbol{J}`, :math:`\boldsymbol{Z}`

Duality
-------

Dual quantities or operators are denoted by :math:`\boldsymbol{(\cdot)^T}`

* Force transformation matrix: :math:`\boldsymbol{X}^T`
* Inertia/wrench projection matrix: :math:`\boldsymbol{P}^T`
* Jacobian matrix: :math:`\boldsymbol{J}^T`, :math:`\boldsymbol{Z}^T`