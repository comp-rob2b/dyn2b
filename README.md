# dyn2b: Composable Building Blocks for Kinematics and Dynamics Solvers

`dyn2b` is a C software library that provides the _building blocks_ (the `2b`, aka. operators or functions) for composing recursive _kinematics and dynamics solvers_ for rigid-body systems of arbitrary topology (serial chains, tree structures or arbitrary graphs). _Composability_ means that the building blocks are ready to be composed; but their concrete composition is left to other tools such as [`kindyngen`](https://github.com/comp-rob2b/kindyngen).

`dyn2b` _includes_ the following features:

* Functions that implement operators on spatial quantities, i.e. poses, screws (to represent velocity, acceleration or force) and inertia, that are required to implement recursive kinematics and dynamics solvers.
* Functions that work with the [compact representation]((docs/conventions.md)) (or tuples) of spatial quantities and a few functions that map from the compact representation to the full matrix representation.
* Where required by solvers, the functions operate on one _or more_ instances of spatial quantities (mostly for the propagation of forces).
* The functions act on 3D spatial quantities (yet, 2D versions of those functions would be in scope of a future extension).
* The library naturally supports acceleration constraints as required for the most complete dynamics solver, the _acceleration-constrained hybrid dynamics_ (ACHD) solver by Popov and Vereshchagin.
* All functions are [_pure_](https://en.wikipedia.org/wiki/Pure_function) to avoid hiding state. This is one pre-condition for composability.
* Built on and compatible with BLAS/LAPACK

`dyn2b` deliberately _excludes_ the following features:

* Implementations of concrete kinematics or dynamics solver algorithms on kinematic chains, as those are very application-dependent and should be contributed by external tools, instead of being implemented in a software library.
* Functions that work with the full matrix representation of spatial quantities, as those are already covered by libraries such as BLAS or LAPACK.
* Functions that implement operators on joint-space quantities (e.g. addition of joint forces or mapping joint forces to joint accelerations) or operators for transmissions (that transmit energy from actuators to links). The reason is that ...
  1. ... the most common type of 1D joints are natively supported by scalar, floating-point operations.
  2. ... there exists a wide range of other joints and transmissions, potentially with multiple representations (e.g. spherical joints with orientations represented as Euler angles or rotation matrices), that warrant their custom software libraries.
* Highly-variable domains that warrant their own software libraries to be composed with the instantaneous kinematics and dynamics solvers for specialized applications:
  - The non-linear effects of stiffness and damping with their wide range of models. This includes joint limits.
  - Numerical integrators to compute velocities and positions from the accelerations that the dynamics solvers produce.
  - Collision detection and collision resolution
  - Controllers and estimators
  - Trajectory generators

# Documentation

Installation instructions are available [here](docs/installation.md).

# Third-party software

* [`CodeCoverage.cmake`](thirdparty/cmake/codecoverage/CodeCoverage.cmake) is licensed under the BSD-3-Clause license and originates from Lars Bilke's project [Additional CMake Modules](https://github.com/bilke/cmake-modules).
* [`FindSphinx.cmake`](thirdparty/cmake/sphinx/FindSphinx.cmake) is licensed under the BSD-3-Clause license and originates from Jeroen Koekkoek's project [Sphinx integration for CMake](https://github.com/k0ekk0ek/cmake-sphinx).
* [`markdown-math-filter.pl`](docs/config/markdown-math-filter.pl) is licensed under the MIT license and originates from the [SpECTRE](https://github.com/sxs-collaboration/spectre) project.

# Acknowledgement

This work is part of a project that has received funding from the European Union's Horizon 2020 research and innovation programme SESAME under grant agreement No 101017258.