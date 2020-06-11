# Geometry & Intersection

This library provides geometry primitives and intersection checks for calculating gantry paths. It's somewhat limited in capability, but should do everything we need.

To run the tests, use `make test`. To build it for use, use `make all RELEASE=TRUE`.

This requires the Boost Quaternion library. To run tests, the Boost unit test library is also required.


-----


## Types

The following types are defined:

### `Vec3`

A 3D position and direction vector.

- It has three `double` fields: `x`, `y`, `z`.
- The following operators are overloaded:
    - `Vec3 + Vec3`, `Vec3 - Vec3`: vector addition and subtraction
    - `-Vec3`: unary negation
    - `double * Vec3`: scalar multiplication
    - `Vec3 * Vec3`: dot product
    - `ostream << Vec3`: printing convinience
- It has the following static constructors (could be constexpr if we could use newer C++):
    - `Vec3::zero()`: the zero vector
    - `Vec3::basis_x()`: the vector (1, 0, 0)
    - `Vec3::basis_y()`: the vector (0, 1, 0)
    - `Vec3::basis_z()`: the vector (0, 0, 1)
- It has the following functions defined:
    - `norm : Vec3 -> double`: vector norm
    - `cross : Vec3 -> Vec3 -> Vec3`: cross product

### `LineSegment`

Represents a line segment between two position vectors.

- It has two `Vec3` fields, `a`, `b`.

### `Quaternion`

Represents a quaternion. Used to represent 3d rotations.

- It has four `double` fields, `w`, `x`, `y`, `z`
- `w` is the real part, the rest are the vector parts
- The following operators are defined:
    - `Quaternion + Quaternion`: elementwise addition
    - `Quaternion - Quaternion`: elementwise subtraction
    - `Quaternion * Quaternion`: Hamilton product
    - `Quaternion / Quaternion`: Division w.r.t Hamilton product
- The following functions are defined:
    - `scalar : Quaternion -> double`: the scalar component w
    - `vector : Quaternion -> Vec3`: the vector component (x, y, z)
    - `norm : Quaternion -> double`: the norm
    - `inverse : Quaternion -> Quaternion`: the inverse of the function (w.r.t. the Hamilton product)
    - `conjugate : Quaternion -> Quaternion`: the quaternion conjugate

### `Prism`

Represents a rectangular prism

- It has the following fields:
    - One `Vec3` field: `center`, the centroid of the prism.
    - Three `double` fields: `ex`, `ey`, `ez`, the extent in local coordinates in each direction.
    - One `Angle` field, the direction of the prism.
- `Sphere`: represents a sphere.
  - It has the following fields:
    - One `Vec3` field: `center`, the center of the sphere.
    - One `double` field: `r`, the radius.
- `Cylinder`: represents a finite cylinder.
  - It has the following fields:
    - One `Vec3` field: `center`, the center of the sphere.
    - Two double fields: `r`, the radius; and `e`, the extent along local $z$.
    - One `Angle` field, the direction.


----


## Intersections

The following intersections are defined:


- `Vec3` ⟺ `Prism`
  - This tests if the position vector `Vec3` is inside the prism.
  - Algorithm:
    - Find coordinates of position vector in the space of the prism.
    - Check that each dimension is smaller than the prisms extent in that dimension.
- `Prism` ⟺ `Prism`
  - Algorithm:
    - Check if bounding spheres of prisms intersect. If not, return false.
    - _currently disabled:_ check if either of the prisms contains any of the other's points. _need to benchmark if this gives speedup_
    - Do a full test using the Separation of Axes Theorem.
      - This has been optimized from the version of the algorithm for arbitrary convex polyhedra. See comments for information.
- `Prism` ⟺ `Sphere`
  - Algorithm:
    - For each line segment on the prism, find the closest point to the sphere.
    - If any point's distance is less than the radius, return true.
    - Return false.
- `Prism` ⟺ `Cylinder`
  - **NOTE**: this check will **fail** in the case that the cylinder is internal to the prism. Since this will never happen for us, I didn't think it was worth fixing.
    - For example: consider the case with a prism centred at the origin with extent 1 in every direction, and a cylinder at the origin with radius 0.5 and extent 1. They do intersect, but the algorithm fails.
  - **NOTE**: this purposefully does not check for intersections with the top or bottom of a cylinder, only the side.
  - Algorithm:
    - Find all line segments of the prism, and transform to cylinder's frame.
    - For each segment, see if there is a solution where the line segment intersects the circle of the cylinder (anywhere in $z$).
      - If it does, if the resulting $z$ coordinate doesn't exceed the extent of the cylinder, return true.s
    - If none do, return false.
