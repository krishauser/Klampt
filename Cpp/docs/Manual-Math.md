# Klamp't Manual: Math concepts

* [API Summary](#api-summary)
* [3D Points and Directions](#3d-points-and-directions)
* [3D Rotations](#3d-rotations)
* [Rigid Transformations](#rigid-transformations)
* [Linear Algebra](#linear-algebra)

Klamp't assumes basic familiarity with 3D geometry and linear algebra concepts. It heavily uses structures that representing vectors, matrices, 3D points, 3D rotations, and 3D transformations. These routines are heavily tested and fast.

The main mathematical objects used in Klampt are as follows:
- `Vector`: a variable-length (n-D) vector.
- `Matrix`: a variable-size (m x n) matrix.
- `Vector3`: a 3-D vector.
- `Matrix3`: a 3x3 matrix.
- `Rotation`: a 3D rotation, specifically an element of the special orthogonal group SO(3), usually represented by a `Matrix3`.
- `RigidTransform`: a rigid transformation `T(x) = R*x + t`, with `R` a rotation and `t` a `Vector3`

![Illustration of concepts](images/concepts-math.png)

## API Summary

Users should become familiar with the definitions in the following files:

- KrisLibrary/math/math.h contains definitions for basic mathematical routines.  `Real` is typedef'ed to `double` and (probably) should not be changed.
- KrisLibrary/math/vector.h contains a `Vector` class (typedef'ed to `VectorTemplate<Real>`).
- KrisLibrary/math/matrix.h contains a `Matrix` class (typedef'ed to `MatrixTemplate<Real>`).
- KrisLibrary/math/angle.h contains functions for interpolating and measuring distances of angles on SO(2).
- KrisLibrary/math3d/primitives.h contains 2D and 3D mathematical primitives. The classes `Vector2`, `Vector3`, `Matrix2`, `Matrix3`, `Matrix4`, `RigidTransform2D` and `RigidTransform` are efficient implementations of 2D and 3D vector/matrix operations.
- KrisLibrary/math3d/rotation contains several representations of rigid 3D rotations, including euler angles, moments (aka exponential maps), angle-axis form, and quaternions. All representations can be transformed into one another. All routines are implemented to be numerically robust.

The `Vector`, `Vector3`, and `RigidTransform` classes are the most widely used math classes in Klamp't. `Vector`s accept all the basic arithmetic operations as well as dot products, norms, and distances.  Applying a transformation (`Matrix3` or `RigidTransform`) to a point (`Vector3`) is expressed using the \* operator.

## 3D Points and Directions

Points and directions are represented using the Math3D::Vector3 class in KrisLibrary/math3d/primitives.h. This class has many convenience methods, including adding, subtracting, multiplying, and dividing using the standard operators +, -, *, and /. The individual elements of the vector are given by the x, y, and z members, or you may also use array access [0], [1], and [2].

```python
#include <math3d/primitives.h>
#include <iostream>
using namespace Math3D;

int main(int argc,char** argv) {
  Vector3 a(0.3,1.5,-2.4);
  printf("original: %g %g %g\n",a.x,a.y,a.z);
  Vector3 b=a*4.0;   //scale by 4
  printf("times 4: %g %g %g\n",b.x,b.y,b.z);
  Vector3 c = b-a;   //subtraction
  std::cout<<"b-a: "<<c<<std::endl;
  c.inplaceNormalize();   //change to unit vector
  std::cout<<"normalized: "<<c<<std::endl;
  return 0;
}
```

Please consult the [Vector3 documentation](http://motion.cs.illinois.edu/klampt/krislibrary_docs/classMath3D_1_1Vector3.html) for more details.

## 3D Rotations

The Klamp't C++ API typically represents 3D rotations as a [Math3D::Matrix3](http://motion.cs.illinois.edu/klampt/krislibrary_docs/classMath3D_1_1Matrix3.html) object, as found in the KrisLibrary/math3d/primitives.h file.

For some basic operation, try the following code:

```python
#include <math3d/primitives.h>
#include <math/math.h>
#include <iostream>
using namespace Math3D;

int main(int argc,char** argv) {
  Matrix3D A;
  A.setIdentity();
  std::cout<<"Identity: "<<A<<std::endl;
  A.setRotateZ(DtoR(90.0));
  std::cout<"90 degree rotation about Z: "<<A<<std::endl;

  Vector3 x(1.0,2.0,3.0),y;
  std::cout<<"Applied to: "<<x<<": "<<A*x<<endl;
  A.mul(x,y);   //does the same as y=A*x, but does
                //not construct a temporary object
  std::cout<<"Applied to: "<<x<<": "<<y<<endl;
  return 0;
}
```

Klamp't also supports conversions to four other commonly used rotation representations: Euler angles, axis-angle, moment (aka exponential map, rotation vector), and quaternions. These representations are given in the KrisLibrary/math3d/rotation.h file. Each representation can be converted to/from a Matrix3 representation using the getMatrix() and setMatrix() functions.

1. Euler angle representations are given by the EulerAngleRotation class, and consist of three angles phi, psi, theta about some axes A, B, and C. The axes are given by convention, some of the most common being roll-pitch-yaw (A.K.A. Z-Y-X) and Z-Y-Z convention. To convert from a matrix, the convention must be specified in the suffixes setMatrix[ABC]() and getMatrix[ABC]().
2. Axis-angle representations are given by the AxisAngleRotation class, and consist of a Vector3 axis and a Real angle (in radians).
3. Moment representations are given by the MomentRotation class, and are very similar to axis-angle representations but are more compact. They are a Vector3 (mx,my,mz) equivalent to axis*angle.
4. Quaternion representations are given by the QuaternionRotation class, and are a 4-tuple (x,y,z,w) representing a unit quaternion.

Rotation composition can be computed by the operator A*B or the Matrix3.mul(A,B) method. Note that the result corresponds to a rotation first by B, and then by A. (Recall that rotation composition is not symmetric! A*B != B*A unless the two rotations share the same axis of rotation) Inversion of a rotation is accomplished via the Matrix3.setTranspose(A) function (this assumes A is a rotation, because inversion is equivalent to the matrix transpose, since rotation matrices are orthogonal.) There is also a setInverse() method which will compute the same result, but setTranspose() is faster.

The space of rotations is fundamentally different from Cartesian space, and hence computing interpolations and finding the difference between rotations is not as simple as taking standard linear interpolations of the matrices. The KrisLibrary/math3d/interpolation.h header provides functionality for properly computing geodesics on SO(3). Specifically, interpolateRotation should be used. The absolute angle (calculated via the axis-angle representation) is also the most appropriate way to represent distances between rotations.

```python
#include <math3d/primitives.h>
#include <math3d/interpolate.h>
#include <math3d/rotation.h>
#include <math/math.h>
#include <iostream>
using namespace Math3D;

int main(int argc,char** argv) {
  Matrix3D A,B;
  A.setRotateZ(DtoR(90.0));
  B.setRotateX(DtoR(-90.0));
  #WRONG WAY! SO3 is not a cartesian space
  #std::cout<<"Halfway: "<<(A+B)*0.5<<std::endl;
  #std::cout<<"Difference: "<<(B-A)*0.5<<std::endl;

  #RIGHT WAY! 
  AxisAngleRotation aa;
  Matrix3 diff,temp;
  diff.mulTransposeB(A,B);
  aa.setMatrix(diff);
  std::cout<<"Distance: "<<Abs(aa.angle)<<std::endl;
  interpolateRotation(A,B,0.0,temp);
  std::cout<<"Start of interpolation: "<<temp<<std::endl;
  interpolateRotation(A,B,0.5,temp);
  std::cout<<"Halfway: "<<temp<<std::endl;
  interpolateRotation(A,B,1.0,temp);
  std::cout<<"End of interpolation: "<<temp<<std::endl;
  return 0;
}
```

## Rigid Transformations

Rigid transformations are used throughout Klamp't, and represent an function y = R*x+t, where R is a 3x3 rotation matrix, t is a 3D translation vector, x is the input 3D point, and y is the 3D output point. The transform is represented via the [Math3D::RigidTransform](http://motion.cs.illinois.edu/klampt/krislibrary_docs/classMath3D_1_1RigidTransform.html) class. The object simply consists of members Matrix3 R and Vector3 t.

You may apply a transform T to a Vector3 x using T*x, or the function T.mul(x,y), or T.mulPoint(x,y). If x is a direction vector, and you wish to apply only the rotation part of the transform, you can either do this manually via T.R*x or via the convenience function T.mulVector(x,y)

Transforms may be composed using the RigidTransform.mul(A,B) function and inverted using the RigidTransform.setInverse(A) function.

## Linear Algebra

Klamp't comes with a large suite of linear algebra routines in the KrisLibrary library. The [Vector](http://motion.cs.illinois.edu/klampt/krislibrary_docs/classMath_1_1VectorTemplate.html) class in KrisLibrary/math/vector.h contains basic linear algebra routines on vectors (adding, subtracting, multiplying, interpolating). The [Matrix](http://motion.cs.illinois.edu/klampt/krislibrary_docs/classMath_1_1MatrixTemplate.html) class in KrisLibrary/math/matrix.h contains basic matrix-matrix and matrix-vector operations. There are also various sparse vector / sparse matrix structures available. Note that these are designed primarily for convenience, and are unlikely to be quite as fast as more specialized packages for basic operations (e.g., BLAS, ATLAS, Intel MKL).

Linear algebra routines are available for the following operations:

*   Gram-Schmidt orthogonalization
*   LU decomposition
*   Cholesky/LDL decomposition
*   QR decomposition
*   Singular value decomposition

Note that these are designed primarily for convenience and robustness rather than speed, and you may get better performance using other packages. Please consult the [KrisLibrary documentation](http://motion.cs.illinois.edu/klampt/krislibrary_docs) for more details.
