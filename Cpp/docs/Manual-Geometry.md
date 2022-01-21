# Klamp't Manual: Geometry and Appearance

* [Geometries and collision geometries](#geometries-and-collision-geometries)
    + [Geometric operation support](#geometric-operation-support)
    + [Geometry caching](#geometry-caching)
    + [API summary](#api-summary)
* [Appearance](#appearance)

Klamp't uses a variety of geometry types to define geometric primitives, triangulated meshes, and point clouds.  Each geometry may also be associated with an [Appearance](#appearance).

## Geometries and collision geometries
Klamp't stores Geometry datain an object's local frame.

![Illustration of concepts](images/concepts-geometry.png)

The notion of a _collision geometry_ combines some underlying geometric data with transformations and collision acceleration structures. Collision geometries have a _current transformation_ that sets where they exist in space, and is used for collision testing. Collision geometries also support an additional, nonnegative margin setting that &quot;expands&quot; the underlying geometry when performing collision testing.  The margin does not actually affect the geometric data, but rather it changes the distance threshold that is used to consider colliding vs. noncolliding geometries.

### Geometric operation support

The following geometries are currently supported:
- _Triangle meshes_: complete, optimized, and well-tested.
- _Point clouds_: nearly complete support.
- _Geometric primitives_: nearly complete support. 
- _Implicit surface volume grids_: mostly well supported.
- _Convex hulls_: only weakly supported, mostly for convex hull-convex hull collision and distance checks.

The following operations are supported:
- _Drawing_: All types supported.
- _Collision detection in planning_. All types supported. Note: Point cloud collision detection is currently inefficient for large point clouds.
- _Tolerance verification_. All types supported. Note: Point cloud collision detection is currently inefficient for large point clouds.
- _Distance detection in planning_.  primitive/primitive, triangle mesh/triangle mesh, convex hull/convex hull, point cloud / volume grid distance functions are available.
- _Ray casting_. Triangle meshes, point clouds, geometric primitives, convex hulls.
- _Contact detection in simulation_. Triangle mesh / triangle mesh, triangle mesh / point cloud, point cloud / volume grid only.


### Geometry caching

When multiple objects load the same geometry file, Klamp't uses a caching mechanism to avoid reloading the file from disk and re-creating collision acceleration structures. This is essential for loading very large scenes with many replicated objects. However, when geometries are transformed by API calls, they are removed from the cache. So, to achieve maximum performance with many duplicated geometries, it is recommended to transform the geometry files themselves in advance rather than dynamically through the API.

### API summary

Geometry data is stored in the `AnyGeometry3D` type and collision geometries are stored in the `AnyCollisionGeometry3D` type. These are essentially container types that abstract the underlying geometry and collision acceleration data structures. To operate on the data therein, users will need to inspect the geometry's type and cast to the appropriate type. Detailed documentation can be found in the following files:

- KrisLibrary/math3d/geometry3d.h defines 3D geometric primitives, including `Point3D`, `Segment3D`, `Triangle3D`, `AABB3D`, `Box3D`, `Sphere3D`, and `Ellipsoid3D`.  There is also a `GeometricPrimitive3D` class that abstracts common operations on any geometric primitive.
- KrisLibrary/meshing/TriMesh.h defines 3D triangle meshes.
- KrisLibrary/meshing/PointCloud.h defines a 3D point cloud. Each point may contain a variety of other named properties, including color, normal, id, etc.
- KrisLibrary/geometry/CollisionMesh.h contains the `CollisionMesh` and `CollisionMeshQuery` data structures. `CollisionMesh` overloads the `Meshing::TriMeshWithTopology` class and represents a preprocessed triangle mesh for collision detection. It can be rigidly transformed arbitrarily in space for making fast collision queries via the `CollisionMeshQuery` class and the `Collide`/`Distances`/`WithinDistance` functions. Mesh-mesh proximity testing (collision and distance computation) are handled by the open source PQP library developed by UNC Chapel Hill. These routines are heavily tested and fast.
- KrisLibrary/geometry/AnyGeometry.h defines the `AnyGeometry3D`, `AnyCollisionGeometry3D`, and `AnyCollisionQuery` classes.  It is recommended to use these classes for geometric operations because they are abstract and may be extended to handle more geometry representations in the future.


## Appearance

TODO