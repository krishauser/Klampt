Geometry and Appearance
=======================================

Klamp't uses a variety of geometry types to define geometric primitives,
triangulated meshes, and point clouds. Each geometry may also be
associated with an `Appearance <#appearance>`__.

Geometries and collision geometries
-----------------------------------

Klamp't stores Geometry datain an object's local frame.

|Illustration of concepts|

The notion of a *collision geometry* combines some underlying geometric
data with transformations and collision acceleration structures.

Collision geometries have a *current transformation* that sets where
they exist in space, and is used for collision testing.

Collision geometries also support an additional, nonnegative *collision margin*
setting that "expands" the underlying geometry when performing collision testing. The
margin does not actually affect the geometric data, but rather it
changes the distance threshold that is used to consider colliding vs.
noncolliding geometries.

If either object has a collision margin, the collision detection functions
treat the underyling data as though it were "expanded" by that amount.  So,
for distance queries, the sum of the objects' collision margins is subtracted
from the distance result.

.. note::
   In visualization, a geometry is drawn in its current transformation, but
   **its collision margin is not**.  If collision detection is reporting that
   two objects are touching but they don't appear to be, you should check their
   collision margins.

Geometric operation support
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following geometry types are currently supported:

-  *Triangle mesh*: complete, optimized, and well-tested.
-  *Point cloud*: nearly complete support.
-  *Geometric primitive*: nearly complete support.
-  *Implicit surface volume grid*: partial support.

The following operations are supported:

-  *Drawing*: All types supported.
-  *Collision detection in planning*. All types supported. 
-  *Tolerance verification*. All types supported. 
-  *Distance detection in planning*. primitive/primitive, primitive/point cloud,
   triangle mesh/triangle mesh, point cloud / point cloud, point cloud / implicit
   surface distance functions are available.
-  *Ray casting*. Triangle meshes, point clouds, and geometric primitives.
-  *Contact detection in simulation*. Triangle mesh / triangle mesh and
   triangle mesh / point cloud only.


In addition, there is a notion of a geometry *Group*, which can combine multiple
sub-geometries.


Geometry caching
~~~~~~~~~~~~~~~~

When multiple objects load the same geometry file, Klamp't uses a
caching mechanism to avoid reloading the file from disk and re-creating
collision acceleration structures. This is essential for loading very
large scenes with many replicated objects. However, when geometries are
transformed by API calls, they are removed from the cache. So, to
achieve maximum performance with many duplicated geometries, it is
recommended to transform the geometry files themselves in advance rather
than dynamically through the API.

API summary
~~~~~~~~~~~

The :class:`~klampt.Geometry3D` container class is an abstraction of all supported types of
geometries.  Each data type is represented by one of the data classes,
:class:`~klampt.TriangleMesh`, :class:`~klampt.PointCloud`, :class:`~klampt.GeometricPrimitive`, and
:class:`~klampt.VolumeGrid`.

**Basic construction**:

-  ``geom=Geometry3D()``: creates a new standalone geometry, not
   associated with any world object.
-  ``geom=Geometry3D(data)``: creates a new standalone geometry from a geometry
   data class.
-  ``geom=[RobotModelLink/RigidObjectModel/TerrainModel].geometry()``:
   retrieves a reference to the object's geometry.
-  ``geom.clone()``: duplicates the geometry.
-  ``geom.empty()``: returns True if the geometry is empty.
-  ``geom.free()``: if the geometry is standalone, deletes the data
   associated with it.
-  ``geom.set(geom2)``: copies the contents of geom2 into this
   geometry.
-  ``geom.loadFile(fn)``: loads a geometry from a file.
-  ``geom.saveFile(fn)``: saves a geometry to a file.
-  ``geom.convert(type, param=0)``: converts a geometry in-place to another
   type.  GeometricPrimitive -> any, TriangleMesh -> PointCloud,
   TriangleMesh -> VolumeGrid, and VolumeGrid -> TriangleMesh are
   supported.

**Extraction of data**:

-  ``geom.type()``: returns a string giving the type of the object.
-  ``geom.getTriangleMesh()``: returns the :class:`~klampt.TriangleMesh` data of the
   geometry if the type is ``'TriangleMesh'``
-  ``geom.getPointCloud()``: returns the :class:`~klampt.PointCloud` data of the
   geometry if the type is ``'PointCloud'``
-  ``geom.getGeometricPrimitive()``: returns the :class:`~klampt.GeometricPrimitive` data of the
   geometry if the type is ``'GeometricPrimitive'``
-  ``geom.getVolumeGrid()``: returns the :class:`~klampt.VolumeGrid` data of the
   geometry if the type is ``'VolumeGrid'``
-  ``geom.numElements()``: returns the number of elements.
-  ``geom.getElement(id)``: returns a sub-object of a Group, TriangleMesh, or
   PointCloud geometry.

** Modifying current transform and collision margin **

The current transform of a geometry of a world object is updated
when its configuration changes.  However, if you are using standalone
geometries, you will have to set the transformation yourself.  All quantities
are measured with respect to world coordinates.

-  ``geom.setCurrentTransform(R,t)``: sets the object's current transformation to the
   rotation R and translation t
-  ``geom.getCurrentTransform()``: returns the object's current transformation.

To change the collision margin, use the following.  Collision margins are
by default 0.

-  ``geom.setCollisionMargin(margin)``: sets the object's collision margin
-  ``geom.getCollisionMargin()``: gets the object's collision margin.


Collision detection
-------------------

The :class:`~klampt.Geometry3D` class allows collision testing between
geometries. All the standard Klamp't geometry types (geometric
primitives, triangle meshes, point clouds) are supported.

For convenience, the :mod:`klampt.model.collide`
module provides utility functions for checking collision with sets of
objects, as well as a :class:`~klampt.model.collide.WorldCollider` class that by checks collision
between any set of objects and any other set of objects. These methods
return an iterator over collision pairs, which allows the user to either
stop at the first collision or enumerate all collisions.

API summary
~~~~~~~~~~~

The :class:`~klampt.Geometry3D` methods may be used for performing collision detection:

-  ``geom.getBB()``: returns a loose approximation to the object's bounding box, in
   its current configuration.
-  ``geom.getBBTight()``: returns a tight approximation to the object's bounding box, in
   its current configuration.  Slower than ``getBB``.
-  ``geom.collides(geom2)``: returns True if the objects collide.
-  ``geom.distance(geom2)``: returns the distance / signed distance between the
   objects.
-  ``geom.distance_simple(geom2,relErr=0,absErr=0)``: returns the distance / signed
   distance between the objects as a float.
-  ``geom.distance_point(pt)``: returns the distance / signed distance between the
   object and a point.
-  ``geom.rayCast(source,direction)``: casts a ray with a given source and direction.
-  ``geom.rayCast_ext(source,direction)``: same as rayCast, but returns the index of the
   first intersected element.

For more control over distance queries, you may use the following functions, which
have the suffix ``_ext`` and accept a :class:`~klampt.DistanceQuerySettings` object: 

-  ``geom.distance_ext(geom2,settings)``
-  ``geom.distance_point_ext(pt,settings)``

The following :class:`~klampt.model.collide.WorldCollider` methods are used most often:

-  ``collisions()``: checks for all collisions.
-  ``collisions(filter)``: checks for all collisions between objects for
   which filter(obj) returns True
-  ``collisions(filter1,filter2)``: checks for all collisions between
   pairs of objects for which filter1(objA) and filter2(objB) both
   return True
-  ``robotSelfCollisions``, ``robotObjectCollisions``,
   ``robotTerrainCollisions``, ``objectObjectCollisions``, and
   ``objectTerrainCollisions`` check collisions only between the
   indicated robots/objects/terrains.
-  ``rayCast(s,d)``: performs ray casting against objects in the world
   and returns the nearest collision found.



Appearance
----------

Forthcoming. See the :class:`~klampt.Appearance` API for detailed documentation.


.. |Illustration of concepts| image:: _static/images/concepts-geometry.png

