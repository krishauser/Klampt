
// File: index.xml

// File: classAABBPoser.xml


%feature("docstring") AABBPoser "
";

%feature("docstring") AABBPoser::AABBPoser "
";

%feature("docstring") AABBPoser::set "
";

%feature("docstring") AABBPoser::setFrame "
";

%feature("docstring") AABBPoser::get "
";

// File: classAppearance.xml


%feature("docstring") Appearance "

Geometry appearance information. Supports vertex/edge/face rendering, per-vertex
color, and basic color texture mapping. Uses OpenGL display lists, so repeated
calls are fast.  

For more complex appearances, you will need to call your own OpenGL calls.  

Appearances can be either references to appearances of objects in the world, or
they can be standalone.  

Performance note: Avoid rebuilding buffers (e.g., via :meth:`refresh`) as much
as possible.  

C++ includes: appearance.h
";

%feature("docstring") Appearance::Appearance "
";

%feature("docstring") Appearance::Appearance "
";

%feature("docstring") Appearance::~Appearance "
";

%feature("docstring") Appearance::refresh "

call this to rebuild internal buffers, e.g., when the OpenGL context changes. If
deep=True, the entire data structure will be revised. Use this for streaming
data, for example.  
";

%feature("docstring") Appearance::clone "

Creates a standalone appearance from this appearance.  
";

%feature("docstring") Appearance::set "

Copies the appearance of the argument into this appearance.  
";

%feature("docstring") Appearance::isStandalone "

Returns true if this is a standalone appearance.  
";

%feature("docstring") Appearance::free "

Frees the data associated with this appearance, if standalone.  
";

%feature("docstring") Appearance::setDraw "
";

%feature("docstring") Appearance::setDraw "

Turns on/off visibility of the object or a feature.  

If one argument is given, turns the object visibility on or off  

If two arguments are given, turns the feature (first int argument) visibility on
or off. feature can be ALL, VERTICES, EDGES, or FACES.  
";

%feature("docstring") Appearance::getDraw "
";

%feature("docstring") Appearance::getDraw "

Returns whether this object or feature is visible.  

If no arguments are given, returns whether the object is visible.  

If one int argument is given, returns whether the given feature is visible.
feature can be ALL, VERTICES, EDGES, or FACES.  
";

%feature("docstring") Appearance::setColor "
";

%feature("docstring") Appearance::setColor "

Sets color of the object or a feature.  

If 3 or 4 arguments are given, changes the object color.  

If 5 arguments are given, changes the color of the given feature. feature can be
ALL, VERTICES, EDGES, FACES, EMISSIVE, or SPECULAR.  
";

%feature("docstring") Appearance::getColor "

Return type: Tuple[float,float,float,float].  
";

%feature("docstring") Appearance::getColor "

Gets color of the object or a feature.  

If 0 arguments are given, retrieves the main object color.  

If 1 arguments are given, returns the color of the given feature. feature.
feature can be ALL, VERTICES, EDGES, FACES, EMISSIVE, or SPECULAR.  

Return type: Tuple[float,float,float,float]  
";

%feature("docstring") Appearance::setColors "

Sets per-element color for elements of the given feature type. Must be an mxn
array. m is the number of features of that type, and n is either 3 or 4.  

If n == 4, they are assumed to be rgba values, and  

If n == 3, each row is an rgb value.  

Only supports feature=VERTICES and feature=FACES  
";

%feature("docstring") Appearance::getColors "

Retrieves per-element color for elements of the given feature type. If per-
element colors are not enabled, then a 1 x 4 array is returned. Otherwise,
returns an m x 4 array, where m is the number of featuress of that type.  

Return type: np.ndarray  
";

%feature("docstring") Appearance::setTintColor "

Sets a temporary tint color that modulates the appearance of the object. This
works with both flat colors and per-vertex / per-face colors.  
";

%feature("docstring") Appearance::getTintColor "

Retrieves the tint color.  

Return type: Tuple[float,float,float,float]  
";

%feature("docstring") Appearance::getTintStrength "

Retrieves the tint strength.  
";

%feature("docstring") Appearance::setShininess "

Sets the specular highlight shininess and strength. To turn off, use
`setShininess(0)`. The specular strength can be set via the second argument.
`setShininess(20,0.1)`. Note that this changes the specular color.  
";

%feature("docstring") Appearance::getShininess "

Retrieves the specular highlight shininess.  
";

%feature("docstring") Appearance::setElementColor "

Sets the per-element color for the given feature.  
";

%feature("docstring") Appearance::getElementColor "

Gets the per-element color for the given feature.  

Return type: Tuple[float,float,float,float]  
";

%feature("docstring") Appearance::setTexture1D_b "

Sets a 1D texture of the given width. Valid format strings are.  

*   \"\": turn off texture mapping  
*   l8: unsigned byte grayscale colors  
";

%feature("docstring") Appearance::setTexture1D_i "

Sets a 1D texture of the given width. Valid format strings are.  

*   \"\": turn off texture mapping  
*   rgba8: unsigned byte RGBA colors with red in the 1st byte and alpha in the
    4th  
*   bgra8: unsigned byte RGBA colors with blue in the 1st byte and alpha in the
    4th  
";

%feature("docstring") Appearance::setTexture1D_channels "

Sets a 1D texture of the given width, given a 2D array of channels. Valid format
strings are.  

*   \"\": turn off texture mapping  
*   rgb8: unsigned byte RGB colors with red in the 1st column, green in the 2nd,
    blue in the 3rd  
*   bgr8: unsigned byte RGB colors with blue in the 1st column, green in the
    2nd, green in the 3rd  
*   rgba8: unsigned byte RGBA colors with red in the 1st column and alpha in the
    4th  
*   bgra8: unsigned byte RGBA colors with blue in the 1st column and alpha in
    the 4th  
*   l8: unsigned byte grayscale colors, one channel  
";

%feature("docstring") Appearance::getTexture1D_format "

Retrieves a 1D texture format, returning '' if the texture is not set.  
";

%feature("docstring") Appearance::getTexture1D_channels "

Retrieves a view into the 1D texture data. If the texture is not set, throws an
exception.  

Return type: np.ndarray  
";

%feature("docstring") Appearance::setTexture2D_b "

Sets a 2D texture of the given width/height. See :func:`setTexture1D_b` for
valid format strings.  

The array is given in top to bottom order if `topdown==True`. Otherwise, it is
given in order bottom to top.  
";

%feature("docstring") Appearance::setTexture2D_i "

Sets a 2D texture of the given width/height. See :func:`setTexture1D_i` for
valid format strings.  

The array is given in top to bottom order if `topdown==True`. Otherwise, it is
given in order bottom to top.  
";

%feature("docstring") Appearance::setTexture2D_channels "

Sets a 2D texture of the given width/height from a 3D array of channels. See
:func:`setTexture1D_channels` for valid format strings.  

The array is given in top to bottom order if `topdown==True`. Otherwise, it is
given in order bottom to top.  
";

%feature("docstring") Appearance::getTexture2D_format "

Retrieves a 2D texture format, returning '' if the texture is not set.  
";

%feature("docstring") Appearance::getTexture2D_channels "

Retrieves a view into the 2D texture data. If the texture is not set, throws an
exception.  

Return type: np.ndarray  
";

%feature("docstring") Appearance::setTexcoords1D "

Sets per-vertex texture coordinates for a 1D texture.  

You may also set uvs to be empty, which turns off texture mapping altogether.  
";

%feature("docstring") Appearance::getTexcoords1D "

Gets per-vertex texture coordinates for a 1D texture. If no 1D texture is set,
throws an exception.  

Return type: np.ndarray  
";

%feature("docstring") Appearance::setTexcoords2D "

Sets per-vertex texture coordinates for a 2D texture. uvs is an array of shape
(nx2) containing U-V coordinates [[u1, v1], [u2, v2], ..., [un, vn]].  

You may also set uvs to be empty, which turns off texture mapping altogether.  
";

%feature("docstring") Appearance::getTexcoords2D "

Gets per-vertex texture coordinates for a 2D texture. If no 2D texture is set,
throws an exception.  

Return type: np.ndarray  
";

%feature("docstring") Appearance::setTexgen "

Sets the texture generation. The array must be size m x 4, with m in the range
0,...,4. If worldcoordinates=true, the texture generation is performed in world
coordinates rather than object coordinates.  
";

%feature("docstring") Appearance::getTexgenMatrix "

Retrieves the texture generation. The array will be size m x 4, with m in the
range 0,...,4. The texture generation is performed in  

Return type: np.ndarray  
";

%feature("docstring") Appearance::isTexgenWorld "

Returns whether texture generation is performed in world coordinates.  
";

%feature("docstring") Appearance::setTexWrap "

Sets whether textures are to wrap (default true)  
";

%feature("docstring") Appearance::setPointSize "

For point clouds, sets the point size.  
";

%feature("docstring") Appearance::setCreaseAngle "

For meshes, sets the crease angle. Set to 0 to disable smoothing.  
";

%feature("docstring") Appearance::setSilhouette "

For meshes sets a silhouette radius and color. Set the radius to 0 to disable
silhouette drawing.  
";

%feature("docstring") Appearance::drawGL "

Draws the currently associated geometry with this appearance. A geometry is
assocated with this appearance if this appearance comes from an element of the
WorldMode, or if drawGL(geom) was previously called.  

Note that the geometry's current transform is NOT respected, and this only draws
the geometry in its local transform.  
";

%feature("docstring") Appearance::drawGL "

Draws the given geometry with this appearance. NOTE: for best performance, an
appearance should only be drawn with a single geometry. Otherwise, the OpenGL
display lists will be completely recreated.  

Note that the geometry's current transform is NOT respected, and this only draws
the geometry in its local transform.  
";

%feature("docstring") Appearance::drawWorldGL "

Draws the given geometry with this appearance. NOTE: for best performance, an
appearance should only be drawn with a single geometry. Otherwise, the OpenGL
display lists will be completely recreated.  

Differs from drawGL in that the geometry's current transform is applied before
drawing.  
";

// File: classBoxPoser.xml


%feature("docstring") BoxPoser "
";

%feature("docstring") BoxPoser::BoxPoser "
";

%feature("docstring") BoxPoser::set "
";

%feature("docstring") BoxPoser::setTransform "
";

%feature("docstring") BoxPoser::setDims "
";

%feature("docstring") BoxPoser::getTransform "
";

%feature("docstring") BoxPoser::getDims "
";

// File: classContactParameters.xml


%feature("docstring") ContactParameters "

Stores contact parameters for an entity. Currently only used for simulation, but
could be used for contact mechanics in the future.  

Attributes:  

    kFriction (float): The coefficient of (Coulomb) friction, in range
        [0,inf).
    kRestitution (float): The coefficient of restitution, in range [0,1].
    kStiffness (float): The stiffness of the material, in range (0,inf)
        (default inf, perfectly rigid).
    kDamping (float): The damping of the material, in range (0,inf)
        (default inf, perfectly rigid).
  

C++ includes: robotmodel.h
";

%feature("docstring") ContactParameters::ContactParameters "
";

// File: classContactQueryResult.xml


%feature("docstring") ContactQueryResult "

The result from a contact query of :class:`~klampt.Geometry3D`. The number of
contacts n is variable.  

Attributes:  

    depths (list of n floats): penetration depths for each contact point.
        The depth is measured with respect to the padded geometry, and must
        be nonnegative. A value of 0 indicates that depth cannot be
        determined accurately.
    points1, points2 (list of n lists of floats): contact points on self vs
        other,  The top level list has n entries, and each entry is a
        3-list expressed in world coordinates.  If an object is padded,
        these points are on the surface of the padded geometry.
    normals (list of n lists of floats): the outward-facing contact normal
        from this to other at each contact point, given in world
        coordinates.  Each entry is a 3-list, and can be a unit vector,
        or [0,0,0] if the the normal cannot be computed properly.
    elems1, elems2 (list of n ints): for compound objects, these are the
        element indices corresponding to each contact.
  

C++ includes: geometry.h
";

%feature("docstring") ContactQueryResult::ContactQueryResult "
";

// File: structConvexHull.xml


%feature("docstring") ConvexHull "

Stores a set of points to be set into a ConvexHull type. Note: These may not
actually be the vertices of the convex hull; the actual convex hull may be
computed internally for some datatypes.  

Attributes:  

    points (numpy array): an nx3 numpy array of points.
  

C++ includes: geometry.h
";

%feature("docstring") ConvexHull::ConvexHull "
";

%feature("docstring") ConvexHull::ConvexHull "
";

%feature("docstring") ConvexHull::~ConvexHull "
";

%feature("docstring") ConvexHull::copy "

Creates a standalone object that is a copy of this.  
";

%feature("docstring") ConvexHull::set "

Copies the data of the argument into this.  
";

%feature("docstring") ConvexHull::getPoints "

Retrieves a view of the points.  

Returns:  

    ndarray: an nx3 Numpy array. Setting elements of this array will
    change the points.
 Return type: np.ndarray  
";

%feature("docstring") ConvexHull::setPoints "

Sets all points to the given nx3 Numpy array.  
";

%feature("docstring") ConvexHull::addPoint "

Adds a point.  
";

%feature("docstring") ConvexHull::translate "

Translates all the vertices by v=v+t.  
";

%feature("docstring") ConvexHull::transform "

Transforms all the vertices by the rigid transform v=R*v+t.  
";

// File: classCSpaceInterface.xml


%feature("docstring") CSpaceInterface "

A raw interface for a configuration space.  

..note::  

    The native Python CSpace interface class in cspace.py is easier to use.  

You can either set a single feasibility test function using setFeasibility() or
add several feasibility tests, all of which need to be satisfied, using
addFeasibilityTest(). In the latter case, planners may be able to provide
debugging statistics, solve Minimum Constraint Removal problems, run faster by
eliminating constraint tests, etc.  

Either setVisibility() or setVisibilityEpsilon() must be called to define a
visibility checker between two (feasible) configurations. In the latter case,
the path will be discretized at the resolution sent to setVisibilityEpsilon. If
you have special single-constraint visibility tests, you can call that using
addVisibilityTest (for example, for convex constraints you can set it to the
lambda function that returns true regardless of its arguments).  

Supported properties include \"euclidean\" (boolean), \"metric\" (string),
\"geodesic\" (boolean), \"minimum\" (vector), and \"maximum\" (vector). These
may be used by planners to make planning faster or more accurate. For a complete
list see KrisLibrary/planning/CSpace.h.  

C++ includes: motionplanning.h
";

%feature("docstring") CSpaceInterface::CSpaceInterface "
";

%feature("docstring") CSpaceInterface::CSpaceInterface "
";

%feature("docstring") CSpaceInterface::~CSpaceInterface "
";

%feature("docstring") CSpaceInterface::destroy "
";

%feature("docstring") CSpaceInterface::setFeasibility "
";

%feature("docstring") CSpaceInterface::addFeasibilityTest "
";

%feature("docstring") CSpaceInterface::setVisibility "
";

%feature("docstring") CSpaceInterface::addVisibilityTest "
";

%feature("docstring") CSpaceInterface::setVisibilityEpsilon "
";

%feature("docstring") CSpaceInterface::setSampler "
";

%feature("docstring") CSpaceInterface::setNeighborhoodSampler "
";

%feature("docstring") CSpaceInterface::setDistance "
";

%feature("docstring") CSpaceInterface::setInterpolate "
";

%feature("docstring") CSpaceInterface::setProperty "
";

%feature("docstring") CSpaceInterface::getProperty "
";

%feature("docstring") CSpaceInterface::isFeasible "

Queries whether a given configuration is feasible.  
";

%feature("docstring") CSpaceInterface::isVisible "

Queries whether two configurations are visible.  
";

%feature("docstring") CSpaceInterface::testFeasibility "

Queries whether a given configuration is feasible with respect to a given
constraint.  
";

%feature("docstring") CSpaceInterface::testVisibility "

Queries whether two configurations are visible with respect to a given
constraint.  
";

%feature("docstring") CSpaceInterface::feasibilityFailures "

Returns a list of all failed feasibility constraints.  
";

%feature("docstring") CSpaceInterface::visibilityFailures "

Returns a list of all failed visibility constraints.  
";

%feature("docstring") CSpaceInterface::sample "

Samples a configuration.  
";

%feature("docstring") CSpaceInterface::distance "

Returns the distance between two configurations.  
";

%feature("docstring") CSpaceInterface::interpolate "

Interpolates between two configurations.  
";

%feature("docstring") CSpaceInterface::adaptiveQueriesEnabled "

optional: adaptive queries can be used to automatically minimize the total cost
of testing feasibility / visibility using empirical estimates. Off by default.  
";

%feature("docstring") CSpaceInterface::enableAdaptiveQueries "

Call this to enable adaptive queries. (It has a small overhead.)  
";

%feature("docstring") CSpaceInterface::optimizeQueryOrder "

Call this to optimize the feasibility / visibility testing order.  
";

%feature("docstring") CSpaceInterface::setFeasibilityDependency "

Marks that a certain feasibility test must be performed before another.  
";

%feature("docstring") CSpaceInterface::setFeasibilityPrior "

Resets the data for a certain feasibility test. Default values give a data-
gathering behavior.  
";

%feature("docstring") CSpaceInterface::setVisibilityDependency "

Marks that a certain feasibility test must be performed before another.  
";

%feature("docstring") CSpaceInterface::setVisibilityPrior "

Resets the data for a certain visibility test. Default values give a data-
gathering behavior.  
";

%feature("docstring") CSpaceInterface::feasibilityCost "

Retrieves the empirical average cost of a given feasibility test.  
";

%feature("docstring") CSpaceInterface::feasibilityProbability "

Retrieves the empirical average success rate of a given feasibility test.  
";

%feature("docstring") CSpaceInterface::visibilityCost "

Retrieves the empirical average cost of a given visibility test.  
";

%feature("docstring") CSpaceInterface::visibilityProbability "

Retrieves the empirical average success rate of a given visibility test.  
";

%feature("docstring") CSpaceInterface::feasibilityQueryOrder "

Retrieves the current order of feasibility tests.  
";

%feature("docstring") CSpaceInterface::visibilityQueryOrder "

Retrieves the current order of visibility tests.  
";

%feature("docstring") CSpaceInterface::getStats "

Returns constraint testing statistics. If adaptive queries are enabled, this
returns the stats on each constraint.  
";

// File: classDistanceQueryResult.xml


%feature("docstring") DistanceQueryResult "

The result from a \"fancy\" distance query of :class:`~klampt.Geometry3D`.  

Attributes:  

    d (float): The calculated distance, with negative values indicating
        penetration.  Can also be upperBound if the branch was hit.
    hasClosestPoints (bool):  If true, the closest point information is
        given in cp0 and cp1, and elem1 and elem2
    hasGradients (bool):  f true, distance gradient information is given
        in grad0 and grad1.
    cp1, cp2 (list of 3 floats, optional): closest points on self vs other,
        both given in world coordinates
    grad1, grad2 (list of 3 floats, optional): the gradients of the
        objects' signed distance fields at the closest points.  Given in
        world coordinates.

        I.e., to move object1 to touch object2, move it in direction
        grad1 by distance -d.  Note that grad2 is always -grad1.
    elems1, elems2 (int): for compound objects, these are the
        element indices corresponding to the closest points.
  

C++ includes: geometry.h
";

%feature("docstring") DistanceQueryResult::DistanceQueryResult "
";

// File: classDistanceQuerySettings.xml


%feature("docstring") DistanceQuerySettings "

Configures the _ext distance queries of :class:`~klampt.Geometry3D`.  

The calculated result satisfies :math:`Dcalc \\leq D(1+relErr) + absErr` unless
:math:`D \\geq upperBound`, in which case Dcalc=upperBound may be returned.  

Attributes:  

    relErr (float, optional): Allows a relative error in the reported
        distance to speed up computation.  Default 0.
    absErr (float, optional): Allows an absolute error in the reported
        distance to speed up computation.  Default 0.
    upperBound (float, optional): The calculation may branch if D exceeds
        this bound.
  

C++ includes: geometry.h
";

%feature("docstring") DistanceQuerySettings::DistanceQuerySettings "
";

// File: classGeneralizedIKObjective.xml


%feature("docstring") GeneralizedIKObjective "

An inverse kinematics target for matching points between two robots and/or
objects.  

The objects are chosen upon construction, so the following are valid:  

*   GeneralizedIKObjective(a) is an objective for object a to be constrained
    relative to the environment.  
*   GeneralizedIKObjective(a,b) is an objective for object a to be constrained
    relative to b. Here a and b can be links on any robot or rigid objects.  

Once constructed, call setPoint, setPoints, or setTransform to specify the
nature of the constraint.  

C++ includes: robotik.h
";

%feature("docstring") GeneralizedIKObjective::GeneralizedIKObjective "
";

%feature("docstring") GeneralizedIKObjective::GeneralizedIKObjective "
";

%feature("docstring") GeneralizedIKObjective::GeneralizedIKObjective "
";

%feature("docstring") GeneralizedIKObjective::GeneralizedIKObjective "
";

%feature("docstring") GeneralizedIKObjective::GeneralizedIKObjective "
";

%feature("docstring") GeneralizedIKObjective::GeneralizedIKObjective "
";

%feature("docstring") GeneralizedIKObjective::GeneralizedIKObjective "
";

%feature("docstring") GeneralizedIKObjective::setPoint "
";

%feature("docstring") GeneralizedIKObjective::setPoints "
";

%feature("docstring") GeneralizedIKObjective::setTransform "
";

%feature("docstring") GeneralizedIKObjective::sampleTransform "

Returns a transformation (R,t) from link relative to link2, sampled at random
from the space of transforms that satisfies the objective obj.  
";

// File: classGeneralizedIKSolver.xml


%feature("docstring") GeneralizedIKSolver "

An inverse kinematics solver between multiple robots and/or objects. NOT
IMPLEMENTED YET.  

C++ includes: robotik.h
";

%feature("docstring") GeneralizedIKSolver::GeneralizedIKSolver "
";

%feature("docstring") GeneralizedIKSolver::add "

Adds a new simultaneous objective.  
";

%feature("docstring") GeneralizedIKSolver::setMaxIters "

Sets the max # of iterations (default 100)  
";

%feature("docstring") GeneralizedIKSolver::setTolerance "

Sets the constraint solve tolerance (default 1e-3)  
";

%feature("docstring") GeneralizedIKSolver::getResidual "

Returns a vector describing the error of the objective.  
";

%feature("docstring") GeneralizedIKSolver::getJacobian "

Returns a matrix describing the instantaneous derivative of the objective with
respect to the active parameters.  
";

%feature("docstring") GeneralizedIKSolver::solve "

Tries to find a configuration that satifies all simultaneous objectives up to
the desired tolerance.  

Returns: res,iters (pair of bool, int): res indicates whether x converged, and
iters is the number of iterations used.  
";

%feature("docstring") GeneralizedIKSolver::sampleInitial "

Samples an initial random configuration.  
";

// File: structGeometricPrimitive.xml


%feature("docstring") GeometricPrimitive "

A geometric primitive. So far only points, spheres, segments, and AABBs can be
constructed manually in the Python API.  

Attributes:  

    type (str): Can be \"Point\", \"Sphere\", \"Segment\", \"Triangle\",
        \"Polygon\", \"AABB\", and \"Box\".  Semi-supported types include
        \"Ellipsoid\", and \"Cylinder\".
    properties (numpy array): a list of parameters defining the
        primitive. The interpretation is type-specific.
  

C++ includes: geometry.h
";

%feature("docstring") GeometricPrimitive::GeometricPrimitive "
";

%feature("docstring") GeometricPrimitive::GeometricPrimitive "
";

%feature("docstring") GeometricPrimitive::~GeometricPrimitive "
";

%feature("docstring") GeometricPrimitive::copy "

Creates a standalone object that is a copy of this.  
";

%feature("docstring") GeometricPrimitive::set "

Copies the data of the argument into this.  
";

%feature("docstring") GeometricPrimitive::setPoint "
";

%feature("docstring") GeometricPrimitive::setSphere "
";

%feature("docstring") GeometricPrimitive::setSegment "
";

%feature("docstring") GeometricPrimitive::setTriangle "
";

%feature("docstring") GeometricPrimitive::setPolygon "
";

%feature("docstring") GeometricPrimitive::setAABB "
";

%feature("docstring") GeometricPrimitive::setBox "
";

%feature("docstring") GeometricPrimitive::getType "
";

%feature("docstring") GeometricPrimitive::getProperties "
";

%feature("docstring") GeometricPrimitive::setProperties "
";

%feature("docstring") GeometricPrimitive::loadString "
";

%feature("docstring") GeometricPrimitive::saveString "
";

// File: classGeometry3D.xml


%feature("docstring") Geometry3D "

The three-D geometry container used throughout Klampt.  

There are eight currently supported types of geometry:  

*   primitives (:class:`GeometricPrimitive`)  
*   convex hulls (:class:`ConvexHull`)  
*   triangle meshes (:class:`TriangleMesh`)  
*   point clouds (:class:`PointCloud`)  
*   implicit surfaces (:class:`ImplicitSurface`)  
*   occupancy grids (:class:`OccupancyGrid`)  
*   heightmaps (:class:`Heightmap`)  
*   groups (\"Group\" type)  

For now we also support the \"VolumeGrid\" identifier which is treated as an
alias for \"ImplicitSurface\". This will be deprecated in a future version  

This class acts as a uniform container of all of these types.  

There are two modes in which a Geometry3D can be used. It can be a standalone
geometry, which means it is a container of geometry data, or it can be a
reference to a world item's geometry. For references, modifiers change the world
item's geometry.  

**Current transform**  

Each geometry stores a \"current\" transform, which is automatically updated for
world items' geometries. Proximity queries are then performed *with respect to
the transformed geometries*. Crucially, the underlying geometry is not changed,
because that could be computationally expensive.  

**Creating / modifying the geometry**  

Use the constructor, the :meth:`set`, or the set[TYPE]() methods to completely
change the geometry's data.  

Note: if you want to set a world item's geometry to be equal to a standalone
geometry, use the set(rhs) function rather than the assignment (=) operator.  

Modifiers include:  

*   :meth:`setCurrentTransform`: updates the current transform. (This call is
    very fast.)  
*   :meth:`translate`, :meth:`scale`, :meth:`rotate`, and :meth:`transform`
    transform the underlying geometry. Any collision data structures will be
    recomputed after transformation.  
*   :meth:`loadFile`: load from OFF, OBJ, STL, PCD, etc. Also supports native
    Klamp't types .geom and .vol.  

.. note::  

    Avoid the use of translate, rotate, and transform to represent object
    movement.  Use setCurrentTransform instead.
 **Proximity queries**  

*   :meth:`collides`: boolean collision query.  
*   :meth:`withinDistance`: boolean proximity query.  
*   :meth:`distance` and :meth:`distance_ext`: numeric-valued distance query.
    The distance may be negative to indicate signed distance, available for
    certain geometry types. Also returns closest points for certain geometry
    types.  
*   :meth:`distance_point` and :meth:`distance_point_ext`: numeric valued
    distance-to-point queries.  
*   :meth:`contacts`: estimates the contact region between two objects.  
*   :meth:`rayCast` and :meth:`rayCast_ext`: ray-cast queries.  

For most geometry types (TriangleMesh, PointCloud, ConvexHull), the first time
you perform a query, some collision detection data structures will be
initialized. This preprocessing step can take some time for complex geometries.  

**Collision margins**  

Each object also has a \"collision margin\" which may virtually fatten the
object, as far as proximity queries are concerned. This is useful for setting
collision avoidance margins in motion planning. Use the
:meth:`setCollisionMargin` and :meth:`getCollisionMargin` methods to access the
margin. By default the margin is zero.  

.. note::  

    The geometry margin is NOT the same thing as simulation body collision
    padding!  All proximity queries are affected by the collision padding,
    inside or outside of simulation.
 **Conversions**  

Many geometry types can be converted to and from one another using the
:meth:`convert` method. This can also be used to remesh TriangleMesh,
PointCloud, ImplicitSurface, OccupancyGrid, and Heightmap objects.  

C++ includes: geometry.h
";

%feature("docstring") Geometry3D::Geometry3D "
";

%feature("docstring") Geometry3D::Geometry3D "
";

%feature("docstring") Geometry3D::Geometry3D "
";

%feature("docstring") Geometry3D::Geometry3D "
";

%feature("docstring") Geometry3D::Geometry3D "
";

%feature("docstring") Geometry3D::Geometry3D "
";

%feature("docstring") Geometry3D::Geometry3D "
";

%feature("docstring") Geometry3D::Geometry3D "
";

%feature("docstring") Geometry3D::Geometry3D "
";

%feature("docstring") Geometry3D::~Geometry3D "
";

%feature("docstring") Geometry3D::copy "

Creates a standalone geometry from this geometry.  
";

%feature("docstring") Geometry3D::set "

Copies the geometry of the argument into this geometry.  
";

%feature("docstring") Geometry3D::isStandalone "

Returns True if this is a standalone geometry.  
";

%feature("docstring") Geometry3D::free "

Frees the data associated with this geometry, if standalone.  
";

%feature("docstring") Geometry3D::type "

Returns the type of geometry: GeometricPrimitive, ConvexHull, TriangleMesh,
PointCloud, ImplicitSurface, OccupancyGrid, Heightmap, or Group.  
";

%feature("docstring") Geometry3D::empty "

Returns True if this has no contents (not the same as numElements()==0)  
";

%feature("docstring") Geometry3D::getTriangleMesh "

Returns a TriangleMesh if this geometry is of type TriangleMesh.  
";

%feature("docstring") Geometry3D::getPointCloud "

Returns a PointCloud if this geometry is of type PointCloud.  
";

%feature("docstring") Geometry3D::getGeometricPrimitive "

Returns a GeometricPrimitive if this geometry is of type GeometricPrimitive.  
";

%feature("docstring") Geometry3D::getConvexHull "

Returns a ConvexHull if this geometry is of type ConvexHull.  
";

%feature("docstring") Geometry3D::getImplicitSurface "

Returns the ImplicitSurface if this geometry is of type ImplicitSurface.  
";

%feature("docstring") Geometry3D::getOccupancyGrid "

Returns the OccupancyGrid if this geometry is of type OccupancyGrid.  
";

%feature("docstring") Geometry3D::getHeightmap "

Returns the Heightmap if this geometry is of type Heightmap.  
";

%feature("docstring") Geometry3D::setTriangleMesh "

Sets this Geometry3D to a TriangleMesh.  
";

%feature("docstring") Geometry3D::setPointCloud "

Sets this Geometry3D to a PointCloud.  
";

%feature("docstring") Geometry3D::setGeometricPrimitive "

Sets this Geometry3D to a GeometricPrimitive.  
";

%feature("docstring") Geometry3D::setConvexHull "

Sets this Geometry3D to a ConvexHull.  
";

%feature("docstring") Geometry3D::setConvexHullGroup "

Sets this Geometry3D to be a convex hull of two geometries. Note: the relative
transform of these two objects is frozen in place; i.e., setting the current
transform of g2 doesn't do anything to this object.  
";

%feature("docstring") Geometry3D::setImplicitSurface "

Sets this Geometry3D to an ImplicitSurface.  
";

%feature("docstring") Geometry3D::setOccupancyGrid "

Sets this Geometry3D to an OccupancyGrid.  
";

%feature("docstring") Geometry3D::setHeightmap "

Sets this Geometry3D to a Heightmap.  
";

%feature("docstring") Geometry3D::setGroup "

Sets this Geometry3D to a group geometry. To add sub-geometries, repeatedly call
setElement() with increasing indices.  
";

%feature("docstring") Geometry3D::getElement "

Returns an element of the Geometry3D if it is a Group, TriangleMesh, or
PointCloud. Raises an error if this is of any other type.  

The element will be in local coordinates.  
";

%feature("docstring") Geometry3D::setElement "

Sets an element of the Geometry3D if it is a Group, TriangleMesh, or PointCloud.
The element will be in local coordinates. Raises an error if this is of any
other type.  
";

%feature("docstring") Geometry3D::numElements "

Returns the number of sub-elements in this geometry.  
";

%feature("docstring") Geometry3D::loadFile "

Loads from file. Standard mesh types, PCD files, and .geom files are supported.  

Returns:  

    True on success, False on failure
  
";

%feature("docstring") Geometry3D::saveFile "

Saves to file. Standard mesh types, PCD files, and .geom files are supported.  
";

%feature("docstring") Geometry3D::setCurrentTransform "

Sets the current transformation (not modifying the underlying data)  
";

%feature("docstring") Geometry3D::getCurrentTransform "

Gets the current transformation.  

Return type: RigidTransform  
";

%feature("docstring") Geometry3D::translate "

Translates the geometry data. Permanently modifies the data and resets any
collision data structures.  
";

%feature("docstring") Geometry3D::scale "

Scales the geometry data uniformly. Permanently modifies the data and resets any
collision data structures.  
";

%feature("docstring") Geometry3D::scale "

Scales the geometry data with different factors on each axis. Permanently
modifies the data and resets any collision data structures.  
";

%feature("docstring") Geometry3D::rotate "

Rotates the geometry data. Permanently modifies the data and resets any
collision data structures.  
";

%feature("docstring") Geometry3D::transform "

Translates/rotates/scales the geometry data. Permanently modifies the data and
resets any collision data structures.  
";

%feature("docstring") Geometry3D::setCollisionMargin "

Sets a padding around the base geometry which affects the results of proximity
queries.  
";

%feature("docstring") Geometry3D::getCollisionMargin "

Returns the padding around the base geometry. Default 0.  
";

%feature("docstring") Geometry3D::getBB "

Returns an axis-aligned bounding box of the object as a tuple (bmin,bmax).  

Note: O(1) time, but may not be tight  

Return type: Tuple[Vector3,Vector3]  
";

%feature("docstring") Geometry3D::getBBTight "

Computes a tighter axis-aligned bounding box of the object than
:meth:`Geometry3D.getBB`. Worst case O(n) time.  

Return type: Tuple[Vector3,Vector3]  
";

%feature("docstring") Geometry3D::convert "

Converts a geometry to another type, if a conversion is available. The
interpretation of param depends on the type of conversion, with 0 being a
reasonable default.  

Available conversions are:  

*   TriangleMesh -> PointCloud. param is the desired dispersion of the points,
    by default set to the average triangle diameter. At least all of the mesh's
    vertices will be returned.  
*   TriangleMesh -> ImplicitSurface. Converted using the fast marching method
    with good results only if the mesh is watertight. param is the grid
    resolution, by default set to the average triangle diameter.  
*   TriangleMesh -> OccupancyGrid. Converted using rasterization. param is the
    grid resolution, by default set to the average triangle diameter.  
*   TriangleMesh -> ConvexHull. If param==0, just calculates a convex hull.
    Otherwise, uses convex decomposition with the HACD library.  
*   TriangleMesh -> Heightmap. Converted using rasterization. param is the grid
    resolution, by default set to max mesh dimension / 256.  
*   PointCloud -> TriangleMesh. Available if the point cloud is structured.
    param is the threshold for splitting triangles by depth discontinuity. param
    is by default infinity.  
*   PointCloud -> OccupancyGrid. param is the grid resolution, by default some
    reasonable number.  
*   PointCloud -> ConvexHull. Converted using SOLID / Qhull.  
*   PointCloud -> Heightmap. param is the grid resolution, by default set to max
    point cloud dimension / 256.  
*   GeometricPrimitive -> anything. param determines the desired resolution.  
*   ImplicitSurface -> TriangleMesh. param determines the level set for the
    marching cubes algorithm.  
*   ImplicitSurface -> PointCloud. param determines the level set.  
*   ImplicitSurface -> Heightmap.  
*   OccupancyGrid -> TriangleMesh. Creates a mesh around each block.  
*   OccupancyGrid -> PointCloud. Outputs a point for each block.  
*   OccupancyGrid -> Heightmap.  
*   ConvexHull -> TriangleMesh.  
*   ConvexHull -> PointCloud. param is the desired dispersion of the points.
    Equivalent to ConvexHull -> TriangleMesh -> PointCloud  
";

%feature("docstring") Geometry3D::contains_point "

Returns true if this geometry contains the given point.  

An approximate method is used for TriangleMesh. For PointCloud, the point is
considered to be contained if it is one of the points in the cloud, or if points
have a radius attribute, within the given radius.  
";

%feature("docstring") Geometry3D::collides "

Returns true if this geometry collides with the other.  

Unsupported types:  

*   ImplicitSurface - GeometricPrimitive [aabb, box, triangle, polygon]  
*   ImplicitSurface - ConvexHull  
";

%feature("docstring") Geometry3D::collides_ext "

Same as collide, but will also return the elements of each geometry that
collide.  

Returns: (elem1, elem2) where elem1 and elem2 are the indices of the elements
that collide. If len(elem1) == 0, then there is no detected collision.  

Return type: Tuple[list,list]  
";

%feature("docstring") Geometry3D::withinDistance "

Returns true if this geometry is within distance `tol` to other.  
";

%feature("docstring") Geometry3D::withinDistance_ext "

Same as withinDistance, but will also return the elements of each geometry that
are within distance tol.  

Returns: (elem1, elem2) where elem1 and elem2 are the indices of the elements
that are within distance tol. If len(elem1) == 0, then there is no detected
proximity.  

Return type: Tuple[list,list]  
";

%feature("docstring") Geometry3D::distance_simple "

Returns the distance from this geometry to the other. If either geometry
contains volume information, this value may be negative to indicate penetration.
See :meth:`Geometry3D.distance` for more information.  
";

%feature("docstring") Geometry3D::distance_point "

Returns the the distance and closest point to the input point, given in world
coordinates. An exception is raised if this operation is not supported with the
given geometry type.  

The return value contains the distance, closest points, and gradients if
available.  

For some geometry types, the signed distance is returned. The signed distance
returns the negative penetration depth if pt is within this. The following
geometry types return signed distances:  

*   GeometricPrimitive  
*   PointCloud  
*   ImplictSurface  
*   Heightmap (approximate, only accurate in the viewing direction)  
*   ConvexHull  

For other types, a signed distance will be returned if the geometry has a
positive collision margin, and the point penetrates less than this margin.  
";

%feature("docstring") Geometry3D::distance_point_ext "

A customizable version of :meth:`Geometry3D.distance_point`. The settings for
the calculation can be customized with relErr, absErr, and upperBound, e.g., to
break if the closest points are at least upperBound distance from one another.  
";

%feature("docstring") Geometry3D::distance "

Returns the the distance and closest points between the given geometries. This
may be either the normal distance or the signed distance, depending on the
geometry type.  

The normal distance returns 0 if the two objects are touching
(this.collides(other)=True).  

The signed distance returns the negative penetration depth if the objects are
touching. Only the following combinations of geometry types return signed
distances:  

*   GeometricPrimitive-GeometricPrimitive (missing some for boxes, segments, and
    tris)  
*   GeometricPrimitive-TriangleMesh (surface only)  
*   GeometricPrimitive-PointCloud  
*   GeometricPrimitive-ImplicitSurface  
*   TriangleMesh (surface only)-GeometricPrimitive  
*   PointCloud-ImplicitSurface  
*   PointCloud-ConvexHull  
*   ConvexHull-ConvexHull  
*   ConvexHull-GeometricPrimitive  

If penetration is supported, a negative distance is returned and cp1,cp2 are the
deepest penetrating points.  

Unsupported types:  

*   PointCloud-PointCloud  
*   ImplicitSurface-TriangleMesh  
*   ImplicitSurface-ImplicitSurface  
*   OccupancyGrid - anything  
*   ConvexHull - anything else besides ConvexHull  

See the comments of the distance_point function  
";

%feature("docstring") Geometry3D::distance_ext "

A customizable version of :meth:`Geometry3D.distance`. The settings for the
calculation can be customized with relErr, absErr, and upperBound, e.g., to
break if the closest points are at least upperBound distance from one another.  
";

%feature("docstring") Geometry3D::rayCast "

Performs a ray cast.  

All types supported, but PointCloud needs a positive collision margin, or points
need to have a 'radius' property assigned)  

Returns:  

    (hit,pt) where hit is true if the ray starting at s and pointing
    in direction d hits the geometry (given in world coordinates); pt is
    the hit point, in world coordinates.
 Return type: Tuple[bool,Vector3]  
";

%feature("docstring") Geometry3D::rayCast_ext "

A more sophisticated ray cast.  

All types supported, but PointCloud needs a positive collision  

margin, or points need to have a 'radius' property assigned) Returns:  

    (hit_element,pt) where hit_element is >= 0 if ray starting at
    s and pointing in direction d hits the geometry (given in world
    coordinates).  

    - hit_element is -1 if the object is not hit, otherwise it gives the
      index of the element (triangle, point, sub-object) that was hit.  
      For geometric primitives, this will be 0.
    - pt is the hit point, in world coordinates.
 Return type: Tuple[int,Vector3]  
";

%feature("docstring") Geometry3D::contacts "

Returns the set of contact points between this and other. This set is a discrete
representation of the region of surface overlap, which is defined as all pairs
of points within distance self.collisionMargin + other.collisionMargin +
padding1 + padding2.  

Relatively few geometry types are supported.  

For some geometry types (TriangleMesh-TriangleMesh, TriangleMesh-PointCloud,
PointCloud-PointCloud) padding must be positive to get meaningful contact poitns
and normals.  

If maxContacts != 0 a clustering postprocessing step is performed.  
";

%feature("docstring") Geometry3D::support "

Calculates the furthest point on this geometry in the direction dir.  

Supported types:  

*   ConvexHull  
*   GeometricPrimitive  
*   PointCloud  
*   TriangleMesh  
*   OccupancyGrid  

Return type: Vector3  
";

%feature("docstring") Geometry3D::slice "

Calculates a 2D slice through the data. The slice is given by the local X-Y
plane of a transform (R,T) with orientation R and translation t. The return
Geometry's data is in the local frame of (R,t), and (R,t) is set as its current
transform.  

The geometry's current transform is respected.  

O(N) time.  

Supported types:  

*   PointCloud. Needs tol > 0. A PointCloud is returned.  
*   TriangleMesh. tol is ignored. A Group of GeometricPrimitives (segments) is
    returned.  
";

%feature("docstring") Geometry3D::roi "

Calculates a region of interest of the data for the bounding box [bmin,bmax].
The geometry's current transform is respected.  

`query` can be \"intersect\", \"touching\", or \"within\". If \"intersect\",
this tries to get a representation of the geometry intersecting the box. If
\"touching\", all elements touching the box are returned. If \"within\", only
elements entirely inside the box are returned.  

`query` can also be prefaced with a '~' which indicates that the ROI should be
inverted, i.e. select everything that does NOT intersect with a box.  

O(N) time.  

Supported types:  

*   PointCloud  
*   TriangleMesh  
";

%feature("docstring") Geometry3D::merge "

Merges another geometry into this geometry. The result is stored inplace and the
type of the result is the same as this geometry. This can be used to calculate
the union of PointClouds, TriangleMeshes, ConvexPolytopes, and ImplicitSurfaces,
OccupancyGrids, and Heightmaps.  

ImplicitSurface, OccupancyGrid, and Heightmap merges preserve the domain of the
current grid. They can also be merged with many other geometries.  
";

// File: classHeightmap.xml


%feature("docstring") Heightmap "

A height (elevation) map or a depth map.  

In elevation-map form (`viewport.perspective=false`), the values are the
z-height of the terrain at each grid point. In depth-map form
(`viewport.perspective=true`), the values are the depths of each grid point (not
distance) from the origin in the +z direction.  

Note that unlike volume grid types, each grid entry is defined at a vertex, not
a cell. In elevation map form, the (i,j) cell is associated with the vertex
((i-cx)/fx,(n-1-j-cy)/fy,heights[i,j]) where n is `heights.shape[1]`. Note that
the y-axis is flipped in the heightmap compared to the viewport such that the
image is given in standard top-down convention.  

In depth map form, the (i,j) cell is associated with the vertex
(d*(i-cx)/fx,d*(j-cy)/fy,d) where d=heights[i,j]. Note that the y-axis is not
flipped in this convention.  

Attributes:  

    viewport (Viewport): contains the size (w,h), projection (perspective)
         intrinsics (fx,fy,cx,cy), and pose (pose) of the heightmap
         reference coordinate system.  Note that to change the viewport, you
         will need to use `vp = hm.viewport; vp.w = ...; hm.viewport = vp`.

    heights (np.ndarray): contains a 2D array of (w,h) values.

    colorImage (np.ndarray): contains a 3D image array of colors in
         grayscale (h,w), RGB (h,w,3), or RGBA (h,w,4) format.
  

C++ includes: geometry.h
";

%feature("docstring") Heightmap::Heightmap "
";

%feature("docstring") Heightmap::Heightmap "
";

%feature("docstring") Heightmap::~Heightmap "
";

%feature("docstring") Heightmap::copy "

Creates a standalone object that is a copy of this.  
";

%feature("docstring") Heightmap::set "

Copies the data of the argument into this.  
";

%feature("docstring") Heightmap::set "

Sets all elements to a uniform value (e.g., 0)  
";

%feature("docstring") Heightmap::set "

Sets the height of a vertex (note, indices are x and y units, which is reversed
from image convention)  
";

%feature("docstring") Heightmap::resize "

Resizes the height map.  
";

%feature("docstring") Heightmap::setSize "

Sets an orthographic projection (elevation map) with the given width and height.  
";

%feature("docstring") Heightmap::setFOV "

Sets an perspective projection (depth map) with the given x and y fields of view
and centered focal point. If fovy=-1, then it will be set so that pixels are
square.  
";

%feature("docstring") Heightmap::setIntrinsics "

Sets an perspective projection (depth map) with the given intrinsics fx, fy, cx,
cy. If cx or cy are negative, then cx = (w-1)/2, cy = (h-1)/2.  
";

%feature("docstring") Heightmap::isPerspective "

Returns true if the heightmaps is in perspective (depth map) mode.  
";

%feature("docstring") Heightmap::isOrthographic "

Returns true if the heightmaps is in orthographic (elevation map) mode.  
";

%feature("docstring") Heightmap::getViewport "

Retrieves the viewport.  
";

%feature("docstring") Heightmap::setViewport "

Sets the viewport.  
";

%feature("docstring") Heightmap::get "

Gets the height of a vertex (note, indices are x and y units, which is reversed
from image convention)  
";

%feature("docstring") Heightmap::shift "

Shifts the height uniformly.  
";

%feature("docstring") Heightmap::scale "

Scales the height uniformly.  
";

%feature("docstring") Heightmap::getHeights "

Returns a 2D Numpy array view of the values. Result has shape w x h and has
float32 dtype.  

Return type: np.ndarray  
";

%feature("docstring") Heightmap::setHeights_f "

Sets the values to a 2D numpy array of shape w x h.  
";

%feature("docstring") Heightmap::hasColors "

Returns true if colors are present.  
";

%feature("docstring") Heightmap::clearColors "

Erases all colors.  
";

%feature("docstring") Heightmap::setColor "

Sets a uniform grayscale color. Call this first if you want to start setting
colors.  
";

%feature("docstring") Heightmap::setColor "

Sets a uniform color. Call this first if you want to start setting colors.  
";

%feature("docstring") Heightmap::setColor "

Gets the grayscale color of a vertex (note, indices are x and y units, which is
reversed from image convention)  
";

%feature("docstring") Heightmap::setColor "

Gets the RGBA color of a vertex (note, indices are x and y units, which is
reversed from image convention)  
";

%feature("docstring") Heightmap::getColor "

Gets the RGBA color of a vertex (note, indices are x and y units, which is
reversed from image convention)  

Return type: Tuple[float,float,float,float]  
";

%feature("docstring") Heightmap::getColorImage "

Returns a 3D Numpy array view of the color image (h x w x (1, 3, or 4)), with
rows ordered top to bottom.  

Return type: np.ndarray  
";

%feature("docstring") Heightmap::setColorImage_b "

Sets the values to a 3D numpy array (h x w x 1, 3, or (1, 3, or 4)), with rows
ordered top to bottom.  
";

%feature("docstring") Heightmap::setColorImage_i "

Sets colors to a 32-bit RGBA image (size h x w) with rows ordered top to bottom.  
";

%feature("docstring") Heightmap::getColorImage_i "

Retrieves a 32-bit RGBA image of the heightmap's colors (h x w)  

Return type: np.ndarray  
";

%feature("docstring") Heightmap::getColorImage_d "

Retrieves a floating point RGB, RGBA, or L image (h x w x (1, 3, or 4)) with
rows ordered from top to bottom.  

Return type: np.ndarray  
";

%feature("docstring") Heightmap::numProperties "

Returns the number of properties.  
";

%feature("docstring") Heightmap::addProperty "

Adds a new property and sets it to 0.  
";

%feature("docstring") Heightmap::addProperty "

Adds a new property and sets it to an array of size (w x h)  
";

%feature("docstring") Heightmap::propertyIndex "

Retrieves the index associated with a property name.  
";

%feature("docstring") Heightmap::setProperty "

Retrieves a property index Sets an individual pixel's property vector.  
";

%feature("docstring") Heightmap::getProperty "

Retrieves an individual pixel's property vector.  

Return type: np.ndarray  
";

%feature("docstring") Heightmap::setProperties "

Sets a property to an array of size (w x h)  
";

%feature("docstring") Heightmap::getProperties "

Retrieves a view of the property of size (w x h)  

Return type: np.ndarray  
";

// File: classIKObjective.xml


%feature("docstring") IKObjective "

A class defining an inverse kinematic target. Either a link on a robot can take
on a fixed position/orientation in the world frame, or a relative
position/orientation to another frame.  

The positionScale and orientationScale attributes scale the solver's residual
vector. This affects whether the convergence tolerance is met, and also controls
the emphasis on each objective / component when the objective cannot be reached.
By default these are both 1.  

C++ includes: robotik.h
";

%feature("docstring") IKObjective::IKObjective "

With no arguments, constructs a blank IKObjective. Given an IKObjective, acts as
a copy constructor.  
";

%feature("docstring") IKObjective::IKObjective "

With no arguments, constructs a blank IKObjective. Given an IKObjective, acts as
a copy constructor.  
";

%feature("docstring") IKObjective::copy "

Copy constructor.  
";

%feature("docstring") IKObjective::link "

The index of the robot link that is constrained.  
";

%feature("docstring") IKObjective::destLink "

The index of the destination link, or -1 if fixed to the world.  
";

%feature("docstring") IKObjective::numPosDims "

Returns: The number of position dimensions constrained (0-3)  
";

%feature("docstring") IKObjective::numRotDims "

Returns: The number of rotation dimensions constrained (0-3)  
";

%feature("docstring") IKObjective::setFixedPoint "

Sets a fixed-point constraint.  
";

%feature("docstring") IKObjective::setFixedPoints "

Sets a multiple fixed-point constraint.  
";

%feature("docstring") IKObjective::setFixedTransform "

Sets a fixed-transform constraint (R,t)  
";

%feature("docstring") IKObjective::setRelativePoint "

Sets a fixed-point constraint relative to link2.  
";

%feature("docstring") IKObjective::setRelativePoints "

Sets a multiple fixed-point constraint relative to link2.  
";

%feature("docstring") IKObjective::setRelativeTransform "

Sets a fixed-transform constraint (R,t) relative to linkTgt.  
";

%feature("docstring") IKObjective::setLinks "

Manual construction.  
";

%feature("docstring") IKObjective::setFreePosition "

Deprecated: use setFreePosConstraint.  
";

%feature("docstring") IKObjective::setFreePosConstraint "

Manual: Sets a free position constraint.  
";

%feature("docstring") IKObjective::setFixedPosConstraint "

Manual: Sets a fixed position constraint.  
";

%feature("docstring") IKObjective::setPlanarPosConstraint "

Manual: Sets a planar position constraint nworld^T T(link)*tlocal + oworld = 0.  
";

%feature("docstring") IKObjective::setLinearPosConstraint "

Manual: Sets a linear position constraint T(link)*tlocal = sworld + u*dworld for
some real value u.  
";

%feature("docstring") IKObjective::setFreeRotConstraint "

Manual: Sets a free rotation constraint.  
";

%feature("docstring") IKObjective::setFixedRotConstraint "

Manual: Sets a fixed rotation constraint.  
";

%feature("docstring") IKObjective::setAxialRotConstraint "

Manual: Sets an axial rotation constraint.  
";

%feature("docstring") IKObjective::getPosition "

Returns the local and global position of the position constraint.  

Return type: Tuple[Vector3,Vector3]  
";

%feature("docstring") IKObjective::getPositionDirection "

For linear and planar constraints, returns the direction.  

Return type: Vector3  
";

%feature("docstring") IKObjective::getRotation "

For fixed rotation constraints, returns the orientation.  

Return type: Matrix3  
";

%feature("docstring") IKObjective::getRotationAxis "

For axis rotation constraints, returns the local and global axes.  

Return type: Tuple[Vector3,Vector3]  
";

%feature("docstring") IKObjective::getTransform "

For fixed-transform constraints, returns the transform (R,t)  

Return type: RigidTransform  
";

%feature("docstring") IKObjective::transform "

Tranforms the target position/rotation of this IK constraint by transform (R,t)  
";

%feature("docstring") IKObjective::transformLocal "

Tranforms the local position/rotation of this IK constraint by transform (R,t)  
";

%feature("docstring") IKObjective::matchDestination "

Sets the destination coordinates of this constraint to fit the given target
transform. In other words, if (R,t) is the current link transform, this sets the
destination position / orientation so that this objective has zero error. The
current position/rotation constraint types are kept.  
";

%feature("docstring") IKObjective::closestMatch "

Gets the transform T that's closest to the transform (R,t) and that satisfies
the IK goal's constraints.  

Return type: RigidTransform  
";

%feature("docstring") IKObjective::sampleTransform "

Returns a transformation (R,t) from link relative to link2, sampled at random
from the space of transforms that satisfies the objective obj.  

Return type: RigidTransform  
";

%feature("docstring") IKObjective::loadString "

Loads the objective from a Klamp't-native formatted string. For a more readable
but verbose format, try the JSON IO routines :meth:`klampt.io.loader.to_json` /
:meth:`klampt.io.loader.from_json`  
";

%feature("docstring") IKObjective::saveString "

Saves the objective to a Klamp't-native formatted string. For a more readable
but verbose format, try the JSON IO routines :meth:`klampt.io.loader.to_json` /
:meth:`klampt.io.loader.from_json`  
";

// File: classIKSolver.xml


%feature("docstring") IKSolver "

An inverse kinematics solver based on the Newton-Raphson technique.  

Typical calling pattern is::  

    s = IKSolver(robot)
    s.add(objective1)
    s.add(objective2)
    s.setMaxIters(100)
    s.setTolerance(1e-4)
    res = s.solve()
    if res:
        print(\"IK solution:\",robot.getConfig(),\"found in\",
            s.lastSolveIters(),\"iterations, residual\",s.getResidual())
    else:
        print(\"IK failed:\",robot.getConfig(),\"found in\",
            s.lastSolveIters(),\"iterations, residual\",s.getResidual())
  

C++ includes: robotik.h
";

%feature("docstring") IKSolver::IKSolver "

Initializes an IK solver. Given a RobotModel, an empty solver is created. Given
an IK solver, acts as a copy constructor.  
";

%feature("docstring") IKSolver::IKSolver "

Initializes an IK solver. Given a RobotModel, an empty solver is created. Given
an IK solver, acts as a copy constructor.  
";

%feature("docstring") IKSolver::copy "

Copy constructor.  
";

%feature("docstring") IKSolver::add "

Adds a new simultaneous objective.  
";

%feature("docstring") IKSolver::set "

Assigns an existing objective added by add.  
";

%feature("docstring") IKSolver::addSecondary "

Adds a new objective to the secondary objectives list.  
";

%feature("docstring") IKSolver::setSecondary "

Assigns an existing objective added by addsecondary.  
";

%feature("docstring") IKSolver::clear "

Clears objectives.  
";

%feature("docstring") IKSolver::setMaxIters "

Sets the max # of iterations (default 100)  
";

%feature("docstring") IKSolver::getMaxIters "

Returns the max # of iterations.  
";

%feature("docstring") IKSolver::setTolerance "

Sets the constraint solve tolerance (default 1e-3)  
";

%feature("docstring") IKSolver::getTolerance "

Returns the constraint solve tolerance.  
";

%feature("docstring") IKSolver::setActiveDofs "

Sets the active degrees of freedom.  
";

%feature("docstring") IKSolver::getActiveDofs "

Returns the active degrees of freedom.  

Return type: List[int]  
";

%feature("docstring") IKSolver::setJointLimits "

Sets limits on the robot's configuration. If empty, this turns off joint limits.  
";

%feature("docstring") IKSolver::getJointLimits "

Returns the limits on the robot's configuration (by default this is the robot's
joint limits.  

Return type: Tuple[Vector,Vector]  
";

%feature("docstring") IKSolver::setBiasConfig "

Biases the solver to approach a given configuration. Setting an empty vector
clears the bias term.  
";

%feature("docstring") IKSolver::getBiasConfig "

Returns the solvers' bias configuration.  

Return type: Vector  
";

%feature("docstring") IKSolver::isSolved "

Returns True if the current configuration residual is less than tol.  
";

%feature("docstring") IKSolver::getResidual "

Returns the vector describing the error of the objective at the current
configuration.  

Return type: Vector  
";

%feature("docstring") IKSolver::getJacobian "

Computes the matrix describing the instantaneous derivative of the objective
with respect to the active Dofs.  

Return type: np.ndarray  
";

%feature("docstring") IKSolver::getSecondaryResidual "

Returns the vector describing the error of the secondary objective at the
current configuration.  

Return type: Vector  
";

%feature("docstring") IKSolver::solve "

Tries to find a configuration that satifies all simultaneous objectives up to
the desired tolerance.  

All of the primary and the secondary objectives are solved simultaneously.  

Returns:  

    True if x converged.
  
";

%feature("docstring") IKSolver::minimize "

Tries to find a configuration that satifies all simultaneous objectives up to
the desired tolerance, or minimizes the residual if they cannot be met. (Only
the last override is documented...)  
";

%feature("docstring") IKSolver::minimize "

Tries to find a configuration that satifies all simultaneous objectives up to
the desired tolerance or minimizes the residual.  

The relation to `:func:solve` is that `solve` uses a root-finding method that
tries indirectly to minimize the residual, but it may stall out when the
objectives are infeasible.  

If secondary objectives are specified, this tries to minimize them once the
primary objectives are satisfied, i.e., it will minimize on the solution
manifold of the primary constraints.  

There are two flavors of secondary objectives. If no arguments are given, then
any constraints added via `addSecondary` will have their residuals minimized.  

If the user provides a pair of functions `(f,grad)`, then a custom objective is
specified. Here, `f(q)` is the secondary objective to minimize and `grad(q)` its
gradient. This will override the secondary objectives added via `addSecondary`.
Specifically, q is a function of all robot DOFs, and `grad(q)` should return a
list or tuple of length `len(q)``.  

.. note::  

    The minimization will occur only over the current active DOFs, which will
    include default active DOFs for secondary objectives.  

Args: secondary_objective (callable): a function `f(q)->float` that should be
minimized. secondary_objective_grad (callable): a function `grad(q)->`sequence
of length `len(q)` giving the gradient of `f` at `q`.  

Returns:  

    True if x converged on the primary objectives.
  
";

%feature("docstring") IKSolver::lastSolveIters "

Returns the number of Newton-Raphson iterations used in the last solve() call or
the number of Quasi-Newton iterations used in the last minimize() call.  
";

%feature("docstring") IKSolver::sampleInitial "

Samples an initial random configuration. More initial configurations can be
sampled in case the prior configs lead to local minima.  
";

// File: classImplicitSurface.xml


%feature("docstring") ImplicitSurface "

An axis-aligned volumetric grid representing a signed distance transform with >
0 indicating outside and < 0 indicating inside.  

In general, values are associated with cells rather than vertices.  

Cell (i,j,k) contains a value, and has size (w,d,h) =
((bmax[0]-bmin[0])/dims[0], (bmax[1]-bmin[1])/dims[1],
(bmax[2]-bmin[2])/dims[2]). It ranges over the box [w*i,w*(i+1)) x [d*j,d*(j+1))
x [h*k,h*(k+1)).  

The field should be assumed sampled at the **centers** of cells, i.e., at
(w*(i+1/2),d*(j+1/2),h*(k+1/2)).  

Attributes:  

    bmin (array of 3 doubles): contains the minimum bounds.

    bmax (array of 3 doubles): contains the maximum bounds.

    values (numpy array): contains a 3D array of
         ``dims[0] x dims[1] x dims[2]`` values.

    truncationDistance (float): inf for SDFs, and the truncation distance
         for TSDFs. Cells whose values are >= d are considered \"sufficiently
         far\" and distance / tolerance queries outside of this range are
         usually not meaningful.
  

C++ includes: geometry.h
";

%feature("docstring") ImplicitSurface::ImplicitSurface "
";

%feature("docstring") ImplicitSurface::ImplicitSurface "
";

%feature("docstring") ImplicitSurface::~ImplicitSurface "
";

%feature("docstring") ImplicitSurface::copy "

Creates a standalone object that is a copy of this.  
";

%feature("docstring") ImplicitSurface::set "

Copies the data of the argument into this.  
";

%feature("docstring") ImplicitSurface::set "

Sets all elements to a uniform value (e.g., 0)  
";

%feature("docstring") ImplicitSurface::set "

Sets a specific element of a cell.  
";

%feature("docstring") ImplicitSurface::getBmin "
";

%feature("docstring") ImplicitSurface::setBmin "
";

%feature("docstring") ImplicitSurface::getBmax "
";

%feature("docstring") ImplicitSurface::setBmax "
";

%feature("docstring") ImplicitSurface::resize "

Resizes the x, y, and z dimensions of the grid.  
";

%feature("docstring") ImplicitSurface::get "

Gets a specific element of a cell.  
";

%feature("docstring") ImplicitSurface::shift "

Shifts the value uniformly.  
";

%feature("docstring") ImplicitSurface::scale "

Scales the value uniformly.  
";

%feature("docstring") ImplicitSurface::getValues "

Returns a 3D Numpy array view of the values.  

Return type: np.ndarray  
";

%feature("docstring") ImplicitSurface::setValues "

Sets the values to a 3D numpy array.  
";

%feature("docstring") ImplicitSurface::setTruncationDistance "

Sets the truncation distance for TSDFs.  
";

%feature("docstring") ImplicitSurface::getTruncationDistance "

Retrieves the truncation distance for TSDFs.  
";

// File: classMass.xml


%feature("docstring") Mass "

Stores mass information for a rigid body or robot link.  

.. note::  

    Recommended to use the set/get functions rather than changing the members
    directly due to strangeness in SWIG's handling of vectors.  

.. note:  

    The inertia matrix is specified in the local frame of the object
    and centered at the center of mass.  

Attributes:  

    mass (float): the actual mass (typically in kg)
    com (list of 3 floats): the center of mass position, in
        local coordinates.
    inertia (list of 3 floats or 9 floats): the inertia matrix
        in local coordinates.  If 3 floats, this is a diagonal matrix.
        If 9 floats, this gives all entries of the 3x3 inertia matrix
        (in column major or row major order, it doesn't matter since
        inertia matrices are symmetric)
  

C++ includes: robotmodel.h
";

%feature("docstring") Mass::Mass "
";

%feature("docstring") Mass::setMass "
";

%feature("docstring") Mass::getMass "
";

%feature("docstring") Mass::setCom "
";

%feature("docstring") Mass::getCom "

Returns the COM.  

Return type: Vector3  
";

%feature("docstring") Mass::setInertia "

Sets an inertia matrix.  
";

%feature("docstring") Mass::getInertia "

Returns the inertia matrix as a list of 3 floats or 9 floats.  

Return type: Vector  
";

%feature("docstring") Mass::estimate "

Estimates the com and inertia of a geometry, with a given total mass.  

For TriangleMesh types, surfaceFraction dictates how much of the object's mass
is concentrated at the surface rather than the interior.  
";

// File: classObjectPoser.xml


%feature("docstring") ObjectPoser "
";

%feature("docstring") ObjectPoser::ObjectPoser "
";

%feature("docstring") ObjectPoser::set "
";

%feature("docstring") ObjectPoser::get "
";

// File: classOccupancyGrid.xml


%feature("docstring") OccupancyGrid "

An occupancy grid with 1 indicating inside and 0 indicating outside. Can also be
a fuzzy (probabilistic / density) grid.  

In general, values are associated with cells rather than vertices.  

Cell (i,j,k) contains an occupancy / density value, and has size (w,d,h) =
((bmax[0]-bmin[0])/dims[0], (bmax[1]-bmin[1])/dims[1],
(bmax[2]-bmin[2])/dims[2]). It ranges over the box [w*i,w*(i+1)) x [d*j,d*(j+1))
x [h*k,h*(k+1)).  

Attributes:  

    bmin (array of 3 doubles): contains the minimum bounds.

    bmax (array of 3 doubles): contains the maximum bounds.

    values (numpy array): contains a 3D array of
         ``dims[0] x dims[1] x dims[2]`` values.

    occupancyThreshold (float): set to 0.5 by default. Collision
        detection treats all cells whose values are greater than this
        value as occupied.
  

C++ includes: geometry.h
";

%feature("docstring") OccupancyGrid::OccupancyGrid "
";

%feature("docstring") OccupancyGrid::OccupancyGrid "
";

%feature("docstring") OccupancyGrid::~OccupancyGrid "
";

%feature("docstring") OccupancyGrid::copy "

Creates a standalone object that is a copy of this.  
";

%feature("docstring") OccupancyGrid::set "

Copies the data of the argument into this.  
";

%feature("docstring") OccupancyGrid::set "

Sets all elements to a uniform value (e.g., 0)  
";

%feature("docstring") OccupancyGrid::set "

Sets a specific element of a cell.  
";

%feature("docstring") OccupancyGrid::getBmin "
";

%feature("docstring") OccupancyGrid::setBmin "
";

%feature("docstring") OccupancyGrid::getBmax "
";

%feature("docstring") OccupancyGrid::setBmax "
";

%feature("docstring") OccupancyGrid::resize "

Resizes the x, y, and z dimensions of the grid.  
";

%feature("docstring") OccupancyGrid::get "

Gets a specific element of a cell.  
";

%feature("docstring") OccupancyGrid::shift "

Shifts the value uniformly.  
";

%feature("docstring") OccupancyGrid::scale "

Scales the value uniformly.  
";

%feature("docstring") OccupancyGrid::getValues "

Returns a 3D Numpy array view of the values.  

Return type: np.ndarray  
";

%feature("docstring") OccupancyGrid::setValues "

Sets the values to a 3D numpy array.  
";

%feature("docstring") OccupancyGrid::setOccupancyThreshold "

Sets the threshold for collision detection.  
";

%feature("docstring") OccupancyGrid::getOccupancyThreshold "

Gets the threshold for collision detection.  
";

// File: classPlannerInterface.xml


%feature("docstring") PlannerInterface "

An interface for a kinematic motion planner. The :class:`MotionPlan` interface
in cspace.py is somewhat easier to use.  

On construction, uses the planner type specified by setPlanType and the settings
currently specified by calls to setPlanSetting.  

Point-to-point planning is enabled by sending two configurations to the
setEndpoints method. This is mandatory for RRT and SBL-style planners. The start
and end milestones are given by indices 0 and 1, respectively  

Point-to-set planning is enabled by sending a *goal test* as the second argument
to the setEndpoints method. It is possible also to send a special goal sampler
by providing a *pair of functions* as the second argument consisting of the two
functions (goaltest,goalsample). The first in this pair tests whether a
configuration is a goal, and the second returns a sampled configuration in a
superset of the goal. Ideally the goal sampler generates as many goals as
possible.  

To plan, call planMore(iters) until getPath(0,1) returns non-NULL. The return
value is a list of configurations.  

Some planners can be used multi-query mode (such as PRM). In multi-query mode,
you may call addMilestone(q) to add a new milestone. addMilestone() returns the
index of that milestone, which can be used in later calls to getPath().  

In point-to-set mode, getSolutionPath will return the optimal path to any goal
milestone.  

All planners work with the standard path-length objective function. Some
planners can work with other cost functions, and you can use setCostFunction to
set the edge / terminal costs. Usually, the results will only be optimal on the
computed graph, and the graph is not specifically computed to optimize that
cost.  

To get a roadmap (V,E), call getRoadmap(). V is a list of configurations (each
configuration is a Python list) and E is a list of edges (each edge is a pair
(i,j) indexing into V).  

To dump the roadmap to disk, call dump(fn). This saves to a Trivial Graph Format
(TGF) format.  

C++ includes: motionplanning.h
";

%feature("docstring") PlannerInterface::PlannerInterface "
";

%feature("docstring") PlannerInterface::~PlannerInterface "
";

%feature("docstring") PlannerInterface::destroy "
";

%feature("docstring") PlannerInterface::setEndpoints "
";

%feature("docstring") PlannerInterface::setEndpointSet "
";

%feature("docstring") PlannerInterface::setCostFunction "
";

%feature("docstring") PlannerInterface::addMilestone "
";

%feature("docstring") PlannerInterface::getClosestMilestone "
";

%feature("docstring") PlannerInterface::getMilestone "
";

%feature("docstring") PlannerInterface::planMore "
";

%feature("docstring") PlannerInterface::getSolutionPath "
";

%feature("docstring") PlannerInterface::getPath "
";

%feature("docstring") PlannerInterface::getPath "
";

%feature("docstring") PlannerInterface::getData "
";

%feature("docstring") PlannerInterface::getStats "
";

%feature("docstring") PlannerInterface::getRoadmap "
";

%feature("docstring") PlannerInterface::dump "
";

// File: structPointCloud.xml


%feature("docstring") PointCloud "

A 3D point cloud class.  

Attributes:  

    vertices (numpy array): a n x 3 array of vertices.
    properties (numpy array): a n x k array of vertex properties.  The
       name of each property is either anonymous or retrieved by
       `getPropertyName`.  

Property names are usually lowercase but follow PCL naming convention, and often
include:  

*   `normal_x`, `normal_y`, `normal_z`: the outward normal  
*   `rgb`, `rgba`: integer encoding of RGB (24 bit int, format 0xrrggbb) or RGBA
    color (32 bit int, format 0xaarrggbb)  
*   `opacity`: opacity, in range [0,1]  
*   `c`: opacity, in range [0,255]  
*   `r,g,b,a`: color channels, in range [0,1]  
*   `u,v`: texture coordinate  
*   `radius`: treats the point cloud as a collection of balls  

Settings are usually lowercase but follow PCL naming convention, and often
include:  

*   `version`: version of the PCL file, typically \"0.7\"  
*   `id`: integer id  
*   `width`: the width (in pixels) of a structured point cloud  
*   `height`: the height (in pixels) of a structured point cloud  
*   `viewpoint`: Camera position and orientation in the form `ox oy oz qw qx qy
    qz`, with (ox,oy,oz) the focal point and (qw,qx,qy,qz) the quaternion
    representation of the orientation (canonical representation, with X right, Y
    down, Z forward).  

Examples::  

    pc = PointCloud()
    pc.addProperty('rgb')
    #add 1 point with coordinates (0,0,0) and color #000000 (black)
    pc.addPoint((0,0,0))
    pc.setProperty(0,0)
    print(len(pc.points)) # prints 1
    #add another point with coordinates (1,2,3)
    pc.addPoint([1,2,3])
    #this prints 2
    print(len(pc.points))
    print(pc.points)   #prints [[0,0,0],[1,2,3]]
    #this prints (2,1), because there is 1 property category x 2 points
    print(pc.properties.shape)
    #this prints 0; this is the default value added when addPoint was called
    print(pc.getProperty(1,0) )
  

C++ includes: geometry.h
";

%feature("docstring") PointCloud::PointCloud "
";

%feature("docstring") PointCloud::PointCloud "
";

%feature("docstring") PointCloud::~PointCloud "
";

%feature("docstring") PointCloud::copy "

Creates a standalone object that is a copy of this.  
";

%feature("docstring") PointCloud::set "

Copies the data of the argument into this.  
";

%feature("docstring") PointCloud::getPoints "

Returns a view of the points.  

Returns:  

    ndarray: an nx3 Numpy array. Setting elements of this array will
    change the points.
 Return type: np.ndarray  
";

%feature("docstring") PointCloud::setPoints "

Sets all the points to the given nx3 Numpy array.  
";

%feature("docstring") PointCloud::addPoint "

Adds a point. Sets all its properties to 0.  

Slow if properties are already set. Setting the points and properties as
matrices is faster.  

Returns the point's index.  
";

%feature("docstring") PointCloud::setPoint "

Sets the position of the point at the given index to p.  
";

%feature("docstring") PointCloud::getPoint "

Returns the position of the point at the given index.  

Return type: Vector3  
";

%feature("docstring") PointCloud::numProperties "

Returns the number of properties.  
";

%feature("docstring") PointCloud::setPointsAndProperties "

Sets all the points and m properties from the given n x (3+k) array.  
";

%feature("docstring") PointCloud::setProperties "

Sets all the properties of all points to the given nxk array.  
";

%feature("docstring") PointCloud::getProperties "

Returns all the properties of all points as an array view.  

Returns:  

    ndarray: an nxk Numpy array. Setting elements of this array will
    change the vertices.
 Return type: np.ndarray  
";

%feature("docstring") PointCloud::addProperty "

Adds a new property. All values for this property are set to 0.  
";

%feature("docstring") PointCloud::addProperty "

Adds a new property with name pname, and sets values for this property to the
given length-n array.  
";

%feature("docstring") PointCloud::setPropertyName "

Sets the name of a given property.  
";

%feature("docstring") PointCloud::getPropertyName "

Returns the name of a given property.  
";

%feature("docstring") PointCloud::propertyIndex "

Returns the index of a named property or -1 if it does not exist.  
";

%feature("docstring") PointCloud::setProperty "

Sets property pindex of point index to the given value.  
";

%feature("docstring") PointCloud::setProperty "

Sets the property named pname of point index to the given value.  
";

%feature("docstring") PointCloud::getProperty "

Returns property pindex of point index.  
";

%feature("docstring") PointCloud::getProperty "

Returns the property named pname of point index.  
";

%feature("docstring") PointCloud::translate "

Translates all the points by v=v+t.  
";

%feature("docstring") PointCloud::transform "

Transforms all the points by the rigid transform v=R*v+t.  
";

%feature("docstring") PointCloud::join "

Adds the given point cloud to this one. They must share the same properties or
else an exception is raised.  
";

%feature("docstring") PointCloud::setSetting "

Sets the given setting.  
";

%feature("docstring") PointCloud::getSetting "

Returns the given setting.  
";

%feature("docstring") PointCloud::setDepthImage_d "

Sets a structured point cloud from a depth image. [fx,fy,cx,cy] are the
intrinsics parameters. The depth is given as a size hxw array, top to bottom.  
";

%feature("docstring") PointCloud::setDepthImage_f "

Sets a structured point cloud from a depth image. [fx,fy,cx,cy] are the
intrinsics parameters. The depth is given as a size hxw array, top to bottom.  
";

%feature("docstring") PointCloud::setDepthImage_s "

Sets a structured point cloud from a depth image. [fx,fy,cx,cy] are the
intrinsics parameters. The depth is given as a size hxw array, top to bottom.  
";

%feature("docstring") PointCloud::setRGBDImages_i_d "

Sets a structured point cloud from an RGBD (color,depth) image pair.
[fx,fy,cx,cy] are the intrinsics parameters. The RGB colors are packed in
0xrrggbb order, size hxw, top to bottom.  
";

%feature("docstring") PointCloud::setRGBDImages_i_f "

Sets a structured point cloud from an RGBD (color,depth) image pair.
[fx,fy,cx,cy] are the intrinsics parameters. The RGB colors are packed in
0xrrggbb order, size hxw, top to bottom.  
";

%feature("docstring") PointCloud::setRGBDImages_i_s "

Sets a structured point cloud from an RGBD (color,depth) image pair.
[fx,fy,cx,cy] are the intrinsics parameters. The RGB colors are packed in
0xrrggbb order, size hxw, top to bottom.  
";

%feature("docstring") PointCloud::setRGBDImages_b_d "

Sets a structured point cloud from an RGBD (color,depth) image pair.
[fx,fy,cx,cy] are the intrinsics parameters. The RGB colors are packed in
0xrrggbb order, size hxw, top to bottom.  
";

%feature("docstring") PointCloud::setRGBDImages_b_f "

Sets a structured point cloud from an RGBD (color,depth) image pair.
[fx,fy,cx,cy] are the intrinsics parameters. The RGB colors are an h x w x 3
array, top to bottom.  
";

%feature("docstring") PointCloud::setRGBDImages_b_s "

Sets a structured point cloud from an RGBD (color,depth) image pair.
[fx,fy,cx,cy] are the intrinsics parameters. The RGB colors are an h x w x 3
array, top to bottom.  
";

// File: classPointPoser.xml


%feature("docstring") PointPoser "
";

%feature("docstring") PointPoser::PointPoser "
";

%feature("docstring") PointPoser::set "
";

%feature("docstring") PointPoser::get "
";

%feature("docstring") PointPoser::setAxes "

Sets the reference axes (by default aligned to x,y,z)  
";

%feature("docstring") PointPoser::enableAxes "
";

// File: classRigidObjectModel.xml


%feature("docstring") RigidObjectModel "

A rigid movable object.  

A rigid object has a name, geometry, appearance, mass, surface properties, and
current transform / velocity.  

State is retrieved/set using get/setTransform, and get/setVelocity  

C++ includes: robotmodel.h
";

%feature("docstring") RigidObjectModel::RigidObjectModel "
";

%feature("docstring") RigidObjectModel::loadFile "

Loads the object from the file fn.  
";

%feature("docstring") RigidObjectModel::saveFile "

Saves the object to the file fn. If geometryName is given, the geometry is saved
to that file.  
";

%feature("docstring") RigidObjectModel::getID "

Returns the ID of the rigid object in its world.  

.. note::  

    The world ID is not the same as the rigid object index.
  
";

%feature("docstring") RigidObjectModel::getName "
";

%feature("docstring") RigidObjectModel::setName "
";

%feature("docstring") RigidObjectModel::geometry "

Returns a reference to the geometry associated with this object.  
";

%feature("docstring") RigidObjectModel::appearance "

Returns a reference to the appearance associated with this object.  
";

%feature("docstring") RigidObjectModel::getMass "

Returns a copy of the Mass of this rigid object.  

.. note::  

    To change the mass properties, you should call ``m=object.getMass()``,
    change the desired properties in m, and then ``object.setMass(m)``
  
";

%feature("docstring") RigidObjectModel::setMass "
";

%feature("docstring") RigidObjectModel::getContactParameters "

Returns a copy of the ContactParameters of this rigid object.  

.. note::  

    To change the contact parameters, you should call
    ``p=object.getContactParameters()``, change the desired properties in
    p, and then call ``object.setContactParameters(p)``
  
";

%feature("docstring") RigidObjectModel::setContactParameters "
";

%feature("docstring") RigidObjectModel::getTransform "

Retrieves the rotation / translation of the rigid object (R,t)  

Returns:  

    se3 object: a pair (R,t), with R a 9-list and t a 3-list of floats,
    giving the transform to world coordinates.
 Return type: RigidTransform  
";

%feature("docstring") RigidObjectModel::setTransform "

Sets the rotation / translation (R,t) of the rigid object.  
";

%feature("docstring") RigidObjectModel::getVelocity "

Retrieves the (angular velocity, velocity) of the rigid object.  

Returns:  

    A pair of 3-lists (w,v) where w is the angular velocity
    vector and v is the translational velocity vector (both in world
    coordinates)
 Return type: Tuple[Vector3,Vector3]  
";

%feature("docstring") RigidObjectModel::setVelocity "

Sets the (angular velocity, velocity) of the rigid object.  
";

%feature("docstring") RigidObjectModel::drawGL "

Draws the object's geometry. If keepAppearance=true, the current appearance is
honored. Otherwise, only the raw geometry is drawn.  

PERFORMANCE WARNING: if keepAppearance is false, then this does not properly
reuse OpenGL display lists. A better approach is to change the object's
Appearance directly.  
";

// File: classRobotModel.xml


%feature("docstring") RobotModel "

A model of a dynamic and kinematic robot.  

Stores both constant information, like the reference placement of the links,
joint limits, velocity limits, etc, as well as a *current configuration* and
*current velocity* which are state-dependent. Several functions depend on the
robot's current configuration and/or velocity. To update that, use the
setConfig() and setVelocity() functions. setConfig() also update's the robot's
link transforms via forward kinematics. You may also use setDOFPosition and
setDOFVelocity for individual changes, but these are more expensive because each
call updates all of the affected the link transforms.  

It is important to understand that changing the configuration of the model
doesn't actually send a command to the physical / simulated robot. Moreover, the
model does not automatically get updated when the physical / simulated robot
moves. In essence, the model maintains temporary storage for performing
kinematics, dynamics, and planning computations, as well as for visualization.  

The state of the robot is retrieved using getConfig/getVelocity calls, and is
set using setConfig/setVelocity. Because many routines change the robot's
configuration, like IK and motion planning, a common design pattern is to
save/restore the configuration as follows::  

    q = robot.getConfig()
    do some stuff that may touch the robot's configuration...
    robot.setConfig(q)  

The model maintains configuration/velocity/acceleration/torque limits. However,
these are not enforced by the model, so you can happily set configurations
outside the limits. Valid commands must rather be enforced by the planner /
controller / simulator.  

As elsewhere in Klampt, the mapping between links and drivers is not one-to one.
A driver is essentially an actuator and transmission, and for most links a link
is driven by a unique driver (e.g., a motor and gearbox). However, there do
exist certain cases in which a link is not driven at all (e.g., the 6 virtual
links of a floating-base robot), or multiple links are driven by a single
actuator (e.g., a parallel-bar mechanism or a compliant hand). There are also
unusual drivers that introduce underactuated dynamics into the system, such as a
differential drive or Dubin's car mobile base. Care must be taken when sending
commands to motor controllers (e.g., Klampt Robot Interface Layer), which often
work in the actuator space rather than joint space. (See
:func:`configToDrivers`, :func:`configFromDrivers`, :func:`velocityToDrivers`,
:func:`velocityFromDrivers`).  

C++ includes: robotmodel.h
";

%feature("docstring") RobotModel::RobotModel "
";

%feature("docstring") RobotModel::loadFile "

Loads the robot from the file fn.  

Returns:  

    True if successful, False if failed.
  
";

%feature("docstring") RobotModel::saveFile "

Saves the robot to the file fn. Geometries may be saved as well.  

If `geometryPrefix == None` (default), the geometry is not saved. Otherwise, the
geometry of each link will be saved to files named `geometryPrefix+name`, where
`name` is either the name of the geometry file that was loaded, or
`[link_name].off`  
";

%feature("docstring") RobotModel::getID "

Returns the ID of the robot in its world.  

.. note::  

    The world ID is not the same as the robot index.
  
";

%feature("docstring") RobotModel::getName "

Gets the name of the robot.  
";

%feature("docstring") RobotModel::setName "

Sets the name of the robot.  
";

%feature("docstring") RobotModel::numLinks "

Returns the number of links = number of DOF's.  
";

%feature("docstring") RobotModel::link "

Returns a reference to the link by index or name.  
";

%feature("docstring") RobotModel::link "

Returns a reference to the link by index or name.  
";

%feature("docstring") RobotModel::numDrivers "

Returns the number of drivers.  
";

%feature("docstring") RobotModel::driver "

Returns a reference to the driver by index or name.  
";

%feature("docstring") RobotModel::driver "

Returns a reference to the driver by index or name.  
";

%feature("docstring") RobotModel::getJointType "

Returns the joint type of the joint connecting the link to its parent, where the
link is identified by index or by name.  
";

%feature("docstring") RobotModel::getJointType "

Returns the joint type of the joint connecting the link to its parent, where the
link is identified by index or by name.  
";

%feature("docstring") RobotModel::getConfig "

Retrieves the current configuration of the robot model.  

Return type: Vector  
";

%feature("docstring") RobotModel::getVelocity "

Retreives the current velocity of the robot model.  

Return type: Vector  
";

%feature("docstring") RobotModel::setConfig "

Sets the current configuration of the robot. Input q is a vector of length
numLinks(). This also updates forward kinematics of all links.  

Again, it is important to realize that the RobotModel is not the same as a
simulated robot, and this will not change the simulation world. Many functions
such as IK and motion planning use the RobotModel configuration as a temporary
variable, so if you need to keep the configuration through a robot-modifying
function call, you should call `q = robot.getConfig()` before the call, and then
`robot.setConfig(q)` after it.  
";

%feature("docstring") RobotModel::setVelocity "

Sets the current velocity of the robot model. Like the configuration, this is
also essentially a temporary variable.  
";

%feature("docstring") RobotModel::getJointLimits "

Returns a pair (qmin,qmax) of min/max joint limit vectors.  

Return type: Tuple[Vector,Vector]  
";

%feature("docstring") RobotModel::setJointLimits "

Sets the min/max joint limit vectors (must have length numLinks())  
";

%feature("docstring") RobotModel::getVelocityLimits "

Returns the velocity limit vector vmax, the constraint is :math:`|dq[i]| \\leq
vmax[i]`  

Return type: Vector  
";

%feature("docstring") RobotModel::setVelocityLimits "

Sets the velocity limit vector vmax, the constraint is :math:`|dq[i]| \\leq
vmax[i]`  
";

%feature("docstring") RobotModel::getAccelerationLimits "

Returns the acceleration limit vector amax, the constraint is :math:`|ddq[i]|
\\leq amax[i]`  

Return type: Vector  
";

%feature("docstring") RobotModel::setAccelerationLimits "

Sets the acceleration limit vector amax, the constraint is :math:`|ddq[i]| \\leq
amax[i]`  
";

%feature("docstring") RobotModel::getTorqueLimits "

Returns the torque limit vector tmax, the constraint is :math:`|torque[i]| \\leq
tmax[i]`  

Return type: Vector  
";

%feature("docstring") RobotModel::setTorqueLimits "

Sets the torque limit vector tmax, the constraint is :math:`|torque[i]| \\leq
tmax[i]`  
";

%feature("docstring") RobotModel::setDOFPosition "

Sets a single DOF's position (by index or by name).  

.. note::  

    If you are setting several joints at once, use setConfig because this
 function computes forward kinematics for all descendant links each time it is
called.  
";

%feature("docstring") RobotModel::setDOFPosition "

Sets a single DOF's position (by index or by name).  

.. note::  

    If you are setting several joints at once, use setConfig because this
    function computes forward kinematics for all descendant links each time
    it is called.
  
";

%feature("docstring") RobotModel::getDOFPosition "

Returns a single DOF's position.  
";

%feature("docstring") RobotModel::getDOFPosition "

Returns a single DOF's position (by name)  
";

%feature("docstring") RobotModel::getCom "

Returns the 3D center of mass at the current config.  

Return type: Vector3  
";

%feature("docstring") RobotModel::getComVelocity "

Returns the 3D velocity of the center of mass at the current config / velocity.  

Return type: Vector3  
";

%feature("docstring") RobotModel::getComJacobian "

Computes the Jacobian matrix of the current center of mass.  

Returns:  

    ndarray: a 3xn matrix J such that np.dot(J,dq) gives the
    COM velocity at the currene configuration
 Return type: np.ndarray  
";

%feature("docstring") RobotModel::getComJacobianCols "

Returns the Jacobian matrix of the current center of mass w.r.t. some links of
the robot.  

Returns:  

    ndarray: a 3xlen(links) matrix J such that np.dot(J,dqlinks)
    gives the COM velocity at the current configuration, and dqlinks
    is the array of velocities of the links given by `links`
 Return type: np.ndarray  
";

%feature("docstring") RobotModel::getLinearMomentum "

Computes the 3D linear momentum vector.  

Return type: Vector3  
";

%feature("docstring") RobotModel::getAngularMomentum "

Computes the 3D angular momentum vector.  

Return type: Vector3  
";

%feature("docstring") RobotModel::getKineticEnergy "

Computes the kinetic energy at the current config / velocity.  
";

%feature("docstring") RobotModel::getTotalInertia "

Computes the 3x3 total inertia matrix of the robot.  

Return type: np.ndarray  
";

%feature("docstring") RobotModel::getMassMatrix "

Computes the nxn mass matrix B(q).  

Takes O(n^2) time  

Return type: np.ndarray  
";

%feature("docstring") RobotModel::getMassMatrixInv "

Computes the inverse of the nxn mass matrix B(q)^-1.  

Takes O(n^2) time, which is much faster than inverting the result of
getMassMatrix  

Return type: np.ndarray  
";

%feature("docstring") RobotModel::getMassMatrixDeriv "

Computes the derivative of the nxn mass matrix with respect to q_i.  

Takes O(n^3) time.  

Return type: np.ndarray  
";

%feature("docstring") RobotModel::getMassMatrixTimeDeriv "

Computes the derivative of the nxn mass matrix with respect to t, given the
robot's current velocity.  

Takes O(n^4) time.  

Return type: np.ndarray  
";

%feature("docstring") RobotModel::getCoriolisForceMatrix "

Computes the Coriolis force matrix C(q,dq) for current config and velocity.  

Takes O(n^2) time.  

Return type: np.ndarray  
";

%feature("docstring") RobotModel::getCoriolisForces "

Computes the Coriolis forces C(q,dq)*dq for current config and velocity.  

Takes O(n) time, which is faster than computing matrix and doing the product.  

(\"Forces\" is somewhat of a misnomer; the result is a joint torque vector)  

Return type: Vector  
";

%feature("docstring") RobotModel::getGravityForces "

Computes the generalized gravity vector G(q) for the given workspace gravity
vector g (usually (0,0,-9.8)).  

.. note::  

    \"Forces\" is somewhat of a misnomer; the result is a vector of joint
    torques.  

Returns:  

    list of floats: the n-element generalized gravity vector at the
    robot's current configuration.
 Return type: Vector  
";

%feature("docstring") RobotModel::torquesFromAccel "

Computes the inverse dynamics. Uses Recursive Newton Euler solver and takes O(n)
time.  

Specifically, solves for :math:`\\tau` in the (partial) dynamics equation:  

.. math::  

    `B(q) \\ddot{q} + C(q,\\dot{q}) = \\tau`  

.. note::  

    Does not include gravity term G(q).  getGravityForces(g) will
    need to be added to the result.  

Returns:  

    list of floats: the n-element torque vector that would produce
    the joint accelerations ddq in the absence of external forces.
 Return type: Vector  
";

%feature("docstring") RobotModel::accelFromTorques "

Computes the foward dynamics. Uses Recursive Newton Euler solver and takes O(n)
time.  

Specifically, solves for :math:`\\ddot{q}` in the (partial) dynamics equation:  

.. math::  

    `B(q) \\ddot{q} + C(q,\\dot{q}) = \\tau`  

.. note::  

    Does not include gravity term G(q).  getGravityForces(g) will
    need to be subtracted from the argument t.  

Returns:  

    list of floats: the n-element joint acceleration vector that would
    result from joint torques t in the absence of external forces.
 Return type: Vector  
";

%feature("docstring") RobotModel::interpolate "

Interpolates smoothly between two configurations, properly taking into account
nonstandard joints.  

Returns:  

    The n-element configuration that is u fraction of the way from a to b.
 Return type: Vector  
";

%feature("docstring") RobotModel::distance "

Computes a distance between two configurations, properly taking into account
nonstandard joints.  
";

%feature("docstring") RobotModel::interpolateDeriv "

Returns the configuration derivative at a as you interpolate toward b at unit
speed.  

Return type: Vector  
";

%feature("docstring") RobotModel::randomizeConfig "

Samples a random configuration and updates the robot's pose. Properly handles
non-normal joints and handles DOFs with infinite bounds using a centered
Laplacian distribution with the given scaling term.  

.. note::  

    Python random module seeding does not affect the result.
  
";

%feature("docstring") RobotModel::configToDrivers "

Converts a full configuration (length numLinks()) to a list of driver values
(length numDrivers()).  

Return type: Vector  
";

%feature("docstring") RobotModel::velocityToDrivers "

Converts a full velocity vector (length numLinks()) to a list of driver
velocities (length numDrivers()).  

Return type: Vector  
";

%feature("docstring") RobotModel::configFromDrivers "

Converts a list of driver values (length numDrivers()) to a full configuration
(length numLinks()).  

Return type: Vector  
";

%feature("docstring") RobotModel::velocityFromDrivers "

Converts a list of driver velocities (length numDrivers()) to a full velocity
vector (length numLinks()).  

Return type: Vector  
";

%feature("docstring") RobotModel::selfCollisionEnabled "

Queries whether self collisions between two links is enabled.  
";

%feature("docstring") RobotModel::enableSelfCollision "

Enables/disables self collisions between two links (depending on value)  
";

%feature("docstring") RobotModel::selfCollides "

Returns true if the robot is in self collision (faster than manual testing)  
";

%feature("docstring") RobotModel::drawGL "

Draws the robot geometry. If keepAppearance=true, the current appearance is
honored. Otherwise, only the raw geometry is drawn.  

PERFORMANCE WARNING: if keepAppearance is false, then this does not properly
reuse OpenGL display lists. A better approach to changing the robot's
appearances is to set the link Appearance's directly.  
";

%feature("docstring") RobotModel::reduce "

Sets self to a reduced version of robot, where all fixed DOFs are eliminated.
The return value is a map from the original robot DOF indices to the reduced
DOFs.  

Note that any geometries fixed to the world will disappear.  

Return type: List[int]  
";

%feature("docstring") RobotModel::mount "

Mounts a sub-robot onto a link, with its origin at a given local transform
(R,t). The sub-robot's links will be renamed to subRobot.getName() + ':' +
link.getName() unless subRobot.getName() is '', in which case the link names are
preserved.  
";

%feature("docstring") RobotModel::numSensors "

Returns the number of sensors.  
";

%feature("docstring") RobotModel::_sensor "
";

%feature("docstring") RobotModel::_sensor "
";

%feature("docstring") RobotModel::addSensor "

Adds a new sensor with a given name and type.  

Returns:  

    The new sensor.
  
";

// File: classRobotModelDriver.xml


%feature("docstring") RobotModelDriver "

A reference to a driver of a RobotModel.  

A driver corresponds to one of the robot's actuators and encodes how its forces
are transmitted to joints.  

A RobotModelDriver is not created by hand, but instead accessed using
:meth:`RobotModel.driver` (index or name)  

C++ includes: robotmodel.h
";

%feature("docstring") RobotModelDriver::RobotModelDriver "
";

%feature("docstring") RobotModelDriver::getName "
";

%feature("docstring") RobotModelDriver::setName "

Sets the name of the driver.  
";

%feature("docstring") RobotModelDriver::robot "

Returns a reference to the driver's robot.  
";

%feature("docstring") RobotModelDriver::getType "

Gets the type of the driver.  

Returns:  

    One of  \"normal\", \"affine\", \"rotation\", \"translation\", or \"custom\"
  
";

%feature("docstring") RobotModelDriver::getAffectedLink "

Returns the single affected link for \"normal\" links.  
";

%feature("docstring") RobotModelDriver::getAffectedLinks "

Returns the indices of the driver's affected links.  

Return type: List[int]  
";

%feature("docstring") RobotModelDriver::getAffineCoeffs "

For \"affine\" links, returns the scale and offset of the driver value mapped to
the world.  

Returns a pair (scale,offset), each of length len(getAffectedLinks()).  

Return type: Tuple[Vector,Vector]  
";

%feature("docstring") RobotModelDriver::setValue "

Sets the robot's config to correspond to the given driver value.  

.. note::  

    Does not update the links' forward kinematics.  Use
    robot.setConfig(robot.getConfig()) to update the forward kinematics.
  
";

%feature("docstring") RobotModelDriver::getValue "

Returns the current driver value from the robot's config.  
";

%feature("docstring") RobotModelDriver::setVelocity "

Sets the robot's velocity to correspond to the given driver velocity value.  
";

%feature("docstring") RobotModelDriver::getVelocity "

Returns the current driver velocity value from the robot's velocity.  
";

%feature("docstring") RobotModelDriver::getLimits "

Returns value limits [xmin,xmax].  
";

%feature("docstring") RobotModelDriver::getVelocityLimits "

Returns velocity limits [vmin,vmax].  

Return type: Tuple[float,float]  
";

%feature("docstring") RobotModelDriver::getAccelerationLimits "

Returns acceleration limits [amin,amax].  

Return type: Tuple[float,float]  
";

%feature("docstring") RobotModelDriver::getTorqueLimits "

Returns generalized torque limits [tmin,tmax].  

Return type: Tuple[float,float]  
";

// File: classRobotModelLink.xml


%feature("docstring") RobotModelLink "

A reference to a link of a RobotModel.  

The link stores many mostly-constant items (id, name, parent, geometry,
appearance, mass, joint axes). There are two exceptions:  

*   the link's current transform, which is affected by the RobotModel's current
    configuration, i.e., the last :meth:`RobotModel.setConfig` call.  
*   The various Jacobians of points on the link, accessed by
    :meth:`RobotModelLink.getJacobian` ,
    :meth:`RobotModelLink.getPositionJacobian` , and
    :meth:`RobotModelLink.getOrientationJacobian` , which are configuration
    dependent.  

A RobotModelLink is not created by hand, but instead accessed using
:meth:`RobotModel.link` (index or name).  

C++ includes: robotmodel.h
";

%feature("docstring") RobotModelLink::RobotModelLink "
";

%feature("docstring") RobotModelLink::getID "

Returns the ID of the robot link in its world.  

.. note::  

    The world ID is not the same as the link's index, retrieved by
    getIndex.
  
";

%feature("docstring") RobotModelLink::getName "

Returns the name of the robot link.  
";

%feature("docstring") RobotModelLink::setName "

Sets the name of the robot link.  
";

%feature("docstring") RobotModelLink::robot "

Returns a reference to the link's robot.  
";

%feature("docstring") RobotModelLink::getIndex "

Returns the index of the link (on its robot).  
";

%feature("docstring") RobotModelLink::getParentIndex "

Returns the index of the link's parent (on its robot). -1 indicates no parent.  
";

%feature("docstring") RobotModelLink::setParentIndex "

Sets the index of the link's parent (on its robot).  
";

%feature("docstring") RobotModelLink::getParentLink "

Returns a reference to the link's parent, or a NULL link if it has no parent.  
";

%feature("docstring") RobotModelLink::setParentLink "

Sets the link's parent (must be on the same robot).  
";

%feature("docstring") RobotModelLink::geometry "

Returns a reference to the link's geometry.  
";

%feature("docstring") RobotModelLink::appearance "

Returns a reference to the link's appearance.  
";

%feature("docstring") RobotModelLink::getMass "

Returns the inertial properties of the link. (Note that the Mass is given with
origin at the link frame, not about the COM.)  
";

%feature("docstring") RobotModelLink::setMass "

Sets the inertial proerties of the link. (Note that the Mass is given with
origin at the link frame, not about the COM.)  
";

%feature("docstring") RobotModelLink::getParentTransform "

Gets the transformation (R,t) to the parent link.  

Returns:  

    se3 object: a pair (R,t), with R a 9-list and t a 3-list of floats,
    giving the local transform from this link to its parent, in the
    reference (zero) configuration.
 Return type: RigidTransform  
";

%feature("docstring") RobotModelLink::setParentTransform "

Sets transformation (R,t) to the parent link.  
";

%feature("docstring") RobotModelLink::getAxis "

Gets the local rotational / translational axis.  

Return type: Vector3  
";

%feature("docstring") RobotModelLink::setAxis "

Sets the local rotational / translational axis.  
";

%feature("docstring") RobotModelLink::isPrismatic "

Returns whether the joint is prismatic.  
";

%feature("docstring") RobotModelLink::isRevolute "

Returns whether the joint is revolute.  
";

%feature("docstring") RobotModelLink::setPrismatic "

Changes a link from revolute to prismatic or vice versa.  
";

%feature("docstring") RobotModelLink::getWorldPosition "

Converts point from local to world coordinates.  

Returns:  

    list of 3 floats: the world coordinates of the local point plocal
 Return type: Vector3  
";

%feature("docstring") RobotModelLink::getWorldDirection "

Converts direction from local to world coordinates.  

Returns:  

    list of 3 floats: the world coordinates of the local direction
    vlocal
 Return type: Vector3  
";

%feature("docstring") RobotModelLink::getLocalPosition "

Converts point from world to local coordinates.  

Returns:  

    list of 3 floats: the local coordinates of the world point pworld
 Return type: Vector3  
";

%feature("docstring") RobotModelLink::getLocalDirection "

Converts direction from world to local coordinates.  

Returns:  

    list of 3 floats: the local coordinates of the world direction
    vworld
 Return type: Vector3  
";

%feature("docstring") RobotModelLink::getTransform "

Gets the link's current transformation (R,t) to the world frame.  

Returns:  

    se3 object: a pair (R,t), with R a 9-list and t a 3-list of floats.
 Return type: RigidTransform  
";

%feature("docstring") RobotModelLink::setTransform "

Sets the link's current transformation (R,t) to the world frame.  

.. note::  

    This does NOT perform inverse kinematics.  The transform is
    overwritten when the robot's setConfig() method is called.
  
";

%feature("docstring") RobotModelLink::getVelocity "

Computes the velocity of the link's origin given the robot's current joint
configuration and velocities. Equivalent to getPointVelocity([0,0,0]).  

Returns:  

    list of 3 floats: the current velocity of the link's origin, in
    world coordinates
 Return type: Vector3  
";

%feature("docstring") RobotModelLink::getAngularVelocity "

Computes the angular velocity of the link given the robot's current joint
configuration and velocities.  

Returns:  

    list of 3 floats: the current angular velocity of the link, in world
    coordinates
 Return type: Vector3  
";

%feature("docstring") RobotModelLink::getPointVelocity "

Computes the world velocity of a point attached to the link, given the robot's
current joint configuration and velocities.  

Returns:  

    list of 3 floats: the current velocity of the point, in
    world coordinates.
 Return type: Vector3  
";

%feature("docstring") RobotModelLink::getJacobian "

Computes the total jacobian of a point on this link w.r.t. the robot's
configuration q.  

The orientation jacobian is given in the first 3 rows, and is stacked on the
position jacobian, which is given in the last 3 rows.  

Returns:  

    ndarray: the 6xn total Jacobian matrix of the
    point given by local coordinates plocal.  
 Return type: np.ndarray  
";

%feature("docstring") RobotModelLink::getPositionJacobian "

Computes the position jacobian of a point on this link w.r.t. the robot's
configuration q.  

This matrix J gives the point's velocity (in world coordinates) via
np.dot(J,dq), where dq is the robot's joint velocities.  

Returns:  

    ndarray: the 3xn Jacobian matrix of the
    point given by local coordinates plocal.  
 Return type: np.ndarray  
";

%feature("docstring") RobotModelLink::getOrientationJacobian "

Computes the orientation jacobian of this link w.r.t. the robot's configuration
q.  

This matrix J gives the link's angular velocity (in world coordinates) via
np.dot(J,dq), where dq is the robot's joint velocities.  

Returns:  

    ndarray:: the 3xn orientation Jacobian matrix of the link.  
 Return type: np.ndarray  
";

%feature("docstring") RobotModelLink::getJacobianCols "

Returns the jacobian of a point on this link w.r.t. specified entries of the
robot's configuration q given by `links`.  

The orientation jacobian is given in the first 3 rows, and is stacked on the
position jacobian, which is given in the last 3 rows.  

Returns:  

    ndarray: the 6xlen(links) Jacobian matrix of the
    point given by local coordinates plocal.
 Return type: np.ndarray  
";

%feature("docstring") RobotModelLink::getPositionJacobianCols "

Returns the position jacobian of a point on this link w.r.t. specified entries
of the robot's configuration q given by `links`.  

This matrix J gives the point's velocity (in world coordinates) via
np.dot(J,dqlinks), where dqlinks are the joint velocities of the links in
`links`  

Returns:  

    ndarray: the 3xlen(links) position Jacobian matrix of the
    point given by local coordinates plocal.
 Return type: np.ndarray  
";

%feature("docstring") RobotModelLink::getOrientationJacobianCols "

Returns the orientation jacobian this link w.r.t. specified entries of the
robot's configuration q given by `links`.  

This matrix J gives the point's angular velocity (in world coordinates) via
np.dot(J,dqlinks), where dqlinks are the joint velocities of the links in
`links`  

Returns:  

    ndarray: the 3xlen(links) orientation Jacobian matrix of the
    link.
 Return type: np.ndarray  
";

%feature("docstring") RobotModelLink::getAcceleration "

Computes the acceleration of the link origin given the robot's current joint
configuration and velocities, and the joint accelerations ddq.  

ddq can be empty, which calculates the acceleration with acceleration 0, and is
a little faster than setting ddq to [0]*n  

Returns:  

    list of 3 floats: the acceleration of the link's origin, in
    world coordinates.
 Return type: Vector3  
";

%feature("docstring") RobotModelLink::getPointAcceleration "

Computes the acceleration of the point given the robot's current joint
configuration and velocities, and the joint accelerations ddq.  

Returns:  

    list of 3 floats: the acceleration of the point, in
    world coordinates.
 Return type: Vector3  
";

%feature("docstring") RobotModelLink::getAngularAcceleration "

Computes the angular acceleration of the link given the robot's current joint
configuration and velocities, and the joint accelerations ddq.  

Returns:  

    list of 3 floats: the angular acceleration of the link, in
    world coordinates.
 Return type: Vector3  
";

%feature("docstring") RobotModelLink::getPositionHessian "

Computes the Hessians of each component of the position p w.r.t the robot's
configuration q.  

Returns:  

    ndarray: a 3xnxn array with each of the elements in the first axis
    corresponding respectively, to the (x,y,z) components of the Hessian.
 Return type: np.ndarray  
";

%feature("docstring") RobotModelLink::getOrientationHessian "

Computes the pseudo-Hessians of each orientation component of the link w.r.t the
robot's configuration q. The pseudo-Hessian is the derivative of the angular
velocity of this link w.r.t. the joint velocities.  

Returns:  

    ndarray: a 3xnxn array with each of the elements in the first axis
    corresponding, respectively, to the (wx,wy,wz) components of the
    pseudo-Hessian.
 Return type: np.ndarray  
";

%feature("docstring") RobotModelLink::drawLocalGL "

Draws the link's geometry in its local frame. If keepAppearance=true, the
current Appearance is honored. Otherwise, just the geometry is drawn.  
";

%feature("docstring") RobotModelLink::drawWorldGL "

Draws the link's geometry in the world frame. If keepAppearance=true, the
current Appearance is honored. Otherwise, just the geometry is drawn.  
";

// File: classRobotPoser.xml


%feature("docstring") RobotPoser "
";

%feature("docstring") RobotPoser::RobotPoser "
";

%feature("docstring") RobotPoser::setActiveDofs "
";

%feature("docstring") RobotPoser::set "
";

%feature("docstring") RobotPoser::get "
";

%feature("docstring") RobotPoser::getConditioned "
";

%feature("docstring") RobotPoser::addIKConstraint "
";

%feature("docstring") RobotPoser::clearIKConstraints "
";

// File: classSensorModel.xml


%feature("docstring") SensorModel "

A sensor on a simulated robot.  

Kinematic models of sensors are retrieved using :meth:`RobotModel.sensor` and
can be created using :meth:`RobotModel.addSensor`.  

Physically-simulated sensors are retrieved from a controller using
:meth:`SimRobotController.sensor`, and can be created using
:meth:`SimRobotController.addSensor`.  

Some types of sensors can be kinematically-simulated such that they make
sensible measurements. To use kinematic simulation, you may arbitrarily set the
robot's position, call :meth:`kinematicReset`, and then call
:meth:`kinematicSimulate`. Subsequent calls assume the robot is being driven
along a coherent trajectory until the next :meth:`kinematicReset` is called.
This is necessary for sensors that estimate accelerations, e.g.,
ForceTorqueSensor, Accelerometer  

Physically-simulated sensors are automatically updated through the
:meth:`Simulator.simulate` call.  

Use :meth:`getMeasurements` to get the currently simulated measurement vector.
You may get garbage measurements before kinematicSimulate / Simulator.simulate
are called.  

LaserSensor, CameraSensor, TiltSensor, AccelerometerSensor, GyroSensor,
JointPositionSensor, JointVelocitySensor support kinematic simulation mode.
FilteredSensor and TimeDelayedSensor also work. The force-related sensors
(ContactSensor and ForceTorqueSensor) return 0's in kinematic simulation.  

To use get/setSetting, you will need to know the sensor attribute names and
types as described in `the Klampt sensor documentation
<https://github.com/krishauser/Klampt/blob/master/Cpp/docs/Manual-
Control.md#sensors>`_ (same as in the world or sensor XML file). Common settings
include:  

*   rate (float): how frequently the sensor is simulated  
*   enabled (bool): whether the simulator simulates this sensor  
*   link (int): the link on which this sensor lies (-1 for world)  
*   Tsensor (se3 transform, serialized with loader.write_se3(T)): the transform
    of the sensor on the robot / world.  

C++ includes: robotmodel.h
";

%feature("docstring") SensorModel::SensorModel "
";

%feature("docstring") SensorModel::getName "

Returns the name of the sensor.  
";

%feature("docstring") SensorModel::setName "

Sets the name of the sensor.  
";

%feature("docstring") SensorModel::getType "

Returns the type of the sensor.  
";

%feature("docstring") SensorModel::robot "

Returns the model of the robot to which this belongs.  
";

%feature("docstring") SensorModel::measurementNames "

Returns a list of names for the measurements (one per measurement).  
";

%feature("docstring") SensorModel::getMeasurements "

Returns an array of measurements from the previous simulation (or
kinematicSimulate) timestep.  

Return type: np.ndarray  
";

%feature("docstring") SensorModel::settings "

Returns all setting names.  
";

%feature("docstring") SensorModel::getSetting "

Returns the value of the named setting (you will need to manually parse this)  
";

%feature("docstring") SensorModel::setSetting "

Sets the value of the named setting (you will need to manually cast an
int/float/etc to a str)  
";

%feature("docstring") SensorModel::getEnabled "

Return whether the sensor is enabled during simulation (helper for getSetting)  
";

%feature("docstring") SensorModel::setEnabled "

Sets whether the sensor is enabled in physics simulation.  
";

%feature("docstring") SensorModel::_getLink "

Returns the link on which the sensor is mounted.  
";

%feature("docstring") SensorModel::_setLink "

Sets the link on which the sensor is mounted (helper for setSetting)  
";

%feature("docstring") SensorModel::_setLink "

Sets the link on which the sensor is mounted (helper for setSetting)  
";

%feature("docstring") SensorModel::getTransform "

Returns the local transform of the sensor on the robot's link. (helper for
getSetting)  

If the sensor doesn't have a transform (such as a joint position or torque
sensor) an exception will be raised.  

Return type: RigidTransform  
";

%feature("docstring") SensorModel::getTransformWorld "

Returns the world transform of the sensor given the robot's current
configuration. (helper for getSetting)  

If the sensor doesn't have a transform (such as a joint position or torque
sensor) an exception will be raised.  

Return type: RigidTransform  
";

%feature("docstring") SensorModel::setTransform "

Sets the local transform of the sensor on the robot's link. (helper for
setSetting)  

If the sensor doesn't have a transform (such as a joint position or torque
sensor) an exception will be raised.  
";

%feature("docstring") SensorModel::drawGL "

Draws a sensor indicator using OpenGL. If measurements are given, the indicator
is drawn as though these are the latest measurements, otherwise only an
indicator is drawn.  
";

%feature("docstring") SensorModel::drawGL "

Draws a sensor indicator using OpenGL. If measurements are given, the indicator
is drawn as though these are the latest measurements, otherwise only an
indicator is drawn.  
";

%feature("docstring") SensorModel::kinematicSimulate "

simulates / advances the kinematic simulation  
";

%feature("docstring") SensorModel::kinematicSimulate "
";

%feature("docstring") SensorModel::kinematicReset "

resets a kinematic simulation so that a new initial condition can be set  
";

// File: classSimBody.xml


%feature("docstring") SimBody "

A reference to a rigid body inside a Simulator (either a RigidObjectModel,
TerrainModel, or a link of a RobotModel).  

Can use this class to directly apply forces to or control positions / velocities
of objects in the simulation.  

.. note::  

    All changes are applied in the current simulation substep, not the duration
    provided to Simulation.simulate().  If you need fine-grained control,
    make sure to call Simulation.simulate() with time steps equal to the value
    provided to Simulation.setSimStep() (this is 0.001s by default).  Or, use
    a hook from :class:`~klampt.sim.simulation.SimpleSimulator`.
 .. note::  

    The transform of the body is centered at the *object's center of mass*
    rather than the object's reference frame given in the RobotModelLink or
    RigidObjectModel.  The object's reference frame is retrieved/set by
    getObjectTransform()/setObjectTransform().
  

C++ includes: robotsim.h
";

%feature("docstring") SimBody::getID "

Returns the object ID that this body associated with.  
";

%feature("docstring") SimBody::enable "

Sets the simulation of this body on/off.  
";

%feature("docstring") SimBody::isEnabled "

Returns true if this body is being simulated.  
";

%feature("docstring") SimBody::enableDynamics "

Turns dynamic simulation of the body on/off. If false, velocities will simply be
integrated forward, and forces will not affect velocity i.e., it will be pure
kinematic simulation.  
";

%feature("docstring") SimBody::isDynamicsEnabled "
";

%feature("docstring") SimBody::applyWrench "

Applies a force and torque about the COM over the duration of the next
Simulator.simulate(t) call.  
";

%feature("docstring") SimBody::applyForceAtPoint "

Applies a force at a given point (in world coordinates) over the duration of the
next Simulator.simulate(t) call.  
";

%feature("docstring") SimBody::applyForceAtCOMLocalPoint "

Applies a force at a given point (in local center-of-mass-centered coordinates)
over the duration of the next Simulator.simulate(t) call.  
";

%feature("docstring") SimBody::applyForceAtObjectLocalPoint "

Applies a force at a given point (in local object-centered coordinates) over the
duration of the next Simulator.simulate(t) call.  
";

%feature("docstring") SimBody::applyForceAtLocalPoint "

Deprecated: use applyForceAtCOMLocalPoint instead to match old behavior.  
";

%feature("docstring") SimBody::setTransform "

Sets the body's transformation at the current simulation time step (in center-
of-mass centered coordinates).  
";

%feature("docstring") SimBody::getTransform "

Gets the body's transformation at the current simulation time step (in center-
of-mass centered coordinates).  

Return type: RigidTransform  
";

%feature("docstring") SimBody::setObjectTransform "

Sets the body's transformation at the current simulation time step (in object-
native coordinates)  
";

%feature("docstring") SimBody::getObjectTransform "

Gets the body's transformation at the current simulation time step (in object-
native coordinates).  

Return type: RigidTransform  
";

%feature("docstring") SimBody::setVelocity "

Sets the angular velocity and translational velocity (of the COM) at the current
simulation time step.  
";

%feature("docstring") SimBody::getVelocity "

Returns the angular velocity and translational velocity (of the COM)  

Return type: Tuple[Vector3,Vector3]  
";

%feature("docstring") SimBody::setObjectVelocity "

Sets the angular velocity and translational velocity (of the object origin) at
the current simulation time step.  
";

%feature("docstring") SimBody::getObjectVelocity "

Returns the angular velocity and translational velocity (of the object origin)  

Return type: Tuple[Vector3,Vector3]  
";

%feature("docstring") SimBody::setCollisionPadding "

Sets the collision padding used for contact generation. At 0 padding the
simulation will be unstable for triangle mesh and point cloud geometries. A
larger value is useful to maintain simulation stability for thin or soft
objects. Default is 0.0025.  
";

%feature("docstring") SimBody::getCollisionPadding "
";

%feature("docstring") SimBody::setCollisionPreshrink "

If set, preshrinks the geometry so that the padded geometry better matches the
original mesh. If shrinkVisualization=true, the underlying mesh is also shrunk
(helps debug simulation artifacts due to preshrink)  
";

%feature("docstring") SimBody::getSurface "

Gets (a copy of) the surface properties.  
";

%feature("docstring") SimBody::setSurface "

Sets the surface properties.  
";

// File: classSimJoint.xml


%feature("docstring") SimJoint "

An interface to ODE's hinge and slider joints. You may use this to create custom
objects, e.g., drawers, doors, cabinets, etc. It can also be used to attach
objects together, e.g., an object to a robot's gripper.  

C++ includes: robotsim.h
";

%feature("docstring") SimJoint::SimJoint "
";

%feature("docstring") SimJoint::~SimJoint "
";

%feature("docstring") SimJoint::makeHinge "

Creates a hinge between `a` and `b`, or `a` and the world. The hinge is located
at point `pt`, with axis `axis`, both in world coordinates.  
";

%feature("docstring") SimJoint::makeHinge "
";

%feature("docstring") SimJoint::makeSlider "

Creates a slider between `a` and `b`, or `a` and the world. The slider restricts
movement to axis `axis`, given in world coordinates.  
";

%feature("docstring") SimJoint::makeSlider "
";

%feature("docstring") SimJoint::makeFixed "

Creates a fixed joint between `a` and `b`. (There's no method to fix a to the
world; just call a.enableDynamics(False))  
";

%feature("docstring") SimJoint::destroy "

Removes the joint from the simulation.  
";

%feature("docstring") SimJoint::setLimits "

Sets the joint limits, relative to the initial configuration of the bodies.
Units are in radians for hinges and meters for sliders.  
";

%feature("docstring") SimJoint::setFriction "

Sets the (dry) friction of the joint.  
";

%feature("docstring") SimJoint::setVelocity "

Locks velocity of the joint, up to force fmax. Can't be used with setFriction.  
";

%feature("docstring") SimJoint::addForce "

Adds a torque for the hinge joint and a force for a slider joint.  
";

// File: classSimRobotController.xml


%feature("docstring") SimRobotController "

A controller for a simulated robot.  

By default a SimRobotController has three possible modes:  

*   Motion queue + PID mode: the controller has an internal trajectory queue
    that may be added to and modified. This queue supports piecewise linear
    interpolation, cubic interpolation, and time-optimal move-to commands.  
*   PID mode: the user controls the motor's PID setpoints directly  
*   Torque control: the user controlls the motor torques directly.  

The \"standard\" way of using this is in move-to mode which accepts a milestone
(setMilestone) or list of milestones (repeated calls to addMilestone) and
interpolates dynamically from the current configuration/velocity. To handle
disturbances, a PID loop is run automatically at the controller's specified
rate.  

To get finer-grained control over the motion queue's timing, you may use the
setLinear/setCubic/addLinear/addCubic functions. In these functions it is up to
the user to respect velocity, acceleration, and torque limits.  

Whether in motion queue or PID mode, the constants of the PID loop are initially
set in the robot file. You can programmatically tune these via the setPIDGains
function.  

Arbitrary trajectories can be tracked by using setVelocity over short time
steps. Force controllers can be implemented using setTorque, again using short
time steps.  

If the setVelocity, setTorque, or setPID command are called, the motion queue
behavior will be completely overridden. To reset back to motion queue control,
setManualMode(False) must be called first.  

Individual joints cannot be addressed with mixed motion queue mode and
torque/PID mode. However, you can mix PID and torque mode between different
joints with a workaround::  


   # setup by zeroing out PID constants for torque controlled joints
   pid_joint_indices = [...]
   torque_joint_indices = [...] # complement of pid_joint_indices
   kp,ki,kp = controller.getPIDGains()
   for i in torque_joint_indices:  #turn off PID gains here
      kp[i] = ki[i] = kp[i] = 0

   # to send PID command (qcmd,dqcmd) and torque commands tcmd, use
   # a PID command with feedforward torques.  First we build a whole-robot
   # command:
   qcmd_whole = [0]*controller.model().numLinks()
   dqcmd_whole = [0]*controller.model().numLinks()
   tcmd_whole = [0]*controller.model().numLinks()
   for i,k in enumerate(pid_joint_indices):
       qcmd_whole[k],dqcmd_whole[i] = qcmd[i],dqcmd[i]
   for i,k in enumerate(torque_joint_indices):
       tcmd_whole[k] = tcmd[i]
   # Then we send it to the controller
   controller.setPIDCommand(qcmd_whole,dqcmd_whole,tcmd_whole)

  

C++ includes: robotsim.h
";

%feature("docstring") SimRobotController::SimRobotController "
";

%feature("docstring") SimRobotController::~SimRobotController "
";

%feature("docstring") SimRobotController::model "

Retrieves the robot model associated with this controller.  
";

%feature("docstring") SimRobotController::setRate "

Sets the current feedback control rate, in s.  
";

%feature("docstring") SimRobotController::getRate "

Returns The current feedback control rate, in s.  
";

%feature("docstring") SimRobotController::getCommandedConfig "

Returns The current commanded configuration (size model().numLinks())  
";

%feature("docstring") SimRobotController::getCommandedVelocity "

Returns The current commanded velocity (size model().numLinks())  
";

%feature("docstring") SimRobotController::getCommandedTorque "

Returns The current commanded (feedforward) torque (size model().numDrivers())  
";

%feature("docstring") SimRobotController::getSensedConfig "

Returns The current \"sensed\" configuration from the simulator (size
model().numLinks())  
";

%feature("docstring") SimRobotController::getSensedVelocity "

Returns The current \"sensed\" velocity from the simulator (size
model().numLinks())  
";

%feature("docstring") SimRobotController::getSensedTorque "

Returns The current \"sensed\" (feedback) torque from the simulator. (size
model().numDrivers())  

Note: a default robot doesn't have a torque sensor, so this will be 0  
";

%feature("docstring") SimRobotController::numSensors "

Returns the number of sensors.  
";

%feature("docstring") SimRobotController::_sensor "
";

%feature("docstring") SimRobotController::_sensor "
";

%feature("docstring") SimRobotController::addSensor "

Adds a new sensor with a given name and type.  

Returns:  

    The new sensor.
  
";

%feature("docstring") SimRobotController::commands "

Returns a custom command list.  
";

%feature("docstring") SimRobotController::sendCommand "

Sends a custom string command to the controller.  
";

%feature("docstring") SimRobotController::settings "

Returns all valid setting names.  
";

%feature("docstring") SimRobotController::getSetting "

Returns a setting of the controller.  
";

%feature("docstring") SimRobotController::setSetting "

Sets a setting of the controller.  
";

%feature("docstring") SimRobotController::setMilestone "

Uses a dynamic interpolant to get from the current state to the desired
milestone (with optional ending velocity). This interpolant is time-optimal with
respect to the velocity and acceleration bounds.  

Arguments have size model().numLinks().  
";

%feature("docstring") SimRobotController::setMilestone "

Uses a dynamic interpolant to get from the current state to the desired
milestone (with optional ending velocity). This interpolant is time-optimal with
respect to the velocity and acceleration bounds.  
";

%feature("docstring") SimRobotController::addMilestone "

Same as setMilestone, but appends an interpolant onto an internal motion queue
starting at the current queued end state.  

Arguments have size model().numLinks().  
";

%feature("docstring") SimRobotController::addMilestone "

Same as setMilestone, but appends an interpolant onto an internal motion queue
starting at the current queued end state.  
";

%feature("docstring") SimRobotController::addMilestoneLinear "

Same as addMilestone, but enforces that the motion should move along a straight-
line joint-space path.  
";

%feature("docstring") SimRobotController::setLinear "

Uses linear interpolation to get from the current configuration to the desired
configuration after time dt.  

q has size model().numLinks(). dt must be > 0.  
";

%feature("docstring") SimRobotController::setCubic "

Uses cubic (Hermite) interpolation to get from the current
configuration/velocity to the desired configuration/velocity after time dt.  

q and v have size model().numLinks(). dt must be > 0.  
";

%feature("docstring") SimRobotController::addLinear "

Same as setLinear but appends an interpolant onto the motion queue.  
";

%feature("docstring") SimRobotController::addCubic "

Same as setCubic but appends an interpolant onto the motion queue.  
";

%feature("docstring") SimRobotController::remainingTime "

Returns the remaining duration of the motion queue.  
";

%feature("docstring") SimRobotController::setVelocity "

Sets a rate controller from the current commanded config to move at rate dq for
time dt > 0. dq has size model().numLinks()  
";

%feature("docstring") SimRobotController::setTorque "

Sets a torque command controller. t can have size model().numDrivers() or
model().numLinks().  
";

%feature("docstring") SimRobotController::setPIDCommand "

Sets a PID command controller. Arguments can have size model().numDrivers() or
model().numLinks().  
";

%feature("docstring") SimRobotController::setPIDCommand "

Sets a PID command controller. If tfeedforward is provided, it is the
feedforward torque vector.  
";

%feature("docstring") SimRobotController::setManualMode "

Turns on/off manual mode, if either the setTorque or setPID command were
previously set.  
";

%feature("docstring") SimRobotController::getControlType "

Returns the control type for the active controller.  

Returns:  

    One of

    - unknown
    - off
    - torque
    - PID
    - locked_velocity
  
";

%feature("docstring") SimRobotController::setPIDGains "

Sets the PID gains. Arguments have size model().numDrivers().  
";

%feature("docstring") SimRobotController::getPIDGains "

Returns the PID gains for the PID controller.  
";

// File: classSimulator.xml


%feature("docstring") Simulator "

A dynamics simulator for a WorldModel.  

C++ includes: robotsim.h
";

%feature("docstring") Simulator::Simulator "

Constructs the simulator from a WorldModel. If the WorldModel was loaded from an
XML file, then the simulation setup is loaded from it.  
";

%feature("docstring") Simulator::~Simulator "
";

%feature("docstring") Simulator::reset "

Resets to the initial state (same as setState(initialState))  
";

%feature("docstring") Simulator::getStatus "

Returns an indicator code for the simulator status.  

Returns:  

    One of the STATUS_X flags.  (Technically, this returns the *worst* status
    over the last simulate() call)
  
";

%feature("docstring") Simulator::getStatusString "

Returns a string indicating the simulator's status. If s is provided and >= 0,
this function maps the indicator code s to a string.  
";

%feature("docstring") Simulator::checkObjectOverlap "

Checks if any objects are overlapping.  

Returns:  

    A pair of lists of integers, giving the pairs of object ids that
    are overlapping.
 Return type: Tuple[list,list]  
";

%feature("docstring") Simulator::getState "

Gets the current simulation state, including controller parameters, etc.  

Returns:  

    A Base64 string representing the binary data for the state
  
";

%feature("docstring") Simulator::setState "

Sets the current simulation state from a Base64 string returned by a prior
getState call.  
";

%feature("docstring") Simulator::simulate "

Advances the simulation by time t, and updates the world model from the
simulation state.  
";

%feature("docstring") Simulator::fakeSimulate "

Advances a faked simulation by time t, and updates the world model from the
faked simulation state.  
";

%feature("docstring") Simulator::getTime "

Returns the simulation time.  
";

%feature("docstring") Simulator::updateWorld "

Updates the world model from the current simulation state. This only needs to be
called if you change the world model and want to revert back to the simulation
state.  
";

%feature("docstring") Simulator::getActualConfig "

Returns the current actual configuration of the robot from the simulator.  

Return type: Vector  
";

%feature("docstring") Simulator::getActualVelocity "

Returns the current actual velocity of the robot from the simulator.  

Return type: Vector  
";

%feature("docstring") Simulator::getActualTorque "

Returns the current actual torques on the robot's drivers from the simulator.  

Return type: Vector  
";

%feature("docstring") Simulator::enableContactFeedback "

Call this to enable contact feedback between the two objects (arguments are
indexes returned by object.getID()). Contact feedback has a small overhead so
you may want to do this selectively. This must be called before using inContact,
getContacts, getContactForces, contactForce, contactTorque, hadContact,
hadSeparation, hadPenetration, and meanContactForce.  
";

%feature("docstring") Simulator::enableContactFeedbackAll "

Call this to enable contact feedback between all pairs of objects. Contact
feedback has a small overhead so you may want to do this selectively.  
";

%feature("docstring") Simulator::inContact "

Returns true if the objects (indexes returned by object.getID()) are in contact
on the current time step. You can set bid=-1 to tell if object `a` is in contact
with any object.  
";

%feature("docstring") Simulator::getContacts "

Returns the nx7 list of contacts (x,n,kFriction) at the last time step. Normals
point into object `a`. Each contact point (x,n,kFriction) is represented as a
7-element vector.  

Return type: np.ndarray  
";

%feature("docstring") Simulator::getContactForces "

Returns the list of contact forces on object a at the last time step. Result is
an nx3 array.  

Return type: np.ndarray  
";

%feature("docstring") Simulator::contactForce "

Returns the contact force on object a at the last time step. You can set bid to
-1 to get the overall contact force on object a.  

Return type: Vector3  
";

%feature("docstring") Simulator::contactTorque "

Returns the contact force on object `a` (about `a`'s origin) at the last time
step. You can set `bid` to -1 to get the overall contact force on object `a`.  

Return type: Vector3  
";

%feature("docstring") Simulator::hadContact "

Returns true if the objects had contact over the last simulate() call. You can
set `bid` to -1 to determine if object `a` had contact with any other object.  
";

%feature("docstring") Simulator::hadSeparation "

Returns true if the objects had ever separated during the last simulate() call.
You can set `bid` to -1 to determine if object `a` had no contact with any other
object.  
";

%feature("docstring") Simulator::hadPenetration "

Returns true if the objects interpenetrated during the last simulate() call. If
so, the simulation may lead to very inaccurate results or artifacts.  

You can set `bid` to -1 to determine if object `a` penetrated any object, or you
can set `aid=bid=-1` to determine whether any object is penetrating any other
(indicating that the simulation will not be functioning properly in general).  
";

%feature("docstring") Simulator::meanContactForce "

Returns the average contact force on object a over the last simulate() call.  

Return type: Vector3  
";

%feature("docstring") Simulator::controller "

Returns a controller for the indicated robot, either by index or by RobotModel.  
";

%feature("docstring") Simulator::controller "

Returns a controller for the indicated robot, either by index or by RobotModel.  
";

%feature("docstring") Simulator::body "

Returns the SimBody corresponding to the given link.  
";

%feature("docstring") Simulator::body "

Returns the SimBody corresponding to the given object.  
";

%feature("docstring") Simulator::body "

Return the SimBody corresponding to the given link, rigid object, or terrain.  
";

%feature("docstring") Simulator::getJointForces "

Returns the joint force and torque local to the link, as would be read by a
force-torque sensor mounted at the given link's origin.  

Returns:  

    6 entries of the wrench (fx,fy,fz,mx,my,mz)
 Return type: Vector  
";

%feature("docstring") Simulator::setGravity "

Sets the overall gravity vector.  
";

%feature("docstring") Simulator::setSimStep "

Sets the internal simulation substep. Values < 0.01 are recommended.  
";

%feature("docstring") Simulator::settings "

Returns all setting names.  
";

%feature("docstring") Simulator::getSetting "

Retrieves some simulation setting.  

Valid names are:  

*   gravity: the gravity vector (default \"0 0 -9.8\")  
*   simStep: the internal simulation step (default \"0.001\")  
*   autoDisable: whether to disable bodies that don't move much between time
    steps (default \"0\", set to \"1\" for many static objects)  
*   boundaryLayerCollisions: whether to use the Klampt inflated boundaries for
    contact detection'(default \"1\", recommended)  
*   rigidObjectCollisions: whether rigid objects should collide (default \"1\")  
*   robotSelfCollisions: whether robots should self collide (default \"0\")  
*   robotRobotCollisions: whether robots should collide with other robots
    (default \"1\")  
*   adaptiveTimeStepping: whether adaptive time stepping should be used to
    improve stability. Slower but more stable. (default \"1\")  
*   minimumAdaptiveTimeStep: the minimum size of an adaptive time step before
    giving up (default \"1e-6\")  
*   maxContacts: max # of clustered contacts between pairs of objects (default
    \"20\")  
*   clusterNormalScale: a parameter for clustering contacts (default \"0.1\")  
*   errorReductionParameter: see ODE docs on ERP (default \"0.95\")  
*   dampedLeastSquaresParameter: see ODE docs on CFM (default \"1e-6\")  
*   instabilityConstantEnergyThreshold: parameter c0 in instability correction
    (default \"1\")  
*   instabilityLinearEnergyThreshold: parameter c1 in instability correction
    (default \"1.5\")  
*   instabilityMaxEnergyThreshold: parameter cmax in instability correction
    (default \"100000\")  
*   instabilityPostCorrectionEnergy: kinetic energy scaling parameter if
    instability is detected (default \"0.8\")  

Instability correction kicks in whenever the kinetic energy K(t) of an object
exceeds min(c0*m + c1*K(t-dt),cmax). m is the object's mass.  

See `Klampt/Simulation/ODESimulator.h
<http://motion.pratt.duke.edu/klampt/klampt_docs/ODESimulator_8h_source.html>`_
for detailed descriptions of these parameters.  

Returns:  

    A string encoding the data. This will need to be cast to int or
    float manually.
  
";

%feature("docstring") Simulator::setSetting "

Sets some simulation setting. Raises an exception if the name is unknown or the
value is of improper format.  
";

// File: classSpherePoser.xml


%feature("docstring") SpherePoser "
";

%feature("docstring") SpherePoser::SpherePoser "
";

%feature("docstring") SpherePoser::set "
";

%feature("docstring") SpherePoser::get "
";

// File: classTerrainModel.xml


%feature("docstring") TerrainModel "

Static environment geometry.  

C++ includes: robotmodel.h
";

%feature("docstring") TerrainModel::TerrainModel "
";

%feature("docstring") TerrainModel::loadFile "

Loads the terrain from the file fn.  
";

%feature("docstring") TerrainModel::saveFile "

Saves the terrain to the file fn. If geometryName is given, the geometry is
saved to that file.  
";

%feature("docstring") TerrainModel::getID "

Returns the ID of the terrain in its world.  

.. note::  

    The world ID is not the same as the terrain index.
  
";

%feature("docstring") TerrainModel::getName "
";

%feature("docstring") TerrainModel::setName "
";

%feature("docstring") TerrainModel::geometry "

Returns a reference to the geometry associated with this object.  
";

%feature("docstring") TerrainModel::appearance "

Returns a reference to the appearance associated with this object.  
";

%feature("docstring") TerrainModel::setFriction "

Changes the friction coefficient for this terrain.  
";

%feature("docstring") TerrainModel::drawGL "

Draws the object's geometry. If keepAppearance=true, the current appearance is
honored. Otherwise, only the raw geometry is drawn.  

PERFORMANCE WARNING: if keepAppearance is false, then this does not properly
reuse OpenGL display lists. A better approach is to change the object's
Appearance directly.  
";

// File: classTransformPoser.xml


%feature("docstring") TransformPoser "
";

%feature("docstring") TransformPoser::TransformPoser "
";

%feature("docstring") TransformPoser::set "
";

%feature("docstring") TransformPoser::get "
";

%feature("docstring") TransformPoser::enableTranslation "
";

%feature("docstring") TransformPoser::enableRotation "
";

%feature("docstring") TransformPoser::enableTranslationAxes "
";

%feature("docstring") TransformPoser::enableRotationAxes "
";

// File: structTriangleMesh.xml


%feature("docstring") TriangleMesh "

A 3D indexed triangle mesh class.  

Attributes:  

    vertices (numpy array):  an n x 3 array of vertices.
    indices (numpy int32 array): an m x 3 list of triangle vertices, given
        as indices into the vertices list, i.e., [[a1,b1,c2], [a2,b2,c2], ...]  

Examples::  

    m = TriangleMesh()
    m.addVertex((0,0,0))
    m.addVertex((1,0,0))
    m.addVertex((0,1,0))
    m.addTriangle(0,1,2)
    print(len(m.vertices))  #prints 3
    print(len(m.indices))   #prints 1
  

C++ includes: geometry.h
";

%feature("docstring") TriangleMesh::TriangleMesh "
";

%feature("docstring") TriangleMesh::TriangleMesh "
";

%feature("docstring") TriangleMesh::~TriangleMesh "
";

%feature("docstring") TriangleMesh::copy "

Creates a standalone object that is a copy of this.  
";

%feature("docstring") TriangleMesh::set "

Copies the data of the argument into this.  
";

%feature("docstring") TriangleMesh::getVertices "

Retrieves an array view of the vertices.  

Returns:  

    ndarray: an nx3 Numpy array. Setting elements of this array will
    change the vertices.
 Return type: np.ndarray  
";

%feature("docstring") TriangleMesh::setVertices "

Sets all vertices to the given nx3 Numpy array.  
";

%feature("docstring") TriangleMesh::addVertex "

Adds a new vertex.  
";

%feature("docstring") TriangleMesh::getIndices "

Retrieves an array view of the triangle indices.  

Returns:  

    ndarray: an mx3 Numpy array of int32 type. Setting elements of this
    array will change the triangle indices.
 Return type: np.ndarray  
";

%feature("docstring") TriangleMesh::setIndices "

Sets all indices to the given mx3 Numpy array.  
";

%feature("docstring") TriangleMesh::addTriangleIndices "

Adds a new triangle.  
";

%feature("docstring") TriangleMesh::translate "

Translates all the vertices by v=v+t.  
";

%feature("docstring") TriangleMesh::transform "

Transforms all the vertices by the rigid transform v=R*v+t.  
";

// File: classViewport.xml


%feature("docstring") Viewport "

A class that represents an idealized pinhole camera.  

Duplicates the functioning of KrisLibrary/Camera/Viewport.  

C++ includes: viewport.h
";

%feature("docstring") Viewport::Viewport "
";

%feature("docstring") Viewport::fromJson "
";

%feature("docstring") Viewport::toJson "
";

%feature("docstring") Viewport::fromText "
";

%feature("docstring") Viewport::toText "
";

%feature("docstring") Viewport::resize "

Resizes the viewport, keeping the same field of view and relative position of
the focal point.  
";

%feature("docstring") Viewport::setFOV "

Sets the horizontal and optionally the vertical FOV. If yfov < 0, square pixels
will be assumed.  
";

%feature("docstring") Viewport::getFOV "

Returns the horizontal FOV.  
";

%feature("docstring") Viewport::getVFOV "

Returns the vertical FOV.  
";

%feature("docstring") Viewport::setPose "

Sets the pose of the camera.  
";

%feature("docstring") Viewport::getPose "

Gets the pose of the camera.  

Return type: RigidTransform  
";

%feature("docstring") Viewport::viewRectangle "

Gets the viewing rectangle (xmin,ymin,xmax,ymax) at a given depth.  

Return type: Tuple[float,float,float,float]  
";

%feature("docstring") Viewport::project "

Projects into image coordinates.  

Return type: Vector3  
";

%feature("docstring") Viewport::clickSource "

Provides the source of a ray for an image coordinate (x,y)  

Return type: Vector3  
";

%feature("docstring") Viewport::clickDirection "

Provides the direction of a ray for an image coordinate (x,y)  

Return type: Vector  
";

// File: classWidget.xml


%feature("docstring") Widget "
";

%feature("docstring") Widget::Widget "
";

%feature("docstring") Widget::~Widget "
";

%feature("docstring") Widget::hover "
";

%feature("docstring") Widget::beginDrag "
";

%feature("docstring") Widget::drag "
";

%feature("docstring") Widget::endDrag "
";

%feature("docstring") Widget::keypress "
";

%feature("docstring") Widget::drawGL "
";

%feature("docstring") Widget::idle "
";

%feature("docstring") Widget::wantsRedraw "
";

%feature("docstring") Widget::hasHighlight "
";

%feature("docstring") Widget::hasFocus "
";

// File: classWidgetSet.xml


%feature("docstring") WidgetSet "
";

%feature("docstring") WidgetSet::WidgetSet "
";

%feature("docstring") WidgetSet::add "
";

%feature("docstring") WidgetSet::remove "
";

%feature("docstring") WidgetSet::enable "
";

// File: classWorldModel.xml


%feature("docstring") WorldModel "

The main world class, containing robots, rigid objects, and static environment
geometry.  

Attributes:  

    - robots: a list of RobotModel instances
    - rigidObjects: a list of RigidObjectModel instances
    - terrains: a list of TerrainModel instances  

.. note:  

    Although a WorldModel instance is typically called a \"world\" it is
    just a model and does not have to reflect the state of a physical world.
    The state of robots and objects in the world can be changed at will --
    in fact planners and simulators will query and modify the state of a
    WorldModel during their operation.

    To keep around some \"authoritative\" world, you can keep around a copy
    (use ``WorldModel.copy()``) or ``config.getConfig(world)`` using the
    :mod:`klampt.model.config` module.  

Every robot/robot link/terrain/rigid object is given a unique ID in the world.
This is potentially a source of confusion because some functions take IDs and
some take indices. Only the WorldModel and Simulator classes use IDs when the
argument has 'id' as a suffix, e.g., geometry(), appearance(),
Simulator.inContact(). All other functions use indices, e.g. robot(0),
terrain(0), etc.  

To get an object's ID, you can see the value returned by loadElement and/or
object.getID().  

To save/restore the state of the model, you must manually maintain copies of the
states of whichever objects you wish to save/restore.  

C++ includes: robotmodel.h
";

%feature("docstring") WorldModel::WorldModel "
";

%feature("docstring") WorldModel::WorldModel "
";

%feature("docstring") WorldModel::WorldModel "
";

%feature("docstring") WorldModel::WorldModel "

Creates a WorldModel.  

*   Given no arguments, creates a new world.  
*   Given another WorldModel instance, creates a reference to an existing world.
    (To create a copy, use the copy() method.)  
*   Given a string, loads from a file. A PyException is raised on failure.  
*   Given a pointer to a C++ RobotWorld structure, a reference to that structure
    is returned. (This is advanced usage, seen only when interfacing C++ and
    Python code)  
";

%feature("docstring") WorldModel::~WorldModel "
";

%feature("docstring") WorldModel::copy "

Creates a copy of the world model. Note that geometries and appearances are
shared, so this is very quick.  
";

%feature("docstring") WorldModel::readFile "

Reads from a world XML file.  

Returns:  

    True if successful, False if failed.
  
";

%feature("docstring") WorldModel::loadFile "

Alias of readFile.  
";

%feature("docstring") WorldModel::saveFile "

Saves to a world XML file. Elements in the world will be saved to a folder.  

If elementDir is provided, then robots, terrains, etc. will be saved there.
Otherwise they will be saved to a folder with the same base name as fn (without
the trailing .xml)  
";

%feature("docstring") WorldModel::numRobots "

Returns the number of robots.  
";

%feature("docstring") WorldModel::numRobotLinks "

Returns the number of links on the given robot.  
";

%feature("docstring") WorldModel::numRigidObjects "

Returns the number of rigid objects.  
";

%feature("docstring") WorldModel::numTerrains "

Returns the number of terrains.  
";

%feature("docstring") WorldModel::numIDs "

Returns the total number of world ids.  
";

%feature("docstring") WorldModel::robot "

Returns a RobotModel in the world by index or name.  
";

%feature("docstring") WorldModel::robot "

Returns a RobotModel in the world by index or name.  
";

%feature("docstring") WorldModel::robotLink "

Returns a RobotModelLink of some RobotModel in the world by index or name.  
";

%feature("docstring") WorldModel::robotLink "

Returns a RobotModelLink of some RobotModel in the world by index or name.  
";

%feature("docstring") WorldModel::rigidObject "

Returns a RigidObjectModel in the world by index or name.  
";

%feature("docstring") WorldModel::rigidObject "

Returns a RigidObjectModel in the world by index or name.  
";

%feature("docstring") WorldModel::terrain "

Returns a TerrainModel in the world by index or name.  
";

%feature("docstring") WorldModel::terrain "

Returns a TerrainModel in the world by index or name.  
";

%feature("docstring") WorldModel::makeRobot "

Creates a new empty robot. (Not terribly useful now since you can't resize the
number of links yet)  
";

%feature("docstring") WorldModel::makeRigidObject "

Creates a new empty rigid object.  
";

%feature("docstring") WorldModel::makeTerrain "

Creates a new empty terrain.  
";

%feature("docstring") WorldModel::loadRobot "

Loads a robot from a .rob or .urdf file. An empty robot is returned if loading
fails.  
";

%feature("docstring") WorldModel::loadRigidObject "

Loads a rigid object from a .obj or a mesh file. An empty rigid object is
returned if loading fails.  
";

%feature("docstring") WorldModel::loadTerrain "

Loads a rigid object from a mesh file. An empty terrain is returned if loading
fails.  
";

%feature("docstring") WorldModel::loadElement "

Loads some element from a file, automatically detecting its type. Meshes are
interpreted as terrains.  

Returns:  

    The element's ID, or -1 if loading failed.
  
";

%feature("docstring") WorldModel::add "

Adds a copy of the given robot, rigid object, or terrain to this world, either
from this WorldModel or another.  
";

%feature("docstring") WorldModel::add "

Adds a copy of the given robot, rigid object, or terrain to this world, either
from this WorldModel or another.  
";

%feature("docstring") WorldModel::add "

Adds a copy of the given robot, rigid object, or terrain to this world, either
from this WorldModel or another.  
";

%feature("docstring") WorldModel::remove "

Removes a robot, rigid object, or terrain from the world. It must be in this
world or an exception is raised.  

IMPORTANT:  

    All other RobotModel, RigidObjectModel, or TerrainModel references
    will be invalidated.
  
";

%feature("docstring") WorldModel::remove "

Removes a robot, rigid object, or terrain from the world. It must be in this
world or an exception is raised.  

IMPORTANT:  

    All other RobotModel, RigidObjectModel, or TerrainModel references
    will be invalidated.
  
";

%feature("docstring") WorldModel::remove "

Removes a robot, rigid object, or terrain from the world. It must be in this
world or an exception is raised.  

IMPORTANT:  

    All other RobotModel, RigidObjectModel, or TerrainModel references
    will be invalidated.
  
";

%feature("docstring") WorldModel::getName "

Retrieves the name for a given element ID.  
";

%feature("docstring") WorldModel::geometry "

Retrieves a geometry for a given element ID.  
";

%feature("docstring") WorldModel::appearance "

Retrieves an appearance for a given element ID.  
";

%feature("docstring") WorldModel::drawGL "

Draws the entire world using OpenGL.  
";

%feature("docstring") WorldModel::enableGeometryLoading "

If geometry loading is set to false, then only the kinematics are loaded from
disk, and no geometry / visualization / collision detection structures will be
loaded. Useful for quick scripts that just use kinematics / dynamics of a robot.  
";

%feature("docstring") WorldModel::enableInitCollisions "

If collision detection is set to true, then collision acceleration data
structures will be automatically initialized, with debugging information. Useful
for scripts that do planning and for which collision initialization may take a
long time.  

Note that even when this flag is off, the collision acceleration data structures
will indeed be initialized the first time that geometry collision, distance, or
ray-casting routines are called.  
";

// File: namespaceKlampt.xml

// File: namespacestd.xml

// File: appearance_8h.xml

// File: geometry_8h.xml

// File: motionplanning_8h.xml

%feature("docstring") set_random_seed "

Sets the random seed used by the RobotModel.randomizeConfig() method and
sampling-based motion planners.  
";

%feature("docstring") set_plan_json_string "

Loads planner values from a JSON string.  
";

%feature("docstring") get_plan_json_string "

Saves planner values to a JSON string.  
";

%feature("docstring") set_plan_type "

Sets the planner type.  

Valid values are  

*   prm: the Probabilistic Roadmap algorithm  
*   rrt: the Rapidly Exploring Random Trees algorithm  
*   sbl: the Single-Query Bidirectional Lazy planner  
*   sblprt: the probabilistic roadmap of trees (PRT) algorithm with SBL as the
    inter-root planner.  
*   rrt*: the RRT* algorithm for optimal motion planning  
*   prm*: the PRM* algorithm for optimal motion planning  
*   lazyprm*: the Lazy-PRM* algorithm for optimal motion planning  
*   lazyrrg*: the Lazy-RRG* algorithm for optimal motion planning  
*   fmm: the fast marching method algorithm for resolution-complete optimal
    motion planning  
*   fmm*: an anytime fast marching method algorithm for optimal motion planning  
";

%feature("docstring") set_plan_setting "
";

%feature("docstring") set_plan_setting "

Sets a numeric or string-valued setting for the planner.  

Valid numeric values are:  

*   \"knn\": k value for the k-nearest neighbor connection strategy (only for
    PRM)  
*   \"connectionThreshold\": a milestone connection threshold  
*   \"perturbationRadius\": (for RRT and SBL)  
*   \"bidirectional\": 1 if bidirectional planning is requested (for RRT)  
*   \"grid\": 1 if a point selection grid should be used (for SBL)  
*   \"gridResolution\": resolution for the grid, if the grid should be used (for
    SBL with grid, FMM, FMM*)  
*   \"suboptimalityFactor\": allowable suboptimality (for RRT*, lazy PRM*, lazy
    RRG*)  
*   \"randomizeFrequency\": a grid randomization frequency (for SBL)  
*   \"shortcut\": nonzero if you wish to perform shortcutting after a first plan
    is found.  
*   \"restart\": nonzero if you wish to restart the planner to get better paths
    with the remaining time.  

Valid string values are:  

*   \"pointLocation\": a string designating a point location data structure.
    \"kdtree\" is supported, optionally followed by a weight vector (for PRM,
    RRT*, PRM*, LazyPRM*, LazyRRG*)  
*   \"restartTermCond\": used if the \"restart\" setting is true. This is a JSON
    string defining the termination condition.  

    The default value is \"{foundSolution:1;maxIters:1000}\", which indicates
    that the planner will restart if it has found a solution, or 1000 iterations
    have passed.  

    To restart after a certain amount of time has elasped, use
    \"{timeLimit:X}\". If you are using an optimizing planner, e.g.,
    shortcutting, you should set foundSolution:0.  
";

%feature("docstring") destroy "

Performs cleanup of all created spaces and planners.  
";

// File: robotik_8h.xml

// File: robotio_8h.xml

%feature("docstring") subscribe_to_stream "

Subscribes a Geometry3D to a stream.  

Only ROS point clouds (PointCloud2) are supported for now. Note that you can
also call `Geometry3D.loadFile(\"ros://[ROS_TOPIC]\")` or
`Geometry3D.loadFile(\"ros:PointCloud2//[ROS_TOPIC]\")` to accomplish the same
thing.  

TODO: It has not yet been determined whether this interferes with Rospy, i.e.,
klampt.io.ros.  

Args:  

    g (Geometry3D): the geometry that will be updated
    protocol (str): only \"ros\" accepted for now.
    name (str): the name of the stream, i.e., ROS topic.
    type (str, optional): If provided, specifies the format of the data
        to be subscribed to. If not, tries to determine the type
        automatically.  

Returns:  

    bool: True if successful.
  
";

%feature("docstring") detach_from_stream "

Unsubscribes from a stream previously subscribed to via
:func:`SubscribeToStream`  
";

%feature("docstring") process_streams "

Does some processing on stream subscriptions.  

Args:  

    protocol (str): either name the protocol to be updated, or \"all\" for
        updating all subscribed streams  

Returns:  

    bool: True if any stream was updated.
  
";

%feature("docstring") wait_for_stream "

Waits up to timeout seconds for an update on the given stream.  

Returns:  

    bool: True if the stream was updated.
  
";

%feature("docstring") threejs_get_scene "

Exports the WorldModel to a JSON string ready for use in Three.js.  
";

%feature("docstring") threejs_get_transforms "

Exports the WorldModel to a JSON string ready for use in Three.js.  
";

// File: robotmodel_8h.xml

// File: robotsim_8h.xml

%feature("docstring") Klampt::set_random_seed "

Sets the random seed used by the configuration sampler and motion planners.  
";

%feature("docstring") Klampt::set_log_level "

Sets the global log level for C++ code. level can be \"DEBUG\", \"INFO\",
\"WARN\", \"ERROR\", or \"FATAL\".  
";

%feature("docstring") Klampt::destroy "

Cleans up all internal data structures. Useful for multithreaded programs to
make sure ODE errors aren't thrown on exit. This is called for you on exit when
importing the Python klampt module.  
";

// File: rootfind_8h.xml

%feature("docstring") setFTolerance "

Sets the termination threshold for the change in f.  
";

%feature("docstring") setXTolerance "

Sets the termination threshold for the change in x.  
";

%feature("docstring") setVectorField "

Sets the vector field object.  

Returns:  

    status (int): 0 if pVFObj = NULL, 1 otherwise.
 See vectorfield.py for an abstract base class that can be overridden to produce
one of these objects.  
";

%feature("docstring") setFunction "

Sets the function object.  

Returns:  

    status (int): 0 if pVFObj = NULL, 1 otherwise.
 See vectorfield.py for an abstract base class that can be overridden to produce
one of these objects.  

Equivalent to setVectorField; just a more intuitive name.  
";

%feature("docstring") findRoots "

Performs unconstrained root finding for up to iter iterations  

Returns:  

    status,x,n (tuple of int, list of floats, int): where status indicates
        the return code, as follows:

            - 0: convergence reached in x
            - 1: convergence reached in f
            - 2: divergence
            - 3: degeneration of gradient (local extremum or saddle point)
            - 4: maximum iterations reached
            - 5: numerical error occurred

        and x is the final point and n is the number of iterations used
  
";

%feature("docstring") findRootsBounded "

Same as findRoots, but with given bounds (xmin,xmax)  
";

%feature("docstring") destroy "

destroys internal data structures  
";

// File: stability_8h.xml

%feature("docstring") set_friction_cone_approximation_edges "

Globally sets the number of edges used in the friction cone approximation. The
default value is 4.  
";

%feature("docstring") force_closure "

Returns true if the array of contact points has force closure. A contact point
is given by a list of n=7 floats, [x,y,z,nx,ny,nz,k] where (x,y,z) is the
position, (nx,ny,nz) is the normal, and k is the coefficient of friction (>= 0)  
";

%feature("docstring") force_closure "

Returns true if the list of contact points has force closure.  

In the 1-argument version, each contact point is specified by a list of 7
floats, [x,y,z,nx,ny,nz,k] where (x,y,z) is the position, (nx,ny,nz) is the
normal, and k is the coefficient of friction.  

The 2-argument version is a \"fancy\" version that allows more control over the
constraint planes.  

Args:  

    contacts (list of 7-float lists or tuples): the list of contacts, each
        specified as a 7-list or tuple [x,y,z,nx,ny,nz,k], with:

        - (x,y,z): the contact position
        - (nx,ny,nz): the contact normal
        - k: the coefficient of friction (>= 0)

    contactPositions (list of 3-float lists or tuples): the list of contact
        point positions.
    frictionCones (list of lists): Each item of this list specifies linear
        inequalities that must be met of the force at the corresponding
        contact point.  The item must have length k*4 where k is an integer,
        and each inequality gives the entries (ax,ay,az,b) of a constraint
        ax*fx+ay*fy+az*fz <= b that limits the contact force (fx,fy,fz) at
        the i'th contact.  Each of the k 4-tuples is laid out sequentially
        per-contact.
  
";

%feature("docstring") force_closure_2d "

Returns true if the list of 2D contact points has force closure. A contact point
is given by a list of n=4 floats, [x,y,theta,k] where (x,y) is the position,
theta is the normal angle, and k is the coefficient of friction (>= 0)  
";

%feature("docstring") force_closure_2d "

Returns true if the list of 2D contact points has force closure.  

In the 1-argument version, each contact point is given by a list of 4 floats,
[x,y,theta,k] where (x,y) is the position, theta is the normal angle, and k is
the coefficient of friction  

The 2-argument version is a \"fancy\" version that allows more control over the
constraint planes.  

Args:  

    contacts (list of 4-float lists or tuples): the list of contacts, each
        specified as a 4-list or tuple [x,y,theta,k], with:

        - (x,y): the contact position
        - theta: is the normal angle (in radians, CCW to the x axis)
        - k: the coefficient of friction (>= 0)

    contactPositions (list of 2-float lists or tuples): the list of contact
        point positions.
    frictionCones (list of lists): The i'th element in this list has length
        k*3 (for some integer k), and gives the contact force constraints
        (ax,ay,b) where ax*fx+ay*fy <= b limits the contact force (fx,fy)
        at the i'th contact. Each of the k 3-tuples is laid out sequentially
        per-contact.
  
";

%feature("docstring") com_equilibrium "

Tests whether the given COM com is stable for the given contacts and the given
external force fext. A contact point is given by a list of 7 floats,
[x,y,z,nx,ny,nz,k] as usual.  

The return value is either None, or a list of 3-tuples giving the support forces
at the contacts.  

com can also be set to None in which case this tests if ANY COM has at the
contacts. The return value is True or False.  
";

%feature("docstring") com_equilibrium "

Tests whether the given COM com is stable for the given contacts and the given
external force fext.  

The 2-argument version is a \"fancy\" version that allows more control over the
constraint planes.  

Args:  

    contacts (list of 7-float lists or tuples): the list of contacts, each
        specified as a 7-list or tuple [x,y,z,nx,ny,nz,k], with:

        - (x,y,z): the contact position
        - (nx,ny,nz): the contact normal
        - k: the coefficient of friction (>= 0)

    contactPositions (list of 3-float lists or tuples): the list of contact
        point positions.
    frictionCones (list of lists): Each item of this list specifies linear
        inequalities that must be met of the force at the corresponding
        contact point.  The item must have length k*4 where k is an integer,
        and each inequality gives the entries (ax,ay,az,b) of a constraint
        ax*fx+ay*fy+az*fz <= b that limits the contact force (fx,fy,fz) at
        the i'th contact.  Each of the k 4-tuples is laid out sequentially
        per-contact.
    fext (3-tuple or list): the external force vector.
    com (3-tuple or list, or None): the center of mass coordinates.  If
        None, assumes that you want to test whether ANY COM may be in
        equilibrium for the given contacts.  

Returns:  

    bool, None, or list: if com is given, and there are feasible
    equilibrium forces, this returns a list of 3 tuples giving
    equilibrium forces at each of the contacts. None is returned if
    no such forces exist.  

    If com = None, the result is True or False.
  
";

%feature("docstring") com_equilibrium_2d "

Tests whether the given COM com is stable for the given contacts and the given
external force fext. A contact point is given by a list of 4 floats,
[x,y,theta,k] as usual.  

The return value is either None, or a list of 2-tuples giving the support forces
at the contacts.  

com can also be set to None in which case this tests if ANY COM has at the
contacts. The return value is True or False.  
";

%feature("docstring") com_equilibrium_2d "

Tests whether the given COM com is stable for the given contacts and the given
external force fext.  

The 2-argument version is a \"fancy\" version that allows more control over the
constraint planes.  

Args:  

    contacts (list of 4-float lists or tuples): the list of contacts, each
        specified as a 4-list or tuple [x,y,theta,k], with:

        - (x,y,z): the contact position
        - theta: is the normal angle (in radians, CCW to the x axis)
        - k: the coefficient of friction (>= 0)

    contactPositions (list of 2-float lists or tuples): the list of contact
        point positions.
    frictionCones (list of lists): The i'th element in this list has length
        k*3 (for some integer k), and gives the contact force constraints
        (ax,ay,b) where ax*fx+ay*fy <= b limits the contact force (fx,fy)
        at the i'th contact. Each of the k 3-tuples is laid out sequentially
        per-contact.
    fext (2-tuple or list): the external force vector.
    com (2-tuple or list, or None): the center of mass coordinates.  If None,
        assumes that you want to test whether ANY COM may be in equilibrium
        for the given contacts.  

Returns:  

    bool, None, or list: if com is given, and there are feasible
    equilibrium forces, this returns a list of 2-tuples giving
    equilibrium forces at each of the contacts. None is returned if
    no such forces exist.

    If com = None, the result is True or False.
  
";

%feature("docstring") support_polygon "

Calculates the support polygon for a given set of contacts and a downward
external force (0,0,-g). A contact point is given by a list of 7 floats,
[x,y,z,nx,ny,nz,k] as usual.  

The return value is a list of 3-tuples giving the sorted plane boundaries of the
polygon. The format of a plane is (nx,ny,ofs) where (nx,ny) are the outward
facing normals, and ofs is the offset from 0. In other words to test stability
of a com [x,y], you can test whether dot([nx,ny],[x,y]) <= ofs for all planes.  
";

%feature("docstring") support_polygon "

Calculates the support polygon for a given set of contacts and a downward
external force (0,0,-g).  

In the 1-argument version, a contact point is given by a list of 7 floats,
[x,y,z,nx,ny,nz,k] as usual. The 2-argument version is a \"fancy\" version that
allows more control over the constraint planes.  

Args:  

    contacts (list of 7-float lists or tuples): the list of contacts, each
        specified as a 7-list or tuple [x,y,z,nx,ny,nz,k], with:

        - (x,y,z): the contact position
        - (nx,ny,nz): the contact normal
        - k: the coefficient of friction (>= 0)

    contactPositions (list of 3-float lists or tuples): the list of contact
        point positions.
    frictionCones (list of lists): Each item of this list specifies linear
        inequalities that must be met of the force at the corresponding
        contact point.  The item must have length k*4 where k is an integer,
        and each inequality gives the entries (ax,ay,az,b) of a constraint
        ax*fx+ay*fy+az*fz <= b that limits the contact force (fx,fy,fz) at
        the i'th contact.  Each of the k 4-tuples is laid out sequentially
        per-contact.  

Returns:  

    list of 3-tuples: The sorted plane boundaries of the support
    polygon. The format of a plane is (nx,ny,ofs) where (nx,ny) are the
    outward facing normals, and ofs is the offset from 0.  In other words
    to test stability of a com with x-y coordinates [x,y], you can test
    whether dot([nx,ny],[x,y]) <= ofs  for all planes.

    Hint: with numpy, you can do::

        Ab = np.array(supportPolygon(args))
        A=Ab[:,0:2]
        b=Ab[:,2]
        myComEquilibrium = lambda x: np.all(np.dot(A,x)<=b)
  
";

%feature("docstring") support_polygon_2d "

Calculates the support polygon (interval) for a given set of contacts and a
downward external force (0,-g). A contact point is given by a list of 4 floats,
[x,y,theta,k] as usual.  

The return value is a 2-tuple giving the min / max extents of the support
polygon. If they are both infinite, the support polygon is empty.  
";

%feature("docstring") support_polygon_2d "

Calculates the support polygon (interval) for a given set of contacts and a
downward external force (0,-g).  

The 2-argument version is a \"fancy\" version that allows more control over the
constraint planes.  

Args:  

    contacts (list of 4-float lists or tuples): the list of contacts, each
        specified as a 4-list or tuple [x,y,theta,k], with:

        - (x,y,z): the contact position
        - theta: is the normal angle (in radians, CCW to the x axis)
        - k: the coefficient of friction (>= 0)

    contactPositions (list of 2-float lists or tuples): the list of contact
        point positions.
    frictionCones (list of lists): The i'th element in this list has length
         k*3 (for some integer k), and gives the contact force constraints
         (ax,ay,b) where ax*fx+ay*fy <= b limits the contact force (fx,fy)
         at the i'th contact. Each of the k 3-tuples is laid out sequentially
         per-contact.  

Returns:  

    2-tuple: gives the min/max extents of the support polygon.
    If the support interval is empty, (inf,inf) is returned.
  
";

%feature("docstring") equilibrium_torques "

Solves for the torques / forces that keep the robot balanced against gravity.  

Args:  

    robot (RobotModel): the robot model, posed in its current configuration
    contacts (np.ndarray): an nx7 array of contact points, each given as 7-lists
[x,y,z,nx,ny,nz,kFriction]
    links (list of n ints): a list of the links on which those contact points
lie
    fext (Vector3): the external force (e.g., gravity)
    norm (float): the torque norm to minimize.  If 0, minimizes the l-infinity
norm (default)
        If 1, minimizes the l-1 norm.  If 2, minimizes the l-2 norm
(experimental,
         may not get good results)  

Returns:  

    tuple: a pair (t,f) giving the joint torques and frictional
    contact forces, if a solution exists, or None if no solution exists.
  
";

%feature("docstring") equilibrium_torques "

Solves for the torques / forces that keep the robot balanced against gravity.  

The problem being solved is  

:math:`min_{t,f_1,...,f_N} \\|t\\|_p`  

:math:`s.t. t_{int} + G(q) = t + sum_{i=1}^N J_i(q)^T f_i`  

:math:`|t| \\leq t_{max}`  

:math:`f_i \\in FC_i`  

Args:  

    robot (RobotModel): the robot, posed in its current configuration
    contacts (ndarray): an N x 7 array of contact points, each given as 7-lists
        [x,y,z,nx,ny,nz,kFriction]
    links (list of N ints): a list of the links on which those contact points
        lie
    fext (list of 3 floats): the external force (e.g., gravity)
    norm (double): the torque norm to minimize.  

        * If 0, minimizes the l-infinity norm (default)
        * If 1, minimizes the l-1 norm.
        * If 2, minimizes the l-2 norm (experimental, may not get good results).

    internalTorques (list of robot.numLinks() floats, optional): allows you to
        solve for dynamic situations, e.g., with coriolis forces taken into
        account.  These are added to the RHS of the torque balance equation.
        If not given, t_int is assumed to be zero.

        To use dynamics, set the robot's joint velocities dq, calculate
        then calculate the torques via robot.torquesFromAccel(ddq), and pass
        the result into internalTorques.  

Returns:  

    pair of lists, optional: a pair (torque,force) if a solution exists,
    giving valid joint torques t and frictional contact forces (f1,...,fn).
    None is returned if no solution exists.
  
";

// File: viewport_8h.xml

// File: widget_8h.xml

