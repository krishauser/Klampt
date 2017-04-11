
// File: index.xml

// File: structContactParameters.xml
%feature("docstring") ContactParameters "

Stores contact parameters for an entity. Currently only used for
simulation, but could be used for contact mechanics in the future.

C++ includes: robotmodel.h ";


// File: classCSpaceInterface.xml
%feature("docstring") CSpaceInterface "

A raw interface for a configuration space. Note: the native Python
CSpace interface class in cspace.py is easier to use.

You can either set a single feasibility test function using
setFeasibility() or add several feasibility tests, all of which need
to be satisfied, using addFeasibilityTest(). In the latter case,
planners may be able to provide debugging statistics, solve Minimum
Constraint Removal problems, run faster by eliminating constraint
tests, etc.

Either setVisibility() or setVisibilityEpsilon() must be called to
define a visibility checker between two (feasible) configurations. In
the latter case, the path will be discretized at the resolution sent
to setVisibilityEpsilon. If you have special single-constraint
visibility tests, you can call that using addVisibilityTest (for
example, for convex constraints you can set it to the lambda function
that returns true regardless of its arguments).

Supported properties include \"euclidean\" (boolean), \"metric\"
(string), \"geodesic\" (boolean), \"minimum\" (vector), and
\"maximum\" (vector). These may be used by planners to make planning
faster or more accurate. For a complete list see
KrisLibrary/planning/CSpace.h.

C++ includes: motionplanning.h ";

%feature("docstring")  CSpaceInterface::CSpaceInterface "";

%feature("docstring")  CSpaceInterface::CSpaceInterface "";

%feature("docstring")  CSpaceInterface::~CSpaceInterface "";

%feature("docstring")  CSpaceInterface::destroy "";

%feature("docstring")  CSpaceInterface::setFeasibility "";

%feature("docstring")  CSpaceInterface::addFeasibilityTest "";

%feature("docstring")  CSpaceInterface::setVisibility "";

%feature("docstring")  CSpaceInterface::addVisibilityTest "";

%feature("docstring")  CSpaceInterface::setVisibilityEpsilon "";

%feature("docstring")  CSpaceInterface::setSampler "";

%feature("docstring")  CSpaceInterface::setNeighborhoodSampler "";

%feature("docstring")  CSpaceInterface::setDistance "";

%feature("docstring")  CSpaceInterface::setInterpolate "";

%feature("docstring")  CSpaceInterface::setProperty "";

%feature("docstring")  CSpaceInterface::getProperty "";

%feature("docstring")  CSpaceInterface::isFeasible "

Queries whether a given configuration is feasible.

queries ";

%feature("docstring")  CSpaceInterface::isVisible "

Queries whether two configurations are visible. ";

%feature("docstring")  CSpaceInterface::testFeasibility "

Queries whether a given configuration is feasible with respect to a
given constraint. ";

%feature("docstring")  CSpaceInterface::testVisibility "

Queries whether two configurations are visible with respect to a given
constraint. ";

%feature("docstring")  CSpaceInterface::feasibilityFailures "

Returns a list of all failed feasibility constraints. ";

%feature("docstring")  CSpaceInterface::visibilityFailures "

Returns a list of all failed visibility constraints. ";

%feature("docstring")  CSpaceInterface::sample "

Samples a configuration. ";

%feature("docstring")  CSpaceInterface::distance "

Returns the distance between two configurations. ";

%feature("docstring")  CSpaceInterface::interpolate "

Interpolates between two configurations. ";

%feature("docstring")  CSpaceInterface::adaptiveQueriesEnabled "

optional: adaptive queries can be used to automatically minimize the
total cost of testing feasibility / visibility using empirical
estimates. Off by default. ";

%feature("docstring")  CSpaceInterface::enableAdaptiveQueries "

Call this to enable adaptive queries. (It has a small overhead.) ";

%feature("docstring")  CSpaceInterface::optimizeQueryOrder "

Call this to optimize the feasibility / visibility testing order. ";

%feature("docstring")  CSpaceInterface::setFeasibilityDependency "

Marks that a certain feasibility test must be performed before
another. ";

%feature("docstring")  CSpaceInterface::setFeasibilityPrior "

Resets the data for a certain feasibility test. Default values give a
data-gathering behavior. ";

%feature("docstring")  CSpaceInterface::setVisibilityDependency "

Marks that a certain feasibility test must be performed before
another. ";

%feature("docstring")  CSpaceInterface::setVisibilityPrior "

Resets the data for a certain visibility test. Default values give a
data-gathering behavior. ";

%feature("docstring")  CSpaceInterface::feasibilityCost "

Retrieves the empirical average cost of a given feasibility test. ";

%feature("docstring")  CSpaceInterface::feasibilityProbability "

Retrieves the empirical average success rate of a given feasibility
test. ";

%feature("docstring")  CSpaceInterface::visibilityCost "

Retrieves the empirical average cost of a given visibility test. ";

%feature("docstring")  CSpaceInterface::visibilityProbability "

Retrieves the empirical average success rate of a given visibility
test. ";

%feature("docstring")  CSpaceInterface::feasibilityQueryOrder "

Retrieves the current order of feasibility tests. ";

%feature("docstring")  CSpaceInterface::visibilityQueryOrder "

Retrieves the current order of visibility tests. ";

%feature("docstring")  CSpaceInterface::getStats "

Returns constraint testing statistics. If adaptive queries are
enabled, this returns the stats on each constraint. ";


// File: classGeneralizedIKObjective.xml
%feature("docstring") GeneralizedIKObjective "

An inverse kinematics target for matching points between two robots
and/or objects.

The objects are chosen upon construction, so the following are valid:
GeneralizedIKObjective(a) is an objective for object a to be
constrained relative to the environment.

GeneralizedIKObjective(a,b) is an objective for object a to be
constrained relative to b. Here a and b can be links on any robot or
rigid objects.

Once constructed, call setPoint, setPoints, or setTransform to specify
the nature of the constraint.

C++ includes: robotik.h ";

%feature("docstring")  GeneralizedIKObjective::GeneralizedIKObjective
"";

%feature("docstring")  GeneralizedIKObjective::GeneralizedIKObjective
"";

%feature("docstring")  GeneralizedIKObjective::GeneralizedIKObjective
"";

%feature("docstring")  GeneralizedIKObjective::GeneralizedIKObjective
"";

%feature("docstring")  GeneralizedIKObjective::GeneralizedIKObjective
"";

%feature("docstring")  GeneralizedIKObjective::GeneralizedIKObjective
"";

%feature("docstring")  GeneralizedIKObjective::GeneralizedIKObjective
"";

%feature("docstring")  GeneralizedIKObjective::setPoint "";

%feature("docstring")  GeneralizedIKObjective::setPoints "";

%feature("docstring")  GeneralizedIKObjective::setTransform "";


// File: classGeneralizedIKSolver.xml
%feature("docstring") GeneralizedIKSolver "

An inverse kinematics solver between multiple robots and/or objects.
NOT IMPLEMENTED YET.

C++ includes: robotik.h ";

%feature("docstring")  GeneralizedIKSolver::GeneralizedIKSolver "";

%feature("docstring")  GeneralizedIKSolver::add "

Adds a new simultaneous objective. ";

%feature("docstring")  GeneralizedIKSolver::setMaxIters "

Sets the max # of iterations (default 100) ";

%feature("docstring")  GeneralizedIKSolver::setTolerance "

Sets the constraint solve tolerance (default 1e-3) ";

%feature("docstring")  GeneralizedIKSolver::getResidual "

Returns a vector describing the error of the objective. ";

%feature("docstring")  GeneralizedIKSolver::getJacobian "

Returns a matrix describing the instantaneous derivative of the
objective with respect to the active parameters. ";

%feature("docstring")  GeneralizedIKSolver::solve "

Tries to find a configuration that satifies all simultaneous
objectives up to the desired tolerance. Returns (res,iters) where res
indicates whether x converged. ";

%feature("docstring")  GeneralizedIKSolver::sampleInitial "

Samples an initial random configuration. ";


// File: structGeometricPrimitive.xml
%feature("docstring") GeometricPrimitive "

A geometric primitive. So far only points, spheres, segments, and
AABBs can be constructed manually in the Python API.

C++ includes: geometry.h ";

%feature("docstring")  GeometricPrimitive::setPoint "";

%feature("docstring")  GeometricPrimitive::setSphere "";

%feature("docstring")  GeometricPrimitive::setSegment "";

%feature("docstring")  GeometricPrimitive::setAABB "";

%feature("docstring")  GeometricPrimitive::loadString "";

%feature("docstring")  GeometricPrimitive::saveString "";


// File: classGeometry3D.xml
%feature("docstring") Geometry3D "

A three-D geometry. Can either be a reference to a world item's
geometry, in which case modifiers change the world item's geometry, or
it can be a standalone geometry.

There are four currently supported types of geometry: primitives (
GeometricPrimitive)

triangle meshes ( TriangleMesh)

point clouds ( PointCloud)

groups (Group) This class acts as a uniform container of all of these
types.

Each geometry stores a \"current\" transform, which is automatically
updated for world items' geometries. The proximity queries are
performed with respect to the transformed geometries (note the
underlying geometry is not changed, which could be computationally
expensive. The query is performed, however, as though they were).

If you want to set a world item's geometry to be equal to a standalone
geometry, use the set(rhs) function rather than the assignment (=)
operator.

Modifiers include any setX() functions, translate(), and transform().

Proximity queries include collides(), withinDistance(), distance(),
closestPoint(), and rayCast().

Each object also has a \"collision margin\" which may virtually fatten
the object, as far as proximity queries are concerned. This is useful
for setting collision avoidance margins in motion planning. By default
it is zero. (Note that this is NOT the same thing as simulation body
collision padding!)

C++ includes: geometry.h ";

%feature("docstring")  Geometry3D::Geometry3D "";

%feature("docstring")  Geometry3D::Geometry3D "";

%feature("docstring")  Geometry3D::Geometry3D "";

%feature("docstring")  Geometry3D::Geometry3D "";

%feature("docstring")  Geometry3D::Geometry3D "";

%feature("docstring")  Geometry3D::~Geometry3D "";

%feature("docstring")  Geometry3D::clone "

Creates a standalone geometry from this geometry. ";

%feature("docstring")  Geometry3D::set "

Copies the geometry of the argument into this geometry. ";

%feature("docstring")  Geometry3D::isStandalone "

Returns true if this is a standalone geometry. ";

%feature("docstring")  Geometry3D::free "

Frees the data associated with this geometry, if standalone. ";

%feature("docstring")  Geometry3D::type "

Returns the type of geometry: TriangleMesh, PointCloud, or
GeometricPrimitive. ";

%feature("docstring")  Geometry3D::empty "

Returns true if this has no contents (not the same as
numElements()==0) ";

%feature("docstring")  Geometry3D::getTriangleMesh "

Returns a TriangleMesh if this geometry is of type TriangleMesh. ";

%feature("docstring")  Geometry3D::getPointCloud "

Returns a PointCloud if this geometry is of type PointCloud. ";

%feature("docstring")  Geometry3D::getGeometricPrimitive "

Returns a GeometricPrimitive if this geometry is of type
GeometricPrimitive. ";

%feature("docstring")  Geometry3D::setTriangleMesh "

Sets this Geometry3D to a TriangleMesh. ";

%feature("docstring")  Geometry3D::setPointCloud "

Sets this Geometry3D to a PointCloud. ";

%feature("docstring")  Geometry3D::setGeometricPrimitive "

Sets this Geometry3D to a GeometricPrimitive. ";

%feature("docstring")  Geometry3D::setGroup "

Sets this Geometry3D to a group geometry. To add sub-geometries,
repeatedly call setElement() ";

%feature("docstring")  Geometry3D::getElement "

Returns an element of the Geometry3D if it is a group. Raises an error
if this is of any other type. ";

%feature("docstring")  Geometry3D::setElement "

Sets an element of the Geometry3D if it is a group. Raises an error if
this is of any other type. ";

%feature("docstring")  Geometry3D::numElements "

Returns the number of sub-elements in this geometry. ";

%feature("docstring")  Geometry3D::loadFile "

Loads from file. Standard mesh types, PCD files, and .geom files are
supported. ";

%feature("docstring")  Geometry3D::saveFile "

Saves to file. Standard mesh types, PCD files, and .geom files are
supported. ";

%feature("docstring")  Geometry3D::setCurrentTransform "

Sets the current transformation (not modifying the underlying data) ";

%feature("docstring")  Geometry3D::getCurrentTransform "

Gets the current transformation. ";

%feature("docstring")  Geometry3D::translate "

Translates the geometry data. Permanently modifies the data and resets
any collision data structures. ";

%feature("docstring")  Geometry3D::scale "

Scales the geometry data uniformly. Permanently modifies the data and
resets any collision data structures. ";

%feature("docstring")  Geometry3D::scale "

Scales the geometry data with different factors on each axis.
Permanently modifies the data and resets any collision data
structures. ";

%feature("docstring")  Geometry3D::rotate "

Rotates the geometry data. Permanently modifies the data and resets
any collision data structures. ";

%feature("docstring")  Geometry3D::transform "

Translates/rotates/scales the geometry data. Permanently modifies the
data and resets any collision data structures. ";

%feature("docstring")  Geometry3D::setCollisionMargin "

Sets a padding around the base geometry which affects the results of
proximity queries. ";

%feature("docstring")  Geometry3D::getCollisionMargin "

Returns the padding around the base geometry. Default 0. ";

%feature("docstring")  Geometry3D::getBB "

Returns the axis-aligned bounding box of the object. ";

%feature("docstring")  Geometry3D::collides "

Returns true if this geometry collides with the other. ";

%feature("docstring")  Geometry3D::withinDistance "

Returns true if this geometry is within distance tol to other. ";

%feature("docstring")  Geometry3D::distance "

Returns the distance from this geometry to the other. ";

%feature("docstring")  Geometry3D::closestPoint "

Returns (success,cp) giving the closest point to the input point.
success is false if that operation is not supported with the given
geometry type. cp are given in world coordinates. ";

%feature("docstring")  Geometry3D::rayCast "

Returns (hit,pt) where hit is true if the ray starting at s and
pointing in direction d hits the geometry (given in world
coordinates); pt is the hit point, in world coordinates. ";


// File: classIKObjective.xml
%feature("docstring") IKObjective "

A class defining an inverse kinematic target. Either a link on a robot
can take on a fixed position/orientation in the world frame, or a
relative position/orientation to another frame.

Currently only fixed-point constraints and fixed-transform constraints
are implemented in the Python API.

C++ includes: robotik.h ";

%feature("docstring")  IKObjective::IKObjective "";

%feature("docstring")  IKObjective::link "

The index of the robot link that is constrained. ";

%feature("docstring")  IKObjective::destLink "

The index of the destination link, or -1 if fixed to the world. ";

%feature("docstring")  IKObjective::numPosDims "

Returns the number of position dimensions constrained (0-3) ";

%feature("docstring")  IKObjective::numRotDims "

Returns the number of rotation dimensions constrained (0-3) ";

%feature("docstring")  IKObjective::setFixedPoint "

Sets a fixed-point constraint. ";

%feature("docstring")  IKObjective::setFixedPoints "

Sets a multiple fixed-point constraint. ";

%feature("docstring")  IKObjective::setFixedTransform "

Sets a fixed-transform constraint (R,t) ";

%feature("docstring")  IKObjective::setRelativePoint "

Sets a fixed-point constraint relative to link2. ";

%feature("docstring")  IKObjective::setRelativePoints "

Sets a multiple fixed-point constraint relative to link2. ";

%feature("docstring")  IKObjective::setRelativeTransform "

Sets a fixed-transform constraint (R,t) relative to linkTgt. ";

%feature("docstring")  IKObjective::setLinks "

Manual construction. ";

%feature("docstring")  IKObjective::setFreePosition "

Manual: Sets a free position constraint. ";

%feature("docstring")  IKObjective::setFixedPosConstraint "

Manual: Sets a fixed position constraint. ";

%feature("docstring")  IKObjective::setPlanarPosConstraint "

Manual: Sets a planar position constraint nworld^T T(link)*tlocal +
oworld = 0. ";

%feature("docstring")  IKObjective::setLinearPosConstraint "

Manual: Sets a linear position constraint T(link)*tlocal = sworld +
u*dworld for some real value u. ";

%feature("docstring")  IKObjective::setFreeRotConstraint "

Manual: Sets a free rotation constraint. ";

%feature("docstring")  IKObjective::setFixedRotConstraint "

Manual: Sets a fixed rotation constraint. ";

%feature("docstring")  IKObjective::setAxialRotConstraint "

Manual: Sets an axial rotation constraint. ";

%feature("docstring")  IKObjective::getPosition "

Returns the local and global position of the position constraint. ";

%feature("docstring")  IKObjective::getPositionDirection "

For linear and planar constraints, returns the direction. ";

%feature("docstring")  IKObjective::getRotation "

For fixed rotation constraints, returns the orientation. ";

%feature("docstring")  IKObjective::getRotationAxis "

For axis rotation constraints, returns the local and global axes. ";

%feature("docstring")  IKObjective::getTransform "

For fixed-transform constraints, returns the transform (R,t) ";

%feature("docstring")  IKObjective::transform "

Tranforms the target position/rotation of this IK constraint by
transform (R,t) ";

%feature("docstring")  IKObjective::transformLocal "

Tranforms the local position/rotation of this IK constraint by
transform (R,t) ";

%feature("docstring")  IKObjective::matchDestination "

Sets the destination coordinates of this constraint to fit the given
target transform. In other words, if (R,t) is the current link
transform, this sets the destination position / orientation so that
this objective has zero error. The current position/rotation
constraint types are kept. ";

%feature("docstring")  IKObjective::loadString "

Loads the objective from a Klamp't-native formatted string. For a more
readable but verbose format, try the JSON IO routines
loader.toJson/fromJson() ";

%feature("docstring")  IKObjective::saveString "

Saves the objective to a Klamp't-native formatted string. For a more
readable but verbose format, try the JSON IO routines
loader.toJson/fromJson() ";


// File: classIKSolver.xml
%feature("docstring") IKSolver "

An inverse kinematics solver based on the Newton-Raphson technique.

Typical calling pattern is s = IKSolver(robot) s.add(objective1)
s.add(objective2) s.setMaxIters(100) s.setTolerance(1e-4) res =
s.solve() if res: print \"IK solution:\",robot.getConfig(),\"found
in\",s.lastSolveIters(),\"iterations, residual\",s.getResidual() else:
print \"IK failed:\",robot.getConfig(),\"found
in\",s.lastSolveIters(),\"iterations, residual\",s.getResidual()

sampleInitial() is a convenience routine. More initial configurations
can be sampled in case the prior configs lead to local minima.

C++ includes: robotik.h ";

%feature("docstring")  IKSolver::IKSolver "";

%feature("docstring")  IKSolver::IKSolver "";

%feature("docstring")  IKSolver::add "

Adds a new simultaneous objective. ";

%feature("docstring")  IKSolver::set "

Assigns an existing objective added by add. ";

%feature("docstring")  IKSolver::clear "

Clears objectives. ";

%feature("docstring")  IKSolver::setMaxIters "

Sets the max # of iterations (default 100) ";

%feature("docstring")  IKSolver::getMaxIters "

Gets the max # of iterations. ";

%feature("docstring")  IKSolver::setTolerance "

Sets the constraint solve tolerance (default 1e-3) ";

%feature("docstring")  IKSolver::getTolerance "

Gets the constraint solve tolerance. ";

%feature("docstring")  IKSolver::setActiveDofs "

Sets the active degrees of freedom. ";

%feature("docstring")  IKSolver::getActiveDofs "

Gets the active degrees of freedom. ";

%feature("docstring")  IKSolver::setJointLimits "

Sets limits on the robot's configuration. If empty, this turns off
joint limits. ";

%feature("docstring")  IKSolver::getJointLimits "

Gets the limits on the robot's configuration (by default this is the
robot's joint limits. ";

%feature("docstring")  IKSolver::setBiasConfig "

Biases the solver to approach a given configuration. Setting an empty
vector clears the bias term. ";

%feature("docstring")  IKSolver::getBiasConfig "

Gets the solvers' bias configuration. ";

%feature("docstring")  IKSolver::isSolved "

Returns true if the current configuration residual is less than tol.
";

%feature("docstring")  IKSolver::getResidual "

Returns a vector describing the error of the objective at the current
configuration. ";

%feature("docstring")  IKSolver::getJacobian "

Returns a matrix describing the instantaneous derivative of the
objective with respect to the active Dofs. ";

%feature("docstring")  IKSolver::solve "

Tries to find a configuration that satifies all simultaneous
objectives up to the desired tolerance. Returns true if x converged.
";

%feature("docstring")  IKSolver::solve "

Old-style: will be deprecated. Specify # of iterations and tolerance.
Tries to find a configuration that satifies all simultaneous
objectives up to the desired tolerance. Returns (res,iterations) where
res is true if x converged. ";

%feature("docstring")  IKSolver::lastSolveIters "

Returns the number of Newton-Raphson iterations used in the last
solve() call. ";

%feature("docstring")  IKSolver::sampleInitial "

Samples an initial random configuration. ";


// File: classManualOverrideController.xml
%feature("docstring") ManualOverrideController "";

%feature("docstring")
ManualOverrideController::ManualOverrideController "";

%feature("docstring")  ManualOverrideController::Type "";

%feature("docstring")  ManualOverrideController::Update "";

%feature("docstring")  ManualOverrideController::ReadState "";

%feature("docstring")  ManualOverrideController::WriteState "";

%feature("docstring")  ManualOverrideController::Settings "";

%feature("docstring")  ManualOverrideController::GetSetting "";

%feature("docstring")  ManualOverrideController::SetSetting "";

%feature("docstring")  ManualOverrideController::Commands "";

%feature("docstring")  ManualOverrideController::SendCommand "";


// File: structMass.xml
%feature("docstring") Mass "

Stores mass information for a rigid body or robot link. Note: you
should use the set/get functions rather than changing the members
directly due to strangeness in SWIG's handling of vectors.

C++ includes: robotmodel.h ";

%feature("docstring")  Mass::setMass "";

%feature("docstring")  Mass::getMass "";

%feature("docstring")  Mass::setCom "";

%feature("docstring")  Mass::getCom "";

%feature("docstring")  Mass::setInertia "";

%feature("docstring")  Mass::getInertia "";


// File: classObjectPoser.xml
%feature("docstring") ObjectPoser "";

%feature("docstring")  ObjectPoser::ObjectPoser "";

%feature("docstring")  ObjectPoser::set "";

%feature("docstring")  ObjectPoser::get "";


// File: classPlannerInterface.xml
%feature("docstring") PlannerInterface "

An interface for a motion planner. The MotionPlanner interface in
cspace.py is somewhat easier to use.

On construction, uses the planner type specified by setPlanType and
the settings currently specified by calls to setPlanSetting.

Point-to-point planning is enabled by sending two configurations to
the setEndpoints method. This is mandatory for RRT and SBL-style
planners. The start and end milestones are given by indices 0 and 1,
respectively

Point-to-set planning is enabled by sending a goal test as the second
argument to the setEndpoints method. It is possible also to send a
special goal sampler by providing a pair of functions as the second
argument consisting of the two functions (goaltest,goalsample). The
first in this pair tests whether a configuration is a goal, and the
second returns a sampled configuration in a superset of the goal.
Ideally the goal sampler generates as many goals as possible.

PRM can be used in either point-to-point or multi-query mode. In
multi-query mode, you may call addMilestone(q) to add a new milestone.
addMilestone() returns the index of that milestone, which can be used
in later calls to getPath().

To plan, call planMore(iters) until getPath(0,1) returns non-NULL. The
return value is a list of configurations.

To get a roadmap (V,E), call getRoadmap(). V is a list of
configurations (each configuration is a Python list) and E is a list
of edges (each edge is a pair (i,j) indexing into V).

To dump the roadmap to disk, call dump(fn). This saves to a Trivial
Graph Format (TGF) format.

C++ includes: motionplanning.h ";

%feature("docstring")  PlannerInterface::PlannerInterface "";

%feature("docstring")  PlannerInterface::~PlannerInterface "";

%feature("docstring")  PlannerInterface::destroy "";

%feature("docstring")  PlannerInterface::setEndpoints "";

%feature("docstring")  PlannerInterface::setEndpointSet "";

%feature("docstring")  PlannerInterface::addMilestone "";

%feature("docstring")  PlannerInterface::planMore "";

%feature("docstring")  PlannerInterface::getPathEndpoints "";

%feature("docstring")  PlannerInterface::getPath "";

%feature("docstring")  PlannerInterface::getData "";

%feature("docstring")  PlannerInterface::getStats "";

%feature("docstring")  PlannerInterface::getRoadmap "";

%feature("docstring")  PlannerInterface::dump "";


// File: structPointCloud.xml
%feature("docstring") PointCloud "

A 3D point cloud class.

Attributes: vertices: a list of vertices, given as a list [x1, y1, z1,
x2, y2, ... zn]

properties: a list of vertex properties, given as a list [p11, p21,
..., pk1, p12, p22, ..., pk2, ... , pn1, pn2, ..., pn2] where each
vertex has k properties. The name of each property is given by the
propertyNames member.

Note: because the bindings are generated by SWIG, you can access the
vertices/properties/propertyName members via some automatically
generated accessors / modifiers. In particular len(), append(), and
indexing via [] are useful. Some other methods like resize() are also
provided. However, you CANNOT set these items via assignment.

Examples:

pc = PointCloud() pc.propertyNames.append('rgb') pc.vertices.append(0)
pc.vertices.append(0) pc.vertices.append(0) pc.properties.append(0)
print len(pc.vertices) #prints 3 print pc.numPoints() #prints 1
pc.addPoint([1,2,3]) print pc.numPoints() #prints 2 print
len(pc.properties.size()) #prints 2: 1 property category x 2 points
print pc.getProperty(1,0) #prints 0; this is the default value added
when addPoint is called

C++ includes: geometry.h ";

%feature("docstring")  PointCloud::numPoints "

Returns the number of points. ";

%feature("docstring")  PointCloud::numProperties "

Returns the number of properties. ";

%feature("docstring")  PointCloud::setPoints "

Sets all the points to the given list (a 3n-list) ";

%feature("docstring")  PointCloud::addPoint "

Adds a point. Sets all its properties to 0. Returns the index. ";

%feature("docstring")  PointCloud::setPoint "

Sets the position of the point at the given index to p. ";

%feature("docstring")  PointCloud::getPoint "

Retrieves the position of the point at the given index. ";

%feature("docstring")  PointCloud::addProperty "

Adds a new property. All values for this property are set to 0. ";

%feature("docstring")  PointCloud::addProperty "

Adds a new property with name pname, and sets values for this property
to the given list (a n-list) ";

%feature("docstring")  PointCloud::setProperties "

Sets all the properties of all points to the given list (a kn-list) ";

%feature("docstring")  PointCloud::setProperties "

Sets property pindex of all points to the given list (a n-list) ";

%feature("docstring")  PointCloud::setProperty "

Sets property pindex of point index to the given value. ";

%feature("docstring")  PointCloud::setProperty "

Sets the property named pname of point index to the given value. ";

%feature("docstring")  PointCloud::getProperty "

Gets property pindex of point index. ";

%feature("docstring")  PointCloud::getProperty "

Gets the property named pname of point index. ";

%feature("docstring")  PointCloud::translate "

Translates all the points by v=v+t. ";

%feature("docstring")  PointCloud::transform "

Transforms all the points by the rigid transform v=R*v+t. ";

%feature("docstring")  PointCloud::join "

Adds the given point cloud to this one. They must share the same
properties or else an exception is raised. ";

%feature("docstring")  PointCloud::setSetting "

Sets the given setting. ";

%feature("docstring")  PointCloud::getSetting "

Retrieves the given setting. ";


// File: classPointPoser.xml
%feature("docstring") PointPoser "";

%feature("docstring")  PointPoser::PointPoser "";

%feature("docstring")  PointPoser::set "";

%feature("docstring")  PointPoser::get "";

%feature("docstring")  PointPoser::setAxes "

Sets the reference axes (by default aligned to x,y,z) ";


// File: classPyConstraintSet.xml
%feature("docstring") PyConstraintSet "";

%feature("docstring")  PyConstraintSet::PyConstraintSet "";

%feature("docstring")  PyConstraintSet::~PyConstraintSet "";

%feature("docstring")  PyConstraintSet::Sample "";

%feature("docstring")  PyConstraintSet::Contains "";


// File: classPyCSpace.xml
%feature("docstring") PyCSpace "

A CSpace that calls python routines for its functionality ";

%feature("docstring")  PyCSpace::PyCSpace "";

%feature("docstring")  PyCSpace::~PyCSpace "";

%feature("docstring")  PyCSpace::UpdateTempConfig "";

%feature("docstring")  PyCSpace::UpdateTempConfig2 "";

%feature("docstring")  PyCSpace::ConstraintIndex "";

%feature("docstring")  PyCSpace::Sample "";

%feature("docstring")  PyCSpace::SampleNeighborhood "";

%feature("docstring")  PyCSpace::IsFeasible "";

%feature("docstring")  PyCSpace::IsFeasible "";

%feature("docstring")  PyCSpace::IsVisible "";

%feature("docstring")  PyCSpace::IsVisible "";

%feature("docstring")  PyCSpace::PathChecker "";

%feature("docstring")  PyCSpace::PathChecker "";

%feature("docstring")  PyCSpace::Distance "";

%feature("docstring")  PyCSpace::Interpolate "";

%feature("docstring")  PyCSpace::Properties "";


// File: classPyEdgePlanner.xml
%feature("docstring") PyEdgePlanner "";

%feature("docstring")  PyEdgePlanner::PyEdgePlanner "";

%feature("docstring")  PyEdgePlanner::~PyEdgePlanner "";

%feature("docstring")  PyEdgePlanner::IsVisible "";

%feature("docstring")  PyEdgePlanner::Length "";

%feature("docstring")  PyEdgePlanner::Eval "";

%feature("docstring")  PyEdgePlanner::Start "";

%feature("docstring")  PyEdgePlanner::End "";

%feature("docstring")  PyEdgePlanner::Space "";

%feature("docstring")  PyEdgePlanner::Copy "";

%feature("docstring")  PyEdgePlanner::ReverseCopy "";


// File: classPyGoalSet.xml
%feature("docstring") PyGoalSet "";

%feature("docstring")  PyGoalSet::PyGoalSet "";

%feature("docstring")  PyGoalSet::~PyGoalSet "";

%feature("docstring")  PyGoalSet::Sample "";

%feature("docstring")  PyGoalSet::Contains "";


// File: classPyUpdateEdgePlanner.xml
%feature("docstring") PyUpdateEdgePlanner "";

%feature("docstring")  PyUpdateEdgePlanner::PyUpdateEdgePlanner "";

%feature("docstring")  PyUpdateEdgePlanner::UpdateCSpace "";

%feature("docstring")  PyUpdateEdgePlanner::IsVisible "";


// File: classRigidObjectModel.xml
%feature("docstring") RigidObjectModel "

A rigid movable object.

A rigid object has a name, geometry, appearance, mass, surface
properties, and current transform / velocity.

State is retrieved/set using get/setTransform, and get/setVelocity

C++ includes: robotmodel.h ";

%feature("docstring")  RigidObjectModel::RigidObjectModel "";

%feature("docstring")  RigidObjectModel::getID "";

%feature("docstring")  RigidObjectModel::getName "";

%feature("docstring")  RigidObjectModel::setName "";

%feature("docstring")  RigidObjectModel::geometry "";

%feature("docstring")  RigidObjectModel::appearance "";

%feature("docstring")  RigidObjectModel::getMass "";

%feature("docstring")  RigidObjectModel::setMass "";

%feature("docstring")  RigidObjectModel::getContactParameters "";

%feature("docstring")  RigidObjectModel::setContactParameters "";

%feature("docstring")  RigidObjectModel::getTransform "

Retrieves the rotation / translation of the rigid object (R,t) ";

%feature("docstring")  RigidObjectModel::setTransform "

Sets the rotation / translation (R,t) of the rigid object. ";

%feature("docstring")  RigidObjectModel::getVelocity "

Retrieves the (angular velocity, velocity) of the rigid object. ";

%feature("docstring")  RigidObjectModel::setVelocity "

Sets the (angular velocity, velocity) of the rigid object. ";

%feature("docstring")  RigidObjectModel::drawGL "

Draws the object's geometry. If keepAppearance=true, the current
appearance is honored. Otherwise, only the raw geometry is drawn.
PERFORMANCE WARNING: if keepAppearance is false, then this does not
properly reuse OpenGL display lists. A better approach to changing
object's Appearance directly. ";


// File: classRobotModel.xml
%feature("docstring") RobotModel "

A model of a dynamic and kinematic robot.

Stores both constant information, like the reference placement of the
links, joint limits, velocity limits, etc, as well as a current
configuration and current velocity which are state-dependent. Several
functions depend on the robot's current configuration and/or velocity.
To update that, use the setConfig() and setVelocity() functions.
setConfig() also update's the robot's link transforms via forward
kinematics. You may also use setDOFPosition and setDOFVelocity for
individual changes, but this is more expensive because each call
updates all of the affected the link transforms.

It is important to understand that changing the configuration of the
model doesn't actually send a command to the physical / simulated
robot. Moreover, the model does not automatically get updated when the
physical / simulated robot moves. In essence, the model maintains
temporary storage for performing kinematics, dynamics, and planning
computations, as well as for visualization.

The state of the robot is retrieved using getConfig/getVelocity calls,
and is set using setConfig/setVelocity. Because many routines change
the robot's configuration, like IK and motion planning, a common
design pattern is to save/restore the configuration as follows: q =
robot.getConfig()

do some stuff that may touch the robot's configuration...

robot.setConfig(q)

The model maintains configuration/velocity/acceleration/torque bounds.
However, these are not enforced by the model, so you can happily set
configurations outside must rather be enforced by the planner /
simulator.

C++ includes: robotmodel.h ";

%feature("docstring")  RobotModel::RobotModel "";

%feature("docstring")  RobotModel::getID "

Returns the ID of the robot in its world (Note: not the same as the
robot index) ";

%feature("docstring")  RobotModel::getName "";

%feature("docstring")  RobotModel::setName "";

%feature("docstring")  RobotModel::numLinks "

Returns the number of links = number of DOF's. ";

%feature("docstring")  RobotModel::link "

Returns a reference to the indexed link. ";

%feature("docstring")  RobotModel::link "

Returns a reference to the named link. ";

%feature("docstring")  RobotModel::numDrivers "

Returns the number of drivers. ";

%feature("docstring")  RobotModel::driver "

Returns a reference to the indexed driver. ";

%feature("docstring")  RobotModel::driver "

Returns a reference to the named driver. ";

%feature("docstring")  RobotModel::getConfig "

Retrieves the current configuration of the robot model. ";

%feature("docstring")  RobotModel::getVelocity "

Retreives the current velocity of the robot model. ";

%feature("docstring")  RobotModel::setConfig "

Sets the current configuration of the robot. Input q is a vector of
length numLinks(). This also updates forward kinematics of all links.
Again, it is important to realize that the RobotModel is not the same
as a simulated robot, and this will not change the simulation world.
Many functions such as IK and motion planning use the RobotModel
configuration as a temporary variable, so if you need to keep the
configuration through a robot-modifying function call, you should call
q = robot.getConfig() before the call, and then robot.setConfig(q)
after it. ";

%feature("docstring")  RobotModel::setVelocity "

Sets the current velocity of the robot model. Like the configuration,
this is also essentially a temporary variable. ";

%feature("docstring")  RobotModel::getJointLimits "

Retrieves a pair (qmin,qmax) of min/max joint limit vectors. ";

%feature("docstring")  RobotModel::setJointLimits "

Sets the min/max joint limit vectors (must have length numLinks()) ";

%feature("docstring")  RobotModel::getVelocityLimits "

Retrieve the velocity limit vector vmax, the constraint is |dq[i]| <=
vmax[i]. ";

%feature("docstring")  RobotModel::setVelocityLimits "

Sets the velocity limit vector vmax, the constraint is |dq[i]| <=
vmax[i]. ";

%feature("docstring")  RobotModel::getAccelerationLimits "

Retrieve the acceleration limit vector amax, the constraint is
|ddq[i]| <= amax[i]. ";

%feature("docstring")  RobotModel::setAccelerationLimits "

Sets the acceleration limit vector amax, the constraint is |ddq[i]| <=
amax[i]. ";

%feature("docstring")  RobotModel::getTorqueLimits "

Retrieve the torque limit vector tmax, the constraint is |torque[i]|
<= tmax[i]. ";

%feature("docstring")  RobotModel::setTorqueLimits "

Sets the torque limit vector tmax, the constraint is |torque[i]| <=
tmax[i]. ";

%feature("docstring")  RobotModel::setDOFPosition "

Sets a single DOF's position. Note: if you are setting several joints
at once, use setConfig because this function computes forward
kinematics every time. ";

%feature("docstring")  RobotModel::setDOFPosition "

Sets a single DOF's position (by name). Note: if you are setting
several joints at once, use setConfig because this function computes
forward kinematics every time. ";

%feature("docstring")  RobotModel::getDOFPosition "

Returns a single DOF's position. ";

%feature("docstring")  RobotModel::getDOFPosition "

Returns a single DOF's position (by name) ";

%feature("docstring")  RobotModel::getCom "

Returns the 3D center of mass at the current config. ";

%feature("docstring")  RobotModel::getComJacobian "

Returns the 3xn Jacobian matrix of the current center of mass. ";

%feature("docstring")  RobotModel::getMassMatrix "

Returns the nxn mass matrix B(q) ";

%feature("docstring")  RobotModel::getMassMatrixInv "

Returns the inverse of the nxn mass matrix B(q)^-1 (faster than
inverting result of getMassMatrix) ";

%feature("docstring")  RobotModel::getCoriolisForceMatrix "

Returns the Coriolis force matrix C(q,dq) for current config and
velocity. ";

%feature("docstring")  RobotModel::getCoriolisForces "

Returns the Coriolis forces C(q,dq)*dq for current config and velocity
(faster than computing matrix and doing product). (\"Forces\" is
somewhat of a misnomer; the result is a joint torque vector) ";

%feature("docstring")  RobotModel::getGravityForces "

Returns the generalized gravity vector G(q) for the given workspace
gravity vector g (usually (0,0,-9.8)). (\"Forces\" is somewhat of a
misnomer; the result is a joint torque vector) ";

%feature("docstring")  RobotModel::torquesFromAccel "

Computes the inverse dynamics (using Recursive Newton Euler solver).
Note: does not include gravity term G(q) ";

%feature("docstring")  RobotModel::accelFromTorques "

Computes the foward dynamics (using Recursive Newton Euler solver)
Note: does not include gravity term G(q) ";

%feature("docstring")  RobotModel::interpolate "

Interpolates smoothly between two configurations, properly taking into
account nonstandard joints. ";

%feature("docstring")  RobotModel::distance "

Computes a distance between two configurations, properly taking into
account nonstandard joints. ";

%feature("docstring")  RobotModel::interpolateDeriv "

Returns the configuration derivative at a as you interpolate toward b
at unit speed. ";

%feature("docstring")  RobotModel::randomizeConfig "

Samples a random configuration and updates the robot's pose. Properly
handles non-normal joints and handles DOFs with infinite bounds using
a centered Laplacian distribution with the given scaling term. (Note
that the python random seeding does not affect the result.) ";

%feature("docstring")  RobotModel::selfCollisionEnabled "

Queries whether self collisions between two links is enabled. ";

%feature("docstring")  RobotModel::enableSelfCollision "

Enables/disables self collisions between two links (depending on
value) ";

%feature("docstring")  RobotModel::selfCollides "

Returns true if the robot is in self collision (faster than manual
testing) ";

%feature("docstring")  RobotModel::drawGL "

Draws the robot geometry. If keepAppearance=true, the current
appearance is honored. Otherwise, only the raw geometry is drawn.
PERFORMANCE WARNING: if keepAppearance is false, then this does not
properly reuse OpenGL display lists. A better approach to changing the
robot's appearances is to set the link Appearance's directly. ";


// File: classRobotModelDriver.xml
%feature("docstring") RobotModelDriver "

A reference to a driver of a RobotModel.

A driver corresponds to one of the robot's actuators and its
transmission.

C++ includes: robotmodel.h ";

%feature("docstring")  RobotModelDriver::RobotModelDriver "";

%feature("docstring")  RobotModelDriver::getName "";

%feature("docstring")  RobotModelDriver::robot "

Returns a reference to the driver's robot. ";

%feature("docstring")  RobotModelDriver::getType "

Currently can be \"normal\", \"affine\", \"rotation\",
\"translation\", or \"custom\". ";

%feature("docstring")  RobotModelDriver::getAffectedLink "

Returns the single affected link for \"normal\" links. ";

%feature("docstring")  RobotModelDriver::getAffectedLinks "

Returns the driver's affected links. ";

%feature("docstring")  RobotModelDriver::getAffineCoeffs "

For \"affine\" links, returns the scale and offset of the driver value
mapped to the world. ";

%feature("docstring")  RobotModelDriver::setValue "

Sets the robot's config to correspond to the given driver value. ";

%feature("docstring")  RobotModelDriver::getValue "

Gets the current driver value from the robot's config. ";

%feature("docstring")  RobotModelDriver::setVelocity "

Sets the robot's velocity to correspond to the given driver velocity
value. ";

%feature("docstring")  RobotModelDriver::getVelocity "

Gets the current driver velocity value from the robot's velocity. ";


// File: classRobotModelLink.xml
%feature("docstring") RobotModelLink "

A reference to a link of a RobotModel.

The link stores many mostly-constant items (id, name, parent,
geometry, appearance, mass, joint axes). The exception is the link's
current transform, which is affected by the RobotModel's current
configuration, i.e., the last RobotModel.setConfig(q) call. The
various Jacobians of points on the link, accessed by getJacobianX, are
configuration dependent.

These are not created by hand, but instead accessed using
RobotModel.link([index or name])

C++ includes: robotmodel.h ";

%feature("docstring")  RobotModelLink::RobotModelLink "";

%feature("docstring")  RobotModelLink::getID "

Returns the ID of the robot link in its world (Note: not the same as
getIndex()) ";

%feature("docstring")  RobotModelLink::getName "

Returns the name of the robot link. ";

%feature("docstring")  RobotModelLink::setName "

Sets the name of the robot link. ";

%feature("docstring")  RobotModelLink::robot "

Returns a reference to the link's robot. ";

%feature("docstring")  RobotModelLink::getIndex "

Returns the index of the link (on its robot). ";

%feature("docstring")  RobotModelLink::getParent "

Returns the index of the link's parent (on its robot). ";

%feature("docstring")  RobotModelLink::parent "

Returns a reference to the link's parent, or a NULL link if it has no
parent. ";

%feature("docstring")  RobotModelLink::setParent "

Sets the index of the link's parent (on its robot). ";

%feature("docstring")  RobotModelLink::setParent "

Sets the link's parent (must be on the same robot). ";

%feature("docstring")  RobotModelLink::geometry "

Returns a reference to the link's geometry. ";

%feature("docstring")  RobotModelLink::appearance "

Returns a reference to the link's appearance. ";

%feature("docstring")  RobotModelLink::getMass "

Retrieves the inertial properties of the link. (Note that the Mass is
given with origin at the link frame, not about the COM.) ";

%feature("docstring")  RobotModelLink::setMass "

Sets the inertial proerties of the link. (Note that the Mass is given
with origin at the link frame, not about the COM.) ";

%feature("docstring")  RobotModelLink::getParentTransform "

Gets transformation (R,t) to the parent link. ";

%feature("docstring")  RobotModelLink::setParentTransform "

Sets transformation (R,t) to the parent link. ";

%feature("docstring")  RobotModelLink::getAxis "

Gets the local rotational / translational axis. ";

%feature("docstring")  RobotModelLink::setAxis "

Sets the local rotational / translational axis. ";

%feature("docstring")  RobotModelLink::getWorldPosition "

Converts point from local to world coordinates. ";

%feature("docstring")  RobotModelLink::getWorldDirection "

Converts direction from local to world coordinates. ";

%feature("docstring")  RobotModelLink::getLocalPosition "

Converts point from world to local coordinates. ";

%feature("docstring")  RobotModelLink::getLocalDirection "

Converts direction from world to local coordinates. ";

%feature("docstring")  RobotModelLink::getTransform "

Gets transformation (R,t) to the world frame. ";

%feature("docstring")  RobotModelLink::setTransform "

Sets transformation (R,t) to the world frame. Note: this does NOT
perform inverse kinematics. The transform is overwritten when the
robot's setConfig() method is called. ";

%feature("docstring")  RobotModelLink::getJacobian "

Returns the total jacobian of the local point p (row-major matrix)
(orientation jacobian is stacked on position jacobian) ";

%feature("docstring")  RobotModelLink::getPositionJacobian "

Returns the jacobian of the local point p (row-major matrix) ";

%feature("docstring")  RobotModelLink::getOrientationJacobian "

Returns the orientation jacobian of the link (row-major matrix) ";

%feature("docstring")  RobotModelLink::getVelocity "

Returns the velocity of the origin given the robot's current velocity.
";

%feature("docstring")  RobotModelLink::getAngularVelocity "

Returns the angular velocity given the robot's current velocity. ";

%feature("docstring")  RobotModelLink::getPointVelocity "

Returns the world velocity of the point given the robot's current
velocity. ";

%feature("docstring")  RobotModelLink::drawLocalGL "

Draws the link's geometry in its local frame. If keepAppearance=true,
the current Appearance is honored. Otherwise, just the geometry is
drawn. ";

%feature("docstring")  RobotModelLink::drawWorldGL "

Draws the link's geometry in the world frame. If keepAppearance=true,
the current Appearance is honored. Otherwise, just the geometry is
drawn. ";


// File: classRobotPoser.xml
%feature("docstring") RobotPoser "";

%feature("docstring")  RobotPoser::RobotPoser "";

%feature("docstring")  RobotPoser::setActiveDofs "";

%feature("docstring")  RobotPoser::set "";

%feature("docstring")  RobotPoser::get "";

%feature("docstring")  RobotPoser::getConditioned "";


// File: classSimBody.xml
%feature("docstring") SimBody "

A reference to a rigid body inside a Simulator (either a
RigidObjectModel, TerrainModel, or a link of a RobotModel).

Can use this class to directly apply forces to or control positions /
velocities of objects in the simulation. However, note that the
changes are only applied in the current simulation substep, not the
duration provided to Simulation.simulate(). If you need fine-grained
control, make sure to call simulate() with time steps equal to the
value provided to Simulation.setSimStep() (this is 0.001s by default).

Important: the transform of the object is centered at the object's
center of mass rather than the reference frame given in the
RobotModelLink or RigidObjectModel.

C++ includes: robotsim.h ";

%feature("docstring")  SimBody::getID "

Returns the object ID that this body associated with. ";

%feature("docstring")  SimBody::enable "

Sets the simulation of this body on/off. ";

%feature("docstring")  SimBody::isEnabled "

Returns true if this body is being simulated. ";

%feature("docstring")  SimBody::enableDynamics "

Sets the dynamic simulation of the body on/off. If false, velocities
will simply be integrated forward, and forces will not affect velocity
i.e., it will be pure kinematic simulation. ";

%feature("docstring")  SimBody::isDynamicsEnabled "";

%feature("docstring")  SimBody::applyWrench "

Applies a force and torque about the COM over the duration of the next
Simulator.simulate(t) call. ";

%feature("docstring")  SimBody::applyForceAtPoint "

Applies a force at a given point (in world coordinates) over the
duration of the next Simulator.simulate(t) call. ";

%feature("docstring")  SimBody::applyForceAtLocalPoint "

Applies a force at a given point (in local center-of-mass-centered
coordinates) over the duration of the next Simulator.simulate(t) call.
";

%feature("docstring")  SimBody::setTransform "

Sets the body's transformation at the current simulation time step (in
center-of-mass centered coordinates). ";

%feature("docstring")  SimBody::getTransform "

Gets the body's transformation at the current simulation time step (in
center-of-mass centered coordinates). ";

%feature("docstring")  SimBody::setObjectTransform "

Sets the body's transformation at the current simulation time step (in
object-native coordinates) ";

%feature("docstring")  SimBody::getObjectTransform "

Gets the body's transformation at the current simulation time step (in
object-native coordinates). ";

%feature("docstring")  SimBody::setVelocity "

Sets the angular velocity and translational velocity at the current
simulation time step. ";

%feature("docstring")  SimBody::getVelocity "

Returns the angular velocity and translational velocity. ";

%feature("docstring")  SimBody::setCollisionPadding "

Sets the collision padding (useful for thin objects). Default is
0.0025. ";

%feature("docstring")  SimBody::getCollisionPadding "";

%feature("docstring")  SimBody::setCollisionPreshrink "

If set, preshrinks the geometry so that the padded geometry better
matches the original mesh. If shrinkVisualization=true, the underlying
mesh is also shrunk (helps debug) ";

%feature("docstring")  SimBody::getSurface "

Gets (a copy of) the surface properties. ";

%feature("docstring")  SimBody::setSurface "

Sets the surface properties. ";


// File: structSimData.xml
%feature("docstring") SimData "

Internally used. ";


// File: classSimRobotController.xml
%feature("docstring") SimRobotController "

A controller for a simulated robot.

By default a SimRobotController has three possible modes: Motion queue
+ PID mode: the controller has an internal trajectory queue that may
be added to and modified. This queue supports piecewise linear
interpolation, cubic interpolation, and time-optimal move-to commands.

PID mode: the user controls the motor's PID setpoints directly

Torque control: the user controlls the motor torques directly.

The \"standard\" way of using this is in move-to mode which accepts a
milestone (setMilestone) or list of milestones (repeated calls to
addMilestone) and interpolates dynamically from the current
configuration/velocity. To handle disturbances, a PID loop is run
automatically at the controller's specified rate.

To get finer-grained control over the motion queue's timing, you may
use the setLinear/setCubic/addLinear/addCubic functions. In these
functions it is up to the user to respect velocity, acceleration, and
torque limits.

Whether in motion queue or PID mode, the constants of the PID loop are
initially set in the robot file. You can programmatically tune these
via the setPIDGains function.

Arbitrary trajectories can be tracked by using setVelocity over short
time steps. Force controllers can be implemented using setTorque,
again using short time steps.

If setVelocity, setTorque, or setPID command are called, the motion
queue behavior will be completely overridden. To reset back to motion
queue control, the function setManualMode(False) must be called.

C++ includes: robotsim.h ";

%feature("docstring")  SimRobotController::SimRobotController "";

%feature("docstring")  SimRobotController::~SimRobotController "";

%feature("docstring")  SimRobotController::model "

Retrieves the robot model associated with this controller. ";

%feature("docstring")  SimRobotController::setRate "

Sets the current feedback control rate. ";

%feature("docstring")  SimRobotController::getRate "

Gets the current feedback control rate. ";

%feature("docstring")  SimRobotController::getCommandedConfig "

Returns the current commanded configuration. ";

%feature("docstring")  SimRobotController::getCommandedVelocity "

Returns the current commanded velocity. ";

%feature("docstring")  SimRobotController::getSensedConfig "

Returns the current \"sensed\" configuration from the simulator. ";

%feature("docstring")  SimRobotController::getSensedVelocity "

Returns the current \"sensed\" velocity from the simulator. ";

%feature("docstring")  SimRobotController::sensor "

Returns a sensor by index. If out of bounds, a null sensor is
returned. ";

%feature("docstring")  SimRobotController::sensor "

Returns a sensor by name. If unavailable, a null sensor is returned.
";

%feature("docstring")  SimRobotController::commands "

gets a command list ";

%feature("docstring")  SimRobotController::sendCommand "

sends a command to the controller ";

%feature("docstring")  SimRobotController::getSetting "

gets a setting of the controller ";

%feature("docstring")  SimRobotController::setSetting "

sets a setting of the controller ";

%feature("docstring")  SimRobotController::setMilestone "

Uses a dynamic interpolant to get from the current state to the
desired milestone (with optional ending velocity). This interpolant is
time-optimal with respect to the velocity and acceleration bounds. ";

%feature("docstring")  SimRobotController::setMilestone "

Uses a dynamic interpolant to get from the current state to the
desired milestone (with optional ending velocity). This interpolant is
time-optimal with respect to the velocity and acceleration bounds. ";

%feature("docstring")  SimRobotController::addMilestone "

Same as setMilestone, but appends an interpolant onto an internal
motion queue starting at the current queued end state. ";

%feature("docstring")  SimRobotController::addMilestone "

Same as setMilestone, but appends an interpolant onto an internal
motion queue starting at the current queued end state. ";

%feature("docstring")  SimRobotController::addMilestoneLinear "

Same as addMilestone, but enforces that the motion should move along a
straight-line joint-space path. ";

%feature("docstring")  SimRobotController::setLinear "

Uses linear interpolation to get from the current configuration to the
desired configuration after time dt. ";

%feature("docstring")  SimRobotController::setCubic "

Uses cubic (Hermite) interpolation to get from the current
configuration/velocity to the desired configuration/velocity after
time dt. ";

%feature("docstring")  SimRobotController::addLinear "

Same as setLinear but appends an interpolant onto the motion queue. ";

%feature("docstring")  SimRobotController::addCubic "

Same as setCubic but appends an interpolant onto the motion queue. ";

%feature("docstring")  SimRobotController::remainingTime "

Returns the remaining duration of the motion queue. ";

%feature("docstring")  SimRobotController::setVelocity "

Sets a rate controller from the current commanded config to move at
rate dq for time dt. ";

%feature("docstring")  SimRobotController::setTorque "

Sets a torque command controller. ";

%feature("docstring")  SimRobotController::setPIDCommand "

Sets a PID command controller. ";

%feature("docstring")  SimRobotController::setPIDCommand "

Sets a PID command controller. If tfeedforward is used, it is the
feedforward torque vector. ";

%feature("docstring")  SimRobotController::setManualMode "

Turns on/off manual mode, if either the setTorque or setPID command
were previously set. ";

%feature("docstring")  SimRobotController::getControlType "

Returns the control type for the active controller.

Valid values are: unknown

off

torque

PID

locked_velocity ";

%feature("docstring")  SimRobotController::setPIDGains "

Sets the PID gains. ";

%feature("docstring")  SimRobotController::getPIDGains "

Gets the PID gains for the PID controller. ";


// File: classSimRobotSensor.xml
%feature("docstring") SimRobotSensor "

A sensor on a simulated robot. Retreive this from the controller, and
use getMeasurements to get the currently simulated measurement vector.

Sensors are automatically updated through the sim.simulate call, and
getMeasurements() retrieves the previously updated values. As a
result, you may get garbage measurements before the first sim.simulate
call is made.

There is also a new mode for doing kinematic simulation, which is
supported (i.e., makes sensible measurements) for some types of
sensors when just a robot / world model is given. This is similar to
Simulation.fakeSimulate but the entire controller structure is
bypassed. You can randomly set the robot's position, call
kinematicReset(), and then call kinematicSimulate(). Subsequent calls
assume the robot is being driven along a trajectory until the next
kinematicReset() is called. LaserSensor, CameraSensor, TiltSensor,
AccelerometerSensor, GyroSensor, JointPositionSensor,
JointVelocitySensor support kinematic simulation mode. FilteredSensor
and TimeDelayedSensor also work. The force-related sensors
(ContactSensor and ForceTorqueSensor) return 0's in kinematic
simulation.

To use get/setSetting, you will need to know the sensor attribute
names and types as described in Klampt/Control/*Sensor.h (same as in
the world or sensor XML file).

C++ includes: robotsim.h ";

%feature("docstring")  SimRobotSensor::SimRobotSensor "";

%feature("docstring")  SimRobotSensor::name "

Returns the name of the sensor. ";

%feature("docstring")  SimRobotSensor::type "

Returns the type of the sensor. ";

%feature("docstring")  SimRobotSensor::measurementNames "

Returns a list of names for the measurements (one per measurement). ";

%feature("docstring")  SimRobotSensor::getMeasurements "

Returns a list of measurements from the previous simulation (or
kinematicSimulate) timestep. ";

%feature("docstring")  SimRobotSensor::getSetting "

Returns the value of the named setting (you will need to manually
parse this) ";

%feature("docstring")  SimRobotSensor::setSetting "

Sets the value of the named setting (you will need to manually cast an
int/float/etc to a str) ";

%feature("docstring")  SimRobotSensor::drawGL "

Draws a sensor indicator using OpenGL. ";

%feature("docstring")  SimRobotSensor::drawGL "

Draws a sensor indicator and its measurements using OpenGL. ";

%feature("docstring")  SimRobotSensor::kinematicSimulate "

simulates / advances the kinematic simulation ";

%feature("docstring")  SimRobotSensor::kinematicReset "

resets a kinematic simulation so that a new initial condition can be
set ";


// File: classSimulator.xml
%feature("docstring") Simulator "

A dynamics simulator for a WorldModel.

C++ includes: robotsim.h ";

%feature("docstring")  Simulator::Simulator "

Constructs the simulator from a WorldModel. If the WorldModel was
loaded from an XML file, then the simulation setup is loaded from it.
";

%feature("docstring")  Simulator::~Simulator "";

%feature("docstring")  Simulator::reset "

Resets to the initial state (same as setState(initialState)) ";

%feature("docstring")  Simulator::getStatus "

Returns an indicator code for the simulator status. The return result
is one of the STATUS_X flags. (Technically, this returns the worst
status over the last simulate() call) ";

%feature("docstring")  Simulator::getStatusString "

Returns a string indicating the simulator's status. If s is provided
and >= 0, this function maps the indicator code s to a string. ";

%feature("docstring")  Simulator::getState "

Returns a Base64 string representing the binary data for the current
simulation state, including controller parameters, etc. ";

%feature("docstring")  Simulator::setState "

Sets the current simulation state from a Base64 string returned by a
prior getState call. ";

%feature("docstring")  Simulator::simulate "

Advances the simulation by time t, and updates the world model from
the simulation state. ";

%feature("docstring")  Simulator::fakeSimulate "

Advances a faked simulation by time t, and updates the world model
from the faked simulation state. ";

%feature("docstring")  Simulator::getTime "

Returns the simulation time. ";

%feature("docstring")  Simulator::updateWorld "

Updates the world model from the current simulation state. This only
needs to be called if you change the world model and want to revert
back to the simulation state. ";

%feature("docstring")  Simulator::getActualConfig "

Returns the current actual configuration of the robot from the
simulator. ";

%feature("docstring")  Simulator::getActualVelocity "

Returns the current actual velocity of the robot from the simulator.
";

%feature("docstring")  Simulator::getActualTorques "

Returns the current actual torques on the robot's drivers from the
simulator. ";

%feature("docstring")  Simulator::enableContactFeedback "

Call this to enable contact feedback between the two objects
(arguments are indexes returned by object.getID()). Contact feedback
has a small overhead so you may want to do this selectively. This must
be called before using inContact, getContacts, getContactForces,
contactForce, contactTorque, hadContact, hadSeparation,
hadPenetration, and meanContactForce. ";

%feature("docstring")  Simulator::enableContactFeedbackAll "

Call this to enable contact feedback between all pairs of objects.
Contact feedback has a small overhead so you may want to do this
selectively. ";

%feature("docstring")  Simulator::inContact "

Returns true if the objects (indexes returned by object.getID()) are
in contact on the current time step. You can set bid=-1 to tell if
object a is in contact with any object. ";

%feature("docstring")  Simulator::getContacts "

Returns the list of contacts (x,n,kFriction) at the last time step.
Normals point into object a. The contact point (x,n,kFriction) is
represented as a 7-element vector. ";

%feature("docstring")  Simulator::getContactForces "

Returns the list of contact forces on object a at the last time step.
";

%feature("docstring")  Simulator::contactForce "

Returns the contact force on object a at the last time step. You can
set bid to -1 to get the overall contact force on object a. ";

%feature("docstring")  Simulator::contactTorque "

Returns the contact force on object a (about a's origin) at the last
time step. You can set bid to -1 to get the overall contact force on
object a. ";

%feature("docstring")  Simulator::hadContact "

Returns true if the objects had contact over the last simulate() call.
You can set bid to -1 to determine if object a had contact with any
other object. ";

%feature("docstring")  Simulator::hadSeparation "

Returns true if the objects had ever separated during the last
simulate() call. You can set bid to -1 to determine if object a had no
contact with any other object. ";

%feature("docstring")  Simulator::hadPenetration "

Returns true if the objects interpenetrated during the last simulate()
call. If so, the simulation may lead to very inaccurate results or
artifacts. You can set bid to -1 to determine if object a penetrated
any object, or you can set aid=bid=-1 to determine whether any object
is penetrating any other (indicating that the simulation will not be
functioning properly in general). ";

%feature("docstring")  Simulator::meanContactForce "

Returns the average contact force on object a over the last simulate()
call. ";

%feature("docstring")  Simulator::controller "

Returns a controller for the indicated robot. ";

%feature("docstring")  Simulator::controller "";

%feature("docstring")  Simulator::body "

Returns the SimBody corresponding to the given link. ";

%feature("docstring")  Simulator::body "

Returns the SimBody corresponding to the given object. ";

%feature("docstring")  Simulator::body "

Returns the SimBody corresponding to the given terrain. ";

%feature("docstring")  Simulator::getJointForces "

Returns the joint force and torque local to the link, as would be read
by a force-torque sensor mounted at the given link's origin. The 6
entries are (fx,fy,fz,mx,my,mz) ";

%feature("docstring")  Simulator::setGravity "

Sets the overall gravity vector. ";

%feature("docstring")  Simulator::setSimStep "

Sets the internal simulation substep. Values < 0.01 are recommended.
";

%feature("docstring")  Simulator::getSetting "

Retrieves some simulation setting. Valid names are gravity, simStep,
boundaryLayerCollisions, rigidObjectCollisions, robotSelfCollisions,
robotRobotCollisions, adaptiveTimeStepping, minimumAdaptiveTimeStep,
maxContacts, clusterNormalScale, errorReductionParameter,
dampedLeastSquaresParameter, instabilityConstantEnergyThreshold,
instabilityLinearEnergyThreshold, instabilityMaxEnergyThreshold, and
instabilityPostCorrectionEnergy. See Klampt/Simulation/ODESimulator.h
for detailed descriptions of these parameters.

Retreives some simulation setting. Valid names are gravity, simStep,
boundaryLayerCollisions, rigidObjectCollisions, robotSelfCollisions,
robotRobotCollisions, adaptiveTimeStepping, maxContacts,
clusterNormalScale, errorReductionParameter, and
dampedLeastSquaresParameter. ";

%feature("docstring")  Simulator::setSetting "

Sets some simulation setting. Raises an exception if the name is
unknown or the value is of improper format. ";


// File: classTerrainModel.xml
%feature("docstring") TerrainModel "

Static environment geometry.

C++ includes: robotmodel.h ";

%feature("docstring")  TerrainModel::TerrainModel "";

%feature("docstring")  TerrainModel::getID "";

%feature("docstring")  TerrainModel::getName "";

%feature("docstring")  TerrainModel::setName "";

%feature("docstring")  TerrainModel::geometry "";

%feature("docstring")  TerrainModel::appearance "";

%feature("docstring")  TerrainModel::setFriction "";

%feature("docstring")  TerrainModel::drawGL "

Draws the object's geometry. If keepAppearance=true, the current
appearance is honored. Otherwise, only the raw geometry is drawn.
PERFORMANCE WARNING: if keepAppearance is false, then this does not
properly reuse OpenGL display lists. A better approach to changing
object's Appearance directly. ";


// File: classTransformPoser.xml
%feature("docstring") TransformPoser "";

%feature("docstring")  TransformPoser::TransformPoser "";

%feature("docstring")  TransformPoser::set "";

%feature("docstring")  TransformPoser::get "";

%feature("docstring")  TransformPoser::enableTranslation "";

%feature("docstring")  TransformPoser::enableRotation "";


// File: structTriangleMesh.xml
%feature("docstring") TriangleMesh "

A 3D indexed triangle mesh class.

Attributes: vertices: a list of vertices, given as a flattened
coordinate list [x1, y1, z1, x2, y2, ...]

indices: a list of triangle vertices given as indices into the
vertices list, i.e., [a1,b1,c2, a2,b2,c2, ...]

Note: because the bindings are generated by SWIG, you can access the
indices / vertices members via some automatically generated accessors
/ modifiers. In particular len(), append(), and indexing via [] are
useful. Some other methods like resize() are also provided. However,
you CANNOT set these items via assignment.

Examples:

m = TriangleMesh() m.vertices.append(0) m.vertices.append(0)
m.vertices.append(0) print len(m.vertices) #prints 3 m.vertices =
[0,0,0] #this is an error m.vertices += [1,2,3] #this is also an error

C++ includes: geometry.h ";

%feature("docstring")  TriangleMesh::translate "

Translates all the vertices by v=v+t. ";

%feature("docstring")  TriangleMesh::transform "

Transforms all the vertices by the rigid transform v=R*v+t. ";


// File: classViewport.xml
%feature("docstring") Viewport "";

%feature("docstring")  Viewport::fromJson "";

%feature("docstring")  Viewport::toJson "";

%feature("docstring")  Viewport::setModelviewMatrix "";

%feature("docstring")  Viewport::setRigidTransform "";

%feature("docstring")  Viewport::getRigidTransform "";


// File: classWidget.xml
%feature("docstring") Widget "";

%feature("docstring")  Widget::Widget "";

%feature("docstring")  Widget::~Widget "";

%feature("docstring")  Widget::hover "";

%feature("docstring")  Widget::beginDrag "";

%feature("docstring")  Widget::drag "";

%feature("docstring")  Widget::endDrag "";

%feature("docstring")  Widget::keypress "";

%feature("docstring")  Widget::drawGL "";

%feature("docstring")  Widget::idle "";

%feature("docstring")  Widget::wantsRedraw "";

%feature("docstring")  Widget::hasHighlight "";

%feature("docstring")  Widget::hasFocus "";


// File: structWidgetData.xml
%feature("docstring") WidgetData "

Internally used. ";


// File: classWidgetSet.xml
%feature("docstring") WidgetSet "";

%feature("docstring")  WidgetSet::WidgetSet "";

%feature("docstring")  WidgetSet::add "";

%feature("docstring")  WidgetSet::remove "";

%feature("docstring")  WidgetSet::enable "";


// File: structWorldData.xml
%feature("docstring") WorldData "

Internally used. ";


// File: classWorldModel.xml
%feature("docstring") WorldModel "

The main world class, containing robots, rigid objects, and static
environment geometry.

Note that this is just a model and can be changed at will in fact
planners and simulators will make use of a model to \"display\"
computed

Every robot/robot link/terrain/rigid object is given a unique ID in
the world. This is potentially a source of confusion because some
functions take IDs and some take indices. Only the WorldModel and
Simulator classes use IDs when the argument has 'id' as a suffix,
e.g., geometry(), appearance(), Simulator.inContact(). All other
functions use indices, e.g. robot(0), terrain(0), etc.

To get an object's ID, you can see the value returned by loadElement
and/or object.getID(). states.

To save/restore the state of the model, you must manually maintain
copies of the states of whichever objects you wish to save/restore.

C++ includes: robotmodel.h ";

%feature("docstring")  WorldModel::WorldModel "

Creates a WorldModel. With no arguments, creates a new world. With
another WorldModel instance, creates a reference to an existing world.
(To create a copy, use the copy() method.)

If passed a pointer to a C++ RobotWorld structure, a reference to that
structure is returned. (This is used pretty much only when interfacing
C++ and Python code) ";

%feature("docstring")  WorldModel::WorldModel "";

%feature("docstring")  WorldModel::WorldModel "";

%feature("docstring")  WorldModel::~WorldModel "";

%feature("docstring")  WorldModel::copy "

Creates a copy of the world model. Note that geometries and
appearances are shared... ";

%feature("docstring")  WorldModel::readFile "

Reads from a world XML file. ";

%feature("docstring")  WorldModel::loadFile "

Alias of readFile. ";

%feature("docstring")  WorldModel::saveFile "

Saves to a world XML file. If elementDir is provided, then robots,
terrains, etc. will be saved there. Otherwise they will be saved to a
folder with the same base name as fn (without the trailing .xml) ";

%feature("docstring")  WorldModel::numRobots "";

%feature("docstring")  WorldModel::numRobotLinks "";

%feature("docstring")  WorldModel::numRigidObjects "";

%feature("docstring")  WorldModel::numTerrains "";

%feature("docstring")  WorldModel::numIDs "";

%feature("docstring")  WorldModel::robot "";

%feature("docstring")  WorldModel::robot "";

%feature("docstring")  WorldModel::robotLink "";

%feature("docstring")  WorldModel::robotLink "";

%feature("docstring")  WorldModel::rigidObject "";

%feature("docstring")  WorldModel::rigidObject "";

%feature("docstring")  WorldModel::terrain "";

%feature("docstring")  WorldModel::terrain "";

%feature("docstring")  WorldModel::makeRobot "

Creates a new empty robot. (Not terribly useful now since you can't
resize the number of links yet) ";

%feature("docstring")  WorldModel::makeRigidObject "

Creates a new empty rigid object. ";

%feature("docstring")  WorldModel::makeTerrain "

Creates a new empty terrain. ";

%feature("docstring")  WorldModel::loadRobot "

Loads a robot from a .rob or .urdf file. An empty robot is returned if
loading fails. ";

%feature("docstring")  WorldModel::loadRigidObject "

Loads a rigid object from a .obj or a mesh file. An empty rigid object
is returned if loading fails. ";

%feature("docstring")  WorldModel::loadTerrain "

Loads a rigid object from a mesh file. An empty terrain is returned if
loading fails. ";

%feature("docstring")  WorldModel::loadElement "

Loads some element from a file, automatically detecting its type.
Meshes are interpreted as terrains. The ID is returned, or -1 if
loading failed. ";

%feature("docstring")  WorldModel::add "

Adds a copy of the given robot to this world, either from this
WorldModel or another. ";

%feature("docstring")  WorldModel::add "

Adds a copy of the given rigid object to this world, either from this
WorldModel or another. ";

%feature("docstring")  WorldModel::add "

Adds a copy of the given terrain to this world, either from this
WorldModel or another. ";

%feature("docstring")  WorldModel::remove "

Removes a robot. It must be in this world or an exception is raised.
IMPORTANT: all other references to robots will be invalidated. ";

%feature("docstring")  WorldModel::remove "

Removes a rigid object. It must be in this world or an exception is
raised. IMPORTANT: all other references to rigid objects will be
invalidated. ";

%feature("docstring")  WorldModel::remove "

Removes a terrain. It must be in this world or an exception is raised.
IMPORTANT: all other references to terrains will be invalidated. ";

%feature("docstring")  WorldModel::getName "

Retrieves a name for a given element ID. ";

%feature("docstring")  WorldModel::geometry "

Retrieves a geometry for a given element ID. ";

%feature("docstring")  WorldModel::appearance "

Retrieves an appearance for a given element ID. ";

%feature("docstring")  WorldModel::drawGL "

Draws the entire world using OpenGL. ";

%feature("docstring")  WorldModel::enableGeometryLoading "

If geometry loading is set to false, then only the kinematics are
loaded from disk, and no geometry / visualization / collision
detection structures will be loaded. Useful for quick scripts that
just use kinematics / dynamics of a robot. ";

%feature("docstring")  WorldModel::enableInitCollisions "

If collision detection is set to true, then collision acceleration
data structures will be automatically initialized, with debugging
information. Useful for scripts that do planning and for which
collision initialization may take a long time. Note that even when
this flag is off, the collision acceleration data structures will
indeed be initialized whenever geometry collision, distance, or ray-
casting routines are called. ";


// File: namespacestd.xml


// File: geometry_8h.xml


// File: motionplanning_8cpp.xml
%feature("docstring")  std::setRandomSeed "

Sets the random seed used by the configuration sampler.

Sets the random seed used by the motion planner. ";

%feature("docstring")  std::UpdateStats "";

%feature("docstring")  std::makeNewCSpace "";

%feature("docstring")  std::destroyCSpace "";

%feature("docstring")  std::setPlanJSONString "

Loads planner values from a JSON string. ";

%feature("docstring")  std::getPlanJSONString "

Saves planner values to a JSON string. ";

%feature("docstring")  std::setPlanType "

Sets the planner type.

Valid values are prm: the Probabilistic Roadmap algorithm

rrt: the Rapidly Exploring Random Trees algorithm

sbl: the Single-Query Bidirectional Lazy planner

sblprt: the probabilistic roadmap of trees (PRT) algorithm with SBL as
the inter-root planner.

rrt*: the RRT* algorithm for optimal motion planning

prm*: the PRM* algorithm for optimal motion planning

lazyprm*: the Lazy-PRM* algorithm for optimal motion planning

lazyrrg*: the Lazy-RRG* algorithm for optimal motion planning

fmm: the fast marching method algorithm for resolution-complete
optimal motion planning

fmm*: an anytime fast marching method algorithm for optimal motion
planning ";

%feature("docstring")  std::setPlanSetting "";

%feature("docstring")  std::setPlanSetting "

Sets a numeric or string-valued setting for the planner.

Valid numeric values are: \"knn\": k value for the k-nearest neighbor
connection strategy (only for PRM)

\"connectionThreshold\": a milestone connection threshold

\"perturbationRadius\": (for RRT and SBL)

\"bidirectional\": 1 if bidirectional planning is requested (for RRT)

\"grid\": 1 if a point selection grid should be used (for SBL)

\"gridResolution\": resolution for the grid, if the grid should be
used (for SBL with grid, FMM, FMM*)

\"suboptimalityFactor\": allowable suboptimality (for RRT*, lazy PRM*,
lazy RRG*)

\"randomizeFrequency\": a grid randomization frequency (for SBL)

\"shortcut\": nonzero if you wish to perform shortcutting after a
first plan is found.

\"restart\": nonzero if you wish to restart the planner to get better
paths with the remaining time.

Valid string values are: \"pointLocation\": a string designating a
point location data structure. \"kdtree\" is supported, optionally
followed by a weight vector (for PRM, RRT*, PRM*, LazyPRM*, LazyRRG*)

\"restartTermCond\": used if the \"restart\" setting is true. This is
a JSON string defining the termination condition (default value:
\"{foundSolution:1;maxIters:1000}\") ";

%feature("docstring")  std::makeNewPlan "";

%feature("docstring")  std::destroyPlan "";

%feature("docstring")  std::DumpPlan "";

%feature("docstring")  std::destroy "

Cleans up all internal data structures. Useful for multithreaded
programs to make sure ODE errors aren't thrown on exit. This is called
for you on exit when importing the Python klampt module.

Performs cleanup of all created spaces and planners.

destroys internal data structures ";


// File: motionplanning_8h.xml
%feature("docstring")  setRandomSeed "

Sets the random seed used by the motion planner.

Sets the random seed used by the motion planner. ";

%feature("docstring")  setPlanJSONString "

Loads planner values from a JSON string. ";

%feature("docstring")  getPlanJSONString "

Saves planner values to a JSON string. ";

%feature("docstring")  setPlanType "

Sets the planner type.

Valid values are prm: the Probabilistic Roadmap algorithm

rrt: the Rapidly Exploring Random Trees algorithm

sbl: the Single-Query Bidirectional Lazy planner

sblprt: the probabilistic roadmap of trees (PRT) algorithm with SBL as
the inter-root planner.

rrt*: the RRT* algorithm for optimal motion planning

prm*: the PRM* algorithm for optimal motion planning

lazyprm*: the Lazy-PRM* algorithm for optimal motion planning

lazyrrg*: the Lazy-RRG* algorithm for optimal motion planning

fmm: the fast marching method algorithm for resolution-complete
optimal motion planning

fmm*: an anytime fast marching method algorithm for optimal motion
planning ";

%feature("docstring")  setPlanSetting "";

%feature("docstring")  setPlanSetting "

Sets a numeric or string-valued setting for the planner.

Valid numeric values are: \"knn\": k value for the k-nearest neighbor
connection strategy (only for PRM)

\"connectionThreshold\": a milestone connection threshold

\"perturbationRadius\": (for RRT and SBL)

\"bidirectional\": 1 if bidirectional planning is requested (for RRT)

\"grid\": 1 if a point selection grid should be used (for SBL)

\"gridResolution\": resolution for the grid, if the grid should be
used (for SBL with grid, FMM, FMM*)

\"suboptimalityFactor\": allowable suboptimality (for RRT*, lazy PRM*,
lazy RRG*)

\"randomizeFrequency\": a grid randomization frequency (for SBL)

\"shortcut\": nonzero if you wish to perform shortcutting after a
first plan is found.

\"restart\": nonzero if you wish to restart the planner to get better
paths with the remaining time.

Valid string values are: \"pointLocation\": a string designating a
point location data structure. \"kdtree\" is supported, optionally
followed by a weight vector (for PRM, RRT*, PRM*, LazyPRM*, LazyRRG*)

\"restartTermCond\": used if the \"restart\" setting is true. This is
a JSON string defining the termination condition (default value:
\"{foundSolution:1;maxIters:1000}\") ";

%feature("docstring")  destroy "

Performs cleanup of all created spaces and planners.

Performs cleanup of all created spaces and planners.

destroys internal data structures ";


// File: robotik_8cpp.xml
%feature("docstring")  copy "";

%feature("docstring")  PySequence_ToVector3 "";

%feature("docstring")  PySequence_ToVector3Array "";

%feature("docstring")  SampleTransform "";

%feature("docstring")  SampleTransform "

Returns a transformation (R,t) from link to link2 sampled at random
from the space of transforms that satisfies the objective. ";

%feature("docstring")  SampleTransform "";


// File: robotik_8h.xml
%feature("docstring")  SampleTransform "

Returns a transformation (R,t) from link to link2 sampled at random
from the space of transforms that satisfies the objective. ";

%feature("docstring")  SampleTransform "";


// File: robotmodel_8h.xml


// File: robotsim_8cpp.xml
%feature("docstring")  createWorld "";

%feature("docstring")  derefWorld "";

%feature("docstring")  refWorld "";

%feature("docstring")  createSim "";

%feature("docstring")  destroySim "";

%feature("docstring")  createWidget "";

%feature("docstring")  derefWidget "";

%feature("docstring")  refWidget "";

%feature("docstring")  destroy "

Cleans up all internal data structures. Useful for multithreaded
programs to make sure ODE errors aren't thrown on exit. This is called
for you on exit when importing the Python klampt module.

Performs cleanup of all created spaces and planners.

destroys internal data structures ";

%feature("docstring")  setRandomSeed "

Sets the random seed used by the configuration sampler.

Sets the random seed used by the motion planner. ";

%feature("docstring")  GetManagedGeometry "";

%feature("docstring")  MakeController "";

%feature("docstring")  GetPathController "";

%feature("docstring")  GetMotionQueue "";

%feature("docstring")  GetMesh "";

%feature("docstring")  GetMesh "";

%feature("docstring")  GetPointCloud "";

%feature("docstring")  GetPointCloud "";

%feature("docstring")  copy "";

%feature("docstring")  copy "";

%feature("docstring")  copy "";

%feature("docstring")  copy "";

%feature("docstring")  EnablePathControl "";

%feature("docstring")  GetCameraViewport "";

%feature("docstring")  setFrictionConeApproximationEdges "";

%feature("docstring")  Convert "";

%feature("docstring")  Convert "";

%feature("docstring")  Convert "";

%feature("docstring")  Convert "";

%feature("docstring")  forceClosure "";

%feature("docstring")  forceClosure "";

%feature("docstring")  forceClosure2D "";

%feature("docstring")  forceClosure2D "";

%feature("docstring")  ToPy2 "";

%feature("docstring")  ToPy2 "";

%feature("docstring")  comEquilibrium "";

%feature("docstring")  comEquilibrium "";

%feature("docstring")  comEquilibrium2D "";

%feature("docstring")  comEquilibrium2D "";

%feature("docstring")  supportPolygon "";

%feature("docstring")  supportPolygon "

A fancy version of the normal supportPolygon test. contactPositions is
a list of 3-lists giving the contact point positions. The i'th element
in the list frictionCones has length (k*4), and gives the contact
force constraints (ax,ay,az,b) where ax*fx+ay*fy+az*fz <= b limits the
contact force (fx,fy,fz) at the i'th contact. Each of the k 4-tuples
is laid out sequentially per-contact.

The return value is a list of 3-tuples giving the sorted plane
boundaries of the polygon. The format of a plane is (nx,ny,ofs) where
(nx,ny) are the outward facing normals, and ofs is the offset from 0.
In other words to test stability of a com [x,y], you can test whether
dot([nx,ny],[x,y]) <= ofs for all planes. ";

%feature("docstring")  supportPolygon2D "

Calculates the support polygon (interval) for a given set of contacts
and a downward external force (0,-g). A contact point is given by a
list of 4 floats, [x,y,theta,k] as usual.

The return value is a 2-tuple giving the min / max extents of the
support polygon. If they are both infinite, the support polygon is
empty. ";

%feature("docstring")  supportPolygon2D "";

%feature("docstring")  equilibriumTorques "";

%feature("docstring")  equilibriumTorques "";

%feature("docstring")  SubscribeToStream "";

%feature("docstring")  DetachFromStream "";

%feature("docstring")  ProcessStreams "";

%feature("docstring")  ThreeJSGetScene "

Exports the WorldModel to a JSON string ready for use in Three.js. ";

%feature("docstring")  ThreeJSGetTransforms "

Exports the WorldModel to a JSON string ready for use in Three.js. ";


// File: robotsim_8h.xml
%feature("docstring")  setRandomSeed "

Sets the random seed used by the configuration sampler. ";

%feature("docstring")  destroy "

Cleans up all internal data structures. Useful for multithreaded
programs to make sure ODE errors aren't thrown on exit. This is called
for you on exit when importing the Python klampt module. ";


// File: rootfind_8cpp.xml
%feature("docstring")  setFTolerance "

Sets the termination threshold for the change in f. ";

%feature("docstring")  setXTolerance "

Sets the termination threshold for the change in x. ";

%feature("docstring")  setVectorField "

Sets the vector field object, returns 0 if pVFObj = NULL, 1 otherwise.
See vectorfield.py for an abstract base class that can be overridden
to produce one of these objects. ";

%feature("docstring")  setFunction "

Sets the function object, returns 0 if pVFObj = NULL, 1 otherwise. See
vectorfield.py for an abstract base class that can be overridden to
produce one of these objects. Equivalent to setVectorField; just a
more intuitive name. ";

%feature("docstring")  findRoots "

Performs unconstrained root finding for up to iter iterations Return
values is a tuple indicating: (0,x,n) : convergence reached in x

(1,x,n) : convergence reached in f

(2,x,n) : divergence

(3,x,n) : degeneration of gradient (local extremum or saddle point)

(4,x,n) : maximum iterations reached

(5,x,n) : numerical error occurred where x is the final point and n is
the number of iterations used ";

%feature("docstring")  findRootsBounded "

Same as findRoots, but with given bounds (xmin,xmax) ";

%feature("docstring")  destroy "

Cleans up all internal data structures. Useful for multithreaded
programs to make sure ODE errors aren't thrown on exit. This is called
for you on exit when importing the Python klampt module.

Performs cleanup of all created spaces and planners.

destroys internal data structures ";


// File: rootfind_8h.xml
%feature("docstring")  setFTolerance "

Sets the termination threshold for the change in f. ";

%feature("docstring")  setXTolerance "

Sets the termination threshold for the change in x. ";

%feature("docstring")  setVectorField "

Sets the vector field object, returns 0 if pVFObj = NULL, 1 otherwise.
See vectorfield.py for an abstract base class that can be overridden
to produce one of these objects. ";

%feature("docstring")  setFunction "

Sets the function object, returns 0 if pVFObj = NULL, 1 otherwise. See
vectorfield.py for an abstract base class that can be overridden to
produce one of these objects. Equivalent to setVectorField; just a
more intuitive name. ";

%feature("docstring")  findRoots "

Performs unconstrained root finding for up to iter iterations Return
values is a tuple indicating: (0,x,n) : convergence reached in x

(1,x,n) : convergence reached in f

(2,x,n) : divergence

(3,x,n) : degeneration of gradient (local extremum or saddle point)

(4,x,n) : maximum iterations reached

(5,x,n) : numerical error occurred where x is the final point and n is
the number of iterations used ";

%feature("docstring")  findRootsBounded "

Same as findRoots, but with given bounds (xmin,xmax) ";

%feature("docstring")  destroy "

destroys internal data structures

destroys internal data structures ";


// File: widget_8h.xml

