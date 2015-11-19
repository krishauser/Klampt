
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
(string), \"geodesic\" (boolean). These may be used by planners to
make planning faster or more accurate. For a more complete list see
KrisLibrary/planning/CSpace.h.

C++ includes: motionplanning.h ";

%feature("docstring")  CSpaceInterface::CSpaceInterface "CSpaceInterface::CSpaceInterface() ";

%feature("docstring")  CSpaceInterface::CSpaceInterface "CSpaceInterface::CSpaceInterface(const CSpaceInterface &) ";

%feature("docstring")  CSpaceInterface::~CSpaceInterface "CSpaceInterface::~CSpaceInterface() ";

%feature("docstring")  CSpaceInterface::destroy "void
CSpaceInterface::destroy() ";

%feature("docstring")  CSpaceInterface::setFeasibility "void
CSpaceInterface::setFeasibility(PyObject *pyFeas) ";

%feature("docstring")  CSpaceInterface::addFeasibilityTest "void
CSpaceInterface::addFeasibilityTest(const char *name, PyObject
*pyFeas) ";

%feature("docstring")  CSpaceInterface::setVisibility "void
CSpaceInterface::setVisibility(PyObject *pyVisible) ";

%feature("docstring")  CSpaceInterface::addVisibilityTest "void
CSpaceInterface::addVisibilityTest(const char *name, PyObject
*pyVisible) ";

%feature("docstring")  CSpaceInterface::setVisibilityEpsilon "void
CSpaceInterface::setVisibilityEpsilon(double eps) ";

%feature("docstring")  CSpaceInterface::setSampler "void
CSpaceInterface::setSampler(PyObject *pySamp) ";

%feature("docstring")  CSpaceInterface::setNeighborhoodSampler "void
CSpaceInterface::setNeighborhoodSampler(PyObject *pySamp) ";

%feature("docstring")  CSpaceInterface::setDistance "void
CSpaceInterface::setDistance(PyObject *pyDist) ";

%feature("docstring")  CSpaceInterface::setInterpolate "void
CSpaceInterface::setInterpolate(PyObject *pyInterp) ";

%feature("docstring")  CSpaceInterface::setProperty "void
CSpaceInterface::setProperty(const char *key, const char *value) ";


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
"GeneralizedIKObjective::GeneralizedIKObjective(const
GeneralizedIKObjective &obj) ";

%feature("docstring")  GeneralizedIKObjective::GeneralizedIKObjective
"GeneralizedIKObjective::GeneralizedIKObjective(const RobotModelLink
&link) ";

%feature("docstring")  GeneralizedIKObjective::GeneralizedIKObjective
"GeneralizedIKObjective::GeneralizedIKObjective(const
RigidObjectModel &obj) ";

%feature("docstring")  GeneralizedIKObjective::GeneralizedIKObjective
"GeneralizedIKObjective::GeneralizedIKObjective(const RobotModelLink
&link, const RobotModelLink &link2) ";

%feature("docstring")  GeneralizedIKObjective::GeneralizedIKObjective
"GeneralizedIKObjective::GeneralizedIKObjective(const RobotModelLink
&link, const RigidObjectModel &obj2) ";

%feature("docstring")  GeneralizedIKObjective::GeneralizedIKObjective
"GeneralizedIKObjective::GeneralizedIKObjective(const
RigidObjectModel &obj, const RobotModelLink &link2) ";

%feature("docstring")  GeneralizedIKObjective::GeneralizedIKObjective
"GeneralizedIKObjective::GeneralizedIKObjective(const
RigidObjectModel &obj, const RigidObjectModel &obj2) ";

%feature("docstring")  GeneralizedIKObjective::setPoint "void
GeneralizedIKObjective::setPoint(const double p1[3], const double
p2[3]) ";

%feature("docstring")  GeneralizedIKObjective::setPoints "void
GeneralizedIKObjective::setPoints(PyObject *p1s, PyObject *p2s) ";

%feature("docstring")  GeneralizedIKObjective::setTransform "void
GeneralizedIKObjective::setTransform(const double R[9], const double
t[3]) ";


// File: classGeneralizedIKSolver.xml
%feature("docstring") GeneralizedIKSolver "

An inverse kinematics solver between multiple robots and/or objects.

C++ includes: robotik.h ";

%feature("docstring")  GeneralizedIKSolver::GeneralizedIKSolver "GeneralizedIKSolver::GeneralizedIKSolver(const WorldModel &world) ";

%feature("docstring")  GeneralizedIKSolver::add "void
GeneralizedIKSolver::add(const GeneralizedIKObjective &objective)

Adds a new simultaneous objective. ";

%feature("docstring")  GeneralizedIKSolver::getResidual "void
GeneralizedIKSolver::getResidual(std::vector< double > &out)

Returns a vector describing the error of the objective. ";

%feature("docstring")  GeneralizedIKSolver::getJacobian "void
GeneralizedIKSolver::getJacobian(std::vector< std::vector< double > >
&out)

Returns a matrix describing the instantaneous derivative of the
objective with respect to the active parameters ";

%feature("docstring")  GeneralizedIKSolver::solve "PyObject *
GeneralizedIKSolver::solve(int iters, double tol=1e-3)

Tries to find a configuration that satifies all simultaneous
objectives up to the desired tolerance. Returns (res,iters) where res
indicates whether x converged. ";

%feature("docstring")  GeneralizedIKSolver::sampleInitial "void
GeneralizedIKSolver::sampleInitial()

Samples an initial random configuration. ";


// File: structGeometricPrimitive.xml
%feature("docstring") GeometricPrimitive "

A geometric primitive. So far only points, spheres, segments, and
AABBs can be constructed manually in the Python API.

C++ includes: geometry.h ";

%feature("docstring")  GeometricPrimitive::setPoint "void
GeometricPrimitive::setPoint(const double pt[3]) ";

%feature("docstring")  GeometricPrimitive::setSphere "void
GeometricPrimitive::setSphere(const double c[3], double r) ";

%feature("docstring")  GeometricPrimitive::setSegment "void
GeometricPrimitive::setSegment(const double a[3], const double b[3])
";

%feature("docstring")  GeometricPrimitive::setAABB "void
GeometricPrimitive::setAABB(const double bmin[3], const double
bmax[3]) ";

%feature("docstring")  GeometricPrimitive::loadString "bool
GeometricPrimitive::loadString(const char *str) ";

%feature("docstring")  GeometricPrimitive::saveString "std::string
GeometricPrimitive::saveString() const ";


// File: classGeometry3D.xml
%feature("docstring") Geometry3D "

A three-D geometry. Can either be a reference to a world item's
geometry, in which case modifiers change the world item's geometry, or
it can be a standalone geometry.

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

%feature("docstring")  Geometry3D::Geometry3D "Geometry3D::Geometry3D() ";

%feature("docstring")  Geometry3D::~Geometry3D "Geometry3D::~Geometry3D() ";

%feature("docstring")  Geometry3D::clone "Geometry3D
Geometry3D::clone()

Creates a standalone geometry from this geometry. ";

%feature("docstring")  Geometry3D::set "void Geometry3D::set(const
Geometry3D &)

Copies the geometry of the argument into this geometry. ";

%feature("docstring")  Geometry3D::isStandalone "bool
Geometry3D::isStandalone()

Returns true if this is a standalone geometry. ";

%feature("docstring")  Geometry3D::free "void Geometry3D::free()

Frees the data associated with this geometry, if standalone. ";

%feature("docstring")  Geometry3D::type "string Geometry3D::type()

Returns the type of geometry: TriangleMesh, PointCloud, or
GeometricPrimitive ";

%feature("docstring")  Geometry3D::empty "bool Geometry3D::empty()

Returns true if this has no contents. ";

%feature("docstring")  Geometry3D::getTriangleMesh "TriangleMesh
Geometry3D::getTriangleMesh()

Returns a TriangleMesh if this geometry is of type TriangleMesh. ";

%feature("docstring")  Geometry3D::getPointCloud "PointCloud
Geometry3D::getPointCloud()

Returns a PointCloud if this geometry is of type PointCloud. ";

%feature("docstring")  Geometry3D::getGeometricPrimitive "GeometricPrimitive Geometry3D::getGeometricPrimitive()

Returns a GeometricPrimitive if this geometry is of type
GeometricPrimitive. ";

%feature("docstring")  Geometry3D::setTriangleMesh "void
Geometry3D::setTriangleMesh(const TriangleMesh &) ";

%feature("docstring")  Geometry3D::setPointCloud "void
Geometry3D::setPointCloud(const PointCloud &) ";

%feature("docstring")  Geometry3D::setGeometricPrimitive "void
Geometry3D::setGeometricPrimitive(const GeometricPrimitive &) ";

%feature("docstring")  Geometry3D::loadFile "bool
Geometry3D::loadFile(const char *fn)

Loads from file. Standard mesh types, PCD files, and .geom files are
supported. ";

%feature("docstring")  Geometry3D::saveFile "bool
Geometry3D::saveFile(const char *fn)

Saves to file. Standard mesh types, PCD files, and .geom files are
supported. ";

%feature("docstring")  Geometry3D::setCurrentTransform "void
Geometry3D::setCurrentTransform(const double R[9], const double t[3])

Sets the current transformation (not modifying the underlying data) ";

%feature("docstring")  Geometry3D::translate "void
Geometry3D::translate(const double t[3])

Translates the geometry data. ";

%feature("docstring")  Geometry3D::transform "void
Geometry3D::transform(const double R[9], const double t[3])

Translates/rotates the geometry data. ";

%feature("docstring")  Geometry3D::setCollisionMargin "void
Geometry3D::setCollisionMargin(double margin)

Sets a padding around the base geometry which affects the results of
proximity queries ";

%feature("docstring")  Geometry3D::getCollisionMargin "double
Geometry3D::getCollisionMargin()

Returns the padding around the base geometry. Default 0. ";

%feature("docstring")  Geometry3D::getBB "void
Geometry3D::getBB(double out[3], double out2[3])

Returns the axis-aligned bounding box of the object. ";

%feature("docstring")  Geometry3D::collides "bool
Geometry3D::collides(const Geometry3D &other)

Returns true if this geometry collides with the other. ";

%feature("docstring")  Geometry3D::withinDistance "bool
Geometry3D::withinDistance(const Geometry3D &other, double tol)

Returns true if this geometry is within distance tol to other. ";

%feature("docstring")  Geometry3D::distance "double
Geometry3D::distance(const Geometry3D &other, double relErr=0, double
absErr=0)

Returns the distance from this geometry to the other. ";

%feature("docstring")  Geometry3D::closestPoint "bool
Geometry3D::closestPoint(const double pt[3], double out[3])

Returns (success,cp) giving the closest point to the input point.
success is false if that operation is not supported with the given
geometry type. cp are given in world coordinates. ";

%feature("docstring")  Geometry3D::rayCast "bool
Geometry3D::rayCast(const double s[3], const double d[3], double
out[3])

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

%feature("docstring")  IKObjective::IKObjective "IKObjective::IKObjective() ";

%feature("docstring")  IKObjective::link "int IKObjective::link()
const

The index of the robot link that is constrained. ";

%feature("docstring")  IKObjective::destLink "int
IKObjective::destLink() const

The index of the destination link, or -1 if fixed to the world. ";

%feature("docstring")  IKObjective::numPosDims "int
IKObjective::numPosDims() const

Returns the number of position dimensions constrained (0-3) ";

%feature("docstring")  IKObjective::numRotDims "int
IKObjective::numRotDims() const

Returns the number of rotation dimensions constrained (0-3) ";

%feature("docstring")  IKObjective::setFixedPoint "void
IKObjective::setFixedPoint(int link, const double plocal[3], const
double pworld[3])

Sets a fixed-point constraint. ";

%feature("docstring")  IKObjective::setFixedPoints "void
IKObjective::setFixedPoints(int link, PyObject *plocals, PyObject
*pworlds)

Sets a multiple fixed-point constraint. ";

%feature("docstring")  IKObjective::setFixedTransform "void
IKObjective::setFixedTransform(int link, const double R[9], const
double t[3])

Sets a fixed-transform constraint (R,t) ";

%feature("docstring")  IKObjective::setRelativePoint "void
IKObjective::setRelativePoint(int link1, int link2, const double
p1[3], const double p2[3])

Sets a fixed-point constraint relative to link2. ";

%feature("docstring")  IKObjective::setRelativePoints "void
IKObjective::setRelativePoints(int link1, int link2, PyObject *p1s,
PyObject *p2s)

Sets a multiple fixed-point constraint relative to link2. ";

%feature("docstring")  IKObjective::setRelativeTransform "void
IKObjective::setRelativeTransform(int link, int linkTgt, const double
R[9], const double t[3])

Sets a fixed-transform constraint (R,t) relative to linkTgt. ";

%feature("docstring")  IKObjective::setLinks "void
IKObjective::setLinks(int link, int link2=-1)

Manual construction. ";

%feature("docstring")  IKObjective::setFreePosition "void
IKObjective::setFreePosition()

Manual: Sets a free position constraint. ";

%feature("docstring")  IKObjective::setFixedPosConstraint "void
IKObjective::setFixedPosConstraint(const double tlocal[3], const
double tworld[3])

Manual: Sets a fixed position constraint. ";

%feature("docstring")  IKObjective::setPlanarPosConstraint "void
IKObjective::setPlanarPosConstraint(const double tlocal[3], const
double nworld[3], double oworld)

Manual: Sets a planar position constraint nworld^T T(link)*tlocal +
oworld = 0 ";

%feature("docstring")  IKObjective::setLinearPosConstraint "void
IKObjective::setLinearPosConstraint(const double tlocal[3], const
double sworld[3], const double dworld[3])

Manual: Sets a linear position constraint T(link)*tlocal = sworld +
u*dworld for some real value u ";

%feature("docstring")  IKObjective::setFreeRotConstraint "void
IKObjective::setFreeRotConstraint()

Manual: Sets a free rotation constraint. ";

%feature("docstring")  IKObjective::setFixedRotConstraint "void
IKObjective::setFixedRotConstraint(const double R[9])

Manual: Sets a fixed rotation constraint. ";

%feature("docstring")  IKObjective::setAxialRotConstraint "void
IKObjective::setAxialRotConstraint(const double alocal[3], const
double aworld[3])

Manual: Sets an axial rotation constraint. ";

%feature("docstring")  IKObjective::getPosition "void
IKObjective::getPosition(double out[3], double out2[3]) const

Returns the local and global position of the position constraint. ";

%feature("docstring")  IKObjective::getPositionDirection "void
IKObjective::getPositionDirection(double out[3]) const

For linear and planar constraints, returns the direction. ";

%feature("docstring")  IKObjective::getRotation "void
IKObjective::getRotation(double out[9]) const

For fixed rotation constraints, returns the orientation. ";

%feature("docstring")  IKObjective::getRotationAxis "void
IKObjective::getRotationAxis(double out[3], double out2[3]) const

For axis rotation constraints, returns the local and global axes. ";

%feature("docstring")  IKObjective::getTransform "void
IKObjective::getTransform(double out[9], double out2[3]) const

For fixed-transform constraints, returns the transform (R,T) ";


// File: classIKSolver.xml
%feature("docstring") IKSolver "

An inverse kinematics solver based on the Newton-Raphson technique.

Typical calling pattern is s = IKSolver(robot) s.add(objective1)
s.add(objective2) (res,iters) = s.solve(100,1e-4) if res: print \"IK
solution:\",robot.getConfig(),\"found in\",iters,\"iterations,
residual\",s.getResidual() else: print \"IK
failed:\",robot.getConfig(),\"found in\",iters,\"iterations,
residual\",s.getResidual()

sampleInitial() is a convenience routine. More initial configurations
can be sampled in case the prior configs lead to local minima.

C++ includes: robotik.h ";

%feature("docstring")  IKSolver::IKSolver "IKSolver::IKSolver(const
RobotModel &robot) ";

%feature("docstring")  IKSolver::IKSolver "IKSolver::IKSolver(const
IKSolver &solver) ";

%feature("docstring")  IKSolver::add "void IKSolver::add(const
IKObjective &objective)

Adds a new simultaneous objective. ";

%feature("docstring")  IKSolver::setActiveDofs "void
IKSolver::setActiveDofs(const std::vector< int > &active)

Sets the active degrees of freedom. ";

%feature("docstring")  IKSolver::getActiveDofs "void
IKSolver::getActiveDofs(std::vector< int > &out)

Gets the active degrees of freedom. ";

%feature("docstring")  IKSolver::setJointLimits "void
IKSolver::setJointLimits(const std::vector< double > &qmin, const
std::vector< double > &qmax)

Sets limits on the robot's configuration. If empty, this turns off
joint limits. ";

%feature("docstring")  IKSolver::getJointLimits "void
IKSolver::getJointLimits(std::vector< double > &out, std::vector<
double > &out2)

Gets the limits on the robot's configuration (by default this is the
robot's joint limits. ";

%feature("docstring")  IKSolver::getResidual "void
IKSolver::getResidual(std::vector< double > &out)

Returns a vector describing the error of the objective. ";

%feature("docstring")  IKSolver::getJacobian "void
IKSolver::getJacobian(std::vector< std::vector< double > > &out)

Returns a matrix describing the instantaneous derivative of the
objective with respect to the active Dofs ";

%feature("docstring")  IKSolver::solve "PyObject *
IKSolver::solve(int iters, double tol=1e-3)

Tries to find a configuration that satifies all simultaneous
objectives up to the desired tolerance. Returns (res,iters) where res
indicates whether x converged. ";

%feature("docstring")  IKSolver::sampleInitial "void
IKSolver::sampleInitial()

Samples an initial random configuration. ";


// File: classManualOverrideController.xml
%feature("docstring") ManualOverrideController "";

%feature("docstring")
ManualOverrideController::ManualOverrideController "ManualOverrideController::ManualOverrideController(Robot &robot, const
SmartPointer< RobotController > &_base) ";

%feature("docstring")  ManualOverrideController::Type "virtual const
char* ManualOverrideController::Type() const ";

%feature("docstring")  ManualOverrideController::Update "void
ManualOverrideController::Update(Real dt) ";

%feature("docstring")  ManualOverrideController::ReadState "bool
ManualOverrideController::ReadState(File &f) ";

%feature("docstring")  ManualOverrideController::WriteState "bool
ManualOverrideController::WriteState(File &f) const ";

%feature("docstring")  ManualOverrideController::Settings "virtual
map<string,string> ManualOverrideController::Settings() const ";

%feature("docstring")  ManualOverrideController::GetSetting "virtual
bool ManualOverrideController::GetSetting(const string &name, string
&str) const ";

%feature("docstring")  ManualOverrideController::SetSetting "virtual
bool ManualOverrideController::SetSetting(const string &name, const
string &str) ";

%feature("docstring")  ManualOverrideController::Commands "virtual
vector<string> ManualOverrideController::Commands() const ";

%feature("docstring")  ManualOverrideController::SendCommand "virtual
bool ManualOverrideController::SendCommand(const string &name, const
string &str) ";


// File: structMass.xml
%feature("docstring") Mass "

Stores mass information for a rigid body or robot link.

C++ includes: robotmodel.h ";

%feature("docstring")  Mass::setMass "void Mass::setMass(double
_mass) ";

%feature("docstring")  Mass::getMass "double Mass::getMass() const ";

%feature("docstring")  Mass::setCom "void Mass::setCom(const
std::vector< double > &_com) ";

%feature("docstring")  Mass::getCom "void Mass::getCom(std::vector<
double > &out) const ";

%feature("docstring")  Mass::setInertia "void Mass::setInertia(const
std::vector< double > &_inertia) ";

%feature("docstring")  Mass::getInertia "void
Mass::getInertia(std::vector< double > &out) const ";


// File: classPlannerInterface.xml
%feature("docstring") PlannerInterface "

An interface for a motion planner. The MotionPlanner interface in
cspace.py is somewhat easier to use.

On construction, uses the planner type specified by setPlanType and
the settings currently specified by calls to setPlanSetting.

Point-to-point planning is enabled using the setEndpoints method. This
is mandatory for RRT and SBL planners. The start and end milestones
are given by indices 0 and 1, respectively

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

%feature("docstring")  PlannerInterface::PlannerInterface "PlannerInterface::PlannerInterface(const CSpaceInterface &cspace) ";

%feature("docstring")  PlannerInterface::~PlannerInterface "PlannerInterface::~PlannerInterface() ";

%feature("docstring")  PlannerInterface::destroy "void
PlannerInterface::destroy() ";

%feature("docstring")  PlannerInterface::setEndpoints "bool
PlannerInterface::setEndpoints(PyObject *start, PyObject *goal) ";

%feature("docstring")  PlannerInterface::addMilestone "int
PlannerInterface::addMilestone(PyObject *milestone) ";

%feature("docstring")  PlannerInterface::planMore "void
PlannerInterface::planMore(int iterations) ";

%feature("docstring")  PlannerInterface::getPathEndpoints "PyObject *
PlannerInterface::getPathEndpoints() ";

%feature("docstring")  PlannerInterface::getPath "PyObject *
PlannerInterface::getPath(int milestone1, int milestone2) ";

%feature("docstring")  PlannerInterface::getData "double
PlannerInterface::getData(const char *setting) ";

%feature("docstring")  PlannerInterface::getStats "PyObject *
PlannerInterface::getStats() ";

%feature("docstring")  PlannerInterface::getRoadmap "PyObject *
PlannerInterface::getRoadmap() ";

%feature("docstring")  PlannerInterface::dump "void
PlannerInterface::dump(const char *fn) ";


// File: structPointCloud.xml
%feature("docstring") PointCloud "

A 3D point cloud class.

Attributes: vertices: a list of vertices, given as a list [x1, y1, z1,
x2, y2, ... zn]

properties: a list of vertex properties, given as a list [p11, p21,
..., pk1, p12, p22, ..., pk2, ... , pn1, pn2, ..., pn2] where each
vertex has k properties. The name of each property is given by the
propertyNames member.

C++ includes: geometry.h ";

%feature("docstring")  PointCloud::numPoints "int
PointCloud::numPoints() const

Returns the number of points. ";

%feature("docstring")  PointCloud::numProperties "int
PointCloud::numProperties() const

Returns the number of properties. ";

%feature("docstring")  PointCloud::setPoints "void
PointCloud::setPoints(int num, const double plist[])

Sets all the points to the given list (a 3n-list) ";

%feature("docstring")  PointCloud::addPoint "int
PointCloud::addPoint(const double p[3])

Adds a point. Sets all its properties to 0. Returns the index. ";

%feature("docstring")  PointCloud::setPoint "void
PointCloud::setPoint(int index, const double p[3])

Sets the position of a point. ";

%feature("docstring")  PointCloud::getPoint "void
PointCloud::getPoint(int index, double out[3]) const

Retrieves the position of a point. ";

%feature("docstring")  PointCloud::setProperties "void
PointCloud::setProperties(const double properties[])

Sets all the properties of all points to the given list (a kn-list) ";

%feature("docstring")  PointCloud::setProperties "void
PointCloud::setProperties(int pindex, const double properties[])

Sets property pindex of all points to the given list (a n-list) ";

%feature("docstring")  PointCloud::setProperty "void
PointCloud::setProperty(int index, int pindex, double value)

Sets property pindex of point index to the given value. ";

%feature("docstring")  PointCloud::setProperty "void
PointCloud::setProperty(int index, const std::string &pname, double
value)

Sets the property named pname of point index to the given value. ";

%feature("docstring")  PointCloud::getProperty "double
PointCloud::getProperty(int index, int pindex) const

Gets property pindex of point index. ";

%feature("docstring")  PointCloud::getProperty "double
PointCloud::getProperty(int index, const std::string &pname) const

Gets the property named pname of point index. ";

%feature("docstring")  PointCloud::translate "void
PointCloud::translate(const double t[3])

Translates all the points by v=v+t. ";

%feature("docstring")  PointCloud::transform "void
PointCloud::transform(const double R[9], const double t[3])

Transforms all the points by the rigid transform v=R*v+t. ";

%feature("docstring")  PointCloud::join "void PointCloud::join(const
PointCloud &pc)

Adds the given point cloud to this one. They must share the same
properties or else an exception is raised ";


// File: classPyCSpace.xml
%feature("docstring") PyCSpace "

A CSpace that calls python routines for its functionality ";

%feature("docstring")  PyCSpace::PyCSpace "PyCSpace::PyCSpace() ";

%feature("docstring")  PyCSpace::~PyCSpace "virtual
PyCSpace::~PyCSpace() ";

%feature("docstring")  PyCSpace::Sample "virtual void
PyCSpace::Sample(Config &x) ";

%feature("docstring")  PyCSpace::SampleNeighborhood "virtual void
PyCSpace::SampleNeighborhood(const Config &c, double r, Config &x) ";

%feature("docstring")  PyCSpace::IsFeasible "virtual bool
PyCSpace::IsFeasible(const Config &x) ";

%feature("docstring")  PyCSpace::LocalPlanner "EdgePlanner *
PyCSpace::LocalPlanner(const Config &a, const Config &b) ";

%feature("docstring")  PyCSpace::Distance "virtual double
PyCSpace::Distance(const Config &x, const Config &y) ";

%feature("docstring")  PyCSpace::Interpolate "virtual void
PyCSpace::Interpolate(const Config &x, const Config &y, double u,
Config &out) ";

%feature("docstring")  PyCSpace::Properties "virtual void
PyCSpace::Properties(PropertyMap &props) const ";


// File: classPyEdgePlanner.xml
%feature("docstring") PyEdgePlanner "";

%feature("docstring")  PyEdgePlanner::PyEdgePlanner "PyEdgePlanner::PyEdgePlanner(PyCSpace *_space, const Config &_a, const
Config &_b) ";

%feature("docstring")  PyEdgePlanner::~PyEdgePlanner "virtual
PyEdgePlanner::~PyEdgePlanner() ";

%feature("docstring")  PyEdgePlanner::IsVisible "virtual bool
PyEdgePlanner::IsVisible() ";

%feature("docstring")  PyEdgePlanner::Eval "virtual void
PyEdgePlanner::Eval(double u, Config &x) const ";

%feature("docstring")  PyEdgePlanner::Start "virtual const Config&
PyEdgePlanner::Start() const ";

%feature("docstring")  PyEdgePlanner::Goal "virtual const Config&
PyEdgePlanner::Goal() const ";

%feature("docstring")  PyEdgePlanner::Space "virtual CSpace*
PyEdgePlanner::Space() const ";

%feature("docstring")  PyEdgePlanner::Copy "virtual EdgePlanner*
PyEdgePlanner::Copy() const ";

%feature("docstring")  PyEdgePlanner::ReverseCopy "virtual
EdgePlanner* PyEdgePlanner::ReverseCopy() const ";


// File: classRigidObjectModel.xml
%feature("docstring") RigidObjectModel "

A rigid movable object.

State is retrieved/set using get/setTransform. No velocities are
stored.

C++ includes: robotmodel.h ";

%feature("docstring")  RigidObjectModel::RigidObjectModel "RigidObjectModel::RigidObjectModel() ";

%feature("docstring")  RigidObjectModel::getID "int
RigidObjectModel::getID() ";

%feature("docstring")  RigidObjectModel::getName "const char *
RigidObjectModel::getName() ";

%feature("docstring")  RigidObjectModel::geometry "Geometry3D
RigidObjectModel::geometry() ";

%feature("docstring")  RigidObjectModel::appearance "Appearance
RigidObjectModel::appearance() ";

%feature("docstring")  RigidObjectModel::getMass "Mass
RigidObjectModel::getMass() ";

%feature("docstring")  RigidObjectModel::setMass "void
RigidObjectModel::setMass(const Mass &mass) ";

%feature("docstring")  RigidObjectModel::getContactParameters "ContactParameters RigidObjectModel::getContactParameters() ";

%feature("docstring")  RigidObjectModel::setContactParameters "void
RigidObjectModel::setContactParameters(const ContactParameters
&params) ";

%feature("docstring")  RigidObjectModel::getTransform "void
RigidObjectModel::getTransform(double out[9], double out2[3]) ";

%feature("docstring")  RigidObjectModel::setTransform "void
RigidObjectModel::setTransform(const double R[9], const double t[3])
";

%feature("docstring")  RigidObjectModel::drawGL "void
RigidObjectModel::drawGL(bool keepAppearance=true) ";


// File: classRobotModel.xml
%feature("docstring") RobotModel "

A model of a dynamic and kinematic robot.

It is important to understand that changing the configuration of the
model doesn't actually send a command to the robot. In essence, this
model maintains temporary storage for performing kinematics and
dynamics computations.

The robot maintains configuration/velocity/acceleration/torque bounds
which are not enforced by the model, but must rather be enforced by
the planner / simulator.

The state of the robot is retrieved using getConfig/getVelocity calls,
and is set using setConfig/setVelocity.

C++ includes: robotmodel.h ";

%feature("docstring")  RobotModel::RobotModel "RobotModel::RobotModel() ";

%feature("docstring")  RobotModel::getID "int RobotModel::getID()

Returns the ID of the robot in its world (Note: not the same as the
robot index) ";

%feature("docstring")  RobotModel::getName "const char *
RobotModel::getName() ";

%feature("docstring")  RobotModel::numLinks "int
RobotModel::numLinks()

Returns the number of links = number of DOF's. ";

%feature("docstring")  RobotModel::link "RobotModelLink
RobotModel::link(int index)

Returns a reference to the indexed link. ";

%feature("docstring")  RobotModel::link "RobotModelLink
RobotModel::link(const char *name)

Returns a reference to the named link. ";

%feature("docstring")  RobotModel::getLink "RobotModelLink
RobotModel::getLink(int index)

Old-style: will be deprecated. Returns a reference to the indexed
link. ";

%feature("docstring")  RobotModel::getLink "RobotModelLink
RobotModel::getLink(const char *name)

Old-style: will be deprecated. Returns a reference to the named link.
";

%feature("docstring")  RobotModel::numDrivers "int
RobotModel::numDrivers()

Returns the number of drivers. ";

%feature("docstring")  RobotModel::driver "RobotModelDriver
RobotModel::driver(int index)

Returns a reference to the indexed driver. ";

%feature("docstring")  RobotModel::driver "RobotModelDriver
RobotModel::driver(const char *name)

Returns a reference to the named driver. ";

%feature("docstring")  RobotModel::getDriver "RobotModelDriver
RobotModel::getDriver(int index)

Old-style: will be deprecated. Returns a reference to the indexed
driver. ";

%feature("docstring")  RobotModel::getDriver "RobotModelDriver
RobotModel::getDriver(const char *name)

Old-style: will be deprecated. Returns a reference to a
RobotModelDriver. ";

%feature("docstring")  RobotModel::getConfig "void
RobotModel::getConfig(std::vector< double > &out) ";

%feature("docstring")  RobotModel::getVelocity "void
RobotModel::getVelocity(std::vector< double > &out) ";

%feature("docstring")  RobotModel::setConfig "void
RobotModel::setConfig(const std::vector< double > &q) ";

%feature("docstring")  RobotModel::setVelocity "void
RobotModel::setVelocity(const std::vector< double > &dq) ";

%feature("docstring")  RobotModel::getJointLimits "void
RobotModel::getJointLimits(std::vector< double > &out, std::vector<
double > &out2) ";

%feature("docstring")  RobotModel::setJointLimits "void
RobotModel::setJointLimits(const std::vector< double > &qmin, const
std::vector< double > &qmax) ";

%feature("docstring")  RobotModel::getVelocityLimits "void
RobotModel::getVelocityLimits(std::vector< double > &out) ";

%feature("docstring")  RobotModel::setVelocityLimits "void
RobotModel::setVelocityLimits(const std::vector< double > &vmax) ";

%feature("docstring")  RobotModel::getAccelerationLimits "void
RobotModel::getAccelerationLimits(std::vector< double > &out) ";

%feature("docstring")  RobotModel::setAccelerationLimits "void
RobotModel::setAccelerationLimits(const std::vector< double > &amax)
";

%feature("docstring")  RobotModel::getTorqueLimits "void
RobotModel::getTorqueLimits(std::vector< double > &out) ";

%feature("docstring")  RobotModel::setTorqueLimits "void
RobotModel::setTorqueLimits(const std::vector< double > &tmax) ";

%feature("docstring")  RobotModel::setDOFPosition "void
RobotModel::setDOFPosition(int i, double qi)

Sets a single DOF's position. Note: if you are setting several joints
at once, use setConfig because this function computes forward
kinematics every time. ";

%feature("docstring")  RobotModel::setDOFPosition "void
RobotModel::setDOFPosition(const char *name, double qi) ";

%feature("docstring")  RobotModel::getDOFPosition "double
RobotModel::getDOFPosition(int i)

Returns a single DOF's position. ";

%feature("docstring")  RobotModel::getDOFPosition "double
RobotModel::getDOFPosition(const char *name) ";

%feature("docstring")  RobotModel::getCom "void
RobotModel::getCom(double out[3])

Returns the 3D center of mass at the current config. ";

%feature("docstring")  RobotModel::getComJacobian "void
RobotModel::getComJacobian(std::vector< std::vector< double > > &out)

Returns the 3xn Jacobian matrix of the current center of mass. ";

%feature("docstring")  RobotModel::getMassMatrix "void
RobotModel::getMassMatrix(std::vector< std::vector< double > > &out)

Returns the nxn mass matrix B(q) ";

%feature("docstring")  RobotModel::getMassMatrixInv "void
RobotModel::getMassMatrixInv(std::vector< std::vector< double > >
&out)

Returns the inverse of the nxn mass matrix B(q)^-1 (faster than
inverting result of getMassMatrix) ";

%feature("docstring")  RobotModel::getCoriolisForceMatrix "void
RobotModel::getCoriolisForceMatrix(std::vector< std::vector< double >
> &out)

Returns the Coriolis force matrix C(q,dq) for current config and
velocity. ";

%feature("docstring")  RobotModel::getCoriolisForces "void
RobotModel::getCoriolisForces(std::vector< double > &out)

Returns the Coriolis forces C(q,dq)*dq for current config and velocity
(faster than computing matrix and doing product). (\"Forces\" is
somewhat of a misnomer; the result is a joint torque vector) ";

%feature("docstring")  RobotModel::getGravityForces "void
RobotModel::getGravityForces(const double g[3], std::vector< double >
&out)

Returns the generalized gravity vector G(q) for the given workspace
gravity vector g (usually (0,0,-9.8)). (\"Forces\" is somewhat of a
misnomer; the result is a joint torque vector) ";

%feature("docstring")  RobotModel::torquesFromAccel "void
RobotModel::torquesFromAccel(const std::vector< double > &ddq,
std::vector< double > &out)

Computes the inverse dynamics (using Recursive Newton Euler solver).
Note: does not include gravity term G(q) ";

%feature("docstring")  RobotModel::accelFromTorques "void
RobotModel::accelFromTorques(const std::vector< double > &t,
std::vector< double > &out)

Computes the foward dynamics (using Recursive Newton Euler solver)
Note: does not include gravity term G(q) ";

%feature("docstring")  RobotModel::interpolate "void
RobotModel::interpolate(const std::vector< double > &a, const
std::vector< double > &b, double u, std::vector< double > &out)

Interpolates smoothly between two configurations, properly taking into
account nonstandard joints. ";

%feature("docstring")  RobotModel::distance "double
RobotModel::distance(const std::vector< double > &a, const
std::vector< double > &b)

Computes a distance between two configurations, properly taking into
account nonstandard joints. ";

%feature("docstring")  RobotModel::interpolate_deriv "void
RobotModel::interpolate_deriv(const std::vector< double > &a, const
std::vector< double > &b, std::vector< double > &out)

Returns the configuration derivative at a as you interpolate toward b
at unit speed. ";

%feature("docstring")  RobotModel::selfCollisionEnabled "bool
RobotModel::selfCollisionEnabled(int link1, int link2)

Queries whether self collisions between two links is enabled. ";

%feature("docstring")  RobotModel::enableSelfCollision "void
RobotModel::enableSelfCollision(int link1, int link2, bool value)

Enables/disables self collisions between two links (depending on
value) ";

%feature("docstring")  RobotModel::selfCollides "bool
RobotModel::selfCollides()

Returns true if the robot is in self collision (faster than manual
testing) ";

%feature("docstring")  RobotModel::drawGL "void
RobotModel::drawGL(bool keepAppearance=true)

Draws the robot geometry. If keepAppearance=true, the current
appearance is honored. Otherwise, only the raw geometry is drawn. ";


// File: classRobotModelDriver.xml
%feature("docstring") RobotModelDriver "

A reference to a driver of a RobotModel.

C++ includes: robotmodel.h ";

%feature("docstring")  RobotModelDriver::RobotModelDriver "RobotModelDriver::RobotModelDriver() ";

%feature("docstring")  RobotModelDriver::getName "const char *
RobotModelDriver::getName() ";

%feature("docstring")  RobotModelDriver::robot "RobotModel
RobotModelDriver::robot()

Returns a reference to the driver's robot. ";

%feature("docstring")  RobotModelDriver::getRobot "RobotModel
RobotModelDriver::getRobot()

Old-style: will be deprecated. ";

%feature("docstring")  RobotModelDriver::getType "const char *
RobotModelDriver::getType()

Currently can be \"normal\", \"affine\", \"rotation\",
\"translation\", or \"custom\". ";

%feature("docstring")  RobotModelDriver::getAffectedLink "int
RobotModelDriver::getAffectedLink()

Returns the single affected link for \"normal\" links. ";

%feature("docstring")  RobotModelDriver::getAffectedLinks "void
RobotModelDriver::getAffectedLinks(std::vector< int > &links)

Returns the driver's affected links. ";

%feature("docstring")  RobotModelDriver::getAffineCoeffs "void
RobotModelDriver::getAffineCoeffs(std::vector< double > &scale,
std::vector< double > &offset)

For \"affine\" links, returns the scale and offset of the driver value
mapped to the world. ";

%feature("docstring")  RobotModelDriver::setValue "void
RobotModelDriver::setValue(double val)

Sets the robot's config to correspond to the given driver value. ";

%feature("docstring")  RobotModelDriver::getValue "double
RobotModelDriver::getValue()

Gets the current driver value from the robot's config. ";

%feature("docstring")  RobotModelDriver::setVelocity "void
RobotModelDriver::setVelocity(double val)

Sets the robot's velocity to correspond to the given driver velocity
value. ";

%feature("docstring")  RobotModelDriver::getVelocity "double
RobotModelDriver::getVelocity()

Gets the current driver velocity value from the robot's velocity. ";


// File: classRobotModelLink.xml
%feature("docstring") RobotModelLink "

A reference to a link of a RobotModel.

C++ includes: robotmodel.h ";

%feature("docstring")  RobotModelLink::RobotModelLink "RobotModelLink::RobotModelLink() ";

%feature("docstring")  RobotModelLink::getID "int
RobotModelLink::getID()

Returns the ID of the robot link in its world (Note: not the same as
getIndex()) ";

%feature("docstring")  RobotModelLink::getName "const char *
RobotModelLink::getName() ";

%feature("docstring")  RobotModelLink::robot "RobotModel
RobotModelLink::robot()

Returns a reference to the link's robot. ";

%feature("docstring")  RobotModelLink::getRobot "RobotModel
RobotModelLink::getRobot()

Old-style: will be deprecated. ";

%feature("docstring")  RobotModelLink::getIndex "int
RobotModelLink::getIndex()

Returns the index of the link (on its robot). ";

%feature("docstring")  RobotModelLink::getParent "int
RobotModelLink::getParent()

Returns the index of the link's parent (on its robot). ";

%feature("docstring")  RobotModelLink::setParent "void
RobotModelLink::setParent(int p)

Sets the index of the link's parent (on its robot). ";

%feature("docstring")  RobotModelLink::geometry "Geometry3D
RobotModelLink::geometry()

Returns a reference to the link's geometry. ";

%feature("docstring")  RobotModelLink::appearance "Appearance
RobotModelLink::appearance()

Returns a reference to the link's appearance. ";

%feature("docstring")  RobotModelLink::getMass "Mass
RobotModelLink::getMass()

Retrieves the inertial properties of the link. (Note that the Mass is
given with origin at the link frame, not about the COM.) ";

%feature("docstring")  RobotModelLink::setMass "void
RobotModelLink::setMass(const Mass &mass)

Sets the inertial proerties of the link. (Note that the Mass is given
with origin at the link frame, not about the COM.) ";

%feature("docstring")  RobotModelLink::getParentTransform "void
RobotModelLink::getParentTransform(double out[9], double out2[3])

Gets transformation (R,t) to the parent link. ";

%feature("docstring")  RobotModelLink::setParentTransform "void
RobotModelLink::setParentTransform(const double R[9], const double
t[3]) ";

%feature("docstring")  RobotModelLink::getAxis "void
RobotModelLink::getAxis(double out[3])

Gets the local rotational / translational axis. ";

%feature("docstring")  RobotModelLink::setAxis "void
RobotModelLink::setAxis(const double axis[3]) ";

%feature("docstring")  RobotModelLink::getWorldPosition "void
RobotModelLink::getWorldPosition(const double plocal[3], double
out[3])

Converts point from local to world coordinates. ";

%feature("docstring")  RobotModelLink::getWorldDirection "void
RobotModelLink::getWorldDirection(const double vlocal[3], double
out[3])

Converts direction from local to world coordinates. ";

%feature("docstring")  RobotModelLink::getLocalPosition "void
RobotModelLink::getLocalPosition(const double pworld[3], double
out[3])

Converts point from world to local coordinates. ";

%feature("docstring")  RobotModelLink::getLocalDirection "void
RobotModelLink::getLocalDirection(const double vworld[3], double
out[3])

Converts direction from world to local coordinates. ";

%feature("docstring")  RobotModelLink::getTransform "void
RobotModelLink::getTransform(double out[9], double out2[3])

Gets transformation (R,t) to the world frame. ";

%feature("docstring")  RobotModelLink::setTransform "void
RobotModelLink::setTransform(const double R[9], const double t[3])

Sets transformation (R,t) to the world frame. Note: this does NOT
perform inverse kinematics. The transform is overwritten when the
robot's setConfig() method is called. ";

%feature("docstring")  RobotModelLink::getJacobian "void
RobotModelLink::getJacobian(const double p[3], std::vector<
std::vector< double > > &out)

Returns the total jacobian of the local point p (row-major matrix)
(orientation jacobian is stacked on position jacobian) ";

%feature("docstring")  RobotModelLink::getPositionJacobian "void
RobotModelLink::getPositionJacobian(const double p[3], std::vector<
std::vector< double > > &out)

Returns the jacobian of the local point p (row-major matrix) ";

%feature("docstring")  RobotModelLink::getOrientationJacobian "void
RobotModelLink::getOrientationJacobian(std::vector< std::vector<
double > > &out)

Returns the orientation jacobian of the link (row-major matrix) ";

%feature("docstring")  RobotModelLink::getVelocity "void
RobotModelLink::getVelocity(double out[3])

Returns the velocity of the origin given the robot's current velocity.
";

%feature("docstring")  RobotModelLink::getAngularVelocity "void
RobotModelLink::getAngularVelocity(double out[3])

Returns the angular velocity given the robot's current velocity. ";

%feature("docstring")  RobotModelLink::getPointVelocity "void
RobotModelLink::getPointVelocity(const double plocal[3], double
out[3])

Returns the world velocity of the point given the robot's current
velocity. ";

%feature("docstring")  RobotModelLink::drawLocalGL "void
RobotModelLink::drawLocalGL(bool keepAppearance=true)

Draws the link's geometry in its local frame. If keepAppearance=true,
the current Appearance is honored. Otherwise, just the geometry is
drawn. ";

%feature("docstring")  RobotModelLink::drawWorldGL "void
RobotModelLink::drawWorldGL(bool keepAppearance=true)

Draws the link's geometry in the world frame. If keepAppearance=true,
the current Appearance is honored. Otherwise, just the geometry is
drawn. ";


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

C++ includes: robotsim.h ";

%feature("docstring")  SimBody::enable "void SimBody::enable(bool
enabled=true)

Sets the simulation of this body on/off. ";

%feature("docstring")  SimBody::isEnabled "bool SimBody::isEnabled()

Returns true if this body is being simulated. ";

%feature("docstring")  SimBody::enableDynamics "void
SimBody::enableDynamics(bool enabled=true)

Sets the dynamic simulation of the body on/off. If false, velocities
will simply be integrated forward, and forces will not affect velocity
i.e., it will be pure kinematic simulation. ";

%feature("docstring")  SimBody::isDynamicsEnabled "bool
SimBody::isDynamicsEnabled() ";

%feature("docstring")  SimBody::applyWrench "void
SimBody::applyWrench(const double f[3], const double t[3])

Applies a force and torque about the COM at the current simulation
time step. ";

%feature("docstring")  SimBody::applyForceAtPoint "void
SimBody::applyForceAtPoint(const double f[3], const double pworld[3])

Applies a force at a given point (in world coordinates) at the current
simulation time step. ";

%feature("docstring")  SimBody::applyForceAtLocalPoint "void
SimBody::applyForceAtLocalPoint(const double f[3], const double
plocal[3])

Applies a force at a given point (in local coordinates) at the current
simulation time step. ";

%feature("docstring")  SimBody::setTransform "void
SimBody::setTransform(const double R[9], const double t[3])

Sets the body's transformation at the current simulation time step. ";

%feature("docstring")  SimBody::getTransform "void
SimBody::getTransform(double out[9], double out2[3]) ";

%feature("docstring")  SimBody::setVelocity "void
SimBody::setVelocity(const double w[3], const double v[3])

Sets the angular velocity and translational velocity at the current
simulation time step. ";

%feature("docstring")  SimBody::getVelocity "void
SimBody::getVelocity(double out[3], double out2[3])

Returns the angular velocity and translational velocity. ";

%feature("docstring")  SimBody::setCollisionPadding "void
SimBody::setCollisionPadding(double padding)

Sets the collision padding (useful for thin objects). Default is
0.0025. ";

%feature("docstring")  SimBody::getCollisionPadding "double
SimBody::getCollisionPadding() ";

%feature("docstring")  SimBody::getSurface "ContactParameters
SimBody::getSurface()

Gets (a copy of) the surface properties. ";

%feature("docstring")  SimBody::setSurface "void
SimBody::setSurface(const ContactParameters &params)

Sets the surface properties. ";


// File: structSimData.xml
%feature("docstring") SimData "

Internally used. ";


// File: classSimRobotController.xml
%feature("docstring") SimRobotController "

A controller for a simulated robot.

The basic way of using this is in \"standard\" move-to mode which
accepts a milestone (setMilestone) or list of milestones (repeated
calls to addMilestone) and interpolates dynamically from the current
configuration/velocity. To handle disturbances, a PID loop is run. The
constants of this loop are initially set in the robot file, or you can
perform tuning via setPIDGains.

Move-to motions are handled using a motion queue. To get finer-grained
control over the motion queue you may use the setLinear/setCubic/
addLinear/addCubic functions.

Arbitrary trajectories can be tracked by using setVelocity over short
time steps. Force controllers can be implemented using setTorque,
again using short time steps. These set the controller into manual
override mode. To reset back to regular motion queue control,

C++ includes: robotsim.h ";

%feature("docstring")  SimRobotController::SimRobotController "SimRobotController::SimRobotController() ";

%feature("docstring")  SimRobotController::~SimRobotController "SimRobotController::~SimRobotController() ";

%feature("docstring")  SimRobotController::setRate "void
SimRobotController::setRate(double dt)

Sets the current feedback control rate. ";

%feature("docstring")  SimRobotController::getCommandedConfig "void
SimRobotController::getCommandedConfig(std::vector< double > &out)

Returns the current commanded configuration. ";

%feature("docstring")  SimRobotController::getCommandedVelocity "void
SimRobotController::getCommandedVelocity(std::vector< double > &out)

Returns the current commanded velocity. ";

%feature("docstring")  SimRobotController::getSensedConfig "void
SimRobotController::getSensedConfig(std::vector< double > &out)

Returns the current \"sensed\" configuration from the simulator. ";

%feature("docstring")  SimRobotController::getSensedVelocity "void
SimRobotController::getSensedVelocity(std::vector< double > &out)

Returns the current \"sensed\" velocity from the simulator. ";

%feature("docstring")  SimRobotController::sensor "SimRobotSensor
SimRobotController::sensor(int index)

Returns a sensor by index. If out of bounds, a null sensor is
returned. ";

%feature("docstring")  SimRobotController::sensor "SimRobotSensor
SimRobotController::sensor(const char *name)

Returns a sensor by name. If unavailable, a null sensor is returned.
";

%feature("docstring")  SimRobotController::getSensor "SimRobotSensor
SimRobotController::getSensor(int index)

Old-style: will be deprecated. ";

%feature("docstring")  SimRobotController::getNamedSensor "SimRobotSensor SimRobotController::getNamedSensor(const std::string
&name)

Old-style: will be deprecated. ";

%feature("docstring")  SimRobotController::commands "std::vector<
std::string > SimRobotController::commands()

gets a command list ";

%feature("docstring")  SimRobotController::sendCommand "bool
SimRobotController::sendCommand(const std::string &name, const
std::string &args)

sends a command to the controller ";

%feature("docstring")  SimRobotController::getSetting "std::string
SimRobotController::getSetting(const std::string &name)

gets/sets settings of the controller ";

%feature("docstring")  SimRobotController::setSetting "bool
SimRobotController::setSetting(const std::string &name, const
std::string &val) ";

%feature("docstring")  SimRobotController::setMilestone "void
SimRobotController::setMilestone(const std::vector< double > &q)

Uses a dynamic interpolant to get from the current state to the
desired milestone (with optional ending velocity). This interpolant is
time-optimal with respect to the velocity and acceleration bounds. ";

%feature("docstring")  SimRobotController::setMilestone "void
SimRobotController::setMilestone(const std::vector< double > &q, const
std::vector< double > &dq) ";

%feature("docstring")  SimRobotController::addMilestone "void
SimRobotController::addMilestone(const std::vector< double > &q)

Same as setMilestone, but appends an interpolant onto an internal
motion queue starting at the current queued end state. ";

%feature("docstring")  SimRobotController::addMilestone "void
SimRobotController::addMilestone(const std::vector< double > &q, const
std::vector< double > &dq) ";

%feature("docstring")  SimRobotController::addMilestoneLinear "void
SimRobotController::addMilestoneLinear(const std::vector< double > &q)

Same as addMilestone, but enforces that the motion should move along a
straight-line joint-space path ";

%feature("docstring")  SimRobotController::setLinear "void
SimRobotController::setLinear(const std::vector< double > &q, double
dt)

Uses linear interpolation to get from the current configuration to the
desired configuration after time dt ";

%feature("docstring")  SimRobotController::setCubic "void
SimRobotController::setCubic(const std::vector< double > &q, const
std::vector< double > &v, double dt)

Uses cubic (Hermite) interpolation to get from the current
configuration/velocity to the desired configuration/velocity after
time dt ";

%feature("docstring")  SimRobotController::appendLinear "void
SimRobotController::appendLinear(const std::vector< double > &q,
double dt)

Same as setLinear but appends an interpolant onto the motion queue. ";

%feature("docstring")  SimRobotController::addCubic "void
SimRobotController::addCubic(const std::vector< double > &q, const
std::vector< double > &v, double dt)

Same as setCubic but appends an interpolant onto the motion queue. ";

%feature("docstring")  SimRobotController::remainingTime "double
SimRobotController::remainingTime() const

Returns the remaining duration of the motion queue. ";

%feature("docstring")  SimRobotController::setVelocity "void
SimRobotController::setVelocity(const std::vector< double > &dq,
double dt)

Sets a rate controller from the current commanded config to move at
rate dq for time dt. ";

%feature("docstring")  SimRobotController::setTorque "void
SimRobotController::setTorque(const std::vector< double > &t)

Sets a torque command controller. ";

%feature("docstring")  SimRobotController::setPIDCommand "void
SimRobotController::setPIDCommand(const std::vector< double > &qdes,
const std::vector< double > &dqdes)

Sets a PID command controller. ";

%feature("docstring")  SimRobotController::setPIDCommand "void
SimRobotController::setPIDCommand(const std::vector< double > &qdes,
const std::vector< double > &dqdes, const std::vector< double >
&tfeedforward)

Sets a PID command controller with feedforward torques. ";

%feature("docstring")  SimRobotController::setManualMode "void
SimRobotController::setManualMode(bool enabled)

Turns on/off manual mode, if either the setTorque or setPID command
were previously set. ";

%feature("docstring")  SimRobotController::getControlType "std::string SimRobotController::getControlType()

Returns the control type for the active controller valid values are:
unknown

off

torque

PID

locked_velocity ";

%feature("docstring")  SimRobotController::setPIDGains "void
SimRobotController::setPIDGains(const std::vector< double > &kP, const
std::vector< double > &kI, const std::vector< double > &kD)

Sets the PID gains. ";


// File: classSimRobotSensor.xml
%feature("docstring") SimRobotSensor "

A sensor on a simulated robot. Retreive this from the controller, and
use getMeasurements to get the currently simulated measurement vector.

type() gives you a string defining the sensor type. measurementNames()
gives you a list of names for the measurements.

C++ includes: robotsim.h ";

%feature("docstring")  SimRobotSensor::SimRobotSensor "SimRobotSensor::SimRobotSensor(SensorBase *sensor) ";

%feature("docstring")  SimRobotSensor::name "std::string
SimRobotSensor::name() ";

%feature("docstring")  SimRobotSensor::type "std::string
SimRobotSensor::type() ";

%feature("docstring")  SimRobotSensor::measurementNames "std::vector<
std::string > SimRobotSensor::measurementNames() ";

%feature("docstring")  SimRobotSensor::getMeasurements "void
SimRobotSensor::getMeasurements(std::vector< double > &out) ";


// File: classSimulator.xml
%feature("docstring") Simulator "

A dynamics simulator for a WorldModel.

C++ includes: robotsim.h ";

%feature("docstring")  Simulator::Simulator "Simulator::Simulator(const WorldModel &model, const char
*settings=NULL)

Constructs the simulator from a WorldModel. If the WorldModel was
loaded from an XML file, then the simulation setup is loaded from it.
";

%feature("docstring")  Simulator::~Simulator "Simulator::~Simulator()
";

%feature("docstring")  Simulator::reset "void Simulator::reset()

Resets to the initial state (same as setState(initialState)) ";

%feature("docstring")  Simulator::getWorld "WorldModel
Simulator::getWorld() const

Old-style: will be deprecated. ";

%feature("docstring")  Simulator::getState "string
Simulator::getState()

Returns a Base64 string representing the binary data for the current
simulation state, including controller parameters, etc. ";

%feature("docstring")  Simulator::setState "void
Simulator::setState(const std::string &str)

Sets the current simulation state from a Base64 string returned by a
prior getState call. ";

%feature("docstring")  Simulator::simulate "void
Simulator::simulate(double t)

Advances the simulation by time t, and updates the world model from
the simulation state. ";

%feature("docstring")  Simulator::fakeSimulate "void
Simulator::fakeSimulate(double t)

Advances a faked simulation by time t, and updates the world model
from the faked simulation state. ";

%feature("docstring")  Simulator::getTime "double
Simulator::getTime()

Returns the simulation time. ";

%feature("docstring")  Simulator::updateWorld "void
Simulator::updateWorld()

Updates the world model from the current simulation state. This only
needs to be called if you change the world model and want to revert
back to the simulation state. ";

%feature("docstring")  Simulator::getActualConfig "void
Simulator::getActualConfig(int robot, std::vector< double > &out)

Returns the current actual configuration of the robot from the
simulator. ";

%feature("docstring")  Simulator::getActualVelocity "void
Simulator::getActualVelocity(int robot, std::vector< double > &out)

Returns the current actual velocity of the robot from the simulator.
";

%feature("docstring")  Simulator::getActualTorques "void
Simulator::getActualTorques(int robot, std::vector< double > &out)

Returns the current actual torques on the robot's drivers from the
simulator ";

%feature("docstring")  Simulator::enableContactFeedback "void
Simulator::enableContactFeedback(int obj1, int obj2)

Call this to enable contact feedback between the two objects
(arguments are indexes returned by object.getID()). Contact feedback
has a small overhead so you may want to do this selectively. ";

%feature("docstring")  Simulator::enableContactFeedbackAll "void
Simulator::enableContactFeedbackAll()

Call this to enable contact feedback between all pairs of objects.
Contact feedback has a small overhead so you may want to do this
selectively. ";

%feature("docstring")  Simulator::inContact "bool
Simulator::inContact(int aid, int bid)

Returns true if the objects (indexes returned by object.getID()) are
in contact on the current time step ";

%feature("docstring")  Simulator::getContacts "void
Simulator::getContacts(int aid, int bid, std::vector< std::vector<
double > > &out)

Returns the list of contacts (x,n,kFriction) at the last time step.
Normals point into object a. ";

%feature("docstring")  Simulator::getContactForces "void
Simulator::getContactForces(int aid, int bid, std::vector<
std::vector< double > > &out)

Returns the list of contact forces on object a at the last time step.
";

%feature("docstring")  Simulator::contactForce "void
Simulator::contactForce(int aid, int bid, double out[3])

Returns the contact force on object a at the last time step. ";

%feature("docstring")  Simulator::contactTorque "void
Simulator::contactTorque(int aid, int bid, double out[3])

Returns the contact force on object a (about a's origin) at the last
time step. ";

%feature("docstring")  Simulator::hadContact "bool
Simulator::hadContact(int aid, int bid)

Returns true if the objects had contact over the last simulate() call.
";

%feature("docstring")  Simulator::hadSeparation "bool
Simulator::hadSeparation(int aid, int bid)

Returns true if the objects had ever separated during the last
simulate() call ";

%feature("docstring")  Simulator::meanContactForce "void
Simulator::meanContactForce(int aid, int bid, double out[3])

Returns the average contact force on object a over the last simulate()
call ";

%feature("docstring")  Simulator::controller "SimRobotController
Simulator::controller(int robot)

Returns a controller for the indicated robot. ";

%feature("docstring")  Simulator::controller "SimRobotController
Simulator::controller(const RobotModel &robot) ";

%feature("docstring")  Simulator::body "SimBody Simulator::body(const
RobotModelLink &link)

Returns the SimBody corresponding to the given link. ";

%feature("docstring")  Simulator::body "SimBody Simulator::body(const
RigidObjectModel &object)

Returns the SimBody corresponding to the given object. ";

%feature("docstring")  Simulator::body "SimBody Simulator::body(const
TerrainModel &terrain)

Returns the SimBody corresponding to the given terrain. ";

%feature("docstring")  Simulator::getController "SimRobotController
Simulator::getController(int robot)

Old-style: will be deprecated. ";

%feature("docstring")  Simulator::getController "SimRobotController
Simulator::getController(const RobotModel &robot)

Old-style: will be deprecated. ";

%feature("docstring")  Simulator::getBody "SimBody
Simulator::getBody(const RobotModelLink &link)

Old-style: will be deprecated. ";

%feature("docstring")  Simulator::getBody "SimBody
Simulator::getBody(const RigidObjectModel &object)

Old-style: will be deprecated. ";

%feature("docstring")  Simulator::getBody "SimBody
Simulator::getBody(const TerrainModel &terrain)

Old-style: will be deprecated. ";

%feature("docstring")  Simulator::getJointForces "void
Simulator::getJointForces(const RobotModelLink &link, double out[6])

Returns the joint force and torque local to the link, as would be read
by a force-torque sensor mounted at the given link's origin. The 6
entries are (fx,fy,fz,mx,my,mz) ";

%feature("docstring")  Simulator::setGravity "void
Simulator::setGravity(const double g[3])

Sets the overall gravity vector. ";

%feature("docstring")  Simulator::setSimStep "void
Simulator::setSimStep(double dt)

Sets the internal simulation substep. Values < 0.01 are recommended.
";


// File: classTerrainModel.xml
%feature("docstring") TerrainModel "

Static environment geometry.

C++ includes: robotmodel.h ";

%feature("docstring")  TerrainModel::TerrainModel "TerrainModel::TerrainModel() ";

%feature("docstring")  TerrainModel::getID "int TerrainModel::getID()
";

%feature("docstring")  TerrainModel::getName "const char *
TerrainModel::getName() ";

%feature("docstring")  TerrainModel::geometry "Geometry3D
TerrainModel::geometry() ";

%feature("docstring")  TerrainModel::appearance "Appearance
TerrainModel::appearance() ";

%feature("docstring")  TerrainModel::setFriction "void
TerrainModel::setFriction(double friction) ";

%feature("docstring")  TerrainModel::drawGL "void
TerrainModel::drawGL(bool keepAppearance=true) ";


// File: structTriangleMesh.xml
%feature("docstring") TriangleMesh "

A 3D indexed triangle mesh class.

Attributes: vertices: a list of vertices, given as a flattened
coordinate list [x1, y1, z1, x2, y2, ...]

indices: a list of triangle vertices given as indices into the
vertices list, i.e., [a1,b1,c2, a2,b2,c2, ...]

C++ includes: geometry.h ";

%feature("docstring")  TriangleMesh::translate "void
TriangleMesh::translate(const double t[3])

Translates all the vertices by v=v+t. ";

%feature("docstring")  TriangleMesh::transform "void
TriangleMesh::transform(const double R[9], const double t[3])

Transforms all the vertices by the rigid transform v=R*v+t. ";


// File: structWidgetData.xml
%feature("docstring") WidgetData "

Internally used. ";


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

%feature("docstring")  WorldModel::WorldModel "WorldModel::WorldModel()

Creates a WorldModel. With no arguments, creates a new world. With an
integer or another WorldModel instance, creates a reference to an
existing world. If passed a pointer to a C++ RobotWorld structure, a
reference is returned. (This is used pretty much only when interfacing
C++ and Python code) ";

%feature("docstring")  WorldModel::WorldModel "WorldModel::WorldModel(void *ptrRobotWorld) ";

%feature("docstring")  WorldModel::WorldModel "WorldModel::WorldModel(int index) ";

%feature("docstring")  WorldModel::WorldModel "WorldModel::WorldModel(const WorldModel &w) ";

%feature("docstring")  WorldModel::~WorldModel "WorldModel::~WorldModel() ";

%feature("docstring")  WorldModel::readFile "bool
WorldModel::readFile(const char *fn)

Reads from a world XML file. ";

%feature("docstring")  WorldModel::numRobots "int
WorldModel::numRobots() ";

%feature("docstring")  WorldModel::numRobotLinks "int
WorldModel::numRobotLinks(int robot) ";

%feature("docstring")  WorldModel::numRigidObjects "int
WorldModel::numRigidObjects() ";

%feature("docstring")  WorldModel::numTerrains "int
WorldModel::numTerrains() ";

%feature("docstring")  WorldModel::numIDs "int WorldModel::numIDs()
";

%feature("docstring")  WorldModel::robot "RobotModel
WorldModel::robot(int index) ";

%feature("docstring")  WorldModel::robot "RobotModel
WorldModel::robot(const char *name) ";

%feature("docstring")  WorldModel::robotLink "RobotModelLink
WorldModel::robotLink(int robot, int index) ";

%feature("docstring")  WorldModel::robotLink "RobotModelLink
WorldModel::robotLink(const char *robot, const char *name) ";

%feature("docstring")  WorldModel::rigidObject "RigidObjectModel
WorldModel::rigidObject(int index) ";

%feature("docstring")  WorldModel::rigidObject "RigidObjectModel
WorldModel::rigidObject(const char *name) ";

%feature("docstring")  WorldModel::terrain "TerrainModel
WorldModel::terrain(int index) ";

%feature("docstring")  WorldModel::terrain "TerrainModel
WorldModel::terrain(const char *name) ";

%feature("docstring")  WorldModel::makeRobot "RobotModel
WorldModel::makeRobot(const char *name)

Creates a new empty robot. (Not terribly useful now since you can't
resize the number of links yet) ";

%feature("docstring")  WorldModel::makeRigidObject "RigidObjectModel
WorldModel::makeRigidObject(const char *name)

Creates a new empty rigid object. ";

%feature("docstring")  WorldModel::makeTerrain "TerrainModel
WorldModel::makeTerrain(const char *name)

Creates a new empty terrain. ";

%feature("docstring")  WorldModel::loadRobot "RobotModel
WorldModel::loadRobot(const char *fn)

Loads a robot from a .rob or .urdf file. An empty robot is returned if
loading fails. ";

%feature("docstring")  WorldModel::loadRigidObject "RigidObjectModel
WorldModel::loadRigidObject(const char *fn)

Loads a rigid object from a .obj or a mesh file. An empty rigid object
is returned if loading fails. ";

%feature("docstring")  WorldModel::loadTerrain "TerrainModel
WorldModel::loadTerrain(const char *fn)

Loads a rigid object from a mesh file. An empty terrain is returned if
loading fails. ";

%feature("docstring")  WorldModel::loadElement "int
WorldModel::loadElement(const char *fn)

Loads some element from a file, automatically detecting its type.
Meshes are interpreted as terrains. The ID is returned, or -1 if
loading failed. ";

%feature("docstring")  WorldModel::remove "void
WorldModel::remove(const RobotModel &robot)

Removes a robot. It must be in this world or an exception is raised.
IMPORTANT: all other references to robots will be invalidated. ";

%feature("docstring")  WorldModel::remove "void
WorldModel::remove(const RigidObjectModel &object)

Removes a rigid object. It must be in this world or an exception is
raised. IMPORTANT: all other references to rigid objects will be
invalidated. ";

%feature("docstring")  WorldModel::remove "void
WorldModel::remove(const TerrainModel &terrain)

Removes a terrain. It must be in this world or an exception is raised.
IMPORTANT: all other references to terrains will be invalidated. ";

%feature("docstring")  WorldModel::getName "std::string
WorldModel::getName(int id)

Retrieves a name for a given element ID. ";

%feature("docstring")  WorldModel::geometry "Geometry3D
WorldModel::geometry(int id)

Retrieves a geometry for a given element ID. ";

%feature("docstring")  WorldModel::appearance "Appearance
WorldModel::appearance(int id)

Retrieves an appearance for a given element ID. ";

%feature("docstring")  WorldModel::drawGL "void WorldModel::drawGL()

Draws the entire world. ";

%feature("docstring")  WorldModel::enableGeometryLoading "void
WorldModel::enableGeometryLoading(bool enabled)

If geometry loading is set to false, then only the kinematics are
loaded from disk, and no geometry / visualization / collision
detection structures will be loaded. Useful for quick scripts that
just use kinematics / dynamics of a robot. ";


// File: namespacestd.xml


// File: geometry_8h.xml


// File: motionplanning_8cpp.xml
%feature("docstring")  std::setRandomSeed "void setRandomSeed(int
seed)

Sets the random seed used by the motion planner. ";

%feature("docstring")  std::PyListFromVector "PyObject*
PyListFromVector(const std::vector< double > &x) ";

%feature("docstring")  std::PyListToVector "bool
PyListToVector(PyObject *seq, std::vector< double > &x) ";

%feature("docstring")  std::makeNewCSpace "int makeNewCSpace() ";

%feature("docstring")  std::destroyCSpace "void destroyCSpace(int
cspace) ";

%feature("docstring")  std::setPlanJSONString "void
setPlanJSONString(const char *string)

Loads planner values from a JSON string. ";

%feature("docstring")  std::getPlanJSONString "std::string
getPlanJSONString()

Saves planner values to a JSON string. ";

%feature("docstring")  std::setPlanType "void setPlanType(const char
*type)

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

%feature("docstring")  std::setPlanSetting "void setPlanSetting(const
char *setting, double value) ";

%feature("docstring")  std::setPlanSetting "void setPlanSetting(const
char *setting, const char *value)

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

%feature("docstring")  std::makeNewPlan "int makeNewPlan(int cspace)
";

%feature("docstring")  std::destroyPlan "void destroyPlan(int plan)
";

%feature("docstring")  std::DumpPlan "void
DumpPlan(MotionPlannerInterface *planner, const char *fn) ";

%feature("docstring")  std::destroy "void destroy()

destroys internal data structures

Performs cleanup of all created spaces and planners. ";


// File: motionplanning_8h.xml
%feature("docstring")  setRandomSeed "void setRandomSeed(int seed)

Sets the random seed used by the motion planner. ";

%feature("docstring")  setPlanJSONString "void
setPlanJSONString(const char *string)

Loads planner values from a JSON string. ";

%feature("docstring")  getPlanJSONString "std::string
getPlanJSONString()

Saves planner values to a JSON string. ";

%feature("docstring")  setPlanType "void setPlanType(const char
*type)

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

%feature("docstring")  setPlanSetting "void setPlanSetting(const char
*setting, double value) ";

%feature("docstring")  setPlanSetting "void setPlanSetting(const char
*setting, const char *value)

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

%feature("docstring")  destroy "void destroy()

Performs cleanup of all created spaces and planners.

Performs cleanup of all created spaces and planners. ";


// File: robotik_8cpp.xml
%feature("docstring")  copy "void copy(const Matrix &mat, vector<
vector< double > > &v) ";

%feature("docstring")  PySequence_ToVector3 "bool
PySequence_ToVector3(PyObject *seq, Vector3 &val) ";

%feature("docstring")  PySequence_ToVector3Array "bool
PySequence_ToVector3Array(PyObject *seq, vector< Vector3 > &array) ";

%feature("docstring")  SampleTransform "void SampleTransform(const
IKGoal &goal, RigidTransform &T) ";

%feature("docstring")  SampleTransform "void SampleTransform(const
IKObjective &obj, double out[9], double out2[3])

Returns a transformation (R,t) from link to link2 sampled at random
from the space of transforms that satisfies the objective. ";

%feature("docstring")  SampleTransform "void SampleTransform(const
GeneralizedIKObjective &obj, double out[9], double out2[3]) ";


// File: robotik_8h.xml
%feature("docstring")  SampleTransform "void SampleTransform(const
IKObjective &obj, double out[9], double out2[3])

Returns a transformation (R,t) from link to link2 sampled at random
from the space of transforms that satisfies the objective. ";

%feature("docstring")  SampleTransform "void SampleTransform(const
GeneralizedIKObjective &obj, double out[9], double out2[3]) ";


// File: robotmodel_8h.xml


// File: robotsim_8cpp.xml
%feature("docstring")  createWorld "int createWorld(RobotWorld
*ptr=NULL) ";

%feature("docstring")  derefWorld "void derefWorld(int index) ";

%feature("docstring")  refWorld "void refWorld(int index) ";

%feature("docstring")  createSim "int createSim() ";

%feature("docstring")  destroySim "void destroySim(int index) ";

%feature("docstring")  createWidget "int createWidget() ";

%feature("docstring")  derefWidget "void derefWidget(int index) ";

%feature("docstring")  refWidget "void refWidget(int index) ";

%feature("docstring")  MakeController "MyController*
MakeController(Robot *robot) ";

%feature("docstring")  GetMotionQueue "PolynomialMotionQueue*
GetMotionQueue(RobotController *controller) ";

%feature("docstring")  MakeSensors "void MakeSensors(Robot *robot,
RobotSensors &sensors) ";

%feature("docstring")  GetMesh "void GetMesh(const
Geometry::AnyCollisionGeometry3D &geom, TriangleMesh &tmesh) ";

%feature("docstring")  GetMesh "void GetMesh(const TriangleMesh
&tmesh, Geometry::AnyCollisionGeometry3D &geom) ";

%feature("docstring")  GetPointCloud "void GetPointCloud(const
Geometry::AnyCollisionGeometry3D &geom, PointCloud &pc) ";

%feature("docstring")  GetPointCloud "void GetPointCloud(const
PointCloud &pc, Geometry::AnyCollisionGeometry3D &geom) ";

%feature("docstring")  copy "void copy(const Vector &vec, vector<
double > &v) ";

%feature("docstring")  copy "void copy(const vector< double > &vec,
Vector &v) ";

%feature("docstring")  copy "void copy(const Matrix &mat, vector<
double > &v) ";

%feature("docstring")  copy "void copy(const Matrix &mat, vector<
vector< double > > &v) ";

%feature("docstring")  GetCameraViewport "Camera::Viewport
GetCameraViewport(const Viewport &viewport) ";


// File: robotsim_8h.xml


// File: rootfind_8cpp.xml
%feature("docstring")  setFTolerance "void setFTolerance(double tolf)

Sets the termination threshold for the change in f. ";

%feature("docstring")  setXTolerance "void setXTolerance(double tolx)

Sets the termination threshold for the change in x. ";

%feature("docstring")  setVectorField "int setVectorField(PyObject
*pVFObj)

Sets the vector field object, returns 0 if pVFObj = NULL, 1 otherwise.
";

%feature("docstring")  setFunction "int setFunction(PyObject *pVFObj)

Sets the function object, returns 0 if pVFObj = NULL, 1 otherwise
Equivalent to setVectorField; just a more intuitive name. ";

%feature("docstring")  PyListFromVector "PyObject*
PyListFromVector(const Vector &x) ";

%feature("docstring")  findRoots "PyObject* findRoots(PyObject
*startVals, int iter)

Performs unconstrained root finding for up to iter iterations Return
values is a tuple indicating: (0,x,n) : convergence reached in x

(1,x,n) : convergence reached in f

(2,x,n) : divergence

(3,x,n) : degeneration of gradient (local extremum or saddle point)

(4,x,n) : maximum iterations reached

(5,x,n) : numerical error occurred where x is the final point and n is
the number of iterations used ";

%feature("docstring")  findRootsBounded "PyObject*
findRootsBounded(PyObject *startVals, PyObject *boundVals, int iter)

Same as findRoots, but with given bounds (xmin,xmax) ";

%feature("docstring")  destroy "void destroy()

destroys internal data structures

Performs cleanup of all created spaces and planners. ";


// File: rootfind_8h.xml
%feature("docstring")  setFTolerance "void setFTolerance(double tolf)

Sets the termination threshold for the change in f. ";

%feature("docstring")  setXTolerance "void setXTolerance(double tolx)

Sets the termination threshold for the change in x. ";

%feature("docstring")  setVectorField "int setVectorField(PyObject
*pVFObj)

Sets the vector field object, returns 0 if pVFObj = NULL, 1 otherwise.
";

%feature("docstring")  setFunction "int setFunction(PyObject *pVFObj)

Sets the function object, returns 0 if pVFObj = NULL, 1 otherwise
Equivalent to setVectorField; just a more intuitive name. ";

%feature("docstring")  findRoots "PyObject* findRoots(PyObject
*startVals, int iter)

Performs unconstrained root finding for up to iter iterations Return
values is a tuple indicating: (0,x,n) : convergence reached in x

(1,x,n) : convergence reached in f

(2,x,n) : divergence

(3,x,n) : degeneration of gradient (local extremum or saddle point)

(4,x,n) : maximum iterations reached

(5,x,n) : numerical error occurred where x is the final point and n is
the number of iterations used ";

%feature("docstring")  findRootsBounded "PyObject*
findRootsBounded(PyObject *startVals, PyObject *boundVals, int iter)

Same as findRoots, but with given bounds (xmin,xmax) ";

%feature("docstring")  destroy "void destroy()

destroys internal data structures ";

