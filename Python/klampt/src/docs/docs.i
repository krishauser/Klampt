
// File: index.xml

// File: structContactParameters.xml
%feature("docstring") ContactParameters "";


// File: classCSpaceInterface.xml
%feature("docstring") CSpaceInterface "

A raw interface for a configuration space. The CSpace interface in
cspace.py is easier to use.

C++ includes: motionplanning.h ";

%feature("docstring")  CSpaceInterface::CSpaceInterface "CSpaceInterface::CSpaceInterface() ";

%feature("docstring")  CSpaceInterface::CSpaceInterface "CSpaceInterface::CSpaceInterface(const CSpaceInterface &) ";

%feature("docstring")  CSpaceInterface::~CSpaceInterface "CSpaceInterface::~CSpaceInterface() ";

%feature("docstring")  CSpaceInterface::destroy "void
CSpaceInterface::destroy() ";

%feature("docstring")  CSpaceInterface::setFeasibility "void
CSpaceInterface::setFeasibility(PyObject *pyFeas) ";

%feature("docstring")  CSpaceInterface::setVisibility "void
CSpaceInterface::setVisibility(PyObject *pyVisible) ";

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


// File: structGeomCollisionQuery.xml
%feature("docstring") GeomCollisionQuery "";


// File: classGeometry3D.xml
%feature("docstring") Geometry3D "

A reference to a world item's three-D geometry.

C++ includes: robotmodel.h ";

%feature("docstring")  Geometry3D::Geometry3D "Geometry3D::Geometry3D() ";

%feature("docstring")  Geometry3D::type "string Geometry3D::type() ";

%feature("docstring")  Geometry3D::getTriangleMesh "TriangleMesh
Geometry3D::getTriangleMesh() ";

%feature("docstring")  Geometry3D::getPointCloud "PointCloud
Geometry3D::getPointCloud() ";

%feature("docstring")  Geometry3D::setTriangleMesh "void
Geometry3D::setTriangleMesh(const TriangleMesh &) ";

%feature("docstring")  Geometry3D::setPointCloud "void
Geometry3D::setPointCloud(const PointCloud &) ";

%feature("docstring")  Geometry3D::translate "void
Geometry3D::translate(const double t[3]) ";

%feature("docstring")  Geometry3D::transform "void
Geometry3D::transform(const double R[9], const double t[3]) ";

%feature("docstring")  Geometry3D::setCollisionMargin "void
Geometry3D::setCollisionMargin(double margin) ";

%feature("docstring")  Geometry3D::getCollisionMargin "double
Geometry3D::getCollisionMargin() ";


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

The link that is constrained. ";

%feature("docstring")  IKObjective::destLink "int
IKObjective::destLink() const

The destination link, or -1 if fixed to the world. ";

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

Stores mass information for a rigid body.

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

To get a roadmap dump, call dump(fn). This saves to a Trivial Graph
Format (TGF) format.

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

%feature("docstring")  PlannerInterface::dump "void
PlannerInterface::dump(const char *fn) ";


// File: structPointCloud.xml
%feature("docstring") PointCloud "

A 3D point cloud class. vertices is a list of vertices, given as a
list [x1, y1, z1, x2, y2, ... zn] properties is a list of vertex
properties, given as a list [p11, p21, ..., pk1, p12, p22, ..., pk2,
... , pn1, pn2, ..., pn2] where each vertex has k properties. The name
of each property is given by the propertyNames member.

C++ includes: robotmodel.h ";

%feature("docstring")  PointCloud::translate "void
PointCloud::translate(const double t[3]) ";

%feature("docstring")  PointCloud::transform "void
PointCloud::transform(const double R[9], const double t[3]) ";


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

%feature("docstring")  RigidObjectModel::getGeometry "Geometry3D
RigidObjectModel::getGeometry() ";

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

%feature("docstring")  RobotModel::getID "int RobotModel::getID() ";

%feature("docstring")  RobotModel::getName "const char *
RobotModel::getName() ";

%feature("docstring")  RobotModel::numLinks "int
RobotModel::numLinks() ";

%feature("docstring")  RobotModel::getLink "RobotModelLink
RobotModel::getLink(int index) ";

%feature("docstring")  RobotModel::getLink "RobotModelLink
RobotModel::getLink(const char *name) ";

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

%feature("docstring")  RobotModel::getCom "void
RobotModel::getCom(double out[3]) ";

%feature("docstring")  RobotModel::getComJacobian "void
RobotModel::getComJacobian(std::vector< std::vector< double > > &out)
";

%feature("docstring")  RobotModel::getMassMatrix "void
RobotModel::getMassMatrix(std::vector< std::vector< double > > &out)
";

%feature("docstring")  RobotModel::getMassMatrixInv "void
RobotModel::getMassMatrixInv(std::vector< std::vector< double > >
&out) ";

%feature("docstring")  RobotModel::getCoriolisForceMatrix "void
RobotModel::getCoriolisForceMatrix(std::vector< std::vector< double >
> &out) ";

%feature("docstring")  RobotModel::getCoriolisForces "void
RobotModel::getCoriolisForces(std::vector< double > &out) ";

%feature("docstring")  RobotModel::getGravityForces "void
RobotModel::getGravityForces(const double g[3], std::vector< double >
&out) ";

%feature("docstring")  RobotModel::torquesFromAccel "void
RobotModel::torquesFromAccel(const std::vector< double > &ddq,
std::vector< double > &out) ";

%feature("docstring")  RobotModel::accelFromTorques "void
RobotModel::accelFromTorques(const std::vector< double > &t,
std::vector< double > &out) ";

%feature("docstring")  RobotModel::interpolate "void
RobotModel::interpolate(const std::vector< double > &a, const
std::vector< double > &b, double u, std::vector< double > &out) ";

%feature("docstring")  RobotModel::distance "double
RobotModel::distance(const std::vector< double > &a, const
std::vector< double > &b) ";

%feature("docstring")  RobotModel::interpolate_deriv "void
RobotModel::interpolate_deriv(const std::vector< double > &a, const
std::vector< double > &b, std::vector< double > &out) ";

%feature("docstring")  RobotModel::selfCollisionEnabled "bool
RobotModel::selfCollisionEnabled(int link1, int link2) ";

%feature("docstring")  RobotModel::enableSelfCollision "void
RobotModel::enableSelfCollision(int link1, int link2, bool value) ";

%feature("docstring")  RobotModel::drawGL "void
RobotModel::drawGL(bool keepAppearance=true) ";


// File: classRobotModelLink.xml
%feature("docstring") RobotModelLink "

A reference to a link of a RobotModel.

Note that the mass is given local to the link frame, not about the
COM.

C++ includes: robotmodel.h ";

%feature("docstring")  RobotModelLink::RobotModelLink "RobotModelLink::RobotModelLink() ";

%feature("docstring")  RobotModelLink::getID "int
RobotModelLink::getID() ";

%feature("docstring")  RobotModelLink::getName "const char *
RobotModelLink::getName() ";

%feature("docstring")  RobotModelLink::getRobot "RobotModel
RobotModelLink::getRobot() ";

%feature("docstring")  RobotModelLink::getParent "int
RobotModelLink::getParent() ";

%feature("docstring")  RobotModelLink::setParent "void
RobotModelLink::setParent(int p) ";

%feature("docstring")  RobotModelLink::getGeometry "Geometry3D
RobotModelLink::getGeometry() ";

%feature("docstring")  RobotModelLink::getMass "Mass
RobotModelLink::getMass() ";

%feature("docstring")  RobotModelLink::setMass "void
RobotModelLink::setMass(const Mass &mass) ";

%feature("docstring")  RobotModelLink::getParentTransform "void
RobotModelLink::getParentTransform(double out[9], double out2[3])

Gets transformation (R,t) to the parent link. ";

%feature("docstring")  RobotModelLink::setParentTransform "void
RobotModelLink::setParentTransform(const double R[9], const double
t[3]) ";

%feature("docstring")  RobotModelLink::getAxis "void
RobotModelLink::getAxis(double out[3])

Gets the local rotational axis. ";

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
RobotModelLink::drawLocalGL(bool keepAppearance=true) ";

%feature("docstring")  RobotModelLink::drawWorldGL "void
RobotModelLink::drawWorldGL(bool keepAppearance=true) ";


// File: classSimBody.xml
%feature("docstring") SimBody "

A reference to a rigid body inside a Simulator (either a
RigidObjectModel, TerrainModel, or a link of a RobotModel).

C++ includes: robotsim.h ";

%feature("docstring")  SimBody::applyWrench "void
SimBody::applyWrench(const double f[3], const double t[3])

Applies a force and torque about the COM. ";

%feature("docstring")  SimBody::setVelocity "void
SimBody::setVelocity(const double w[3], const double v[3])

Sets the angular velocity and translational velocity. ";

%feature("docstring")  SimBody::getVelocity "void
SimBody::getVelocity(double out[3], double out2[3])

Returns the angular velocity and translational velocity. ";

%feature("docstring")  SimBody::setTransform "void
SimBody::setTransform(const double R[9], double t[3]) ";

%feature("docstring")  SimBody::getTransform "void
SimBody::getTransform(double out[9], double out2[3]) ";

%feature("docstring")  SimBody::setCollisionPadding "void
SimBody::setCollisionPadding(double padding)

Sets the collision padding (useful for thin objects) ";

%feature("docstring")  SimBody::getCollisionPadding "double
SimBody::getCollisionPadding() ";

%feature("docstring")  SimBody::surface "ODESurfaceProperties *
SimBody::surface()

Gets/sets the surface properties. ";


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

%feature("docstring")  SimRobotController::getSensor "SimRobotSensor
SimRobotController::getSensor(int index)

Returns a sensor by index. If out of bounds, a null sensor is
returned. ";

%feature("docstring")  SimRobotController::getNamedSensor "SimRobotSensor SimRobotController::getNamedSensor(const std::string
&name)

Returns a sensor by name. If unavailable, a null sensor is returned.
";

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

%feature("docstring")  Simulator::Simulator "Simulator::Simulator(const WorldModel &model)

Constructs the simulator from a WorldModel. If the WorldModel was
loaded from an XML file, then the simulation setup is loaded from it.
";

%feature("docstring")  Simulator::~Simulator "Simulator::~Simulator()
";

%feature("docstring")  Simulator::reset "void Simulator::reset()

Resets to the initial state (same as setState(initialState)) ";

%feature("docstring")  Simulator::getWorld "WorldModel
Simulator::getWorld() const

Returns the associated world model. ";

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

%feature("docstring")  Simulator::getController "SimRobotController
Simulator::getController(int robot)

Returns a controller for the indicated robot. ";

%feature("docstring")  Simulator::getController "SimRobotController
Simulator::getController(const RobotModel &robot) ";

%feature("docstring")  Simulator::getBody "SimBody
Simulator::getBody(const RobotModelLink &link) ";

%feature("docstring")  Simulator::getBody "SimBody
Simulator::getBody(const RigidObjectModel &object) ";

%feature("docstring")  Simulator::getBody "SimBody
Simulator::getBody(const TerrainModel &terrain) ";

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

%feature("docstring")  TerrainModel::getGeometry "Geometry3D
TerrainModel::getGeometry() ";

%feature("docstring")  TerrainModel::setFriction "void
TerrainModel::setFriction(double friction) ";

%feature("docstring")  TerrainModel::drawGL "void
TerrainModel::drawGL(bool keepAppearance=true) ";


// File: structTriangleMesh.xml
%feature("docstring") TriangleMesh "

A 3D indexed triangle mesh class.

vertices is a list of vertices, given as a list [x1, y1, z1, x2, y2,
...] indices is a list of triangle vertices given as indices into the
vertices list, i.e., [a1,b1,c2, a2,b2,c2, ...]

C++ includes: robotmodel.h ";

%feature("docstring")  TriangleMesh::translate "void
TriangleMesh::translate(const double t[3]) ";

%feature("docstring")  TriangleMesh::transform "void
TriangleMesh::transform(const double R[9], const double t[3]) ";


// File: structWorldData.xml
%feature("docstring") WorldData "

Internally used. ";


// File: classWorldModel.xml
%feature("docstring") WorldModel "

The main world class, containing robots, rigid objects, and static
environment geometry.

Note that this is just a model and can be changed at will in fact
planners and simulators will make use of a model to \"display\"
computed states.

To save/restore the state of the model, you must manually maintain
copies of the states of whichever objects you wish to save/restore.

C++ includes: robotmodel.h ";

%feature("docstring")  WorldModel::WorldModel "WorldModel::WorldModel() ";

%feature("docstring")  WorldModel::WorldModel "WorldModel::WorldModel(int index) ";

%feature("docstring")  WorldModel::WorldModel "WorldModel::WorldModel(const WorldModel &w) ";

%feature("docstring")  WorldModel::~WorldModel "WorldModel::~WorldModel() ";

%feature("docstring")  WorldModel::readFile "bool
WorldModel::readFile(const char *fn) ";

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
WorldModel::makeRobot(const char *name) ";

%feature("docstring")  WorldModel::makeRigidObject "RigidObjectModel
WorldModel::makeRigidObject(const char *name) ";

%feature("docstring")  WorldModel::makeTerrain "TerrainModel
WorldModel::makeTerrain(const char *name) ";

%feature("docstring")  WorldModel::loadRobot "RobotModel
WorldModel::loadRobot(const char *fn) ";

%feature("docstring")  WorldModel::loadRigidObject "RigidObjectModel
WorldModel::loadRigidObject(const char *fn) ";

%feature("docstring")  WorldModel::loadTerrain "TerrainModel
WorldModel::loadTerrain(const char *fn) ";

%feature("docstring")  WorldModel::loadElement "int
WorldModel::loadElement(const char *fn) ";

%feature("docstring")  WorldModel::drawGL "void WorldModel::drawGL()
";


// File: namespaceGeometry.xml


// File: namespaceGLDraw.xml


// File: namespaceMath3D.xml


// File: namespacestd.xml


// File: collide_8cpp.xml
%feature("docstring")  std::newGeom "int newGeom() ";

%feature("docstring")  std::destroyGeom "void destroyGeom(int geom)
";

%feature("docstring")  std::loadGeom "bool loadGeom(int geom, const
char *fn)

Loads a geometry from a file. Sets it to the correct type based on the
file contents.

Currently supported file extensions are Trimeshes: .tri, any other
mesh files that Assimp may support (if Klamp't is built using Assimp
support).

Point clouds: .pcd

Primitive geometries: .geom

Returns False if there is a load error. Raises an exception if the ID
is invalid. ";

%feature("docstring")  std::makeTriMeshGeom "void makeTriMeshGeom(int
geom, const char *fn)

Makes a geometry into a trimesh loaded from the file fn. ";

%feature("docstring")  std::makeTriMeshGeom "void makeTriMeshGeom(int
geom, const double *verts, const int *inds, int nv, int nt)

Makes a geometry into a trimesh given the vertex and index data. verts
is of length nv*3, and inds is of length nt*3.

Note: in Python, must use doubleArray and intArray for the verts and
inds objects. ";

%feature("docstring")  std::setTriMeshTranslation "void
setTriMeshTranslation(int geom, const double t[3])

Sets the translation of a trimesh geom. ";

%feature("docstring")  std::setTriMeshRotation "void
setTriMeshRotation(int geom, const double r[9])

Sets the rotation of a trimesh geom. ";

%feature("docstring")  std::getTriMeshTranslation "void
getTriMeshTranslation(int geom, double t[3])

Gets the translation of a trimesh geom. ";

%feature("docstring")  std::getTriMeshRotation "void
getTriMeshRotation(int geom, double r[9])

Gets the rotation of a trimesh geom. ";

%feature("docstring")  std::getTriMeshBB "void getTriMeshBB(int geom,
double bmin[3], double bmax[3])

Gets the bounding box of a trimesh geom. ";

%feature("docstring")  std::getTriMeshNumVerts "int
getTriMeshNumVerts(int geom)

Gets the number of vertices of a trimesh geom. ";

%feature("docstring")  std::getTriMeshNumTris "int
getTriMeshNumTris(int geom)

Gets the number of triangles of a trimesh geom. ";

%feature("docstring")  std::getTriMeshVerts "double*
getTriMeshVerts(int geom)

Gets the vertex data of a trimesh geom (length nv*3). ";

%feature("docstring")  std::getTriMeshTris "int* getTriMeshTris(int
geom)

Gets the index data of a trimesh geom (length nt*3). ";

%feature("docstring")  std::makePointGeom "void makePointGeom(int
geom, const double x[3])

Makes a geom into a point x. ";

%feature("docstring")  std::makeSphereGeom "void makeSphereGeom(int
geom, const double c[3], double r)

Makes a geom into a sphere centered at c with radius r. ";

%feature("docstring")  std::makeRayGeom "void makeRayGeom(int geom,
const double s[3], const double d[3])

Makes a geom into a ray with source s and direction d. ";

%feature("docstring")  std::makeLineGeom "void makeLineGeom(int geom,
const double s[3], const double d[3])

Makes a geom into a line with source s and direction d. ";

%feature("docstring")  std::makeSegmentGeom "void makeSegmentGeom(int
geom, const double a[3], const double b[3])

Makes a geom into a segment with endpoints a, b. ";

%feature("docstring")  std::makeAABBGeom "void makeAABBGeom(int geom,
const double bmin[3], const double bmax[3])

Makes a geom into an axis-aligned bounding box with lower bound bmin
and upper bound bmax. ";

%feature("docstring")  std::checkCircularReference "bool
checkCircularReference(int geom, int checkIndex) ";

%feature("docstring")  std::makeGroupGeom "void makeGroupGeom(int
geom, int *elements, int numelements)

Makes a geom into a group geom from an array of other geoms. Note: in
Python, must use an intArray for geoms argument. ";

%feature("docstring")  std::Collide "bool Collide(const Point3D &p1,
const Point3D &p2) ";

%feature("docstring")  std::Collide "bool Collide(const Point3D &p,
const Sphere3D &s) ";

%feature("docstring")  std::Collide "bool Collide(const Point3D &p,
const Segment3D &s) ";

%feature("docstring")  std::Collide "bool Collide(const Point3D &p,
const Ray3D &r) ";

%feature("docstring")  std::Collide "bool Collide(const Point3D &p,
const Line3D &l) ";

%feature("docstring")  std::Collide "bool Collide(const Point3D &p,
const AABB3D &a) ";

%feature("docstring")  std::Collide "bool Collide(const Point3D &p,
const CollisionMesh &m) ";

%feature("docstring")  std::Collide "bool Collide(const Sphere3D &s,
const Point3D &p) ";

%feature("docstring")  std::Collide "bool Collide(const Sphere3D &s1,
const Sphere3D &s2) ";

%feature("docstring")  std::Collide "bool Collide(const Sphere3D &s,
const Segment3D &seg) ";

%feature("docstring")  std::Collide "bool Collide(const Sphere3D &s,
const Ray3D &r) ";

%feature("docstring")  std::Collide "bool Collide(const Sphere3D &s,
const Line3D &r) ";

%feature("docstring")  std::Collide "bool Collide(const Sphere3D &s,
const AABB3D &a) ";

%feature("docstring")  std::Collide "bool Collide(const Sphere3D &s,
const CollisionMesh &m) ";

%feature("docstring")  std::Collide "bool Collide(const Segment3D &s,
const Point3D &p) ";

%feature("docstring")  std::Collide "bool Collide(const Segment3D
&seg, const Sphere3D &s) ";

%feature("docstring")  std::Collide "bool Collide(const Segment3D
&s1, const Segment3D &s2) ";

%feature("docstring")  std::Collide "bool Collide(const Segment3D &s,
const Ray3D &r) ";

%feature("docstring")  std::Collide "bool Collide(const Segment3D &s,
const Line3D &l) ";

%feature("docstring")  std::Collide "bool Collide(const Segment3D
&seg, const AABB3D &a) ";

%feature("docstring")  std::Collide "bool Collide(const Segment3D &s,
const CollisionMesh &m) ";

%feature("docstring")  std::Collide "bool Collide(const Ray3D &r,
const Point3D &p) ";

%feature("docstring")  std::Collide "bool Collide(const Ray3D &r,
const Sphere3D &s) ";

%feature("docstring")  std::Collide "bool Collide(const Ray3D &r,
const Segment3D &s) ";

%feature("docstring")  std::Collide "bool Collide(const Ray3D &r1,
const Ray3D &r2) ";

%feature("docstring")  std::Collide "bool Collide(const Ray3D &r,
const Line3D &l) ";

%feature("docstring")  std::Collide "bool Collide(const Ray3D &r,
const AABB3D &a) ";

%feature("docstring")  std::Collide "bool Collide(const Ray3D &r,
const CollisionMesh &m) ";

%feature("docstring")  std::Collide "bool Collide(const Line3D &l,
const Point3D &p) ";

%feature("docstring")  std::Collide "bool Collide(const Line3D &l,
const Sphere3D &s) ";

%feature("docstring")  std::Collide "bool Collide(const Line3D &l,
const Segment3D &s) ";

%feature("docstring")  std::Collide "bool Collide(const Line3D &l,
const Ray3D &r) ";

%feature("docstring")  std::Collide "bool Collide(const Line3D &l1,
const Line3D &l2) ";

%feature("docstring")  std::Collide "bool Collide(const Line3D &l,
const AABB3D &a) ";

%feature("docstring")  std::Collide "bool Collide(const Line3D &l,
const CollisionMesh &m) ";

%feature("docstring")  std::Collide "bool Collide(const AABB3D &a,
const Point3D &p) ";

%feature("docstring")  std::Collide "bool Collide(const AABB3D &a,
const Sphere3D &s) ";

%feature("docstring")  std::Collide "bool Collide(const AABB3D &a,
const Segment3D &seg) ";

%feature("docstring")  std::Collide "bool Collide(const AABB3D &a,
const Ray3D &r) ";

%feature("docstring")  std::Collide "bool Collide(const AABB3D &a,
const Line3D &l) ";

%feature("docstring")  std::Collide "bool Collide(const AABB3D &a1,
const AABB3D &a2) ";

%feature("docstring")  std::Collide "bool Collide(const AABB3D &a,
const CollisionMesh &m) ";

%feature("docstring")  std::Collide "bool Collide(const CollisionMesh
&m, const Point3D &p) ";

%feature("docstring")  std::Collide "bool Collide(const CollisionMesh
&m, const Segment3D &s) ";

%feature("docstring")  std::Collide "bool Collide(const CollisionMesh
&m, const Ray3D &r) ";

%feature("docstring")  std::Collide "bool Collide(const CollisionMesh
&m, const Line3D &l) ";

%feature("docstring")  std::collideMesh "bool
collideMesh(CollisionMesh *m1, int geom2) ";

%feature("docstring")  std::collidePoint "bool collidePoint(Point3D
*p1, int geom2) ";

%feature("docstring")  std::collideSphere "bool
collideSphere(Sphere3D *s1, int geom2) ";

%feature("docstring")  std::collideSegment "bool
collideSegment(Segment3D *s1, int geom2) ";

%feature("docstring")  std::collideRay "bool collideRay(Ray3D *s1,
int geom2) ";

%feature("docstring")  std::collideLine "bool collideLine(Line3D *s1,
int geom2) ";

%feature("docstring")  std::collideAABB "bool collideAABB(AABB3D *a1,
int geom2) ";

%feature("docstring")  std::collideGroup "bool collideGroup(const
vector< int > &group, int geom2) ";

%feature("docstring")  std::collide "bool collide(int geom1, int
geom2)

Tests whether the two geometries collide. ";

%feature("docstring")  std::withinTolerance "bool withinTolerance(int
geom1, int geom2, double tol)

Tests whether the two geometries are within the given tolerance. ";

%feature("docstring")  std::distance "double distance(int geom1, int
geom2, double relErr, double absErr)

Returns the distance between the two geometries, possibly with an
approximation error (useful to speed up mesh-mesh distance detection)

Error of result is no more than D*relErr+absErr where D is the actual
distance. Set relErr=absErr=0 to get exact distance.

NOTE: Not yet implemented. ";

%feature("docstring")  std::closestPoints "void closestPoints(int
geom1, int geom2, double p1[3], double p2[3])

Returns the closest points between the two geometries. These are given
in world coordinates.

NOTE: Not yet implemented. ";

%feature("docstring")  std::rayCast "bool rayCast(int geom, const
double s[3], const double d[3], double out[3])

Returns true if the geometry is hit by the given ray, and also returns
the hit point (in world coordinates). ";

%feature("docstring")  std::makeCollQuery "int makeCollQuery(int
geom1, int geom2)

Creates a collision query object attachd to the two given geometries.
For mesh-mesh collisions, on repeated calls, this may be somewhat
faster than querying from scratch. ";

%feature("docstring")  std::destroyCollQuery "void
destroyCollQuery(int query)

Deletes a collision query object. ";

%feature("docstring")  std::queryCollide "bool queryCollide(int
query)

Checks if the two geoms associated with this query are colliding. ";

%feature("docstring")  std::queryWithinTolerance "bool
queryWithinTolerance(int query, double tol)

Checks if the two geoms associated with this query are within the
given tolerance.

See:   withinTolerance ";

%feature("docstring")  std::queryDistance "double queryDistance(int
query, double relErr, double absErr)

Computes the distance betweeen the two geoms associated with this
query.

See:   distance ";

%feature("docstring")  std::queryClosestPoints "void
queryClosestPoints(int query, double p1[3], double p2[3])

Computes points that give rise to the closest distance betweeen the
two geoms associated with this query.

See:   closestPoints ";

%feature("docstring")  std::queryTolerancePoints "void
queryTolerancePoints(int query, double p1[3], double p2[3])

If the two geoms associated with this query are within a given
tolerance (from a previous queryWithinTolerance call), this produces
the points on geom1 and geom2, respectively that are within that
tolerance. ";

%feature("docstring")  std::destroy "void destroy()

Frees all memory allocated by the collide module. All existing
geometry ids and collision query ids are invalidated.

Performs cleanup of all created spaces and planners.

destroys internal data structures ";


// File: collide_8h.xml
%feature("docstring")  newGeom "int newGeom() ";

%feature("docstring")  destroyGeom "void destroyGeom(int geom) ";

%feature("docstring")  destroy "void destroy()

Frees all memory allocated by the collide module. All existing
geometry ids and collision query ids are invalidated. ";

%feature("docstring")  loadGeom "bool loadGeom(int geom, const char
*fn)

Loads a geometry from a file. Sets it to the correct type based on the
file contents.

Currently supported file extensions are Trimeshes: .tri, any other
mesh files that Assimp may support (if Klamp't is built using Assimp
support).

Point clouds: .pcd

Primitive geometries: .geom

Returns False if there is a load error. Raises an exception if the ID
is invalid. ";

%feature("docstring")  makeTriMeshGeom "void makeTriMeshGeom(int
geom, const char *fn)

Makes a geometry into a trimesh loaded from the file fn. ";

%feature("docstring")  makeTriMeshGeom "void makeTriMeshGeom(int
geom, const double *verts, const int *inds, int nv, int nt)

Makes a geometry into a trimesh given the vertex and index data. verts
is of length nv*3, and inds is of length nt*3.

Note: in Python, must use doubleArray and intArray for the verts and
inds objects. ";

%feature("docstring")  setTriMeshTranslation "void
setTriMeshTranslation(int geom, const double t[3])

Sets the translation of a trimesh geom. ";

%feature("docstring")  setTriMeshRotation "void
setTriMeshRotation(int geom, const double r[9])

Sets the rotation of a trimesh geom. ";

%feature("docstring")  getTriMeshTranslation "void
getTriMeshTranslation(int geom, double out[3])

Gets the translation of a trimesh geom. ";

%feature("docstring")  getTriMeshRotation "void
getTriMeshRotation(int geom, double out[9])

Gets the rotation of a trimesh geom. ";

%feature("docstring")  getTriMeshBB "void getTriMeshBB(int geom,
double out[3], double out2[3])

Gets the bounding box of a trimesh geom. ";

%feature("docstring")  getTriMeshNumVerts "int getTriMeshNumVerts(int
geom)

Gets the number of vertices of a trimesh geom. ";

%feature("docstring")  getTriMeshNumTris "int getTriMeshNumTris(int
geom)

Gets the number of triangles of a trimesh geom. ";

%feature("docstring")  getTriMeshVerts "double* getTriMeshVerts(int
geom)

Gets the vertex data of a trimesh geom (length nv*3). ";

%feature("docstring")  getTriMeshTris "int* getTriMeshTris(int geom)

Gets the index data of a trimesh geom (length nt*3). ";

%feature("docstring")  makePointGeom "void makePointGeom(int geom,
const double x[3])

Makes a geom into a point x. ";

%feature("docstring")  makeSphereGeom "void makeSphereGeom(int geom,
const double c[3], double r)

Makes a geom into a sphere centered at c with radius r. ";

%feature("docstring")  makeRayGeom "void makeRayGeom(int geom, const
double s[3], const double d[3])

Makes a geom into a ray with source s and direction d. ";

%feature("docstring")  makeLineGeom "void makeLineGeom(int geom,
const double s[3], const double d[3])

Makes a geom into a line with source s and direction d. ";

%feature("docstring")  makeSegmentGeom "void makeSegmentGeom(int
geom, const double a[3], const double b[3])

Makes a geom into a segment with endpoints a, b. ";

%feature("docstring")  makeAABBGeom "void makeAABBGeom(int geom,
const double bmin[3], const double bmax[3])

Makes a geom into an axis-aligned bounding box with lower bound bmin
and upper bound bmax. ";

%feature("docstring")  makeGroupGeom "void makeGroupGeom(int geom,
int *geoms, int numgeoms)

Makes a geom into a group geom from an array of other geoms. Note: in
Python, must use an intArray for geoms argument. ";

%feature("docstring")  collide "bool collide(int geom1, int geom2)

Tests whether the two geometries collide. ";

%feature("docstring")  withinTolerance "bool withinTolerance(int
geom1, int geom2, double tol)

Tests whether the two geometries are within the given tolerance. ";

%feature("docstring")  distance "double distance(int geom1, int
geom2, double relErr, double absErr)

Returns the distance between the two geometries, possibly with an
approximation error (useful to speed up mesh-mesh distance detection)

Error of result is no more than D*relErr+absErr where D is the actual
distance. Set relErr=absErr=0 to get exact distance.

NOTE: Not yet implemented. ";

%feature("docstring")  closestPoints "void closestPoints(int geom1,
int geom2, double out[3], double out2[3])

Returns the closest points between the two geometries. These are given
in world coordinates.

NOTE: Not yet implemented. ";

%feature("docstring")  rayCast "bool rayCast(int geom, const double
s[3], const double d[3], double out[3])

Returns true if the geometry is hit by the given ray, and also returns
the hit point (in world coordinates). ";

%feature("docstring")  makeCollQuery "int makeCollQuery(int geom1,
int geom2)

Creates a collision query object attachd to the two given geometries.
For mesh-mesh collisions, on repeated calls, this may be somewhat
faster than querying from scratch. ";

%feature("docstring")  destroyCollQuery "void destroyCollQuery(int
query)

Deletes a collision query object. ";

%feature("docstring")  queryCollide "bool queryCollide(int query)

Checks if the two geoms associated with this query are colliding. ";

%feature("docstring")  queryWithinTolerance "bool
queryWithinTolerance(int query, double tol)

Checks if the two geoms associated with this query are within the
given tolerance.

See:   withinTolerance ";

%feature("docstring")  queryDistance "double queryDistance(int query,
double relErr, double absErr)

Computes the distance betweeen the two geoms associated with this
query.

See:   distance ";

%feature("docstring")  queryClosestPoints "void
queryClosestPoints(int query, double out[3], double out2[3])

Computes points that give rise to the closest distance betweeen the
two geoms associated with this query.

See:   closestPoints ";

%feature("docstring")  queryTolerancePoints "void
queryTolerancePoints(int query, double out[3], double out2[3])

If the two geoms associated with this query are within a given
tolerance (from a previous queryWithinTolerance call), this produces
the points on geom1 and geom2, respectively that are within that
tolerance. ";


// File: motionplanning_8cpp.xml
%feature("docstring")  setRandomSeed "void setRandomSeed(int seed)

Sets the random seed used by the motion planner. ";

%feature("docstring")  PyListFromVector "PyObject*
PyListFromVector(const std::vector< double > &x) ";

%feature("docstring")  PyListToVector "bool PyListToVector(PyObject
*seq, std::vector< double > &x) ";

%feature("docstring")  makeNewCSpace "int makeNewCSpace() ";

%feature("docstring")  destroyCSpace "void destroyCSpace(int cspace)
";

%feature("docstring")  setPlanType "void setPlanType(const char
*type)

Sets the planner type.

Valid values are \"prm\": Probabilistic roadmap

\"rrt\": Rapidly-exploring Random Trees

\"sbl\": The SBL (single-query, bidirectional, lazy) planner ";

%feature("docstring")  setPlanSetting "void setPlanSetting(const char
*setting, double value)

Sets a setting for the planner.

Valid values are \"knn\": k value for the k-nearest neighbor
connection strategy (only for PRM)

\"connectionThreshold\": a milestone connection threshold

\"perturbationRadius\": (only for RRT and SBL)

\"bidirectional\": 1 if bidirectional planning is requested (only for
RRT)

\"grid\": 1 if a point selection grid should be used (only for SBL)

\"gridResolution\": resolution for the grid, if the grid should be
used

\"randomizeFrequency\": a grid randomization frequency (only for SBL)
";

%feature("docstring")  makeNewPlan "int makeNewPlan(int cspace) ";

%feature("docstring")  destroyPlan "void destroyPlan(int plan) ";

%feature("docstring")  DumpPlan "void DumpPlan(MotionPlannerInterface
*planner, const char *fn) ";

%feature("docstring")  destroy "void destroy()

Frees all memory allocated by the collide module. All existing
geometry ids and collision query ids are invalidated.

Performs cleanup of all created spaces and planners.

destroys internal data structures ";


// File: motionplanning_8h.xml
%feature("docstring")  setRandomSeed "void setRandomSeed(int seed)

Sets the random seed used by the motion planner. ";

%feature("docstring")  setPlanType "void setPlanType(const char
*type)

Sets the planner type.

Valid values are \"prm\": Probabilistic roadmap

\"rrt\": Rapidly-exploring Random Trees

\"sbl\": The SBL (single-query, bidirectional, lazy) planner ";

%feature("docstring")  setPlanSetting "void setPlanSetting(const char
*setting, double value)

Sets a setting for the planner.

Valid values are \"knn\": k value for the k-nearest neighbor
connection strategy (only for PRM)

\"connectionThreshold\": a milestone connection threshold

\"perturbationRadius\": (only for RRT and SBL)

\"bidirectional\": 1 if bidirectional planning is requested (only for
RRT)

\"grid\": 1 if a point selection grid should be used (only for SBL)

\"gridResolution\": resolution for the grid, if the grid should be
used

\"randomizeFrequency\": a grid randomization frequency (only for SBL)
";

%feature("docstring")  destroy "void destroy()

Performs cleanup of all created spaces and planners.

Performs cleanup of all created spaces and planners.

destroys internal data structures ";


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
%feature("docstring")  GLDraw::createWorld "int createWorld() ";

%feature("docstring")  GLDraw::derefWorld "void derefWorld(int index)
";

%feature("docstring")  GLDraw::refWorld "void refWorld(int index) ";

%feature("docstring")  GLDraw::createSim "int createSim() ";

%feature("docstring")  GLDraw::destroySim "void destroySim(int index)
";

%feature("docstring")  GLDraw::MakeController "MyController*
MakeController(Robot *robot) ";

%feature("docstring")  GLDraw::MakeSensors "void MakeSensors(Robot
*robot, RobotSensors &sensors) ";

%feature("docstring")  GLDraw::GetMesh "void GetMesh(const
Geometry::AnyCollisionGeometry3D &geom, TriangleMesh &tmesh) ";

%feature("docstring")  GLDraw::GetMesh "void GetMesh(const
TriangleMesh &tmesh, Geometry::AnyCollisionGeometry3D &geom) ";

%feature("docstring")  GLDraw::GetPointCloud "void
GetPointCloud(const Geometry::AnyCollisionGeometry3D &geom, PointCloud
&pc) ";

%feature("docstring")  GLDraw::GetPointCloud "void
GetPointCloud(const PointCloud &pc, Geometry::AnyCollisionGeometry3D
&geom) ";

%feature("docstring")  GLDraw::copy "void copy(const Vector &vec,
vector< double > &v) ";

%feature("docstring")  GLDraw::copy "void copy(const vector< double >
&vec, Vector &v) ";

%feature("docstring")  GLDraw::copy "void copy(const Matrix &mat,
vector< double > &v) ";

%feature("docstring")  GLDraw::copy "void copy(const Matrix &mat,
vector< vector< double > > &v) ";


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

%feature("docstring")  PyListFromVector "PyObject*
PyListFromVector(const Vector &x) ";

%feature("docstring")  findRoots "PyObject* findRoots(PyObject
*startVals, int iter)

Performs unconstrained root finding for up to iter iterations Return
values is a tuple indicating (0,x,n) : convergence reached in x
(1,x,n) : convergence reached in f (2,x,n) : divergence (3,x,n) :
degeneration of gradient (local extremum or saddle point) (4,x,n) :
maximum iterations reached (5,x,n) : numerical error occurred where x
is the final point and n is the number of iterations used ";

%feature("docstring")  findRootsBounded "PyObject*
findRootsBounded(PyObject *startVals, PyObject *boundVals, int iter)

Same as findRoots, but with given bounds (xmin,xmax) ";

%feature("docstring")  destroy "void destroy()

Frees all memory allocated by the collide module. All existing
geometry ids and collision query ids are invalidated.

Performs cleanup of all created spaces and planners.

destroys internal data structures ";


// File: rootfind_8h.xml
%feature("docstring")  setFTolerance "void setFTolerance(double tolf)

Sets the termination threshold for the change in f. ";

%feature("docstring")  setXTolerance "void setXTolerance(double tolx)

Sets the termination threshold for the change in x. ";

%feature("docstring")  setVectorField "int setVectorField(PyObject
*pVFObj)

Sets the vector field object, returns 0 if pVFObj = NULL, 1 otherwise.
";

%feature("docstring")  findRoots "PyObject* findRoots(PyObject
*startVals, int iter)

Performs unconstrained root finding for up to iter iterations Return
values is a tuple indicating (0,x,n) : convergence reached in x
(1,x,n) : convergence reached in f (2,x,n) : divergence (3,x,n) :
degeneration of gradient (local extremum or saddle point) (4,x,n) :
maximum iterations reached (5,x,n) : numerical error occurred where x
is the final point and n is the number of iterations used ";

%feature("docstring")  findRootsBounded "PyObject*
findRootsBounded(PyObject *startVals, PyObject *boundVals, int iter)

Same as findRoots, but with given bounds (xmin,xmax) ";

%feature("docstring")  destroy "void destroy()

destroys internal data structures ";

