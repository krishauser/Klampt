#if defined (__APPLE__) || defined (MACOSX)
  #include "mac_fixes.h"
#endif //Mac fixes 

#include <string>
#include <vector>
#include "robotik.h"
#include <KrisLibrary/robotics/IKFunctions.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math3d/random.h>
#include <Klampt/Planning/RobotCSpace.h>
#include <Klampt/Modeling/World.h>
#include <Klampt/Planning/RobotCSpace.h>
#include "pyerr.h"
#include "pyconvert.h"
using namespace std;

//defined in robotsim.cpp
inline void MakeNumpyArray(double** out,int* m,int* n,int _m,int _n,Matrix& ref)
{
  *m = _m;
  *n = _n;
  *out = (double*)malloc(_m*_n*sizeof(double));
  ref.setRef(*out,_m*_n,0,_n,1,_m,_n);
}

bool PySequence_ToVector3(PyObject* seq,Vector3& val)
{
  if(!PySequence_Check(seq)) return false;
  if(PySequence_Size(seq) != 3) return false;
  val.x = PyFloat_AsDouble(PySequence_GetItem(seq, 0));
  val.y = PyFloat_AsDouble(PySequence_GetItem(seq, 1));
  val.z = PyFloat_AsDouble(PySequence_GetItem(seq, 2));
  return true;
}

bool PySequence_ToVector3Array(PyObject* seq,vector<Vector3>& array)
{
  if(!PySequence_Check(seq)) return false;
  array.resize(PySequence_Size(seq));
  for(size_t i = 0; i < array.size(); i++) {
    if(!PySequence_ToVector3(PySequence_GetItem(seq, i),array[i])) return false;
  }
  return true;
}



IKObjective::IKObjective()
:positionScale(1),rotationScale(1)
{
}

IKObjective::IKObjective(const IKObjective& rhs)
:goal(rhs.goal),positionScale(rhs.positionScale),rotationScale(rhs.rotationScale)
{
}

IKObjective IKObjective::copy() const
{
  return IKObjective(*this);
}

int IKObjective::link() const
{
  return goal.link;
}

int IKObjective::destLink() const
{
  return goal.destLink;
}

void IKObjective::setFixedPoint(int link,const double plocal[3],const double pworld[3])
{
  setRelativePoint(link,-1,plocal,pworld);
}

void IKObjective::setFixedPoints(int link,PyObject* plocals,PyObject* pworlds)
{
  setRelativePoints(link,-1,plocals,pworlds);
}

void IKObjective::setFixedTransform(int link,const double R[9],const double t[3])
{
  setRelativeTransform(link,-1,R,t);
}

void IKObjective::setRelativePoint(int link1,int link2,const double p1[3],const double p2[3])
{
  goal.link = link1;
  goal.destLink = link2;
  goal.SetFreeRotation();
  goal.SetFixedPosition(Vector3(p2));
  goal.localPosition.set(p1);
}

void IKObjective::setRelativePoints(int link1,int link2,PyObject* p1s,PyObject* p2s)
{
  vector<Vector3> localPos,worldPos;
  if(!PySequence_ToVector3Array(p1s,localPos)) 
    throw PyException("Unable to convert local point array");
  if(!PySequence_ToVector3Array(p2s,worldPos)) 
    throw PyException("Unable to convert target point array");
  if(localPos.size() != worldPos.size())
    throw PyException("Point array size mismatch");

  goal.link = link1;
  goal.destLink = link2;
  goal.SetFromPoints(localPos,worldPos);
}

void IKObjective::setRelativeTransform(int link,int linkTgt,const double R[9],const double t[3])
{
  goal.link = link;
  goal.destLink = linkTgt;
  goal.localPosition.setZero();
  goal.SetFixedRotation(Matrix3(R));
  goal.SetFixedPosition(Vector3(t));
}

void IKObjective::setLinks(int link,int link2)
{
  goal.link = link;
  goal.destLink = link2;
}

void IKObjective::setFreePosition()
{
  printf("IKObjective::setFreePosition: deprecated, use setFreePosConstraint");
  goal.SetFreePosition();
}

void IKObjective::setFreePosConstraint()
{
  goal.SetFreePosition();
}

void IKObjective::setFixedPosConstraint(const double tlocal[3],const double tworld[3])
{
  goal.localPosition.set(tlocal);
  goal.SetFixedPosition(Vector3(tworld));
}

void IKObjective::setPlanarPosConstraint(const double tlocal[3],const double nworld[3],double oworld)
{
  goal.localPosition.set(tlocal);
  goal.SetPlanarPosition(Vector3(nworld)*oworld,Vector3(nworld));
}

void IKObjective::setLinearPosConstraint(const double tlocal[3],const double sworld[3],const double dworld[3])
{
  goal.localPosition.set(tlocal);
  goal.SetLinearPosition(Vector3(sworld),Vector3(dworld));
}

void IKObjective::setFreeRotConstraint()
{
  goal.SetFreeRotation();
}

void IKObjective::setFixedRotConstraint(const double R[9])
{
  goal.SetFixedRotation(Matrix3(R));
}

void IKObjective::setAxialRotConstraint(const double alocal[3],const double aworld[3])
{
  goal.SetAxisRotation(Vector3(alocal),Vector3(aworld));
}

int IKObjective::numPosDims() const
{
  return IKGoal::NumDims(goal.posConstraint);
}

int IKObjective::numRotDims() const
{
  return IKGoal::NumDims(goal.rotConstraint);
}

void IKObjective::getPosition(double out[3],double out2[3]) const
{
  goal.localPosition.get(out);
  goal.endPosition.get(out2);
}

void IKObjective::getPositionDirection(double out[3]) const
{
  goal.direction.get(out);
}

void IKObjective::getRotation(double out[9]) const
{
  if(goal.rotConstraint == IKGoal::RotFixed) {
    Matrix3 R;
    goal.GetFixedGoalRotation(R);
    R.get(out);
  }
  else {
    PyException("getRotation called on non-fixed rotation");
  }
}

void IKObjective::getRotationAxis(double out[3],double out2[3]) const
{
  goal.localAxis.get(out);
  goal.endRotation.get(out2);
}

void IKObjective::getTransform(double out[9],double out2[3]) const
{
  if(goal.posConstraint == IKGoal::PosFixed && goal.rotConstraint == IKGoal::RotFixed) {
    RigidTransform T;
    goal.GetFixedGoalTransform(T);
    T.R.get(out);
    T.t.get(out2);
  }
  else {
    PyException("getTransform called on non-fixed transform");
  }
}
void IKObjective::transform(const double R[9],const double t[3])
{
  RigidTransform T;
  T.R = Matrix3(R);
  T.t = Vector3(t);
  goal.Transform(T);
}

void IKObjective::transformLocal(const double R[9],const double t[3]) 
{
  RigidTransform T;
  T.R = Matrix3(R);
  T.t = Vector3(t);
  goal.TransformLocal(T);
}

void IKObjective::matchDestination(const double R[9],const double t[3])
{
  RigidTransform T;
  T.R = Matrix3(R);
  T.t = Vector3(t);
  goal.MatchGoalTransform(T);
}

void IKObjective::closestMatch(const double R[9],const double t[3],double out[9],double out2[3]) const
{
  RigidTransform T,Tout;
  T.R = Matrix3(R);
  T.t = Vector3(t);
  goal.GetClosestGoalTransform(T,Tout);
  Tout.R.get(out);
  Tout.t.get(out2);
}

bool IKObjective::loadString(const char* str)
{
  stringstream ss(str);
  ss>>goal;
  if(ss) return true;
  return false;
}

std::string IKObjective::saveString() const
{
  stringstream ss;
  ss<<goal;
  return ss.str();
}


GeneralizedIKObjective::GeneralizedIKObjective(const GeneralizedIKObjective& obj)
  :link1(obj.link1),link2(obj.link2),obj1(obj.obj1),obj2(obj.obj2),
   isObj1(obj.isObj1),isObj2(obj.isObj2),goal(obj.goal)
{
}

GeneralizedIKObjective::GeneralizedIKObjective(const RobotModelLink& link)
  :link1(link),isObj1(false),isObj2(false)
{
}

GeneralizedIKObjective::GeneralizedIKObjective(const RigidObjectModel& obj)
  :obj1(obj),isObj1(true),isObj2(false)
{
}

GeneralizedIKObjective::GeneralizedIKObjective(const RobotModelLink& _link,const RobotModelLink& _link2)
  :link1(_link),link2(_link2),isObj1(false),isObj2(false)
{}

GeneralizedIKObjective::GeneralizedIKObjective(const RobotModelLink& link,const RigidObjectModel& obj)
  :link1(link),obj2(obj),isObj1(false),isObj2(true)
{}

GeneralizedIKObjective::GeneralizedIKObjective(const RigidObjectModel& obj,const RobotModelLink& link)
  :link2(link),obj1(obj),isObj1(true),isObj2(false)
{
}

GeneralizedIKObjective::GeneralizedIKObjective(const RigidObjectModel& o1,const RigidObjectModel& o2)
  :obj1(o1),obj2(o2),isObj1(true),isObj2(true)
{
}

void GeneralizedIKObjective::setPoint(const double p1[3],const double p2[3])
{
  goal.localPosition.set(p1);
  goal.SetFixedPosition(Vector3(p2));
}

void GeneralizedIKObjective::setPoints(PyObject* p1s,PyObject* p2s)
{
  vector<Vector3> localPos,worldPos;
  if(!PySequence_ToVector3Array(p1s,localPos)) 
    throw PyException("Unable to convert local point array");
  if(!PySequence_ToVector3Array(p2s,worldPos)) 
    throw PyException("Unable to convert target point array");
  if(localPos.size() != worldPos.size())
    throw PyException("Point array size mismatch");

  goal.SetFromPoints(localPos,worldPos);
}

void GeneralizedIKObjective::setTransform(const double R[9],const double t[3])
{
  goal.localPosition.setZero();
  goal.SetFixedPosition(Vector3(t));
  goal.SetFixedRotation(Matrix3(R));
}


IKSolver::IKSolver(const RobotModel& _robot)
  :robot(_robot),tol(1e-3),maxIters(100),useJointLimits(true),lastIters(0)
{}

IKSolver::IKSolver(const IKSolver& solver)
  :robot(solver.robot),objectives(solver.objectives),secondary_objectives(solver.secondary_objectives),
  tol(solver.tol),maxIters(solver.maxIters),activeDofs(solver.activeDofs),useJointLimits(solver.useJointLimits),qmin(solver.qmin),qmax(solver.qmax),lastIters(solver.lastIters)
{}


IKSolver IKSolver::copy() const
{
  return IKSolver(*this);
}

void IKSolver::add(const IKObjective& objective)
{
  objectives.push_back(objective);
}

void IKSolver::set(int i,const IKObjective& objective)
{
  if(i < 0 || i >= (int)objectives.size()) throw PyException("Invalid index specified in set");
  objectives[i] = objective;
}

void IKSolver::addSecondary(const IKObjective& objective)
{
  secondary_objectives.push_back(objective);
}
  
void IKSolver::setSecondary(int i,const IKObjective& objective)
{
  if(i < 0 || i >= (int)secondary_objectives.size()) throw PyException("Invalid index specified in setSecondary");
  objectives[i] = objective;
}

void IKSolver::clear()
{
  objectives.resize(0);
  secondary_objectives.resize(0);
}

void IKSolver::setMaxIters(int iters)
{
  maxIters = iters;
}

int IKSolver::getMaxIters()
{
  return maxIters;
}

void IKSolver::setTolerance(double res)
{
  tol = res;
}

double IKSolver::getTolerance()
{
  return tol;
}

void IKSolver::setActiveDofs(const std::vector<int>& active)
{
  activeDofs = active;
}

void IKSolver::getActiveDofs(std::vector<int>& out)
{
  if(activeDofs.empty()) {
    vector<IKGoal> goals(objectives.size() + secondary_objectives.size());
    for(size_t i=0;i<objectives.size();i++)
      goals[i] = objectives[i].goal;
    for(size_t i=0;i<secondary_objectives.size();i++)
      goals[i+objectives.size()] = secondary_objectives[i].goal;
    ArrayMapping map;
    GetDefaultIKDofs(*robot.robot,goals,map);
    out = map.mapping;
  }
  else {
    out = activeDofs;
  }
}

void IKSolver::setJointLimits(const std::vector<double>& _qmin,const std::vector<double>& _qmax)
{
  if(_qmin.empty()) {
    useJointLimits=false;
    qmin.resize(0);
    qmax.resize(0);
  }
  else {
    qmin = Vector(_qmin);
    qmax = Vector(_qmax);
    useJointLimits = true;
  }
}

void IKSolver::getJointLimits(std::vector<double>& out,std::vector<double>& out2)
{
  if(!useJointLimits) {
    out.resize(0);
    out2.resize(0);
  }
  else {
    if(qmin.empty()) {
      robot.getJointLimits(out,out2);
    }
    else {
      out = qmin;
      out2 = qmax;
    }
  }
}

void IKSolver::setBiasConfig(const std::vector<double>& _biasConfig)
{
  biasConfig = _biasConfig;
}

void IKSolver::getBiasConfig(std::vector<double>& out)
{
  out = biasConfig;
}

bool IKSolver::isSolved()
{
  std::vector<double> res,qmin,qmax;
  getResidual(res);
  for(size_t i=0;i<res.size();i++)
    if(Abs(res[i]) > tol) return false;
  getJointLimits(qmin,qmax);
  for(size_t i=0;i<qmin.size();i++)
    if(robot.robot->q(i) < qmin[i] || robot.robot->q(i) > qmax[i]) return false;
  return true;
}

void IKSolver::getResidual(std::vector<double>& out)
{
  int size = 0;
  for(size_t i=0;i<objectives.size();i++) {
    int m=IKGoal::NumDims(objectives[i].goal.posConstraint);
    int n=IKGoal::NumDims(objectives[i].goal.rotConstraint);
    size += m + n;
  }
  out.resize(size);
  size = 0;
  for(size_t i=0;i<objectives.size();i++) {
    const IKGoal& goal = objectives[i].goal;
    Assert(goal.link >= 0 && goal.link < robot.robot->q.n);
    Assert(goal.destLink >= -1 && goal.destLink < robot.robot->q.n);
    int m=IKGoal::NumDims(goal.posConstraint);
    int n=IKGoal::NumDims(goal.rotConstraint);
    Real poserr[3],orierr[3];
    if(goal.destLink < 0)
      goal.GetError(robot.robot->links[goal.link].T_World,poserr,orierr);
    else {
      RigidTransform Trel;
      Trel.mulInverseB(robot.robot->links[goal.link].T_World,robot.robot->links[goal.destLink].T_World);
      goal.GetError(Trel,poserr,orierr);
    }
    for(int k=0;k<m;k++,size++)
      out[size] = poserr[k];
    for(int k=0;k<n;k++,size++)
      out[size] = orierr[k];
  }
}

void IKSolver::getSecondaryResidual(std::vector<double>& out)
{
  int size = 0;
  for(size_t i=0;i<secondary_objectives.size();i++) {
    int m=IKGoal::NumDims(secondary_objectives[i].goal.posConstraint);
    int n=IKGoal::NumDims(secondary_objectives[i].goal.rotConstraint);
    size += m + n;
  }
  out.resize(size);
  size = 0;
  for(size_t i=0;i<secondary_objectives.size();i++) {
    const IKGoal& goal = secondary_objectives[i].goal;
    Assert(goal.link >= 0 && goal.link < robot.robot->q.n);
    Assert(goal.destLink >= -1 && goal.destLink < robot.robot->q.n);
    int m=IKGoal::NumDims(goal.posConstraint);
    int n=IKGoal::NumDims(goal.rotConstraint);
    Real poserr[3],orierr[3];
    if(goal.destLink < 0)
      goal.GetError(robot.robot->links[goal.link].T_World,poserr,orierr);
    else {
      RigidTransform Trel;
      Trel.mulInverseB(robot.robot->links[goal.link].T_World,robot.robot->links[goal.destLink].T_World);
      goal.GetError(Trel,poserr,orierr);
    }
    for(int k=0;k<m;k++,size++)
      out[size] = poserr[k];
    for(int k=0;k<n;k++,size++)
      out[size] = orierr[k];
  }
}

void IKSolver::getJacobian(double** out,int* m,int* n)
{
  RobotIKFunction f(*robot.robot);
  vector<IKGoal> goals(objectives.size());
  for(size_t i=0;i<objectives.size();i++) {
    const IKGoal& goal = objectives[i].goal;
    Assert(goal.link >= 0 && goal.link < robot.robot->q.n);
    Assert(goal.destLink >= -1 && goal.destLink < robot.robot->q.n);
    goals[i] = goal;
  }
  f.UseIK(goals);
  if(activeDofs.empty()) GetDefaultIKDofs(*robot.robot,goals,f.activeDofs);
  else f.activeDofs.mapping = activeDofs;

  Vector x(f.activeDofs.Size());
  f.GetState(x);
  Matrix J;
  MakeNumpyArray(out,m,n,f.NumDimensions(),x.n,J);
  f.Jacobian(x,J);
}

/* //old style
PyObject* IKSolver::solve(int iters,double tol)
{
  static bool warned=false;
  if(!warned) {
    printf("IKSolver.solve(iters,tol) will be deprecated, use setMaxIters(iters)/setTolerance(tol) and solve() instead\n");
    warned=true;
  }
  if(useJointLimits) {
    getJointLimits(qmin,qmax);
    for(size_t i=0;i<qmin.size();i++) {
      if(robot.robot->q(i) < qmin[i] || robot.robot->q(i) > qmax[i]) {
        if(robot.robot->q(i) < qmin[i]-Epsilon || robot.robot->q(i) > qmax[i]+Epsilon) 
          printf("Joint limits %f < %f <%f exceeded on joint %i. Clamping to limit...\n", qmin[i],robot.robot->q(i),qmax[i],i);
        if(robot.robot->q(i) < qmin[i]) {
          robot.robot->q(i) = qmin[i];
        } else {
          robot.robot->q(i) = qmax[i];
        }
      }
    }
  }
  RobotIKFunction f(*robot.robot);
  vector<IKGoal> goals(objectives.size());
  for(size_t i=0;i<objectives.size();i++)
    goals[i] = objectives[i].goal;
  f.UseIK(goals);
  for(size_t i=0;i<objectives.size();i++) {
    IKGoalFunction* obji = dynamic_cast<IKGoalFunction*>(f.functions[i].get());
    obji->positionScale = objectives[i].positionScale;
    obji->rotationScale = objectives[i].rotationScale;
  }
  if(activeDofs.empty()) GetDefaultIKDofs(*robot.robot,goals,f.activeDofs);
  else f.activeDofs.mapping = activeDofs;
  robot.robot->ConfigureDriverConstraints(f);

  RobotIKSolver solver(f);
  if(useJointLimits) {
    if(qmin.empty())
      solver.UseJointLimits();
    else {
      if(qmin.size() != robot.robot->links.size()) throw PyException("Invalid size on qmin");
      if(qmax.size() != robot.robot->links.size()) throw PyException("Invalid size on qmax");
      solver.UseJointLimits(Vector(qmin),Vector(qmax));
    }
  }
  solver.solver.verbose = 0;

  bool res = solver.Solve(tol,iters);
  robot.robot->UpdateGeometry();
  PyObject* tuple = PyTuple_New(2);
  PyTuple_SetItem(tuple,0,PyBool_FromLong(res));
  PyTuple_SetItem(tuple,1,PyInt_FromLong(iters));
  return tuple;
}
*/

void SetupSolver(IKSolver* s,RobotIKFunction& f,RobotIKSolver& solver,bool add_secondary=false)
{
  Klampt::RobotModel* robot=s->robot.robot;
  if(s->useJointLimits) {
    const Real* usedQmin = (s->qmin.empty() ? &robot->qMin[0] : &s->qmin[0]);
    const Real* usedQmax = (s->qmax.empty() ? &robot->qMax[0] : &s->qmax[0]);
    for(int i=0;i<robot->q.size();i++) {
      if(robot->q(i) < usedQmin[i] || robot->q(i) > usedQmax[i]) {
        if(robot->q(i) < usedQmin[i]-Epsilon || robot->q(i) > usedQmax[i] + Epsilon) 
          printf("IKSolver:: Joint limits on joint %i exceeded: %g <= %g <= %g. Clamping to limits...\n", i,usedQmin[i],robot->q(i),usedQmax[i]);
        robot->q(i) = Clamp(robot->q(i),usedQmin[i],usedQmax[i]);
      }
    }
  }
  vector<const IKGoal*> goals(s->objectives.size());  // function stores *reference* to goal
  for(size_t i=0;i<s->objectives.size();i++) {
    goals[i] = &s->objectives[i].goal;
  }
  if(add_secondary) {
    goals.resize(s->objectives.size()+s->secondary_objectives.size());
    for(size_t i=0;i<s->secondary_objectives.size();i++)
      goals[i+s->objectives.size()] = &s->secondary_objectives[i].goal;
  }
  for(size_t i=0;i<goals.size();i++) {
    if(goals[i]->link < 0 || goals[i]->link >= robot->q.n) throw PyException("Invalid IKObjective link");
    if(goals[i]->destLink < -1 || goals[i]->destLink >= robot->q.n) throw PyException("Invalid IKObjective destination link");
    f.UseIK(*goals[i]);
  }

  for(size_t i=0;i<s->objectives.size();i++) {
    IKGoalFunction* obji = dynamic_cast<IKGoalFunction*>(f.functions[i].get());
    obji->positionScale = s->objectives[i].positionScale;
    obji->rotationScale = s->objectives[i].rotationScale;
  }
  if(add_secondary) {
    for(size_t i=0;i<s->secondary_objectives.size();i++) {
      IKGoalFunction* obji = dynamic_cast<IKGoalFunction*>(f.functions[i+s->objectives.size()].get());
      obji->positionScale = s->secondary_objectives[i].positionScale;
      obji->rotationScale = s->secondary_objectives[i].rotationScale;
    }
  }
  if(s->activeDofs.empty()) {
    vector<IKGoal> goalCopy(goals.size());
    for(size_t i=0;i<goals.size();i++)
      goalCopy[i] = *goals[i];
    GetDefaultIKDofs(*robot,goalCopy,f.activeDofs);
  }
  else f.activeDofs.mapping = s->activeDofs;
  robot->ConfigureDriverConstraints(f);

  if(s->useJointLimits) {
    if(s->qmin.empty())
      solver.UseJointLimits();
    else {
      if(s->qmin.size() != robot->links.size()) throw PyException("Invalid size on qmin");
      if(s->qmax.size() != robot->links.size()) throw PyException("Invalid size on qmax");
      solver.UseJointLimits(Vector(s->qmin),Vector(s->qmax));
    }
  }
  if(!s->biasConfig.empty()) {
    if(s->biasConfig.size() != robot->links.size()) throw PyException("Invalid size on biasConfig");
    solver.UseBiasConfiguration(Vector(s->biasConfig));
  }
  solver.solver.verbose = 0;
}

bool IKSolver::solve()
{
  RobotIKFunction f(*robot.robot);
  RobotIKSolver solver(f);
  SetupSolver(this,f,solver,true);

  int iters=maxIters;
  bool res = solver.Solve(tol,iters);
  robot.robot->UpdateGeometry();
  lastIters = iters;
  return res;
}

bool IKSolver::minimize()
{
  RobotIKFunction f(*robot.robot);
  RobotIKSolver solver(f);
  SetupSolver(this,f,solver);

  int iters=maxIters;
  bool res;
  if(secondary_objectives.empty())
    res = solver.MinimizeResidual(tol,tol*0.01,iters);
  else {
    RobotIKFunction fsecondary(*robot.robot);
    vector<IKGoal> goals(secondary_objectives.size());
    for(size_t i=0;i<secondary_objectives.size();i++)
      goals[i] = secondary_objectives[i].goal;
    fsecondary.UseIK(goals);
    fsecondary.activeDofs = f.activeDofs;
    res = solver.PrioritizedSolve(fsecondary,tol,tol*0.01,iters);
  }
  robot.robot->UpdateGeometry();
  lastIters = iters;
  return res;
}

PyObject* PyTupleFromVector(const Vector& x)
{
  PyObject* ls = PyTuple_New(x.n);
  PyObject* pItem;
  if(ls == NULL) {
    goto fail;
  }
	
  for(int i = 0; i < x.n; i++) {
    pItem = PyFloat_FromDouble(x[i]);
    if(pItem == NULL)
      goto fail;
    PyTuple_SetItem(ls, (Py_ssize_t)i, pItem);
  }
  
  return ls;
  
 fail:
  Py_XDECREF(ls);
  return NULL;
}
class PyScalarFieldFunction : public ScalarFieldFunction {
	private:
    Vector qref;
    vector<int> activeDofs;
		PyObject *pFunc, *pGrad;
		PyObject *pXTemp;
		
	public:
		PyScalarFieldFunction(const Vector& _qref,const vector<int>& _activeDofs,PyObject* _pFunc,PyObject* _pGrad)
      :qref(_qref),activeDofs(_activeDofs),pFunc(_pFunc),pGrad(_pGrad) {
      Py_INCREF(_pFunc);
      Py_INCREF(_pGrad);
    }
		~PyScalarFieldFunction() {
      Py_DECREF(pFunc);
      Py_DECREF(pGrad);
      Py_XDECREF(pXTemp);
    }
		
		string Label() const { return "Scalar Field Function interface to Python"; }
    void PreEval(const Vector& x) {
      if(x.n != (int)activeDofs.size()) {
        throw PyException("Uh... PreEval got a wrong sized vector?");
      }
      Vector xfull(qref);
      for(size_t i=0;i<activeDofs.size();i++)
        xfull[activeDofs[i]] = x[i];

      Py_XDECREF(pXTemp);
      pXTemp = PyTupleFromVector(xfull);	
      if(pXTemp == NULL) {
        if(!PyErr_Occurred()) {
          throw PyException("PyScalarFieldFunction::PreEval: Couldn't "	\
          "build variable-value tuple.");
        }
      }
    }
		Real Eval(const Vector& x) {
      Assert(pXTemp != NULL);
      PyObject* pResult = PyObject_CallFunctionObjArgs(pFunc, pXTemp, NULL);
      
      // Get result as double
      if(PyFloat_Check(pResult) || PyLong_Check(pResult)) {
        Real result = PyFloat_AsDouble(pResult);
        assert(!PyErr_Occurred());
        Py_DECREF(pResult);
        return result;
      }
      Py_DECREF(pResult);
      throw PyException("PyScalarFieldFunction::Eval: returned an invalid object.");
      return 0.0;
    }

		void Gradient(const Vector& x, Vector& g) {
      if(x.n != (int)activeDofs.size()) {
        throw PyException("Uh... Gradient got a wrong sized vector?");
      }
      g.resize(x.n);
      Vector gfull(qref.n,0.0);
      Assert(pXTemp != NULL);
      PyObject* pResult = PyObject_CallFunctionObjArgs(pGrad, pXTemp, NULL);

      if(PySequence_Check(pResult)) {
        if(PySequence_Size(pResult) != (Py_ssize_t)x.n) {
          Py_DECREF(pResult);
          throw PyException("PyScalarFieldFunction::Gradient: returned a list of incorrect size.");
        }
        if(!PyListToConfig(pResult,gfull)) {
          Py_DECREF(pResult);
          throw PyException("PyScalarFieldFunction::Gradient: could not convert result to a vector.");
        }
        for(size_t i=0;i<activeDofs.size();i++)
          g[i] = gfull[activeDofs[i]];
        return;
      }
      Py_DECREF(pResult);
      throw PyException("PyScalarFieldFunction::Gradient: returned an invalid object.");
    }
};


bool IKSolver::minimize(PyObject* secondary_objective,PyObject* secondary_objective_grad)
{
  vector<int> activeDofs;
  getActiveDofs(activeDofs);
  PyScalarFieldFunction fsecondary(robot.robot->q,activeDofs,secondary_objective,secondary_objective_grad);
  RobotIKFunction f(*robot.robot);
  RobotIKSolver solver(f);
  SetupSolver(this,f,solver);

  int iters=maxIters;
  bool res = solver.PrioritizedSolve(fsecondary,tol,tol*0.01,iters);
  robot.robot->UpdateGeometry();
  lastIters = iters;
  return res;
}

int IKSolver::lastSolveIters()
{
  return lastIters;
}

void IKSolver::sampleInitial()
{
  vector<int> active;
  getActiveDofs(active);
  if(qmin.empty()) {
    //this method correctly updates non-normal joints and handles infinite bounds
    Config qorig = robot.robot->q;
    Klampt::RobotCSpace space(*robot.robot);
    space.Sample(robot.robot->q);
    swap(robot.robot->q,qorig);
    for(size_t i=0;i<active.size();i++)
      robot.robot->q(active[i]) = qorig(active[i]);
    //for(size_t i=0;i<active.size();i++)
    //  robot.robot->q(active[i]) = Rand(robot.robot->qMin(active[i]),robot.robot->qMax(active[i]));
  }
  else {
    for(size_t i=0;i<active.size();i++)
      robot.robot->q(active[i]) = Rand(qmin[active[i]],qmax[active[i]]);
  }
  robot.robot->UpdateFrames();
}


GeneralizedIKSolver::GeneralizedIKSolver(const WorldModel& world)
  :world(world)
{}

void GeneralizedIKSolver::add(const GeneralizedIKObjective& objective)
{
  objectives.push_back(objective);
}

void GeneralizedIKSolver::setMaxIters(int iters)
{
  maxIters = iters;
}

void GeneralizedIKSolver::setTolerance(double res)
{
  tol = res;
}

void GeneralizedIKSolver::getResidual(std::vector<double>& out)
{
  throw PyException("Not implemented yet");
}

void GeneralizedIKSolver::getJacobian(double** out,int* m, int* n)
{
  throw PyException("Not implemented yet");
}

PyObject* GeneralizedIKSolver::solve()
{
  throw PyException("Not implemented yet");
  return NULL;
}

void GeneralizedIKSolver::sampleInitial()
{
  throw PyException("Not implemented yet");
}




void SampleTransform(const IKGoal& goal,RigidTransform& T)
{
  assert(goal.posConstraint == IKGoal::PosFixed);
  if(goal.rotConstraint == IKGoal::RotFixed) {
    goal.GetFixedGoalTransform(T);
  }
  else if (goal.rotConstraint == IKGoal::RotAxis) {
    goal.GetEdgeGoalTransform(Rand(0,TwoPi),T);
  }
  else {
    QuaternionRotation q=RandRotation();
    q.getMatrix(T.R);
    T.t = goal.endPosition - T.R*goal.localPosition;
  }
}

void IKObjective::sampleTransform(double out[9],double out2[3]) const
{
  RigidTransform T;
  SampleTransform(goal,T);
  T.R.get(out);
  T.t.get(out2);
}

void GeneralizedIKObjective::sampleTransform(double out[9],double out2[3]) const
{
  RigidTransform T;
  SampleTransform(goal,T);
  T.R.get(out);
  T.t.get(out2);
}
