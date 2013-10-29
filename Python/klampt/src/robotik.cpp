#include <string>
#include <vector>
#include "robotik.h"
#include <robotics/IKFunctions.h>
#include <math/random.h>
#include <math3d/random.h>
#include <Python.h>
#include "Modeling/World.h"
#include "pyerr.h"

//defined in robotsim.cpp
void copy(const Matrix& mat,vector<vector<double> >& v);

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
{
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
  goal.SetFixedPosition(p2);
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

int IKObjective::numPosDims() const
{
  return IKGoal::NumDims(goal.posConstraint);
}

int IKObjective::numRotDims() const
{
  return IKGoal::NumDims(goal.posConstraint);
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

GeneralizedIKObjective::GeneralizedIKObjective(const RobotModelLink& link,const RobotModelLink& link2)
  :link1(link1),link2(link2),isObj1(false),isObj2(false)
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
  goal.SetFixedPosition(p2);
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
  :robot(_robot),useJointLimits(true)
{}

IKSolver::IKSolver(const IKSolver& solver)
  :robot(solver.robot),objectives(solver.objectives),useJointLimits(solver.useJointLimits),activeDofs(solver.activeDofs)
{}

void IKSolver::add(const IKObjective& objective)
{
  objectives.push_back(objective);
}

void IKSolver::setActiveDofs(const std::vector<int>& active)
{
  activeDofs = active;
}

void IKSolver::getActiveDofs(std::vector<int>& out)
{
  if(activeDofs.empty()) {
    vector<IKGoal> goals(objectives.size());
    for(size_t i=0;i<objectives.size();i++)
      goals[i] = objectives[i].goal;
    ArrayMapping map;
    GetDefaultIKDofs(*robot.robot,goals,map);
    out = map.mapping;
  }
  else {
    out = activeDofs;
  }
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

void IKSolver::getJacobian(std::vector<std::vector<double> >& out)
{
  RobotIKFunction f(*robot.robot);
  vector<IKGoal> goals(objectives.size());
  for(size_t i=0;i<objectives.size();i++)
    goals[i] = objectives[i].goal;
  f.UseIK(goals);
  if(activeDofs.empty()) GetDefaultIKDofs(*robot.robot,goals,f.activeDofs);
  else f.activeDofs.mapping = activeDofs;

  Vector x(f.activeDofs.Size());
  Matrix J;
  f.GetState(x);
  J.resize(f.NumDimensions(),x.n);
  f.Jacobian(x,J);

  //copy to out
  copy(J,out);
}

PyObject* IKSolver::solve(int iters,double tol)
{
  RobotIKFunction f(*robot.robot);
  vector<IKGoal> goals(objectives.size());
  for(size_t i=0;i<objectives.size();i++)
    goals[i] = objectives[i].goal;
  f.UseIK(goals);
  if(activeDofs.empty()) GetDefaultIKDofs(*robot.robot,goals,f.activeDofs);
  else f.activeDofs.mapping = activeDofs;

  RobotIKSolver solver(f);
  if(useJointLimits) solver.UseJointLimits();
  solver.solver.verbose = 0;

  bool res = solver.Solve(tol,iters);
  PyObject* tuple = PyTuple_New(2);
  PyTuple_SetItem(tuple,0,PyBool_FromLong(res));
  PyTuple_SetItem(tuple,1,PyInt_FromLong(iters));
  return tuple;
}

void IKSolver::sampleInitial()
{
  for(int i=0;i<robot.robot->q.n;i++)
    robot.robot->q(i) = Rand(robot.robot->qMin(i),robot.robot->qMax(i));
  robot.robot->UpdateFrames();
}

GeneralizedIKSolver::GeneralizedIKSolver(const WorldModel& world)
  :world(world)
{}

void GeneralizedIKSolver::add(const GeneralizedIKObjective& objective)
{
  objectives.push_back(objective);
}

void GeneralizedIKSolver::getResidual(std::vector<double>& out)
{
  throw PyException("Not implemented yet");
}

void GeneralizedIKSolver::getJacobian(std::vector<std::vector<double> >& out)
{
  throw PyException("Not implemented yet");
}

PyObject* GeneralizedIKSolver::solve(int iters,double tol)
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

void SampleTransform(const IKObjective& obj,double out[9],double out2[3])
{
  RigidTransform T;
  SampleTransform(obj.goal,T);
  T.R.get(out);
  T.t.get(out2);
}

void SampleTransform(const GeneralizedIKObjective& obj,double out[9],double out2[3])
{
  RigidTransform T;
  SampleTransform(obj.goal,T);
  T.R.get(out);
  T.t.get(out2);
}
