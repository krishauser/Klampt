#include "TabulatedController.h"
#include "Sensing/JointSensors.h"
#include <KrisLibrary/robotics/NewtonEuler.h>
#include <KrisLibrary/math/sparsematrix.h>
#include <KrisLibrary/math/misc.h>
#include <KrisLibrary/math/angle.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/optimization/LSQRInterface.h>
#include <fstream>

namespace Klampt {

TabulatedController::TabulatedController(RobotModel& robot)
  :RobotController(robot),torqueMode(true),commands(0)
{}

void TabulatedController::StateToFeature(const Config& q,const Vector& dq,Vector& x) const
{
  x.resize(q.n + dq.n);
  x.copySubVector(0,q);
  for(int i=0;i<q.n;i++)
    if(robot.joints[i].type == RobotModelJoint::Spin) 
      x(i) = AngleNormalize(x(i));
  x.copySubVector(q.n,dq);
}

void TabulatedController::FeatureToState(const Vector& x,Config& q,Vector& dq) const
{
  q.resize(x.n/2);
  dq.resize(x.n/2);
  x.getSubVectorCopy(0,q);
  x.getSubVectorCopy(q.n,dq);
}

void TabulatedController::Update(Real dt)
{
  Assert(command != NULL);
  Assert(sensors != NULL);

  Vector v;
  StateToFeature(sensors->GetTypedSensor<JointPositionSensor>()->q,sensors->GetTypedSensor<JointVelocitySensor>()->dq,v);
    
  Assert((int)robot.links.size()*2==commands.grid.h.n);
  Config u;
  Geometry::Grid::Index index;
  commands.grid.PointToIndex(v,index);
  for(size_t i=0;i<index.size();i++) {
    if(index[i] < commands.imin[i]) {
      printf("Index %d=%d out of lower bound: %g\n",i,index[i],v[i]);
      index[i] = commands.imin[i];
    }
    if(index[i] > commands.imax[i]) {
      printf("Index %d=%d out of upper bound: %g\n",i,index[i],v[i]);
      index[i] = commands.imax[i];  
    }    
  }
  u = commands[index];
  Assert(u.n == (int)robot.drivers.size());
    
  for(size_t i=0;i<robot.drivers.size();i++) {
    if(robot.drivers[i].type == RobotModelDriver::Normal) {
      if(torqueMode) 
	command->actuators[i].SetTorque(u(i));
      else
	command->actuators[i].SetPID(u(i),0,command->actuators[i].iterm);
    }
  }
  RobotController::Update(dt);
}

bool TabulatedController::Load(istream& in)
{
  in>>commands.grid.h;
  if(!in) return false;
  commands.imin.resize(commands.grid.h.n);
  commands.imax.resize(commands.grid.h.n);
  for(size_t i=0;i<commands.imin.size();i++)
    in>>commands.imin[i];
  for(size_t i=0;i<commands.imax.size();i++)
    in>>commands.imax[i];
  if(!in) return false;
  commands.Init(commands.imin,commands.imax);
  for(size_t i=0;i<commands.values.size();i++) {
    in>>commands.values[i];
    if(!in) return false;
  }
  return true;
}

bool TabulatedController::Save(ostream& out)
{
  out<<commands.grid.h<<endl;
  for(size_t i=0;i<commands.imin.size();i++)
    out<<commands.imin[i]<<" ";
  out<<endl;
  for(size_t i=0;i<commands.imax.size();i++)
    out<<commands.imax[i]<<" ";
  out<<endl;
  for(size_t i=0;i<commands.values.size();i++)
    out<<commands.values[i]<<endl;
  return true;
}

Real timeStep = 0.01;

//upon integrating from q, how long does the state stay in the cell centered
//at center?  Increments to the next index
Real NextCell(RobotModel& robot,const Geometry::GridTable<Vector>& commands,
	      IntTuple& index,const Vector& center,const Config& q,const Vector& dq,const Vector& ddq)
{
  Config newq,newdq;
  newdq = dq+ddq*timeStep;
  newq = q + dq*timeStep + ddq*(0.5*Sqr(timeStep));
  for(int i=0;i<q.n;i++)
    if(robot.joints[i].type == RobotModelJoint::Spin) 
      newq(i) = AngleNormalize(newq(i));

  Config f(newq.n+newdq.n);
  f.copySubVector(0,newq);
  f.copySubVector(newq.n,newdq);
  commands.grid.PointToIndex(f,index);

  for(size_t i=0;i<index.size();i++) {
    if(index[i] > commands.imax[i]) {
      if((int)i < q.n && robot.joints[i].type == RobotModelJoint::Spin) 
	index[i]=commands.imin[i];
      else
	index[i]=commands.imax[i];
    }
    else if(index[i] < commands.imin[i]) {
      if((int)i < q.n && robot.joints[i].type == RobotModelJoint::Spin)
	index[i] = commands.imax[i];
      else
	index[i] = commands.imin[i];
    }
  }
  return timeStep;

  int exit = 0;
  Real texit = Inf;
  int dqofs = q.n;
  for(int i=0;i<dq.n;i++) {
    if(dq(i)+ddq(i)*texit > center(i+dqofs)+commands.grid.h(i+dqofs)*0.5) {
      texit = (commands.grid.h(i+dqofs)*0.5+center(i+dqofs)-dq(i))/ddq(i);
      exit = dqofs+i+1;
    }
    else if(dq(i)+ddq(i)*texit < center(i+dqofs)-commands.grid.h(i+dqofs)*0.5) {
      texit = (center(i+dqofs)-dq(i)-commands.grid.h(i+dqofs)*0.5)/ddq(i);
      exit = -(dqofs+i+1);
    }
  }
  for(int i=0;i<q.n;i++) {
    //q + t*dq + t^2*ddq/2 = c+h/2 or c-h/2
    Real a=0.5*ddq(i);
    Real b=robot.dq(i);
    Real c=q(i)-center(i)+commands.grid.h(i)*0.5;
    Real t1,t2;
    int res=quadratic(a,b,c,t1,t2);
    if(res > 1 && t2 < 0) { res--; }
    if(res > 0 && t1 < 0) { t1=t2; res--; }
    if(res >= 2 && t1 > t2) Swap(t1,t2);
    if(res >= 1 && t1 < texit) {
      texit = t1;
      exit = -(i+1);
    }
    c=q(i)-center(i)-commands.grid.h(i)*0.5;
    res=quadratic(a,b,c,t1,t2);
    if(res > 1 && t2 < 0) { res--; }
    if(res > 0 && t1 < 0) { t1=t2; res--; }
    if(res >= 2 && t1 > t2) Swap(t1,t2);
    if(res >= 1 && t1 < texit) {
      texit = t1;
      exit = i+1;
    }
  }
  
  //determine next index
  if(exit > 0) {
    int i=exit-1;
    index[i]++;
    if(index[i] > commands.imax[i]) {
      if(i < dqofs && robot.joints[i].type == RobotModelJoint::Spin) 
	index[i]=commands.imin[i];
      else
	index[i]=commands.imax[i];
    }
  }
  else if(exit < 0) {
    int i=-exit-1;
    index[i]--;
    if(index[i] < commands.imin[i]) {
      if(i < dqofs && robot.joints[i].type == RobotModelJoint::Spin) 
	index[i] = commands.imax[i];
      else
	index[i] = commands.imin[i];
    }
  }
  return texit;
}

void OptimizeMDP(TabulatedController& controller,
		 const Config& qdes,const Vector& w,
		 int numTransitionSamples,Real discount)
{
  RobotModel& robot=controller.robot;
  Geometry::GridTable<Vector>& commands=controller.commands;
  //set up an MDP on the grid
  int n=(int)commands.values.size();
  vector<Vector> actions;
  Vector a(robot.drivers.size());
  //add zero action
  a.setZero();
  actions.push_back(a);
  //enumerate all minima and maxima of torque space
  vector<int> maxs(robot.drivers.size(),2);
  //if an actuator is turned off, don't consider it separately
  for(size_t i=0;i<robot.drivers.size();i++) 
    if(robot.drivers[i].tmax == robot.drivers[i].tmin) maxs[i]=1;
  IntTuple index;
  index.resize(robot.drivers.size(),0);
  do {
    for(size_t i=0;i<robot.drivers.size();i++) {
      if(index[i]==0) a[i] = robot.drivers[i].tmin;
      else a[i] = robot.drivers[i].tmax;
    }
    actions.push_back(a);
  } while (IncrementIndex(index,maxs)==0);
  //sample some additional actions
  size_t na = 2*(actions.size()-1);
  for(size_t k=0;k<na;k++) {
    for(size_t i=0;i<robot.drivers.size();i++) 
      a[i] = Rand(robot.drivers[i].tmin,robot.drivers[i].tmax);
    actions.push_back(a);
  }
  printf("Constructing an MDP with %d states and %d actions\n",n,actions.size());
  
  vector<SparseMatrix> T(actions.size());
  vector<Vector> cost(actions.size());
  for(size_t a=0;a<actions.size();a++) {
    T[a].resize(n,n);
    cost[a].resize(n);
  }
  
  NewtonEulerSolver solver(robot);
  solver.SetGravityWrenches(Vector3(0,0,-9.8));
  Vector c,cmin,cmax;
  index = commands.imin;
  do {
    commands.grid.CellCenter(index,c);
    commands.grid.CellBounds(index,cmin,cmax);
    controller.FeatureToState(c,robot.q,robot.dq);
    robot.UpdateFrames();
    solver.SetGravityWrenches(Vector3(0,0,-9.8));
    Assert(robot.q.n == (int)robot.links.size());
    int elementIndex = commands.ElementIndex(index);
    //assess cost
    Real qcost=0;
    for(int i=0;i<robot.q.n;i++) {
      if(robot.joints[i].type == RobotModelJoint::Spin) 
	qcost += Sqr(AngleDiff(robot.q(i),qdes(i)))*w(i);
      else
	qcost += Sqr(robot.q(i) - qdes(i))*w(i);
    }
    qcost = Sqrt(qcost);
    
    //set the robot state
    for(size_t a=0;a<actions.size();a++) {
      //FeatureToState(c,robot.q,robot.dq);
      //Assert(robot.q.n == (int)robot.links.size());
      //robot.UpdateFrames();
      //solver.SetGravityWrenches(Vector3(0,0,-9.8));
      
      Vector torques(robot.links.size(),0.0),accels,Jd;
      //convert driver torques to link torques
      for(size_t j=0;j<robot.drivers.size();j++) {
	robot.GetDriverJacobian(j,Jd);
	torques.madd(Jd,actions[a][j]);
      }
      solver.CalcAccel(torques,accels);
      
      /*
	IntTuple nextIndex = index;
	Real timeexit = NextCell(robot,nextIndex,c,robot.q,robot.dq,accels);
	int nextElementIndex=commands.ElementIndex(nextIndex);
	T[a](elementIndex,nextElementIndex)=1;
	cost[a](elementIndex) = qcost*timeexit;
      */
      for(int sample=0;sample<numTransitionSamples;sample++) {
	//sample robot.q, robot.dq from cell
	for(int i=0;i<robot.q.n;i++)
	  robot.q(i) = Rand(cmin(i),cmax(i));
	for(int i=0;i<robot.q.n;i++)
	  robot.dq(i) = Rand(cmin(i+robot.q.n),cmax(i+robot.q.n));
	IntTuple nextIndex = index;
	Real timeexit = NextCell(robot,commands,nextIndex,c,robot.q,robot.dq,accels);
	int nextElementIndex=commands.ElementIndex(nextIndex);
	T[a](elementIndex,nextElementIndex)+=1.0/numTransitionSamples;
	//integral from o to texit of discount^t = e^(log(discount)t)
	//1/log(discount) (discount^texit - 1)
	Real scale = timeexit;
	if(discount < 1.0)
	  scale = (Pow(discount,timeexit)-1)/Log(discount);
	cost[a](elementIndex) += qcost*scale/numTransitionSamples;
      }
    }
  } while (IncrementIndex(index,commands.imin,commands.imax)==0);
  
  printf("Solving MDP with value iteration...\n");
  //solve for optimal actions via policy iteration
  Real tolerance = 1e-5;
  vector<int> policy(n,0);
  Vector values(n,0.0);
  Vector costp(n);
  SparseMatrix Tp(n,n);
  //I*V = -cp + disc*Tp*V
  //cp = (Tp-I/disc)*V
  for(int i=0;i<n;i++) {
    Tp.rows[i] = T[policy[i]].rows[i];
    Tp(i,i) -= 1.0/discount;
  }
  for(int i=0;i<n;i++)
    costp(i) = cost[policy[i]](i)/discount;

  Optimization::LSQRInterface lsqr;
  Vector r;
  int solveIters=0;
  bool changed=true;
  while(changed) {
    printf("Iteration %d\n",solveIters);
    solveIters++;
    
    //sparse solve Ax=b with A=Tp, b=costp
    int maxiters = n*10;
    lsqr.x0 = values;
    lsqr.maxIters = maxiters;
    lsqr.verbose = 0;
    lsqr.Solve(Tp,costp);
    swap(values,lsqr.x);
    Tp.mul(values,r);
    r -= costp;
    if(r.norm() > 1e-1) {
      cout<<"Quitting due to error in linear system solve?"<<endl;
      break;
    }

    //find better actions
    changed=false;
    int numchanged = 0;
    Real improvement=0;
    for(int i=0;i<n;i++) {
      Real best=values(i);
      Real vp = -cost[policy[i]](i) + discount*T[policy[i]].dotRow(i,values);
      Assert(FuzzyEquals(vp,best));
      bool ichanged=false;
      for(size_t a=0;a<actions.size();a++) {
	if((int)a == policy[i]) continue;
	Real va = -cost[a](i) + discount*T[a].dotRow(i,values);
	if(va > best) {
	  if(va > best + tolerance)
	    changed=true;
	  best = va;
	  policy[i] = a;
	  ichanged=true;
	}
      }
      if(ichanged) {
	numchanged++;
	improvement += best - values(i);

	//modify rows of Tp and costp
	Tp.rows[i] = T[policy[i]].rows[i];
	Tp(i,i) -= 1.0/discount;
	costp(i) = cost[policy[i]](i)/discount;
      }
    }
    printf("%d actions of policy changed, amount %g\n",numchanged,improvement);
  }
  printf("Done.  Saving values and actions to mdp.txt...\n");

  //read out the policy
  for(int i=0;i<n;i++)
    commands.values[i] = actions[policy[i]];

  index = commands.imin;
  ofstream out("mdp.txt",ios::out);
  do {
    int elementIndex = commands.ElementIndex(index);
    Vector c;
    commands.grid.CellCenter(index,c);
    out<<c<<"\t"<<values[elementIndex]<<"\t"<<actions[policy[elementIndex]]<<endl;
  } while (IncrementIndex(index,commands.imin,commands.imax)==0);
  out.close();
}

} //namespace Klampt