#include "ContactCSpace.h"
#include <robotics/IKFunctions.h>
#include <robotics/JointStructure.h>
#include <math3d/random.h>
#include <Timer.h>

#define TEST_NO_JOINT_LIMITS 0
#define DO_TIMING 1


ContactCSpace::ContactCSpace(RobotWorld& world,int index,
			     WorldPlannerSettings* settings)
  :SingleRobotCSpace2(world,index,settings),
   numSolveContact(0),numIsFeasible(0),solveContactTime(0),isFeasibleTime(0)
{}

ContactCSpace::ContactCSpace(const SingleRobotCSpace& space)
  :SingleRobotCSpace2(space),
   numSolveContact(0),numIsFeasible(0),solveContactTime(0),isFeasibleTime(0)
{}

ContactCSpace::ContactCSpace(const ContactCSpace& space)
  :SingleRobotCSpace2(space),contactIK(space.contactIK),
   numSolveContact(0),numIsFeasible(0),solveContactTime(0),isFeasibleTime(0)
{}

void ContactCSpace::Sample(Config& x)
{
  SingleRobotCSpace2::Sample(x);
  Robot* robot = GetRobot();
  bool floating = false;
  for(size_t i=0;i<robot->joints.size();i++)
    if(robot->joints[i].type == RobotJoint::Floating) {
      floating = true;
      break;
    }
  if(floating) {
    //need to solve for floating joint structure
    JointStructure js(*robot);
    js.Init();
    js.SolveWorkspaceBounds(contactIK);
    for(size_t i=0;i<robot->joints.size();i++)
      if(robot->joints[i].type == RobotJoint::Floating) {
	vector<int> indices;
	robot->GetJointIndices(i,indices);
	if(js.bounds[robot->joints[i].linkIndex].IsEmpty()) {
	  cout<<"Joint structure on joint "<<i<<" was empty"<<endl;
	  x[indices[0]] = x[indices[1]] = x[indices[2]] = 0;
	}
	else {
	  Sphere3D s;
	  js.bounds[robot->joints[i].linkIndex].GetBounds(s);
	  Vector3 v;
	  for(int iters = 0; iters < 10; iters++) {
	    SampleSphere(s.radius,v);
	    v += s.center;
	    if(js.bounds[robot->joints[i].linkIndex].Contains(v)) {
	      break;
	    }
	  }
	  v.get(x[indices[0]],x[indices[1]],x[indices[2]]);
	}
      } 
  }
  robot->UpdateConfig(x);
  SolveContact();
  x = robot->q;
}

void ContactCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  SingleRobotCSpace2::SampleNeighborhood(c,r,x);
  GetRobot()->UpdateConfig(x);
  SolveContact();
  x = GetRobot()->q;
}

bool ContactCSpace::IsFeasible(const Config& q)
{
  numIsFeasible++;

#if DO_TIMING
  Timer timer;
#endif // DO_TIMING

  GetRobot()->UpdateConfig(q);
  if(!CheckContact()) {
    printf("ContactCSpace:: Configuration fails distance check: %g > %g\n",ContactDistance(),settings->robotSettings[index].contactEpsilon*1.1);
#if DO_TIMING
    isFeasibleTime += timer.ElapsedTime();
#endif //DO_TIMING
    return false;
  }
  bool res = SingleRobotCSpace2::IsFeasible(q);
#if DO_TIMING
    isFeasibleTime += timer.ElapsedTime();
#endif //DO_TIMING
    return res;
}

EdgePlanner* ContactCSpace::LocalPlanner(const Config& a,const Config& b)
{
  return new BisectionEpsilonEdgePlanner(this,a,b,settings->robotSettings[index].collisionEpsilon);
}

void ContactCSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{
  SingleRobotCSpace2::Interpolate(x,y,u,out);
  GetRobot()->UpdateConfig(out);
  SolveContact();
  out = GetRobot()->q;
}

void ContactCSpace::Midpoint(const Config& x,const Config& y,Config& out)
{
  SingleRobotCSpace2::Midpoint(x,y,out);
  GetRobot()->UpdateConfig(out);
  SolveContact();
  out = GetRobot()->q;
}


void ContactCSpace::AddContact(const IKGoal& goal)
{
  //DEBUG
  for(size_t i=0;i<contactIK.size();i++) {
    if(contactIK[i].link == goal.link) {
      fprintf(stderr,"ContactCSpace::AddContact: adding goal on existing link, link %d, constraint %d\n",goal.link,i);
    }
    Assert(contactIK[i].link != goal.link);
  }

  contactIK.push_back(goal);
}

void ContactCSpace::AddContact(int link,const Vector3& localPos,const Vector3& worldPos)
{
  IKGoal goal;
  goal.link = link;
  goal.localPosition = localPos;
  goal.SetFixedPosition(worldPos);
  goal.SetFreeRotation();
}

void ContactCSpace::AddContact(int link,const vector<Vector3>& localPos,const vector<Vector3>& worldPos)
{
  IKGoal goal;
  goal.link = link;
  goal.SetFromPoints(localPos,worldPos);
}

void ContactCSpace::RemoveContact(int link)
{
  for(size_t i=0;i<contactIK.size();i++)
    if(contactIK[i].link==link) {
      contactIK.erase(contactIK.begin()+i);
      return;
    }
}


Real ContactCSpace::ContactDistance(const Config& q)
{
  GetRobot()->UpdateConfig(q);
  return ContactDistance();
}


Real ContactCSpace::ContactDistance()
{
  Real emax=0;
  for(size_t i=0;i<contactIK.size();i++) {
    emax = Max(emax,RobotIKError(*GetRobot(),contactIK[i]));
  }
  return emax;
}

bool ContactCSpace::CheckContact(const Config& q,Real dist)
{
  GetRobot()->UpdateConfig(q);
  return CheckContact(dist);
}

bool ContactCSpace::CheckContact(Real dist)
{
  if(dist==0) dist = settings->robotSettings[index].contactEpsilon;
  return (ContactDistance() <= dist);
}

bool ContactCSpace::SolveContact(int numIters,Real dist)
{
  numSolveContact++;
#if DO_TIMING
  Timer timer;
#endif // DO_TIMING
  if(dist==0) dist = settings->robotSettings[index].contactEpsilon*0.9;
  if(numIters==0) numIters = settings->robotSettings[index].contactIKMaxIters;
  Robot* robot=GetRobot();
  RobotIKFunction equality(*robot);
  equality.UseIK(contactIK);
  GetDefaultIKDofs(*robot,contactIK,equality.activeDofs);
  if(!fixedDofs.empty()) {
    vector<bool> active(robot->links.size(),false);
    for(size_t j=0;j<equality.activeDofs.mapping.size();j++) 
      active[equality.activeDofs.mapping[j]]=true;
    for(size_t i=0;i<fixedDofs.size();i++) {
      robot->q[fixedDofs[i]] = fixedValues[i];
      active[fixedDofs[i]]=false;
    }
    equality.activeDofs.mapping.resize(0);
    for(size_t i=0;i<active.size();i++)
      if(active[i]) equality.activeDofs.mapping.push_back(i);
  }
  RobotIKSolver solver(equality);

  //use or don't use joint limits?  threshold for some revolute joints?
#if TEST_NO_JOINT_LIMITS
  solver.UseJointLimits(Inf);
#else
  //if the valid angles of a revolute joint are over 5/4 of a half circle,
  //this allows the solver to pass through the joint limit.  May speed up
  //solving.
  solver.UseJointLimits(Real(1.25)*Pi);
#endif //TEST_NO_JOINT_LIMITS

  solver.solver.verbose = 0;
  bool res = solver.Solve(dist,numIters);
#if DO_TIMING
  solveContactTime += timer.ElapsedTime();
#endif // DO_TIMING

  //for some reason the fixed DOFs get moved slightly... TODO debug this
  for(size_t i=0;i<fixedDofs.size();i++) 
    robot->q(fixedDofs[i]) = fixedValues[i];

  return res;
}

void ContactCSpace::Properties(PropertyMap& map) const
{
  SingleRobotCSpace2::Properties(map);
  if(!contactIK.empty()) {
    int dim;
    if(map.get("intrinsicDimensionality",dim))
      ;
    else
      dim = GetRobot()->q.n;
    for(size_t i=0;i<contactIK.size();i++)
      dim -= IKGoal::NumDims(contactIK[i].posConstraint)+IKGoal::NumDims(contactIK[i].rotConstraint);
    map.set("intrinsicDimensionality",dim);
    map.set("submanifold",1);
  }
}
  


MultiContactCSpace::MultiContactCSpace(RobotWorld& world,WorldPlannerSettings* settings)
  :MultiRobotCSpace(world,settings),
   numSolveContact(0),numIsFeasible(0),solveContactTime(0),isFeasibleTime(0)
{}

MultiContactCSpace::MultiContactCSpace(const MultiRobotCSpace& space)
  :MultiRobotCSpace(space),
   numSolveContact(0),numIsFeasible(0),solveContactTime(0),isFeasibleTime(0)
{}

MultiContactCSpace::MultiContactCSpace(const MultiContactCSpace& space)
  :MultiRobotCSpace(space),
   contactPairs(space.contactPairs),
   aggregateRobot(space.aggregateRobot),closedChainConstraints(space.closedChainConstraints),
   numSolveContact(0),numIsFeasible(0),solveContactTime(0),isFeasibleTime(0)
{}

void MultiContactCSpace::InitContactPairs(const vector<ContactPair>& pairs)
{
  //TODO: error checking on contact pairs?

  contactPairs = pairs;
  vector<bool> robotActive(world.robots.size(),false);
  vector<bool> objectActive(world.rigidObjects.size(),false);
  map<int,int> worldIDtoRobotID;

  for(size_t i=0;i<pairs.size();i++) {
    int rob1=world.IsRobotLink(pairs[i].id1).first;
    int rob2=world.IsRobotLink(pairs[i].id2).first;
    int obj1=world.IsRigidObject(pairs[i].id1);
    int obj2=world.IsRigidObject(pairs[i].id2);
    if(rob1 >= 0) robotActive[rob1]=true;
    if(rob2 >= 0) robotActive[rob2]=true;
    if(obj1 >= 0) objectActive[obj1]=true;
    if(obj2 >= 0) objectActive[obj2]=true;
    if(rob1 < 0 && rob2 < 0 && obj1 < 0 && obj2 < 0 ) 
      FatalError("Initializing MultiContactCSpace: Contact pairs between two terrains or robot groups?");
  }
  int numRobots=0,numObjects=0;
  for(size_t i=0;i<robotActive.size();i++)
    if(robotActive[i]) {
      numRobots++;
      int robotid = robot.Add(world.robots[i],world.robots[i]->name.c_str());
      worldIDtoRobotID[world.RobotID(i)] = robotid;
    }
  for(size_t i=0;i<objectActive.size();i++)
    if(objectActive[i]){
      int robotid = robot.Add(world.rigidObjects[i],world.rigidObjects[i]->name.c_str());
      worldIDtoRobotID[world.RigidObjectID(i)] = robotid;
      numObjects++;
    }

  //build the aggregate robot
  robot.GetMegaRobot(aggregateRobot);

  //convert the contact pairs to IK constraints
  closedChainConstraints.resize(contactPairs.size());
  aggregateStance.clear();
  for(size_t i=0;i<contactPairs.size();i++) {
    pair<int,int> rob1=world.IsRobotLink(contactPairs[i].id1);
    pair<int,int> rob2=world.IsRobotLink(contactPairs[i].id2);
    int obj1=world.IsRigidObject(contactPairs[i].id1);
    int obj2=world.IsRigidObject(contactPairs[i].id2);
    if(rob1.first >= 0) {
      Assert(rob1.second >= 0);
      closedChainConstraints[i].link = robot.Dof(worldIDtoRobotID[world.RobotID(rob1.first)],rob1.second);
    }
    else {
      Assert(obj1 >= 0);
      closedChainConstraints[i].link = robot.Dof(worldIDtoRobotID[obj1],5);
    }
    if(rob2.first >= 0) {
      Assert(rob2.second >= 0);
      closedChainConstraints[i].destLink = robot.Dof(worldIDtoRobotID[world.RobotID(rob2.first)],rob2.second);
    }
    else if(obj2 >= 0) {
      closedChainConstraints[i].destLink = robot.Dof(worldIDtoRobotID[obj2],5);
    }
    else
      closedChainConstraints[i].destLink = -1;
    closedChainConstraints[i].SetFromPoints(contactPairs[i].c1,contactPairs[i].c2);

    int link = closedChainConstraints[i].link;
    aggregateStance[link].contacts.resize(contactPairs[i].c1.size());
    RigidTransform T,Tinv;
    T = world.GetTransform(contactPairs[i].id1);
    Tinv.setInverse(T);
    for(size_t k=0;k<contactPairs[i].c2.size();k++) {
      aggregateStance[link].contacts[k].x = contactPairs[i].c2[k];
      aggregateStance[link].contacts[k].n = contactPairs[i].n2[k];
      Real friction = 0;
      if(contactPairs[i].kFriction.size()==1)
	friction = contactPairs[i].kFriction[0];
      else if(contactPairs[i].kFriction.size()>=1)
	friction = contactPairs[i].kFriction[k];
      aggregateStance[link].contacts[k].kFriction = friction;
    }
    aggregateStance[link].ikConstraint = closedChainConstraints[i];
  }
}

int MultiContactCSpace::NumDimensions() const
{
  return MultiRobotCSpace::NumDimensions();
}

void MultiContactCSpace::Sample(Config& x)
{
  MultiRobotCSpace::Sample(x);
  x.resize(NumDimensions());
  vector<Config> robotConfigs;
  robot.SplitRefs(x,robotConfigs);
  for(size_t i=0;i<robotConfigs.size();i++)
    elementSpaces[i]->Sample(robotConfigs[i]);
  SolveContact(x);
}

void MultiContactCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  x.resize(NumDimensions());
  Assert(c.n == x.n);
  vector<Config> crobotConfigs,robotConfigs;
  robot.SplitRefs(c,crobotConfigs);
  robot.SplitRefs(x,robotConfigs);
  for(size_t i=0;i<robotConfigs.size();i++)
    elementSpaces[i]->SampleNeighborhood(crobotConfigs[i],r,robotConfigs[i]);
  SolveContact(x);
}

bool MultiContactCSpace::IsFeasible(const Config& x)
{
  numIsFeasible++;
  if(!CheckContact(x)) return false;

  vector<Config> elementConfigs;
  robot.SplitRefs(x,elementConfigs);

  robot.SetConfig(x);
  for(size_t i=0;i<elementSpaces.size();i++)
    if(!elementSpaces[i]->IsFeasible(elementConfigs[i])) return false;
  //TODO: allow element-element collision
  return true;
}

bool MultiContactCSpace::SolveContact(Config& x,int numIters,Real tol)
{
  numSolveContact++;
  Timer timer;
  if(tol==0) tol = settings->robotSettings[0].contactEpsilon*0.9;
  if(numIters==0) numIters = settings->robotSettings[0].contactIKMaxIters;
  aggregateRobot.UpdateConfig(x);
  RobotIKFunction equality(aggregateRobot);
  equality.UseIK(closedChainConstraints);
  GetDefaultIKDofs(aggregateRobot,closedChainConstraints,equality.activeDofs);
  RobotIKSolver solver(equality);

  //use or don't use joint limits?  threshold for some revolute joints?
#if TEST_NO_JOINT_LIMITS
  solver.UseJointLimits(Inf);
#else
  //if the valid angles of a revolute joint are over 5/4 of a half circle,
  //this allows the solver to pass through the joint limit.  May speed up
  //solving.
  solver.UseJointLimits(Real(1.25)*Pi);
#endif //TEST_NO_JOINT_LIMITS

  solver.solver.verbose = 0;
  bool res = solver.Solve(tol,numIters);
  x = aggregateRobot.q;
#if DO_TIMING
  solveContactTime += timer.ElapsedTime();
#endif // DO_TIMING
  return res;  
}

Real MultiContactCSpace::ContactDistance(const Config& x)
{
  aggregateRobot.UpdateConfig(x);
  return ContactDistance();
}

Real MultiContactCSpace::ContactDistance()
{
  Real emax=0;
  for(size_t i=0;i<closedChainConstraints.size();i++) {
    const IKGoal& g=closedChainConstraints[i];
    emax = Max(emax,RobotIKError(aggregateRobot,g));
  }
  return emax;
}

bool MultiContactCSpace::CheckContact(const Config& x,Real dist)
{
  if(dist==0) dist = settings->robotSettings[0].contactEpsilon;
  return ContactDistance(x) <= dist;
}

bool MultiContactCSpace::CheckContact(Real dist)
{
  if(dist==0) dist = settings->robotSettings[0].contactEpsilon;
  return ContactDistance() <= dist;
}



void MultiContactCSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{
  MultiRobotCSpace::Interpolate(x,y,u,out);
  SolveContact(out);
}

void MultiContactCSpace::Midpoint(const Config& x,const Config& y,Config& out)
{
  MultiRobotCSpace::Midpoint(x,y,out);
  SolveContact(out);
}

void MultiContactCSpace::Properties(PropertyMap& map) const
{
  MultiRobotCSpace::Properties(map);
  if(!closedChainConstraints.empty()) {
    int dim;
    if(map.get("intrinsicDimensionality",dim))
      ;
    else
      dim = aggregateRobot.q.n;
    for(size_t i=0;i<closedChainConstraints.size();i++)
      dim -= IKGoal::NumDims(closedChainConstraints[i].posConstraint)+IKGoal::NumDims(closedChainConstraints[i].rotConstraint);
    map.set("intrinsicDimensionality",dim);
    map.set("submanifold",1);
  }
}
