#include "ContactCSpace.h"
#include <robotics/IKFunctions.h>
#include <robotics/JointStructure.h>
#include <math3d/random.h>
#include <math3d/rotation.h>
#include <math3d/interpolate.h>

#define TEST_NO_JOINT_LIMITS 0

void ConfigToTransform(const Config& q,RigidTransform& T)
{
  Assert(q.n >= 6);
  T.t.set(q(0),q(1),q(2));
  EulerAngleRotation e(q(3),q(4),q(5));
  e.getMatrixZYX(T.R);
}

void TransformToConfig(const RigidTransform& T,Config& q)
{
  Assert(q.n >= 6);
  T.t.get(q(0),q(1),q(2));
  EulerAngleRotation e;
  e.setMatrixZYX(T.R);
  e.get(q(3),q(4),q(5));
}




ContactCSpace::ContactCSpace(RobotWorld& world,int index,
			     WorldPlannerSettings* settings)
  :SingleRobotCSpace2(world,index,settings)
{}

ContactCSpace::ContactCSpace(const SingleRobotCSpace& space)
  :SingleRobotCSpace2(space)
{}

ContactCSpace::ContactCSpace(const ContactCSpace& space)
  :SingleRobotCSpace2(space),contactIK(space.contactIK)
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
  GetRobot()->UpdateConfig(q);
  if(!CheckContact()) {
    printf("ContactCSpace:: Configuration fails distance check: %g > %g\n",ContactDistance(),settings->robotSettings[index].contactEpsilon*1.1);
    return false;
  }
  return SingleRobotCSpace2::IsFeasible(q);
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

Real ContactCSpace::ContactDistance()
{
  Real emax=0;
  for(size_t i=0;i<contactIK.size();i++) {
    emax = Max(emax,RobotIKError(*GetRobot(),contactIK[i]));
  }
  return emax;
}

bool ContactCSpace::CheckContact(Real dist)
{
  if(dist==0) dist = settings->robotSettings[index].contactEpsilon;
  return (ContactDistance() <= dist);
}

bool ContactCSpace::SolveContact(int numIters,Real dist)
{
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
    for(size_t i=0;i<fixedDofs.size();i++) 
      active[fixedDofs[i]]=false;
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
  if(solver.Solve(dist,numIters)) {
    return true;
  }
  return false;
}
  


MultiContactCSpace::MultiContactCSpace(RobotWorld& world,WorldPlannerSettings* settings)
  :MultiRobotCSpace(world,settings)
{}

MultiContactCSpace::MultiContactCSpace(const MultiRobotCSpace& space)
  :MultiRobotCSpace(space)
{}

MultiContactCSpace::MultiContactCSpace(const MultiContactCSpace& space)
  :MultiRobotCSpace(space),
   contactPairs(space.contactPairs),activeIDs(space.activeIDs),robotActive(space.robotActive),objectActive(space.objectActive),
   aggregateRobot(space.aggregateRobot),closedChainConstraints(space.closedChainConstraints)
{}

void MultiContactCSpace::InitContactPairs(const vector<ContactPair>& pairs)
{
  //TODO: error checking on contact pairs?

  contactPairs = pairs;
  robotActive.resize(world.robots.size());
  objectActive.resize(world.rigidObjects.size());
  fill(robotActive.begin(),robotActive.end(),false);
  fill(objectActive.begin(),objectActive.end(),false);
  activeIDs.resize(0);

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
      activeIDs.push_back(world.RobotID(i));
      numRobots++;
      MultiRobotCSpace::AddRobot(i);
    }
  for(size_t i=0;i<objectActive.size();i++)
    if(objectActive[i]){
      activeIDs.push_back(world.RigidObjectID(i));
      numObjects++;
    }

  //build the aggregate robot
  vector<RobotKinematics3D> subRobots(activeIDs.size());
  vector<int> robotOffsets(world.robots.size(),-1);
  vector<int> objectOffsets(world.rigidObjects.size(),-1);  
  int k=0;
  for(size_t i=0;i<activeIDs.size();i++) {
    int index=world.IsRobot(activeIDs[i]);
    if(index >= 0) {
      subRobots[i] = *world.robots[index].robot;
      robotOffsets[index] = k;
      k += (int)subRobots[i].links.size();
    }
    else {
      //build an object into a fake robot
      index=world.IsRigidObject(activeIDs[i]);
      Assert(index >= 0);
      subRobots[i].InitializeRigidObject();
      //init configuration
      TransformToConfig(world.rigidObjects[index].object->T,subRobots[i].q);
      subRobots[i].UpdateFrames();
      objectOffsets[index] = k;
      k += 6;
    }
  }
  aggregateRobot.Merge(subRobots);

  closedChainConstraints.resize(contactPairs.size());
  for(size_t i=0;i<contactPairs.size();i++) {
    pair<int,int> rob1=world.IsRobotLink(contactPairs[i].id1);
    pair<int,int> rob2=world.IsRobotLink(contactPairs[i].id2);
    int obj1=world.IsRigidObject(contactPairs[i].id1);
    int obj2=world.IsRigidObject(contactPairs[i].id2);
    if(rob1.first >= 0) {
      Assert(rob1.second >= 0);
      closedChainConstraints[i].link = robotOffsets[rob1.first]+rob1.second;
    }
    else {
      Assert(obj1 >= 0);
      closedChainConstraints[i].link = objectOffsets[obj1];
    }
    if(rob2.first >= 0) {
      Assert(rob2.second >= 0);
      closedChainConstraints[i].destLink = robotOffsets[rob2.first]+rob2.second;
    }
    else if(obj2 >= 0) {
      closedChainConstraints[i].destLink = objectOffsets[obj2];
    }
    else
      closedChainConstraints[i].destLink = -1;
    closedChainConstraints[i].SetFromPoints(contactPairs[i].c1,contactPairs[i].c2);
  }
}

int MultiContactCSpace::NumDimensions() const
{
  int nrd = MultiRobotCSpace::NumDimensions();
  int n=nrd;
  for(size_t i=0;i<objectActive.size();i++)
    if(objectActive[i]) n+=6;
  return n;
}

void MultiContactCSpace::Sample(Config& x)
{
  x.resize(NumDimensions());
  vector<Config> robotConfigs;
  vector<Config> objectConfigs;
  SplitRefs(x,robotConfigs,objectConfigs);
  for(size_t i=0;i<robotConfigs.size();i++)
    robotSpaces[i]->Sample(robotConfigs[i]);
  int k=0;
  for(size_t i=0;i<objectActive.size();i++) {
    if(!objectActive[i]) continue;
    QuaternionRotation q;
    RigidTransform T;
    RandRotation(q);
    q.getMatrix(T.R);
    SampleAABB(settings->objectSettings[i].worldBounds.bmin,settings->objectSettings[i].worldBounds.bmax,T.t);
    TransformToConfig(T,objectConfigs[k]);
    k++;
  }
  SolveContact(x);
}

void MultiContactCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  x.resize(NumDimensions());
  Assert(c.n == x.n);
  vector<Config> crobotConfigs,robotConfigs;
  vector<Config> cobjectConfigs,objectConfigs;
  SplitRefs(c,crobotConfigs,cobjectConfigs);
  SplitRefs(x,robotConfigs,objectConfigs);
  for(size_t i=0;i<robotConfigs.size();i++)
    robotSpaces[i]->SampleNeighborhood(crobotConfigs[i],r,robotConfigs[i]);
  int k=0;
  for(size_t i=0;i<objectActive.size();i++) {
    if(!objectActive[i]) continue;
    AngleAxisRotation aa;
    RigidTransform T,T0;
    aa.angle = Rand(0,r);
    SampleSphere(1.0,aa.axis);
    aa.getMatrix(T.R);
    SampleCube(r,T.t);
    ConfigToTransform(cobjectConfigs[k],T0);
    TransformToConfig(T*T0,objectConfigs[k]);
    k++;
  }
  SolveContact(x);
}

bool MultiContactCSpace::IsFeasible(const Config& x)
{
  if(!CheckContact(x)) return false;

  vector<Config> robotConfigs;
  vector<Config> objectConfigs;
  SplitRefs(x,robotConfigs,objectConfigs);

  SetWorldConfig(x);
  for(size_t i=0;i<robotSpaces.size();i++)
    if(!robotSpaces[i]->IsFeasible(robotConfigs[i])) return false;
  //TODO: check object-object, object-terrain collision  
  return true;
}

void MultiContactCSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{
  out.resize(NumDimensions());
  Assert(out.n == x.n);
  Assert(out.n == y.n);

  vector<Config> xrob,yrob,orob;
  vector<Config> xobj,yobj,oobj;
  SplitRefs(x,xrob,xobj);
  SplitRefs(y,yrob,yobj);
  SplitRefs(out,orob,oobj);
  for(size_t i=0;i<robotSpaces.size();i++)
    robotSpaces[i]->Interpolate(xrob[i],yrob[i],u,orob[i]);
  for(size_t i=0;i<xobj.size();i++) {
    RigidTransform xT,yT,oT;
    ConfigToTransform(xobj[i],xT);
    ConfigToTransform(yobj[i],yT);
    interpolate(xT,yT,u,oT); 
    TransformToConfig(oT,oobj[i]);
  }
  SolveContact(out);
}

void MultiContactCSpace::Midpoint(const Config& x,const Config& y,Config& out)
{
  Interpolate(x,y,0.5,out);
}

bool MultiContactCSpace::SolveContact(Config& x,int numIters,Real tol)
{
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
  if(solver.Solve(tol,numIters)) {
    x = aggregateRobot.q;
    return true;
  }
  x = aggregateRobot.q;
  return false;  
}

Real MultiContactCSpace::ContactDistance(const Config& x)
{
  aggregateRobot.UpdateConfig(x);
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

void MultiContactCSpace::SplitRefs(const Config& x,vector<Config>& robotConfigs,vector<Config>& objectConfigs) const
{
  robotConfigs.resize(0);
  objectConfigs.resize(0);
  int k=0;
  for(size_t i=0;i<activeIDs.size();i++) {
    int index=world.IsRobot(activeIDs[i]);
    if(index >= 0) {
      robotConfigs.push_back(Config());
      robotConfigs.back().setRef(x,k,1,world.robots[index].robot->links.size());
      k += (int)world.robots[index].robot->links.size();
    }
    else {
      //build an object into a fake robot
      index=world.IsRigidObject(activeIDs[i]);
      Assert(index >= 0);
      objectConfigs.push_back(Config());
      objectConfigs.back().setRef(x,k,1,6);
      k += 6;
    }
  }
}

void MultiContactCSpace::SetWorldConfig(const Config& x)
{
  vector<Config> robotConfigs,objectConfigs;
  SplitRefs(x,robotConfigs,objectConfigs);
  for(size_t i=0;i<robotSpaces.size();i++)
    robotSpaces[i]->GetRobot()->UpdateConfig(robotConfigs[i]);
  int k=0;
  for(size_t i=0;i<objectActive.size();i++) {
    if(objectActive[i]) {
      ConfigToTransform(objectConfigs[k],world.rigidObjects[i].object->T);
      k++;
    }
  }
}

void MultiContactCSpace::GetWorldConfig(Config& x)
{
  vector<Config> robotConfigs,objectConfigs;
  SplitRefs(x,robotConfigs,objectConfigs);
  for(size_t i=0;i<robotSpaces.size();i++)
    robotConfigs[i]=robotSpaces[i]->GetRobot()->q;
  int k=0;
  for(size_t i=0;i<objectActive.size();i++) {
    if(objectActive[i]) {
      TransformToConfig(world.rigidObjects[i].object->T,objectConfigs[k]);
      k++;
    }
  }
}


