#include "ODERobot.h"
#include "ODECommon.h"
#include "ODECustomGeometry.h"
#include "Modeling/RigidObject.h"
#include <ode/ode.h>
#include <KrisLibrary/math/angle.h>
#include <KrisLibrary/math3d/interpolate.h>
#include <KrisLibrary/robotics/Rotation.h>
#include <KrisLibrary/Logger.h>
#include "Settings.h"


DECLARE_LOGGER(ODESimulator)

namespace Klampt {

double ODERobot::defaultPadding = gDefaultRobotPadding;
//k restitution of 0.1, friction of 1, infinite stiffness
ODESurfaceProperties ODERobot::defaultSurface = {0.1,1.0,Inf,Inf};

//defined in ODESimulator.cpp
void* RobotIndexToGeomData(int robot,int link);

//given inertia matrix Hc about c, returns inertia matrix around origin
Matrix3 TranslateInertia(const Matrix3& Hc,const Vector3& c,Real mass)
{
  Matrix3 Ho=Hc;
  Ho(0,0) += mass*(c.y*c.y+c.z*c.z);
  Ho(1,1) += mass*(c.x*c.x+c.z*c.z);
  Ho(2,2) += mass*(c.y*c.y+c.x*c.x);
  Ho(0,1) -= mass*c.x*c.y;
  Ho(0,2) -= mass*c.x*c.z;
  Ho(1,2) -= mass*c.y*c.z;
  Ho(1,0) = Ho(0,1);
  Ho(2,0) = Ho(0,2);
  Ho(2,1) = Ho(1,2);
  return Ho;
}

ODERobot::ODERobot(RobotModel& _robot)
  :robot(_robot),jointGroupID(0),spaceID(0)
{
}

ODERobot::~ODERobot()
{
  Clear();
}

void ODERobot::Clear()
{
  for(size_t i=0;i<bodyID.size();i++) SafeDeleteProc(bodyID[i],dBodyDestroy);
  for(size_t i=0;i<jointID.size();i++) SafeDeleteProc(jointID[i],dJointDestroy);
  for(size_t i=0;i<geometry.size();i++) SafeDelete(geometry[i]);
  SafeDeleteProc(jointGroupID,dJointGroupDestroy);
  SafeDeleteProc(spaceID,dSpaceDestroy);

  bodyID.resize(0);
  geometry.resize(0);
  jointID.resize(0);
  jointFeedback.resize(0);
  tempGeometries.resize(0);
}


//for attached links, returns the base body for the link
dBodyID ODERobot::baseBody(int link) const
{
  while(link >= 0 && bodyID[link] == NULL) {
    link = robot.parents[link];
  }
  return (link == -1? 0 : bodyID[link]);
}

void ODERobot::EnableSelfCollisions(bool enabled)
{
  if(SelfCollisionsEnabled() != enabled) {
    //remove geometry
    if(spaceID) {
      for(size_t i=0;i<geometry.size();i++)
        if(geometry[i]) dSpaceRemove(spaceID,geometry[i]->geom());
    }

    SafeDeleteProc(spaceID,dSpaceDestroy);
    if(enabled) {
      spaceID = dSimpleSpaceCreate(0);
    }
    else {
      spaceID = dHashSpaceCreate(0);
      dHashSpaceSetLevels(spaceID,-3,0);
    }
    dSpaceSetCleanup(spaceID,0);
    //dGeomSetData(spaceID,(void*)this);

    //add back geometry
    for(size_t i=0;i<geometry.size();i++)
      if(geometry[i]) dSpaceAdd(spaceID,geometry[i]->geom());
  }
}

bool ODERobot::SelfCollisionsEnabled() const
{
  if(!spaceID) return false;
  if(dSpaceGetClass(spaceID) == dSimpleSpaceClass) return true;
  else return false;
}

void ODERobot::Create(int robotIndex,dWorldID worldID,bool useBoundaryLayer)
{
  Clear();

  spaceID = dHashSpaceCreate(0);
  dHashSpaceSetLevels(spaceID,-3,0);
  dSpaceSetCleanup(spaceID,0);
  //dGeomSetData(spaceID,(void*)this);

  //must do this to get proper zero reference points for joints
  Config oldQ = robot.q;
  robot.q.setZero();
  robot.UpdateFrames();


  //get an articulated rigid body representation of the robot
  //merge floating links and welded links into a composite body
  vector<int> linkToJoint(robot.links.size(),-1);
  for(size_t i=0;i<robot.joints.size();i++) {
    vector<int> inds;
    robot.GetJointIndices(i,inds);
    for(size_t j=0;j<inds.size();j++) {
      Assert(linkToJoint[inds[j]] < 0);
      linkToJoint[inds[j]] = (int)i;
    }
  }
  for(size_t i=0;i<linkToJoint.size();i++)
    Assert(linkToJoint[i] >= 0);

  vector<vector<int> > bodyJoints;
  vector<vector<int> > bodyLinks;
  vector<int> jointToBody(robot.joints.size(),-1);
  for(size_t i=0;i<robot.joints.size();i++) {
    if(robot.joints[i].type == RobotModelJoint::Floating || robot.joints[i].type == RobotModelJoint::FloatingPlanar || robot.joints[i].type == RobotModelJoint::BallAndSocket) {
      jointToBody[i] = bodyJoints.size();
      bodyJoints.push_back(vector<int>(1,i));
      bodyLinks.push_back(vector<int>(1,robot.joints[i].linkIndex));
    }
    else if(robot.joints[i].type == RobotModelJoint::Weld) {
      int link=robot.joints[i].linkIndex;
      int parent = robot.parents[link];
      if(parent >= 0) {
        if(robot.links[link].mass != 0 && robot.links[parent].mass != 0) {
          //printf("Weld joint %d (link %d -> %d) considered different bodies\n",i,link,parent);
          //make a new body
          jointToBody[i] = bodyJoints.size();
          bodyJoints.push_back(vector<int>(1,i));
          bodyLinks.push_back(vector<int>(1,robot.joints[i].linkIndex));
        }
        else {
          //printf("Weld joint %d (link %d -> %d) considered as same body\n",i,link,parent);
          //attach to an existing body
          int body = jointToBody[linkToJoint[parent]];
          Assert(body >= 0);
          bodyJoints[body].push_back(i);
          bodyLinks[body].push_back(link);
          jointToBody[i] = body;
        }
      }
      else {
        //create a new body attached to the world
        jointToBody[i] = bodyJoints.size();
        bodyJoints.push_back(vector<int>(1,i));
        bodyLinks.push_back(vector<int>(1,robot.joints[i].linkIndex));
      }
    }
    else {
      jointToBody[i] = bodyJoints.size();
      bodyJoints.push_back(vector<int>(1,i));
      bodyLinks.push_back(vector<int>(1,robot.joints[i].linkIndex));
    }
  }
  //construct rigid bodies
  vector<RigidObjectModel> bodyObjects(bodyJoints.size());
  vector<ODEGeometry*> bodyGeometry(bodyJoints.size(),NULL);
  for(size_t i=0;i<bodyJoints.size();i++) {
    int joint = bodyJoints[i][0];
    int baseLink = robot.joints[joint].linkIndex;
    if(robot.joints[i].type == RobotModelJoint::Floating || robot.joints[i].type == RobotModelJoint::FloatingPlanar || robot.joints[i].type == RobotModelJoint::BallAndSocket) {
      //can't have any mass/inertia/geometry leading up to link
      vector<int> indices;
      robot.GetJointIndices(joint,indices);
      for(size_t j=0;j+1<indices.size();j++) {
        Assert(indices[j] != baseLink);
        Assert(robot.links[indices[j]].mass == 0.0);
        Assert(robot.links[indices[j]].inertia.isZero());
        Assert(robot.IsGeometryEmpty(indices[j]));
      }
    }
    if(bodyJoints[i].size()==1) { //single joint
      bodyObjects[i].mass = robot.links[baseLink].mass;
      bodyObjects[i].com = robot.links[baseLink].com;
      bodyObjects[i].inertia = robot.links[baseLink].inertia;
      bodyObjects[i].T.R = robot.links[baseLink].T_World.R; 
      bodyObjects[i].T.t = robot.links[baseLink].T_World * robot.links[baseLink].com; 
      if(!robot.IsGeometryEmpty(baseLink)) {
        bodyGeometry[i] = new ODEGeometry;
        bodyGeometry[i]->Create(robot.geometry[baseLink].get(),spaceID,-robot.links[baseLink].com,useBoundaryLayer);
      }
    }
    else {
      vector<Geometry::AnyGeometry3D> meshes(bodyLinks[i].size());
      //merge all the masses/coms/inertias onto the base link
      bodyObjects[i].mass = 0;
      //compute com/inertia relative to baseLink
      bodyObjects[i].com.setZero();
      bodyObjects[i].inertia.setZero();
      for(size_t j=0;j<bodyLinks[i].size();j++) {
        int link = bodyLinks[i][j];
        RigidTransform Trel;
        Trel.mulInverseA(robot.links[baseLink].T_World,robot.links[link].T_World);
        if(robot.links[link].mass > 0) {
          bodyObjects[i].mass += robot.links[link].mass;
          bodyObjects[i].com += robot.links[link].mass*(Trel*robot.links[link].com);
          //transform inertia matrix
          Matrix3 temp,inertiaLink;
          temp.mulTransposeB(robot.links[link].inertia,Trel.R);
          inertiaLink.mul(Trel.R,temp);
          inertiaLink = TranslateInertia(inertiaLink,Trel.t,robot.links[link].mass);
          bodyObjects[i].inertia += (inertiaLink*robot.links[link].mass);
        }

        if(!robot.IsGeometryEmpty(link)) {
          //get transformed mesh
          meshes[j] = Geometry::AnyGeometry3D(*robot.geometry[link]);
          meshes[j].Transform(Trel);
        }
      }
      bodyObjects[i].com /= bodyObjects[i].mass;
      bodyObjects[i].inertia /= bodyObjects[i].mass;

      //the COM is the origin of the body
      bodyObjects[i].T.R = robot.links[baseLink].T_World.R; 
      bodyObjects[i].T.t = robot.links[baseLink].T_World * bodyObjects[i].com; 
      
      tempGeometries.resize(tempGeometries.size()+1);
      tempGeometries.back().reset(new RobotWithGeometry::CollisionGeometry);
      tempGeometries.back()->Merge(meshes);
      if(!tempGeometries.back()->Empty()) {
        bodyGeometry[i] = new ODEGeometry;
        bodyGeometry[i]->Create(tempGeometries.back().get(),spaceID,-bodyObjects[i].com,useBoundaryLayer);
      }
    }
    if(bodyObjects[i].mass == 0.0) {
      LOG4CXX_WARN(GET_LOGGER(ODESimulator),"ODERobot: warning, body "<<i<<" has mass zero");
      stringstream ss;
      for(size_t j=0;j<bodyLinks[i].size();j++)
        ss<<robot.LinkName(bodyLinks[i][j])<<", ";
      LOG4CXX_WARN(GET_LOGGER(ODESimulator),"  Consists of links: "<<ss.str());
    }
    if(bodyObjects[i].inertia.isZero()) {
      LOG4CXX_WARN(GET_LOGGER(ODESimulator),"ODERobot: warning, body "<<i<<" has zero inertia");
      stringstream ss;
      for(size_t j=0;j<bodyLinks[i].size();j++)
        ss<<robot.LinkName(bodyLinks[i][j])<<", ";
      LOG4CXX_WARN(GET_LOGGER(ODESimulator),"  Consists of links: "<<ss.str());
    }
  }

  //set up members
  bodyID.resize(robot.links.size(),NULL);
  geometry.resize(robot.links.size(),NULL);
  jointID.resize(robot.links.size(),NULL);
  jointFeedback.resize(robot.links.size());

  T_bodyToLink.resize(robot.links.size());
  vector<int> linkToBody(robot.links.size(),-1);
  for(size_t i=0;i<bodyLinks.size();i++) {
    for(size_t j=0;j<bodyLinks[i].size();j++) {
      linkToBody[bodyLinks[i][j]] = (int)i;
      T_bodyToLink[bodyLinks[i][j]].mulInverseA(robot.links[bodyLinks[i][j]].T_World,bodyObjects[i].T);
      //cout<<"Body to link: "<<bodyLinks[i][j]<<endl;
      //cout<<T_bodyToLink[bodyLinks[i][j]]<<endl;
      //cout<<"Compare to com: "<<robot.links[bodyLinks[i][j]].com<<endl;
    }

    //only the primary bodies get a non-NULL geometry/bodyID
    int primaryLink = robot.joints[bodyJoints[i][0]].linkIndex;
    Matrix3 ident; ident.setIdentity();
    if(!T_bodyToLink[primaryLink].R.isEqual(ident,1e-4)) {
      LOG4CXX_FATAL(GET_LOGGER(ODESimulator),"Mismatch betwen body and primary link orientations?");
      LOG4CXX_FATAL(GET_LOGGER(ODESimulator),"Body: "<<bodyObjects[i].T.R);
      LOG4CXX_FATAL(GET_LOGGER(ODESimulator),"Link: "<<robot.links[primaryLink].T_World.R);
      LOG4CXX_FATAL(GET_LOGGER(ODESimulator),"T_bodyToLink: "<<T_bodyToLink[primaryLink].R);
    }
    Assert(T_bodyToLink[primaryLink].R.isEqual(ident,1e-4));
    T_bodyToLink[primaryLink].R = ident;

    //set up body
    bodyID[primaryLink] = dBodyCreate(worldID);
    RigidObjectModel& body = bodyObjects[i];
    //set body transform
    dBodySetPosition(bodyID[primaryLink],body.T.t.x,body.T.t.y,body.T.t.z);
    dMatrix3 rot;
    CopyMatrix(rot,body.T.R);
    dBodySetRotation(bodyID[primaryLink],rot);
    //set body mass
    dMass mass;
    mass.mass = body.mass;
    //NOTE: in ODE, COM must be zero vector!
    mass.c[0] = mass.c[1] = mass.c[2] = 0; mass.c[3] = 1;
    CopyMatrix(mass.I,body.inertia);
    int res=dMassCheck(&mass);
    if(res != 1) {
      LOG4CXX_WARN(GET_LOGGER(ODESimulator),"Uh... mass of body "<<i<<" is not considered to be valid by ODE?");
      if(body.mass <= 0) {
        LOG4CXX_WARN(GET_LOGGER(ODESimulator),"  Mass: "<<body.mass);
        mass.mass = 0.001;
        LOG4CXX_WARN(GET_LOGGER(ODESimulator),"  Setting mass to 0.001.");
      }
      LOG4CXX_WARN(GET_LOGGER(ODESimulator),"  Inertia: "<<body.inertia);
      body.inertia.setIdentity(); body.inertia *= 0.001;
      CopyMatrix(mass.I,body.inertia);
      LOG4CXX_WARN(GET_LOGGER(ODESimulator),"  Setting inertia to 0.001*identity.");
      KrisLibrary::loggerWait();
      Assert(dMassCheck(&mass)==1);
    }
    dBodySetMass(bodyID[primaryLink],&mass);

    //set up geometry
    geometry[primaryLink] = bodyGeometry[i];    
    if(geometry[primaryLink] != NULL) {
      dGeomSetBody(geometry[primaryLink]->geom(),bodyID[primaryLink]);
      dGeomSetData(geometry[primaryLink]->geom(),RobotIndexToGeomData(robotIndex,primaryLink));

      //printf("Robot %d link %d GeomData set to %p\n",robotIndex,primaryLink,RobotIndexToGeomData(robotIndex,primaryLink));
      //set defaults
      geometry[primaryLink]->SetPadding(defaultPadding);
      geometry[primaryLink]->surf() = defaultSurface;
    }
  }

  Assert(jointGroupID == 0);
  jointGroupID = dJointGroupCreate(0);
  for(size_t i=0;i<robot.joints.size();i++) {
    switch(robot.joints[i].type) {
    case RobotModelJoint::Weld:
      //ignore weld joints that are attached to another rigid body
      //if the parent is -1, then it's attached to the world
      {
        int link = robot.joints[i].linkIndex;
        int parent = robot.parents[link];
        if(parent < 0) {
          Assert(bodyID[link] != 0);
          jointID[link] = dJointCreateFixed(worldID,jointGroupID);
          dJointAttach(jointID[link],bodyID[link],NULL);
          dJointSetFeedback(jointID[link],&jointFeedback[link]);
          dJointSetFixed(jointID[link]);
        }
        else {
          if(bodyID[link] != 0) {
            //not considered the same body as the parent
            jointID[link] = dJointCreateFixed(worldID,jointGroupID);
            dJointAttach(jointID[link],bodyID[link],bodyID[parent]);
            dJointSetFeedback(jointID[link],&jointFeedback[link]);
            dJointSetFixed(jointID[link]);
          }
        }
      }
      break;
    case RobotModelJoint::Normal:
    case RobotModelJoint::Spin:
      {
        int link = robot.joints[i].linkIndex;
        Assert(bodyID[link] != 0);
        int parent = robot.parents[link];
        dBodyID bp = baseBody(parent);
        if(robot.links[link].type == RobotLink3D::Revolute) {
          jointID[link] = dJointCreateHinge(worldID,jointGroupID);
          dJointAttach(jointID[link],bodyID[link],bp);
          Vector3 pos = robot.links[link].T_World.t;
          Vector3 axis = robot.links[link].T_World.R*robot.links[link].w;
          dJointSetHingeAnchor(jointID[link],pos.x,pos.y,pos.z);
          dJointSetHingeAxis(jointID[link],axis.x,axis.y,axis.z);
          if(robot.joints[i].type != RobotModelJoint::Spin) {
            if(USE_JOINT_STOPS) {
              //printf("Joint %d (link %s) ODE joint stops %g %g\n",i,robot.LinkName(link).c_str(),robot.qMin(link),robot.qMax(link));
              //stops are not working correctly if they are out of the range (-pi,pi) -- ODE flips out
              if(robot.qMin(link) <= -Pi || robot.qMax(link) >= Pi) {
                LOG4CXX_WARN(GET_LOGGER(ODESimulator),"ODERobot: Warning, turning off joint stops on joint "<<i<<" (link "<<robot.LinkName(link)<<") because of ODE range mismatch");
              }
              else {
                if(robot.qMin(link) <= -Pi+0.1 || robot.qMax(link) >= Pi-0.1) {
                  LOG4CXX_WARN(GET_LOGGER(ODESimulator),"ODERobot: Warning, robot joint limits on joint "<<i<<" (link "<<robot.LinkName(link)<<") are close to [-pi,pi].");
                  LOG4CXX_WARN(GET_LOGGER(ODESimulator),"   Due to ODE joint stop handling quirks this may cause the robot to flip out");
                  LOG4CXX_WARN(GET_LOGGER(ODESimulator),"   Turn off USE_JOINT_STOPS in Klampt/Simulation/Settings.h if this becomes an issue");
                }
                dJointSetHingeParam(jointID[link],dParamLoStop,robot.qMin(link));
                dJointSetHingeParam(jointID[link],dParamHiStop,robot.qMax(link));
              }
            }
            dJointSetHingeParam(jointID[link],dParamBounce,0);
          }
          dJointSetHingeParam(jointID[link],dParamFMax,0);
        }
        else {
          Assert(robot.joints[i].type != RobotModelJoint::Spin);
          jointID[link] = dJointCreateSlider(worldID,jointGroupID);
          dJointAttach(jointID[link],bodyID[link],bp);
          Vector3 axis = robot.links[link].T_World.R*robot.links[link].w;
          dJointSetSliderAxis(jointID[link],axis.x,axis.y,axis.z);
          //stops are not working correctly if they are out of the range [-pi,pi]
          if(USE_JOINT_STOPS) {
            dJointSetSliderParam(jointID[link],dParamLoStop,robot.qMin(link));
            dJointSetSliderParam(jointID[link],dParamHiStop,robot.qMax(link));
          }
          dJointSetSliderParam(jointID[link],dParamBounce,0);
          dJointSetSliderParam(jointID[link],dParamFMax,0);
        }
        dJointSetFeedback(jointID[link],&jointFeedback[link]);
      }
      break;
    case RobotModelJoint::Floating:
      break;
    default:
      FatalError("TODO: setup FloatingPlanar, BallAndSocket, and Custom joints\n");
      break;
    }
  }

  robot.UpdateConfig(oldQ);
  SetConfig(oldQ);
}

void ODERobot::SetJointDryFriction(int joint,Real coeff)
{
  int k=robot.joints[joint].linkIndex;
  SetLinkDryFriction(k,coeff);
}

void ODERobot::SetLinkDryFriction(int k,Real coeff)
{
  if(!jointID[k]) return;
  if(robot.links[k].type == RobotLink3D::Revolute) {
    dJointSetHingeParam(jointID[k],dParamVel,0);
    dJointSetHingeParam(jointID[k],dParamFMax,coeff);
  }
  else {
    dJointSetSliderParam(jointID[k],dParamVel,0);
    dJointSetSliderParam(jointID[k],dParamFMax,coeff);
  }
}

void ODERobot::SetJointFixedVelocity(int joint,Real vel,Real tmax)
{
  SetLinkFixedVelocity(robot.joints[joint].linkIndex,vel,tmax);
}

void ODERobot::SetLinkFixedVelocity(int k,Real vel,Real tmax)
{
  if(!jointID[k]) return;
  if(robot.links[k].type == RobotLink3D::Revolute) {
    dJointSetHingeParam(jointID[k],dParamVel,vel);
    dJointSetHingeParam(jointID[k],dParamFMax,tmax);
  }
  else {
    dJointSetSliderParam(jointID[k],dParamVel,vel);
    dJointSetSliderParam(jointID[k],dParamFMax,tmax);
  }
}

void ODERobot::SetConfig(const Config& q)
{
  if(q != robot.q) {
    LOG4CXX_FATAL(GET_LOGGER(ODESimulator),"ODERobot::SetConfig() TODO: We're asserting that the q is the");
    LOG4CXX_FATAL(GET_LOGGER(ODESimulator),"active configuration in order to avoid unexpected changes in the temporary");
    LOG4CXX_FATAL(GET_LOGGER(ODESimulator),"robot configuration");
  }
  Assert(q == robot.q);
  for(size_t i=0;i<robot.links.size();i++) {
    SetLinkTransform(i,robot.links[i].T_World);
  }
}

void ODERobot::GetConfig(Config& q) const
{
  q.resize(robot.links.size());
  for(size_t i=0;i<robot.joints.size();i++) {
    int k=robot.joints[i].linkIndex;
    switch(robot.joints[i].type) {
    case RobotModelJoint::Weld:
      q(k)=robot.q(k);
      break;
    case RobotModelJoint::Normal:
    case RobotModelJoint::Spin:
      q(k) = GetLinkAngle(k);
      break;
    case RobotModelJoint::Floating:
      {
        //first, get the root position
        RigidTransform T;
        GetLinkTransform(k,T);
        robot.SetJointByTransform(i,k,T);
        vector<int> indices;
        robot.GetJointIndices(i,indices);
        for(size_t j=0;j<indices.size();j++)
          q(indices[j]) = robot.q(indices[j]);
        break;
      }
    default:
      FatalError("TODO: affine and other joints");
      break;
    }
  }
  robot.NormalizeAngles(q);
  //cout<<"q = "<<q<<endl;
}

Real ODERobot::GetJointAngle(int joint) const
{
  return GetLinkAngle(robot.joints[joint].linkIndex);
}

Real ODERobot::GetLinkAngle(int i) const
{
  if(!jointID[i]) return 0;
  if(robot.links[i].type == RobotLink3D::Revolute) {
    Real val=dJointGetHingeAngle(jointID[i]);
    //normalize angle
    Real qi = AngleNormalize(val);
    if(qi > robot.qMax(i)) {
      if(qi - TwoPi >= robot.qMin(i)) 
        qi -= TwoPi;
      else if(Abs(qi - TwoPi - robot.qMin(i)) < Abs(qi - robot.qMax(i)))
        qi -= TwoPi;
    }
    if(qi < robot.qMin(i)) {
      if(qi + TwoPi <= robot.qMax(i))
        qi += TwoPi;
      else if(Abs(qi + TwoPi - robot.qMax(i)) < Abs(qi - robot.qMin(i)))
        qi += TwoPi;
    }
    val = qi;
    return val;
  }
  else {
    return dJointGetSliderPosition(jointID[i]);
  }
}

void ODERobot::SetLinkTransform(int link,const RigidTransform& T)
{
  dBodyID bodyid = body(link);
  if(!bodyid) return;
  RigidTransform Tbody;
  Tbody.mul(T,T_bodyToLink[link]);
  //Vector3 comPos = T*robot.links[link].com;
  dBodySetPosition(bodyid,Tbody.t.x,Tbody.t.y,Tbody.t.z);
  dMatrix3 rot;
  CopyMatrix(rot,Tbody.R);
  dBodySetRotation(bodyid,rot);
}

void ODERobot::GetLinkTransform(int link,RigidTransform& T) const
{
  dBodyID bodyid = body(link);
  if(!bodyid) {
    bodyid = baseBody(link);
    if(!bodyid) {
      //when does this happen?
      //fprintf(stderr,"ODERobot: baseBody returned NULL\n");
      T.setIdentity();
      return;
    }
  }
  const dReal* pos = dBodyGetPosition(bodyid);
  const dReal* rot = dBodyGetRotation(bodyid);
  RigidTransform Tbody;
  CopyMatrix(Tbody.R,rot);
  Tbody.t.set(pos[0],pos[1],pos[2]);
  T.mulInverseB(Tbody,T_bodyToLink[link]);
}

void ODERobot::SetLinkVelocity(int link,const Vector3& w,const Vector3& v)
{
  dBodyID bodyid = body(link);
  if(!bodyid) return;

  //v is the velocity at the link frame
  //v' = v + w x (cm - linkorigin)
  //dBody needs velocity at the com
  RigidTransform T;
  GetLinkTransform(link,T);
  const dReal* cm = dBodyGetPosition(bodyid);
  Vector3 vcm = v + cross(w,Vector3(cm[0],cm[1],cm[2])-T.t);

  dBodySetLinearVel(bodyid,vcm.x,vcm.y,vcm.z);
  dBodySetAngularVel(bodyid,w.x,w.y,w.z);
}

void ODERobot::GetLinkVelocity(int link,Vector3& w,Vector3& v) const
{
  dBodyID bodyid = body(link);
  if(!bodyid) {
    bodyid = baseBody(link);
    if(!bodyid) {
      //when does this happen?
      //fprintf(stderr,"ODERobot: baseBody returned NULL\n");
      v.setZero();
      w.setZero();
      return;
    }
  }
  CopyVector(v,dBodyGetLinearVel(bodyid));
  CopyVector(w,dBodyGetAngularVel(bodyid));

  //v is the velocity at the body's com
  //v = x' + w x (cm - linkorigin)
  //x' = v - w x (cm - linkorigin)
  RigidTransform T;
  GetLinkTransform(link,T);
  const dReal* cm = dBodyGetPosition(bodyid);
  v = v-cross(w,Vector3(cm[0],cm[1],cm[2]) - T.t);
}

void ODERobot::SetVelocities(const Config& dq)
{
  Vector3 zero(Zero);
  Vector3 v,w;
  for(size_t i=0;i<robot.links.size();i++) {
    if(!bodyID[i]) continue;
    robot.GetWorldVelocity(T_bodyToLink[i].t,i,dq,v);
    robot.GetWorldAngularVelocity(i,dq,w);
    dBodySetLinearVel(bodyID[i],v.x,v.y,v.z);
    dBodySetAngularVel(bodyID[i],w.x,w.y,w.z);
  }

  //DEBUG
  Vector temp=dq;
  GetVelocities(temp);
  if(!temp.isEqual(dq,1e-4)) {
    LOG4CXX_WARN(GET_LOGGER(ODESimulator),"ODERobot::SetVelocities: Error, Get/SetVelocities don't match");
    LOG4CXX_WARN(GET_LOGGER(ODESimulator),"dq = "<<dq);
    LOG4CXX_WARN(GET_LOGGER(ODESimulator),"from GetVelocities = "<<temp);
    LOG4CXX_WARN(GET_LOGGER(ODESimulator),"Error: "<<temp.distance(dq));
    LOG4CXX_WARN(GET_LOGGER(ODESimulator),"did you remember to set the robot's configuration?");
    KrisLibrary::loggerWait();
  }
  //Assert(temp.isEqual(dq,1e-4));
}

void ODERobot::GetVelocities(Config& dq) const
{
  dq.resize(robot.links.size(),Zero);
  //get dq for the joints
  for(size_t i=0;i<robot.joints.size();i++) {
    int k=robot.joints[i].linkIndex;
    switch(robot.joints[i].type) {
    case RobotModelJoint::Weld:
      break;
    case RobotModelJoint::Normal:
    case RobotModelJoint::Spin:
      Assert(jointID[k] != NULL);
      if(robot.links[k].type == RobotLink3D::Revolute) 
        dq(k)=dJointGetHingeAngleRate(jointID[k]);
      else
        dq(k)=dJointGetSliderPositionRate(jointID[k]);
      break;
    case RobotModelJoint::Floating: 
      {
        Assert(bodyID[k] != NULL);
        vector<int> indices;
        robot.GetJointIndices(i,indices);
        Assert(indices.size()==6 || indices.size()==3);
        Assert(indices.back()==k);
        //get dq for the root link (assumes robot frames are updated correctly)
        Vector3 w,v;
        GetLinkVelocity(k,w,v);
        robot.SetJointVelocityByMoment(i,k,w,v);
        for(size_t j=0;j<indices.size();j++)
          dq(indices[j]) = robot.dq(indices[j]);
      }
      break;
    default:
      FatalError("TODO: affine and other joints");
      break;
    }
  }
}

Real ODERobot::GetJointVelocity(int joint) const
{
  return GetLinkVelocity(robot.joints[joint].linkIndex);
}

Real ODERobot::GetLinkVelocity(int i) const
{
  if(!jointID[i]) return 0;
  if(robot.links[i].type == RobotLink3D::Revolute) 
    return dJointGetHingeAngleRate(jointID[i]);
  else
    return dJointGetSliderPositionRate(jointID[i]);
}

Real ODERobot::GetKineticEnergy() const
{
  Real ke = 0;
  for(size_t i=0;i<robot.links.size();i++)
    ke += GetKineticEnergy(i);
  return ke;
}

Real ODERobot::GetKineticEnergy(int link) const
{
  dBodyID bodyid = body(link);
  if(!bodyid) {
    bodyid = baseBody(link);
    if(!bodyid) {
      //when does this happen?
      //fprintf(stderr,"ODERobot: baseBody returned NULL\n");
      return 0;
    }
  }
  Vector3 v,w;
  CopyVector(v,dBodyGetLinearVel(bodyid));
  CopyVector(w,dBodyGetAngularVel(bodyid));
  const dReal* rot = dBodyGetRotation(bodyid);
  Matrix3 Rbody;
  CopyMatrix(Rbody,rot);
  Vector3 wlocal;
  Rbody.mulTranspose(w,wlocal);
  return robot.links[link].mass*v.normSquared() + wlocal.dot(robot.links[link].inertia*wlocal);
}

void ODERobot::AddTorques(const Config& t)
{
  Assert(t.n == (int)robot.links.size());
  for(int i=0;i<t.n;i++)
    if(!IsFinite(t(i))) {
      LOG4CXX_ERROR(GET_LOGGER(ODESimulator),"ODERobot::AddTorques: Error, commanding link "<<i<<" to a non-finite torque!");
      KrisLibrary::loggerWait();
      return;
    }

  /*
  EulerAnglesToAngVel(theta,Vector3(t(3),t(4),t(5)),m);
  dBodyAddTorque(bodyID[0],m.x,m.y,m.z);
  //additional force due to COM shift
  Vector3 f2 = cross(m,Rcm);
  //Matrix3 inertiaInv; inertiaInv.setInverse(robot.links[5].inertia);
  //Vector3 f2=cross(inertiaInv*m,T.R*robot.links[5].com)*robot.links[5].mass;
  dBodyAddForce(bodyID[0],f2.x,f2.y,f2.z);
  */

  for(size_t i=0;i<robot.joints.size();i++) {
    int k=robot.joints[i].linkIndex;
    switch(robot.joints[i].type) {
    case RobotModelJoint::Weld:
      break;
    case RobotModelJoint::Normal:
      if(robot.links[k].type == RobotLink3D::Revolute) 
        dJointAddHingeTorque(jointID[k],t(k));
      else
        dJointAddSliderForce(jointID[k],t(k));
      break;
    case RobotModelJoint::Floating:
      {
        //for floating, convert generalized torques to true force/moment
        //Jt (f,m) = t
        //J = [dcm/dq] = [I A]*[t[1-3]]
        //    [dR/dq ]   [0 B] [t[4-6]]
        //A = -[Rcm]B
        //f = t[1-3]
        //m = B^-t t[4-6] + t[1-3] x Rcm
        //TODO: generalize to anything besides RPY rotations
        int tx,ty,tz;
        int rx,ry,rz;
        vector<int> indices;
        robot.GetJointIndices(i,indices);
        Assert(indices.size()==6 || indices.size()==3);
        if(indices.size()==6) {
          tx = indices[0]; ty=indices[1]; tz=indices[2];
          rz = indices[3]; ry=indices[4]; rx=indices[5];
          dBodyAddForce(bodyID[k],t(tx),t(ty),t(tz));
          RigidTransform T;
          GetLinkTransform(k,T);
          EulerAngleRotation theta;
          theta.setMatrixZYX(T.R);
          Vector3 m;
          Vector3 Rcm = T.R*robot.links[k].com;
          Matrix3 Bt,Btinv;
          EulerAngleMoments(theta,2,1,0,Bt);
          Bt.inplaceTranspose();
          Btinv.setInverse(Bt);
          //Btinv = Bt;
          Btinv.mul(Vector3(t(rx),t(ry),t(rz)),m);
          m += cross(Vector3(t(tx),t(ty),t(tz)),Rcm);
          dBodyAddTorque(bodyID[k],m.x,m.y,m.z);
        }
        else {
          tx = indices[0]; ty=indices[1]; tz=indices[2];
          dBodyAddForce(bodyID[k],t(tx),t(ty),t(tz));
        }
      }
      break;
    default:
      FatalError("TODO");
    }
  }
}

void ODERobot::AddJointTorque(int joint,Real t)
{
  AddLinkTorque(robot.joints[joint].linkIndex,t);
}

void ODERobot::AddLinkTorque(int i,Real t)
{
  if(!jointID[i]) { 
    LOG4CXX_WARN(GET_LOGGER(ODESimulator),"ODERobot::AddLinkTorque: Warning, no link "<<i);
    return;
  }
  if(!IsFinite(t)) {
    LOG4CXX_ERROR(GET_LOGGER(ODESimulator),"ODERobot::AddLinkTorque: Error, commanding link "<<i<<" to a non-finite torque!");
    KrisLibrary::loggerWait();
    return;
  }
  if(robot.links[i].type == RobotLink3D::Revolute)   
    dJointAddHingeTorque(jointID[i],t);
  else
    dJointAddSliderForce(jointID[i],t);
}

Real ODERobot::GetDriverValue(int driver) const
{
  const RobotModelDriver& d=robot.drivers[driver];
  switch(d.type) {
  case RobotModelDriver::Normal:
    return GetLinkAngle(d.linkIndices[0]);
  case RobotModelDriver::Translation:
    {
      RigidTransform T;
      GetLinkTransform(d.linkIndices[1],T);
      return robot.links[d.linkIndices[0]].w.dot(T.t);
    }
    break;
  case RobotModelDriver::Rotation:
    {
      RigidTransform T;
      GetLinkTransform(d.linkIndices[1],T);
      Vector3 axis = robot.links[d.linkIndices[0]].w;
      EulerAngleRotation ea;
      ea.setMatrixZYX(T.R);
      if(axis.x == 1) return ea.z;
      else if(axis.y == 1) return ea.y;
      else if(axis.z == 1) return ea.x;
      else {
        LOG4CXX_ERROR(GET_LOGGER(ODESimulator),"ODERobot: Invalid axis for rotation driver, simulation will likely be unstable!");
        return MatrixAngleAboutAxis(T.R,axis);
      }
    }
    break;
  case RobotModelDriver::Affine: 
    {
      Real vavg=0;
      for(size_t i=0;i<d.linkIndices.size();i++) {
        Real q1 = GetLinkAngle(d.linkIndices[i]);
        if(robot.links[d.linkIndices[i]].type == RobotLink3D::Revolute) {
          //resolve ambiguity of wrapped joint angle
          Real vref = vavg + d.affOffset[i];
          if (i > 0) vref = vavg*d.affScaling[i] / i;
          Real q2 = (q1 > 0? q1-TwoPi : q1+TwoPi);
          if(Abs(vref - q1) > Abs(vref - q2)) 
            vavg += (q2-d.affOffset[i])/d.affScaling[i];
          else
            vavg += (q1-d.affOffset[i])/d.affScaling[i];
        }
        else 
          vavg += (q1-d.affOffset[i])/d.affScaling[i];
      }
      return vavg / d.linkIndices.size();
    }
    break;
  default:
    FatalError("TODO");
    return 0;
    break;
  }
}

Real ODERobot::GetDriverVelocity(int driver) const
{
  const RobotModelDriver& d=robot.drivers[driver];
  switch(d.type) {
  case RobotModelDriver::Normal:
    return GetLinkVelocity(d.linkIndices[0]);
  case RobotModelDriver::Translation:
    {
      Vector3 w,v;
      GetLinkVelocity(d.linkIndices[1],w,v);
      return robot.links[d.linkIndices[0]].w.dot(v);
    }
    break;
  case RobotModelDriver::Rotation:
    {
      Vector3 w,v;
      GetLinkVelocity(d.linkIndices[1],w,v);
      return robot.links[d.linkIndices[0]].w.dot(w);
    }
    break;
  case RobotModelDriver::Affine: 
    {
      Real vavg=0;
      for(size_t i=0;i<d.linkIndices.size();i++)
        vavg += GetLinkVelocity(d.linkIndices[i])/d.affScaling[i];
      return vavg / d.linkIndices.size();
    }
    break;
  default:
    FatalError("TODO");
    return 0;
    break;
  }
}

void ODERobot::AddDriverTorque(int driver,Real t)
{
  const RobotModelDriver& d=robot.drivers[driver];
  switch(d.type) {
  case RobotModelDriver::Normal:
    AddLinkTorque(d.linkIndices[0],t);
    break;
  case RobotModelDriver::Translation:
    {
      Assert(bodyID[d.linkIndices[1]] != NULL);
      Vector3 f=robot.links[d.linkIndices[0]].w*t;
      dBodyAddForce(bodyID[d.linkIndices[1]],f.x,f.y,f.z);
    }
    break;
  case RobotModelDriver::Rotation:
    {
      Assert(bodyID[d.linkIndices[1]] != NULL);
      Vector3 m=robot.links[d.linkIndices[0]].w*t;
      dBodyAddTorque(bodyID[d.linkIndices[1]],m.x,m.y,m.z);
    }
    break;
  case RobotModelDriver::Affine:
    {
      for(size_t i=0;i<d.linkIndices.size();i++)
        AddLinkTorque(d.linkIndices[i],t*d.affScaling[i]);
    }
    break;
  default:
    FatalError("TODO");
  }
}

void ODERobot::SetDriverFixedVelocity(int driver,Real vel,Real tmax)
{
  const RobotModelDriver& d=robot.drivers[driver];
  switch(d.type) {
  case RobotModelDriver::Normal:
    SetLinkFixedVelocity(d.linkIndices[0],vel,tmax);
    break;
  case RobotModelDriver::Affine:
    for(size_t i=0;i<d.linkIndices.size();i++) {
      SetLinkFixedVelocity(d.linkIndices[i],vel*d.affScaling[i],tmax);
    }
    break;
  default:
    FatalError("TODO");
  }
}

void ODERobot::AddDriverTorques(const Config& t)
{
  Assert(t.n == (int)robot.drivers.size());
  for(int i=0;i<t.n;i++)
    AddDriverTorque(i,t(i));
}


bool ODERobot::ReadState(File& f)
{
  //int initPos = f.Position();
  for(size_t i=0;i<robot.links.size();i++) {
    if(bodyID[i] == NULL)  continue;
    dReal w[3],v[3];
    dReal pos[3];
    dReal q[4];
    dReal frc[3];
    dReal trq[3];
    if(!ReadArrayFile(f,pos,3)) return false;
    if(!ReadArrayFile(f,q,4)) return false;
    if(!ReadArrayFile(f,w,3)) return false;
    if(!ReadArrayFile(f,v,3)) return false;
    if(!ReadArrayFile(f,frc,3)) return false;
    if(!ReadArrayFile(f,trq,3)) return false;

    dBodySetPosition(bodyID[i],pos[0],pos[1],pos[2]);
    dBodySetQuaternion(bodyID[i],q);
    //need to do this to avoid the normalization
    dReal* bq = (dReal*)dBodyGetQuaternion(bodyID[i]);
    for(int j=0;j<4;j++)
      bq[j] = q[j];
    dBodySetLinearVel(bodyID[i],v[0],v[1],v[2]);
    dBodySetAngularVel(bodyID[i],w[0],w[1],w[2]);
    dBodySetForce(bodyID[i],frc[0],frc[1],frc[2]);
    dBodySetTorque(bodyID[i],trq[0],trq[1],trq[2]);
  }

  /*
  File f2;
  f2.OpenData();
  WriteState(f2);
  char* d1=(char*)f.GetDataBuffer();
  char* d2=(char*)f2.GetDataBuffer();
  int l1=f.Position()-initPos;
  int l2=f2.Position();
  if(l1 != l2) {
    FatalError("ODERobot::Write/ReadState() do not read the same number of bits %d vs %d!",l1,l2);
  }
  for(int i=0;i<l1;i++) {
    if(d1[initPos+i] != d2[i]) {
      printf("Invalid byte written at position %d\n",i);
      FatalError("ODERobot::Write/ReadState() do not work correctly!");
    }
  }
  */
  return true;
}

bool ODERobot::WriteState(File& f) const
{
  for(size_t i=0;i<robot.links.size();i++) {
    if(bodyID[i] == NULL)  continue;
    const dReal* pos=dBodyGetPosition(bodyID[i]);
    const dReal* q=dBodyGetQuaternion(bodyID[i]);
    const dReal* v=dBodyGetLinearVel(bodyID[i]);
    const dReal* w=dBodyGetAngularVel(bodyID[i]);
    //do we need this?
    const dReal* frc=dBodyGetForce(bodyID[i]);
    const dReal* trq=dBodyGetTorque(bodyID[i]);

    if(!WriteArrayFile(f,pos,3)) return false;
    if(!WriteArrayFile(f,q,4)) return false;
    if(!WriteArrayFile(f,w,3)) return false;
    if(!WriteArrayFile(f,v,3)) return false;
    if(!WriteArrayFile(f,frc,3)) return false;
    if(!WriteArrayFile(f,trq,3)) return false;
  }
  return true;
}

/*
void ODERobot::InterpolateState(const Vector& x,const Vector& y,Real u)
{
  Assert(x.n == (int)robot.links.size()*30);   //position, rotation, velocity, ang velocity, last transform
  Assert(y.n == (int)robot.links.size()*30);   //position, rotation, velocity, ang velocity, last transform

  int k=0;
  for(size_t i=0;i<robot.links.size();i++) {
    RigidTransform Tx,Txold,Ty,Tyold;
    Vector3 wx,vx,wy,vy;
    for(int p=0;p<3;p++)
      for(int q=0;q<3;q++) {
        Tx.R(p,q)=x(k+p*3+q);
        Ty.R(p,q)=y(k+p*3+q);
      }
    k += 9;
    Tx.t.set(x(k),x(k+1),x(k+2));
    Ty.t.set(y(k),y(k+1),y(k+2));
    k+=3;
    wx.set(x(k),x(k+1),x(k+2));
    wy.set(y(k),y(k+1),y(k+2));
    k+=3;
    vx.set(x(k),x(k+1),x(k+2));
    vy.set(y(k),y(k+1),y(k+2));
    k+=3;
    for(int p=0;p<3;p++)
      for(int q=0;q<3;q++) {
        Txold.R(p,q)=x(k+p*3+q);
        Tyold.R(p,q)=y(k+p*3+q);
      }
    k += 9;
    Txold.t.set(x(k),x(k+1),x(k+2));
    Tyold.t.set(y(k),y(k+1),y(k+2));
    k+=3;

    RigidTransform T;
    Vector3 w,v;
    interpolate(Tx,Ty,u,T);
    interpolate(wx,wy,u,w);
    interpolate(vx,vy,u,v);
    for(int p=0;p<3;p++)
      for(int q=0;q<3;q++) 
        Assert(IsFinite(T.R(p,q)));
    for(int p=0;p<3;p++) {
      Assert(IsFinite(T.t[p]));
      Assert(IsFinite(w[p]));
      Assert(IsFinite(v[p]));
    }

    SetLinkTransform(i,T);
    SetLinkVelocity(i,w,v);
  }
  Assert(k == x.n);
}
*/

} //namespace Klampt