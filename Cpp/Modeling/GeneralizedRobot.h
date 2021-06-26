#ifndef GENERALIZED_ROBOT_H
#define GENERALIZED_ROBOT_H

#include "Robot.h"
#include "RigidObject.h"
#include "World.h"
#include <map>

namespace Klampt {

/** @ingroup Modeling
 * @brief A collection of robots and objects that can be treated like one "big robot".
 *
 * Allows treating configuration in a single vector. Configurations of objects are
 * specified as 6 DOF (x,y,z,rz,ry,rx).
 */
class GeneralizedRobotModel
{
 public:
  GeneralizedRobotModel();
  GeneralizedRobotModel(WorldModel& world);
  int Add(RobotModel* robot,const char* name=NULL);
  int Add(RigidObjectModel* object,const char* name=NULL);
  void Remove(int id);
  void Remove(const char* name) { Remove(ID(name)); }
  void Remove(RobotModel* robot) { Remove(ID(robot)); }
  void Remove(RigidObjectModel* object) { Remove(ID(object)); }
  ///Returns the array index of the given named element
  int ID(const char* name) const;
  ///Returns the array index of the given robot
  int ID(RobotModel* robot) const;
  ///Returns the array index of the given object
  int ID(RigidObjectModel* object) const;
  ///Returns the total number of DOF
  int NumDof() const;
  ///Returns a name for the given DOF index
  string DofName(int index) const;
  ///Returns the ID for the indicated DOF
  int DofToID(int index) const;
  ///Returns the DOF index range associated with the given id
  pair<int,int> Dofs(int id) const;
  ///Returns the DOF index range associated with the given named element 
  pair<int,int> Dofs(const char* name) const { return Dofs(ID(name)); }
  ///Returns the DOF index range associated with the given robot
  pair<int,int> Dofs(RobotModel* robot) const { return Dofs(ID(robot)); }
  ///Returns the DOF index range associated with the given object
  pair<int,int> Dofs(RigidObjectModel* object) const { return Dofs(ID(object)); }
  ///Returns the DOF index associated with the element of the given id, offset by link
  int Dof(int id,int link) const { return Dofs(id).first+link; }
  ///Returns the DOF index associated with the element of the given id, offset by link
  int Dof(RobotModel* robot,int link) const { return Dof(ID(robot),link); }
  ///Returns the DOF index associated with the element of the given name
  int Dof(const char* name,int link) const { return Dof(ID(name),link); }
  ///Sets a joint configuration of all the elements
  void SetConfig(const Config& q);
  ///Sets a joint velocity of all the elements
  void SetVelocity(const Vector& v);
  ///Gets the joint configuration of all the elements
  void GetConfig(Config& q) const;
  ///Gets the joint velocity of all the elements
  void GetVelocity(Vector& v) const;
  ///Updates the geometry of all the elements
  void UpdateGeometry();
  ///Splits the joint configuration to a list of configurations of individual elements
  void Split(const Config& q,vector<Config>& qsplit) const;
  ///Same as split, but generates references to q in order to minimize copying
  void SplitRefs(const Config& q,vector<Config>& qsplit) const;
  ///Joins a list of configurations of indivdual elements into a single joint configuration
  void Join(const vector<Config>& qsplit,Config& q) const;
  ///Interpolates two configurations
  void Interpolate(const Config& a,const Config& b,Real u,Config& out) const;
  ///Returns the velocity vector that will move from a to b at the parameter u
  void InterpolateVelocity(const Config& a,const Config& b,Real u,Vector& dq) const;
  //Integrates a velocity vector dq from q to obtain the configuration out
  void Integrate(const Config& q,const Vector& dq,Config& out) const;
  ///Returns a distance metric between two configurations
  Real Distance(const Config& a,const Config& b,Real floatingRotationWeight=1.0) const;
  ///Gets joint limits among all objects
  void GetJointLimits(Config& qmin,Config& qmax) const;
  ///Gets the overall center of mass 
  Vector3 GetCOM() const;
  ///Gets the "mega robot" that merges all robots and objects together
  void GetMegaRobot(RobotModel& voltron) const;

  struct Element
  {
    string name;
    RobotModel* robot;
    RigidObjectModel* object;
    ///indices governed by this element are in range [indexStart,indexEnd)
    int indexStart,indexEnd;
  };
  map<int,Element> elements;
};


///Creates a Robot object corresponding to the given object's geometric / dynamic
///characteristics. Configuration is (x,y,z,rz,ry,rx)
void ObjectToRobot(const RigidObjectModel& object,RobotModel& robot);

///Converts a 6-dof configuration of a rigid object to the RigidTransform
void ConfigToTransform(const Vector& q,RigidTransform& T);

///Converts a RigidTransform of an object to a 6-dof configuration 
void TransformToConfig(const RigidTransform& T,Vector& q);

} // namespace Klampt

#endif
