#ifndef ROBOT_CSPACE_H
#define ROBOT_CSPACE_H

#include <Klampt/Modeling/World.h>
#include <Klampt/Modeling/GeneralizedRobot.h>
#include "PlannerSettings.h"
#include <KrisLibrary/planning/CSpaceHelpers.h>
#include <KrisLibrary/planning/RigidBodyCSpace.h>
#include <KrisLibrary/utils/ArrayMapping.h>

/** @defgroup Planning */

/** @ingroup Planning
 * @brief Implements a basic robot configuration space with only joint limit
 * constraint testing. 
 *
 * Note: floating joint translations are not sampled properly.
 *
 * Weighted metrics are implemented by filling out the following:
 * jointWeights: weights each deviation of the robot's joints.
 * floatingRotationWeight: additionally weights the rotation components of
 *   floating joints.
 *
 * The 'XRadiusScale' members should be filled out if SampleNeigborhood
 * should use a different metric for sampling.
 *
 * This class implements the proper geodesic for different robot
 * joint types (e.g. floating joints.)
 */
class RobotCSpace : public GeodesicCSpace
{
public:
  RobotCSpace(Robot& robot);
  RobotCSpace(const RobotCSpace& space);
  virtual int NumDimensions();
  virtual string VariableName(int i);
  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& out);
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual Real Distance(const Config& x,const Config& y);
  virtual void Properties(PropertyMap&);

  virtual void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx);
  virtual void InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx);
  virtual void InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx);
  virtual void InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx);
  virtual void Integrate(const Config& a,const Vector& da,Config& b);

  Robot& robot;
  //optional: can edit weights for distance metric and neighborhood sampling
  Real norm;
  vector<Real> jointWeights;
  Real floatingRotationWeight;
  vector<Real> jointRadiusScale;
  Real floatingRotationRadiusScale;
};


/** @ingroup Planning
 * @brief A CSpace for just a few dofs of a robot.  Slightly faster than using
 * a regular RobotCSpace then picking out subvectors.
 */
class ActiveRobotCSpace : public GeodesicCSpace 
{
public:
  ActiveRobotCSpace(Robot& robot,const ArrayMapping& dofs);
  virtual int NumDimensions();
  virtual string VariableName(int i);
  virtual void Sample(Config& x);
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual Real Distance(const Config& x,const Config& y);
  virtual void Properties(PropertyMap&) const;

  virtual void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx);
  virtual void InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx);
  virtual void InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx);
  virtual void InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx);
  virtual void Integrate(const Config& a,const Vector& da,Config& b);

  Robot& robot;
  ArrayMapping dofs;
  Config xq,yq,tempq;
  vector<int> invMap;
  vector<int> joints;
};



/** @ingroup Planning
 * @brief A cspace consisting of a single robot configuration in a
 * RobotWorld.  Feasibility constraints are joint and collision constraints.
 *
 * Uses WorldPlannerSettings to determine the settings for collision constraints.
 *
 * Allows fixing dofs and ignoring collisions between certain object pairs using the
 * FixDof() / IgnoreCollisions() functions. 
 * IMPORTANT: After you call FixDof / IgnoreCollisions, you must call Init to reset the
 * constraints.
 */
class SingleRobotCSpace : public RobotCSpace
{
 public:
  SingleRobotCSpace(RobotWorld& world,int index,
		    WorldPlannerSettings* settings);
  SingleRobotCSpace(const SingleRobotCSpace& space);

  ///Fixes a single DOF to a given value
  void FixDof(int dof,Real value);
  ///Ignores collisions between world ID a and world ID b
  void IgnoreCollisions(int a,int b);
  void Init();

  virtual ~SingleRobotCSpace() {}
  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual bool IsFeasible(const Config& x);
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b,int obstacle);
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b);
  virtual void Properties(PropertyMap& map);

  virtual void GetJointLimits(Vector& bmin,Vector& bmax);

  bool UpdateGeometry(const Config& x);
  bool CheckJointLimits(const Config& x);
  bool CheckCollisionFree(const Config& x);

  RobotWorld& world;
  int index;
  WorldPlannerSettings* settings;

  vector<pair<int,int> > collisionPairs;
  vector<Geometry::AnyCollisionQuery> collisionQueries;

  vector<int> fixedDofs;
  vector<Real> fixedValues;
  vector<pair<int,int> > ignoreCollisions;
  bool constraintsDirty;
};

/** @ingroup Planning
 * @brief A configuration space for a rigid object, treated like a robot.
 *
 * For compatibility with GeneralizedRobot, the configuration is (x,y,z,rz,ry,rx).
 */
class SingleRigidObjectCSpace: public SE3CSpace
{
 public:
  SingleRigidObjectCSpace(RobotWorld& world,int index,WorldPlannerSettings* settings);
  RigidObject* GetObject() const;  
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b);

  ///Ignores collisino between this object and world ID id
  void IgnoreCollisions(int id);
  void Init();
  bool UpdateGeometry(const Config& x);

  RobotWorld& world;
  int index;
  WorldPlannerSettings* settings;

  vector<pair<int,int> > collisionPairs;
  vector<Geometry::AnyCollisionQuery> collisionQueries;

  bool constraintsDirty;
};


#endif
