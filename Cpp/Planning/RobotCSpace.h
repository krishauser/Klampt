#ifndef ROBOT_CSPACE_H
#define ROBOT_CSPACE_H

#include <Klampt/Modeling/World.h>
#include <Klampt/Modeling/GeneralizedRobot.h>
#include "PlannerSettings.h"
#include <KrisLibrary/planning/CSpaceHelpers.h>
#include <KrisLibrary/planning/RigidBodyCSpace.h>
#include <KrisLibrary/utils/ArrayMapping.h>

namespace Klampt {

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
  RobotCSpace(RobotModel& robot);
  RobotCSpace(const RobotCSpace& space);
  virtual int NumDimensions() override;
  virtual string VariableName(int i) override;
  virtual void Sample(Config& x) override;
  virtual void SampleNeighborhood(const Config& c,Real r,Config& out) override;
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out) override;
  virtual Real Distance(const Config& x,const Config& y) override;
  virtual void Properties(PropertyMap&) override;

  virtual void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx) override;
  virtual void InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx) override;
  virtual void InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx) override;
  virtual void InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx) override;
  virtual void Integrate(const Config& a,const Vector& da,Config& b) override;

  RobotModel& robot;
  //optional: can edit weights for distance metric and neighborhood sampling
  Real norm;
  vector<Real> jointWeights;
  Real floatingRotationWeight;
  vector<Real> jointRadiusScale;
  Real floatingRotationRadiusScale;
  Real unboundedStdDeviation;
};


/** @ingroup Planning
 * @brief A CSpace for just a few dofs of a robot.  Slightly faster than using
 * a regular RobotCSpace then picking out subvectors.
 */
class ActiveRobotCSpace : public GeodesicCSpace 
{
public:
  ActiveRobotCSpace(RobotModel& robot,const ArrayMapping& dofs);
  virtual int NumDimensions() override;
  virtual string VariableName(int i) override;
  virtual void Sample(Config& x) override;
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out) override;
  virtual Real Distance(const Config& x,const Config& y) override;
  virtual void Properties(PropertyMap&) override;

  virtual void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx) override;
  virtual void InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx) override;
  virtual void InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx) override;
  virtual void InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx) override;
  virtual void Integrate(const Config& a,const Vector& da,Config& b) override;

  RobotModel& robot;
  ArrayMapping dofs;
  Config xq,yq,tempq;
  vector<int> invMap;
  vector<int> joints;
};



/** @ingroup Planning
 * @brief A cspace consisting of a single robot configuration in a
 * WorldModel.  Feasibility constraints are joint and collision constraints.
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
  SingleRobotCSpace(WorldModel& world,int index,
		    WorldPlannerSettings* settings);
  SingleRobotCSpace(const SingleRobotCSpace& space);

  ///Fixes a single DOF to a given value
  void FixDof(int dof,Real value);
  ///Ignores collisions between world ID a and world ID b
  void IgnoreCollisions(int a,int b);
  void Init();

  virtual ~SingleRobotCSpace() {}
  virtual void Sample(Config& x) override;
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x) override;
  virtual bool IsFeasible(const Config& x) override;
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b,int obstacle) override;
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b) override;
  virtual void Properties(PropertyMap& map) override;

  virtual void GetJointLimits(Vector& bmin,Vector& bmax);

  bool UpdateGeometry(const Config& x);
  bool CheckJointLimits(const Config& x);
  bool CheckCollisionFree(const Config& x);

  WorldModel& world;
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
  SingleRigidObjectCSpace(WorldModel& world,int index,WorldPlannerSettings* settings);
  RigidObjectModel* GetObject() const;  
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b) override;

  ///Ignores collisino between this object and world ID id
  void IgnoreCollisions(int id);
  void Init();
  bool UpdateGeometry(const Config& x);

  WorldModel& world;
  int index;
  WorldPlannerSettings* settings;

  vector<pair<int,int> > collisionPairs;
  vector<Geometry::AnyCollisionQuery> collisionQueries;

  bool constraintsDirty;
};

} // namespace Klampt

#endif
