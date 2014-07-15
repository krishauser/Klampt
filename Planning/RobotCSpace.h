#ifndef ROBOT_CSPACE_H
#define ROBOT_CSPACE_H

#include "Modeling/World.h"
#include "Modeling/GeneralizedRobot.h"
#include "PlannerSettings.h"
#include <planning/ExplicitCSpace.h>
#include <planning/GeodesicSpace.h>
#include <utils/ArrayMapping.h>
#include <utils/SmartPointer.h>

/** @defgroup Planning */

/** @ingroup Planning
 * @brief Implements a basic robot configuration space with only joint limit
 * constraint testing.  Note: floating joint translations are not sampled
 * properly.
 *
 * Weighted metrics are implemented by filling out the following:
 * jointWeights: weights each deviation of the robot's joints.
 * floatingRotationWeight: additionally weights the rotation components of
 *   floating joints.
 *
 * The 'XRadiusScale' members should be filled out if SampleNeigborhood
 * should use a different metric for sampling.
 */
class RobotCSpace : public CSpace
{
public:
  RobotCSpace(Robot& robot);
  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& out);
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual void Midpoint(const Config& x,const Config& y,Config& out);
  virtual Real Distance(const Config& x,const Config& y);
  virtual bool IsFeasible(const Config& x);
  virtual EdgePlanner* LocalPlanner(const Config& x,const Config& y);

  Robot& robot;
  //optional: can edit weights for distance metric and neighborhood sampling
  Real norm;
  vector<Real> jointWeights;
  Real floatingRotationWeight;
  vector<Real> jointRadiusScale;
  Real floatingRotationRadiusScale;
};

/** @ingroup Planning
 * @brief Implements the proper geodesic for different robot joint types (e.g.,
 * floating joints.
 */
class RobotGeodesicManifold : public GeodesicManifold 
{
public:
  RobotGeodesicManifold(Robot& robot);
  virtual void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx);
  virtual void InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx);
  virtual void InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx);
  virtual void InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx);
  virtual void Integrate(const Config& a,const Vector& da,Config& b);

  Robot& robot;
};

/** @ingroup Planning
 * @brief A CSpace for just a few dofs of a robot.  Slightly faster than using
 * a regular RobotCSpace then picking out subvectors.
 *
 * TODO: implement Sample, IsFeasible, LocalPlanner.
 */
class ActiveRobotCSpace : public CSpace 
{
public:
  ActiveRobotCSpace(Robot& robot,const ArrayMapping& dofs);
  virtual void Sample(Config& x);
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual void Midpoint(const Config& x,const Config& y,Config& out);
  virtual Real Distance(const Config& x,const Config& y);
  virtual bool IsFeasible(const Config& x);
  virtual EdgePlanner* LocalPlanner(const Config& x,const Config& y);

  Robot& robot;
  const ArrayMapping& dofs;
  Config xq,yq,tempq;
  vector<int> invMap;
  vector<int> joints;
};

/** @ingroup Planning
 * @brief A GeodesicManifold for just a few dofs of a robot.
 */
class ActiveRobotGeodesicManifold : public GeodesicManifold 
{
public:
  ActiveRobotGeodesicManifold(Robot& robot,const ArrayMapping& dofs);
  virtual void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx);
  virtual void InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx);
  virtual void InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx);
  virtual void InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx);
  virtual void Integrate(const Config& a,const Vector& da,Config& b);

  Robot& robot;
  const ArrayMapping& dofs;
  Config xq,yq,tempq;
  vector<int> invMap;
  vector<int> joints;
};


/** @ingroup Planning
 * @brief A cspace consisting of a single robot configuration in a
 * RobotWorld.  Feasibility constraints are joint and collision constraints.
 *
 * Uses WorldPlannerSettings to determine the settings for collision constraints.
 */
class SingleRobotCSpace : public ExplicitCSpace
{
 public:
  SingleRobotCSpace(RobotWorld& world,int index,
		    WorldPlannerSettings* settings);
  SingleRobotCSpace(const SingleRobotCSpace& space);

  virtual ~SingleRobotCSpace() {}
  virtual int NumDimensions() const;
  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual int NumObstacles();
  virtual string ObstacleName(int obstacle);
  virtual bool IsFeasible(const Config&,int obstacle);
  virtual bool IsFeasible(const Config&);
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b,int obstacle);
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b);
  virtual void CheckObstacles(const Config& x,vector<bool>& infeasible);

  virtual Real Distance(const Config& x, const Config& y);
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual void Midpoint(const Config& x,const Config& y,Config& out);

  virtual void GetJointLimits(Vector& bmin,Vector& bmax);
  virtual Real FreeWorkspaceBound(const Config& x,int constraint);
  virtual Real WorkspaceMovementBound(const Config& x,const Vector& dx,int constraint);

  void InitializeCollisionPairs();
  bool CheckJointLimits(const Config& x);
  bool CheckCollisionFree();
  Robot* GetRobot() const;

  RobotWorld& world;
  int index;
  WorldPlannerSettings* settings;

  bool collisionPairsInitialized;
  vector<pair<int,int> > collisionPairs;
  vector<Geometry::AnyCollisionQuery> collisionQueries;
};

/** @ingroup Planning
 * @brief A slightly more sophisticated single-robot cspace.
 * Allows fixing dofs and ignoring collisions between certain object pairs.
 */
class SingleRobotCSpace2 : public SingleRobotCSpace
{
 public:
  SingleRobotCSpace2(RobotWorld& world,int index,
		    WorldPlannerSettings* settings);
  SingleRobotCSpace2(const SingleRobotCSpace& space);
  virtual ~SingleRobotCSpace2() {}
  void FixDof(int dof,Real value);
  void IgnoreCollisions(int a,int b);
  void Init();
  virtual int NumDimensions() const;
  virtual int NumObstacles();
  virtual string ObstacleName(int obstacle);
  virtual bool IsFeasible(const Config&);
  virtual bool IsFeasible(const Config&,int obstacle);
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b,int obstacle);
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b);
  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);

  vector<int> fixedDofs;
  vector<Real> fixedValues;
  vector<pair<int,int> > ignoreCollisions;
};

/** @ingroup Planning
 * @brief A configuration space for a rigid object, treated like a robot.
 *
 * For compatibility with GeneralizedRobot, the configuration is (x,y,z,rz,ry,rx).
 */
class SingleRigidObjectCSpace: public CSpace
{
 public:
  SingleRigidObjectCSpace(RobotWorld& world,int index,WorldPlannerSettings* settings);
  RigidObject* GetObject() const;
  virtual int NumDimensions() const { return 6; }
  virtual bool IsFeasible(const Config& q);

  virtual void Sample(Config& q);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& q);
  virtual Real Distance(const Config& a,const Config& b);
  virtual void Interpolate(const Config& a,const Config& b,Real u,Config& out);
  virtual void Midpoint(const Config& a,const Config& b,Config& out);
  virtual EdgePlanner* LocalPlanner(const Config& x,const Config& y);

  void InitializeCollisionPairs();
  bool CheckCollisionFree(const RigidTransform& T);

  RobotWorld& world;
  int index;
  WorldPlannerSettings* settings;

  bool collisionPairsInitialized;
  vector<pair<int,int> > collisionPairs;
  vector<Geometry::AnyCollisionQuery> collisionQueries;
};

/** @ingroup Planning
 * @brief A multi-robot coordination space.  Not tested thoroughly.
 */
class MultiRobotCSpace : public CSpace
{
 public:
  MultiRobotCSpace(RobotWorld& world,WorldPlannerSettings* settings);
  MultiRobotCSpace(const MultiRobotCSpace&);
  virtual ~MultiRobotCSpace() {}
  virtual void AddRobot(int index);
  virtual void AddRigidObject(int index);
  virtual void InitRobots(const vector<int>& indices);

  virtual int NumDimensions() const;
  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual bool IsFeasible(const Config&);
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b);

  virtual Real Distance(const Config& x, const Config& y);
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual void Midpoint(const Config& x,const Config& y,Config& out);

  RobotWorld& world;
  WorldPlannerSettings* settings;
  ///All added objects go into this GeneralizedRobot object
  GeneralizedRobot robot;
  ///Store the mapping from robot elements to world IDs
  vector<int> robotElementIDs;
  ///Each element has its own CSpace
  vector<SmartPointer<CSpace> > elementSpaces;
};


#endif
