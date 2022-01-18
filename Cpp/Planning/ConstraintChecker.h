#ifndef CONSTRAINT_CHECKER_H
#define CONSTRAINT_CHECKER_H

#include <Klampt/Modeling/Robot.h>
#include <Klampt/Modeling/Terrain.h>
#include <Klampt/Contact/Stance.h>

namespace Klampt {

/** @ingroup Planning
 * @brief Checks for static constraints for a robot at a given stance.
 */
struct ConstraintChecker
{
  static Real ContactDistance(const RobotModel& robot,const Stance& stance);
  static bool HasContact(const RobotModel& robot,const Stance& stance,Real maxDist);
  static bool HasContactVelocity(const RobotModel& robot,const Stance& stance,Real maxErr);
  static bool HasJointLimits(const RobotModel& robot);
  static bool HasVelocityLimits(const RobotModel& robot);
  static bool HasSupportPolygon(const RobotModel& robot,const Stance& stance,const Vector3& gravity,int numFCEdges=4);
  static bool HasSupportPolygon_Robust(const RobotModel& robot,const Stance& stance,const Vector3& gravity,Real robustnessFactor,int numFCEdges=4);
  static bool HasEnvCollision(RobotModel& robot,TerrainModel& env);
  //same as above, but ignores fixed links
  static bool HasEnvCollision(RobotModel& robot,TerrainModel& env,const Stance& stance, const vector<int>& ignoreList);
  static bool HasEnvCollision(RobotModel& robot,TerrainModel& env,const vector<IKGoal>& fixedLinks, const vector<int>& ignoreList);
  static bool HasEnvCollision(RobotModel& robot,TerrainModel& env,const vector<IKGoal>& fixedLinks);
  static bool HasSelfCollision(RobotModel& robot);
  static bool HasTorqueLimits(RobotModel& robot,const Stance& stance,const Vector3& gravity,int numFCEdges=4);
};

} //namespace Klampt

#endif
