#ifndef CONSTRAINT_CHECKER_H
#define CONSTRAINT_CHECKER_H

#include "Modeling/Robot.h"
#include "Modeling/Environment.h"
#include "Contact/Stance.h"

/** @ingroup Planning
 * @brief Checks for static constraints for a robot at a given stance.
 */
struct ConstraintChecker
{
  static Real ContactDistance(const Robot& robot,const Stance& stance);
  static bool HasContact(const Robot& robot,const Stance& stance,Real maxDist);
  static bool HasContactVelocity(const Robot& robot,const Stance& stance,Real maxErr);
  static bool HasJointLimits(const Robot& robot);
  static bool HasVelocityLimits(const Robot& robot);
  static bool HasSupportPolygon(const Robot& robot,const Stance& stance,const Vector3& gravity,int numFCEdges=4);
  static bool HasSupportPolygon_Robust(const Robot& robot,const Stance& stance,const Vector3& gravity,Real robustnessFactor,int numFCEdges=4);
  static bool HasEnvCollision(Robot& robot,Environment& env);
  //same as above, but ignores fixed links
  static bool HasEnvCollision(Robot& robot,Environment& env,const Stance& stance, const vector<int>& ignoreList);
  static bool HasEnvCollision(Robot& robot,Environment& env,const vector<IKGoal>& fixedLinks, const vector<int>& ignoreList);
  static bool HasEnvCollision(Robot& robot,Environment& env,const vector<IKGoal>& fixedLinks);
  static bool HasSelfCollision(Robot& robot);
  static bool HasTorqueLimits(Robot& robot,const Stance& stance,const Vector3& gravity,int numFCEdges=4);
};

#endif
