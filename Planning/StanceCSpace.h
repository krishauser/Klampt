#ifndef ROBOT_STANCE_CSPACE_H
#define ROBOT_STANCE_CSPACE_H

#include "ContactCSpace.h"
#include "Contact/Stance.h"
#include <robotics/Stability.h>
#include <robotics/TorqueSolver.h>

/** @brief A configuration space that constrains a robot to the IK constraints
 * in a stance, and checks for stability against gravity.
 *
 * The kinematic contraints of contact are handled by the ContactCSpace superclass.
 * Collisions are handled by the SingleRobotCSpace2 superclass of ContactCSpace.
 *
 * This class adds the functionality of testing two stability conditions:
 * - rigid-body stability
 * - articulated robot stability with torques
 *
 * This supports from-scratch testing of RB equilibrium or batch testing using
 * the SupportPolygon class by calling CalculateSP().  The latter is more
 * expensive at first but each IsFeasible call will be faster.
 *
 * Uing support polygons you can modify the margin using SetSPMargin().
 */
class StanceCSpace : public ContactCSpace
{
 public:
  StanceCSpace(RobotWorld& world,int index,
		WorldPlannerSettings* settings);
  StanceCSpace(const SingleRobotCSpace& space);
  StanceCSpace(const StanceCSpace& space);
  virtual ~StanceCSpace() {}
  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual bool IsFeasible(const Config&);

  ///Sets the current stance for this space
  void SetStance(const Stance& s);
  ///Adds the given hold to the space's stance
  void SetHold(const Hold& h);
  ///Calculates the support polygon for faster equilibrium testing
  void CalculateSP();
  ///Enables robust equilbrium solving by shrinking the margin
  void SetSPMargin(Real margin);
  ///Initializes the torque solver.  This is done automatically, and you only
  ///should use this if you are modifying the torque solver's parameters
  void InitTorqueSolver();

  ///Check if the current robot COM satisfies rigid body equilibrium
  bool CheckRBStability();
  ///Check if the current robot configuration satisfies articulated robot
  ///equilibrium with torque limits
  bool CheckTorqueStability();

  Stance stance;
  Vector3 gravity;
  int numFCEdges;
  bool spCalculated;
  Real spMargin;
  SupportPolygon sp;
  ContactFormation formation;
  TorqueSolver torqueSolver;
};

#endif
