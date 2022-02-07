#ifndef OPERATIONAL_SPACE_CONTROLLER_H
#define OPERATIONAL_SPACE_CONTROLLER_H

#include "Controller.h"
#include <Klampt/Sensing/StateEstimator.h>
#include <KrisLibrary/robotics/IK.h>
#include <KrisLibrary/robotics/Contact.h>
#include <KrisLibrary/utils/SmartPointer.h>

namespace Klampt {

//task is q''[indices] = ddqdes
struct JointAccelTask
{
  vector<int> indices;
  Vector ddqdes;
  Real weight;
};

//task is x''[indices] = ddxdes
//position terms go first like in IKFunctions
struct WorkspaceAccelTask
{
  IKGoal workspace;
  Vector ddxdes;
  Real weight;
};

//task is (R*cm'')[1:numAxes] = ddxdes
struct COMAccelTask
{
  Matrix3 R;
  int numAxes;
  Vector ddxdes;
  Real weight;
};

//task is T[indices] = Tdes
struct TorqueTask
{
  vector<int> indices;
  Vector Tdes;
  Real weight;
};

//Task is Af = fdes
//contacts point into the robot and are given the LOCAL frame
struct ContactForceTask
{
  vector<int> links;
  vector<ContactPoint> contacts;
  Matrix A;
  Vector fdes;
  Real weight,penetrationWeight;
};

/** @ingroup Control
 * @brief A combination of multiple "tasks" that define a weighted
 * optimization objective for the joint torques.
 *
 * For a contact-free robot, we have
 *   min_{T} gj(q''(T)) + gw(x''(T)) + gt(T) 
 *   s.t. |T| <= Tmax
 * where
 * - gj(q'') = wj||(q'' - ddqdes)[indices]||^2
 * - gw(x'') = ww||(x'' - ddxdes)[indices]||^2
 * - gt(T) = wt||(T - Tdes)[indices]||^2
 * and
 * - q''(T) = B^-1(Jd^T T + Jext^T*fext) + ddq0
 * - x''(T) = Jx q''(T) + Jx' q'
 * Jd is the jacobian of the drivers in robot and Jx is the jacobian of the pts in x
 *
 * For robots with contacts, we must have an estimate for the contact
 * points in the list of ContactForceTasks.  The problem is now:
 *
 *   min_{T,f} gj(q''(T,f)) + gw(x''(T,f)) + gt(T,f) + gf(f)
 *     s.t. |T| <= Tmax
 *          f >= 0  (represent f as a list of friction cone vectors)
 *          xf''(T,f) = 0  (contacts don't move into the object)
 * with
 * - gf(f) = wf||Af-fdes||^2
 * - q''(T,f) = B^-1(Jd^T T + Jext^T*fext + Jf^T*f) + ddq0
 * - x''(T,f) as before 
 * - xf''(T,f) = Jf q''(T,f) + Jf' q' 
 * and Jf the jacobian of force points in their normal directions.
 * 
 * If the xf''=0 constraint is not solvable,
 * we add a penalty for xf'' movement and treat it as another workspace task.
 *
 * Warning: not tested thoroughly.
 */
struct OperationalSpaceController : public RobotController
{
  OperationalSpaceController(RobotModel& robot);

  virtual const char* Type() const { return "OperationalSpaceController"; }
  virtual void Update(Real dt);
  virtual void Reset();
  /*
  virtual bool ReadState(File& f) {
    if(!ReadFile(f,time)) return false;
    return true;
  }
  virtual bool WriteState(File& f) const {
    if(!WriteFile(f,time)) return false;
    return true;
  }
  */

  //the bulk of the work is done here
  void TasksToTorques(Vector& t);
  bool IsValid() const;

  SmartPointer<IntegratedStateEstimator> stateEstimator;
  Vector3 gravity;
  vector<JointAccelTask> jointTasks;
  vector<WorkspaceAccelTask> workspaceTasks;
  vector<COMAccelTask> comTasks;
  vector<TorqueTask> torqueTasks;
  vector<ContactForceTask> contactForceTasks;
};

} //namespace Klampt

#endif
