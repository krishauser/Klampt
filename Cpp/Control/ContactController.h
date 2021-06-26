#ifndef CONTACT_CONTROLLER_H
#define CONTACT_CONTROLLER_H

#include "OperationalSpaceController.h"
#include "JointTrackingController.h"
#include <KrisLibrary/robotics/TorqueSolver.h>

namespace Klampt {

/** @brief Makes a joint tracking controller 'aware' of a contact formation.
 */
struct ContactJointTrackingController : public RobotController
{
  ContactJointTrackingController(RobotModel& robot,const SmartPointer<JointTrackingController>& base);
  void SetGravity(const Vector3& gravity);
  //contact estimates are given in the LOCAL frame
  void SetContactEstimate(const ContactFormation& contacts,Real penetrationWeight=100);
  //uses lowest point on robot to select contacts (standing on a plane)
  void SetFlatContactEstimate(Real kFriction,Real penetrationWeight=100,Real tol=1e-3);
  //uses contact sensors to produce the contact estimate
  void SenseContactEstimate(Real kFriction,Real penetrationWeight=100);
  //penalizes large torques
  void SetTorqueRegularization(Real regularizationFactor);
  void DesiredToAccel(Real dt,const Config& qdes,const Vector& dqdes,Vector& ddqdes);

  virtual void Update(Real dt);
  virtual void Reset();
  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;

  SmartPointer<JointTrackingController> base;
  OperationalSpaceController opSpaceController;
  Config qdes_last,dqdes_last;
};

} //namespace Klampt

#endif
