#ifndef TABULATED_CONTROLLER_H
#define TABULATED_CONTROLLER_H

#include "Controller.h"
#include <KrisLibrary/geometry/GridTable.h>

namespace Klampt {

/** @ingroup Control
 * @brief A controller that reads from a grid of torque/desired
 * configuration commands.
 *
 * If the state is outside of the grid it uses the closest available value.
 *
 * Only realistic for low dimensional problems (q <= 2? 3?)
 */
class TabulatedController : public RobotController
{
 public:
  TabulatedController(RobotModel& robot);
  ~TabulatedController() {}

  ///Can implement arbitrary feature mappings by overloading this
  virtual void StateToFeature(const Config& q,const Vector& dq,Vector& x) const;
  ///Convert feature vector only used during optimization
  virtual void FeatureToState(const Vector& x,Config& q,Vector& dq) const;

  virtual void Update(Real dt);

  bool Load(istream& in);
  bool Save(ostream& out);

  ///Set this to true if torques should be used
  bool torqueMode;
  Geometry::GridTable<Vector> commands;
};

/** @ingroup Control
 * @brief Optimizes the given tabulated controller to reach the desired
 * configuration qdes, with cost weights w, using an MDP.
 */
void OptimizeMDP(TabulatedController& controller,
		 const Config& qdes,const Vector& w,
		 int numTransitionSamples,Real discount=1.0);

} //namespace Klampt

#endif

