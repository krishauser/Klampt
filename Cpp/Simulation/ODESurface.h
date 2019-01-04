#ifndef ODE_SURFACE_H
#define ODE_SURFACE_H

/** @ingroup Simulation
 * @brief surface properties for any ODE rigid object, robot link, or fixed
 * object.
 */
struct ODESurfaceProperties
{
  double kRestitution;
  double kFriction;
  double kStiffness;
  double kDamping;
};

#endif
