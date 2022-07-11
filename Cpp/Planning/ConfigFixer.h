#ifndef CONFIG_FIXER_H
#define CONFIG_FIXER_H

#include <KrisLibrary/planning/CSpace.h>

namespace Klampt {

/** @brief A class for "fixing" a configuration by sampling a feasible
 * configuration near it. 
 *
 * Will first find a feasible config by growing radii for maxIters
 * until hitting dmax.  If one is found, then the distance and radii
 * will be shrunk by sampling and bisecting.
 */
class ConfigFixer
{
 public:
  ConfigFixer();
  bool Fix(CSpace* space,const Config& q0,Config& q);

  int maxIters;
  Real d0,dmax;  //radii to sample from: initial, maximum
  Real dRate;    //stop if rate of shrinking towards q decreases past this rate
};

} //namespace Klampt

#endif
