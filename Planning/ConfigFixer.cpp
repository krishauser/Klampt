#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "ConfigFixer.h"

ConfigFixer::ConfigFixer()
  :maxIters(1000),d0(0.1),dmax(1.0),dRate(1e-4)
{}

bool ConfigFixer::Fix(CSpace* space,const Config& q0,Config& q)
{
  if(space->IsFeasible(q)) return true;
  Config closest;
  bool found=false;
  int iters;
  Real dcur = Inf;
  int dcuriter = 0;
  Real r = d0, rmax = dmax, dr = (dmax-d0)/maxIters;
  for(iters=0;iters<maxIters;iters++) {
    space->SampleNeighborhood(q0,r,q);
    bool shrink = false;
    if(space->IsFeasible(q)) {
      Real d = space->Distance(q0,q);
      if(d < dcur) {
	shrink = true;
	found=true;
	closest = q;
	//shrink down by midpoint
	space->Midpoint(q0,closest,q);
	while(space->IsFeasible(q) && iters < maxIters) {
	  d = space->Distance(q0,q);
	  closest = q;
	  space->Midpoint(q0,closest,q);
	  iters++;
	}
	dcur = rmax = d;
	dcuriter = iters;
	r = rmax*0.5;
	dr = (rmax-r)/(maxIters-iters);
	LOG4CXX_INFO(KrisLibrary::logger(),"ConfigFixer: found a config with distance "<<dcur);
      }
    }
    if(!shrink) {  //grow r towards rmax
      r += dr;
      if((dcur-r)/Real(iters-dcuriter) < dRate) {
	LOG4CXX_INFO(KrisLibrary::logger(),"ConfigFixer: breaking by rate, current iter "<<iters<<", radius "<<r);
	LOG4CXX_INFO(KrisLibrary::logger(),"ConfigFixer: last iter "<<dcuriter<<", radius "<<dcur);
	break;
      }
    }
  }
  if(found) {
    q = closest;
    LOG4CXX_INFO(KrisLibrary::logger(),"ConfigFixer: closest distance "<<dcur <<", press enter to continue\n");
    if(KrisLibrary::logger()->isEnabledFor(log4cxx::Level::ERROR_INT)) getchar();
    return true;
  }
  return false;
}


