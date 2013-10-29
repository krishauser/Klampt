#ifndef RAMP_CSPACE_H
#define RAMP_CSPACE_H

#include "Modeling/ParabolicRamp.h"
#include "Modeling/DynamicPath.h"
#include <planning/CSpace.h>
#include <planning/KinodynamicCSpace.h>

/** @brief A CSpace where configurations are given by (q,dq) config, velocity
 * pairs.  Local paths are time-optimal acceleration bounded curves.
 *
 * Configurations lie in a kinematically constrained cspace given on
 * initialization.  Velocities are bounded.
 */
class RampCSpaceAdaptor : public CSpace
{
public:
  RampCSpaceAdaptor(CSpace* cspace,const Vector& velMax,const Vector& accMax);
  virtual bool IsFeasible(const State& s);
  virtual void Sample(State& s);
  virtual EdgePlanner* LocalPlanner(const State& a,const State& b);
  virtual Real Distance(const State& x, const State& y);
  virtual void Interpolate(const State& x,const State& y,Real u,State& out);
  bool IsFeasible(const Config& q,const Config& dq);

  CSpace* cspace;
  std::vector<Real> qMin,qMax;
  std::vector<Real> velMax,accMax;
  Real visibilityTolerance;
};


///Edge planner class for the RampCSpaceAdaptor
class RampEdgePlanner : public EdgePlanner
{
public:
  RampEdgePlanner(RampCSpaceAdaptor* _space,const State& a,const State& b);
  RampEdgePlanner(RampCSpaceAdaptor* _space,const ParabolicRamp::ParabolicRampND& ramp);
  RampEdgePlanner(RampCSpaceAdaptor* _space,const ParabolicRamp::DynamicPath& path);
  virtual ~RampEdgePlanner() {}
  virtual bool IsVisible();
  virtual void Eval(Real u,Config& x) const;
  virtual const Config& Start() const { return start; }
  virtual const Config& Goal() const { return goal; }
  virtual CSpace* Space() const { return space; }
  virtual EdgePlanner* Copy() const;
  virtual EdgePlanner* ReverseCopy() const;
  Real Duration() const;
  bool IsValid() const;

  RampCSpaceAdaptor* space;
  ParabolicRamp::DynamicPath path;
  State start,goal;
  int checked;
};

///adapter for the ParabolicRamp feasibility checking routines
class CSpaceFeasibilityChecker : public ParabolicRamp::FeasibilityCheckerBase
{
public:
  CSpaceFeasibilityChecker(CSpace* _space) : space(_space) {}
    virtual bool ConfigFeasible(const ParabolicRamp::Vector& x) { return space->IsFeasible(x); }
  virtual bool SegmentFeasible(const ParabolicRamp::Vector& a,const ParabolicRamp::Vector& b) {
    EdgePlanner* e=IsVisible(space,a,b);
    if(e) { delete e; return true; }
    else return false;
  }
  CSpace* space;
};


#endif //RAMP_CSPACE_H
