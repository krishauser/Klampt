#ifndef RAMP_CSPACE_H
#define RAMP_CSPACE_H

#include <Klampt/Modeling/ParabolicRamp.h>
#include <Klampt/Modeling/DynamicPath.h>
#include <KrisLibrary/planning/CSpace.h>
#include <KrisLibrary/planning/EdgePlanner.h>
#include <KrisLibrary/planning/KinodynamicSpace.h>

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
  virtual int NumDimensions() const;
  virtual bool IsFeasible(const State& s);
  virtual void Sample(State& s);
  virtual EdgePlannerPtr LocalPlanner(const State& a,const State& b);
  virtual Real Distance(const State& x, const State& y);
  virtual void Interpolate(const State& x,const State& y,Real u,State& out);
  virtual void Properties(PropertyMap& props) const;
  bool IsFeasible(const Config& q,const Config& dq);

  CSpace* cspace;
  std::vector<Real> qMin,qMax;
  std::vector<Real> velMax,accMax;
  Real visibilityTolerance;
};

class RampInterpolator: public Interpolator
{
public:
  RampInterpolator(const ParabolicRamp::ParabolicRampND& ramp);
  virtual const Config& Start() const;
  virtual const Config& End() const;
  virtual CSpace* Space() const;
  virtual void Eval(Real u,Config& x) const;
  virtual Real Length() const;

  ParabolicRamp::ParabolicRampND ramp;
};

class RampPathInterpolator: public Interpolator
{
public:
  RampPathInterpolator(const ParabolicRamp::DynamicPath& ramp);
  virtual const Config& Start() const;
  virtual const Config& End() const;
  virtual CSpace* Space() const;
  virtual void Eval(Real u,Config& x) const;
  virtual Real Length() const;

  ParabolicRamp::DynamicPath path;
};

///Edge planner class for the RampCSpaceAdaptor
class RampEdgeChecker : public EdgePlanner
{
public:
  RampEdgeChecker(RampCSpaceAdaptor* _space,const State& a,const State& b);
  RampEdgeChecker(RampCSpaceAdaptor* _space,const ParabolicRamp::ParabolicRampND& ramp);
  RampEdgeChecker(RampCSpaceAdaptor* _space,const ParabolicRamp::DynamicPath& path);
  virtual ~RampEdgeChecker() {}
  virtual bool IsVisible();
  virtual void Eval(Real u,Config& x) const;
  virtual Real Length() const;
  virtual const Config& Start() const { return start; }
  virtual const Config& End() const { return goal; }
  virtual CSpace* Space() const { return space; }
  virtual EdgePlannerPtr Copy() const;
  virtual EdgePlannerPtr ReverseCopy() const;
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
    virtual bool ConfigFeasible(const ParabolicRamp::Vector& x) { return space->IsFeasible(Vector(x)); }
  virtual bool SegmentFeasible(const ParabolicRamp::Vector& a,const ParabolicRamp::Vector& b) {
    EdgePlannerPtr e=IsVisible(space,Vector(a),Vector(b));
    if(e) return true; 
    else return false;
  }
  CSpace* space;
};


#endif //RAMP_CSPACE_H
