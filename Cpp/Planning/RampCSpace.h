#ifndef RAMP_CSPACE_H
#define RAMP_CSPACE_H

#include <Klampt/Modeling/ParabolicRamp.h>
#include <Klampt/Modeling/DynamicPath.h>
#include <KrisLibrary/planning/CSpace.h>
#include <KrisLibrary/planning/EdgePlanner.h>
#include <KrisLibrary/planning/KinodynamicSpace.h>

namespace Klampt {

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
  virtual int NumDimensions() override;
  virtual bool IsFeasible(const State& s) override;
  virtual void Sample(State& s) override;
  virtual EdgePlannerPtr LocalPlanner(const State& a,const State& b) override;
  virtual Real Distance(const State& x, const State& y) override;
  virtual void Interpolate(const State& x,const State& y,Real u,State& out) override;
  virtual void Properties(PropertyMap& props) override;
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
  virtual const Config& Start() const override;
  virtual const Config& End() const override;
  virtual void Eval(Real u,Config& x) const override;
  virtual Real Length() const override;

  ParabolicRamp::ParabolicRampND ramp;
};

class RampPathInterpolator: public Interpolator
{
public:
  RampPathInterpolator(const ParabolicRamp::DynamicPath& ramp);
  virtual const Config& Start() const override;
  virtual const Config& End() const override;
  virtual void Eval(Real u,Config& x) const override;
  virtual Real Length() const override;

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
  virtual bool IsVisible() override;
  virtual void Eval(Real u,Config& x) const override;
  virtual Real Length() const override;
  virtual const Config& Start() const override { return start; }
  virtual const Config& End() const override { return goal; }
  virtual CSpace* Space() const override { return space; }
  virtual EdgePlannerPtr Copy() const override;
  virtual EdgePlannerPtr ReverseCopy() const override;
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
  virtual bool SegmentFeasible(const ParabolicRamp::Vector& a,const ParabolicRamp::Vector& b) override {
    EdgePlannerPtr e=IsVisible(space,Vector(a),Vector(b));
    if(e) return true; 
    else return false;
  }
  CSpace* space;
};

} //namespace Klampt

#endif //RAMP_CSPACE_H
