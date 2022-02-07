#include <KrisLibrary/spline/PiecewisePolynomial.h>
#include "ParabolicRamp.h"
#include "DynamicPath.h"

namespace Klampt {

inline Spline::PiecewisePolynomial Cast(const ParabolicRamp::ParabolicRamp1D& ramp)
{
  Spline::PiecewisePolynomial res;
  res.segments.resize(3);
  res.timeShift.resize(3);
  res.times.resize(4);
  res.times[0] = 0;
  res.times[1] = ramp.tswitch1;
  res.times[2] = ramp.tswitch2;
  res.times[3] = ramp.ttotal;
  res.segments[0].Resize(3);
  res.segments[0].coef[0] = ramp.x0;
  res.segments[0].coef[1] = ramp.dx0;
  res.segments[0].coef[2] = 0.5*ramp.a1;
  res.timeShift[0] = 0;
  res.segments[1].Resize(2);
  res.segments[1].coef[0] = ramp.Evaluate(ramp.tswitch1);
  res.segments[1].coef[1] = ramp.Derivative(ramp.tswitch1);
  res.timeShift[1] = ramp.tswitch1;
  res.segments[2].Resize(3);
  res.segments[2].coef[0] = ramp.x1;
  res.segments[2].coef[1] = ramp.dx1;
  res.segments[2].coef[2] = 0.5*ramp.a2;
  res.timeShift[2] = ramp.ttotal;

  if(ramp.ttotal == ramp.tswitch2) {
    res.times.erase(--res.times.end());
    res.segments.erase(--res.segments.end());
    res.timeShift.erase(--res.timeShift.end());
  }
  if(ramp.tswitch1 == ramp.tswitch2) {
    res.times.erase(++res.times.begin());
    res.segments.erase(++res.segments.begin());
    res.timeShift.erase(++res.timeShift.begin());
  }
  if(ramp.tswitch1 == 0 && res.segments.size()>1) {
    res.times.erase(res.times.begin());
    res.segments.erase(res.segments.begin());
    res.timeShift.erase(res.timeShift.begin());
  }
  return res;
}

inline Spline::PiecewisePolynomialND Cast(const ParabolicRamp::ParabolicRampND& ramp)
{
  Spline::PiecewisePolynomialND res;
  res.elements.resize(ramp.ramps.size());
  for(size_t i=0;i<ramp.ramps.size();i++)
    res.elements[i] = Cast(ramp.ramps[i]);
  return res;
}

//concatenates the ramps
inline Spline::PiecewisePolynomial Cast(const std::vector<ParabolicRamp::ParabolicRamp1D>& ramps)
{
  assert(!ramps.empty());
  if(ramps.size()==1) return Cast(ramps[0]);
  Spline::PiecewisePolynomial p = Cast(ramps[0]);
  for(size_t i=1;i<ramps.size();i++)
    p.Concat(Cast(ramps[i]),true);
  return p;
}

inline Spline::PiecewisePolynomialND Cast(const std::vector<std::vector<ParabolicRamp::ParabolicRamp1D> >& ramps)
{
  Spline::PiecewisePolynomialND res;
  res.elements.resize(ramps.size());
  for(size_t i=0;i<ramps.size();i++)
    res.elements[i] = Cast(ramps[i]);
  return res;
}

inline Spline::PiecewisePolynomialND Cast(const ParabolicRamp::DynamicPath& path)
{
  Spline::PiecewisePolynomialND res;
  for(size_t i=0;i<path.ramps.size();i++) {
    if(i==0) res = Cast(path.ramps[i]);
    else res.Concat(Cast(path.ramps[i]),true);
  }
  return res;
}

} //namespace Klampt