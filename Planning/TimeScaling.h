#ifndef TIME_SCALING_H
#define TIME_SCALING_H

#include <KrisLibrary/planning/GeneralizedBezierCurve.h>
#include <KrisLibrary/spline/TimeSegmentation.h>
#include <KrisLibrary/spline/PiecewisePolynomial.h>
#include <KrisLibrary/math/vector.h>
#include <vector>
using namespace Math;
using namespace std;

/** @ingroup Planning
 * @brief Maps time into a given path parameter range (e.g., [0,1]).  Stores a 
 * piecewise quadratic time scaling.
 *
 * The optimization techniques are numerically stable.
 */
class TimeScaling
{
public:
  ///evaluation (structures must be set up first)
  int TimeToSegment(Real t) const;
  Real TimeToParam(Real t) const;
  Real TimeToParam(int segment,Real t) const;
  Real TimeToParamDeriv(int segment,Real t) const;
  Real TimeToParamAccel(int segment,Real t) const;
  int ParamToSegment(Real s) const;
  Real ParamToTime(Real s) const;
  Real ParamToTime(int segment,Real s) const;

  ///Finds an optimal time parameterization of a path with bounded velocity and
  ///acceleration.
  ///Input is the vel/acc bounds, a list of parameters s1,...,sN in paramdivs,
  ///and a list of 1st and 2nd derivative bounds on each of the N-1 segments
  ///[si,si+1].  If ds0 and dsEnd are given, this constrains the start and end
  ///velocity
  bool SolveMinTime(const Vector& vmin,const Vector& vmax,
		    const Vector& amin,const Vector& amax,
		    const vector<Real>& paramdivs,
		    const vector<Vector>& dxMins,const vector<Vector>& dxMaxs,
		    const vector<Vector>& ddxMins,const vector<Vector>& ddxMaxs,
		    Real ds0=-1,Real dsEnd=-1);

  ///Same as above, but uses a more effective derivative bounding technique assuming
  ///a cartesian space.
  bool SolveMinTime(const Vector& vmin,const Vector& vmax,
		    const Vector& amin,const Vector& amax,
		    const GeneralizedCubicBezierSpline& path,
		    Real ds0=-1,Real dsEnd=-1);

  ///convenience approximation function -- assumes all segments are monotonic
  bool SolveMinTime(const Vector& vmin,const Vector& vmax,
		    const Vector& amin,const Vector& amax,
		    const vector<Real>& paramdivs,
		    const vector<Vector>& dxs,
		    Real ds0=-1,Real dsEnd=-1);

  ///Sort of improves the conditioning of the time scaling near singularities --
  ///support is experimental
  void ConditionMinTime(vector<Real>& paramdivs,vector<Vector>& dxs,
			vector<Vector>& dxMins,vector<Vector>& dxMaxs,
			vector<Vector>& ddxMins,vector<Vector>& ddxMaxs);

  ///Same as above except that the time parameterization is on the arc-length
  ///version of the given path.  The user must take care of the arc-length separately.
  bool SolveMinTimeArcLength(const Vector& vmin,const Vector& vmax,
			     const Vector& amin,const Vector& amax,
			     const vector<Real>& paramdivs,
			     const vector<Vector>& dxMins,const vector<Vector>& dxMaxs,
			     const vector<Vector>& ddxMins,const vector<Vector>& ddxMaxs,
			     Real ds0=-1,Real dsEnd=-1);
  
  ///convenience approximation function -- assumes all segments are monotonic
  bool SolveMinTimeArcLength(const Vector& vmin,const Vector& vmax,
			     const Vector& amin,const Vector& amax,
			     const vector<Real>& paramdivs,
			     const vector<Vector>& dxs,
			     Real ds0=-1,Real dsEnd=-1);

  ///Retreives the piecewise polynomial time-to-parameter mapping
  void GetTimeToParam(Spline::PiecewisePolynomial& poly) const;

  Spline::TimeSegmentation params,times;
  vector<Real> ds;
};



/** @ingroup Planning
 * @brief A convenience class that stores a Bezier curve and its time
 * scaling.
 */
class TimeScaledBezierCurve
{
public:
  bool OptimizeTimeScaling(const Vector& vmin,const Vector& vmax,const Vector& amin,const Vector& amax);
  void GetPiecewiseLinear(std::vector<Real>& times,std::vector<Config>& milestones) const;
  void GetDiscretizedPath(Real dt,std::vector<Config>& milestones) const;
  Real EndTime() const;
  void Eval(Real t,Vector& x) const;
  void Deriv(Real t,Vector& dx) const;
  void Accel(Real t,Vector& ddx) const;

  //plots time-scaling constraints at resolution res
  void Plot(const char* fn,const Vector& vmin,const Vector& vmax,const Vector& amin,const Vector& amax,Real res=1e-3);

  GeneralizedCubicBezierSpline path;
  Spline::TimeSegmentation pathSegments;  //optionally set to the partial sums of path.durations
  TimeScaling timeScaling;
};

/** @ingroup Planning
 * @brief Optimizes the given path according to velocity and acceleration
 * bounds.
 */
bool OptimizeTimeScaling(const GeneralizedCubicBezierSpline& path,
			 const Vector& vmin,const Vector& vmax,
			 const Vector& amin,const Vector& amax,
			 TimeScaling& scaling);

#endif
