#ifndef TIME_SCALING_H
#define TIME_SCALING_H

#include <KrisLibrary/planning/GeneralizedBezierCurve.h>
#include <KrisLibrary/spline/TimeSegmentation.h>
#include <KrisLibrary/spline/PiecewisePolynomial.h>
#include <KrisLibrary/math/vector.h>
#include <vector>
#include <utility>

namespace Klampt {
  using namespace Math;
  using namespace std;

/** @ingroup Planning
 * @brief Maps time into a given path parameter range (e.g., [0,1]) with 
 * joint space velocity and acceleration bounds.  Stores a piecewise
 * quadratic time scaling. Most users will use the TimeScaledBezierCurve
 * class or OptimizeTimeScaling methods instead.
 *
 * The optimization techniques are numerically stable.
 *
 * Usage: call any of the SolveMinTime routines.  Then, either evaluate the
 * time scale s(t) using the TimeToSegment/TimeToParam routines, or extract
 * s(t) completely as a a piecewise polynomial function using the GetTimeToParam
 * routine. 
 *
 * Note: most Klamp't users will want to use the much more convenient 
 * functions in RobotTimeScaling.h.
 */
class TimeScaling
{
public:
  ///evaluation (params and times structures must be set up first)
  int TimeToSegment(Real t) const;
  Real TimeToParam(Real t) const;
  Real TimeToParam(int segment,Real t) const;
  Real TimeToParamDeriv(int segment,Real t) const;
  Real TimeToParamAccel(int segment,Real t) const;
  int ParamToSegment(Real s) const;
  Real ParamToTime(Real s) const;
  Real ParamToTime(int segment,Real s) const;

  ///Finds an optimal time parameterization s(t) of a path with bounded velocity and
  ///acceleration.  Stores the result in this->params, times, and ds.
  ///
  ///Arguments:
  ///- vmin,vmax: velocity bounds
  ///- amin,amax: acceleration bounds
  ///- paramdivs: a list of path segment parameters s1,...,sN
  ///- dxMins,dxMaxs: a list of 1st derivative bounds on each of the N-1 segments
  ///  [si,si+1]
  ///- ddxMins,ddxMaxs: a list of 2nd derivative bounds on each of the N-1 segments
  ///  [si,si+1].
  ///- ds0, dsEnd: if nonnegative, constrains the start and end
  ///  velocity s'(0) and s'(T), respectively
  ///- velocityLimitedVariables (out): if != NULL, returns a list of pairs (param,dim)
  ///  where the velocity limit on dof dim is active at is params[param].
  ///- accelerationLimitedSegments (out): if != NULL, returns a list of pairs (seg,dim)
  ///  where the acceleration limit on dof dim is active over the range of s from
  ///   [params[seg],params[seg+1]]
  ///
  ///Returns true if a time scaling that satisfies all constraints could be found
  bool SolveMinTime(const Vector& vmin,const Vector& vmax,
		    const Vector& amin,const Vector& amax,
		    const vector<Real>& paramdivs,
		    const vector<Vector>& dxMins,const vector<Vector>& dxMaxs,
		    const vector<Vector>& ddxMins,const vector<Vector>& ddxMaxs,
		    Real ds0=-1,Real dsEnd=-1,
        vector<pair<int,int> >* velocityLimitedVariables=NULL,
        vector<pair<int,int> >* accelerationLimitedSegments=NULL);

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
 * scaling.  Useful for evaluating the scaled curve, and for plotting it.
 * 
 * Users need to set up path to be the geometric path to optimize over,
 * and pathSegments to be the grid of path parameters in the range [0,1].
 *
 * After Optimize, the timeScaling variable will be filled out.
 *
 * Note: most Klamp't users will want to use the much more convenient 
 * functions in RobotTimeScaling.h.
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

  ///Plots time-scaling and constraints at resolution res.  Saves in CSV format.
  void Plot(const char* fn,const Vector& vmin,const Vector& vmax,const Vector& amin,const Vector& amax,Real res=1e-3);

  GeneralizedCubicBezierSpline path;
  Spline::TimeSegmentation pathSegments;  //optionally set to the partial sums of path.durations
  TimeScaling timeScaling;
};

/** @ingroup Planning
 * @brief Optimizes the given path according to velocity and acceleration
 * bounds.  The resulting time scaling is constrained to start and stop at 0 velocity.
 *
 * Note: most Klamp't users will want to use the much more convenient 
 * functions in RobotTimeScaling.h.
 */
bool OptimizeTimeScaling(const GeneralizedCubicBezierSpline& path,
			 const Vector& vmin,const Vector& vmax,
			 const Vector& amin,const Vector& amax,
			 TimeScaling& scaling);

} //namespace Klampt

#endif
