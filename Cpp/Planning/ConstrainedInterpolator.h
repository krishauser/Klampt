#ifndef CONSTRAINED_INTERPOLATOR_H
#define CONSTRAINED_INTERPOLATOR_H

#include <KrisLibrary/math/function.h>
#include <KrisLibrary/planning/GeneralizedBezierCurve.h>
#include <KrisLibrary/optimization/Newton.h>

namespace Klampt {
  using namespace std;

/** @ingroup Planning
 * @brief Construct a polyline between a and b such that each point is near
 * the constraint C(x)=0.
 *
 * The method uses a recursive bisection technique, where the midpoint of
 * each segment is projected to the constraint until termination.  If
 * checkConstraints is true, the feasibility of each projected
 * point is checked.
 *
 * The projection uses a Newton-Raphson solver, capped at maxNewtonIters
 * iterations.  It ensures that each milestone satisfies ||C(x[k])|| <= ftol,
 * and d(x[k],x[k+1])<=xtol.
 *
 * maxGrowth defines the maximum extra distance that the path through a projected configuration can
 * add to the total length of the path.  That is, when going from x1 to x2, the projected midpoint
 * xm is checked so that d(x1,xm) + d(xm,x2) <= (1+maxGrowth)d(x1,x2).
 * To ensure convergence this parameter should be < 1 (default 0.9).
 */
class ConstrainedInterpolator
{
 public:
  ConstrainedInterpolator(CSpace* space,VectorFieldFunction* constraint);
  virtual ~ConstrainedInterpolator() {}
  bool Make(const Config& a,const Config& b,vector<Config>& path,bool checkConstraints=false);
  virtual void ConstraintValue(const Config& x,Vector& v);
  virtual bool Project(Config& x);

  CSpace* space;
  VectorFieldFunction* constraint;
  Config xmin,xmax; ///< if set, uses bounds in the newton solver
  VectorFieldFunction* inequalities;  ///< if set, uses a nonlinear constraint in the newton solver
  int maxNewtonIters;
  Real ftol,xtol;
  Real maxGrowth;

  //temp: solver
  Optimization::NewtonRoot solver;
};

/** @ingroup Planning
 * @brief Constructs a piecewise polynomial path between a and b such that
 * each point is near the constraint C(x)=0.
 *
 * Interpolation is accomplished via a cubic bezier curve.  This is slightly
 * faster than taking a regular ConstrainedInterpolator and then post-smoothing
 * it via a Bezier curve.
 *
 * The method uses a recursive bisection technique, where the midpoint of each segment
 * is projected to the constraint, and bisection continues until a given resolution
 * is met.  If checkConstraints is true, the feasibility of each projected
 * point is also checked.
 *
 * The projection uses a Newton-Raphson solver, capped at maxNewtonIters iterations.
 * It ensures that each milestone satisfies ||C(x[k])|| <= ftol, and
 * d(x[k],x[k+1])<=xtol.
 *
 * maxGrowth defines the maximum extra distance that the path through a projected
 * configuration can add to the total length of the path.  That is, when going from
 * x1 to x2, the projected midpoint
 * xm is checked so that d(x1,xm), d(xm,x2) <= (1+maxGrowth)/2 d(x1,x2).
 * To ensure convergence this parameter should be < 1 (default 0.9).
 */
class SmoothConstrainedInterpolator
{
 public:
  SmoothConstrainedInterpolator(CSpace* space,VectorFieldFunction* constraint);
  virtual ~SmoothConstrainedInterpolator() {}
  bool Make(const Config& a,const Config& b,
	    GeneralizedCubicBezierSpline& path,
	    bool checkConstraints=false);
  bool Make(const Config& a,const Vector& da,const Config& b,const Vector& db,
	    GeneralizedCubicBezierSpline& path,
	    bool checkConstraints=false);
  virtual void ConstraintValue(const Config& x,Vector& v);
  virtual bool Project(Config& x);
  virtual bool ProjectVelocity(const Config& x,Config& v);

  CSpace* space;
  GeodesicSpace* manifold;
  VectorFieldFunction* constraint;
  Config xmin,xmax; ///< if set, uses bounds in the newton solver
  VectorFieldFunction* inequalities;  ///< if set, uses a nonlinear constraint in the newton solver
  int maxNewtonIters;
  Real ftol,xtol;
  Real maxGrowth;

  //temp: solver
  Optimization::NewtonRoot solver;
};

/// @ingroup Planning
/// Interpolates through several points smoothly using the given interpolator.
/// The output durations are such that point i corresponds precisely to path
/// parameter i.
bool MultiSmoothInterpolate(SmoothConstrainedInterpolator& interp,
			    const vector<Vector>& pts,
			    GeneralizedCubicBezierSpline& path);


/// @ingroup Planning
/// Interpolates through several points smoothly using the given interpolator.
/// The output durations are such that point i corresponds precisely to path
/// parameter i.
/// The endpoint derivatives are set to dq0 and dq1, respectively.
bool MultiSmoothInterpolate(SmoothConstrainedInterpolator& interp,
			    const vector<Vector>& pts,
			    const Vector& dq0,const Vector& dq1,
			    GeneralizedCubicBezierSpline& path);


/// @ingroup Planning
/// Adds a smooth path to the given point at the end of the provided path.
/// The path extension has duration suffixDuration.
bool AppendInterpolate(SmoothConstrainedInterpolator& interp,
		       const Vector& pt,Real suffixDuration,
		       GeneralizedCubicBezierSpline& path);


} //namespace Klampt

#endif
