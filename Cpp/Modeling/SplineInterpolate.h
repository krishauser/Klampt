#ifndef INTERPOLATE_CUBIC_SPLINES_H
#define INTERPOLATE_CUBIC_SPLINES_H

#include <KrisLibrary/planning/GeneralizedBezierCurve.h>
#include <KrisLibrary/math/misc.h>
#include <vector>

namespace Klampt {
  using namespace Math;
  using namespace std;

/** @addtogroup Modeling */
/*@{*/

/// Interpolates the given points.  The resulting path segments are C1
/// continuous when connected via a uniform parameterization.
void SplineInterpolate(const vector<Vector>& pts,
		       vector<GeneralizedCubicBezierCurve>& paths,
		       CSpace* space=NULL,GeodesicSpace* manifold=NULL);
/// Interpolates the given points and returns a timed path.  If
/// coxDeBoorParameter = 0, then a uniform parameterization is used.  If
/// 1, then the distance between points is used to define the parametrization
/// (chordal parameterization).  If 0.5, the centripetal parametrization is
/// used, which has some advantages.
void SplineInterpolate(const vector<Vector>& pts,
		       GeneralizedCubicBezierSpline& path,
		       CSpace* space=NULL,GeodesicSpace* manifold=NULL,
		       Real coxDeBoorParameter=0);
/// Interpolates the given points at the given times
void SplineInterpolate(const vector<Vector>& pts,const vector<Real>& times,
		       vector<GeneralizedCubicBezierCurve>& paths,
		       CSpace* space=NULL,GeodesicSpace* manifold=NULL);
/// Same as above but outputs a spline, sets the path's durations
void SplineInterpolate(const vector<Vector>& pts,const vector<Real>& times,
		       GeneralizedCubicBezierSpline& path,
		       CSpace* space=NULL,GeodesicSpace* manifold=NULL);
/// Interpolates ensuring that each intermediate segment is monotonically
/// increasing / decreasing (potentially less oscillation). The resulting
/// path segments are C1 continuous when connected via a uniform
///parameterization.
void MonotonicInterpolate(const vector<Vector>& pts,
			  vector<GeneralizedCubicBezierCurve>& paths,
			  CSpace* space=NULL,GeodesicSpace* manifold=NULL);
/// Interpolates the given points using monotonic interpolation, returning a
/// timed path.  If coxDeBoorParameter = 0, then a uniform parameterization
/// is used.  If 1, then the distance between points is used to define
/// the parametrization (chordal parameterization).  If 0.5, the centripetal
/// parametrization is/ used, which has some advantages.
void MonotonicInterpolate(const vector<Vector>& pts,
			  GeneralizedCubicBezierSpline& path,
			  CSpace* space=NULL,GeodesicSpace* manifold=NULL,
			  Real coxDeBoorParameter=0);
/// Same as above but with times
void MonotonicInterpolate(const vector<Vector>& pts,const vector<Real>& times,
			  vector<GeneralizedCubicBezierCurve>& paths,
			  CSpace* space=NULL,GeodesicSpace* manifold=NULL);
/// Same as above but outputs a spline, sets the path's durations
void MonotonicInterpolate(const vector<Vector>& pts,const vector<Real>& times,
			  GeneralizedCubicBezierSpline& path,
			  CSpace* space=NULL,GeodesicSpace* manifold=NULL);
///
void MonotonicAccelInterpolate(const vector<Vector>& pts,
			  vector<GeneralizedCubicBezierCurve>& paths,
			  CSpace* space=NULL,GeodesicSpace* manifold=NULL);
/*@}*/

} //namespace Klampt

#endif
