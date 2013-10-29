#ifndef INTERPOLATE_CUBIC_SPLINES_H
#define INTERPOLATE_CUBIC_SPLINES_H

#include <planning/GeneralizedBezierCurve.h>
#include <math/misc.h>
#include <vector>
using namespace Math;
using namespace std;

/** @addtogroup Modeling */
/*@{*/

/// Interpolates the given points
void SplineInterpolate(const vector<Vector>& pts,
		       vector<GeneralizedCubicBezierCurve>& paths,
		       CSpace* space=NULL,GeodesicManifold* manifold=NULL);
/// Interpolates the given points and sets a path with empty durations
void SplineInterpolate(const vector<Vector>& pts,
		       GeneralizedCubicBezierSpline& path,
		       CSpace* space=NULL,GeodesicManifold* manifold=NULL);
/// Interpolates the given points at the given times
void SplineInterpolate(const vector<Vector>& pts,const vector<Real>& times,
		       vector<GeneralizedCubicBezierCurve>& paths,
		       CSpace* space=NULL,GeodesicManifold* manifold=NULL);
/// Same as above but outputs a spline, sets the path's durations
void SplineInterpolate(const vector<Vector>& pts,const vector<Real>& times,
		       GeneralizedCubicBezierSpline& path,
		       CSpace* space=NULL,GeodesicManifold* manifold=NULL);
/// Interpolates ensuring that each intermediate segment is monotonically
/// increasing / decreasing (potentially less oscillation)
void MonotonicInterpolate(const vector<Vector>& pts,
			  vector<GeneralizedCubicBezierCurve>& paths,
			  CSpace* space=NULL,GeodesicManifold* manifold=NULL);
/// Same as above but assigns empty durations to the path
void MonotonicInterpolate(const vector<Vector>& pts,
			  GeneralizedCubicBezierSpline& path,
			  CSpace* space=NULL,GeodesicManifold* manifold=NULL);
/// Same as above but with times
void MonotonicInterpolate(const vector<Vector>& pts,const vector<Real>& times,
			  vector<GeneralizedCubicBezierCurve>& paths,
			  CSpace* space=NULL,GeodesicManifold* manifold=NULL);
/// Same as above but outputs a spline, sets the path's durations
void MonotonicInterpolate(const vector<Vector>& pts,const vector<Real>& times,
			  GeneralizedCubicBezierSpline& path,
			  CSpace* space=NULL,GeodesicManifold* manifold=NULL);
///
void MonotonicAccelInterpolate(const vector<Vector>& pts,
			  vector<GeneralizedCubicBezierCurve>& paths,
			  CSpace* space=NULL,GeodesicManifold* manifold=NULL);
/*@}*/

#endif
