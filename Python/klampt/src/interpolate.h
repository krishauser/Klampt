#ifndef INTERPOLATE_H
#define INTERPOLATE_H

#include <vector>
using namespace std;

/// Computes a min-time polynomial from (x0,v0) to (x1,v1) under the given
/// position bounds x in [xmin,xmax], velocity bounds |v|<=vmax, and
/// acceleration bounds |a|<=amax.
///
/// Returns (times,positions,velocities) which may be interpolated using cubic
/// interpolation.  If a path cannot be found, then empty arrays are returned.
void interpolate1DMinTime(double x0,double v0,double x1,double v1,
             double xmin,double xmax,double vmax,double amax,
             vector<double>& out,vector<double>& out2,vector<double>& out3);

/// Computes a min-acceleration polynomial from (x0,v0) to (x1,v1) that
/// finishes at a fixed end time endTime.  Enforces position bounds x in
/// [xmin,xmax] and the velocity bounds |v|<=vmax.
///
/// Returns (times,positions,velocities) which may be interpolated using cubic
/// interpolation.  If a path cannot be found, then empty arrays are returned.
void interpolate1DMinAccel(double x0,double v0,double x1,double v1,
              double endTime,double xmin,double xmax,double vmax,
              vector<double>& out,vector<double>& out2,vector<double>& out3);

/// vector<double> version of :func:`interpolate1DMinTime`.
///
/// Computes a min-time polynomial from (x0,v0) to (x1,v1) under the given
/// position bounds x in [xmin,xmax], velocity bounds |v|<=vmax, and
/// acceleration bounds |a|<=amax (element-wise).  Each channel is required
/// to finish exactly at the same time.  The limiting channel will be solved
/// in min-time form, while the others will be solved in min-accel form.
///
/// Returns (times,positions,velocities) which give each of the channels'
/// interpolants, each of which may be interpolated using cubic interpolation.
/// Specifically, len(times), len(positions), and len(velocities) == len(x0),
/// and for each channel i, the 1D interpolator hermite(times[i],positions[i],velocities[i])
/// will give the path for that channel.
/// If a path cannot be found, then empty arrays are returned.
void interpolateNDMinTime(const vector<double>& x0,const vector<double>& v0,const vector<double>& x1,const vector<double>& v1,
             const vector<double>& xmin,const vector<double>& xmax,const vector<double>& vmax,const vector<double>& amax,
             vector<vector<double> >& out,vector<vector<double> >& out2,vector<vector<double> >& out3);

/// vector<double> version of above.
///
/// Computes a min-acceleration polynomial from (x0,v0) to (x1,v1) that
/// finishes at a fixed end time endTime.  Enforces position bounds x in
/// [xmin,xmax] and the velocity bounds |v|<=vmax. (element-wise)
///
/// Returns (times,positions,velocities) which give each of the channels'
/// interpolants, each of which may be interpolated using cubic interpolation.
/// Specifically, len(times), len(positions), and len(velocities) == len(x0),
/// and for each channel i, the 1D interpolator hermite(times[i],positions[i],velocities[i])
/// will give the path for that channel.
/// If a path cannot be found, then empty arrays are returned.
void interpolateNDMinAccel(const vector<double>& x0,const vector<double>& v0,const vector<double>& x1,const vector<double>& v1,
             double endTime,const vector<double>& xmin,const vector<double>& xmax,const vector<double>& vmax,
             vector<vector<double> >& out,vector<vector<double> >& out2,vector<vector<double> >& out3);

/// Computes a min-time polynomial from x0 to x1 that moves in a straight
/// line in configuration space, and minimizes time under velocity bounds
/// |v|<=vmax, and acceleration bounds |a|<=amax (element-wise). 
///
/// Returns (times,positions,velocities) which give the parameters of an
/// ND cubic curve.  (len(times)==len(positions)==len(velocities)), and
/// len(positions[i])==len(velocities[i])==N for all i.  The
/// ND interpolator hermite(times,positions[i],velocities[i])
/// will give the optimized trajectory.
void interpolateNDMinTimeLinear(const vector<double>& x0,const vector<double>& x1,
             const vector<double>& vmax,const vector<double>& amax,
             vector<double>& out,vector<vector<double> >& out2,vector<vector<double> >& out3);

/// For a set of N cubic splines, defined in the channels times[i], positions[i],
/// and velocities[i], i=0,...,N-1, produces a unified N-D cubic curve
/// (times',positions',velocities') suitable for input into an N-D cubic
/// interpolator.
///
/// Use this to post-process the results from interpolateNDMinTime and
/// interpolateNDMinAccel.
void combineNDCubic(const vector<vector<double> >& times,const vector<vector<double> >& positions,const vector<vector<double> >& velocities,
    vector<double>& out,vector<vector<double> >& out2,vector<vector<double> >& out3);

#endif // INTERPOLATE_H