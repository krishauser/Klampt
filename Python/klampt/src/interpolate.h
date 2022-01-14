#ifndef INTERPOLATE_H
#define INTERPOLATE_H

#include <vector>
using namespace std;

/// Computes a min-time polynomial from (x0,v0) to (x1,v1) under the given
/// position bounds x in [xmin,xmax], velocity bounds |v|<=vmax, and
/// acceleration bounds |a|<=amax.
///
/// Returns:
///
///     (times,positions,velocities) which may be interpolated using hermite
///     interpolation.  If a path cannot be found, then empty arrays are
///     returned.
///
void interpolate_1d_min_time(double x0,double v0,double x1,double v1,
             double xmin,double xmax,double vmax,double amax,
             vector<double>& out,vector<double>& out2,vector<double>& out3);

/// Computes a min-acceleration polynomial from (x0,v0) to (x1,v1) that
/// finishes at a fixed end time endTime.  Enforces position bounds x in
/// [xmin,xmax] and the velocity bounds |v|<=vmax.
///
/// Returns:
///
///     (times,positions,velocities) which may be interpolated using hermite
///     interpolation.  If a path cannot be found, then empty arrays are
///     returned.
void interpolate_1d_min_accel(double x0,double v0,double x1,double v1,
              double endTime,double xmin,double xmax,double vmax,
              vector<double>& out,vector<double>& out2,vector<double>& out3);

/// Vector version of :func:`interpolate_1d_min_time`.
///
/// Computes a min-time polynomial from (x0,v0) to (x1,v1) under the given
/// position bounds x in [xmin,xmax], velocity bounds |v|<=vmax, and
/// acceleration bounds |a|<=amax (element-wise).  Each channel is required
/// to finish exactly at the same time.  The limiting channel will be solved
/// in min-time form, while the others will be solved in min-accel form.
///
/// Returns:
///
///     (times,positions,velocities) which give each of the channels'
///     interpolants, each of which may be interpolated using cubic
///     interpolation. If a path cannot be found, then empty arrays are
///     returned.
/// 
///     Specifically, len(times), len(positions), and len(velocities) ==
///     len(x0), and for each channel i, the 1D interpolator
///     ``hermite(times[i],positions[i],velocities[i])``
///     will give the path for that channel.
/// 
void interpolate_nd_min_time(const vector<double>& x0,const vector<double>& v0,const vector<double>& x1,const vector<double>& v1,
             const vector<double>& xmin,const vector<double>& xmax,const vector<double>& vmax,const vector<double>& amax,
             vector<vector<double> >& out,vector<vector<double> >& out2,vector<vector<double> >& out3);

/// Vector version of :func:`interpolate_1d_min_accel`.
///
/// Computes a min-acceleration polynomial from (x0,v0) to (x1,v1) that
/// finishes at a fixed end time endTime.  Enforces position bounds x in
/// [xmin,xmax] and the velocity bounds |v|<=vmax. (element-wise)
///
/// Returns:
///
///     (times,positions,velocities) which give each of the channels'
///     interpolants, each of which may be interpolated using cubic
///     interpolation.  If a path cannot be found, then empty arrays are
///     returned.
///
///     Specifically, len(times), len(positions), and len(velocities) ==
///     len(x0), and for each channel i, the 1D interpolator
///     ``hermite(times[i],positions[i],velocities[i])`` will give the path
///     for that channel.
/// 
void interpolate_nd_min_accel(const vector<double>& x0,const vector<double>& v0,const vector<double>& x1,const vector<double>& v1,
             double endTime,const vector<double>& xmin,const vector<double>& xmax,const vector<double>& vmax,
             vector<vector<double> >& out,vector<vector<double> >& out2,vector<vector<double> >& out3);

/// Computes a min-time polynomial from x0 to x1 that moves in a straight
/// line in configuration space, and minimizes time under velocity bounds
/// |v|<=vmax, and acceleration bounds |a|<=amax (element-wise). 
///
/// Returns:
///
///     (times,positions,velocities) which give the parameters of an
///     ND cubic curve.
///
///     (len(times)==len(positions)==len(velocities)), and
///     len(positions[i])==len(velocities[i])==N for all i.  The
///     ND interpolator ``hermite(times,positions[i],velocities[i])``
///     will give the optimized trajectory.
///
void interpolate_nd_min_time_linear(const vector<double>& x0,const vector<double>& x1,
             const vector<double>& vmax,const vector<double>& amax,
             vector<double>& out,vector<vector<double> >& out2,vector<vector<double> >& out3);

/// Computes a max-braking polynomial from (x0,v0) to a stopped state obeying
/// acceleration constraints |a|<=amax.
///
/// Returns:
///
///     (times,positions,velocities) which may be interpolated using cubic
///     interpolation.
///
void brake_1d(double x0,double v0,double amax,
              vector<double>& out,vector<double>& out2,vector<double>& out3);

/// Computes a max-braking polynomial from (x0,v0) to a stopped state
/// obeying acceleration bounds |a|<=amax (element-wise).  Position bounds
/// x in [xmin,xmax] are also enforced, if possible. 
///
/// The stopped state is reached simultaneously if possible within position
/// bounds.  This looks  visually pleasing, but some joints will be braking
/// slower than they can.  To brake faster, just call :func:`braking_1d` on
/// each dimension.
///
/// If position bounds cannot be met, stopping is done as fast as possible.
///
/// Returns:
///
///     (times,positions,velocities) which give the parameters of an
///     ND cubic curve.
///
///     (len(times)==len(positions)==len(velocities)), and
///     len(positions[i])==len(velocities[i])==N for all i.  The
///     ND interpolator ``hermite(times,positions[i],velocities[i])``
///     will give the optimized trajectory.
///
void brake_nd(const vector<double>& x0,const vector<double>& v0,
              const vector<double>& xmin,const vector<double>& xmax,const vector<double>& amax,
              vector<vector<double> >& out,vector<vector<double> >& out2,vector<vector<double> >& out3);

/// For a set of N cubic splines, defined in the channels times[i], positions[i],
/// and velocities[i], i=0,...,N-1, produces a unified N-D cubic curve
/// (times',positions',velocities') suitable for input into an N-D cubic
/// interpolator.
///
/// Use this to post-process the results from :func:`interpolate_nd_min_time` 
/// and :func:`interpolate_nd_min_accel`.
void combine_nd_cubic(const vector<vector<double> >& times,const vector<vector<double> >& positions,const vector<vector<double> >& velocities,
    vector<double>& out,vector<vector<double> >& out2,vector<vector<double> >& out3);

#endif // INTERPOLATE_H