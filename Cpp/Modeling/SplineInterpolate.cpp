#include "SplineInterpolate.h"

namespace Klampt {

void SplineInterpolate(const vector<Vector>& pts,
		       vector<GeneralizedCubicBezierCurve>& paths,
		       CSpace* space,GeodesicSpace* manifold)
{
  paths.resize(pts.size()-1);
  for(size_t i=0;i<paths.size();i++) {
    paths[i].x0 = pts[i];
    paths[i].x3 = pts[i+1];
    paths[i].space = space;
    paths[i].manifold = manifold;
  }
  if(pts.size() == 2) {
    paths[0].SetSmoothTangents(NULL,NULL);
    return;
  }
  for(size_t i=0;i<paths.size();i++) {
    const Vector* prev=(i > 0 ? &pts[i-1] : NULL);
    const Vector* next=(i+2 <pts.size() ? &pts[i+2] : NULL);
    paths[i].SetSmoothTangents(prev,next);
  }
}

void SplineInterpolate(const vector<Vector>& pts,
		       GeneralizedCubicBezierSpline& path,
		       CSpace* space,GeodesicSpace* manifold,
		       Real coxDeBoorParameter)
{
  if(coxDeBoorParameter == 0) {
    //uniform
    SplineInterpolate(pts,path.segments,space,manifold);
    path.durations.resize(path.segments.size());
    fill(path.durations.begin(),path.durations.end(),1);
  }
  else {
    vector<Real> times(pts.size());
    times[0] = 0;
    path.durations.resize(pts.size()-1);
    for(size_t i=0;i+1<pts.size();i++) {
      path.durations[i] = Pow(pts[i].distance(pts[i+1]),coxDeBoorParameter);
      times[i+1] = times[i] + path.durations[i];
    }
    SplineInterpolate(pts,times,path.segments,space,manifold);
  }
}

void SplineInterpolate(const vector<Vector>& pts,const vector<Real>& times,
		       vector<GeneralizedCubicBezierCurve>& paths,
		       CSpace* space,GeodesicSpace* manifold)
{
  Assert(pts.size()==times.size());
  paths.resize(pts.size()-1);
  for(size_t i=0;i<paths.size();i++) {
    paths[i].x0 = pts[i];
    paths[i].x3 = pts[i+1];
    paths[i].space = space;
    paths[i].manifold = manifold;
  }
  if(pts.size() == 2) {
    paths[0].SetSmoothTangents(NULL,NULL);
    return;
  }
  for(size_t i=0;i<paths.size();i++) {
    const Vector* prev=(i > 0 ? &pts[i-1] : NULL);
    const Vector* next=(i+2 <pts.size() ? &pts[i+2] : NULL);
    Real dtprev = (i > 0? times[i]-times[i-1]: 1.0);
    Real dtnext = (i+2 < pts.size()? times[i+2]-times[i+1]: 1.0);
    Real dt = times[i+1]-times[i];
    paths[i].SetSmoothTangents(prev,next,dtprev/dt,dtnext/dt);
  }
}


void SplineInterpolate(const vector<Vector>& pts,const vector<Real>& times,
		       GeneralizedCubicBezierSpline& path,
		       CSpace* space,GeodesicSpace* manifold)
{
  SplineInterpolate(pts,times,path.segments,space,manifold);
  Assert(path.segments.size()+1==times.size());
  path.durations.resize(path.segments.size());
  for(size_t i=0;i<path.segments.size();i++) path.durations[i] = times[i+1]-times[i];
}

void MonotonicInterpolate(const vector<Vector>& pts,vector<GeneralizedCubicBezierCurve>& paths,CSpace* space,GeodesicSpace* manifold)
{
  Assert(pts.size() >= 2);
  
  paths.resize(pts.size()-1);
  for(size_t i=0;i<paths.size();i++) {
    paths[i].x0 = pts[i];
    paths[i].x3 = pts[i+1];
    paths[i].space = space;
    paths[i].manifold = manifold;
  }
  vector<Vector> tangents(pts.size()),inslopes(pts.size()-1),outslopes(pts.size()-1);
  if(pts.size() == 2) {
    paths[0].SetSmoothTangents(NULL,NULL);
    return;
  }
  paths[0].SetSmoothTangents(NULL,&pts[2]);
  paths[0].Deriv(0,tangents[0]);
  paths.back().x0 = pts[pts.size()-2];
  paths.back().x3 = pts[pts.size()-1];
  paths.back().SetSmoothTangents(&pts[pts.size()-3],NULL);
  paths.back().Deriv(1,tangents.back());
  for(size_t i=1;i<pts.size();i++) {
    if(!manifold) {
      inslopes[i-1] = pts[i]-pts[i-1];
      outslopes[i-1].setRef(inslopes[i-1]);
    }
    else {
      manifold->InterpolateDeriv(pts[i-1],pts[i],0,inslopes[i-1]);
      manifold->InterpolateDeriv(pts[i],pts[i-1],0,outslopes[i-1]);
      outslopes[i-1].inplaceNegative();
    }
    if(i+1<pts.size()) {
      if(!manifold)
	tangents[i] = (pts[i+1]-pts[i-1])*0.5;
      else {
	Vector n,p;
	manifold->InterpolateDeriv(pts[i],pts[i+1],0,n);
	manifold->InterpolateDeriv(pts[i],pts[i-1],0,p);
	tangents[i] = (n-p)*0.5;
      }
    }
  }
  int n=pts[0].n;
  for(size_t i=0;i<pts.size();i++) {
    if(tangents[i].n != n) printf("%d / %d\n",i,tangents.size());
    Assert(tangents[i].n == n);
  }
  for(size_t i=0;i+1<pts.size();i++) {
    for(int j=0;j<n;j++) {
      if(Sign(tangents[i][j]) != Sign(inslopes[i][j]))
	tangents[i][j] = 0;
      else {
	if(tangents[i][j]>0) {
	  if(tangents[i][j] > 3.0*inslopes[i][j])
	    tangents[i][j] = 3.0*inslopes[i][j];
	}
	else {
	  if(tangents[i][j] < 3.0*inslopes[i][j])
	    tangents[i][j] = 3.0*inslopes[i][j];
	}
      }
      if(Sign(tangents[i+1][j]) != Sign(outslopes[i][j]))
	tangents[i+1][j] = 0;
      else {
	if(tangents[i+1][j]>0) {
	  if(tangents[i+1][j] > 3.0*outslopes[i][j])
	    tangents[i+1][j] = 3.0*outslopes[i][j];
	}
	else {
	  if(tangents[i+1][j] < 3.0*outslopes[i][j])
	    tangents[i+1][j] = 3.0*outslopes[i][j];
	}
      }
    }
  }
  for(size_t i=0;i+1<pts.size();i++) {
    paths[i].SetNaturalTangents(tangents[i],tangents[i+1]);
  }
}

void MonotonicInterpolate(const vector<Vector>& pts,
			  GeneralizedCubicBezierSpline& path,
			  CSpace* space,GeodesicSpace* manifold,
			  Real coxDeBoorParameter)
{

  if(coxDeBoorParameter == 0) {
    //uniform
    MonotonicInterpolate(pts,path.segments,space,manifold);
    path.durations.resize(path.segments.size());
    fill(path.durations.begin(),path.durations.end(),1);
  }
  else {
    vector<Real> times(pts.size());
    times[0] = 0;
    path.durations.resize(pts.size()-1);
    for(size_t i=0;i+1<pts.size();i++) {
      path.durations[i] = Pow(pts[i].distance(pts[i+1]),coxDeBoorParameter);
      times[i+1] = times[i] + path.durations[i];
    }
    MonotonicInterpolate(pts,times,path.segments,space,manifold);
  }
}

void MonotonicInterpolate(const vector<Vector>& pts,const vector<Real>& times,vector<GeneralizedCubicBezierCurve>& paths,CSpace* space,GeodesicSpace* manifold)
{
  Assert(times.size()==pts.size());
  Assert(pts.size() >= 2);
  
  paths.resize(pts.size()-1);
  for(size_t i=0;i<paths.size();i++) {
    paths[i].x0 = pts[i];
    paths[i].x3 = pts[i+1];
    paths[i].space = space;
    paths[i].manifold = manifold;
  }
  vector<Real> durations(pts.size()-1);
  vector<Real> rates(pts.size()-1);
  for(size_t i=0;i+1<pts.size();i++) {
    durations[i] = times[i+1]-times[i];
    rates[i] = 1.0/durations[i];
  }
  vector<Vector> tangents(pts.size()),inslopes(pts.size()-1),outslopes(pts.size()-1);
  if(pts.size() == 2) {
    paths[0].SetSmoothTangents(NULL,NULL);
    return;
  }
  paths[0].SetSmoothTangents(NULL,&pts[2],1.0,durations[1]*rates[0]);
  paths[0].Deriv(0,tangents[0]);
  paths.back().x0 = pts[pts.size()-2];
  paths.back().x3 = pts[pts.size()-1];
  paths.back().SetSmoothTangents(&pts[pts.size()-3],NULL,durations[durations.size()-2]*rates.back(),1.0);
  paths.back().Deriv(1,tangents.back());
  for(size_t i=1;i<pts.size();i++) {
    if(!manifold) {
      inslopes[i-1] = pts[i]-pts[i-1];
      inslopes[i-1] *= rates[i-1];
      outslopes[i-1].setRef(inslopes[i-1]);
    }
    else {
      manifold->InterpolateDeriv(pts[i-1],pts[i],0,inslopes[i-1]);
      manifold->InterpolateDeriv(pts[i],pts[i-1],0,outslopes[i-1]);
      outslopes[i-1].inplaceNegative();
      inslopes[i-1] *= rates[i-1];
      outslopes[i-1] *= rates[i-1];
    }
    /* xi = y(0)
     * xp = y(-dtp)
     * xn = y(dtn)
     * fit a quadratic 
     *   y(u) = a u^2 + b u + c
     * and find y'(0)=b
     * 
     * c = xi
     * a dtn^2 + b dtn = xn-xi
     * a dtp^2 - b dtp = xp-xi
     * [dtn^2  dtn][a] = [xn-xi]
     * [dtp^2 -dtp][b]   [xp-xi]
     * -1/(dtn dtp)(dtn + dtp) [ -dtp   -dtn ][xn-xi] = [a]
     *                         [ -dtp^2 dtn^2][xp-xi]   [b]
     * b = dtp^2/(dtn dtp)(dtn + dtp) (xn-xi) - dtn^2/(dtn dtp)(dtn + dtp)(xp-xi)
     *   = 1/(dtn+dtp) (dtp/dtn (xn-xi) - dtn/dtp (xp-xi))
     */
    if(i+1<pts.size()) {
      Vector n,p;
      Real s1 = durations[i-1]*rates[i];
      Real s2 = durations[i]*rates[i-1];
      if(!manifold) {
	n = pts[i+1] - pts[i];
	p = pts[i-1] - pts[i];
      }
      else {
	manifold->InterpolateDeriv(pts[i],pts[i+1],0,n);
	manifold->InterpolateDeriv(pts[i],pts[i-1],0,p);
      }
      tangents[i] = (s2*n-s1*p)/(durations[i-1]+durations[i]);
    }
  }
  int n=pts[0].n;
  for(size_t i=0;i<pts.size();i++) {
    if(tangents[i].n != n) printf("%d / %d\n",i,tangents.size());
    Assert(tangents[i].n == n);
  }
  for(size_t i=0;i+1<pts.size();i++) {
    for(int j=0;j<n;j++) {
      if(Sign(tangents[i][j]) != Sign(inslopes[i][j]))
	tangents[i][j] = 0;
      else {
	if(tangents[i][j]>0) {
	  if(tangents[i][j] > 3.0*inslopes[i][j])
	    tangents[i][j] = 3.0*inslopes[i][j];
	}
	else {
	  if(tangents[i][j] < 3.0*inslopes[i][j])
	    tangents[i][j] = 3.0*inslopes[i][j];
	}
      }
      if(Sign(tangents[i+1][j]) != Sign(outslopes[i][j]))
	tangents[i+1][j] = 0;
      else {
	if(tangents[i+1][j]>0) {
	  if(tangents[i+1][j] > 3.0*outslopes[i][j])
	    tangents[i+1][j] = 3.0*outslopes[i][j];
	}
	else {
	  if(tangents[i+1][j] < 3.0*outslopes[i][j])
	    tangents[i+1][j] = 3.0*outslopes[i][j];
	}
      }
    }
  }
  for(size_t i=0;i+1<pts.size();i++) {
    paths[i].SetNaturalTangents(tangents[i]*durations[i],tangents[i+1]*durations[i]);
  }
}

void MonotonicInterpolate(const vector<Vector>& pts,const vector<Real>& times,
			  GeneralizedCubicBezierSpline& path,
			  CSpace* space,GeodesicSpace* manifold)
{
  MonotonicInterpolate(pts,times,path.segments,space,manifold);
  Assert(path.segments.size()+1==times.size());
  path.durations.resize(path.segments.size());
  for(size_t i=0;i<path.segments.size();i++) path.durations[i] = times[i+1]-times[i];
}

void MonotonicAccelInterpolate(const vector<Vector>& pts,vector<GeneralizedCubicBezierCurve>& paths,CSpace* space,GeodesicSpace* manifold)
{
  Assert(pts.size() >= 2);
  
  vector<Vector> tangents(pts.size()),inslopes(pts.size()-1),outslopes(pts.size()-1);
  paths.resize(pts.size()-1);
  for(size_t i=0;i<paths.size();i++) {
    paths[i].x0 = pts[i];
    paths[i].x3 = pts[i+1];
    paths[i].space = space;
    paths[i].manifold = manifold;
  }
  paths[0].x0 = pts[0];
  paths[0].x3 = pts[1];
  if(pts.size() == 2) {
    paths[0].SetSmoothTangents(NULL,NULL);
    return;
  }
  paths[0].SetSmoothTangents(NULL,&pts[2]);
  paths[0].Deriv(0,tangents[0]);
  paths.back().x0 = pts[pts.size()-2];
  paths.back().x3 = pts[pts.size()-1];
  paths.back().SetSmoothTangents(&pts[pts.size()-3],NULL);
  paths.back().Deriv(1,tangents.back());
  for(size_t i=1;i<pts.size();i++) {
    if(!manifold) {
      inslopes[i-1] = pts[i]-pts[i-1];
      outslopes[i-1].setRef(inslopes[i-1]);
    }
    else {
      manifold->InterpolateDeriv(pts[i-1],pts[i],0,inslopes[i-1]);
      manifold->InterpolateDeriv(pts[i],pts[i-1],0,outslopes[i-1]);
      outslopes[i-1].inplaceNegative();
    }
    if(i+1<pts.size()) {
      if(!manifold)
	tangents[i] = (pts[i+1]-pts[i-1])*0.5;
      else {
	Vector n,p;
	manifold->InterpolateDeriv(pts[i],pts[i+1],0,n);
	manifold->InterpolateDeriv(pts[i],pts[i-1],0,p);
	tangents[i] = (n-p)*0.5;
      }
    }
  }
  int n=pts[0].n;
  for(size_t i=0;i<pts.size();i++) {
    if(tangents[i].n != n) printf("%d / %d\n",i,tangents.size());
    Assert(tangents[i].n == n);
  }
  for(size_t i=0;i+1<pts.size();i++) {
    if(i == 1)
      cout<<"Orig tangent 2: "<<tangents[i+1]<<endl;
    for(int j=0;j<n;j++) {
      if(j==0) {
	printf("Segment %d: accel in %g, out %g\n",i,3.0*inslopes[i][j] - tangents[i+1][j]-2*tangents[i][j],2*tangents[i+1][j]+tangents[i][j] - 3.0*outslopes[i][j]);
      }
      if(Sign(3.0*inslopes[i][j] - tangents[i+1][j] - 2*tangents[i][j]) != Sign(2*tangents[i+1][j]+tangents[i][j] - 3.0*outslopes[i][j])) {
	//(3.0*mi - x - 2*t0)*(2*x+t0 - 3.0*mo) = 0
	//(- x + 3.0*mi - 2*t0)*(2*x+t0 - 3.0*mo) = 
	//    -2x^2 + x*(-t0+3mo+6mi-4t0) + (3mi-2t0)(t0-3mo) = 0
	//solve quadratic to set tangents[i+1][j] so one accel becomes
	//nullified
	Real a = -2.0;
	Real b = 6*inslopes[i][j] + 3*outslopes[i][j]-5*tangents[i][j];
	Real c = (3*inslopes[i][j]-2*tangents[i][j])*(tangents[i][j]-3*outslopes[i][j]);
	Real t1,t2;
	int res=quadratic(a,b,c,t1,t2);
	if(res == 0) {
	  if(j==0) 
	    printf("No solution to quadratic %g %g %g\n",a,b,c);
	}
	else {
	  assert(res > 0);
	  if(res == 2) {
	    //pick the closer one to the existing tangent
	    if(Abs(t1 - tangents[i+1][j]) > Abs(t2 - tangents[i+1][j]))
	      t1 = t2;
	  }
	  tangents[i+1][j] = t1;
	  if(j==0) {
	    printf("New accel in %g, out %g\n",3.0*inslopes[i][j] - tangents[i+1][j]-2*tangents[i][j],2*tangents[i+1][j]+tangents[i][j] - 3.0*outslopes[i][j]);
	  }
	}
      }
    }
    if(i == 1)
      cout<<"New tangent 2: "<<tangents[i+1]<<endl;
  }
  for(size_t i=0;i+1<pts.size();i++) {
    paths[i].SetNaturalTangents(tangents[i],tangents[i+1]);
    Vector temp,temp2;
    paths[i].Accel(0,temp);
    paths[i].Accel(1,temp2);
    cout<<"in "<<temp[0]<<" out "<<temp2[0]<<endl;
  }
}

} //namespace Klampt