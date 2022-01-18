#include "ContactTimeScaling.h"
#include "ZMP.h"
#include <KrisLibrary/robotics/NewtonEuler.h>
#include <KrisLibrary/robotics/TorqueSolver.h>
#include <KrisLibrary/optimization/LinearProgram.h>
#include <KrisLibrary/optimization/GLPKInterface.h>
#include <KrisLibrary/geometry/PolytopeProjection.h>
using namespace Math3D;
using namespace Optimization;
using namespace Klampt;

#define TEST_NO_CONTACT 0



ZMPTimeScaling::ZMPTimeScaling(RobotModel& robot)
  :CustomTimeScaling(robot)
{
}

void ZMPTimeScaling::SetParams(const MultiPath& path,const vector<Real>& colocationParams,
          const vector<Vector2>& supportPoly,Real groundHeight)
{
  Assert(path.sections.size()==1);
  vector<ConvexPolygon2D> sps(1);
  sps[0].vertices = supportPoly;
  if(!sps[0].isValid()) {
    FatalError("Convex hull must be given in CCW order\n");
  }
  vector<Real> groundHeights(1);
  groundHeights[0] = groundHeight;
  SetParams(path,colocationParams,sps,groundHeights);
}

/*
 derivation:
 x'' = fcx / m
 y'' = fcy / m
 z'' = fcz / m - g 
 with no rotational moment, (fcx,fcy,fcz) = f*(x-gx,y-gy,z-gh)
 where f is some unknown parameter.
 so gx = x - m x'' / f
 so gy = y - m y'' / f
 If z'' is known, then z'' + g = fcz / m = f*(z-gh)/m
 so m/f = (z-gh)/(z'' + g)

 Finally we have
   gx = x - x''*(z-gh)/(z'' + g)
   gy = y - y''*(z-gh)/(z'' + g)

 When z'' is not 0, then both x'' and z'' depend on the time scaling
 x'' = ax*ds^2 + bx*dds
 z'' = az*ds^2 + bz*dds
   gx = x - (ax*ds^2 + bx*dds)(z-gh)/(az*ds^2 + bz*dds + g)
   Consider 1st order taylor expansion about ds, dds = 0
   gx = x - (ax*ds^2 + bx*dds)(z-gh)/g
 */
void ZMPTimeScaling::SetParams(const MultiPath& path,const vector<Real>& paramDivs,const vector<ConvexPolygon2D>& supportPolys,const vector<Real>& groundHeights)
{
  RobotModel& robot = cspace.robot;
  Assert(path.sections.size()==supportPolys.size());
  Assert(path.sections.size()==groundHeights.size());
  this->supportPolys = supportPolys;
  this->groundHeights = groundHeights;
  CustomTimeScaling::SetPath(path,paramDivs);
  CustomTimeScaling::SetDefaultBounds();
  CustomTimeScaling::SetStartStop();
  NewtonEulerSolver ne(robot);
  for(size_t i=0;i<xs.size();i++) {
    Vector3 cm,dcm0,ddcm0;
    GetCOMDerivs(robot,xs[i],dxs[i],ddxs[i],cm,dcm0,ddcm0,ne);
    int s = paramSections[i];
    //ZMP.x = cm.x - (cm.z - groundHeights[s])/(9.8 - ddcm.z) * ddcm.x 
    //ZMP.y = cm.y - (cm.z - groundHeights[s])/(9.8 - ddcm.z) * ddcm.y
    //to first order approximation about speed = 0, ddcm.z does not factor into the ZMP
    //ddcm = ddcm0*ds2 + dcm0*dds
    Real ddcmScale = (cm.z - groundHeights[s])/(9.8);
    Plane2D p;
    for(size_t j=0;j<supportPolys[s].vertices.size();j++) {
      supportPolys[s].getPlane(j,p);
      //p.normal.x * ZMP.x + p.normal.y * ZMP.y <= p.offset
      //p.normal.x * (cm.x - ddcmScale * ddcm.x ) + p.normal.y * (cm.y - ddcmScale * ddcm.y ) <= p.offset
      //(-p.normal.x * ddcmScale * ddcm0.x - p.normal.y * ddcmScale * ddcm0.y)*ds2 
      //  + (-p.normal.x * ddcmScale * dcm0.x - p.normal.y * ddcmScale * dcm0.y)*dds 
      //  <= p.offset - p.normal.x * cm.x - p.normal.y * cm.y)
      Real ds2term = - p.normal.x * ddcmScale * ddcm0.x - p.normal.y * ddcmScale * ddcm0.y;
      Real ddsterm = - p.normal.x * ddcmScale * dcm0.x - p.normal.y * ddcmScale * dcm0.y;
      ds2ddsConstraintNormals[i].push_back(Vector2(ds2term,ddsterm));
      ds2ddsConstraintOffsets[i].push_back(p.offset - p.normal.x * cm.x - p.normal.y * cm.y);
      if(saveConstraintNames) {
        stringstream ss;
        ss<<"sp_"<<s<<"_"<<j;
        ds2ddsConstraintNames[i].push_back(ss.str());
      }
    }
  }
}

TorqueTimeScaling::TorqueTimeScaling(RobotModel& robot)
  :CustomTimeScaling(robot),torqueLimitShift(0),torqueLimitScale(1)
{}

void TorqueTimeScaling::SetParams(const MultiPath& path,const vector<Real>& colocationParams)
{
  RobotModel& robot = cspace.robot;
  CustomTimeScaling::SetPath(path,colocationParams);
  CustomTimeScaling::SetDefaultBounds();
  CustomTimeScaling::SetStartStop();

  NewtonEulerSolver ne(robot);
  //kinetic energy, coriolis force, gravity
  Vector3 gravity(0,0,-9.8);
  Matrix B;
  Vector C,G;
  //coefficients of time scaling
  Vector a,b,c;
  for(size_t i=0;i<paramDivs.size();i++) {
    robot.UpdateConfig(xs[i]);
    robot.dq = dxs[i];
    ne.CalcKineticEnergyMatrix(B);
    ne.CalcResidualTorques(C);
    robot.GetGravityTorques(gravity,G);
    ne.MulKineticEnergyMatrix(dxs[i],a);
    ne.MulKineticEnergyMatrix(ddxs[i],b);
    b += C;
    c = G;
    //Torque is given by a*dds + b*ds^2 + c = t
    for(int j=0;j<robot.torqueMax.n;j++) {
      Real tmax = robot.torqueMax(j)*torqueLimitScale + torqueLimitShift;
      if(tmax < 0) tmax=0;
      //b*ds^2 + a*dds <= tmax - c
      //-b*ds^2 - a*dds <= tmax + c
      ds2ddsConstraintNormals[i].push_back(Vector2(b(j),a(j)));
      ds2ddsConstraintOffsets[i].push_back(tmax-c(j));
      ds2ddsConstraintNormals[i].push_back(Vector2(-b(j),-a(j)));
      ds2ddsConstraintOffsets[i].push_back(tmax+c(j));
      if(saveConstraintNames) {
        stringstream ss;
        ss<<"tmax_"<<j;
        ds2ddsConstraintNames[i].push_back(ss.str());
      }
      if(saveConstraintNames) {
        stringstream ss;
        ss<<"tmin_"<<j;
        ds2ddsConstraintNames[i].push_back(ss.str());
      }
    }
  }
}


ContactTimeScaling::ContactTimeScaling(RobotModel& robot)
  :CustomTimeScaling(robot),torqueLimitShift(0),torqueLimitScale(1.0),frictionRobustness(0),forceRobustness(0)
{
}

bool ContactTimeScaling::SetParams(const MultiPath& path,const vector<Real>& paramDivs,int numFCEdges)
{
  RobotModel& robot = cspace.robot;
  CustomTimeScaling::SetPath(path,paramDivs);
  CustomTimeScaling::SetDefaultBounds();
  CustomTimeScaling::SetStartStop();

  ContactFormation formation;
  int oldSection = -1;
  LinearProgram_Sparse lp;
  NewtonEulerSolver ne(robot);
  //kinetic energy, coriolis force, gravity
  Vector3 gravity(0,0,-9.8);
  Vector C,G;
  //coefficients of time scaling
  Vector a,b,c;
  bool feasible=true;
  for(size_t i=0;i<paramDivs.size();i++) {
    Assert(paramSections[i] >= 0 && paramSections[i] < (int)path.sections.size());
    if(paramSections[i] != oldSection) {
      //reconstruct LP for the contacts in this section
      Stance stance;
      path.GetStance(stance,paramSections[i]);
      ToContactFormation(stance,formation);
      for(size_t j=0;j<formation.contacts.size();j++)
	for(size_t k=0;k<formation.contacts[j].size();k++) {
	  Assert(formation.contacts[j][k].kFriction > 0);
	  Assert(frictionRobustness < 1.0);
	  formation.contacts[j][k].kFriction *= (1.0-frictionRobustness);
	}

      //now formulate the LP.  Variable 0 is dds, variable 1 is ds^2
      //rows 1-n are torque max
      //rows n+1 - 2n are acceleration max
      //rows 2n+1 + 2n+numFCEdges*nc are the force constraints
      //vel max is encoded in the velocity variable
      int n = (int)robot.links.size();
      int nc = formation.numContactPoints();
#if TEST_NO_CONTACT
      nc = 0;
#endif // TEST_NO_CONTACT
      lp.Resize(n*2+numFCEdges*nc,2+3*nc);
      lp.A.setZero();
      lp.c.setZero();
      //fill out wrench matrix FC*f <= 0
#if !TEST_NO_CONTACT
      SparseMatrix FC;
      GetFrictionConePlanes(formation,numFCEdges,FC);
      lp.A.copySubMatrix(n*2,2,FC);
      for(int j=0;j<FC.m;j++)
	lp.p(n*2+j) = -forceRobustness;
#endif // !TEST_NO_CONTACT

      lp.l(0) = 0.0;
      lp.l(1) = -Inf;

      oldSection = paramSections[i];
    }
    //configuration specific 
    robot.UpdateConfig(xs[i]);
    robot.dq = dxs[i];
    ne.CalcResidualTorques(C);
    robot.GetGravityTorques(gravity,G);
    ne.MulKineticEnergyMatrix(dxs[i],a);
    ne.MulKineticEnergyMatrix(ddxs[i],b);
    b += C;
    c = G;

    //|a dds + b ds^2 + c - Jtf| <= torquemax*scale+shift
    for(int j=0;j<a.n;j++) {
      lp.A(j,0) = b(j);
      lp.A(j,1) = a(j);
      Real tmax = robot.torqueMax(j)*torqueLimitScale+torqueLimitShift;
      if(tmax < 0) tmax=0;
      lp.p(j) = tmax-c(j);
      lp.q(j) = -tmax-c(j);
    }
#if TEST_NO_CONTACT
    lp.p.set(Inf);
    lp.q.set(-Inf);
#else
    //fill out jacobian transposes
    int ccount=0;
    for(size_t l=0;l<formation.links.size();l++) {
      int link = formation.links[l];
      int target = (formation.targets.empty() ? -1 : formation.targets[l]);
      for(size_t j=0;j<formation.contacts[l].size();j++,ccount++) {
	Vector3 p=formation.contacts[l][j].x;
	//if it's a self-contact, then transform to world
	if(target >= 0)
	  p = robot.links[target].T_World*p;
	Vector3 v;
	int k=link;
	while(k!=-1) {
	  robot.links[k].GetPositionJacobian(robot.q[k],p,v);
	  if(v.x != 0.0) lp.A(k,2+ccount*3)=-v.x;
	  if(v.y != 0.0) lp.A(k,2+ccount*3+1)=-v.y;
	  if(v.z != 0.0) lp.A(k,2+ccount*3+2)=-v.z;
	  k=robot.parents[k];
	}
	k = target;
	while(k!=-1) {
	  robot.links[k].GetPositionJacobian(robot.q[k],p,v);
	  if(v.x != 0.0) lp.A(k,2+ccount*3)+=v.x;
	  if(v.y != 0.0) lp.A(k,2+ccount*3+1)+=v.y;
	  if(v.z != 0.0) lp.A(k,2+ccount*3+2)+=v.z;
	  k=robot.parents[k];
	}
      }
    }
    Assert(ccount == formation.numContactPoints());
#endif //TEST_NO_CONTACT

    //fill out acceleration constraint |ddx*ds^2 + dx*dds| <= amax
    for(int j=0;j<a.n;j++) {
      lp.q(a.n+j) = -robot.accMax(j);
      lp.p(a.n+j) = robot.accMax(j);
      lp.A(a.n+j,0) = ddxs[i][j];
      lp.A(a.n+j,1) = dxs[i][j];
    }

    //compute upper bounds from vel and acc max
    lp.u(0) = Inf; lp.u(1) = Inf;
    for(int j=0;j<a.n;j++) {
      if(dxs[i][j] < 0)
	lp.u(0) = Min(lp.u(0),Sqr(robot.velMin(j)/dxs[i][j]));
      else
	lp.u(0) = Min(lp.u(0),Sqr(robot.velMax(j)/dxs[i][j]));
    }

    //expand polytope
    Geometry::PolytopeProjection2D proj(lp);
    Geometry::UnboundedPolytope2D poly;
    proj.Solve(poly);
    if(poly.vertices.empty()) {
      //problem is infeasible?
      printf("Problem is infeasible at segment %d\n",i);
      cout<<"x = "<<xs[i]<<endl;
      cout<<"dx = "<<dxs[i]<<endl;
      cout<<"ddx = "<<ddxs[i]<<endl;
      lp.Print(cout);
      getchar();
      feasible=false;
    }
    /*
    if(i == 49) {
      cout<<"x = "<<xs[i]<<endl;
      cout<<"dx = "<<dxs[i]<<endl;
      cout<<"ddx = "<<ddxs[i]<<endl;
      lp.Print(cout);
      cout<<"Result: "<<endl;
      for(size_t j=0;j<poly.planes.size();j++) 
	cout<<poly.planes[j].normal.x<<" ds^2 + "<<poly.planes[j].normal.y<<" dds <= "<<poly.planes[j].offset<<endl;
      cout<<"Vertices: "<<endl;
      for(size_t j=0;j<poly.vertices.size();j++) {
	cout<<poly.vertices[j]<<endl;
      }
      getchar();
    }
    */
    ds2ddsConstraintNormals[i].resize(poly.planes.size());
    ds2ddsConstraintOffsets[i].resize(poly.planes.size());
    for(size_t j=0;j<poly.planes.size();j++) {
      ds2ddsConstraintNormals[i][j] = poly.planes[j].normal;
      ds2ddsConstraintOffsets[i][j] = poly.planes[j].offset;
      if(saveConstraintNames) {
        stringstream ss;
        ss<<"projected_constraint_plane_"<<j;
        ds2ddsConstraintNames[i].push_back(ss.str());
      }
    }
  }
  //done!
  return feasible;
}

bool ContactTimeScaling::Check(const MultiPath& path)
{
  RobotModel& robot = cspace.robot;
  //test at the colocation points
  Assert(traj.timeScaling.times.size()==paramDivs.size());
  Vector x,dx,ddx;
  bool feasible = true;
  for(size_t i=0;i<paramDivs.size();i++) {
    int seg=paramSections[i];
    Real t=traj.timeScaling.times[i];
    traj.Eval(t,x);
    traj.Deriv(t,dx);
    traj.Accel(t,ddx);

    for(int j=0;j<dx.n;j++)
      if(Abs(dx[j]) > robot.velMax[j]*(1+Epsilon)) {
	printf("Vel at param %d (time %g/%g) is infeasible\n",i,t,traj.timeScaling.times.back());
	printf("   |%g| > %g  at link %s\n",dx[j],robot.velMax[j],robot.LinkName(j).c_str());
	feasible = false;
      }

    for(int j=0;j<ddx.n;j++)
      if(Abs(ddx[j]) > robot.accMax[j]*(1+Epsilon)) {
	printf("Acc at param %d (time %g/%g) is infeasible\n",i,t,traj.timeScaling.times.back());
	printf("   |%g| > %g  at link %s\n",ddx[j],robot.accMax[j],robot.LinkName(j).c_str());
	feasible = false;
      }


    ContactFormation formation;
    Stance stance;
    path.GetStance(stance,seg);
    ToContactFormation(stance,formation);
    if(formation.contacts.empty()) continue;

    robot.UpdateConfig(x);
    robot.dq = dx;
    TorqueSolver solver(robot,formation);
    solver.SetGravity(Vector3(0,0,-9.8));
    solver.SetDynamics(ddx);
    bool res=solver.Solve();
    if(!res) {
      printf("TorqueSolver was not able to compute a solution at param %d (time %g/%g)\n",i,t,traj.timeScaling.times.back());
      feasible = false;
      continue;
    }
    for(int j=0;j<solver.t.n;j++) {
      if(Abs(solver.t(j)) > robot.torqueMax(j)*(1+Epsilon)) {
	printf("Torque at param %d (time %g/%g) is infeasible\n",i,t,traj.timeScaling.times.back());
	printf("   |%g| > %g  at link %s\n",solver.t(j),robot.torqueMax(j),robot.LinkName(j).c_str());
	feasible = false;
      }
    }
  }
  return feasible;
}
