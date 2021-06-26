#include "NumericalConstraint.h"
#include <KrisLibrary/math/differentiation.h>

namespace Klampt {

string JointLimitConstraint::Label() const { return "JointLimit"; }
string JointLimitConstraint::Label(int i) const 
{
  if(i>=bmin.n) i-=bmin.n;
  string str="JointLimit[";
  str += robot.linkNames[i];
  str += "]";
  return str;
}

SuppPolyConstraint::SuppPolyConstraint(RobotModel& _robot, SupportPolygon& _sp):robot(_robot),sp(_sp),cmInequality(A,b)
{
  //Initialize the linear inequality representation
  A.resize(sp.planes.size(),2);
  b.resize(sp.planes.size());
  //Ax>=b
  //normals of planes point outward, so valid region is n.x<=o
  for(int i=0;i<A.m;i++) {
    A(i,0) = -sp.planes[i].normal.x;
    A(i,1) = -sp.planes[i].normal.y;
    b(i) = -sp.planes[i].offset;
  }

}

string SuppPolyConstraint::Label() const { return "SuppPoly"; }

int SuppPolyConstraint::NumDimensions() const
{
  return sp.planes.size();
}

void SuppPolyConstraint::PreEval(const Vector& x)
{
  Assert(x.n == (int)robot.links.size());
  robot.UpdateConfig(x);
  Vector3 com=robot.GetCOM();
  vcom.resize(2);
  vcom.copy(com);

  Jcom.dirty = true;
  Hcomx.dirty = true;
  Hcomy.dirty = true;
  Hcomz.dirty = true;
}

void SuppPolyConstraint::Eval(const Vector& x,Vector& v)
{
  cmInequality.Eval(vcom,v);
}

Real SuppPolyConstraint::Eval_i(const Vector& x,int i)
{
  return cmInequality.Eval_i(vcom,i);
}


void SuppPolyConstraint::Jacobian(const Vector& x,Matrix& J)
{
  //f(x) = g(h(x)) => f'(x) = g'(h(x))*h'(x)
  Assert(x.n == (int)robot.links.size());
  //grasp jacobian is constant (=grasp.normals)

  if(Jcom.dirty) {
    robot.GetCOMJacobian(Jcom);
    Jcom.m=2;
    Jcom.dirty = false;
  }
  J.mul(A,Jcom);
}

void SuppPolyConstraint::Jacobian_i(const Vector& x,int i,Vector& Ji)
{
  if(Jcom.dirty) {
    robot.GetCOMJacobian(Jcom);
    Jcom.m=2;
    Jcom.dirty = false;
  }
  Vector gi(2);
  cmInequality.Jacobian_i(x,i,gi);
  Jcom.mulTranspose(gi,Ji);
}

void SuppPolyConstraint::Hessian_i(const Vector& x,int m,Matrix& Hm)
{
  //all hessians of the grasp are 0
  if(Hcomx.dirty) {
    Assert(Hcomy.dirty && Hcomz.dirty);
    robot.GetCOMHessian(Hcomx,Hcomy,Hcomz);
    Hcomx.dirty = Hcomy.dirty = Hcomz.dirty = false;
  }
  Hm.mul(Hcomx,cmInequality.Jacobian_ij(x,m,0));
  Hm.madd(Hcomy,cmInequality.Jacobian_ij(x,m,1));
  Hm.madd(Hcomz,cmInequality.Jacobian_ij(x,m,2));
}


Real SuppPolyConstraint::Margin(const Vector& x,int& minConstraint)
{
  return cmInequality.Margin(vcom,minConstraint);
}

bool SuppPolyConstraint::Satisfies_i(const Vector& x,int i,Real d)
{
  return cmInequality.Satisfies_i(vcom,i,d);
}



CollisionConstraint::CollisionConstraint(RobotModel& _robot, Geometry::AnyCollisionGeometry3D& _geometry):robot(_robot),geometry(_geometry)
{
	query.resize(robot.links.size());
	robot.InitMeshCollision(geometry);
}

string CollisionConstraint::Label() const { return "EnvCollision"; }

string CollisionConstraint::Label(int i) const 
{
	string str = "EnvCollision";
	str += "[";
	str += robot.LinkName(i);
	str += "]";
	return str;
}

void CollisionConstraint::PreEval(const Vector& x)
{
	//Yajia added
	Config q = robot.q;
	this->activeDofs.Map( x, q);

	robot.UpdateConfig(q);
	robot.UpdateGeometry();

	//update query status
	for(size_t i = 0; i < robot.links.size();i++) {
		if(robot.envCollisions[i]) {
			query[i].query = robot.envCollisions[i];
			query[i].NextCycle();
		}
	}
}

void CollisionConstraint::Eval(const Vector& x, Vector& v)
{
	Assert(v.n == (int)robot.links.size());
	for(size_t i=0;i<robot.links.size();i++)
		v(i) = Eval_i(x,i);

//	cout << "values:" << endl;
//	for( int i = 0; i < v.size(); i++)
//	{
//		cout << i << "\t" << v[i] << endl;
//	}
//	getchar();
	return;
}

Real CollisionConstraint::Eval_i(const Vector& x,int i)
{
  //Here we use the status of the previous evaluation to 
  //get some coherence (hopefully speeding up evaluation)
	if(!query[i].query)
		return Inf;
	Real res = query[i].UpdateQuery();
	if(res < 0)
		res -= 1e-2;
	return res;
}

void CollisionConstraint::Jacobian(const Vector& x,Matrix& J)
{
	Abort();
	Assert(J.hasDims(robot.links.size(),robot.links.size()));

	Vector3 cpi,cpmesh,dir;
	Vector3 dv;
	Real dirNorm = Zero;

	J.setZero();
	for(size_t i=0;i<robot.links.size();i++) {
		if(query[i].query && query[i].ClosestPoints(cpi,cpmesh,dir)) {
			dirNorm = dir.norm();
			if(dirNorm >= Epsilon)
				dir /= dirNorm;
			//jacobian Ji is Jpi*dir/|dir|
			for(int j=i;j!=-1;j=robot.parents[j]) {
				robot.GetPositionJacobian(cpi,i,j,dv);
				if(dirNorm < Epsilon)
					J(i,j) = dot(dv,dv);
				else
					J(i,j) = dot(dir,dv);
			}
		}
	}
}

void CollisionConstraint::Jacobian_i(const Vector& x,int i,Vector& Ji_active)
{
  //approximate jacobian -- assumes the cp doesn't change
  //D = cp(q)-cpmesh
  //d/dq |D| = d/dq sqrt(D.D) = 1/|D|*D^t*dD/dq = 1/|D|*D^t*Jcp

	Assert( Ji_active.n == (int)this->activeDofs.Size());
	Vector Ji; Ji.resize( robot.links.size());

	Vector3 cpi,cpmesh,dir;
	Vector3 dv;
	Real dirNorm;

	Ji.setZero();
	if( query[i].query && query[i].ClosestPoints(cpi,cpmesh,dir)) {
		dirNorm = dir.norm();
		if(dirNorm >= Epsilon)
			dir /= dirNorm;
		//jacobian Ji is Jpi*dir/|dir|
		for(int j = i; j != -1; j = robot.parents[j]) {
			robot.GetPositionJacobian(cpi,i,j,dv);
			if(dirNorm < Epsilon)
				Ji(j) = dot(dv,dv);
			else
				Ji(j) = dot(dir,dv);
		}
	}
//	cout << "Ji ********************* :" << endl;
//	cout << Ji << endl;
//	cout << "direction:" << dir << endl;
	/*
	Config q; readConfig("q_inCollision.txt", q);
	Config q_pushout = q + Ji;
	logConfig("q_pushout.txt", q_pushout);
	*/
//	getchar();
	for( int i = 0; i < this->activeDofs.Size(); i++)
	{
//		cout << "mapping " << i << ":" << this->activeDofs.mapping[i] << endl;
		Ji_active[i] = Ji[ this->activeDofs.mapping[i]];
	}
//	exit(-1);

//	Assert(Ji.n == (int)robot.links.size());
//
//	Vector3 cpi,cpmesh,dir;
//	Vector3 dv;
//	Real dirNorm;
//
//	Ji.setZero();
//	if( query[i].query && query[i].ClosestPoints(cpi,cpmesh,dir)) {
//		dirNorm = dir.norm();
//		if(dirNorm >= Epsilon)
//			dir /= dirNorm;
//		//jacobian Ji is Jpi*dir/|dir|
//		for(int j = i; j != -1; j = robot.parents[j]) {
//			robot.GetPositionJacobian(cpi,i,j,dv);
//			if(dirNorm < Epsilon)
//				Ji(j) = dot(dv,dv);
//			else
//				Ji(j) = dot(dir,dv);
//		}
//	}
}

void CollisionConstraint::Hessian_i(const Vector& x,int i,Matrix& Hi)
{
  //approximate Hessian -- assumes the cp AND normal dir doesn't change
  //D = |p(q)-pmesh| = y
  //d^2D/dq^2 ~= sum_k dD/dyk * d^2pk/dq^2= sum_k N * d^2pk/dq^2
	Abort();
  Assert(Hi.m == (int)robot.links.size() && Hi.n == (int)robot.links.size());

  Vector3 cpi,cpmesh,dir;
  Real dirNorm;

  Hi.resize(x.n,x.n,Zero);
  if(query[i].query && query[i].ClosestPoints(cpi,cpmesh,dir)) {
    dirNorm = dir.norm();
    if(dirNorm >= Epsilon) dir /= dirNorm;
    //copied from DynamicChain3D::GetDirectionalHessian
    Vector3 ddtheta,ddp;
    //NOTE: H(j,k) != 0 only if j,k are ancestors of i
    //Also H(j,k) is symmetric, only need to consider k as an ancestor of j (k<j)
    for(int j=i;j!=-1;j=robot.parents[j]) {
      for(int k=j;k!=-1;k=robot.parents[k]) {
	robot.GetJacobianDeriv_Fast(cpi,i,j,k,ddtheta,ddp);
	if(dirNorm < Epsilon) Hi(j,k)=Hi(k,j) = dot(ddp,ddp);
	else Hi(j,k) = Hi(k,j) = dot(dir,ddp);
      }
    }
  }
}

bool CollisionConstraint::Satisfies_i(const Vector& x,int i,Real d) 
{ 
	if(!robot.envCollisions[i]) return true;
	if(d == Zero) {
		return(!robot.envCollisions[i]->Collide());
	}
	else if(d > Zero) {
		return (!robot.envCollisions[i]->WithinDistance(d));
	}
	else {
		return (robot.envCollisions[i]->PenetrationDepth() <= -d);
	}
}

SelfCollisionConstraint::SelfCollisionConstraint(RobotModel& _robot)
  :robot(_robot)
{
  for(size_t i=0;i<robot.links.size();i++) 
    for(size_t j=0;j<robot.links.size();j++) 
      if(robot.selfCollisions(i,j))
	collisionPairs.push_back(pair<int,int>(i,j));
  query.resize(collisionPairs.size());
  int n=0;
  for(size_t i=0;i<robot.links.size();i++) 
    for(size_t j=0;j<robot.links.size();j++) 
      if(robot.selfCollisions(i,j)) {
	query[n].query = robot.selfCollisions(i,j);
	n++;
      }
}


string SelfCollisionConstraint::Label() const { return "SelfCollision"; }

string SelfCollisionConstraint::Label(int i) const 
{
  int a=collisionPairs[i].first;
  int b=collisionPairs[i].second;
  string str = "SelfCollision";
  str += "[";
  str += robot.LinkName(a);
  str += ",";
  str += robot.LinkName(b);
  str += "]";
  return str;
}

void SelfCollisionConstraint::PreEval(const Vector& x)
{
  robot.UpdateConfig(x);
  robot.UpdateGeometry();
  for(size_t i=0;i<query.size();i++)
    query[i].NextCycle();
}

void SelfCollisionConstraint::Eval(const Vector& x, Vector& v)
{
  int n=NumDimensions();
  Assert(v.n == n);
  for(int i=0;i<n;i++) {
    v(i) = Eval_i(x,i);
  }
}

Real SelfCollisionConstraint::Eval_i(const Vector& x,int i)
{
  Real res=query[i].UpdateQuery();
  if(res < 0) res -= 1e-2;
  return res;
}

void SelfCollisionConstraint::Jacobian(const Vector& x,Matrix& J)
{
  int n=NumDimensions();
  Assert(J.hasDims(n,robot.links.size()));

  Vector3 cpa,cpb,dir;
  //Vector3 cpa_world,cpb_world;
  Vector3 dv;
  Real dirNorm=Zero;

  J.setZero();
  for(int i=0;i<n;i++) {
    int a=collisionPairs[i].first,b=collisionPairs[i].second;

    if(query[i].ClosestPoints(cpa,cpb,dir)) {
      dirNorm = dir.norm();

      int lca = robot.LCA(a,b);
      //jacobian Ji is (Jpa*dir-Jpb*dir)/|dir|
      for(int j=a;j!=lca;j=robot.parents[j]) {
	robot.GetPositionJacobian(cpa,a,j,dv);
	if(dirNorm < Epsilon)
	  J(i,j) = dot(dv,dv);
	else 
	  J(i,j) = dot(dir,dv) / dirNorm;
      }
      for(int j=b;j!=lca;j=robot.parents[j]) {
	robot.GetPositionJacobian(cpb,b,j,dv);
	if(dirNorm < Epsilon)
	  J(i,j) -= dot(dv,dv);
	else 
	  J(i,j) -= dot(dir,dv) / dirNorm;
      }
    }
  }
}

void SelfCollisionConstraint::Jacobian_i(const Vector& x,int i,Vector& Ji)
{
  //approximate jacobian -- assumes the cp doesn't change
  //D = cp(q)-cpmesh
  //d/dq |D| = d/dq sqrt(D.D) = 1/|D|*D^t*dD/dq = 1/|D|*D^t*Jcp
  Assert(Ji.n == (int)robot.links.size());

  Vector3 cpa,cpb,dir;
  //Vector3 cpa_world,cpb_world;
  Vector3 dv;
  Real dirNorm;
  int a=collisionPairs[i].first,b=collisionPairs[i].second;

  Ji.setZero();
  if(query[i].ClosestPoints(cpa,cpb,dir)) {
    dirNorm = dir.norm();
  
    //jacobian Ji is (Jpa*dir-Jpb*dir)/|dir|
    if(dirNorm >= Epsilon) dir /= dirNorm;
    for(int j=a;j!=-1;j=robot.parents[j]) {
      robot.GetPositionJacobian(cpa,a,j,dv);
      if(dirNorm < Epsilon)
	Ji(j) = dot(dv,dv);
      else 
	Ji(j) = dot(dir,dv);
    }
    for(int j=b;j!=-1;j=robot.parents[j]) {
      robot.GetPositionJacobian(cpb,b,j,dv);
      if(dirNorm < Epsilon)
	Ji(j) -= dot(dv,dv);
      else 
	Ji(j) -= dot(dir,dv);
    }
  }
}

void SelfCollisionConstraint::Hessian_i(const Vector& x,int i,Matrix& Hi)
{
  //approximate jacobian -- assumes the cp AND normal doesn't change
  //D = |pa(q)-pb(p)| = y
  //d^2D/dq^2 ~= sum_k dD/dyk * (d^2pak/dq^2 - d^2pbk/dq^2)
  Assert(Hi.m == (int)robot.links.size() && Hi.n == (int)robot.links.size());

  Vector3 cpa,cpb,dir;
  //Vector3 cpa_world,cpb_world;
  Real dirNorm;
  int a=collisionPairs[i].first,b=collisionPairs[i].second;

  Hi.resize(x.n,x.n,Zero);
  if(query[i].ClosestPoints(cpa,cpb,dir)) {
    dirNorm = dir.norm();
  
    Vector3 ddtheta,ddp;
    //NOTE: H(j,k) != 0 only if j,k are ancestors of i
    //Also H(j,k) is symmetric, only need to consider k as an ancestor of j (k<j)
    for(int j=a;j!=-1;j=robot.parents[j]) {
      for(int k=j;k!=-1;k=robot.parents[k]) {
	robot.GetJacobianDeriv_Fast(cpa,a,j,k,ddtheta,ddp);
	if(dirNorm < Epsilon) Hi(j,k) = dot(ddp,ddp);
	else Hi(j,k) = dot(dir,ddp);
      }
    }
    for(int j=b;j!=-1;j=robot.parents[j]) {
      for(int k=j;k!=-1;k=robot.parents[k]) {
	robot.GetJacobianDeriv_Fast(cpb,b,j,k,ddtheta,ddp);
	if(dirNorm < Epsilon) {
	  Hi(j,k) -= dot(ddp,ddp);
//	  Hi(k,j) = Hi(k,j);
	}
	else {
	  Hi(j,k) -= dot(dir,ddp);
	  Hi(k,j) = Hi(j,k);
	}
      }
    }
  }
}

bool SelfCollisionConstraint::Satisfies_i(const Vector& x,int i,Real d) 
{ 
  int a=collisionPairs[i].first;
  int b=collisionPairs[i].second;
  if(d == Zero) {
    return (!robot.selfCollisions(a,b)->Collide());
  }
  else if(d > Zero) {
    return (!robot.selfCollisions(a,b)->WithinDistance(d));
  }
  else {
    return (robot.selfCollisions(a,b)->PenetrationDepth() <= -d);
  }
}




TorqueLimitConstraint::TorqueLimitConstraint(TorqueSolver& _solver)
  :solver(_solver)
{
}

string TorqueLimitConstraint::Label() const { return "TorqueLimit"; }
int TorqueLimitConstraint::NumDimensions() const { return 1; }

void TorqueLimitConstraint::PreEval(const Vector& x)
{
  solver.robot.UpdateConfig(x);
  solver.Solve();
}

void TorqueLimitConstraint::Eval(const Vector& x,Vector& v)
{
  Real maxSat = 0;
  for(int i=0;i<solver.t.n;i++) {
    Real sat = solver.t(i) / solver.robot.torqueMax(i);
    if(sat > maxSat) maxSat = sat;
  }
  v(0) = 1-maxSat;
}

} //namespace Klampt
