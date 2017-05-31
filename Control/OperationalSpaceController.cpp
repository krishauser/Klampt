#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "OperationalSpaceController.h"
#include <KrisLibrary/robotics/NewtonEuler.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include <KrisLibrary/math/indexing.h>
#include <KrisLibrary/math/VectorPrinter.h>
#include <KrisLibrary/optimization/MinNormProblem.h>
using namespace Optimization;

#define OPTIMIZE_DRIVER_TORQUES 0

int kNumFCEdges = 4;

//from link velocities/torques to driver velocities/torques
void MulDriverJacobian(Robot& robot,const Vector& in,Vector& out)
{
  assert(in.n == (int)robot.links.size());
  out.resize(robot.drivers.size());
  out.setZero();
  for(size_t i=0;i<robot.drivers.size();i++) {
    if(robot.drivers[i].type == RobotJointDriver::Normal) 
      out[i] = in[robot.drivers[i].linkIndices[0]];
    else {
      Vector Ji;
      robot.GetDriverJacobian(i,Ji);
      out[i] = Ji.dot(in);
    }
  }
}

//from driver velocities/torques to link velocities/torques
void MulDriverJacobianT(Robot& robot,const Vector& in,Vector& out)
{
  assert(in.n == (int)robot.drivers.size());
  out.resize(robot.links.size());
  out.setZero();
  for(size_t i=0;i<robot.drivers.size();i++) {
    if(robot.drivers[i].type == RobotJointDriver::Normal) 
      out[robot.drivers[i].linkIndices[0]] = in[i];
    else {
      Vector Ji;
      robot.GetDriverJacobian(i,Ji);
      out.madd(Ji,in[i]);
    }
  }
}

//from link velocities/torques to driver velocities/torques
//the in matrix is of size num_links x m, and out will be of size num_drivers x m
//equivalent to out = Jd*in
void MulDriverJacobian(Robot& robot,const Matrix& in,Matrix& out)
{
  assert(in.m == (int)robot.links.size());
  out.resize(robot.drivers.size(),in.n);
  for(int i=0;i<in.n;i++) {
    Vector vin,vout;
    in.getColRef(i,vin);
    out.getColRef(i,vout);
    MulDriverJacobian(robot,vin,vout);
  }
}


//from driver velocities/torques to link velocities/torques
//the in matrix is of size num_drivers x m, and out will be of size num_links x m
//equivalent to out = Jd*in
void MulDriverJacobianT(Robot& robot,const Matrix& in,Matrix& out)
{
  assert(in.m == (int)robot.drivers.size());
  out.resize(robot.links.size(),in.n);
  for(int i=0;i<in.n;i++) {
    Vector vin,vout;
    in.getColRef(i,vin);
    out.getColRef(i,vout);
    MulDriverJacobianT(robot,vin,vout);
  }
}

//from linke velocities/torques to driver velocities/torques
//the in matrix is of size m x num_links, and out will be of size m x num_drivers
//equivalent to out = in*Jd^T
void PostMulDriverJacobianT(Robot& robot,const Matrix& in,Matrix& out)
{
  assert(in.n == (int)robot.links.size());
  out.resize(in.m,robot.drivers.size());
  for(int i=0;i<in.m;i++) {
    Vector vin,vout;
    in.getRowRef(i,vin);
    out.getRowRef(i,vout);
    MulDriverJacobian(robot,vin,vout);
  }
}


//from driver velocities/torques to link velocities/torques
//the in matrix is of size m x num_drivers, and out will be of size m x num_links
//equivalent to out = in*Jd
void PostMulDriverJacobian(Robot& robot,const Matrix& in,Matrix& out)
{
  assert(in.n == (int)robot.drivers.size());
  out.resize(in.m,robot.links.size());
  for(int i=0;i<in.m;i++) {
    Vector vin,vout;
    in.getRowRef(i,vin);
    out.getRowRef(i,vout);
    MulDriverJacobianT(robot,vin,vout);
  }
}


void GetGoalAccel0(RobotDynamics3D& robot,const IKGoal& goal,Vector& ddx0)
{
  //get ddx0
  Vector3 ddp0(Zero),ddr0(Zero);
  robot.GetResidualAcceleration(goal.localPosition,goal.link,ddr0,ddp0);
  ddx0.resize(IKGoal::NumDims(goal.posConstraint)+IKGoal::NumDims(goal.rotConstraint));
  int nofs=0;
  if(goal.posConstraint == IKGoal::PosFixed) {
    ddp0.get(ddx0(0),ddx0(1),ddx0(2));
    nofs = 3;
  }
  else if(goal.posConstraint == IKGoal::PosNone) {
  }
  else {
    FatalError("TODO: other position tasks");
  }
  if(goal.rotConstraint == IKGoal::RotFixed) {
    ddr0.get(ddx0(nofs),ddx0(nofs+1),ddx0(nofs+2));
  }
  else if(goal.rotConstraint == IKGoal::RotNone) {
  }
  else {
    FatalError("TODO: other position tasks");
  }
}

OperationalSpaceController::OperationalSpaceController(Robot& _robot)
  :RobotController(_robot),gravity(0,0,-9.8)
{
  stateEstimator = new IntegratedStateEstimator(_robot);
}

bool OperationalSpaceController::IsValid() const
{
  for(size_t i=0;i<jointTasks.size();i++) {
    if(jointTasks[i].ddqdes.n != (int)jointTasks[i].indices.size()) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"OperationalSpaceController::IsValid(): invalid ddqdes["<<i);
      return false;
    }
    for(size_t j=0;j<jointTasks[i].indices.size();j++) {
      if(jointTasks[i].indices[j] < 0 || jointTasks[i].indices[j] > (int)robot.links.size()) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"OperationalSpaceController::IsValid(): invalid joint index["<<i<<"]["<<j<<"]="<<jointTasks[i].indices[j]);
	return false;
      }
    }
  }
  for(size_t i=0;i<workspaceTasks.size();i++) {
    if(workspaceTasks[i].workspace.link < 0 || workspaceTasks[i].workspace.link > (int)robot.links.size()) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"OperationalSpaceController::IsValid(): invalid workspace index["<<i<<"]="<<workspaceTasks[i].workspace.link);
	return false;
    }
  }
  for(size_t i=0;i<comTasks.size();i++) {
    if(comTasks[i].numAxes < 0 || comTasks[i].numAxes > 3) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"OperationalSpaceController::IsValid(): invalid com axes["<<i<<"] = "<<comTasks[i].numAxes);
      return false;
    }
    if(comTasks[i].numAxes != comTasks[i].ddxdes.n) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"OperationalSpaceController::IsValid(): invalid com axes["<<i<<"] = "<<comTasks[i].numAxes);
      return false;
    }
  }
  for(size_t i=0;i<torqueTasks.size();i++) { 
    if(torqueTasks[i].Tdes.n != (int)torqueTasks[i].indices.size()) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"OperationalSpaceController::IsValid(): invalid Tdes["<<i);
      return false;
    }
#if OPTIMIZE_DRIVER_TORQUES
    int maxtorqueind = (int)robot.drivers.size();
#else
    int maxtorqueind = (int)robot.links.size();
#endif //OPTIMIZE_DRIVER_TORQUES
    for(size_t j=0;j<torqueTasks[i].indices.size();j++) {
      if(torqueTasks[i].indices[j] < 0 || torqueTasks[i].indices[j] > maxtorqueind) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"OperationalSpaceController::IsValid(): invalid torque index["<<i<<"]["<<j<<"]="<<torqueTasks[i].indices[j]);
	return false;
      }
    }
  }
  for(size_t i=0;i<contactForceTasks.size();i++) {
    if(contactForceTasks[i].links.size() != contactForceTasks[i].contacts.size()) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"OperationalSpaceController::IsValid(): invalid contact links["<<i);
      return false;
    }
    for(size_t j=0;j<contactForceTasks[i].links.size();j++) {
      if(contactForceTasks[i].links[j] < 0 || contactForceTasks[i].links[j] > (int)robot.links.size()) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"OperationalSpaceController::IsValid(): invalid contact link["<<i<<"]["<<j<<"]="<<contactForceTasks[i].links[j]);
	return false;
      }
    }
  }
  return true;
}

//subclasses fill these out
void OperationalSpaceController::Update(Real dt)
{
  if(!IsValid()) {
    FatalError("OperationalSpaceController not set up correctly");
  }
  if(stateEstimator) {
    Vector q_predicted = stateEstimator->q_predicted;
    Vector dq_predicted = stateEstimator->dq_predicted;
    stateEstimator->ReadSensors(*sensors);
    stateEstimator->UpdateModel();
    /*
    LOG4CXX_INFO(KrisLibrary::logger(),"Estimated state: "<<"\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"   "<<stateEstimator->q_predicted<<"\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"   "<<stateEstimator->dq_predicted<<"\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"Forward predicted state"<<"\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"   "<<q_predicted<<"\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"   "<<dq_predicted<<"\n");
    LOG4CXX_ERROR(KrisLibrary::logger(),"   DDQ error "<<(dq_predicted-stateEstimator->dq_predicted)/dt<<"\n");
    */
  }
  Vector torques;
  TasksToTorques(torques);
  //LOG4CXX_INFO(KrisLibrary::logger(),torques<<"\n");
#if OPTIMIZE_DRIVER_TORQUES
  command->SetTorque(torques);
#else
  //convert to driver torques
  Vector dtorques;
  MulDriverJacobian(robot,torques,dtorques);
  command->SetTorque(dtorques);
#endif

  if(stateEstimator) {
    stateEstimator->ReadCommand(*command);
    stateEstimator->Advance(dt);
  }
  RobotController::Update(dt);
}

void OperationalSpaceController::Reset()
{
  RobotController::Reset(); 
  if(stateEstimator) stateEstimator->Reset();
} 
/*
  virtual bool ReadState(File& f) {
  if(!ReadFile(f,time)) return false;
  return true;
  }
  virtual bool WriteState(File& f) const {
  if(!WriteFile(f,time)) return false;
  return true;
  }
*/

void OperationalSpaceController::TasksToTorques(Vector& t) 
{
#if OPTIMIZE_DRIVER_TORQUES
  int numTorques = (int)robot.drivers.size();
#else
  int numTorques = (int)robot.links.size();
#endif //OPTIMIZE_DRIVER_TORQUES
  t.resize(numTorques);

  NewtonEulerSolver nr(robot);
  nr.SetGravityWrenches(gravity);

  //use torques that closely satisfy ddq
  Matrix Binv;
  nr.CalcKineticEnergyMatrixInverse(Binv);
  Vector ddq0;
  nr.CalcResidualAccel(ddq0);

  Matrix BinvJdT;
#if OPTIMIZE_DRIVER_TORQUES
  PostMulDriverJacobianT(robot,Binv,BinvJdT);
  //LOG4CXX_INFO(KrisLibrary::logger(),"B^{-1}Jd^T: "<<"\n");
  //LOG4CXX_INFO(KrisLibrary::logger(),BinvJdT<<"\n");

  //look for fixed links -- todo: add this as a constraint to the LP solver
  for(size_t i=0;i<robot.joints.size();i++)
    if(robot.joints[i].type == RobotJoint::Weld) {
      assert(robot.parents[robot.joints[i].linkIndex] == -1);
      ddq0(robot.joints[i].linkIndex) = 0;
      Vector temp;
      Binv.getColRef(robot.joints[i].linkIndex,temp);
      temp.setZero();
      Binv.getRowRef(robot.joints[i].linkIndex,temp);
      temp.setZero();
    }
#else
  BinvJdT.setRef(Binv);
#endif //OPTIMIZE_DRIVER_TORQUES

  int numContactPoints=0;
  int numContactForces=0;
  for(size_t i=0;i<contactForceTasks.size();i++) {
    numContactPoints += contactForceTasks[i].contacts.size();
    vector<ContactPoint> newContacts;
    FrictionToFrictionlessContacts(contactForceTasks[i].contacts,kNumFCEdges,newContacts);
    numContactForces += newContacts.size();
  }

  //compute the jacobian of contact forces
  Matrix Jfx,Jf;
  Vector ddxf0;
  if(numContactPoints != 0) {
    Jfx.resize(numContactPoints,robot.links.size());
    Jf.resize(numContactForces,robot.links.size());
    ddxf0.resize(numContactPoints);
  }
  int xindex=0;
  int findex=0;
  for(size_t i=0;i<contactForceTasks.size();i++) {
    for(size_t j=0;j<contactForceTasks[i].contacts.size();j++,xindex++) {
      Assert(contactForceTasks[i].links.size()==contactForceTasks[i].contacts.size());
      int link = contactForceTasks[i].links[j];
      const ContactPoint& cp = contactForceTasks[i].contacts[j];
      Vector3 pw = robot.links[link].T_World*cp.x;
      Vector3 ploc = cp.x;
      Vector3 nloc = cp.n;
      Vector3 nw = robot.links[link].T_World.R*cp.n;
      Matrix Jfi;
      robot.GetPositionJacobian(ploc,link,Jfi);
      Vector3 ddr0,ddp0;
      robot.GetResidualAcceleration(ploc,link,ddr0,ddp0);

      Vector Jff;
      Jfx.getRowRef(xindex,Jff);
      Jfi.mulTranspose(Vector(3,cp.n),Jff);
      ddxf0[xindex] = nw.dot(ddp0);
      Jff *= contactForceTasks[i].penetrationWeight;
      ddxf0[xindex] *= contactForceTasks[i].penetrationWeight;

      //change to normal coordinates
      if(cp.kFriction == 0) {
	//straight copy
	Vector Jff;
	Jf.getRowRef(findex,Jff);
	Jfi.mulTranspose(Vector(3,nw),Jff);
	findex++;
      }
      else {
	FrictionConePolygon fc;
	fc.set(kNumFCEdges,cp.n,cp.kFriction);
	Vector Jff;
	for(int e=0;e<kNumFCEdges;e++) {
	  Jf.getRowRef(findex,Jff);
	  Jfi.mulTranspose(Vector(3,robot.links[link].T_World.R*fc.edges[e]),Jff);
	  findex++;
	}
      }
    }
  }
  assert(xindex == numContactPoints);
  assert(findex == numContactForces);

  //start putting this together
  MinNormProblem lp;
  lp.norm = 1;
  int numTasks = 0;
  for(size_t i=0;i<jointTasks.size();i++)
    numTasks += (int)jointTasks[i].indices.size();
  for(size_t i=0;i<workspaceTasks.size();i++)
    numTasks += (int)workspaceTasks[i].ddxdes.size();
  for(size_t i=0;i<comTasks.size();i++)
    numTasks += comTasks[i].numAxes;
  for(size_t i=0;i<torqueTasks.size();i++)
    numTasks += (int)torqueTasks[i].indices.size();
  for(size_t i=0;i<contactForceTasks.size();i++)
    numTasks += contactForceTasks[i].A.m;
  numTasks += numContactPoints;
  LOG4CXX_INFO(KrisLibrary::logger(),"OperationalSpaceController: "<<numTasks<<" tasks, "<<numContactPoints<<" contacts"<<"\n");
  //LOG4CXX_INFO(KrisLibrary::logger(),"ddq0: "<<ddq0<<"\n");
  lp.C.resize(numTasks,numTorques+numContactForces);
  lp.d.resize(numTasks);
  lp.l.resize(numTorques+numContactForces,Zero);
  lp.u.resize(numTorques+numContactForces,Inf);
#if OPTIMIZE_DRIVER_TORQUES
  for(size_t i=0;i<robot.drivers.size();i++) {
    //TODO: find dynamic torque limits via powers?
    lp.l[i] = robot.drivers[i].tmin;
    lp.u[i] = robot.drivers[i].tmax;
  }
#else
  for(size_t i=0;i<robot.links.size();i++) {
    //TODO: find dynamic torque limits via powers?
    lp.l[i] = -robot.torqueMax[i];
    lp.u[i] = robot.torqueMax[i];
  }
#endif //OPTIMIZE_DRIVER_TORQUES

  numTasks = 0;
  //B^-1*(Jd^T T + Jf^T*f) - (ddqdes-ddq0)
  for(size_t i=0;i<jointTasks.size();i++) {
    Matrix atemp,atemp2;
    Vector btemp;
    btemp.setRef(lp.d,numTasks,1,jointTasks[i].indices.size());
    atemp.setRef(lp.C,numTasks,0,1,1,jointTasks[i].indices.size(),numTorques);
    GetElements(ddq0,jointTasks[i].indices,btemp);
    btemp = jointTasks[i].ddqdes - btemp;
    GetRows(BinvJdT,jointTasks[i].indices,atemp);
    atemp *= jointTasks[i].weight;
    btemp *= jointTasks[i].weight;
    //LOG4CXX_INFO(KrisLibrary::logger(),"ddqdes - ddq0: "<<"\n");
    //LOG4CXX_INFO(KrisLibrary::logger(),btemp<<"\n");
    if(numContactForces > 0) {
      atemp2.setRef(lp.C,numTasks,numTorques,1,1,jointTasks[i].indices.size(),numContactForces);
      atemp2.mulTransposeB(atemp,Jf);
      atemp2 *= jointTasks[i].weight;
    }
    numTasks += (int)jointTasks[i].indices.size();
  }
  //Jx*B^-1*(Jd^T T + Jf^T*f) - (ddxdes-ddx0-Jx*ddq0)
  for(size_t i=0;i<workspaceTasks.size();i++) {
    int nt = workspaceTasks[i].ddxdes.size();
    const IKGoal& goal = workspaceTasks[i].workspace;
    Matrix Jx(nt,robot.links.size());
    ArrayMapping allActive;
    allActive.imax = robot.links.size();
    IKGoalFunction gf(robot,goal,allActive);
    gf.Jacobian(robot.q,Jx);
    Vector ddx0;
    GetGoalAccel0(robot,goal,ddx0);

    Matrix atemp,atemp2;
    Vector btemp;
    btemp.setRef(lp.d,numTasks,1,nt);
    atemp.setRef(lp.C,numTasks,0,1,1,nt,numTorques);
    Jx.mul(ddq0,btemp);
    btemp = workspaceTasks[i].ddxdes - btemp - ddx0;
    atemp.mul(Jx,BinvJdT);
    atemp *= workspaceTasks[i].weight;
    btemp *= workspaceTasks[i].weight;
    if(numContactForces > 0) {
      atemp2.setRef(lp.C,numTasks,numTorques,1,1,nt,numContactForces);
      atemp2.mulTransposeB(atemp,Jf);
      atemp2 *= workspaceTasks[i].weight;
    }
    numTasks += nt;
  }
  for(size_t i=0;i<comTasks.size();i++) {
    int nt = comTasks[i].numAxes;
    Matrix Jcm;
    robot.GetCOMJacobian(Jcm);
    Vector3 ddcm0(Zero);
    Vector3 dw,dv;
    for(size_t k=0;k<robot.links.size();k++) {
      if(robot.links[k].mass == 0) continue;
      robot.GetResidualAcceleration(robot.links[k].com,k,dw,dv);
      ddcm0 += robot.links[k].mass*dv;
    }
    ddcm0 /= robot.GetTotalMass();
    Matrix Jx(comTasks[i].numAxes,Jcm.n);
    Vector ddx0(comTasks[i].numAxes);
    ddcm0 = comTasks[i].R*ddcm0;
    for(size_t k=0;k<robot.links.size();k++) {
      Vector3 val=comTasks[i].R*Vector3(Jcm(0,k),Jcm(1,k),Jcm(2,k));
      Jx.copyCol(k,val);
    }
    ddx0.copy(ddcm0);

    Matrix atemp,atemp2;
    Vector btemp;
    btemp.setRef(lp.d,numTasks,1,nt);
    atemp.setRef(lp.C,numTasks,0,1,1,nt,numTorques);
    Jx.mul(ddq0,btemp);
    btemp = comTasks[i].ddxdes - btemp - ddx0;
    atemp.mul(Jx,BinvJdT);
    atemp *= comTasks[i].weight;
    btemp *= comTasks[i].weight;
    if(numContactForces > 0) {
      atemp2.setRef(lp.C,numTasks,numTorques,1,1,nt,numContactForces);
      atemp2.mulTransposeB(atemp,Jf);
      atemp2 *= comTasks[i].weight;
    }
    numTasks += nt;
  }
  //Jd^T*T+0*f - Tdes
  for(size_t i=0;i<torqueTasks.size();i++) {
    int nt = (int)torqueTasks[i].indices.size();
    if(nt==0) continue;
    Matrix atemp,atemp2;
    Vector btemp;
    btemp.setRef(lp.d,numTasks,1,nt);
    atemp.setRef(lp.C,numTasks,0,1,1,nt,numTorques);
    atemp2.setRef(lp.C,numTasks,numTorques,1,1,nt,numContactForces);
    atemp2.setZero();
    atemp.setZero();
    btemp = torqueTasks[i].Tdes;
    for(size_t j=0;j<torqueTasks[i].indices.size();j++)
      atemp(j,torqueTasks[i].indices[j])=1.0;
    atemp *= torqueTasks[i].weight;
    atemp2 *= torqueTasks[i].weight;
    btemp *= torqueTasks[i].weight;
    numTasks += nt;
  }
  //0*T+A*f - fdes
  for(size_t i=0;i<contactForceTasks.size();i++) {
    int nt = contactForceTasks[i].A.m;
    if(nt==0) continue;
    Matrix atemp,atemp2;
    Vector btemp;
    btemp.setRef(lp.d,numTasks,1,nt);
    atemp.setRef(lp.C,numTasks,0,1,1,nt,numTorques);
    atemp.setZero();
    atemp2.setRef(lp.C,numTasks,numTorques,1,1,nt,numContactForces);
    atemp2 = contactForceTasks[i].A;
    btemp = contactForceTasks[i].fdes;
    atemp *= contactForceTasks[i].weight;
    atemp2 *= contactForceTasks[i].weight;
    btemp *= contactForceTasks[i].weight;
    numTasks += nt;
  }
  //contact normal position constraints
  //Jfx*B^-1*(Jd^T T + Jf^T*f) - (-ddxf0-Jfx*ddq0)
  if(!Jfx.isEmpty()) {
    int nt = Jfx.m;
    Matrix atemp,atemp2;
    Vector btemp;
    btemp.setRef(lp.d,numTasks,1,nt);
    atemp.setRef(lp.C,numTasks,0,1,1,nt,numTorques);
    atemp2.setRef(lp.C,numTasks,numTorques,1,1,nt,numContactForces);
    atemp.mul(Jfx,Binv);
    atemp2.mulTransposeB(atemp,Jf);
    Jfx.mul(ddq0,btemp);
    btemp += ddxf0;
    btemp.inplaceNegative();
    numTasks += nt;
  }
  assert(numTasks == lp.C.m);
  assert(numTasks == lp.d.n);
  /*
  LOG4CXX_INFO(KrisLibrary::logger(),"C:"<<"\n");
  LOG4CXX_INFO(KrisLibrary::logger(),lp.C<<"\n");
  LOG4CXX_INFO(KrisLibrary::logger(),"d: "<<lp.d<<"\n");
  */

  Assert(lp.IsValid());
  lp.Assemble();
  Vector x,f;
  LinearProgram::Result res=lp.Solve(x);
  switch(res) {
  case LinearProgram::Feasible:
  case LinearProgram::Error:
    x.getSubVectorCopy(0,t);
    f.setRef(x,t.n,1,numContactForces);
    //LOG4CXX_INFO(KrisLibrary::logger(),"Commanded torques: "<<VectorPrinter(t,VectorPrinter::AsciiShade)<<"\n");
    LOG4CXX_ERROR(KrisLibrary::logger(),"L"<<lp.norm<<" error: "<<lp.Norm(x)<<"\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"solved t: "<<VectorPrinter(t)<<"\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"solved f: "<<VectorPrinter(f)<<"\n");
    if(lp.Norm(x) > 10 ) {
      Vector temp;
      lp.C.mul(x,temp);
      temp -= lp.d;
      LOG4CXX_ERROR(KrisLibrary::logger(),"Errors:"<<"\n");
      int numTasks = 0;
      for(size_t i=0;i<jointTasks.size();i++) {
	Vector vtemp;
	vtemp.setRef(temp,numTasks,1,jointTasks[i].indices.size());
	LOG4CXX_INFO(KrisLibrary::logger(),"  Joint task "<<i<<": "<<vtemp<<"\n");
	numTasks += (int)jointTasks[i].indices.size();
      }
      for(size_t i=0;i<workspaceTasks.size();i++) {
	Vector vtemp;
	vtemp.setRef(temp,numTasks,1,workspaceTasks[i].ddxdes.size());
	LOG4CXX_INFO(KrisLibrary::logger(),"  Workspace task "<<i<<": "<<vtemp<<"\n");
	numTasks += (int)workspaceTasks[i].ddxdes.size();
      }
      for(size_t i=0;i<torqueTasks.size();i++) {
	Vector vtemp;
	vtemp.setRef(temp,numTasks,1,torqueTasks[i].indices.size());
	LOG4CXX_INFO(KrisLibrary::logger(),"  Torque task "<<i<<": "<<vtemp<<"\n");
	numTasks += (int)torqueTasks[i].indices.size();
      }
      for(size_t i=0;i<contactForceTasks.size();i++) {
	Vector vtemp;
	vtemp.setRef(temp,numTasks,1,contactForceTasks[i].A.m);
	LOG4CXX_INFO(KrisLibrary::logger(),"  Force task "<<i<<": "<<vtemp<<"\n");
	numTasks += (int)contactForceTasks[i].A.m;
      }
      if(!Jfx.isEmpty()) {
	Vector vtemp;
	vtemp.setRef(temp,numTasks,1,Jfx.m);
	LOG4CXX_INFO(KrisLibrary::logger(),"  Contact pt task "<<": "<<vtemp<<"\n");
      }
      //LOG4CXX_INFO(KrisLibrary::logger(),Jfx<<"\n");
      //if(KrisLibrary::logger()->isEnabledFor(log4cxx::Level::ERROR_INT)) getchar();
    }
    break;
  default:
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error computing torques! result "<<res<<"\n");
    if(KrisLibrary::logger()->isEnabledFor(log4cxx::Level::ERROR_INT)) getchar();
    t.setZero();
    f.resize(numContactForces,0);
    break;
  }
  if(stateEstimator) {
    Vector tl,Tf,ddq_predicted;
#if OPTIMIZE_DRIVER_TORQUES
    //driver to link torques
    MulDriverJacobianT(robot,t,tl);
#else
    tl = t;
#endif //OPTIMIZE_DRIVER_TORQUES
    if(numContactForces != 0) {
      Jf.mulTranspose(f,Tf);
      tl += Tf;
    }
    nr.CalcAccel(tl,ddq_predicted);
    LOG4CXX_INFO(KrisLibrary::logger(),"Predicted q'': "<<ddq_predicted<<"\n");
    stateEstimator->SetDDQ(ddq_predicted);
  }
}
