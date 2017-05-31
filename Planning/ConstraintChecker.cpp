#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "ConstraintChecker.h"
#include <KrisLibrary/robotics/IKFunctions.h>
#include <KrisLibrary/robotics/Stability.h>
#include <KrisLibrary/robotics/TorqueSolver.h>

Real ConstraintChecker::ContactDistance(const Robot& robot,const Stance& stance)
{
  Real maxErr = 0;
  Vector res;
  for(Stance::const_iterator i=stance.begin();i!=stance.end();i++) {
    const IKGoal& g=i->second.ikConstraint;
    res.resize(IKGoal::NumDims(g.posConstraint)+IKGoal::NumDims(g.rotConstraint));
    EvalIKError(g,robot.links[i->first].T_World,res);
    Real m=res.maxAbsElement();
    if(m > maxErr) maxErr = m;
  }
  return maxErr;
}

bool ConstraintChecker::HasContact(const Robot& robot,const Stance& stance,Real dist)
{
  Vector res;
  for(Stance::const_iterator i=stance.begin();i!=stance.end();i++) {
    const IKGoal& g=i->second.ikConstraint;
    res.resize(IKGoal::NumDims(g.posConstraint)+IKGoal::NumDims(g.rotConstraint));
    EvalIKError(g,robot.links[i->first].T_World,res);
    if(res.maxAbsElement() > dist) return false;
  }
  return true;
}

bool ConstraintChecker::HasContactVelocity(const Robot& robot,const Stance& stance,Real maxErr)
{
  Vector res;
  Vector3 dw,dv;
  for(Stance::const_iterator i=stance.begin();i!=stance.end();i++) {
    const IKGoal& g=i->second.ikConstraint;
    res.resize(IKGoal::NumDims(g.posConstraint)+IKGoal::NumDims(g.rotConstraint));
    robot.GetWorldAngularVelocity(g.link,robot.dq,dw);
    robot.GetWorldVelocity(Vector3(Zero),g.link,robot.dq,dv);
    EvalIKGoalDeriv(g,robot.links[i->first].T_World,dw,dv,res);
    if(res.maxAbsElement() > maxErr) return false;
  }
  return true;
}

bool ConstraintChecker::HasJointLimits(const Robot& robot) 
{
  return robot.InJointLimits(robot.q);
}

bool ConstraintChecker::HasVelocityLimits(const Robot& robot) 
{
  return robot.InVelocityLimits(robot.dq);
}

bool ConstraintChecker::HasSupportPolygon(const Robot& robot,const Stance& stance,const Vector3& gravity,int numFCEdges) 
{
  vector<ContactPoint> cps(NumContactPoints(stance));
  int k=0;
  for(Stance::const_iterator i=stance.begin();i!=stance.end();i++) {
    const Hold& h=i->second;
    for(size_t j=0;j<h.contacts.size();j++,k++)
      cps[k]=h.contacts[j];
  }
  Vector3 com = robot.GetCOM();
  vector<Vector3> f;
  return TestCOMEquilibrium(cps,gravity,numFCEdges,com,f);
}

bool ConstraintChecker::HasSupportPolygon_Robust(const Robot& robot,const Stance& stance,const Vector3& gravity,Real robustnessFactor,int numFCEdges) 
{
  vector<ContactPoint> cps(NumContactPoints(stance));
  int k=0;
  for(Stance::const_iterator i=stance.begin();i!=stance.end();i++) {
    const Hold& h=i->second;
    for(size_t j=0;j<h.contacts.size();j++,k++)
      cps[k]=h.contacts[j];
  }
  Vector3 com = robot.GetCOM();
  EquilibriumTester eq;
  eq.Setup(cps,gravity,numFCEdges,com);
  eq.SetRobustnessFactor(robustnessFactor);
  return eq.TestCurrent();
}


bool ConstraintChecker::HasEnvCollision(Robot& robot,Terrain& env)
{
  robot.UpdateGeometry();
  return robot.MeshCollision(*env.geometry);
}

bool ConstraintChecker::HasEnvCollision(Robot& robot,Terrain& env,const Stance& stance, const vector<int>& ignoreList)
{
	robot.UpdateGeometry();
	robot.InitMeshCollision(*env.geometry);
	vector<bool> fixed(robot.links.size(), false);
	for (Stance::const_iterator i = stance.begin(); i != stance.end(); i++)
		fixed[i->first] = true;

	for (int i = 0; i < ignoreList.size(); i++)
		fixed[ignoreList[i]] = true;

	for (size_t i = 0; i < robot.links.size(); i++) {
		if (!fixed[i]) {
			if (robot.MeshCollision(i)) {
				return true;
			}
		}
	}
	return false;
}

bool ConstraintChecker::HasEnvCollision(Robot& robot,Terrain& env,const vector<IKGoal>& constraints, const vector<int>& ignoreList)
{
	robot.UpdateGeometry();
	robot.InitMeshCollision(*env.geometry);
	vector<bool> fixed(robot.links.size(), false);
	for (size_t i=0;i<constraints.size();i++) {
	  if(constraints[i].destLink == -1)
	    fixed[constraints[i].link] = true;
	}

	for (int i = 0; i < ignoreList.size(); i++)
		fixed[ignoreList[i]] = true;

	for (size_t i = 0; i < robot.links.size(); i++) {
		if (!fixed[i]) {
			if (robot.MeshCollision(i)) {
			  LOG4CXX_INFO(KrisLibrary::logger(),"Collision between robot link "<<robot.linkNames[i].c_str());
				return true;
			}
		}
	}
	return false;
}


bool ConstraintChecker::HasEnvCollision(Robot& robot,Terrain& env,const vector<IKGoal>& constraints)
{
	robot.UpdateGeometry();
	robot.InitMeshCollision(*env.geometry);
	vector<bool> fixed(robot.links.size(), false);
	for ( int i = 0;i < constraints.size(); i++)
	{
		int linkIndex = constraints[i].link;
		fixed[ linkIndex] = true;
	}
	for (size_t i = 0; i < robot.links.size(); i++) {
		if (!fixed[i]) {
			LOG4CXX_INFO(KrisLibrary::logger(), "Checking link " << i << ":" << "\n");
			if (robot.MeshCollision(i)) {
				LOG4CXX_INFO(KrisLibrary::logger(), "  Link " << i << " is in collision!" << "\n");
				return true;
			}
			else{
				LOG4CXX_INFO(KrisLibrary::logger(), "  Link " << i << " is not in collision!" << "\n");
			}
		}
	}
	return false;
}

bool ConstraintChecker::HasSelfCollision(Robot& robot)
{
  robot.UpdateGeometry();
  return robot.SelfCollision();
}

bool ConstraintChecker::HasTorqueLimits(Robot& robot,const Stance& stance,const Vector3& gravity,int numFCEdges)
{
  ContactFormation contacts;
  ToContactFormation(stance,contacts);
  TorqueSolver solver(robot,contacts);
  solver.SetGravity(gravity);
  return solver.InTorqueBounds();
}
