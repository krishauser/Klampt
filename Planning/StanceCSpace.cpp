#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "StanceCSpace.h"
#include <boost/functional.hpp>

StanceCSpace::StanceCSpace(RobotWorld& world,int index,
			   WorldPlannerSettings* settings)
  :ContactCSpace(world,index,settings),gravity(0,0,-9.8),numFCEdges(4),
   spCalculated(false),spMargin(0),torqueSolver(robot,formation,numFCEdges)
{}

StanceCSpace::StanceCSpace(const SingleRobotCSpace& space)
  :ContactCSpace(space),gravity(0,0,-9.8),numFCEdges(4),
   spCalculated(false),spMargin(0),torqueSolver(robot,formation,numFCEdges)
{}

StanceCSpace::StanceCSpace(const StanceCSpace& space)
  :ContactCSpace(space),gravity(0,0,-9.8),numFCEdges(4),
  spCalculated(false),spMargin(space.spMargin),torqueSolver(robot,formation,numFCEdges)
{
  SetStance(space.stance);
}

void StanceCSpace::SetStance(const Stance& s)
{
  stance = s;
  spCalculated = false;
  torqueSolver.Clear();
  ToContactFormation(stance,formation);
  torqueSolver.contacts.set(formation, numFCEdges);

  contactIK.resize(0);
  for(Stance::const_iterator i=s.begin();i!=s.end();i++)
    contactIK.push_back(i->second.ikConstraint);

  AddConstraint("rigid_equilibrium",boost::bind1st(std::mem_fun(&StanceCSpace::CheckRBStability),this));
  AddConstraint("torque_balance",boost::bind1st(std::mem_fun(&StanceCSpace::CheckTorqueStability),this));
}

void StanceCSpace::SetHold(const Hold& h)
{
  stance[h.link]=h;
  spCalculated = false;
  torqueSolver.Clear();
  ToContactFormation(stance,formation);
  torqueSolver.contacts.set(formation, numFCEdges);

  contactIK.resize(0);
  for(Stance::const_iterator i=stance.begin();i!=stance.end();i++)
    contactIK.push_back(i->second.ikConstraint);

  AddConstraint("rigid_equilibrium",boost::bind1st(std::mem_fun(&StanceCSpace::CheckRBStability),this));
  AddConstraint("torque_balance",boost::bind1st(std::mem_fun(&StanceCSpace::CheckTorqueStability),this));
}

void StanceCSpace::CalculateSP()
{
  if(!spCalculated) {
    vector<ContactPoint> cps;
    GetContactPoints(stance,cps);
    sp.Set(cps,gravity,numFCEdges);
    spCalculated=true;
  }
}

void StanceCSpace::InitTorqueSolver()
{
  torqueSolver.SetGravity(gravity);
  torqueSolver.Init();
}

void StanceCSpace::SetSPMargin(Real margin)
{
  spMargin = margin;
  if(!spCalculated) CalculateSP();
}

bool StanceCSpace::CheckRBStability(const Config& x)
{
  if(spCalculated) return sp.TestCOM(robot.GetCOM());
  else {
    if(spMargin != 0) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Warning: spMargin is nonzero but the SP has not been calculated\n");
    }
    vector<ContactPoint> cps;
    vector<Vector3> f;
    GetContactPoints(stance,cps);
    return TestCOMEquilibrium(cps,gravity,numFCEdges,robot.GetCOM(),f);
  }
}

bool StanceCSpace::CheckTorqueStability(const Config& x)
{
  if(torqueSolver.active.empty() && torqueSolver.passive.empty()) {
    InitTorqueSolver();
  }
  return torqueSolver.InTorqueBounds();
}

