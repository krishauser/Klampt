#include "StanceCSpace.h"
using namespace Klampt;

StanceCSpace::StanceCSpace(WorldModel& world,int index,
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

  AddConstraint("rigid_equilibrium",std::bind(std::mem_fun(&StanceCSpace::CheckRBStability),this,std::placeholders::_1));
  AddConstraint("torque_balance",std::bind(std::mem_fun(&StanceCSpace::CheckTorqueStability),this,std::placeholders::_1));
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

  AddConstraint("rigid_equilibrium",std::bind(std::mem_fun(&StanceCSpace::CheckRBStability),this,std::placeholders::_1));
  AddConstraint("torque_balance",std::bind(std::mem_fun(&StanceCSpace::CheckTorqueStability),this,std::placeholders::_1));
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
      fprintf(stderr,"Warning: spMargin is nonzero but the SP has not been calculated\n");
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

