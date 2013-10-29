#include "StanceCSpace.h"

StanceCSpace::StanceCSpace(RobotWorld& world,int index,
			   WorldPlannerSettings* settings)
  :ContactCSpace(world,index,settings),gravity(0,0,-9.8),numFCEdges(4),
   spCalculated(false),spMargin(0),torqueSolver(*GetRobot(),formation)
{}

StanceCSpace::StanceCSpace(const SingleRobotCSpace& space)
  :ContactCSpace(space),gravity(0,0,-9.8),numFCEdges(4),
   spCalculated(false),spMargin(0),torqueSolver(*GetRobot(),formation)
{}

StanceCSpace::StanceCSpace(const StanceCSpace& space)
  :ContactCSpace(space),gravity(0,0,-9.8),numFCEdges(4),
  spCalculated(false),spMargin(space.spMargin),torqueSolver(*GetRobot(),formation)
{
  SetStance(space.stance);
}

void StanceCSpace::Sample(Config& x)
{
  ContactCSpace::Sample(x);
}

void StanceCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  ContactCSpace::SampleNeighborhood(c,r,x);
}

bool StanceCSpace::IsFeasible(const Config& q)
{
  //heuristic testing order
  if(!CheckJointLimits(q)) return false;
  if(!CheckContact()) return false;
  if(!CheckRBStability()) return false;
  if(!CheckCollisionFree()) return false;
  if(!CheckTorqueStability()) return false;
  return true;
}

void StanceCSpace::SetStance(const Stance& s)
{
  stance = s;
  spCalculated = false;
  torqueSolver.Clear();
  ToContactFormation(stance,formation);

  contactIK.resize(0);
  for(Stance::const_iterator i=s.begin();i!=s.end();i++)
    contactIK.push_back(i->second.ikConstraint);
}

void StanceCSpace::SetHold(const Hold& h)
{
  stance[h.link]=h;
  spCalculated = false;
  torqueSolver.Clear();
  ToContactFormation(stance,formation);

  contactIK.resize(0);
  for(Stance::const_iterator i=stance.begin();i!=stance.end();i++)
    contactIK.push_back(i->second.ikConstraint);
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
  torqueSolver.Init(numFCEdges);
}

void StanceCSpace::SetSPMargin(Real margin)
{
  spMargin = margin;
}

bool StanceCSpace::CheckRBStability()
{
  if(spCalculated) return sp.TestCOM(GetRobot()->GetCOM());
  else {
    if(spMargin != 0) {
      fprintf(stderr,"Warning: spMargin is nonzero but the SP has not been calculated\n");
    }
    vector<ContactPoint> cps;
    vector<Vector3> f;
    GetContactPoints(stance,cps);
    return TestCOMEquilibrium(cps,gravity,numFCEdges,GetRobot()->GetCOM(),f);
  }
}

bool StanceCSpace::CheckTorqueStability()
{
  if(torqueSolver.active.empty() && torqueSolver.passive.empty()) {
    InitTorqueSolver();
  }
  return torqueSolver.InTorqueBounds();
}

