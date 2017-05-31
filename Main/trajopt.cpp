#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "Planning/ConstrainedInterpolator.h"
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math/differentiation.h>
#include <KrisLibrary/utils/AnyCollection.h>
#include "Planning/TimeScaling.h"
#include "Planning/RobotTimeScaling.h"
#include "Planning/RobotConstrainedInterpolator.h"
#include "Planning/ContactTimeScaling.h"
#include "Contact/Utils.h"
#include "Modeling/MultiPath.h"
#include <KrisLibrary/Timer.h>
#include <fstream>
using namespace std;
using namespace Math3D;

/** @brief Completely interpolates, optimizes, and time-scales the
 * given MultiPath to satisfy its contact constraints.
 *
 * Arguments
 * - robot: the robot
 * - path: the MultiPath containing the milestones / stances of the motion
 * - interpTol: the tolerance that must be met for contact constraints for the
 *   interpolated path.
 * - numdivs: the number of grid points used for time-scaling
 * - traj (out): the output timed trajectory. (Warning, the spaces and manifolds in
 *   traj.path.segments will be bogus pointers.)
 * - torqueRobustness: a parameter in [0,1] determining the robustness margin 
 *   added to the torque constraint.  The robot's torques are scaled by a factor
 *   of (1-torqueRobustness).
 * - frictionRobustness: a parameter in [0,1] determinining the robustness margin
 *   added to the friction constraint.  The friction coefficients are scaled by
 *   a factor of (1-frictionRobustness).  Helps the path avoid slipping.
 * - forceRobustness: a minimum normal contact force that must be applied
 *   *at each contact point* (in Newtons).  Helps the trajectory avoid contact
 *   separation.
 * - savePath: if true, saves the interpolated MultiPath to disk.
 * - saveConstraints: if true, saves the time scaling convex program constraints
 *   to disk.
 * 
 * Return value is true if interpolation / time scaling was successful.
 * Failure indicates that the milestones could not be interpolated, or 
 * the path is not dynamically feasible.  For example, the milestones or
 * interpolated path might violate stability constraints.
 */
bool ContactOptimizeMultipath(Robot& robot,const MultiPath& path,
			      Real interpTol,int numdivs,
			      TimeScaledBezierCurve& traj,
			      Real torqueRobustness=0.0,
			      Real frictionRobustness=0.0,
			      Real forceRobustness=0.0,
			      bool savePath = true,
			      bool saveConstraints = false)
{
  Assert(torqueRobustness < 1.0);
  Assert(frictionRobustness < 1.0);

  Timer timer;
  MultiPath ipath;
  bool res=DiscretizeConstrainedMultiPath(robot,path,ipath,interpTol);
  if(!res) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Could not discretize path, failing\n");
    return false;
  }

  int ns = 0;
  for(size_t i=0;i<ipath.sections.size();i++)
    ns += ipath.sections[i].milestones.size()-1;
  LOG4CXX_INFO(KrisLibrary::logger(),"Interpolated at resolution "<<interpTol<<" to "<<ns<<" segments, time "<<timer.ElapsedTime());
  if(savePath)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"Saving interpolated geometric path to temp.xml"<<"\n");
    ipath.Save("temp.xml");
  }

  vector<Real> divs(numdivs);
  Real T = ipath.Duration();
  for(size_t i=0;i<divs.size();i++)
    divs[i] = T*Real(i)/(divs.size()-1);

  timer.Reset();
  ContactTimeScaling scaling(robot);
  //CustomTimeScaling scaling(robot);

  scaling.frictionRobustness = frictionRobustness;
  scaling.torqueLimitScale = 1.0-torqueRobustness;
  scaling.forceRobustness = forceRobustness;
  res=scaling.SetParams(ipath,divs);

  /*
  scaling.SetPath(ipath,divs);
  scaling.SetDefaultBounds();
  scaling.SetStartStop();
  bool res=true;
  */
  if(!res) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Unable to set contact scaling, time "<<timer.ElapsedTime());
    if(saveConstraints) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Saving to scaling_constraints.csv\n");
      ofstream outc("scaling_constraints.csv",ios::out);
      outc<<"collocation point,planeindex,normal x,normal y,offset"<<endl;
      for(size_t i=0;i<scaling.ds2ddsConstraintNormals.size();i++) 
	for(size_t j=0;j<scaling.ds2ddsConstraintNormals[i].size();j++) 
	  outc<<i<<","<<j<<","<<scaling.ds2ddsConstraintNormals[i][j].x<<","<<scaling.ds2ddsConstraintNormals[i][j].y<<","<<scaling.ds2ddsConstraintOffsets[i][j]<<endl;
    }
    return false;
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"Contact scaling init successful, time "<<timer.ElapsedTime());
  if(saveConstraints) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Saving to scaling_constraints.csv\n");
    ofstream outc("scaling_constraints.csv",ios::out);
    outc<<"collocation point,planeindex,normal x,normal y,offset"<<endl;
    for(size_t i=0;i<scaling.ds2ddsConstraintNormals.size();i++) 
      for(size_t j=0;j<scaling.ds2ddsConstraintNormals[i].size();j++) 
	outc<<i<<","<<j<<","<<scaling.ds2ddsConstraintNormals[i][j].x<<","<<scaling.ds2ddsConstraintNormals[i][j].y<<","<<scaling.ds2ddsConstraintOffsets[i][j]<<endl;
  }

  res=scaling.Optimize();
  if(!res) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Time scaling failed in time "<<timer.ElapsedTime());
    return false;
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"Time scaling solved in time "<<timer.ElapsedTime()<<", execution time "<<scaling.traj.timeScaling.times.back());
  //scaling.Check(ipath);

  traj = scaling.traj;
  return true;
}

class ContactOptimizeSettings : public AnyCollection
{
public:
  ContactOptimizeSettings() {
    (*this)["numdivs"]=101;
    (*this)["xtol"]=0.05;
    (*this)["ignoreForces"]=false;
    (*this)["torqueRobustness"]=0.0;
    (*this)["frictionRobustness"]=0;
    //(*this)["frictionRobustness"]=0.25;
    (*this)["forceRobustness"]=0.5;
    //(*this)["forceRobustness"]=0;
    //(*this)["forceRobustness"]=5;
    (*this)["outputPath"] = string("trajopt.path");
    (*this)["outputDt"] = 0.1;
  }
  bool read(const char* fn) {
    ifstream in(fn,ios::in);
    if(!in) return false;
    AnyCollection newEntries;
    if(!newEntries.read(in)) return false;
    merge(newEntries);
    return true;
  }
};

void ContactOptimizeMultipath(const char* robfile,const char* pathfile,const char* settingsfile = NULL)
{
  ContactOptimizeSettings settings;
  if(settingsfile != NULL) {
    if(!settings.read(settingsfile)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Unable to read settings file "<<settingsfile);
      return;
    }
  }
  Real xtol=Real(settings["xtol"]);
  int numdivs = int(settings["numdivs"]);
  bool ignoreForces = bool(settings["ignoreForces"]);
  Real torqueRobustness = Real(settings["torqueRobustness"]);
  Real frictionRobustness = Real(settings["frictionRobustness"]);
  Real forceRobustness = Real(settings["forceRobustness"]);
  string outputPath;
  settings["outputPath"].as(outputPath);
  Real outputDt = Real(settings["outputDt"]);

  Robot robot;
  if(!robot.Load(robfile)) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Unable to load robot file "<<robfile);
    return;
  }

  MultiPath path;
  if(!path.Load(pathfile)) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Unable to load path file "<<pathfile);
    return;
  }

  //double check friction
  for(size_t i=0;i<path.sections.size();i++) {
    Stance s;
    path.GetStance(s,i);
    bool changed = false;
    int k=0;
    int numContacts = 0;
    for(Stance::iterator h=s.begin();h!=s.end();h++,k++) {
      numContacts += (int)h->second.contacts.size();
      for(size_t j=0;j<h->second.contacts.size();j++)
	if(h->second.contacts[j].kFriction <= 0) {
	  if(!changed)
	    LOG4CXX_WARN(KrisLibrary::logger(),"Warning, contact "<<j <<" of hold "<<k<<" has invalid friction "<<h->second.contacts[j].kFriction<<", setting to 0.5\n");
    h->second.contacts[j].kFriction = 0.5;
	  changed = true;
	}
    }
    path.SetStance(s,i);
    if(numContacts == 0 && !ignoreForces && robot.joints[0].type == RobotJoint::Floating) {
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning, no contacts given in stance "<<i);
      LOG4CXX_INFO(KrisLibrary::logger(),"Should set ignoreForces = true in trajopt.settings if you wish\n");
      LOG4CXX_INFO(KrisLibrary::logger(),"to ignore contact force constraints.\n");
      LOG4CXX_INFO(KrisLibrary::logger(),"Press enter to continue...\n");
      if(KrisLibrary::logger()->isEnabledFor(log4cxx::Level::ERROR_INT)) getchar();
    }
  }

  TimeScaledBezierCurve opttraj;
  if(ignoreForces) {
    bool res=GenerateAndTimeOptimizeMultiPath(robot,path,xtol,outputDt);
    if(!res) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Time optimization failed\n");
      return;
    }
    Assert(path.HasTiming());
    LOG4CXX_INFO(KrisLibrary::logger(),"Saving dynamically optimized path to "<<outputPath<<"\n");
    ofstream out(outputPath.c_str(),ios::out);
    for(size_t i=0;i<path.sections.size();i++) {
      for(size_t j=0;j<path.sections[i].milestones.size();j++) 
	out<<path.sections[i].times[j]<<"\t"<<path.sections[i].milestones[j]<<endl;
    }
    out.close();
  }
  else {
    
    bool res=ContactOptimizeMultipath(robot,path,xtol,
				 numdivs,opttraj,
				 torqueRobustness,
				 frictionRobustness,
				 forceRobustness);
    if(!res) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Time optimization failed\n");
      return;
    }
    RobotCSpace cspace(robot);
    for(size_t i=0;i<opttraj.path.segments.size();i++) {
      opttraj.path.segments[i].space = &cspace;
      opttraj.path.segments[i].manifold = &cspace;
    }
    vector<Config> milestones;
    opttraj.GetDiscretizedPath(outputDt,milestones);
    LOG4CXX_INFO(KrisLibrary::logger(),"Saving dynamically optimized path to "<<outputPath<<"\n");
    ofstream out(outputPath.c_str(),ios::out);
    for(size_t i=0;i<milestones.size();i++) {
      out<<i*outputDt<<"\t"<<milestones[i]<<endl;
    }
    out.close();

    LOG4CXX_INFO(KrisLibrary::logger(),"Plotting vel/acc constraints to trajopt_plot.csv...\n");
    opttraj.Plot("trajopt_plot.csv",robot.velMin,robot.velMax,-1.0*robot.accMax,robot.accMax);
  }
}


int main(int argc, char** argv)
{
  Robot::disableGeometryLoading = true;
  if(argc < 3) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Usage: TrajOpt robot multipath [settings]\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"Saving settings template to trajopt_default.settings...\n");
    ContactOptimizeSettings settings;
    ofstream out("trajopt_default.settings",ios::out);
    out<<settings<<endl;
    return 1;
  }
  const char* settingsFile = NULL;
  if(argc >= 4)
    settingsFile = argv[3];
  ContactOptimizeMultipath(argv[1],argv[2],settingsFile);
  return 0;
}
