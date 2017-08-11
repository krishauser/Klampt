//these two include files are needed for SimplePlan
#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "Planning/RobotCSpace.h"
  //defines WorldPlannerSettings and SingleRobotCSpace
  //includes definitions for RobotWorld, Config
#include <KrisLibrary/planning/AnyMotionPlanner.h>
  //defines AdaptiveCSpace, which helps debugging and
  //can reorder constraints for faster performance
#include <KrisLibrary/planning/CSpaceHelpers.h>
  //defines MotionPlannerFactory
  //includes MilestonePath, HaltingCondition

//the following include files are used for IO and command line processing
#include "IO/XmlWorld.h"
  //defines XmlWorld for loading RobotWorlds from .xml files
#include "Modeling/Paths.h"
#include "Modeling/MultiPath.h"
  //defines file paths and conversion routines
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/utils/stringutils.h>
  //for easy timing 
#include <KrisLibrary/Timer.h>
#include <string.h>
#include <time.h>
#include <fstream>

/** @brief Performs basic path planning in collision-free space for the
 * given robot and start/end points.
 * 
 * The output is given in path.  At most cond.maxIters iterations and at
 * most cond.timeLimit seconds are spent planning.
 *
 * The constraint specifications are given in WorldPlannerSettings. If you
 * have custom requirements, you will need to set them up.
 */
bool SimplePlan(RobotWorld& world,int robot,const Config& qstart,const Config& qgoal,MilestonePath& path,
		const HaltingCondition& cond,const string& plannerSettings="")
{
  ///If you don't call this, everything will run fine due to on-demand
  ///collision initialization, but at least here you get some debug information
  ///if your world is really complex and the collision detection structures
  ///take a long time to initialize.
  world.InitCollisions();

  //1. Create and initialize a WorldPlannerSettings object for the given
  //world. 
  WorldPlannerSettings settings;
  settings.InitializeDefault(world);
  //Here you can modify the setting object to do more constraint setup
  //if desired, e.g., which collisions are tested, edge collision checking
  //resolution, etc.  See the Planning/PlannerSettings.h file for more
  //information.
  //
  //If you wish to add extra collision avoidance margins, you may do so by
  //adding margins onto the geometries in the world. For example, to avoid
  //collisions by 0.05 units, you can use the following code:
  //  Real margin = 0.05
  //  for(size_t i=0;i<world.robots[robot]->geometry.size();i++)
  //    world.robots[robot]->geometry[i].margin += margin;
  //(make sure to restore the old margins if you want to perform multiple
  // planning runs)

  //2. Create a SingleRobotCSpace object for the indicated robot.
  //
  //If you want to plan only for certain degrees-of-freedom, you can
  //use the SingleRobotCSpace2 class.  See its documentation in
  //Planning/RobotCSpace.h
  //
  //If you wish to add custom kinematic constraints, you can use the
  //CSpace.AddConstraint method.  See its documentation in KrisLibrary/planning/CSpace.h
  SingleRobotCSpace sspace(world,robot,&settings); 
  //The AdaptiveCSpace class profiles the CSpace's constraints and will help get
  //better stats for debugging.
  AdaptiveCSpace cspace(&sspace);
  cspace.SetupAdaptiveInfo();

  //3. Some sanity checks -- make sure the start and goal configurations
  //are feasible.
  if(!cspace.IsFeasible(qstart)) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Start configuration is infeasible, violated constraints:"<<"\n");
    cspace.PrintInfeasibleNames(qstart);
    return false;
  }
  if(!cspace.IsFeasible(qgoal)) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Goal configuration is infeasible, violated constraints:"<<"\n");
    cspace.PrintInfeasibleNames(qstart);
    return false;
  }

  //4. Set up the motion planner settings from a given planner settings string
  //(see Examples/PlanDemo/*.settings)
  MotionPlannerFactory factory;
  if(!plannerSettings.empty()) {
    bool res = factory.LoadJSON(plannerSettings);
    if(!res) 
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning, incorrectly formatted planner settings file\n");
  }
  //You may also manually do more planner setup here if desired, e.g.,
  //change planner type, perturbation size, connection radius, etc.
  //See KrisLibrary/planning/AnyMotionPlanner.h

  //5. Create the planner and run until the termination criterion stops it
  MotionPlannerInterface* planner = factory.Create(&cspace,qstart,qgoal);
  Timer timer;
  string res = planner->Plan(path,cond);
  //print some debugging information
  LOG4CXX_INFO(KrisLibrary::logger(),"Planner terminated with condition "<<res<<" after "<<planner->NumIterations()<<" iters and time "<<timer.ElapsedTime()<<"s"<<"\n");
  if(!path.edges.empty())
    LOG4CXX_INFO(KrisLibrary::logger(),"Solution path length: "<<path.Length()<<"\n");
  PropertyMap stats;
  planner->GetStats(stats);
  LOG4CXX_INFO(KrisLibrary::logger(),"Planner stats: ");
  stats.Print(cout);
  LOG4CXX_INFO(KrisLibrary::logger(),"\n");
  PropertyMap sstats;
  cspace.GetStats(sstats);
  LOG4CXX_INFO(KrisLibrary::logger(),"Space stats: ");
  sstats.Print(cout);
  LOG4CXX_INFO(KrisLibrary::logger(),"\n");
  delete planner;
  //return true if a solution was found
  return !path.edges.empty();
}

int main(int argc,const char** argv)
{
  if(argc <= 2) {
    LOG4CXX_INFO(KrisLibrary::logger(),"USAGE: PlanDemo [options] world_file configs\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"OPTIONS:\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"-o filename: the output linear path or multipath (default plandemo.xml)\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"-p settings: set the planner configuration file\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"-opt: do optimal planning (do not terminate on the first found solution)\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"-n iters: set the default number of iterations (default 1000)\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"-t time: set the planning time limit (default infinity)\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"-r robotindex: set the robot index (default 0)\n");
    return 0;
  }
  Srand(time(NULL));
  int robot = 0;
  const char* outputfile = "plandemo.xml";
  HaltingCondition termCond;
  string plannerSettings;
  int i;
  //parse command line arguments
  for(i=1;i<argc;i++) {
    if(argv[i][0]=='-') {
      if(0==strcmp(argv[i],"-n")) {
	termCond.maxIters = atoi(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-t")) {
	termCond.timeLimit = atof(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-opt")) {
	termCond.foundSolution = false;
      }
      else if(0==strcmp(argv[i],"-p")) {
	if(!GetFileContents(argv[i+1],plannerSettings)) {
	  LOG4CXX_INFO(KrisLibrary::logger(),"Unable to load planner settings file "<<argv[i+1]);
	  return 1;
	}
	i++;
      }
      else if(0==strcmp(argv[i],"-r")) {
	robot = atoi(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-o")) {
	outputfile = argv[i+1];
	i++;
      }
      else {
	LOG4CXX_INFO(KrisLibrary::logger(),"Invalid option "<<argv[i]);
	return 1;
      }
    }
    else break;
  }
  if(i+2 < argc) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Too few arguments provided\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"USAGE: PlanDemo [options] world_file configs\n");
    return 1;
  }
  if(i+2 > argc) {
    LOG4CXX_WARN(KrisLibrary::logger(),"Warning: extra arguments provided\n");
  }
  const char* worldfile = argv[i];
  const char* configsfile = argv[i+1];

  //Read in the world file
  XmlWorld xmlWorld;
  RobotWorld world;
  if(!xmlWorld.Load(worldfile)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error loading world XML file "<<worldfile);
    return 1;
  }
  if(!xmlWorld.GetWorld(world)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error loading world file "<<worldfile);
    return 1;
  }

  //Read in the configurations specified in configsfile
  vector<Config> configs;
  ifstream in(configsfile);
  if(!in) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error opening configs file "<<configsfile);
    return false;
  }
  while(in) {
    Config temp;
    in >> temp;
    if(in) configs.push_back(temp);
  }
  if(configs.size() < 2) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Configs file does not contain 2 or more configs\n");
    return 1;
  }

  //set up the command line, store it into the MultiPath
  string cmdline;
  cmdline = argv[0];
  for(int i=1;i<argc;i++) {
    cmdline += " ";
    cmdline += argv[i];
  }
  MultiPath path;
  path.settings["robot"] = world.robots[robot]->name;
  path.settings["command"] = cmdline;
  //begin planning
  bool feasible = true;
  for(size_t i=0;i+1<configs.size();i++) {
    MilestonePath mpath;
    if(!SimplePlan(world,robot,configs[i],configs[i+1],mpath,termCond,plannerSettings)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Planning from configuration "<<i<<" to "<<i+1);
      path.sections.resize(path.sections.size()+1);
      path.sections.back().settings["infeasible"]=1;
      path.sections.back().milestones.resize(2);
      path.sections.back().milestones[0] = configs[i];
      path.sections.back().milestones[1] = configs[i+1];
      feasible = false;
    }
    else {
      path.sections.resize(path.sections.size()+1);
      path.sections.back().milestones.resize(mpath.NumMilestones());
      for(int j=0;j<mpath.NumMilestones();j++)
	path.sections.back().milestones[j] = mpath.GetMilestone(j);
    }
  }
  if(feasible){
    LOG4CXX_INFO(KrisLibrary::logger(),"Path planning success! Saving to "<<outputfile);
  }
  else{
    LOG4CXX_INFO(KrisLibrary::logger(),"Path planning failure. Saving placeholder path to "<<outputfile);
  }
  const char* ext = FileExtension(outputfile);
  if(ext && 0==strcmp(ext,"path")) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Converted to linear path format\n");
    LinearPath lpath;
    Convert(path,lpath);
    ofstream f(outputfile,ios::out);
    lpath.Save(f);
    f.close();
  }
  else 
    path.Save(outputfile);
  return 0;
}
