#include "Planning/RobotCSpace.h"
#include <planning/AnyMotionPlanner.h>
#include "Modeling/MultiPath.h"
#include "IO/XmlWorld.h"
#include <string.h>
#include <fstream>

/** @brief Performs basic path planning in collision-free space for the
 * given robot and start/end points.
 * The output is given in path.  At most maxIters iterations are spent
 * planning.
 *
 * The constraint specifications are given in WorldPlannerSettings. If you
 * have custom requirements, you will need to 
 */
bool SimplePlan(RobotWorld& world,int robot,const Config& qstart,const Config& qgoal,MilestonePath& path,int maxIters=1000)
{
  WorldPlannerSettings settings;
  settings.InitializeDefault(world);
  //do more constraint setup here (modifying the settings object) if desired,
  //e.g., set collision margins, edge collision checking resolution, etc.
  SingleRobotCSpace cspace(world,robot,&settings); 
  MotionPlannerFactory factory;
  //do more planner setup here if desired, e.g., change perturbation size,
  //connection radius, etc
  MotionPlannerInterface* planner = factory.Create(&cspace);
  int istart=planner->AddMilestone(qstart); //should be 0
  int igoal=planner->AddMilestone(qgoal); //should be 1
  for(int i=0;i<maxIters;i++) {
    planner->PlanMore();
    if(planner->IsConnected(0,1)) {
      planner->GetPath(0,1,path);
      delete planner;
      return true;
    }
  }
  delete planner;
  return false;
}

int main(int argc,const char** argv)
{
  if(argc <= 2) {
    printf("USAGE: PlanDemo [options] world_file configs\n");
    printf("OPTIONS:\n");
    printf("-o filename: the output linear path or multipath (default plandemo.xml)\n");
    printf("-n iters: set the default number of iterations (default 1000)\n");
    printf("-r robotindex: set the robot index (default 0)\n");
    return 0;
  }
  int robot = 0;
  const char* outputfile = "plandemo.xml";
  int maxIters = 1000;
  int i;
  //parse command line arguments
  for(i=1;i<argc;i++) {
    if(argv[i][0]=='-') {
      if(0==strcmp(argv[i],"-n")) {
	maxIters = atoi(argv[i+1]);
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
	printf("Invalid option %s\n",argv[i]);
	return 1;
      }
    }
    else break;
  }
  if(i+2 < argc) {
    printf("Too few arguments provided\n");
    printf("USAGE: PlanDemo [options] world_file configs\n");
    return 1;
  }
  if(i+2 > argc) {
    printf("Warning: extra arguments provided\n");
  }
  const char* worldfile = argv[i];
  const char* configsfile = argv[i+1];

  //Read in the world file
  XmlWorld xmlWorld;
  RobotWorld world;
  if(!xmlWorld.Load(worldfile)) {
    printf("Error loading world XML file %s\n",worldfile);
    return 1;
  }
  if(!xmlWorld.GetWorld(world)) {
    printf("Error loading world file %s\n",worldfile);
    return 1;
  }

  //Read in the configurations specified in configsfile
  vector<Config> configs;
  ifstream in(configsfile);
  if(!in) {
    printf("Error opening configs file %s\n",configsfile);
    return false;
  }
  while(in) {
    Config temp;
    in >> temp;
    if(in) configs.push_back(temp);
  }
  if(configs.size() < 2) {
    printf("Configs file does not contain 2 or more configs\n");
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
  path.settings["robot"] = world.robots[robot].name;
  path.settings["command"] = cmdline;
  //begin planning
  bool feasible = true;
  for(size_t i=0;i+1<configs.size();i++) {
    MilestonePath mpath;
    if(!SimplePlan(world,robot,configs[i],configs[i+1],mpath,maxIters)) {
      printf("Planning from configuration %d to %d failed\n",i,i+1);
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
  if(feasible)
    printf("Path planning success! Saving to %s\n",outputfile);
  else
    printf("Path planning failure. Saving placeholder path to %s\n",outputfile);
  path.Save(outputfile);
  return 0;
}
