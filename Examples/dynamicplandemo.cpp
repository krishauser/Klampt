#include "Planning/RobotCSpace.h"
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include "IO/XmlWorld.h"
#include "Modeling/MultiPath.h"
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/utils/stringutils.h>
#include <string.h>
#include <time.h>
#include <fstream>

//includes for dynamic path checking
#include "Modeling/DynamicPath.h"
//includes CSpaceFeasibilityChecker
#include "Planning/RampCSpace.h"
//includes Convert to convert between path representations
#include "Modeling/Paths.h"

/** @brief Converts a kinematically planned path to a dynamic one with
 * velocity and acceleration bounds.
 * 
 * Performs at most maxIters iterations of shortcutting.
 *
 * The output is placed in dynamicPath.
 */
void DynamicShortcut(RobotWorld& world,int robot,const MilestonePath& path,int maxIters,
		     ParabolicRamp::DynamicPath& dynamicPath)
{
  //1. Make initial start-and-stop path.
  //set up joint/velocity/acceleration limits
  dynamicPath.xMin = world.robots[robot]->qMin;
  dynamicPath.xMax = world.robots[robot]->qMax;
  dynamicPath.velMax = world.robots[robot]->velMax;
  dynamicPath.accMax = world.robots[robot]->accMax;
  //extract milestones from MilestonePath
  vector<ParabolicRamp::Vector> milestones(path.NumMilestones());
  for(size_t i=0;i<milestones.size();i++)
    milestones[i] = path.GetMilestone(i);
  dynamicPath.SetMilestones(milestones);
  printf("Initial path duration: %g seconds\n",dynamicPath.GetTotalTime());

  //2. Set up the collision checker
  WorldPlannerSettings settings;
  settings.InitializeDefault(world);
  SingleRobotCSpace cspace(world,robot,&settings); 
  CSpaceFeasibilityChecker feasibilityChecker(&cspace);
  Real tolerance = settings.robotSettings[robot].collisionEpsilon;
  ParabolicRamp::RampFeasibilityChecker checker(&feasibilityChecker,tolerance);

  //3. Perform shortcutting
  int numShortcuts = dynamicPath.Shortcut(maxIters,checker);
  printf("%d dynamic shortcuts performed\n",numShortcuts);
  printf("Optimized path duration: %g seconds\n",dynamicPath.GetTotalTime());
}

/** @brief Performs basic path planning in collision-free space for the
 * given robot and start/end points.
 * 
 * See plandemo.cpp
 */
bool SimplePlan(RobotWorld& world,int robot,const Config& qstart,const Config& qgoal,MilestonePath& path,
		const HaltingCondition& cond,const string& plannerSettings="")
{
  ///If you don't call this, everything will run fine due to on-demand
  ///collision initialization, but at least here you get some debug information
  ///if your world is really complex and the collision detection structures
  ///take a long time to initialize.
  world.InitCollisions();

  WorldPlannerSettings settings;
  settings.InitializeDefault(world);
  SingleRobotCSpace cspace(world,robot,&settings); 

  if(!cspace.IsFeasible(qstart)) {
    cout<<"Start configuration is infeasible, violated constraints:"<<endl;
    cspace.PrintInfeasibleNames(qstart);
    return false;
  }
  if(!cspace.IsFeasible(qgoal)) {
    cout<<"Goal configuration is infeasible, violated constraints:"<<endl;
    cspace.PrintInfeasibleNames(qgoal);
    return false;
  }

  MotionPlannerFactory factory;
  if(!plannerSettings.empty()) {
    bool res = factory.LoadJSON(plannerSettings);
    if(!res) 
      printf("Warning, incorrectly formatted planner settings file\n");
  }

  MotionPlannerInterface* planner = factory.Create(&cspace,qstart,qgoal);
  string res = planner->Plan(path,cond);
  cout<<"Planner terminated with condition "<<res<<endl;
  delete planner;
  return !path.edges.empty();
}

int main(int argc,const char** argv)
{
  if(argc <= 2) {
    printf("USAGE: DynamicPlanDemo [options] world_file configs\n");
    printf("OPTIONS:\n");
    printf("-o filename: the output linear path or multipath (default dynamicplandemo.xml)\n");
    printf("-h timestep: resolution for linear path output (default 0.01)\n");
    printf("-p settings: set the planner configuration file\n");
    printf("-opt: do optimal planning (do not terminate on the first found solution)\n");
    printf("-n iters: set the default number of iterations (default 1000)\n");
    printf("-t time: set the planning time limit (default infinity)\n");
    printf("-s iters: set the number of shortcuts (default 100)\n");
    printf("-r robotindex: set the robot index (default 0)\n");
    return 0;
  }
  Srand(time(NULL));
  int robot = 0;
  const char* outputfile = "dynamicplandemo.xml";
  HaltingCondition termCond;
  string plannerSettings;
  int numShortcutIters = 100;
  Real discretizeTimeStep = 0.01;
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
	  printf("Unable to load planner settings file %s\n",argv[i+1]);
	  return 1;
	}
	i++;
      }
      else if(0==strcmp(argv[i],"-s")) {
	numShortcutIters = atoi(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-h")) {
	discretizeTimeStep = atof(argv[i+1]);
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
    printf("USAGE: DynamicPlanDemo [options] world_file configs\n");
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
  path.settings["robot"] = world.robots[robot]->name;
  path.settings["command"] = cmdline;
  //begin planning
  bool feasible = true;
  for(size_t i=0;i+1<configs.size();i++) {
    MilestonePath mpath;
    if(!SimplePlan(world,robot,configs[i],configs[i+1],mpath,termCond,plannerSettings)) {
      printf("Planning from configuration %d to %d failed\n",i,i+1);
      path.sections.resize(path.sections.size()+1);
      path.sections.back().settings["infeasible"]=1;
      path.sections.back().milestones.resize(2);
      path.sections.back().milestones[0] = configs[i];
      path.sections.back().milestones[1] = configs[i+1];
      feasible = false;
    }
    else {
      ParabolicRamp::DynamicPath dpath;
      DynamicShortcut(world,robot,mpath,numShortcutIters,dpath);
      MultiPath dpathSection;
      Convert(dpath,dpathSection);
      path.Concat(dpathSection);
    }
  }
  if(feasible)
    printf("Path planning success! Saving to %s\n",outputfile);
  else
    printf("Path planning failure. Saving placeholder path to %s\n",outputfile);
  const char* ext = FileExtension(outputfile);
  if(ext && 0==strcmp(ext,"path")) {
    printf("Converted to linear path format at resolution %g\n",discretizeTimeStep);
    LinearPath lpath;
    Discretize(path,discretizeTimeStep,lpath.times,lpath.milestones);
    ofstream f(outputfile,ios::out);
    lpath.Save(f);
    f.close();
  }
  else 
    path.Save(outputfile);
  return 0;
}
