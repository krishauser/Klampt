#include "Planning/StanceCSpace.h"
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/utils/stringutils.h>
#include "Modeling/Paths.h"
#include "Modeling/MultiPath.h"
#include "IO/XmlWorld.h"
#include <time.h>
#include <KrisLibrary/Timer.h>
#include <string.h>
#include <fstream>

///A margin used in StancePlan on the distance between the COM and the
///edge of the support polygon
Real gSupportPolygonMargin = 0;


/** @brief Performs path planning in collision-free space for the
 * given robot at the stance s between start configuration qstart and
 * destination qgoal.
 * Stability is tested at each configuration.
 *
 * The output is given in path.  cond controls the number of iterations/time
 * for planning.
 *
 * The constraint specifications are given in WorldPlannerSettings. If you
 * have custom requirements, you will need to set them up.
 */
bool StancePlan(RobotWorld& world,int robot,Config& qstart,Config& qgoal,const Stance& s,MilestonePath& path,
		 const HaltingCondition& cond,const string& plannerSettings="")
{
  WorldPlannerSettings settings;
  settings.InitializeDefault(world);
  //do more constraint setup here (modifying the settings object) if desired,
  //e.g., set collision margins, edge collision checking resolution, etc.
  StanceCSpace cspace(world,robot,&settings); 
  cspace.SetStance(s);
  cspace.CalculateSP();
  if(gSupportPolygonMargin != 0)
    cspace.SetSPMargin(gSupportPolygonMargin);
  //sanity checking
  if(!cspace.CheckContact(qstart)) {
    printf("Start configuration does not meet contact constraint, error %g\n",cspace.ContactDistance(qstart));
    if(!cspace.SolveContact()) {
      printf("  Solving for contact failed.\n");
      return false;
    }
    else {
      printf("  IK problem was solved, using new start configuration\n");
      qstart = cspace.robot.q;
    }
  }
  if(!cspace.CheckContact(qgoal)) {
    printf("Goal configuration does not meet contact constraint, error %g\n",cspace.ContactDistance(qgoal));
    if(!cspace.SolveContact()) {
      printf("  Solving for contact failed.\n");
      return false;
    }
    else {
      printf("  IK problem was solved, using new goal configuration\n");
      qgoal = cspace.robot.q;
    }
  }
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
  if(!plannerSettings.empty())
    factory.LoadJSON(plannerSettings);
  //do more planner setup here if desired, e.g., change planner type,
  //perturbation size, connection radius, etc

  //make the planner and do the planning
  MotionPlannerInterface* planner = factory.Create(&cspace,qstart,qgoal);
  string res = planner->Plan(path,cond);
  cout<<"Planner terminated with condition "<<res<<endl;
  printf("  Stats: SolveContact %d, %gs.\n",cspace.numSolveContact,cspace.solveContactTime);
  delete planner;
  return !path.edges.empty();
}


/** @brief Performs path planning in collision-free space for the
 * given robot at the stance s between start configuration qstart and
 * destination qgoal
 * Stability is *not* tested.  If you wish to maintain stability use
 * StancePlan
 *
 * The output is given in path.  cond controls the number of iterations/time
 * for planning.
 *
 * The constraint specifications are given in WorldPlannerSettings. If you
 * have custom requirements, you will need to set them up.
 */
bool ContactPlan(RobotWorld& world,int robot,Config& qstart,Config& qgoal,const Stance& s,MilestonePath& path,
		 const HaltingCondition& cond,const string& plannerSettings="")
{
  WorldPlannerSettings settings;
  settings.InitializeDefault(world);
  //do more constraint setup here (modifying the settings object) if desired,
  //e.g., set collision margins, edge collision checking resolution, etc.
  ContactCSpace cspace(world,robot,&settings); 
  for(Stance::const_iterator i=s.begin();i!=s.end();i++)
    cspace.AddContact(i->second.ikConstraint);
  //sanity checking
  if(!cspace.CheckContact(qstart)) {
    printf("Start configuration does not meet contact constraint, error %g\n",cspace.ContactDistance(qstart));
    if(!cspace.SolveContact()) {
      printf("  Solving for contact failed.\n");
      return false;
    }
    else {
      printf("  IK problem was solved, using new start configuration\n");
      qstart = cspace.robot.q;
    }
  }
  if(!cspace.CheckContact(qgoal)) {
    printf("Goal configuration does not meet contact constraint, error %g\n",cspace.ContactDistance(qgoal));
    if(!cspace.SolveContact()) {
      printf("  Solving for contact failed.\n");
      return false;
    }
    else {
      printf("  IK problem was solved, using new goal configuration\n");
      qgoal = cspace.robot.q;
    }
  }
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
  if(!plannerSettings.empty())
    factory.LoadJSON(plannerSettings);
  //do more planner setup here if desired, e.g., change planner type,
  //perturbation size, connection radius, etc

  //make the planner and do the planning
  Timer timer;
  MotionPlannerInterface* planner = factory.Create(&cspace,qstart,qgoal);
  cout<<"  Create time: "<<timer.ElapsedTime()<<endl;
  string res = planner->Plan(path,cond);
  cout<<"Planner terminated with condition "<<res<<endl;
  printf("  Stats: SolveContact %d, %gs\n",cspace.numSolveContact,cspace.solveContactTime);
  delete planner;
  return !path.edges.empty();
}


int main(int argc,const char** argv)
{
  if(argc <= 3) {
    printf("USAGE: ContactPlan [options] world_file configs stance\n");
    printf("OPTIONS:\n");
    printf("-o filename: the output linear path or multipath (default contactplan.xml)\n");
    printf("-p settings: set the planner configuration file\n");
    printf("-opt: do optimal planning (do not terminate on the first found solution)\n");
    printf("-n iters: set the default number of iterations per step (default 1000)\n");
    printf("-t time: set the planning time limit (default infinity)\n");
    printf("-m margin: set support polygon margin (default 0)\n");
    printf("-r robotindex: set the robot index (default 0)\n");
    return 0;
  }
  Srand(time(NULL));
  int robot = 0;
  const char* outputfile = "contactplan.xml";
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
	  printf("Unable to load planner settings file %s\n",argv[i+1]);
	  return 1;
	}
	i++;
      }
      else if(0==strcmp(argv[i],"-r")) {
	robot = atoi(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-m")) {
	gSupportPolygonMargin = atof(argv[i+1]);
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
  if(i+3 < argc) {
    printf("USAGE: ContactPlan [options] world_file configs stance\n");
    return 1;
  }
  if(i+3 > argc) {
    printf("Warning: extra arguments provided\n");
  }
  const char* worldfile = argv[i];
  const char* configsfile = argv[i+1];
  const char* stancefile = argv[i+2];

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

  vector<Config> configs;
  {
    //Read in the configurations specified in configsfile
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
    in.close();
  }

  Stance stance;
  {
    //read in the stance specified by stancefile
    ifstream in(stancefile,ios::in);
    in >> stance;
    if(!in) {
      printf("Error loading stance file %s\n",stancefile);
      return 1;
    }
    in.close();
  }

  //If the stance has no contacts, use ContactPlan.  Otherwise, use StancePlan
  bool ignoreContactForces = false;
  if(NumContactPoints(stance)==0) {
    printf("No contact points in stance, planning without stability constraints\n");
    ignoreContactForces = true;
  }

  //set up the command line, store it into the MultiPath settings
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
  Config qstart = world.robots[robot]->q;
  for(size_t i=0;i+1<configs.size();i++) {
    MilestonePath mpath;
    bool res = false;
    if(ignoreContactForces)
      res = ContactPlan(world,robot,configs[i],configs[i+1],stance,mpath,termCond,plannerSettings);
    else
      res = StancePlan(world,robot,configs[i],configs[i+1],stance,mpath,termCond,plannerSettings);
    if(!res) {
      printf("Planning from stance %d to %d failed\n",i,i+1);
      path.sections.resize(path.sections.size()+1);
      path.SetStance(stance,path.sections.size()-1);
      path.sections.back().milestones[0] = configs[i];
      path.sections.back().milestones[1] = configs[i+1];
      break;
    }
    else {
      path.sections.resize(path.sections.size()+1);
      path.sections.back().milestones.resize(mpath.NumMilestones());
      path.SetStance(stance,path.sections.size()-1);
      for(int j=0;j<mpath.NumMilestones();j++)
	path.sections.back().milestones[j] = mpath.GetMilestone(j);
      qstart = path.sections.back().milestones.back();
    }
  }
  if(feasible)
    printf("Path planning success! Saving to %s\n",outputfile);
  else
    printf("Path planning failure. Saving placeholder path to %s\n",outputfile);
  const char* ext = FileExtension(outputfile);
  if(ext && 0==strcmp(ext,"path")) {
    printf("Converted to linear path format\n");
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
