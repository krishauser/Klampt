#include "Planning/StanceCSpace.h"
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/utils/stringutils.h>
#include "Modeling/Paths.h"
#include "Modeling/MultiPath.h"
#include "IO/XmlWorld.h"
#include <string.h>
#include <fstream>

///A margin used in StancePlan on the distance between the COM and the
///edge of the support polygon
Real gSupportPolygonMargin = 0;

/** @brief Plans from a start configuration to a goal set.
 */
bool PlanToSpace(CSpace* space,const Config& qstart,CSpace* goalSpace,
		 MilestonePath& path,
		 const HaltingCondition& cond,const string& plannerSettings="")
{
  MotionPlannerFactory factory;
  if(!plannerSettings.empty())
    factory.LoadJSON(plannerSettings);
  //do more planner setup here if desired, e.g., change planner type,
  //perturbation size, connection radius, etc
  MotionPlannerInterface* planner = factory.Create(space,qstart,goalSpace);
  string res = planner->Plan(path,cond);
  cout<<"Planner terminated with condition "<<res<<endl;
  delete planner;
  return !path.edges.empty();
}


/** @brief Performs path planning in collision-free space for the
 * given robot at the stance sstart and destination stance sgoal.
 * Stability is tested at each configuration.
 *
 * The output is given in path.  cond controls the number of iterations/time
 * for planning.
 *
 * The constraint specifications are given in WorldPlannerSettings. If you
 * have custom requirements, you will need to set them up.
 */
bool StancePlan(RobotWorld& world,int robot,const Config& qstart,const Stance& sstart,const Stance& sgoal,
		MilestonePath& path,
		const HaltingCondition& cond,const string& plannerSettings="")
{
  WorldPlannerSettings settings;
  settings.InitializeDefault(world);
  //do more constraint setup here (modifying the settings object) if desired,
  //e.g., set collision margins, edge collision checking resolution, etc.
  StanceCSpace cspace(world,robot,&settings); 
  StanceCSpace transitionCspace(world,robot,&settings); 
  cspace.SetStance(sstart);
  cspace.CalculateSP();
  if(gSupportPolygonMargin != 0)
    cspace.SetSPMargin(gSupportPolygonMargin);

  //need to determine the transition stance conditions
  vector<Hold> addedHolds,removedHolds;
  GetDifference(sstart,sgoal,removedHolds);
  GetDifference(sgoal,sstart,addedHolds);
  if(!addedHolds.empty() && !removedHolds.empty()) {
    fprintf(stderr,"StancePlan: A stance change must either remove or add holds, not both\n");
    return false;
  }
  if(addedHolds.empty()) {
    transitionCspace.SetStance(sgoal);
    //add the IK constraint of the removed holds but not the contacts
    for(size_t i=0;i<removedHolds.size();i++) {
      removedHolds[i].contacts.clear();
      transitionCspace.SetHold(removedHolds[i]);
    }
  }
  else {
    transitionCspace.SetStance(sstart);
    //add the IK constraint of the added holds but not the contacts
    for(size_t i=0;i<addedHolds.size();i++) {
      addedHolds[i].contacts.clear();
      transitionCspace.SetHold(addedHolds[i]);
    }
  }

  //plan to reach the transition cspace
  return PlanToSpace(&cspace,qstart,&transitionCspace,path,cond,plannerSettings);
}

/** @brief Performs path planning in collision-free space for the
 * given robot at the stance sstart and destination stance sgoal.
 * Stability is *not* tested.  If you wish to maintain stability use
 * StancePlan
 *
 * The output is given in path.  cond controls the number of iterations/time
 * for planning.
 *
 * The constraint specifications are given in WorldPlannerSettings. If you
 * have custom requirements, you will need to set them up.
 */
bool ContactPlan(RobotWorld& world,int robot,const Config& qstart,const Stance& sstart,const Stance& sgoal,MilestonePath& path,
		 const HaltingCondition& cond,const string& plannerSettings="")
{
  WorldPlannerSettings settings;
  settings.InitializeDefault(world);
  //do more constraint setup here (modifying the settings object) if desired,
  //e.g., set collision margins, edge collision checking resolution, etc.

  //create the space corresponding to sstart, and the transition space
  //corresponding to the union of sstart and sgoal
  ContactCSpace cspace(world,robot,&settings); 
  for(Stance::const_iterator i=sstart.begin();i!=sstart.end();i++)
    cspace.AddContact(i->second.ikConstraint);
  vector<Hold> addedHolds,removedHolds;
  GetDifference(sgoal,sstart,addedHolds);

  if(addedHolds.empty()) {
    //no added holds! no change in configuration needed
    path.edges.resize(1);
    path.edges[0] = cspace.LocalPlanner(qstart,qstart);
    return true;
  }
  else {
    for(Stance::const_iterator i=sstart.begin();i!=sstart.end();i++)
      transitionCspace.AddContact(i->second.ikConstraint);
    //add the IK constraint of the added holds 
    for(size_t i=0;i<addedHolds.size();i++) {
      transitionCspace.AddContact(addedHolds[i].ikConstraint);
    }
  }

  //plan to reach the transition cspace
  return PlanToSpace(&cspace,qstart,&transitionCspace,path,cond);
}



int main(int argc,const char** argv)
{
  if(argc <= 2) {
    printf("USAGE: MultiContactPlan [options] world_file stance0 stance1 ...\n");
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
    printf("USAGE: ContactPlan [options] world_file stance0 stance1 ...\n");
    return 1;
  }
  const char* worldfile = argv[i];

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

  vector<Stance> stances;
  for(int k=i+1;k<argc;k++) {
    stances.resize(stances.size()+1);
    ifstream in(argv[k],ios::in);
    in >> stances.back();
    if(!in) {
      printf("Error loading stance file %s\n",argv[k]);
      return 1;
    }
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
  Config qstart = world.robots[robot].robot->q;
  for(size_t i=0;i+1<stances.size();i++) {
    MilestonePath mpath;
    if(!StancePlan(world,robot,qstart,stances[i],stances[i+1],mpath,termCond,plannerSettings)) {
      printf("Planning from stance %d to %d failed\n",i,i+1);
      path.sections.resize(path.sections.size()+1);
      path.SetStance(stances[i],path.sections.size()-1);
      path.sections.back().milestones[0] = qstart;
      path.sections.back().milestones[1] = qstart;
      break;
    }
    else {
      path.sections.resize(path.sections.size()+1);
      path.sections.back().milestones.resize(mpath.NumMilestones());
      path.SetStance(stances[i],path.sections.size()-1);
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
