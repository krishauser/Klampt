#include "Control/PathController.h"
#include "Control/FeedforwardController.h"
#include "Control/LoggingController.h"
#include "Sensing/JointSensors.h"
#include "Simulation/Simulator.h"
#include "IO/XmlWorld.h"
#include "IO/XmlODE.h"
#include "IO/three.js.h"
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/utils/AnyCollection.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include <fstream>
using namespace Math3D;
using namespace Klampt;

string ReadFileAsString(const char* fn)
{
  string s;
  ifstream in(fn,ios::in | ios::binary);
  //read binary string
  char buf[1025];
  while(!in.eof()) {
    in.read(buf,1024);
    int n = (int)in.gcount();
    buf[n]=0;
    s += buf;
  }
  return s;
}

typedef LoggingController MyController;
typedef PolynomialPathController MyMilestoneController;


/*
	char buf[256];
	for(size_t i=0;i<sim.robotControllers.size();i++) {
	  sprintf(buf,"robot%d_commands.log",i);
	  LoggingController<FeedforwardController>* c = dynamic_cast<LoggingController<FeedforwardController>* >(&*sim.robotControllers[i]);
	  if(!c->SaveLog(buf)) {
	    fprintf(stderr,"Error writing to %s\n",buf);
	  }
	  else
	    printf("Saved commands to %s\n",buf);
	}
*/

/*
	char buf[256];
	for(size_t i=0;i<sim.robotControllers.size();i++) {
	  sprintf(buf,"robot%d_commands.log",i);
	  LoggingController<FeedforwardController>* c = dynamic_cast<LoggingController<FeedforwardController>* >(&*sim.robotControllers[i]);
	  if(!c->LoadLog(buf)) {
	    fprintf(stderr,"Error reading commands from %s\n",buf);
	  }
	  else {
	    printf("Loaded commands from %s\n",buf);
	    c->replay = true;
	    c->replayIndex = 0;
	    c->onlyJointCommands = true;
	    //hack
	    printf("HACK: removing delays from recorded commands\n");
	    c->RemoveDelays(0.2);
	    printf("Read %d commands\n",c->trajectory.size());
	    //check if it's for the right robot
	    if(!c->trajectory.empty()) {
	      if(c->trajectory[0].second.actuators.size() != c->command.actuators.size()) {
		fprintf(stderr,"Command file %s doesn't have the right number of actuators\n",buf);
		c->replay = false;
	      }
	    }
	  }
	}
*/

bool SetupCommands(Simulator& sim,const string& fn)
{
  if(fn.empty()) return true;
  if(0==strcmp(FileExtension(fn.c_str()),"log")) {
    Assert(sim.robotControllers.size()==1);
    LoggingController* c = dynamic_cast<LoggingController*>(&*sim.robotControllers[0]);
    if(!c->LoadLog(fn.c_str())) {
      fprintf(stderr,"Error reading commands from %s\n",fn.c_str());
      return false;
    }
    else {
      printf("Loaded commands from %s\n",fn.c_str());
      c->replay = true;
      c->replayIndex = 0;
      c->onlyJointCommands = true;
      //hack
      printf("HACK: removing delays from recorded commands\n");
      c->RemoveDelays(0.2);
      printf("Read %d commands\n",c->trajectory.size());
      //check if it's for the right robot
      if(!c->trajectory.empty()) {
	if(c->trajectory[0].second.actuators.size() != c->command->actuators.size()) {
	  fprintf(stderr,"Command file %s doesn't have the right number of actuators\n",fn.c_str());
	  c->replay = false;
	}
      }
    }
  }
  else { //load milestone path
    vector<Vector> milestones;
    vector<Vector> dmilestones;
    ifstream in(fn.c_str(),ios::in);
    if(!in) return false;
    Vector x,dx;
    while(in) {
      in >> x >> dx;
      if(in) {
	cout<<x<<", "<<dx<<endl;
	milestones.push_back(x);
	dmilestones.push_back(dx);
      }
    }
    in.close();

    Assert(sim.robotControllers.size()==1);
    for(size_t i=0;i<milestones.size();i++) {
      stringstream ss;
      ss<<milestones[i]<<"\t"<<dmilestones[i];
      if(i==0) {
	if(!sim.robotControllers[0]->SendCommand("set_qv",ss.str())) {
	  fprintf(stderr,"set_qv command does not work with the robot's controller\n");
	  return false;
	}
      }
      else {
	if(!sim.robotControllers[0]->SendCommand("append_qv",ss.str())) {
	  fprintf(stderr,"append_qv command does not work with the robot's controller\n");
	  return false;
	}
      }
    }
  }
  return true;
}

enum Format { None, Raw, Base64, ThreeJS };

const char* FormatExtension(Format format)
{
  if(format == None) return "";
  else if(format == ThreeJS) return "json";
  else return "state";
}

string ReadSimState(const char* fn,Format format)
{
  string str = ReadFileAsString(fn);
  if(format == Raw) return str;
  else return FromBase64(str);
}
bool WriteSimState(Simulator& sim,const char* fn,Format format)
{
  if(format == None) return true;
  else if(format == ThreeJS) {
    sim.UpdateModel();
    AnyCollection obj;
    ThreeJSExport(sim,obj);
    ofstream out(fn,ios::out|ios::binary);
    if(!out) {
      fprintf(stderr,"Unable to open file %s for writing\n",fn);
      return false;
    }
    out<<obj<<endl;
    out.close();
    return true;
  }
  else {
    string finalState;
    sim.WriteState(finalState);
    ofstream out(fn,ios::out|ios::binary);
    if(!out) {
      fprintf(stderr,"Unable to open file %s for writing\n",fn);
      return false;
    }
    string data;
    if(format == Raw) data = finalState;
    else data = ToBase64(finalState);
    out.write(data.c_str(),data.length());
    out.close();
    return true;
  }
}


const char* OPTIONS_STRING = "Options:\n\
\t-init file: resume simulating from a given initial state. \n\
\t            Multiple initial states may be issued.\n\
\t-settle time: set the settling time (default 0). \n\
\t-end time: set the final simulation time (default inf). \n\
\t-duration time: set the total simulation time (default 10). \n\
\t-m file: command a milestone file.  Multiple commands may be issued. \n\
\t-l file: command a log file.  Multiple commands may be issued. \n\
\t-prefix p: save states to files using the prefix p. \n\
\t-log file: save log files of the low-level robot commands. \n\
\t-step s: sets the simulation time step (default 1/1000)\n\
\t-format f: state encoding format (raw, base64, three.js default base64)\n\
";


int main(int argc, char** argv)
{ 
  if(argc < 2) {
    printf("USAGE: SimUtil [options] [xml_files, robot_files, terrain_files, object_files]\n");
    printf(OPTIONS_STRING);
    return 0;
  }
  XmlWorld xmlWorld;
  WorldModel world;
  Simulator sim;
  vector<string> commandFiles;
  double settlingTime = 0;
  double simEndTime = Inf;
  double simDuration = 10;
  string prefix="trial";
  vector<string> initialStates;
  string logFile;
  Format format = Base64;

  for(int i=1;i<argc;i++) {
    if(argv[i][0] == '-') {
      if(0==strcmp(argv[i],"-m")) {
	commandFiles.push_back(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-l")) {
	commandFiles.push_back(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-settle")) {
	settlingTime = atof(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-end")) {
	simEndTime = atof(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-duration")) {
	simDuration = atof(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-step")) {
	sim.simStep = atof(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-prefix")) {
	prefix = argv[i+1];
	i++;
      }
      else if(0==strcmp(argv[i],"-init")) {
	initialStates.push_back(ReadSimState(argv[i+1],format));
	i++;
      }
      else if(0==strcmp(argv[i],"-format")) {
	if(0==strcmp(argv[i+1],"none"))
	  format = None;
	else if(0==strcmp(argv[i+1],"raw"))
	  format = Raw;
	else if(0==strcmp(argv[i+1],"base64"))
	  format = Base64;
	else if(0==strcmp(argv[i+1],"three.js"))
	  format = ThreeJS;
	i++;
      }
      else {
	fprintf(stderr,"Unknown option %s\n",argv[i]);
	printf(OPTIONS_STRING);
	return 1;
      }
    }
    else {
      const char* ext=FileExtension(argv[i]);
      if(0==strcmp(ext,"xml")) {
	if(!xmlWorld.Load(argv[i])) {
	  printf("Error loading world file %s\n",argv[i]);
	  return 1;
	}
	if(!xmlWorld.GetWorld(world)) {
	  printf("Error loading world from %s\n",argv[i]);
	  return 1;
	}
      }
      else {
	if(world.LoadElement(argv[i]) < 0) {
	  return 1;
	}
      }
    }
  }

  //initialize simulation
  printf("Initializing simulation...\n");
  sim.Init(&world);
  printf("Done\n");

  //setup controllers
  sim.robotControllers.resize(world.robots.size());
  for(size_t i=0;i<sim.robotControllers.size();i++) {    
    RobotModel* robot=world.robots[i].get();
    sim.SetController(i,MakeDefaultController(robot)); 
    sim.controlSimulators[i].sensors.MakeDefault(robot);
  }

  //setup ODE settings, if any
  TiXmlElement* e=xmlWorld.GetElement("simulation");
  if(e) {
    printf("Reading simulation settings...\n");
    XmlSimulationSettings s(e);
    if(!s.GetSettings(sim)) {
      fprintf(stderr,"Warning, simulation settings not read correctly\n");
    }
    printf("Done\n");
  }
  
  //setup feedback
  //world-object
  for(size_t i=0;i<world.rigidObjects.size();i++) 
    sim.EnableContactFeedback(world.RigidObjectID(i),world.TerrainID(0));
  //robot-object
  for(size_t i=0;i<world.rigidObjects.size();i++) {
    for(size_t j=0;j<world.robots[0]->links.size();j++) {
      sim.EnableContactFeedback(world.RigidObjectID(i),world.RobotLinkID(0,j));
    }
  }
  for(size_t i=0;i<world.terrains.size();i++) {
    for(size_t j=0;j<world.robots[0]->links.size();j++) {
      sim.EnableContactFeedback(world.TerrainID(i),world.RobotLinkID(0,j));
    }
  }


  if(initialStates.empty()) {  
    initialStates.push_back(string());
    sim.WriteState(initialStates.back());
  }

  //allow the objects to settle and save that state
  if(settlingTime > 0) {
    for(size_t i=0;i<initialStates.size();i++) {
      if(initialStates.size() > 1) {
	if(!sim.ReadState(initialStates[i])) {
	  fprintf(stderr,"Warning, ReadState didn't work\n");
	  abort();
	}
      }

      sim.Advance(settlingTime);
      sim.WriteState(initialStates[i]);
    }
  }

  if(commandFiles.empty())
    commandFiles.resize(1);

  //run the commands
  for(size_t init=0;init<initialStates.size();init++) {
    for(size_t trial=0;trial<commandFiles.size();trial++) {
      if(initialStates.size() > 1 || trial > 0) {
	if(!sim.ReadState(initialStates[init])) {
	  fprintf(stderr,"Warning, ReadState didn't work\n");
	  abort();
	}
      }
      Real time0 = sim.time;

      printf("Loading commands...\n");
      bool res=SetupCommands(sim,commandFiles[trial]);
      if(!res) {
	fprintf(stderr,"Unable to load commands from %s\n",commandFiles[trial].c_str());
	return 1;
      }
      
      while(sim.time < simEndTime && sim.time < simDuration + time0 ) {
	printf("Time %g\n",sim.time);
	sim.Advance(sim.simStep);
      }
      
      //write the final state
      char buf[256];
      if(commandFiles.size()==1) {
	if(initialStates.size()==1) snprintf(buf,1024,"%s.%s",prefix.c_str(),FormatExtension(format));
	else snprintf(buf,1024,"%s%04d.%s",prefix.c_str(),init,FormatExtension(format));
      }
      else {
	if(initialStates.size()==1) snprintf(buf,1024,"%s%04d.%s",prefix.c_str(),trial,FormatExtension(format));
	else snprintf(buf,1024,"%s_%04d_%04d.%s",prefix.c_str(),init,trial,FormatExtension(format));
      }

      printf("Writing state at time %g to file %s\n",sim.time,buf);
      if(!WriteSimState(sim,buf,format)) {
	return 1;
      }
    }
  }
  return 0;
}
