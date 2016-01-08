#include "Interface/SimulationGUI.h"
#include "Control/TabulatedController.h"
#include "Control/SerialControlledRobot.h"
#include <KrisLibrary/utils/indexing.h>
#include <KrisLibrary/utils/stringutils.h>
#include <fstream>

void OptimizeCartPole(Robot& robot)
{
  TabulatedController controller(robot);
  Assert(robot.q.n==2);
  Vector bmin(4),bmax(4),h(4);
  bmin.copySubVector(0,robot.qMin);
  bmax.copySubVector(0,robot.qMax);
  bmin.copySubVector(robot.q.n,robot.velMin);
  bmax.copySubVector(robot.q.n,robot.velMax);
  h(0) = h(1) = 0.25;
  h(2) = h(3) = 0.5;
  Vector qdes=robot.q,w(2);
  w(0) = 0.1;
  w(1) = 1.0;
  controller.commands.grid.h = h;
  controller.commands.grid.PointToIndex(bmin,controller.commands.imin);
  controller.commands.grid.PointToIndex(bmax,controller.commands.imax);
  controller.commands.Init(controller.commands.imin,controller.commands.imax);
  OptimizeMDP(controller,qdes,w,100,0.999);
  ofstream out("cartpole.policy",ios::out);
  controller.Save(out);
  out.close();
}

void OptimizeSwingUp(Robot& robot)
{
  TabulatedController controller(robot);
  Assert(robot.q.n==1);
  Vector bmin(2),bmax(2),h(2);
  bmin.copySubVector(0,robot.qMin);
  bmax.copySubVector(0,robot.qMax);
  bmin.copySubVector(robot.q.n,robot.velMin);
  bmax.copySubVector(robot.q.n,robot.velMax);
  //set the resolution here
  //h(0)=0.1;
  //h(1)=0.2;
  //h(0)=0.1;
  //h(1)=0.1;
  h(0)=0.05;
  h(1)=0.1;
  Vector qdes=robot.q,w(1);
  w(0) = 1.0;
  controller.commands.grid.h = h;
  controller.commands.grid.PointToIndex(bmin,controller.commands.imin);
  controller.commands.grid.PointToIndex(bmax,controller.commands.imax);
  controller.commands.Init(controller.commands.imin,controller.commands.imax);
  OptimizeMDP(controller,qdes,w,20,0.999);
  ofstream out("swingup.policy",ios::out);
  controller.Save(out);
  out.close();
}



typedef TabulatedController MyController;
inline MyController* GetController(RobotController* rc)
{
  return dynamic_cast<MyController*>(rc);
}
inline RobotController* MakeController(Robot* robot,const char* file)
{
  TabulatedController* c = new TabulatedController(*robot);
  ifstream in(file,ios::in);
  if(!in) {
    fprintf(stderr,"Unable to open policy file %s\n",file);
    exit(-1);
  }
  if(!c->Load(in)) {
    fprintf(stderr,"Error loading policy from file %s\n",file);
    exit(-1);
  }
  return c;
}
inline void MakeDefaultSensors(Robot* robot,RobotSensors& sensors)
{
  JointPositionSensor* jp = new JointPositionSensor;
  JointVelocitySensor* jv = new JointVelocitySensor;
  jp->name = "q";
  jv->name = "dq";
  jp->q.resize(robot->q.n,Zero);
  jv->dq.resize(robot->q.n,Zero);
  sensors.sensors.push_back(jp);
  sensors.sensors.push_back(jv);
}

int main(int argc, const char** argv)
{  
  if(argc < 2) {
    printf("USAGE: CartPole [options] [robot or world files]\n");
    printf("OPTIONS: \n");
    printf(" -cartpole: use swingup task (default on)\n");
    printf(" -swingup: use swingup task (default off)\n");
    printf(" -optimize: optimize policy for the robot (default on)\n");
    printf(" -control: run a SerialControlledRobot using optimized policy (default off)\n");
    return 0;
  }
  bool cartpole=true,swingup=false,optimize=true,control=false;

  int i;
  for(i=1;i<argc;i++) {
    if(argv[i][0] == '-') {
      if(0==strcmp(argv[i],"-cartpole")) {
	cartpole=true;
	swingup=false;
      }
      else if(0==strcmp(argv[i],"-swingup")) {
	swingup=true;
	cartpole=false;
      }
      else if(0==strcmp(argv[i],"-optimize")) {
	optimize=true;
	control=false;
      }
      else if(0==strcmp(argv[i],"-control")) {
	control=true;
	optimize=false;
      }
      else {
	printf("Unknown option %s",argv[i]);
	return 1;
      }
    }
    else {
      break;
    }
  }
  RobotWorld world;
  SimGUIBackend backend(&world);
  if(!backend.LoadAndInitSim(argc-i+1,&argv[i-1]))
    return 1;

  Robot* robot = world.robots[0];
  if(optimize) {
    cout<<"Optimizing policy around setpoint "<<robot->q<<"..."<<endl;
    if(swingup)
      OptimizeSwingUp(*robot);
    else if(cartpole)
      OptimizeCartPole(*robot);
  }
  else if(control) {
    printf("Starting serial controlled robot server on localhost:3456...\n");
    //start up a controller server
    SerialControlledRobot server("tcp://localhost:3456");
    server.klamptRobotModel = robot;
    server.klamptController = MakeController(robot,"swingup.policy"); 
    server.command.actuators.resize(robot->drivers.size());
    server.sensors = backend.sim.controlSimulators[0].sensors;
    printf("Running forever (press Ctrl+C to quit)...\n");
    server.Run();
  }
  return 0;
}
