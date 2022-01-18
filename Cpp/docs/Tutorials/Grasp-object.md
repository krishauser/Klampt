# Klamp't Tutorial: Grasp an Object in C++

In this tutorial we will learn a bit more about the planning tools in Klamp't by planning and simulating the execution of a simple action sequence: grasping and putting down a simple object.

Difficulty: advanced

Time: 30-45 minutes

## Conceptual overview
Before proceeding to the implementations, let's discuss the concepts used by the method.

We will make a hugely simplifying assumption that we can detect the position and orientation of the object, and we know precisely how the robot's gripper should be opened and closed to grasp it (specifically, the grasp's relative orientation, the gripper's pregrasp configuration, and the gripper's grasp configuration). In highly unstructured environments like a home, these assumptions are rarely satisfied, but they may be reasonable when the robot expects to see known, well localized objects, like in industrial environments.

The planner will have as input:

1.  A world file, with the robot and all objects in their current position.
2.  The name of the object to be grasped.
3.  A Grasp structure defining the transformation of the hand relative to the object, and the configuration of the fingers when the object is grasped. This will be known as the  _grasp_.
4.  A Grasp structure defining the transformation of the hand relative to the object, and the configuration of the fingers  _just before_  the object is grasped. This will be known as the  _pregrasp_.
5.  A desired motion of the object.

The grasp and pregrasp will be used to determine the  _grasp configuration_  and  _pregrasp configuration_  via inverse kinematics. The grasp and pregrasp will be used again to determine the  _ungrasp configuration_  and the  _postungrasp configuration_  for setting the object down. We will make the simplifying assumptions that 1) to grasp the object, you move in a straight line in configuration space from the pregrasp to the grasp configuration, and 2) to ungrasp the object, you move in a straight line from the ungrasp configuration to the postungrasp configuration. We'll also assume that the grasp is secure while the robot moves any link (keeping the fingers closed).

So, the planner will produce five motions:

1.  A transit motion, from the start configuration to the pregrasp configuration,
2.  A grasp motion from the pregrasp configuration in a straight line to the grasp configuration,
3.  A transfer motion, from the grasp configuration to the ungrasp configuration
4.  An ungrasp motion, from the ungrasp configuration in a straight line to the postungrasp configuration
5.  A transit motion, from the postungrasp configuration to the start configuration

Note that in motion 1, the planner must perform collision detection with the object in its original position. In motion 3, the planner must perform collision detection with the object in hand. In motion 5, the planner must perform collision detection with the object in the new position. In motions 2 and 4, we'll ignore collisions with the object because there will be some slight overlap of the hand and the object once the hand closes.

In the C++ API tutorial, we'll also demonstrate some of the dynamic planning tools in Klamp't. After planning a path, we'll dynamically smooth it, while maintaining feasibility, before executing it in simulation.

You may download  [the code for this example here](http://motion.cs.illinois.edu/klampt/0.7/tutorials/grasptutorial_cpp.zip). Throughout, we will assume that your code is placed in a directory that shares the same parent as Klamp't. We also assume you have learned how to build applications that link to Klamp't using CMake, e.g. by completing the  [simulation tutorial](Run-a-simulation-Cpp.md).

In the tutorial code we provide a framework for setting up the environment in pickandplace.h. Browse through the documentation of this file to understand the key members of the environment (robot, object, grasp, pregrasp, freeSpace). The Setup function loads a world file and some predefined Grasp resources from disk into a PickAndPlaceEnvironment structure. With this file you'll be able to set up the tutorial environment using the following skeleton code (in main.cpp):
```
#include "pickandplace.h"

int main(int argc,const char** argv)
{
  WorldModel world;
  PickAndPlaceEnvironment env;
  if(!Setup(env,world)) return 1;

  //TODO: plan a path to pick up the object in env
  return 0;
}
```
Specifically, we'll be using Klampt-examples/data/tx90cylinder.xml as the world file, and the grasps in Klampt-examples/data/resources/tx90pr2/ as grasp files.


### Sampling grasp configurations

We'll start by calculating the grasp and pregrasp configurations. In main.cpp, we'll put this functionality in a function whose prototype is:
```cpp
bool GraspPlan(PickAndPlaceEnvironment& env,int& maxIters,vector<Config>& result)
```
which returns the list of configurations from the start, to pregrasp, to grasp configurations. The following main code will call the function and output the configurations to disk.
```
  int maxIters = 100;
  vector result;
  if(!GraspPlan(env,maxIters,result)) {
    return 1;
  }
  printf("Saving configurations to grasp.configs\n");
  ofstream out("grasp.configs",ios::out);
  for(size_t i=0;i<result.size();i++)
    out<<result[i]<<endl;
```
Now let's begin to fill out that functionality. First, let's define a method for sampling the grasp configuration given an object transformation:
```cpp
bool SampleGraspConfiguration(PickAndPlaceEnvironment& env,
			      const RigidTransform& Tobject,
			      int& maxIters,
			      Config& out)
{
  Grasp worldGrasp;
  env.GetWorldGrasp(Tobject,worldGrasp);

  //we'll use the utilities in ContactCSpace to help sample a feasible
  //configuration
  ContactCSpace cspace(*env.freeSpace);
  //need to tell the cspace that we're using some IK constraints
  for(size_t i=0;i<worldGrasp.constraints.size();i++) 
    cspace.AddContact(worldGrasp.constraints[i]);
  //need to tell the cspace that we're fixing some finger DOFS
  cspace.fixedDofs = worldGrasp.fixedDofs;
  cspace.fixedValues = worldGrasp.fixedValues;

  //now, try solving IK from the given configuration
  if(!out.empty()) {
    env.robot->q = out;
    if(cspace.SolveContact()) {
      out = env.robot->q;
      return true;
    }
  }
  //sample and test
  while(maxIters > 0) {
    cspace.Sample(out);
    if(!cspace.CheckContact(out)) return true;
    maxIters--;
  }
  return false;
}

bool GraspPlan(PickAndPlaceEnvironment& env,int& maxIters,vector<Config>& result)
{
  Config qgrasp;
  //we'll seed the IK solver with the current start configuration
  qgrasp = env.robotStartConfig;
  if(!SampleGraspConfiguration(env,env.objectStartTransform,maxIters,qgrasp)) {
    printf("Unable to sample grasp configuration\n");
    return false;
  }
  //done
  result.push_back(qgrasp);
  return true;
}
```
As described in the IK tutorial, this is a random restart method that first tries to use the robot's start configuration as a seed for the IK solver, and if it fails, will restart from a random configuration. We're also using the ContactCSpace class for a lot of our functionality, because it automatically handles IK solving and fixed degrees of freedom in a robot's fingers.

Make and run your program using
```
cmake .  
make PickAndPlaceTutorial
```
The output spend a few seconds loading files and doing some calculations, and should display a success message. To see the sampled configurations, run:
```
../Klampt/RobotPose ../Klampt-examples/data/tx90cylinder.xml grasp.configs
```
One issue that this code hasn't addressed is that the configuration produced by the sampler may collide with obstacles in the world. To restrict ourselves to feasible configurations, we will need to add feasibility checking. You can do this manually yourself, or you can use the functionality in ContactCSpace for checking collisions. To do this we'll have to add feasibility tests into our SampleGraspConfiguration function:
```cpp
  //now, try solving IK from the given configuration
  if(!out.empty()) {
    env.robot->q = out;
    if(cspace.SolveContact()) {
      out = env.robot->q;
      if(cspace.IsFeasible(out)) 
	return true;
    }
  }
  //sample and test
  while(maxIters > 0) {
    cspace.Sample(out);
    if(cspace.IsFeasible(out)) return true;
    maxIters--;
  }
  return false;
```
Notice that cspace.IsFeasible() is checked for each candidate configuration. If you compile and run, however, you'll get a failure message: "Failed to sample grasp configuration". What's going on?

We can start to debug by temporarily inserting calls to the cspace.PrintInfeasibleNames() method:
```cpp
  if(!out.empty()) {
    env.robot->q = out;
    if(cspace.SolveContact()) {
      out = env.robot->q;
      if(cspace.IsFeasible(out)) 
	return true;
      else {
        printf("Infeasible:\n");
	cspace.PrintInfeasibleNames(out);
	getchar();
      }
    }
  }
  //sample and test
  while(maxIters > 0) {
    cspace.Sample(out);
    if(cspace.IsFeasible(out)) return true;
    else {
      printf("Infeasible:\n");
      cspace.PrintInfeasibleNames(out);
      getchar();
    }
    maxIters--;
  }
```
This code will print out the constraints that are violated at the tested configuration. Running it again, we get output like:
```
Infeasible:  
coll[TX90L-pr2[pr2gripper:Link 2],target]  
coll[TX90L-pr2[pr2gripper:Link 4],target]
```
which tells us which constraints are violated. The "coll[x,y]" constraint indicates a collision between item x and item y. Since "TX90L-pr2[pr2gripper:Link X]" is the X'th link of the pr2 gripper at the end of the robot, and "target" is the name of the object it looks like the gripper fingers are colliding with the grabbed object! This is a relatively common situation with grasps -- you typically set them up so that the robot's fingers are slightly touching the object.

What we can do is ignore certain collision tests in the cspace.IsFeasible() method. To do so, add the following lines before the comment line "//all set up":
```cpp
  int objectID = env.world->RigidObjectID(env.iobject);
  for(size_t i=0;i<worldGrasp.constraints.size();i++) {
    int k=worldGrasp.constraints[i].link;
    cspace.ignoreCollisions.push_back(pair<int,int>(objectID,env.world->RobotLinkID(env.irobot,k)));
  }
  for(size_t i=0;i<worldGrasp.fixedDofs.size();i++) {
    int k=worldGrasp.fixedDofs[i];
    cspace.ignoreCollisions.push_back(pair<int,int>(objectID,env.world->RobotLinkID(env.irobot,k)));
  }
```
The first for loop ignores contact between the object and the hand link, and the second for loop ignores contact between the object and the finger link. Go ahead and recompile, and the sampler should now give a perfect result as it did before.

Now, if we try to plan a collision free path in the robot's free space to the grasp configuration, it will fail, because the endpoint is infeasible: the fingers are still colliding! This is precisely why we need a pregrasp configuration.

To compute the pregrasp configuration, we'll simply want to open the gripper's finger degrees of freedom to those specified in the pregrasp. To do so, we can add the following lines to the GraspPlan function after the //done comment:
```cpp
  Config qpregrasp = qgrasp;
  for(size_t i=0;i<env.pregrasp.fixedDofs.size();i++)
    qpregrasp[env.pregrasp.fixedDofs[i]] = env.pregrasp.fixedValues[i];
  result.push_back(qpregrasp);
  result.push_back(qgrasp);
```
That's it! If you open up graspsample.cpp, you'll see everything that we did, plus a little more extra for debugging, handling different pregrasp shapes, and feasibility testing of the pregrasp (in case the pre-grasp fingers collide with something). Either using the code that you've written into main.cpp or the existing code in graspsample.cpp, Let's move to the next tutorial.
### Motion planning

The first plan we'll run will connect the start configuration to the pregrasp configuration. Let's start by changing some of the framework code to produce paths. We'll change GraspPlan to produce a MultiPath result, with the following code going into main.cpp:
```cpp
  //do the planning, save the result
  int maxIters = 100;
  int numRemainingIters = maxIters;
  MultiPath result;
  Timer timer;
  if(!GraspPlan(env,numRemainingIters,result)) {
    return 1;
  }
  printf("Success, %d iterations and %gs elapsed\n",maxIters-numRemainingIters,timer.ElapsedTime());
  printf("Assigning duration of 5 s\n");
  result.SetDuration(5.0);
  printf("Saving path to grasppath.xml");
  result.Save("grasppath.xml");
```
The new prototype for GraspPlan will be
```
bool GraspPlan(PickAndPlaceEnvironment& env,int& maxIters,MultiPath& result)
```
Now we'll have to plan the path and put the path into the result, rather than simply returning a list of configurations. In GraspPlan, we'll add the following code after the grasp configuration is sampled:
```cpp
  //2. perform motion planning
  printf("Beginning motion planning for grasp path...\n");
  env.object->T = env.objectStartTransform;
  env.object->UpdateGeometry();
  maxItersRemaining = maxIters;
  if(!Plan(env.freeSpace,env.robotStartConfig,qpregrasp,maxIters,graspPath)) {
    printf("Unable to plan grasp path within %d iters\n",maxItersRemaining);
    return false;
  }
```
And add the following code to the end of the function to assemble the output into MultiPath format
```cpp
  //3. assemble the path start->...->pregrasp->grasp
  result.settings["robot"] = env.world->robots[env.irobot].name;
  result.sections.resize(2);
  result.sections[0].settings["name"]="start->pregrasp";
  result.sections[0].milestones.resize(graspPath.NumMilestones());
  for(size_t i=0;i<result.sections[0].milestones.size();i++)
    result.sections[0].milestones[i] = graspPath.GetMilestone(i);
  result.sections[1].settings["name"]="pregrasp->grasp";
  result.sections[1].milestones.push_back(qpregrasp);
  result.sections[1].milestones.push_back(qgrasp);
```
To perform a point-to-point plan in a given configuration space from A to B, we'll define the following utility function somewhere in our file:
```cpp
//a utility function that will do a point-to-point plan and count down
//the number of iterations spent
bool Plan(CSpace* space,const Config& a, const Config& b,
	  int& maxIters,MilestonePath& path)
{
  MotionPlannerFactory factory;
  factory.perturbationRadius = 0.5;
  SmartPointer<MotionPlannerInterface> planner = factory.Create(space,a,b);
  while(maxIters > 0) {
    planner->PlanMore();
    maxIters--;
    if(planner->IsSolved()) {
      planner->GetSolution(path);
      return true;
    }
  }
  return false;
}
```
If you then compile your project and run it, you will see it produce the grasp and pregrasp configuration, then spend a few iterations planning the grasp path. To see the output, run:
```
SimTest ../Klampt-examples/data/tx90cylinder.xml -path grasppath.xml
```
You will see the robot grasping the cylinder! However, it does it in a somewhat jerky fashion. Don't worry, we'll fix that later!

The code for this example is found in grasp.cpp, except with a bit more debugging.

### Planning transfer and ungrasping paths

Coming soon. Code for this example is found in pickandplace.cpp. You can run the planner by building and running PickAndPlaceTutorial3. To see the output, run:
```
SimTest ../Klampt-examples/data/tx90cylinder.xml -path pickandplacepath.xml
```
You will see the robot grasping, placing, and ungrasping the cylinder.

### Dynamic optimization
Coming soon. Code for this example is found in pickandplace_optimized.cpp. You can run the planner by building and running PickAndPlaceTutorial2. To see the output, run:
```
SimTest ../Klampt-examples/data/tx90cylinder.xml -path pickandplacepath_smoothed.xml
```
You will see the robot grasping, placing, and ungrasping the cylinder with smooth motions.
