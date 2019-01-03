


# Klamp't Tutorial: Motion Planning in Python

In this tutorial you will learn how to do the motion planning using built-in motion planner in Klamp't.

Klamp't has already included many motion planning algorithms such as PRM, RRT, FMM*, RRT*, randon-restart RRT planner and each motion planning algorithm can be set to run with optimization, resulting in a smooth trajectory instead of a piecewise linear one. Given the predefined configuration space, the motion planner takes in as input the start configuration and the goal configuration and generates a path connecting these two configurations.

Difficulty: intermediate

Time: 15-30 minutes

### Cspace and Motion Plan
Klampt has powerful tools to solve the motion planning problem. **klampt.plan.cspace** is the basic class to describe a motion planning problem. Two necessary class elements (**CSpace** and **MotionPlan**) are defined under this **cspace** class. 

 **CSpace**
 Motion planners aim to find a feasible path to connect the start configuration to the goal configuration inside the _CSpace_ and a clear definition of the _CSpace_ is a prerequisite for the continuation of the motion planning. A _CSpace_ is well defined if the following three properties are provided:
 - *bound*: a list of lower and upper bounds on the space [(l1,u1),...,(ld,ud)] where d is the dimension of the configuration space.
 - *eps*: the collision tolerance used for checking edges, in the units
  defined by the distance(a,b) method.  (by default, euclidean distance)
 - *feasibility test function*: logical output for the feasibility of given configuration

In Klampt, the first two properties are attributes while the third property can be added into two ways:
 - A direct definition in _CSpace.feasible(self.x)_
 - Added in _CSpace.addFeasibility_ method

**MotionPlan**
A motion planner, instantiated on a _CSpace_, can support kinemtics, point-to-point plans in Klampt. Since the integration of many motion planning algorithms, motion planning becomes a fairly straightforward task in Klampt. A motion planner is well defined if the following elements are provided:
 - CSpace: MotionPlan has an attribute to receive the defined CSpace.
 - Algorithm to be chosen: MotionPlan is initialized with two inputs: CSpace and which algorithm to use. To select the planning algorithm, just type the algorithm name in the init function. Available algorithms are prm, rrt, sbl, sblprt, rrt*, prm* and so on. 
  - Start/Goal configuration: a description of the start and goal configurations set in _[MotionPlan.setEndpoints (self, start, goal)]((http://motion.pratt.duke.edu/klampt/pyklampt_docs/classklampt_1_1plan_1_1cspace_1_1MotionPlan.html#a7f7eb6afc9b77ef1fc22a64b7891b5a9))_
  
An example to illustrate this motion planning process is to use the exercise file in Klampt/Python/exercises/motionplanning/ex.py.

Open up this Python file using text editor. To use the motion planning, the neccesity is to import CSpace class and MotionPlan class from **klampt.plan.cspace**
```
from klampt.plan.cspace import CSpace,MotionPlan
```
After the robot and obstacles have been defined in the configuration space, the motion planning problem can be formed and solved as follows 

 1. define a class which uses the previous configuration space and specifies one motion planning algorithm
 2. choose the initial configuration and goal configuration
 3. call the solver:
```
class CSpaceObstacleProgram(GLProgram):
    def __init__(self,space,start=(0.1,0.5),goal=(0.9,0.5)):
        GLProgram.__init__(self)
        self.space = space
        #PRM planner
        MotionPlan.setOptions(type="prm",knn=10,connectionThreshold=0.1)
        self.optimizingPlanner = False
```
In this class, "PRM" is chose to be the motion planning algorithm, "knn": k value for the k-nearest neighbor connection strategy (only for PRM), "connectionThreshold": a milestone connection threshold, "self.optimizingPlanner = False" means that currently the algorithm is not an optimal motion planning algorithm

klamp't has many motion planning algorithms and they can be switched by changing the type name

For example, an optimal RRT algorithm can be switched by
```
        MotionPlan.setOptions(type="rrt*")
        self.optimizingPlanner = True
```
ex.py has already included the codes of different motion planning algorithm and uncomment them to make switch
```
        #FMM* planner
        #MotionPlan.setOptions(type="fmm*")
        #self.optimizingPlanner = True
        
        #RRT planner
        #MotionPlan.setOptions(type="rrt",perturbationRadius=0.25,bidirectional=True)
        #self.optimizingPlanner = False
        
        #random-restart RRT planner
        #MotionPlan.setOptions(type="rrt",perturbationRadius=0.25,bidirectional=True,shortcut=True,restart=True,restartTermCond="{foundSolution:1,maxIters:1000}")
        #self.optimizingPlanner = True
```
Now let's run this ex.py. The following figure will show up.
<p align="center">
<img src="https://raw.githubusercontent.com/ShihaoWang/Figures/master/motion_planning1.jpg"
width="50%" height="50%">
</p>
Press "p" to see the path after planning for 100 iteration. Press "space" to see the path planning for each iteration.
<p align="center">
<img src="https://raw.githubusercontent.com/ShihaoWang/Figures/master/motion_planning2.jpg"
width="50%" height="50%">
</p>

_The green line is the final trajectory._

However, from this trajectory, we will be able to tell that the robot will collide with the obstacle and the boundary of the platform, so this is not the path we would like our robot to run with.

Why will the planner have infeasible path? currently, only the center is checked, so the robot collides with boundary and obstacles. 

**Feasibility Test**
How to solve this problem? The method is to redefine the configuration space. Previously, the configuration space is defined with respect to the center of the robot. This works if the robot can be considered as a point mass. However, in our exercise, the robot is modeled as a rigid circular body with radius r. So one easy way is to redefine the configuration space and make sure that the distance between the center of the robot and the boundary and the obstacle is at least r. So the method can be modified to be
```
    def feasible(self,q):
        #the radius of the robot r is 0.05
        #bounds test
        # We should decrease the size of the bound due to the radius of the mobile robot
        self.bound = [(0.05,0.95),(0.05,0.95)]
        if not CSpace.feasible(self,q): 
            return False       
        #make sure center point at least distance r from obstacles   
        for o in self.obstacles:
            if o.contains(q): return False
        return True
 ```
 Now the motion planning result is like this
 <p align="center">
<img src="https://raw.githubusercontent.com/ShihaoWang/Figures/master/motion_planning3.jpg"
width="50%" height="50%">
</p>

**Running with Optimization**
Up to now, we have not used the optimal motion planning algorithm. This part we will show how to use the motion planning algorithm with optimization. Inside the CSpaceObstacleProgram class, comment our default "#PRM planner" and uncomment the " #RRT* planner" :
```
        #RRT* planner
        MotionPlan.setOptions(type="rrt*")
        self.optimizingPlanner = True
```
Run this python script again. Press "p" continuously, you will find the the algorithm connects the new generated path which to make the final path towards a smooth path.
 <p align="center">
<img src="https://raw.githubusercontent.com/ShihaoWang/Figures/master/motion_planning4.jpg"
width="50%" height="50%">
</p>
Other motion planning methods can also be uncommented for test.

### Robot Planning
In the above example, the geometry of the robot is considered to be a 2-D circle and the environmental obstacle is also a simple planar shape so the motion planning problem becomes a problem that only requires the path in the global space. However, this simplification cannot address problems where the robot has compliated geometry, high degrees of freedom since in this case the robot's motion needs to be planned on its own _robotcspace_ such as robot manipulation problem in which the robot needs to move an object to a desired position while avoiding collision with the environment and itself during the whole manipulation process. 
 <p align="center">
<img src="https://raw.githubusercontent.com/ShihaoWang/Figures/master/motion_planning5.JPG"
width="50%" height="50%">
</p>
Follow the same idea as the motion planning, three necessary elements are to be provided:

 - RobotCSpace: A basic robot cspace that allows collision free motion. This can be initialized two main methods: one is to use the [klampt.plan.robotplanning.makeSpace()](http://motion.pratt.duke.edu/klampt/pyklampt_docs/namespaceklampt_1_1plan_1_1robotplanning.html) and the other is to use methods in [klampt.plan.robotcspace](http://motion.pratt.duke.edu/klampt/pyklampt_docs/namespaceklampt_1_1plan_1_1robotcspace.html) such as _klampt.plan.robotcspace.ClosedLoopRobotCSpace()_ or _klampt.plan.robotcspace.RobotCSpace_.
 - Goal and subgoals: Due to workspace of the manipulation task to be Euclidean space, the goal configuration can be solved using Inverse Kinematics. If there are other subgoals to be reached before the final goal, all the goals can be put together for the IK sover. However, Klampt has solvers to directly plan for path to Cartesian objectives.
 - Motion planning: Using the same _klampt.plan.cspace.MotionPlan_, the planning can be conducted in the robotcspace and the feasible path is then extracted to control the robot. Sample code is 
 ```
 wholepath = [configs[0]]
for i in range(len(configs)-1):
    t0 = time.time()
    print "Creating plan..."
    #Manual construction of planner 
    plan = cspace.MotionPlan(space, **settings)
    plan.setEndpoints(configs[i],configs[i+1])
    if plan is None:
        break
    print "Planner creation time",time.time()-t0
    t0 = time.time()
    plan.space.cspace.enableAdaptiveQueries(True)
    print "Planning..."
    for round in range(10):
        plan.planMore(50)
    print "Planning time, 500 iterations",time.time()-t0
    #this code just gives some debugging information. it may get expensive
    V,E = plan.getRoadmap()
    print len(V),"feasible milestones sampled,",len(E),"edges connected"
    path = plan.getPath()
    plan.space.close()
    plan.close()
```
See Python/demos/planningtest.py for more information.
 