# Klamp't Manual: Planning

* [Robot-level kinematic motion planning](#robot-level-kinematic-motion-planning)
    + [API summary](#api-summary)
* [Motion Planners](#motion-planners)
    + [Planner Attributes](#planner-attributes)
    + [API summary](#api-summary-1)
* [Randomized kinematic planning with closed-chain constraints](#randomized-kinematic-planning-with-closed-chain-constraints)
    + [API summary](#api-summary-2)
* [C-space-level kinematic motion planning](#c-space-level-kinematic-motion-planning)
    + [API summary](#api-summary-3)
* [Dynamic Trajectory Generation](#dynamic-trajectory-generation)
    + [Time-optimal acceleration-bounded trajectories](#time-optimal-acceleration-bounded-trajectories)
    + [Interpolation and time-optimization with closed-chain constraints](#interpolation-and-time-optimization-with-closed-chain-constraints)
    + [Time-scaling optimization](#time-scaling-optimization)
    + [Real-time motion planning](#real-time-motion-planning)

Motion planning is the problem of connecting two configurations with a feasible kinematic path or dynamic trajectory under certain constraints. The output may also be required to satisfy some optimality criteria.  Klamp't has the ability to plan:

- Collision-free kinematic paths in free space,
- Collision-free, stable kinematic paths on constraint manifolds,
- Minimum-time executions of a fixed trajectory under velocity and acceleration constraint,
- Minimum-time executions of a fixed trajectory under torque and frictional force constraints,
- Replanning under hard real-time constraints.

A variety of kinematic planning algorithms are supported, including:

**Feasible planners**: (only care about the first feasible solution)
- Probabilistic Roadmap (PRM) [Kavraki et al 1996]
- Rapidly-Exploring Random Tree (RRT) [LaValle and Kuffner 2001]
- Expansive Space Trees (EST ) [Hsu et al 1999]
- Single-Query Bidirectional Lazy Planner (SBL) [Sanchez-Ante and Latombe 2004]
- Probabilistic Roadmap of Trees [Akinc et al 2005] w/ SBL (SBL-PRT) 
- Multi-Modal PRM (MMPRM), Incremental-MMPRM [Hauser and Latombe 2009]

**Optimizing planners**: (incrementally improve the solution quality -- path length -- over time)
- RRT\* [Karaman and Frazzoli 2009]
- PRM\* [Karaman and Frazzoli 2009]
- Lazy-PRM\*, Lazy-RRG\* [Hauser 2015]
- Lower-Bound RRT\* (LB-RRT\*) [Salzman and Halperin 2014]
- Fast Marching Method (FMM) [Sethian 1996]
- Asymptotically optimal FMM (FMM\*) [Luo and Hauser 2014]
- Minimum Constraint Removal (MCR) and Minimum Constraint Displacement (MCD) [Hauser 2013]
- Randomized path shortcutting
- Random restarts

There are two levels of planning interface. The _robot-level_ interface is a higher-level interface automatically defines notions of sampling, collision checking, etc. (similar to the functionality of MoveIt!) The _configuration space_ interface is much lower level and more abstract, and requires the user to define feasibility tests and sampling routines (similar to the functionality of OMPL). The lower level approach is more tedious, but provides greater power.

Regardless of which interface you use, the general pipeline is as follows:

1. Construct a **planning problem**. Define the configuration space (C-space) and terminal conditions (start and goal configurations, or in general, sets)
2. Instantiate a **planning algorithm**. _Take care: some algorithms work with some problems and not others_.
3. **Call the planner**. Sampling-based planners are set up for use in any-time fashion:
    1. Plan as long as you want in a while loop, OR
    2. Set up a termination criterion

    Any-time planning means that the likelihood of success increases as more time spent. For _optimizing planners_, the quality of path improves too.  

4. **Retrieve** the path (sequence of milestones)

The resulting paths is then ready for execution or for postprocessing (smoothing).



## Robot-level kinematic motion planning

High-level kinematic motion planning generates collision-free paths for robots. The most basic form of planning considers fixed-base robots in free space (i.e., not in contact with the environment or objects).

- Standard Robot C-Space: avoids collisions
- Contact C-space: avoids collisions, maintains IK constraints
- Stance C-space: same as Contact C-space, but also enforces balance under gravity

You are allowed to do some additional modifications, like adding constraints.


### API summary

Example code is given in Klampt-examples/Cpp/plandemo.cpp (the application can be created via the command `make PlanDemo`).

The general way to plan a path connecting configurations `qstart` and `qgoal` is as follows:

1. Initialize a `WorldPlannerSettings` object for a `WorldModel` with the `InitializeDefault` method.
2. Create a `SingleRobotCSpace` (Klampt/Planning/RobotCSpace.h) with the WorldModel, the index of the robot (typically 0), and the initialized `WorldPlannerSettings` object.
3. Then, a `MotionPlannerFactory` (KrisLibrary/planning/AnyMotionPlanner.h) should be initialized with your desired planning algorithm. The "any" setting will choose an algorithm automatically.
4. Construct a `MotionPlanningInterface*` with the `Create()` method. Call `AddConfig(qstart)` and `AddConfig(qgoal)` on this object.
5. Call `PlanMore(N)` to plan for `N` iterations, or call `PlanMore()` until a time limit is reached. Terminate when `IsConnected(0,1)` returns true, and call `GetPath(0,1,path)` to retrieve the path.
6. Delete the `MotionPlanningInterface*`.

Example code is as follows.

```cpp
#include <Klampt/Planning/RobotCSpace.h>
#include <KrisLibrary/planning/AnyMotionPlanner.h>
using namespace Klampt;

//TODO: setup world
WorldPlannerSettings settings;
settings.InitializeDefault(world);
//do more constraint setup here if desired, e.g., set edge collision checking resolution
SingleRobotCSpace cspace(world,0,&settings); //plan for robot 0
MotionPlannerFactory factory;
factory.type = "any";  //options are "prm", "rrt", "sbl", "prm*", etc
//do more planner setup here if desired, e.g., change perturbation size
MotionPlannerInterface* planner = factory.Create(&cspace);
int istart=planner->AddMilestone(qstart); //istart should be 0
int igoal=planner->AddMilestone(qgoal); //istart should be 1
int maxIters=1000;
bool solved=false;
MilestonePath path;
for(int i=0;i<maxIters;i++) {
 planner->PlanMore();
 if(planner->IsConnected(0,1)) {
  planner->GetPath(0,1,path);
  solved=true;
  break;
 }
}
delete planner;
```

The default settings in `WorldPlannerSettings` (Klampt/Planning/PlannerSettings.h) and `MotionPlannerFactory` should be sufficient for basic testing purposes, but many users will want to tune them for better performance. For example, distance metric weights and contact tolerances may be tuned. Collision margins can be tuned by editing the margins of robot/object/terrain geometries.

To plan for part of a robot (e.g., the arm of a legged robot), the `SingleRobotCSpace2` class can be used instead. Be sure to configure the `fixedDofs` and `fixedValues` members before using it.

Note: although RobotCSpace.h contains multi-robot planning classes, they are not yet well-tested. Use at your own risk.





## Motion Planners

### Planner Attributes
- `type`: the overall planner type. Values include:
    - `prm`: the Probabilistic Roadmap algorithm
    - `rrt`: the Rapidly Exploring Random Trees algorithm
    - `sbl`: the Single-Query Bidirectional Lazy planner
    - `sblprt`: the probabilistic roadmap of trees (PRT) algorithm with SBL as the inter-root planner.
    - `rrt*`: the RRT* algorithm for optimal motion planning
    - `prm*`: the PRM* algorithm for optimal motion planning
    - `lazyprm*`: the Lazy-PRM* algorithm for optimal motion planning
    - `lazyrrg*`: the Lazy-RRG* algorithm for optimal motion planning
    - `fmm`: the fast marching method algorithm for resolution-complete optimal motion planning
    - `fmm*`: an anytime fast marching method algorithm for optimal motion planning

    The code also contains implementations of the following algorithms, but the MotionPlannerFactory
    does not yet support them:

        - `lazyprm`: the Lazy-PRM algorithm (interface not implemented yet)
        - `perturbation`: the PerturbationTree algorithm (interface not implemented yet)
        - `est`: the Expanding Space Trees algorithm (interface not implemented yet)
    
    If KrisLibrary is built with OMPL support, you can also use the type specifier "ompl:`[X]`" where `[X]` is one of:

        - `prm`, `lazyprm`, `prm*`, `lazyprm*`, `spars`
        - `rrt`, `rrtconnect`, `birrt`, `lazyrrt`, `lbtrrt`, `rrt*`, `informedrrt*`
        - `est`, `fmt`, `sbl`, `stride`

    (Note that OMPL's `lazyprm*` implementation performs much worse than the one in Klampt.)

- `knn`: k-nearest neighbors parameter.  Default is 10 for most planners.
- `connectionThreshold`: maximum distance over which a connection between two configurations is attempted.
- `perturbationRadius`: maximum expansion radius for RRT and SBL.
- `bidirectional`: 1 if bidirectional planning should be used.  Valid for RRT, SBL, RRT*.
- `shortcut`: 1 if post-processing smoothing should be used.  Turns a planner into an optimizing planner.
- `restart`: 1 if random-restarts should be used -- turns a planner into an optimizing planner.  If activated, `restartTermCond` must be present as well
- `restartTermCond`: a string that can be converted into a JSON object describing the termination condition.  For example, `"{foundSolution:1,maxIters:1000}"` restarts the underlying planner if it found a solution and spent 1000 iterations of planning.
- `suboptimalityFactor`: Used in RRT\* and PRM\*
- `ignoreConnectedComponents`: Used in PRM to connect nodes in the same connected component (default 0)
- `gridResolution`: Used in FMM, FMM\*, SBL, SBLPRT
- `pointLocation`: Specifies the point location data structure.  Accepted values are "" (brute force), "kdtree" (k-D tree), "random" (pick random point), "randombest [k]" (sample k points, pick closest)

These can also be specified in JSON format. [Examples are found in this directory](../Examples/PlanDemo).
For a complete description of the accepted options, see the `setPlanSetting` documentation in the Python/klampt/src/motionplanning.h file.

### API summary
Configuring a motion planner is done by a `MotionPlannerFactory` (KrisLibrary/planning/AnyMotionPlanner.h) should be initialized with your desired planning algorithm. The "any" setting will choose an algorithm automatically.
4. Construct a `MotionPlanningInterface*` with the `Create()` method. Call `AddConfig(qstart)` and `AddConfig(qgoal)` on this object.




## Randomized kinematic planning with closed-chain constraints

Klamp't has utilities to plan for collision-free motions that satisfy closed chain constraints (e.g., that a robot's hands and feet touch a support surface).  For the most part, once the CSpace has been set up, planning is identical to a standard CSpace.  However, the planner will construct a path whose milestones satisfy the constraints, but the _straight line path in C-Space between milestones will violate constraints_.  This is because the feasible motion lies on a lower-dimensional, nonlinear constraint manifold in configuration space. Rather, the path should be discretized finely on the constraint manifold before sending it to any function that assumes a configuration-space path, like a controller.

### API summary

The `ContactCSpace` class (Klampt/Planning/ContactCSpace.h) should be used in the place of `SingleRobotCSpace`. Fill out the `contactIK` member, optionally using the `Add*()` convenience routines. The kinematic planning approach can then be used as usual. Example code is given in Examples/contactplan.cpp (the application can be created via the command `make ContactPlan`).

To discretize a path into a interpolating piecewise-linear path on the manifold, there are two ways of doing so:
1. Using `MilestonePath.Eval()` with a fine discretization. This uses the internal ContactCSpace::Interpolate method, which interpolates while satisfying constraints.
2. Construct the path via the classes in Klampt/Planning/RobotConstrainedInterpolator.h. This approach guarantees that the resulting path is sufficiently close to the constraint manifold when interpolated linearly.

   To use `RobotConstrainedInterpolator`, construct an instance with the robot and its IK constraints. Then, calling `RobotConstrainedInterpolator.Make()` with two consecutive configurations will produce a list of finely-discretized milestones up to the tolerance `RobotConstrainedInterpolator.xtol`. Alternatively, the `RobotSmoothConstrainedInterpolator` class and the `MultiSmoothInterpolate` function can be used to construct a smoothed cubic path.




## C-space-level kinematic motion planning

For even more control over the planning process, the base C-space interfaces can be overridden with custom behavior. A wide variety of systems can be defined in the configuration space framework, including vehicles and other non-robotic mechanisms.

At the configuration-space-level interface, there is no notion of even a robot, just an abstract configuration space. Instead, you must manually implement the callbacks used by the planning algorithm:
- Feasibility tester `IsFeasible(q)`
- Visibility tester `IsVisible(a,b)`
- Sampling strategy `q <- SampleConfig()`
- *Perturbation sampling strategy `q <- SampleNeighborhood(c,r)`
- *Distance metric `d <- Distance(a,b)`
- *Interpolation function `q <- Interpolate(a,b,u)`

*: default implementation provided, assuming Cartesian space


The feasibility test is an _authoritative_ representation of C-space obstacles, and will be called thousands of times during planning. For sampling-based planners to work well, this must be fast (ideally, microseconds).


### API summary
Each C-space is a subclass of the configuration space interface class `CSpace` defined in KrisLibrary/planning/CSpace.h. Please see the documentation




## Dynamic Trajectory Generation

### Time-optimal acceleration-bounded trajectories

The result of kinematic planning is a sequence of milestones, which ought to be converted to a time-parameterized trajectory to be executed. The standard [path controllers](Manual-Control.md) do accept milestone lists and will do this internally. Occasionally you may want to do this manually, for example, to perform path smoothing before execution.  The example program in Examples/dynamicplandemo.cpp demonstrates how to do this (the program can be built using the command `make DynamicPlanDemo`).

This functionality is contained within the `DynamicPath` class in the Klampt/Modeling/DynamicPath.h file, which builds on the classes in Klampt/Modeling/ParabolicRamp.h. To shortcut a path, the following procedure is used:

1. Set the velocity and acceleration constraints, and optionally, the joint limits in the `DynamicPath`.
2. Call `SetMilestones()`. The trajectory will now interpolate linearly and start and stop at each milestone.
3. Subclass the `FeasibilityCheckerBase` class with the appropriate kinematic constraint checkers overriding `ConfigFeasible` and `SegmentFeasible`. Construct an instance of this checker.
4. Construct a `RampFeasibilityChecker` with a pointer to the `FeasibilityCheckerBase` instance and an appropriate checking resolution.
5. Call `Shortcut(N,checker)` where N is the desired number of shortcuts.

The resulting trajectory will be smoothed, will satisfy velocity and acceleration bounds, and will be feasible.

Warning: free-rotational joints (robots with free-floating bases) will not be interpolated correctly because this method assumes a Cartesian configuration space. Spin joints are also not handled correctly at step 3 but they can be handled by replacing step 5 with the WrappedShortcut method.

For more details, please see: _K. Hauser and V. Ng-Thow-Hing. Fast Smoothing of Manipulator Trajectories using Optimal Bounded-Acceleration Shortcuts. In proceedings of IEEE Int'l Conference on Robotics and Automation (ICRA), 2010._

### Interpolation and time-optimization with closed-chain constraints

Several routines in Klampt/Planning/RobotTimeScaling.h are used to interpolate paths under closed chain constraints. There is also functionality for converting paths to minimum-time, dynamically-feasible trajectories using a time-scaling method. The TrajOpt program will do this from the command line.

The suggested method for doing so is to use a MultiPath with the desired constraints in each section, and to input the control points as milestones. DiscretizeConstrainedMultiPath can be used to produce a new path that interpolates the milestones, but with a finer-grained set of constraint-satisfying configurations. EvaluateMultiPath interpolates a configuration along the path that satisfies the constraints. GenerateAndTimeOptimizeMultiPath does the same as DiscretizeConstrainedMultiPath except that the timing of the configurations is optimized as well.

Each method takes a resolution parameter that describes how finely the path should be discretized. In general, interpolation is slower with finer discretizations.

See the following reference for more details: K. Hauser. _Fast Interpolation and Time-Optimization on Implicit Contact Submanifolds_. Robotics: Science and Systems, 2013.

### Time-scaling optimization

The `TimeOptimizePath` and `GenerateAndTimeOptimizeMultiPath` functions in Klampt/Planning/RobotTimeScaling.h perform time optimization with respect to a robot's velocity and acceleration bounds. `TimeOptimizePath` takes a piecewise linear trajectory as input, interpolates it via a cubic spline, and then generates keyframes of time-optimized trajectory. `GenerateAndTimeOptimizeMultiPath` does the same except that it takes a `MultiPath` as input and output, and the constraints of the multipath may be first interpolated at a finer resolution before time-optimization is performed.

### Real-time motion planning

Real-time motion planning allows a robot to plan while executing a previously planned path. This allows the robot to avoid moving obstacles, improve path quality without large delays, and change its goals in real-time. It is critical to use a system architecture that tightly controls the synchronization between planning and execution; the planner must not spend more than a predetermined amount of time in computation before delivering the updated result, or else the path could change in an uncontrolled manner with catastrophic consequences. Moreover, such a method must be robust to unpredictable communication delays.

Klampt's real-time motion planning routines are built to handle these issues gracefully, and furthermore have the following theoretical guarantees

- The executed path is guaranteed to be continuous and within joint, velocity, and acceleration limits
- In a static environment the path is guaranteed to be collision free
- Any goal will eventually be reached given sufficient time (in wall clock time)

The main files containing this functionality are the `RealTimePlannerBase` base class in Klampt/Planning/RealTimePlanner.h and the subclass `RealTimeTreePlanner` in Klampt/Planning/RealTimeRRTPlanner.h. A complete implementation including communication with the User Interface Thread/Execution Thread is given in the `MTPlannerCommandInterface` class in Klampt/Interface/UserInterface.h.

Conceptually, the main requirement is that the Execution Thread and Planning Thread must be synchronized via a _motion queue_. The motion queue is a modifiable trajectory _y_(_t_) that is steadily executed by the Execution Thread. The Planning Thread is allowed to edit the motion queue asynchronously by _splicing_ in a changed path, which modifies the motion queue after at a given time. Right now, the motion queue must be a DynamicPath (in future implementations this requirement may be relaxed). Splices are specified on an absolute clock, because when a splice is made at time _t<sub>s</sub>_
, the planner must ensure that the old motion queue and the new suffix match at the same configuration _y(t<sub>s</sub>)_ and velocity at _y'(t<sub>s</sub>).

The Planning Thread should be initialized with the robot's initial configuration (or path) and its inner loop should proceed as follows:

1. Globally, the planner's objective is set using Reset and a planning cycle is begun at time _t<sub>p</sub>_ by calling PlanUpdate.
2. The planner determines a split time _t<sub>s</sub>_ and planning duration _t_. It is required that _t<sub>s</sub> &gt; t<sub>p</sub> + t_.
3. The planner tries to compute a path starting from _y(t<sub>s</sub>)_ and _y'(t<sub>s</sub>)_. If unsuccessful, the planning cycle terminates with failure.
4. Otherwise, the planner requests that the path gets spliced to the motion queue via the `SendPathCallbackBase` mechanism. The queue has an opportunity to reject the request, such as if it arrives after the current execution time or has incorrect configuration or velocity. A rejected splice is signaled by returning false to the callback.
5. Return to step 1.

The generic `SendPathCallbackBase` callback must be subclassed and implemented to make splice requests. In practice, properly implementing this callback requires locking and synchronization between threads. Either 1) the motion queue must be synchronized, or 2) splice requests are written to the Execution Thread, and a reply is written to the Planning Thread (as done in `MTPlannerCommandInterface`, the result to `SendPathCallbackBase` is queried via a polling mechanism).

A planning cycle can be interrupted with the `StopPlanning` method. This is useful to maintain responsiveness to changing user input.

There are two policies for determining the planning duration: constant and adaptive.  When using sampling-based planners we recommend using the adaptive time stepping mechanism because it adapts to planning problem difficulty. For deterministic planners, a well-chosen constant time step may be more appropriate.

