# Klamp't Manual: Paths and Trajectories

* [Path and trajectory representations](#path-and-trajectory-representations)
    + [API summary](#api-summary)
* [Multipaths](#multipaths)
    + [API summary](#api-summary-1)
* [Trajectory Execution](#trajectory-execution)
    + [Sending to a Klamp't simulated robot](#sending-to-a-klamp-t-simulated-robot)
    + [Sending to a real robot](#sending-to-a-real-robot)

Klamp't distinguishes between _paths_ and _trajectories_: paths are geometric, time-free curves, while trajectories are paths with an explicit time parameterization. Mathematically, paths are expressed as a continuous curve

<center> <em> y(s):[0,1] -&gt; C </em> </center>

 while trajectories are expressed as continuous curves

<center> <em> y(t):[t<sub>i</sub>,t<sub>f</sub>] -&gt; C </em> </center>

where _C_  is the configuration space and _t<sub>i</sub>,t<sub>f</sub>_ are the initial and final times of the trajectory, respectively.

Classical motion planners compute paths, because time is essentially irrelevant for fully actuated robots in static environments. However, a robot must ultimately execute trajectories, so a planner must somehow prescribe times to paths before executing them. Various methods are available in Klamp't to convert paths into trajectories.  For example, to send a motion planner output to a controller in Python, call `klampt.model.trajectory.execute_trajectory`.

## Path and trajectory representations

| Type | Continuity | Timed? | Description |
|------|------------|--------|-------------|
| `Milestone list` | C0 | No | The simplest path type: a list of `Config` _milestones_ that should be piecewise linearly interpolated.  _Output from classical motion planners_. |
| `Piecewise linear` | C0 | Yes | Given by a list of `times` and `milestones` that should be piecewise linearly interpolated.  _The most compatible trajectory type_. |
| `DynamicPath` | C0 | Yes | Piecewise parabolic curve, which are time-optimal bounded-acceleration trajectories that include configuration, velocity, and time. |
| `Cubic spline` | C1 | Either | Piecewise cubic curve, with (Python) and without (C++) time. |
| `Time-scaled cubic splines` | C1 | Yes | An untimed cubic spline associated with an optimized time-domain parameterization. |
| `MultiPath` | C0 or C1 | Either | A rich container type for paths/trajectories annotated with changing contacts and IK constraints. |

_Note: to properly handle a robot's rotational joints, milestones should be interpolated via robot-specific interpolation functions. Cartesian linear interpolation does not correctly handle floating and spin joints. See the functions in_ Klampt/Modeling/Interpolate.h (C++) _and_ `RobotModel.interpolate()` (Python) _to do so_.  The Python API provides the `RobotTrajectory` class to do this automatically.

Especially for legged robots, the preferred path type is `MultiPath`, which allows storing both untimed paths and timed trajectories. It can also store multiple path sections with inverse kinematics constraints on each section.  More details on the `MultiPath` type [are given below](#multipaths).

### API summary

Piecewise linear trajectories are, with a historical misnomer, given in the `LinearPath` class of Klampt/Modeling/Paths.h (C++).  
Members include a `times` array of reals listing points in time along with a `milestones` list of associated `Config` milestones reached at each of those times.

Cubic spline interpolation of configuration lists are found in Klampt/Modeling/SplineInterpolate.h

Time-scaled cubic splines are found in the `TimeScaledBezierCurve` class in Klampt/Planning/TimeScaling.h (C++).


`DynamicPath` objects are computed via routines in Klampt/Modeling/Paths.h or Klampt/Modeling/DynamicPath.h.  These can take in milestone lists, milestone+velocity lists, and milestone+time lists given velocity and acceleration bounds. 

Conversions between most path types are supported in Klampt/Modeling/Paths.h (C++).





## Multipaths
A `MultiPath` is a rich path representation for legged robot motion. They contain one or more path(or trajectory) _sections_ along with a set of IK constraints and holds that should be satisfied during each of the sections. This information can be used to interpolate between milestones more intelligently, or for controllers to compute feedforward torques more intelligently than a raw path. They are loaded and saved to XML files. 

Each `MultiPath` section maintains a list of IK constraints in the `ikObjectives` member, and a list of `Hold`s in the holds member. There is also support for storing common holds in the `MultiPath`s `holdSet` member, and referencing them through a section's `holdNames` or `holdIndices` lists (keyed via string or integer index, respectively). This functionality helps determine which constraints are shared between sections, and also saves a bit of storage space.

`MultiPath`s also contain arbitrary application-specific settings, which are stored in a string-keyed dictionary member `settings`. Common settings include:

- `robot`, which indicates the name of the robot for which the path was generated.
- `resolution`, which indicates the resolution to which a path has been discretized. If resolution has not been set or is too large for the given application, a program should use IK to interpolate the path.
- `program`, the name of the procedure used to generate the path.
- `command_line`, the shell command used to invoke the program that generated the path.

Sections may also have custom settings. No common settings have yet been defined for sections, these are all application-dependent.

### API summary
Details can be found in Klampt/Modeling/MultiPath.h.




## Trajectory Execution

### Sending to a Klamp't simulated robot

Either run an `eval(t)` loop to send position commands, or use the [controller motion queuing process](Manual-Control.md#default-motion-queue-controller).

To observe a trajectory in simulation, you may save the file to disk as a `LinearPath` and the starting `Config`, then run

```sh
./SimTest [world file] -path [name of path file] -config [start config]
```

### Sending to a real robot

You will most likely have to build your own `eval(t)` loop.

You can also use `ROSPublishTrajectory` in Klampt/IO/ROS.h (C++) or build your own ROS `JointTrajectory` messages.
