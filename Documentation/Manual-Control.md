# Klamp't Manual: Control

Controllers provide the &quot;glue&quot; between the physical robot's actuators, sensors, and planners. They are very similar to planners in that they generate controls for the robot, but the main difference is that a controller is expected to work online and synchronously within a fixed, small time budget. As a result, they can only perform relatively light computations.

## Controllers

![Controller illustration](images/concepts-controller.png)

The number of ways in which a robot may be controlled is infinite, and can range from extremely simple methods, e.g., a linear gain, to extremely complex ones, e.g. an operational space controller or a learned policy. Yet, all controllers are structured as a simple callback loop: repeatedly read off sensor data, perform some processing, and write motor commands. The implementation of the internal processing is open to the user.

### Default motion queue controller

![Motion queue illustration](images/motion-queue.png)

The default controller for each robot is a motion-queued controller with optional feedforward torques, which simulates typical controllers for industrial robots. It supports piecewise linear and piecewise cubic interpolation, as well as time-optimal acceleration-bounded trajectories.   The trajectory interpolation profile is the standard trapezoidal velocity profile, except it also accepts interruption and arbitrary start and goal velocities.

![Trapezoidal velocity profiles](images/trapezoidal-velocity-profile.png)

(Note: One limitation of the API is that it is impossible to have certain joints controlled by a motion queue, while others are controlled by PID commands.)

#### C++ API

TODO

Specifically, the default controller is a `FeedforwardPolynomialPathController`.

#### Python API

**Basic commands**
- `controller = sim.getController(RobotModel or robot index)`: retrieves the simulation controller for the given wobot
- `controller.setPIDGains (kP,kI,kD)`: overrides the PID gains in the RobotModel to kP,kI,kD (lists of floats of lengths robot.numDrivers())
- `controller.setRate(dt)`: sets the time step of the internal controller to update every dt seconds
- `controller.setPIDCommand(qdes,[dqes])`: sets the desired PID setpoint 
- `controller.setVelocity(dqdes,duration)`: sets a linearly increasing PID setpoint for all joints, starting at the current setpoint, and slopes in the list dqdes. After duration time it will stop.
- `controller.setTorque(t)`: sets a constant torque command t, which is a list of n floats.

**Motion queue operations (wraps around a PID controller)**:
Convention: `setX` methods move immediately to the indicated milestone, `add/append` creates a motion from the end of the motion queue to the indicated milestone
- `controller.remainingTime()`: returns the remaining time in the motion queue, in seconds.
- `controller.set/addMilestone(qdes,[dqdes])`: sets/appends a smooth motion to the configuration qdes, ending with optional joint velocities dqdes.
- `controller.addMilestoneLinear(qdes)`: same as addMilestone, except the motion is constrained to a linear joint space path (Note: addMilestone may deviate)
- `controller.set/appendLinear(qdes,dt)`: sets/appends a linear interpolation to the destination qdes, finishing in dt seconds
- `controller.set/addCubic(qdes,dqdes,dt)`: moves immediately along a smooth cubic path to the destination qdes with velocity dqdes, finishing in dt seconds

**Querying robot state**
- `controller.getCommandedConfig()`: retrieve PID setpoint
- `controller.getCommandedVelocity()`: retrieve PID desired velocity
- `controller.getSensedConfig()`: retrieve sensed configuration from joint encoders
- `controller.getSensedVelocity()`: retrieve sensed velocity from joint encoders
- `controller.sensor(index or name)`: retrieve SimRobotSensor reference by index/name

### Custom controllers

Controllers can be dynamically and automatically loaded from world XML files via a statement of the form `<controller type="TheControllerType" attr1="value" ... />` under the `<simulation><robot>` element. The following controllers are supported:

- [<tt>JointTrackingController</tt> (Klampt/Control/JointTrackingController.h)](../Control/JointTrackingController.h): a simple open-loop controller that accepts a desired setpoint.
- [<tt>MilestonePathController</tt> (Klampt/Control/PathController.h)](../Control/PathController.h): an open-loop controller based on a `DynamicPath` trajectory queue.
- [<tt>PolynomialPathController</tt> (Klampt/Control/PathController.h)](../Control/PathController.h): an open-loop controller based on a `PiecewisePolynomialSpline` trajectory queue. Somewhat more flexible than `MilestonePathController`.
- [<tt>FeedforwardJointTrackingController</tt> (Klampt/Control/FeedforwardController.h)](../Control/FeedforwardController.h): a controller that additionally computes feedforward torques for gravity compensation and acceleration compensation. Works properly only with fixed-based robots. Otherwise works exactly like `JointTrackingController`.
- <tt>FeedforwardMilestonePathController</tt>: see above.
- <tt>FeedforwardPolynomialPathController</tt>: see above.
- [<tt>SerialController</tt> (Klampt/Control/SerialController.h)](../Control/SerialController.h): a thin communication layer that serves sensor data and accepts commands to/from a client controller through a serial interface.  It listens on the port given by the setting servAddr and sends sensor data at the rate writeRate (in Hz).  Sensor data and commands are converted to/from JSON format, in a form that is compatible with the Python API dictionaries used by the `control.BaseController` class (see also [Klampt/Python/control/controller.py](../Python/control/controller.py)).


_C++ API_: Any controller must subclass the `RobotController` class ([Klampt/Control/Controller.h](../Control/Controller.h)) and overload the Update method. The members sensors and command are available for the subclass to use. The basic control loop repeatedly executes:

1. The `RobotSensors*` sensors structure is filled in by the Klamp't simulation (or physical robot).
2. The `Update` method is called.  Here, the controller should fill in the `RobotMotorCommands*` command structure as necessary.
3. The command structure is read off by the Klamp't simulation (or physical robot).

New controller types can also be defined for dynamic loading in world XML files using the `RobotControllerFactory::Register(name,ptr)` function. This hook must be called before the world file is loaded. Afterward, the specified controller type will be instantiated whenever the registered type appears in the world file.

Custom controllers may expose various configuration settings to be loaded from XML files by implementing the `*Settings` methods. (These may also be manipulated by GUI programs and higher-level controllers/planners). They may also accept arbitrary external commands by overloading the `*Command*` methods.

_Python API_: To define a custom controller, the user should implement a custom control loop. At every time step, read the robot's sensors, compute the control, and then send the control to the robot via the `setPIDCommand` or `setTorqueCommand` methods.

[simtest.py](../Python/demos/simtest.py) also accepts arbitrary feedback controllers given as input. To do so, give it a .py file with a single `make(robot)` function that returns a controller object.  This object should be an instance of a subclass `BaseController` in [control.controller](../Python/control/controller.py).  For example, to see a controller that interfaces with ROS, see [control/roscontroller.py](../Python/control/roscontroller.py).

A Python controller is a very simple object with three important methods:

- `output(self,**inputs)`: given a set of named inputs, produce a dictionary of named outputs.  The semantics of the inputs and outputs are defined by the caller.
- `advance(self,**inputs)`: advance by a single time step, performing any necessary changes to the controller's state.  _Note: `output` should NOT change internal state!_
- `signal(self,type,**inputs)`: sends some asynchronous signal to the controller.  The usage is caller dependent.  (This method is never called directly by `simtest.py`.)

For simtest.py, the inputs to output and advance will be a sensor message as described in the controller communication protocol (CCP) in Section Error: Reference source not found.  The arguments are Python dictionaries.  `simtest.py` expects output to return a dictionary that represents a command message as described in the CCP.

Internally the controller can produce arbitrarily complex behavior.  Several common design patterns are implemented in [control/controller.py](../Python/control/controller.py):

- `TimedControllerSequence`: runs a sequence of sub-controllers, switching at predefined times.
- `MultiController`: runs several sub-controllers in parallel, with the output of one sub-controller cascading into the input of another. For example, a state estimator could produce a better state estimate q for another controller.
- `ComposeController`: composes several sub-vectors in the input into a single vector in the output. Most often used as the last stage of a MultiController when several parts of the body are controlled with different sub-controllers.
- `LinearController`: outputs a linear function of some number of inputs.
- `LambdaController`: outputs `f(arg1,...,argk)` for any arbitrary Python function `f`.
- `StateMachineController`: a base class for a finite state machine controller.  The subclass must determine when to transition between sub-controllers.
- `TransitionStateMachineController`: a finite state machine controller with an explicit matrix of transition conditions.

A trajectory tracking controller is given in [control/trajectory\_controller.py](../Python/control/trajectory_controller.py).  Its make function accepts a robot model (optionally `None`) and a linear path file name.

A preliminary velocity-based operational space controller is implemented in [control/OperationalSpaceController.py](../Python/control/OperationalSpaceController.py), but its use is highly experimental at the moment.



## Actuators

At the lowest level, a robot is controlled by _actuators_. These receive instructions from the controller and produce link torques that are used by the simulator. Klamp't supports three types of actuator:

- _Torque control_ accepts torques and feeds them directly to links.
- _PID control_ accepts a desired joint value and velocity and uses a PID control loop to compute link torques servo to the desired position. Gain constants kP, kI, and kD should be tuned for behavior similar to those of the physical robot. PID controllers may also accept feedforward torques.
- _Locked velocity__control_ drives a link at a fixed velocity. _Experimental_. (Note: this is different from &quot;soft&quot; velocity control which feeds a piecewise linear path to a PID controller)

Note that the PID control and locked velocity control loops are performed as fast as possible with the simulation time step. This rate is typically faster than that of the robot controller. Hence a PID controlled actuator typically performs better (rejects disturbances faster, is less prone to instability) than a torque controlled actuator with a simulated PID loop at the controller level.

_Important_: When using Klamp't to prototype behaviors for a physical robot, the simulated actuators should be calibrated to mimic the robot's true low-level motor behavior as closely as possible. It is also the responsibility of the user to ensure that the controller uses the simulated actuators in the same fashion as it would use the robot's physical actuators. For example, for a PID controlled robot with no feedforward torque capabilities, it would not be appropriate to use torque control in Klamp't. If a robot does not allow changing the PID gains, then it would not be appropriate to do so in Klamp't. Klamp't will not automatically configure your controller for compatibility with the physical actuators, nor will it complain if such errors are made.

_C++ API_. The `RobotMotorCommand` (Klampt/Control/Command.h) structure contains a list of `ActuatorCommand`s that are then processed by the simulator.

## Sensors

Klamp't can emulate a handful of sensors typically found on robots. At the user's level of abstraction, they generically provide streaming numerical-valued measurements. It is up to the user to process these raw measurements into meaningful information.

The following sensors are natively supported:

- `JointPositionSensor`: Standard joint encoders.
- `JointVelocitySensor`: Velocity sensors. Here velocities are treated as raw measurements, not differenced from a position encoder, and hence they are rarely found in real life. However, these will be good approximations of differenced velocity estimates from high-rate encoders.
- `CameraSensor`: An RGB or RGB-D camera.
- `LaserRangeSensor`: A laser rangefinder sensor.
- `DriverTorqueSensor`: Torques fed back from a robot's motors.
- `ContactSensor`: A contact switch/sensor defined over a rectangular patch.
- `ForceTorqueSensor`: A force/torque sensor at a robot's joint. Can be configured to report values from 1 to 6DOF.
- `Accelerometer`: An accelerometer. Can be configured to report values from 1 to 3 channels.
- `TiltSensor`: A tilt sensor. Can be configured to report values from 1 to 2 axes, and optionally tilt rates.
- `GyroSensor`: A gyroscope. Can be configured to report accelerations, velocities, or absolute rotations.
- `IMUSensor`: An inertial measurement unit that uses an accelerometer and/or gyroscope to provide estimates of a link's transformation and its derivatives. It will fill in the gaps that are not provided by the accelerometer / gyro using either integration or differencing.
- `FilteredSensor`: A &quot;virtual sensor&quot; that simply filters the measurements provided by another sensor.

A robot's sensors are dynamically configured via an XML tag of the form `<sensors> <TheSensorType name="some_name" attr1="value" ... " > </sensors>`. Each of the attribute/value pairs is fed to the sensor's SetSetting method, and details on sensor-specific settings are found in the documentation in [Control/Sensor.h.

These XML strings can be inserted into .rob files under a line property sensors [file], URDF files under the `<klampt>` element, or world XML files under the `<simulation>` and `<robot>` elements

### Python API 

The main interface to sensors is [SimRobotSensor](http://motion.pratt.duke.edu/klampt/pyklampt_docs/classklampt_1_1robotsim_1_1SimRobotSensor.html)

- `sensor = controller.sensor(index or name)`: retrieves a SimRobotSensor reference.
- `sensor.name()`: gets the sensor’s name string
- `sensor.type()`: gets the sensor’s type string
- `sensor.measurementNames()`: returns a list of strings naming the sensor’s measurements
- `sensor.getMeasurements()`: returns a list of floats giving the sensor’s measurements at the current time step

It is often useful to retrieve hypothetical sensor data without actually running a simulation, in particular for visual sensors.
- `sensor.kinematicSimulate(world,dt)`: kinematically simulates the sensor for its corresponding robot in the given world.


## State estimation

Controllers may or may not perform state estimation. If state estimation is performed, it is good practice to define the state estimator as independent of the controller, such as via a subclass of `RobotStateEstimator`. The `RobotStateEstimator` interface is fairly sparse, but the calling convention helps standardize their use in controllers.

### Using state estimators (C++ only)
Controllers should instantiate a state estimator explicitly on construction. Inside the `Update` callback, the controller should:

1. Call `ReadSensors(*sensors)`, then `UpdateModel()` to update the robot's model.
2. Read off the estimated state of the robot model (and potentially other information computed by the state estimator, such as uncertainty levels) and compute its command as usual.
3. Just before returning, call the `ReadCommand(*command)` and `Advance(dt)` methods on the `RobotStateEstimator` object.

A few experimental state estimators are available. `OmniscientStateEstimator` gives the entire actual robot state to the controller, regardless of the sensors available to the robot. `IntegratedStateEstimator` augments accelerometers and gyros with an integrator that tries to track true position. These integrators are then merged (in a rather simple-minded way) to produce the final model.

### Using state estimators (Python only)

Using the controller.py interface, state estimators can be implemented as `BaseController` subclasses that calculate the estimated state objects in the `output()` method.