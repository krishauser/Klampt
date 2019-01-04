# Frequently Asked Questions (FAQ)
  
## Should I learn the Python bindings or C++?

This is mostly a matter of preference. Python tends to be cleaner, easier to use, and faster for prototyping. However, the Python bindings provide a subset of the C++ functionality.


## How do I set up sensors in the simulator and read them?

Sensors are set up in the `property sensors` line of the robot file or world XML file. See [data/robots/huboplus/huboplus\_col.rob](../robots/huboplus/huboplus_col.rob) and [data/simulation\_test\_worlds/sensortest.xml](../data/simulation_test_worlds/sensortest.xml) for some examples. Sensors can be debugged and drawn in RobotTest.

To read sensor data declare a variable `vector<double> measurements` and call
```cpp
WorldSimulation.controlSimulators[robotIndex].sensors.GetNamedSensor(sensorName)->GetMeasurements(measurements);
```


## My simulator goes unstable and/or crashes. Help!

There are two reasons that the simulator may go unstable: 1) the simulated robot is controlled in an inherently unstable manner, or 2) rigid body simulation artifacts due to poor collision handling or numerical errors. The second reason may also cause ODE to crash, typically on Linux systems. In testing we have found that configuring ODE with double precision fixes such crashes.

_Unstable robot_: an unstably controlled robot will oscillate and jitter, and if these oscillations become violent enough they may also cause rigid body simulation instability/crashing. If the robot goes unstable, then its PID constants and dryFriction/viscousFriction terms need to be tuned. These values must be set carefully in order to avoid oscillation and, ideally should be calibrated against the physical motors' behavior. This is currently an entirely manual process that must be done for every new robot. As a rule of thumb, large PID damping terms are usually problematic, and should be emulated as viscous friction.

_Collision handling errors_: Klamp't uses a contact handling method wherein each mesh is wrapped within a thin _boundary layer_ that is used for collision detection. When objects make contact only along their boundary layers, the simulation is robust, but if their underlying meshes penetrate one another, then the simulator must fall back to less robust contact detection methods. This occurs if objects are moving too quickly or light objects in contact are subject to high compressive forces. If this happens, Klamp't will print a warning of the form &quot;ODECustomMesh: Triangles penetrate margin X, cannot trust contact detector&quot;. The simulator status will also return &quot;unreliable.&quot;

To avoid penetration, there are two remedies: 1) increase the thickness of the boundary layer, or 2) make the boundary layer stiffer. See the [Simulation section of the Klamp't Manual](Manual-Simulation.md) for more details on how to implement these fixes.


## The simulator runs slowly. How can I make it faster?

Unless you are simulating a huge number of joints, the limiting steps in simulation are usually contact detection and calculating the contact response.

The speed of contact detection is governed by the resolution of the meshes in contact. Simpler meshes will lead to faster contact detection. Most 3D modeling packages will provide mesh simplification operators.

The speed of contact response is governed by the number of contact points retained in the contact handling procedure after clustering. The maxContacts simulation parameter governs the number of clusters and can be reduced to achieve a faster simulation. However, setting this value too low will lead to a loss of physical realism.


## How do I implement a behavior script?

Many engineers and students tend to approach robotics from a &quot;scripting&quot; approach, whereby a complex behavior is broken down into a script or state machine of painstakingly hand-tuned, heuristic behaviors. Unlike some other packages, Klamp't does not try to make scripting convenient. This choice was made deliberately in order to discourage the use of heuristic behaviors. The philosophy is that _hand-tuned behaviors should be rare in intelligent robots_. However, it is true that scripts / state machines are sometimes the easiest way to accomplish a given behavior with the current generation of robot AI tools.

To implement a behavior script in Klamp't, the script should be launched in a separate thread from the execution thread. It can then monitor the state of the execution thread (e.g., waiting for a movement to finish) and react accordingly. For those new to threading, please see the C++ `<thread>` module or the Python threading module for more information.

To implement a state machine, a controller should manually maintain and simulate its behavior in its feedback loop.

