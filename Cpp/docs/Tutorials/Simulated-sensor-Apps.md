# Klamp't Tutorial: Set up a simulated camera sensor and save frames to disk using Apps

In this tutorial we will learn how to use the sensor simulation capabilities in the Klamp't framework for each of the supported sensor types: visual, tactile, and inertial.

Difficulty: moderate

Time: 30 minutes

### Sensor Overview
Currently Klamp't suppports the following sensors:
-   Joint Sensors
    -   Position
    -   Velocity
    -   Torque
-   Inertial Sensors
    -   Accelerometer
    -   Tilt sensor
    -   Gyroscope
    -   Inertial Measurement Unit (IMU)
-   Visual Sensors
    -   RGB camera
    -   Depth camera
    -   Laser rangefinder
-   Tactile Sensors
    -   Contact sensor
    -   Force/torque sensor
These sensors can be used in the simulation environment to monitor the robot's state and provide feedback for your robot's controller. Each sensor has a set of properties that describe its location on the robot (or in the world) and govern its behavior, e.g. resolution, noise level, update rate, etc. In the case of camera sensor these parameters include the field of view and resolution.

To use sensors in a simulation, they must first be defined in a simulation world file, after which api functions can be called to access their measurements and visualize them

### Sensor definition
Sensors are defined in a world file, just like terrain, robots, and other rigid geometry. Sensors are added under the simulation heading, and must be associated with a robot. When adding a sensor, it must be identified by its class name as defined in the Klamp't c++ API. The currently supported sensor types are as follows: JointPositionSensor, JointVelocitySensor, Accelerometer, Gyroscope, IMUSensor, ForceTorqueSensor, ContactSensor, CameraSensor, and LaserRangeSensor.

You may set the properties of sensors using attributes in their XML tags. An excerpt from a world definition file is shown below to demonstrate the structure. 

### SimTest App
Sensors can be visualized in the SimTest app, providing a convenient way to test their behavior without writing a custom simulation script. Assuming you have defined your sensors correctly in the world file, they can be accessed in the SimTest under Windows->Sensor Plot, or by pressing Ctrl+P. Enabling the Sensor Drawing Options window will produce the screen below.
```
<world>
    <terrain file="/home/shihao/Klampt/data/terrains/plane.off" translation="0 0 0"/>
    <robot name="tx90" file="/home/shihao/Klampt/data/robots/tx90ball.rob">
        <sensors>
            <JointPositionSensor name="encoders"/>
            <JointVelocitySensor name="dencoders"/>
            <!-- <ContactSensor name="contact" link="6" Tsensor="1 0 0 0 1 0 0 0 1 0 0 0.03" patchMin="-0.01 -0.01" patchMax="0.01 0.01" patchTolerance="0.005" hasForce="0 0 1"/>
            <ForceTorqueSensor name="f/t" link="6" hasForce="1 1 1" hasTorque="1 1 1"/>
            <Accelerometer name="accelerometer" link="6" hasAxis="1 1 1"/>
            <IMUSensor name="imu" link="6" hasAxis="1 1 1" hasAngAccel="1" hasAngVel="1"/>
            <LaserRangeSensor name="lidar" link="6" Tsensor="0 1 0 -1 0 0 0 0 1 0 0 0" depthMaximum="4.0" depthMinimum="0.1" depthResolution="0.01" depthVarianceLinear="0.0001"/> -->
            <CameraSensor name="rgbd_camera" link="6" Tsensor="0 1 0 -1 0 0 0 0 1 0 0.1 0" xres="256" yres="128" xfov="1.05" yfov="0.6" zmin="0.4" zresolution="0.01" zvarianceLinear="0.00001"/>
        </sensors>
    </robot>
    <rigidObject name="sphere" position="2 0 1.00000">
        <geometry mesh="/home/shihao/Klampt/data/objects/sphere.geom" scale="0.1"/>
        <physics mass="1.000000" automass="1" kRestitution="1" kFriction="0.500000" kStiffness="inf" kDamping="inf"/>
    </rigidObject>
    <simulation>
        <terrain index="0">
            <geometry kRestitution="0.500000" kFriction="0.500000" kStiffness="inf" kDamping="inf" padding="0.001" preshink="1"/>
        </terrain>
    </simulation>
</world>
```
Open this XML file with SimTest and select the _Sensor Plot_ under _Windows_ menu, we can find the simulated rgbd_camera is recording the data for this simulation environment where at each frame a rectangle area of 256X128 pixels is recorded. 
 <p align="center">
<img src="https://raw.githubusercontent.com/ShihaoWang/Figures/master/simulated%20camera1.JPG"
width="75%" height="75%">
</p>
 <p align="center">
<img src="https://raw.githubusercontent.com/ShihaoWang/Figures/master/simulated%20camera2.JPG"
width="75%" height="75%">
</p>

### Frame saving

SimTest can log in the simulated data through the _Logging_ menu so using this method
 <p align="center">
<img src="https://raw.githubusercontent.com/ShihaoWang/Figures/master/simulated%20camera3.JPG"
width="75%" height="75%">
</p>

After using this _Logging_, the whole simulated data are saved into the local disk and then the camera data can be retrieved from the saved file for further analysis. 
 <p align="center">
<img src="https://raw.githubusercontent.com/ShihaoWang/Figures/master/simulated%20camera4.JPG"
width="75%" height="75%">
</p>