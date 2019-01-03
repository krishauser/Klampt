# Klamp't Tutorial: Process clicks on the robot or world in C++

In this tutorial we learn how to process clicks on the robot or world in the simulation environment with C++. Interactive feedback of the mouse clicks is useful for the determination of the object being clicked and furthermore the click point's x-y position. 

Difficulty: easy

Time: 10 minutes

TODO

The _WorldViewWidget_ class in **Main/WorldProgram.h** provides the _Hover_ method to determine the closest object and robot when clicked via the mouse’s x-y position. This must be provided the current OpenGL viewport such as the _viewport_ member of the _GLScreenshotProgram_ or _GLUINavigationProgram_ classes. 