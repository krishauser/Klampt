# Klamp't Tutorial: Display ROS point cloud messages using Python
In this tutorial, we will learn how to connect the  [Robot Operating System](http://www.ros.org/)  (ROS) to Klamp't. It covers logging, C++ bindings, and subscribing to point clouds in Python.

Note that this tutorial assumes you are familiar with ROS. In addition, you will need to build Klamp't after installing ROS.

Difficulty: intermediate

Time: 10-30 minutes

Currently, the Python interface only provides a method to subscribe to point clouds.

### Subscribing to Point Clouds

Using  [Geometry3D](http://motion.pratt.duke.edu/klampt/pyklampt_docs/classklampt_1_1robotsim_1_1Geometry3D.html), you can subscribe to a ROS topic containing a  [PointCloud](http://motion.pratt.duke.edu/klampt/pyklampt_docs/classklampt_1_1robotsim_1_1PointCloud.html). This is accomplished via Geometry3D's  _attachToStream_  method, which takes as arguments the protocol (currently only "ros" protocol is supported) and the name of the ROS topic to subscribe to. For an example, create a new file called "pointCloudFromROS.py" and copy the following lines:
```
from klampt import PointCloud, Geometry3D, Appearance

#Create point cloud subscriber
topic = "myROSTopic"  #ROS topic containing actual point cloud
pc = PointCloud()     #point cloud listener
g = Geometry3D(pc)    #3d geometry from point cloud

#Subscribe to topic
if g.attachToStream("ros",topic):       #subscribe to myROSTopic
    print "Subscribed!"
else:
    print "Could not subscribe to", topic
Appearance().refresh(True)              #update appearance

#Unsubscribe from topic
#g.detachFromStream("ros",topic)        #unsubscribe from myROSTopic
```
Now run the script via
```
python pointCloudFromROS.py
```
Congratulations, you have subscribed to your first point cloud!
Note that you need to call  [Appearance](http://motion.pratt.duke.edu/klampt/pyklampt_docs/classklampt_1_1robotsim_1_1Appearance.html)_().refresh(True)_  to get the appearance to update. Furthermore,  _detachFromStream_  must be called before deleting the geometry.
Please refer to Klampt/Python/demos/resources/pointcloudtest.py for more information.