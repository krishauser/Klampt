#ifndef IO_ROS_H
#define IO_ROS_H

#include <vector>
#include <math3d/primitives.h>
//forward declarations
namespace Meshing { class PointCloud3D; }
class RobotWorld;
class Robot;
class ControlledRobotSimulator;
class WorldSimulation;
class LinearPath;


///Must call this before all other ROS[X] calls. An optional node name can
///be provided
bool ROSInit(const char* nodename="klampt");
///Must call this after all other ROS[X] calls to cleanly shut down ROS
bool ROSShutdown();
///Updates subscribed topics.  Returns true if any topic was updated.
///Note: if only "tf" is updated, this will still return false...
bool ROSSubscribeUpdate();
///Deletes topics.  To remove the subscribed TF topics, call
bool ROSDetach(const char* topic);
///Returns the number of subscribed topics
int ROSNumSubscribedTopics();
///Returns the number of advertised topics
int ROSNumPublishedTopics();
///Returns true if anyone is sending data to a topic Klampt is subscribing to
///if anyone is listening to a topic Klamp't is publishing to
bool ROSIsConnected(const char* topic);
///Returns the ROS frame ID that the data is attached to, if any.
std::string ROSFrame(const char* topic);
///Blocks until the subscriber on the given topic receives a new update. 
///Returns true if an update was found.
///If this is not subscribed to, it returns false immediately.
///Note: does not work for the "tf" topic yet.
bool ROSWaitForUpdate(const char* topic,double timeout=0);
///Sets the global queue size
bool ROSSetQueueSize(int size);

bool ROSPublishPose(const Math3D::RigidTransform& T,const char* topic="klampt/transform");
bool ROSPublishJointState(const Robot& robot,const char* topic="klampt/joint_state");
bool ROSPublishPointCloud(const Meshing::PointCloud3D& pc,const char* topic="klampt/point_cloud");
///publishes a Trajectory message with default joint names
bool ROSPublishTrajectory(const LinearPath& path,const char* topic="klampt/trajectory");
///publishes a Trajectory with the robot's links as joint names
bool ROSPublishTrajectory(const Robot& robot,const LinearPath& path,const char* topic="klampt/trajectory");
///publishes a Trajectory along the specified robot indices
bool ROSPublishTrajectory(const Robot& robot,const std::vector<int>& indices,const LinearPath& path,const char* topic="klampt/trajectory");
///publishes a JointState about the robot's current commanded joint state
bool ROSPublishCommandedJointState(ControlledRobotSimulator& robot,const char* topic="klampt/joint_state_commanded");
///publishes a JointState about the robot's current sensed joint state
bool ROSPublishSensedJointState(ControlledRobotSimulator& robot,const char* topic="klampt/joint_state_sensed");

///Subscribes to Pose updates from the given topic.  Note: the T
///object must not be destroyed while ROSSubscribeUpdate is being called.
///If you want to detach it from  future updates, call RosDetach([topic]);
bool ROSSubscribePose(Math3D::RigidTransform& T,const char* topic="klampt/joint_state");
///Subscribes to JointState updates from the given topic.  Note: the robot
///object must not be destroyed while ROSSubscribeUpdate is being called.
///If you want to detach it from  future updates, call RosDetach([topic]);
bool ROSSubscribeJointState(Robot& robot,const char* topic="klampt/joint_state");
///Subscribes to PointCloud2 updates from the given topic.  Note: the
///object must not be destroyed while ROSSubscribeUpdate is being called.
///If you want to detach it from  future updates, call RosDetach([topic]);
bool ROSSubscribePointCloud(Meshing::PointCloud3D& pc,const char* topic="klampt/point_cloud");
///Subscribes to Trajectory updates from the given topic.  Note: the 
///object must not be destroyed while ROSSubscribeUpdate is being called.
///If you want to detach it from  future updates, call RosDetach([topic]);
bool ROSSubscribeTrajectory(LinearPath& path,const char* topic="klampt/trajectory");
///Subscribes to Trajectory updates from the given topic, with the given
///robot as reference (Trajectories can specify a subset of robot joints).
///Note: the robot and path must not be destroyed while ROSSubscribeUpdate is being called.
///If you want to detach it from  future updates, call RosDetach([topic]);
bool ROSSubscribeTrajectory(Robot& robot,LinearPath& path,const char* topic="klampt/trajectory");

///Publishes the world to the transform server, under frames named by
///[frameprefix]/[rigid object name] for rigid objects and
///[frameprefix]/[robot name]/[link name] for robot links
bool ROSPublishTransforms(const RobotWorld& world,const char* frameprefix="klampt");
///Publishes the simulation to the transform server, under frames named by
///[frameprefix]/[rigid object name] for rigid objects and
///[frameprefix]/[robot name]/[link name] for robot links
bool ROSPublishTransforms(const WorldSimulation& sim,const char* frameprefix="klampt");
///Publishes the robot link transforms to the transform server under frames
///named by [frameprefix]/[link name]
bool ROSPublishTransforms(const Robot& robot,const char* frameprefix="klampt");
///Publishes the transform to the transform server under the given name.
bool ROSPublishTransform(const Math3D::RigidTransform& T,const char* frame="klampt_transform");

///Subscribes to world updates from the transform server.  Note: the world
///object must not be destroyed while ROSSubscribeUpdate is being called.
///If you want to detach it from  future updates, call RosDetach("tf");
///Note: does NOT set robot configurations.
bool ROSSubscribeTransforms(RobotWorld& world,const char* frameprefix="klampt");
///Subscribes to robot updates from the transform server.  Note: the robot
///object must not be destroyed while ROSSubscribeUpdate is being called.
///If you want to detach it from  future updates, call RosDetach("tf");
///Note: does NOT set robot configurations.
bool ROSSubscribeTransforms(Robot& robot,const char* frameprefix="klampt");
///Subscribes to transform updates from the transform server.  Note: the T
///object must not be destroyed while ROSSubscribeUpdate is being called.
///If you want to detach it from  future updates, call RosDetach("tf");
bool ROSSubscribeTransform(Math3D::RigidTransform& T,const char* frameprefix="klampt_transform_state");


#endif
