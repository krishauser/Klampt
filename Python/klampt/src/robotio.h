#ifndef _ROBOTSIM_IO_H
#define _ROBOTSIM_IO_H

#include <string>

class Geometry3D;
class WorldModel;

/** @file robotio.h 
 * Miscelleaneous IO routines for ROS and Three.js export via the C++ backend.
 *
 * TODO: It has not yet been determined whether this interferes with Rospy.
 */

/** @brief Subscribes a Geometry3D to a stream.
 * 
 * Args:
 * 
 *     g (Geometry3D): the geometry that will be updated
 *     protocol (str): only "ros" accepted for now.
 *     name (str): the name of the stream, i.e., ROS topic.
 *     type (str, optional): If provided, specifies the format of the data
 *         to be subscribed to. If not, tries to determine the type
 *         automatically.
 *
 * Only ROS point clouds (PointCloud2) are supported for now.
 * Note that you can also call ``Geometry3D.loadFile("ros://[ROS_TOPIC]")``
 * or ``Geometry3D.loadFile("ros:PointCloud2//[ROS_TOPIC]")``
 * to accomplish the same thing.
 *
 * TODO: It has not yet been determined whether this interferes with Rospy,
 * i.e., klampt.io.ros.
 *
 * Returns:
 *     (bool): True if successful.
 */
bool subscribe_to_stream(Geometry3D& g,const char* protocol,const char* name,const char* type="");

/** @brief Unsubscribes from a stream previously subscribed to via
 * :func:`SubscribeToStream`
 */
bool detach_from_stream(const char* protocol,const char* name);

/** @brief Does some processing on stream subscriptions.
 * 
 * Args:
 * 
 *     protocol (str): either name the protocol to be updated, or "all" for
 *         updating all subscribed streams
 * 
 * Returns:
 *     (bool): True if any stream was updated.
 */
bool process_streams(const char* protocol="all");

/** @brief Waits up to timeout seconds for an update on the given stream
 * 
 * Return:
 * 
 *     (bool): True if the stream was updated.
 */
bool wait_for_stream(const char* protocol,const char* name,double timeout);

///Exports the WorldModel to a JSON string ready for use in Three.js
std::string threejs_get_scene(const WorldModel&);

///Exports the WorldModel to a JSON string ready for use in Three.js
std::string threejs_get_transforms(const WorldModel&);


#endif //_ROBOTSIM_IO_H