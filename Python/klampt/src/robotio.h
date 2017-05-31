#ifndef _ROBOTSIM_IO_H
#define _ROBOTSIM_IO_H

#include <string>

class Geometry3D;
class WorldModel;

/** @brief Subscribes a Geometry to a stream 
 * 
 * Arguments
 * - protocol: only "ros" accepted for now.
 * - name: the name of the stream. E.g., ROS topic.
 * - type: If provided, specifies the format of the data to be subscribed to. If not, tries to 
 *   determine this automatically.
 *
 * Only ROS point clouds are supported for now.
*/
bool SubscribeToStream(Geometry3D& g,const char* protocol,const char* name,const char* type="");

/** @brief Unsubscribes from a stream
 */
bool DetachFromStream(const char* protocol,const char* name);

/** @brief Does some processing on stream subscriptions.
 * 
 * Arguments:
 * - protocol: either name the protocol to be updated, or "all" for updating all
 *   subscribed streams
 * 
 * Returns true if any stream was updated.
 */
bool ProcessStreams(const char* protocol="all");

/** @brief Waits up to timeout seconds for an update on the given stream
 * Returns true if the stream was updated.
 */
bool WaitForStream(const char* protocol,const char* name,double timeout);

///Exports the WorldModel to a JSON string ready for use in Three.js
std::string ThreeJSGetScene(const WorldModel&);

///Exports the WorldModel to a JSON string ready for use in Three.js
std::string ThreeJSGetTransforms(const WorldModel&);



#endif //_ROBOTSIM_IO_H