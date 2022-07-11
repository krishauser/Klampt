#ifndef CONTACT_UTILS_H
#define CONTACT_UTILS_H

#include <KrisLibrary/robotics/Contact.h>
#include <KrisLibrary/robotics/RobotWithGeometry.h>
#include "Stance.h"

namespace Klampt {

class WorldModel;

/** @addtogroup Contact */
/** @{ */

/** @brief Produces a list of contacts as though the robot were standing on a plane.
 * 
 * tol is the tolerance with which minimum-distance points are generated.
 * All contacts are given zero friction and in the local frame of the robot's
 * links.
 */
void GetFlatContacts(RobotWithGeometry& robot,Real tol,ContactFormation& contacts);


/** @brief Produces a list of contacts as though the link were resting on a plane.
 * 
 * tol is the tolerance with which minimum-distance points are generated.
 * All contacts are given zero friction and in the local frame of the link.
 */
void GetFlatContacts(RobotWithGeometry& robot,int link,Real tol,vector<ContactPoint>& contacts);


/** @brief Produces a stance as though the robot were standing on a plane.
 * 
 * tol is the tolerance with which minimum-distance points are generated.
 * All contacts are given friction kFriction.
 */
void GetFlatStance(RobotWithGeometry& robot,Real tol,Stance& s,Real kFriction=0);


/** @brief Produces a list of contacts for all points on the robot within tol of the
 * other objects in world.
 * 
 * tol is the tolerance with which minimum-distance points are generated.
 * All contacts are given zero friction and in the local frame of the robot's
 * links.
 */
void GetNearbyContacts(RobotWithGeometry& robot,WorldModel& world,Real tol,ContactFormation& contacts);


/** @brief Produces a list of contacts for all points on the link within tol of
 * other objects in world.
 * 
 * tol is the tolerance with which minimum-distance points are generated.
 * All contacts are given zero friction and in the local frame of the link.
 */
void GetNearbyContacts(RobotWithGeometry& robot,int link,WorldModel& world,Real tol,vector<ContactPoint>& contacts);


/** @brief For a set of local contacts on a link, returns a hold for the given robot.
 * Assumes the robot is making those contacts in its current config.
 */
void LocalContactsToHold(const vector<ContactPoint>& contacts,int link,const RobotKinematics3D& robot,Hold& hold);

/** @brief For a local contact formation, returns a stance for the given robot.
 * Assumes the robot is making those contacts in its current config.
 */
void LocalContactsToStance(const ContactFormation& contacts,const RobotKinematics3D& robot,Stance& stance);

/** @brief Reduces a complex set of contacts to a set of representative
 * clusters through kmeans clustering.
 */
void ClusterContacts(vector<ContactPoint>& cps,int numClusters,Real clusterNormalScale=0.1,Real clusterFrictionScale=0.1);

/** @brief Merges contact points within tol distance of each other
 */
void CleanupContacts(vector<ContactPoint>& cp,Real tol);

/** @brief Removes contact points in the convex hull interior of other cp's
 */
void CHContacts(vector<ContactPoint>& cp,Real ntol,Real xtol);

/** @brief Finds the closest point/normal on the mesh to p.  Metric distance
 * is sqrt(||p.x - x||^2 + normalScale*||p.n - n||^2) where x and n are the 
 * points on the mesh.  Returns the triangle.
 */
int ClosestContact(const ContactPoint& p,const Meshing::TriMesh& mesh,ContactPoint& closest,Real normalScale=0.1);

/** @} */

} //namespace Klampt

#endif
