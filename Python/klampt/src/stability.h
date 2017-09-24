#ifndef _STABILITY_H
#define _STABILITY_H

class RobotModel;

/// Globally sets the number of edges used in the friction cone approximation.
/// The default value is 4.
void setFrictionConeApproximationEdges(int numEdges);

/// Returns true if the list of contact points has force closure.  A contact point
/// is given by a list of 7 floats, [x,y,z,nx,ny,nz,k] where (x,y,z) is the position,
/// (nx,ny,nz) is the normal, and k is the coefficient of friction (>= 0)
bool forceClosure(const std::vector<std::vector<double > >& contacts);

/// A "fancy" version of the forceClosure test.  contactPositions is a list of
/// 3-lists giving the contact point positions.  The i'th element in the list frictionCones
/// has length (k*4), and gives the contact force constraints (ax,ay,az,b) where
/// ax*fx+ay*fy+az*fz <= b limits the contact force (fx,fy,fz) at the i'th contact. 
/// Each of the k 4-tuples is laid out sequentially per-contact.
bool forceClosure(const std::vector<std::vector<double > >& contactPositions,const std::vector<std::vector<double > >& frictionCones);

/// Returns true if the list of 2D contact points has force closure.  A contact point
/// is given by a list of 4 floats, [x,y,theta,k] where (x,y) is the position,
/// theta is the normal angle, and k is the coefficient of friction (>= 0)
bool forceClosure2D(const std::vector<std::vector<double > >& contacts);

/// A "fancy" version of the forceClosure2D test.  contactPositions is a list of
/// 2-lists giving the contact point positions.  The i'th element in the list frictionCones
/// has length (k*3), and gives the contact force constraints (ax,ay,b) where
/// ax*fx+ay*fy <= b limits the contact force (fx,fy) at the i'th contact. 
/// Each of the k 3-tuples is laid out sequentially per-contact.
bool forceClosure2D(const std::vector<std::vector<double > >& contactPositions,const std::vector<std::vector<double> >& frictionCones);

/// Tests whether the given COM com is stable for the given contacts and the given
/// external force fext.  A contact point is given by a list of 7 floats,
/// [x,y,z,nx,ny,nz,k] as usual.
///
/// The return value is either None, or a list of 3-tuples giving the support forces
/// at the contacts.
/// 
/// com can also be set to None in which case this tests if ANY COM has
/// at the contacts.  The return value is True or False.
PyObject* comEquilibrium(const std::vector<std::vector<double> >& contacts,const std::vector<double>& fext,PyObject* com);

/// A fancy version of the normal comEquilibrium test.  contactPositions is a list of
/// 3-lists giving the contact point positions.  The i'th element in the list frictionCones
/// has length (k*4), and gives the contact force constraints (ax,ay,az,b) where
/// ax*fx+ay*fy+az*fz <= b limits the contact force (fx,fy,fz) at the i'th contact. 
/// Each of the k 4-tuples is laid out sequentially per-contact.
///
/// The return value is either None, or a list of 3-tuples giving the support forces
/// at the contacts.
/// 
/// com can also be set to None in which case this tests if ANY COM has
/// at the contacts.  The return value is True or False.
PyObject* comEquilibrium(const std::vector<std::vector<double> >& contactPositions,const std::vector<std::vector<double> >& frictionCones,const std::vector<double>& fext,PyObject* com);


/// Tests whether the given COM com is stable for the given contacts and the given
/// external force fext.  A contact point is given by a list of 4 floats,
/// [x,y,theta,k] as usual.
///
/// The return value is either None, or a list of 2-tuples giving the support forces
/// at the contacts.
/// 
/// com can also be set to None in which case this tests if ANY COM has
/// at the contacts.  The return value is True or False.
PyObject* comEquilibrium2D(const std::vector<std::vector<double> >& contacts,const std::vector<double>& fext,PyObject* com);

/// A fancy version of the normal comEquilibrium2D test.  contactPositions is a list of
/// 2-lists giving the contact point positions.  The i'th element in the list frictionCones
/// has length (k*3), and gives the contact force constraints (ax,ay,b) where
/// ax*fx+ay*fy <= b limits the contact force (fx,fy) at the i'th contact. 
/// Each of the k 3-tuples is laid out sequentially per-contact.
///
/// The return value is either None, or a list of 2-tuples giving the support forces
/// at the contacts.
/// 
/// com can also be set to None in which case this tests if ANY COM has
/// at the contacts.  The return value is True or False.
PyObject* comEquilibrium2D(const std::vector<std::vector<double> >& contactPositions,const std::vector<std::vector<double> >& frictionCones,const std::vector<double>& fext,PyObject* com);


/// Calculates the support polygon for a given set of contacts and a downward external force (0,0,-g).
/// A contact point is given by a list of 7 floats, [x,y,z,nx,ny,nz,k] as usual.
/// 
/// The return value is a list of 3-tuples giving the sorted plane boundaries of the polygon.
/// The format of a plane is (nx,ny,ofs) where (nx,ny) are the outward facing normals, and
/// ofs is the offset from 0.  In other words to test stability of a com [x,y], you can test
/// whether dot([nx,ny],[x,y]) <= ofs  for all planes.
PyObject* supportPolygon(const std::vector<std::vector<double> >& contacts);

/// A fancy version of the normal supportPolygon test.
/// contactPositions is a list of 3-lists giving the contact point positions. 
/// The i'th element in the list frictionCones has length (k*4), and gives the contact
/// force constraints (ax,ay,az,b) where ax*fx+ay*fy+az*fz <= b limits the contact force
/// (fx,fy,fz) at the i'th contact.  Each of the k 4-tuples is laid out sequentially per-contact.
/// 
/// The return value is a list of 3-tuples giving the sorted plane boundaries of the polygon.
/// The format of a plane is (nx,ny,ofs) where (nx,ny) are the outward facing normals, and
/// ofs is the offset from 0.  In other words to test stability of a com [x,y], you can test
/// whether dot([nx,ny],[x,y]) <= ofs  for all planes.
PyObject* supportPolygon(const std::vector<std::vector<double> >& contactPositions,const std::vector<std::vector<double> >& frictionCones);


/// Calculates the support polygon (interval)  for a given set of contacts and a downward
/// external force (0,-g). A contact point is given by a list of 4 floats, [x,y,theta,k] as usual.
/// 
/// The return value is a 2-tuple giving the min / max extents of the support polygon.
/// If they are both infinite, the support polygon is empty.
PyObject* supportPolygon2D(const std::vector<std::vector<double> >& contacts);

/// A fancy version of the normal supportPolygon2D test.  contactPositions is a list of
/// 2-lists giving the contact point positions.  The i'th element in the list frictionCones
/// has length (k*3), and gives the contact force constraints (ax,ay,b) where
/// ax*fx+ay*fy <= b limits the contact force (fx,fy) at the i'th contact. 
/// Each of the k 3-tuples is laid out sequentially per-contact.
/// 
/// The return value is a 2-tuple giving the min / max extents of the support polygon.
/// If they are both infinite, the support polygon is empty.
PyObject* supportPolygon2D(const std::vector<std::vector<double> >& contacts,const std::vector<std::vector<double> >& frictionCones);


/// Solves for the torques / forces that keep the robot balanced against gravity
/// 
/// Arguments
/// - robot: the robot model, posed in its current configuration
/// - contacts: a list of contact points, given as 7-lists [x,y,z,nx,ny,nz,kFriction]
/// - links: a list of the links on which those contact points lie
/// - fext: the external force (e.g., gravity)
/// - norm: the torque norm to minimize.  If 0, minimizes the l-infinity norm (default)
///         If 1, minimizes the l-1 norm.  If 2, minimizes the l-2 norm (experimental,
///          may not get good results)
///
/// Return value is a pair (t,f) giving the joint torques and frictional
/// contact forces, if a solution exists, or None if no solution exists.
PyObject* equilibriumTorques(const RobotModel& robot,
							const std::vector<std::vector<double> >& contacts,const std::vector<int>& links,
							const std::vector<double>& fext,
							double norm=0);
/// Same as the above equilibriumTorques, but accepts another argument
/// internalTorques.
///
/// internalTorques is a list of length robot.numLinks(), and allows you
/// to solve for dynamic situations, e.g., with coriolis forces taken
/// into account.  To do so, set the robot's joint velocities dq, calculate
/// then calculate the torques via robot.torquesFromAccel(ddq), and pass
/// the result into internalTorques.
PyObject* equilibriumTorques(const RobotModel& robot,
							const std::vector<std::vector<double> >& contacts,const std::vector<int>& links,
							const std::vector<double>& fext,
							const std::vector<double>& internalTorques,
							double norm=0);


#endif // _STABILITY_H