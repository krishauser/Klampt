#ifndef _STABILITY_H
#define _STABILITY_H

class RobotModel;

/** @file stability.h 
 * Miscelleaneous stability testing routines
 */

/// Globally sets the number of edges used in the friction cone approximation.
/// The default value is 4.
void set_friction_cone_approximation_edges(int numEdges);

/// Returns true if the array of contact points has force closure.  A contact point
/// is given by a list of n=7 floats, [x,y,z,nx,ny,nz,k] where (x,y,z) is the position,
/// (nx,ny,nz) is the normal, and k is the coefficient of friction (>= 0)
bool force_closure(double* contacts,int m,int n);

//note: only the last overload docstring is added to the documentation
/** @brief Returns true if the list of contact points has force closure. 
 * 
 * In the 1-argument version, each contact point is specified by a list of
 * 7 floats, [x,y,z,nx,ny,nz,k] where (x,y,z) is the position,
 * (nx,ny,nz) is the normal, and k is the coefficient of friction.
 *
 * The 2-argument version is a "fancy" version that allows more control over the
 * constraint planes.
 * 
 * Args:
 * 
 *      contacts (list of 7-float lists or tuples): the list of contacts, each
 *          specified as a 7-list or tuple [x,y,z,nx,ny,nz,k], with:
 * 
 *              * (x,y,z): the contact position
 *              * (nx,ny,nz): the contact normal
 *              * k: the coefficient of friction (>= 0)
 * 
 *      contactPositions (list of 3-float lists or tuples): the list of contact
 *          point positions. 
 *      frictionCones (list of lists): Each item of this list specifies linear
 *          inequalities that must be met of the force at the corresponding
 *          contact point.  The item must have length k*4 where k is an integer,
 *          and each inequality gives the entries (ax,ay,az,b) of a constraint
 *          ax*fx+ay*fy+az*fz <= b that limits the contact force (fx,fy,fz) at
 *          the i'th contact.  Each of the k 4-tuples is laid out sequentially
 *          per-contact.
 */
bool force_closure(const std::vector<std::vector<double > >& contactPositions,const std::vector<std::vector<double > >& frictionCones);

/// Returns true if the list of 2D contact points has force closure.  A contact point
/// is given by a list of n=4 floats, [x,y,theta,k] where (x,y) is the position,
/// theta is the normal angle, and k is the coefficient of friction (>= 0)
bool force_closure_2d(double* contacts,int m,int n);

//note: only the last overload docstring is added to the documentation
/** @brief Returns true if the list of 2D contact points has force closure. 
 * 
 *  In the 1-argument version, each contact point is given by a list of 4 floats,
 * [x,y,theta,k] where (x,y) is the position, theta is the normal angle, and k
 * is the coefficient of friction 
 * 
 * The 2-argument version is a "fancy" version that allows more control over the
 * constraint planes.
 * 
 * Args:
 * 
 *      contacts (list of 4-float lists or tuples): the list of contacts, each
 *          specified as a 4-list or tuple [x,y,theta,k], with:
 * 
 *              * (x,y): the contact position
 *              * theta: is the normal angle (in radians, CCW to the x axis)
 *              * k: the coefficient of friction (>= 0)
 * 
 *      contactPositions (list of 2-float lists or tuples): the list of contact
 *          point positions. 
 *      frictionCones (list of lists): The i'th element in this list has length
 *          k*3 (for some integer k), and gives the contact force constraints
 *          (ax,ay,b) where ax*fx+ay*fy <= b limits the contact force (fx,fy)
 *          at the i'th contact. Each of the k 3-tuples is laid out sequentially
 *          per-contact.
 */
bool force_closure_2d(const std::vector<std::vector<double > >& contactPositions,const std::vector<std::vector<double> >& frictionCones);

/// Tests whether the given COM com is stable for the given contacts and the given
/// external force fext.  A contact point is given by a list of 7 floats,
/// [x,y,z,nx,ny,nz,k] as usual.
///
/// The return value is either None, or a list of 3-tuples giving the support forces
/// at the contacts.
/// 
/// com can also be set to None in which case this tests if ANY COM has
/// at the contacts.  The return value is True or False.
PyObject* com_equilibrium(double* contacts,int m,int n,const std::vector<double>& fext,PyObject* com);

//note: only the last overload docstring is added to the documentation
/** @brief Tests whether the given COM com is stable for the given contacts and the given
 * external force fext.  
 *
 * The 2-argument version is a "fancy" version that allows more control over the
 * constraint planes.
 *
 * Args:
 *      contacts (list of 7-float lists or tuples): the list of contacts, each
 *          specified as a 7-list or tuple [x,y,z,nx,ny,nz,k], with:
 * 
 *              * (x,y,z): the contact position
 *              * (nx,ny,nz): the contact normal
 *              * k: the coefficient of friction (>= 0)
 * 
 *      contactPositions (list of 3-float lists or tuples): the list of contact
 *          point positions. 
 *      frictionCones (list of lists): Each item of this list specifies linear
 *          inequalities that must be met of the force at the corresponding
 *          contact point.  The item must have length k*4 where k is an integer,
 *          and each inequality gives the entries (ax,ay,az,b) of a constraint
 *          ax*fx+ay*fy+az*fz <= b that limits the contact force (fx,fy,fz) at
 *          the i'th contact.  Each of the k 4-tuples is laid out sequentially
 *          per-contact.
 *      fext (3-tuple or list): the external force vector.
 *      com (3-tuple or list, or None): the center of mass coordinates.  If
 *          None, assumes that you want to test whether ANY COM may be in
 *          equilibrium for the given contacts.
 *
 * Returns:
 * 
 *     bool, None, or list: if com is given, and there are feasible
 *     equilibrium forces, this returns a list of 3 tuples giving
 *     equilibrium forces at each of the contacts. None is returned if
 *     no such forces exist.  
 *
 *     If com = None, the result is True or False.
 *
 */
PyObject* com_equilibrium(const std::vector<std::vector<double> >& contactPositions,const std::vector<std::vector<double> >& frictionCones,const std::vector<double>& fext,PyObject* com);


/// Tests whether the given COM com is stable for the given contacts and the given
/// external force fext.  A contact point is given by a list of 4 floats,
/// [x,y,theta,k] as usual.
///
/// The return value is either None, or a list of 2-tuples giving the support forces
/// at the contacts.
/// 
/// com can also be set to None in which case this tests if ANY COM has
/// at the contacts.  The return value is True or False.
PyObject* com_equilibrium_2d(double* contacts,int m,int n,const std::vector<double>& fext,PyObject* com);

//note: only the last overload docstring is added to the documentation
/** @brief Tests whether the given COM com is stable for the given contacts and the given
 * external force fext.  
 *
 * The 2-argument version is a "fancy" version that allows more control over the
 * constraint planes.
 *
 * Args:
 * 
 *      contacts (list of 4-float lists or tuples): the list of contacts, each
 *          specified as a 4-list or tuple [x,y,theta,k], with:
 * 
 *              * (x,y,z): the contact position
 *              * theta: is the normal angle (in radians, CCW to the x axis)
 *              * k: the coefficient of friction (>= 0)
 * 
 *      contactPositions (list of 2-float lists or tuples): the list of contact
 *          point positions. 
 *      frictionCones (list of lists): The i'th element in this list has length
 *          k*3 (for some integer k), and gives the contact force constraints
 *          (ax,ay,b) where ax*fx+ay*fy <= b limits the contact force (fx,fy)
 *          at the i'th contact. Each of the k 3-tuples is laid out sequentially
 *          per-contact.
 *      fext (2-tuple or list): the external force vector.
 *      com (2-tuple or list, or None): the center of mass coordinates.  If None,
 *          assumes that you want to test whether ANY COM may be in equilibrium
 *          for the given contacts.
 *
 * Returns:
 * 
 *     bool, None, or list: if com is given, and there are feasible
 *     equilibrium forces, this returns a list of 2-tuples giving
 *     equilibrium forces at each of the contacts. None is returned if
 *     no such forces exist. 
 *
 *     If com = None, the result is True or False.
 *
 */
PyObject* com_equilibrium_2d(const std::vector<std::vector<double> >& contactPositions,const std::vector<std::vector<double> >& frictionCones,const std::vector<double>& fext,PyObject* com);


/// Calculates the support polygon for a given set of contacts and a downward external force (0,0,-g).
/// A contact point is given by a list of 7 floats, [x,y,z,nx,ny,nz,k] as usual.
/// 
/// The return value is a list of 3-tuples giving the sorted plane boundaries of the polygon.
/// The format of a plane is (nx,ny,ofs) where (nx,ny) are the outward facing normals, and
/// ofs is the offset from 0.  In other words to test stability of a com [x,y], you can test
/// whether dot([nx,ny],[x,y]) <= ofs  for all planes.
PyObject* support_polygon(double* contacts,int m,int n);

//note: only the last overload docstring is added to the documentation
/** @brief Calculates the support polygon for a given set of contacts and a downward external force (0,0,-g).
 * 
 * In the 1-argument version, a contact point is given by a list of 7 floats, [x,y,z,nx,ny,nz,k] as usual. 
 * The 2-argument version is a "fancy" version that allows more control over the
 * constraint planes.
 *
 * Args:
 * 
 *      contacts (list of 7-float lists or tuples): the list of contacts, each
 *          specified as a 7-list or tuple [x,y,z,nx,ny,nz,k], with:
 * 
 *              * (x,y,z): the contact position
 *              * (nx,ny,nz): the contact normal
 *              * k: the coefficient of friction (>= 0)
 * 
 *      contactPositions (list of 3-float lists or tuples): the list of contact
 *          point positions. 
 *      frictionCones (list of lists): Each item of this list specifies linear
 *          inequalities that must be met of the force at the corresponding
 *          contact point.  The item must have length k*4 where k is an integer,
 *          and each inequality gives the entries (ax,ay,az,b) of a constraint
 *          ax*fx+ay*fy+az*fz <= b that limits the contact force (fx,fy,fz) at
 *          the i'th contact.  Each of the k 4-tuples is laid out sequentially
 *          per-contact.
 * 
 * Returns:
 * 
 *     list of 3-tuples: The sorted plane boundaries of the support 
 *     polygon. The format of a plane is (nx,ny,ofs) where (nx,ny) are the
 *     outward facing normals, and ofs is the offset from 0.  In other words
 *     to test stability of a com with x-y coordinates [x,y], you can test
 *     whether dot([nx,ny],[x,y]) <= ofs  for all planes.
 *
 *     Hint: with numpy, you can do::
 *
 *         Ab = np.array(supportPolygon(args))
 *         A=Ab[:,0:2]
 *         b=Ab[:,2]
 *         myComEquilibrium = lambda x: np.all(np.dot(A,x)<=b)
 *
 */
PyObject* support_polygon(const std::vector<std::vector<double> >& contactPositions,const std::vector<std::vector<double> >& frictionCones);


/// Calculates the support polygon (interval)  for a given set of contacts and a downward
/// external force (0,-g). A contact point is given by a list of 4 floats, [x,y,theta,k] as usual.
/// 
/// The return value is a 2-tuple giving the min / max extents of the support polygon.
/// If they are both infinite, the support polygon is empty.
PyObject* support_polygon_2d(double* contacts,int m,int n);

//note: only the last overload docstring is added to the documentation
/** @brief Calculates the support polygon (interval)  for a given set of contacts and a downward
 * external force (0,-g).
 *
 * The 2-argument version is a "fancy" version that allows more control over the
 * constraint planes.
 * 
 * Args:
 * 
 *     contacts (list of 4-float lists or tuples): the list of contacts, each
 *         specified as a 4-list or tuple [x,y,theta,k], with:
 * 
 *             * (x,y,z): the contact position
 *             * theta: is the normal angle (in radians, CCW to the x axis)
 *             * k: the coefficient of friction (>= 0)
 * 
 *     contactPositions (list of 2-float lists or tuples): the list of contact
 *         point positions. 
 *     frictionCones (list of lists): The i'th element in this list has length
 *          k*3 (for some integer k), and gives the contact force constraints
 *          (ax,ay,b) where ax*fx+ay*fy <= b limits the contact force (fx,fy)
 *          at the i'th contact. Each of the k 3-tuples is laid out sequentially
 *          per-contact.
 *
 * Returns:
 *
 *     2-tuple: gives the min/max extents of the support polygon. 
 *     If the support interval is empty, (inf,inf) is returned.
 *
 */
PyObject* support_polygon_2d(const std::vector<std::vector<double> >& contacts,const std::vector<std::vector<double> >& frictionCones);


/** @brief Solves for the torques / forces that keep the robot balanced against gravity
 * 
 * Args:
 *
 * - robot: the robot model, posed in its current configuration
 * - contacts: an nx7 array of contact points, each given as 7-lists [x,y,z,nx,ny,nz,kFriction]
 * - links: a list of the links on which those contact points lie
 * - fext: the external force (e.g., gravity)
 * - norm: the torque norm to minimize.  If 0, minimizes the l-infinity norm (default)
 *         If 1, minimizes the l-1 norm.  If 2, minimizes the l-2 norm (experimental,
 *          may not get good results)
 *
 * Returns:
 *
 *     tuple: a pair (t,f) giving the joint torques and frictional
 *     contact forces, if a solution exists, or None if no solution exists.
 *
 */
PyObject* equilibrium_torques(const RobotModel& robot,
							double* contacts,int m,int n,const std::vector<int>& links,
							const std::vector<double>& fext,
							double norm=0);
/** Solves for the torques / forces that keep the robot balanced against gravity.
 *
 * The problem being solved is
 * 
 * :math:`min_{t,f_1,...,f_N} \|t\|_p`
 *
 * :math:`s.t. t_{int} + G(q) = t + sum_{i=1}^N J_i(q)^T f_i`
 *
 * :math:`|t| \leq t_{max}`
 *
 * :math:`f_i \in FC_i`
 *
 * Args:
 *
 *     robot (RobotModel): the robot, posed in its current configuration
 *     contacts (ndarray): an N x 7 array of contact points, each given as 7-lists
 *         [x,y,z,nx,ny,nz,kFriction]
 *     links (list of N ints): a list of the links on which those contact points
 *         lie
 *     fext (list of 3 floats): the external force (e.g., gravity)
 *     norm (double): the torque norm to minimize.  
 *
 *         * If 0, minimizes the l-infinity norm (default)
 *         * If 1, minimizes the l-1 norm. 
 *         * If 2, minimizes the l-2 norm (experimental, may not get good results).
 *     internalTorques (list of robot.numLinks() floats, optional): allows you to
 *         solve for dynamic situations, e.g., with coriolis forces taken into
 *         account.  These are added to the RHS of the torque balance equation. 
 *         If not given, t_int is assumed to be zero.
 * 
 *         To use dynamics, set the robot's joint velocities dq, calculate
 *         then calculate the torques via robot.torquesFromAccel(ddq), and pass
 *         the result into internalTorques.
 *
 * Returns:
 * 
 *     pair of lists, optional: a pair (torque,force) if a solution exists,
 *     giving valid joint torques t and frictional contact forces (f1,...,fn). 
 *
 *     None is returned if no solution exists.
 *
 */
PyObject* equilibrium_torques(const RobotModel& robot,
							double* contacts,int m,int n,const std::vector<int>& links,
							const std::vector<double>& fext,
							const std::vector<double>& internalTorques,
							double norm=0);


#endif // _STABILITY_H