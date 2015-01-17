#ifndef COLLIDE_H
#define COLLIDE_H

/** @file collide.h
 * @brief C bindings for the collide module. (1/17/15: deprecated,
 * use the object-oriented bindings in geometry.h instead) */

/* @brief Creates a new empty geometry ID.  When done, this ID should be
 * freed from memory using destroyGeom(id) or destroy().
 */ 
int newGeom();

/* @brief Deletes a previously created geometry ID. An exception is raised
 * if the ID is invalid.
 */
void destroyGeom(int geom);

/** @brief Frees all memory allocated by the collide module.  All
 * existing geometry ids and collision query ids are invalidated.
 */
void destroy();

/** @brief Loads a geometry from a file.  Sets it to the correct type based on
 * the file contents.
 * 
 * Currently supported file extensions are
 * - Trimeshes: .tri, any other mesh files that Assimp may support
 *   (if Klamp't is built using  Assimp support).
 * - Point clouds: .pcd
 * - Primitive geometries: .geom 
 *
 * Returns False if there is a load error. Raises an exception if the ID is
 * invalid.
 */
bool loadGeom(int geom,const char* fn);

/** @brief Makes a geometry into a trimesh loaded from the file fn */
void makeTriMeshGeom(int geom,const char* fn);

/** @brief Makes a geometry into a trimesh given the vertex and index data.
 * verts is of length nv*3, and inds is of length nt*3.
 *
 * Note: in Python, must use doubleArray and intArray for the verts and inds
 * objects.
 */
void makeTriMeshGeom(int geom,const double* verts,const int* inds,int nv,int nt);
/** @brief Sets the translation of a trimesh geom. */
void setTriMeshTranslation(int geom,const double t[3]);
/** @brief Sets the rotation of a trimesh geom. */
void setTriMeshRotation(int geom,const double r[9]);
/** @brief Gets the translation of a trimesh geom. */
void getTriMeshTranslation(int geom,double out[3]);
/** @brief Gets the rotation of a trimesh geom. */
void getTriMeshRotation(int geom,double out[9]);
/** @brief Gets the bounding box of a trimesh geom. */
void getTriMeshBB(int geom,double out[3],double out2[3]);
/** @brief Gets the number of vertices of a trimesh geom. */
int getTriMeshNumVerts(int geom);
/** @brief Gets the number of triangles of a trimesh geom. */
int getTriMeshNumTris(int geom);
/** @brief Gets the vertex data of a trimesh geom (length nv*3). */
double* getTriMeshVerts(int geom);
/** @brief Gets the index data of a trimesh geom (length nt*3). */
int* getTriMeshTris(int geom);

/** @brief Makes a geom into a point x */
void makePointGeom(int geom,const double x[3]);
/** @brief Makes a geom into a sphere centered at c with radius r */
void makeSphereGeom(int geom,const double c[3],double r);
/** @brief Makes a geom into a ray with source s and direction d */
void makeRayGeom(int geom,const double s[3],const double d[3]);
/** @brief Makes a geom into a line with source s and direction d */
void makeLineGeom(int geom,const double s[3],const double d[3]);
/** @brief Makes a geom into a segment with endpoints a, b */
void makeSegmentGeom(int geom,const double a[3],const double b[3]);
/** @brief Makes a geom into an axis-aligned bounding box with lower bound
 * bmin and upper bound bmax.
 */
void makeAABBGeom(int geom,const double bmin[3],const double bmax[3]);
/** @brief Makes a geom into a group geom from an array of other geoms. 
 * Note: in Python, must use an intArray for geoms argument.
 */
void makeGroupGeom(int geom,int* geoms,int numgeoms);



/** @brief Tests whether the two geometries collide */
bool collide(int geom1,int geom2);

/** @brief Tests whether the two geometries are within the given tolerance */
bool withinTolerance(int geom1,int geom2,double tol);

/** @brief Returns the distance between the two geometries, possibly with
 * an approximation error (useful to speed up mesh-mesh distance detection)
 *
 * Error of result is no more than D*relErr+absErr where D is the actual
 * distance.  Set relErr=absErr=0 to get exact distance.
 *
 * NOTE: Not yet implemented.
 */
double distance(int geom1,int geom2,double relErr,double absErr);

/** @brief Returns the closest points between the two geometries.
 * These are given in world coordinates.
 *
 * NOTE: Not yet implemented.
 */
void closestPoints(int geom1,int geom2,double out[3],double out2[3]);

/** @brief Returns true if the geometry is hit by the given ray, and also
 * returns the hit point (in world coordinates).
 */
bool rayCast(int geom,const double s[3],const double d[3],double out[3]);

/** @brief Creates a collision query object attachd to the two given
 * geometries.  For mesh-mesh collisions, on repeated calls, this may be
 * somewhat faster than querying from scratch.
 */
int makeCollQuery(int geom1,int geom2);
/** @brief Deletes a collision query object. */
void destroyCollQuery(int query);
/** @brief Checks if the two geoms associated with this query are colliding. */
bool queryCollide(int query);
/** @brief Checks if the two geoms associated with this query are within the
 * given tolerance.  @sa withinTolerance */
bool queryWithinTolerance(int query,double tol);
/** @brief Computes the distance betweeen the two geoms associated with this
 * query.  @sa distance*/
double queryDistance(int query,double relErr,double absErr);
/** @brief Computes points that give rise to the closest distance betweeen
 * the two geoms associated with this query.  @sa closestPoints */
void queryClosestPoints(int query,double out[3],double out2[3]);
/** @brief If the two geoms associated with this query are within a given
 * tolerance (from a previous queryWithinTolerance call), this produces the
 * points on geom1 and geom2, respectively that are within that tolerance.  */
void queryTolerancePoints(int query,double out[3],double out2[3]);

#endif
