#ifndef _GEOMETRY_H
#define _GEOMETRY_H

#include <vector>
#include <map>
#include "viewport.h"

/** @file geometry.h
 * @brief C++ bindings for geometry modeling. */

 class Appearance;

/** @brief A 3D indexed triangle mesh class.
 *
 * Attributes:
 *
 *     vertices (numpy array):  an n x 3 array of vertices.
 *     indices (numpy int32 array): an m x 3 list of triangle vertices, given
 *         as indices into the vertices list, i.e., [[a1,b1,c2], [a2,b2,c2], ...]
 *
 * 
 * Examples::
 *
 *     m = TriangleMesh()
 *     m.addVertex((0,0,0))
 *     m.addVertex((1,0,0))
 *     m.addVertex((0,1,0))
 *     m.addTriangle(0,1,2)
 *     print(len(m.vertices))  #prints 3
 *     print(len(m.indices))   #prints 1
 *
 */
struct TriangleMesh
{
  TriangleMesh();
  ~TriangleMesh();
  TriangleMesh(const TriangleMesh& rhs);
  ///Creates a standalone object that is a copy of this 
  TriangleMesh copy() const;
  ///Used internally 
  void operator = (const TriangleMesh& rhs);
  ///Copies the data of the argument into this.
  void set(const TriangleMesh&);

  ///Retrieves an array view of the vertices.
  ///
  ///Returns:
  ///
  ///    ndarray: an nx3 Numpy array. Setting elements of this array will
  ///    change the vertices.
  ///
  ///Return type: np.ndarray
  void getVertices(double** np_view2, int* m, int* n);
  ///Sets all vertices to the given nx3 Numpy array
  void setVertices(double* np_array2, int m, int n);
  ///Adds a new vertex
  void addVertex(double p[3]);
  ///Retrieves an array view of the triangle indices.
  ///
  ///Returns:
  ///
  ///    ndarray: an mx3 Numpy array of int32 type. Setting elements of this 
  ///    array will change the triangle indices.
  /// 
  ///Return type: np.ndarray
  void getIndices(int** np_view2, int* m, int* n);
  ///Sets all indices to the given mx3 Numpy array
  void setIndices(int* np_array2, int m, int n);
  /// Adds a new triangle 
  void addTriangleIndices(int t[3]);
  ///Translates all the vertices by v=v+t
  void translate(const double t[3]);
  ///Transforms all the vertices by the rigid transform v=R*v+t
  void _transform(const double R[9],const double t[3]);

  void* dataPtr;
  bool isStandalone;
};

/** @brief Stores a set of points to be set into a ConvexHull type. Note: 
 * These may not actually be the vertices of the convex hull; the actual
 * convex hull may be computed internally for some datatypes.
 *
 * Attributes:
 * 
 *     points (numpy array): an nx3 numpy array of points.
 *
 */
struct ConvexHull
{
  ConvexHull();
  ~ConvexHull();
  ConvexHull(const ConvexHull& rhs);
  ///Creates a standalone object that is a copy of this 
  ConvexHull copy() const;
  ///Used internally 
  void operator = (const ConvexHull& rhs);
  ///Copies the data of the argument into this.
  void set(const ConvexHull&);

  ///Retrieves a view of the points. 
  ///
  ///Returns:
  ///
  ///    ndarray: an nx3 Numpy array. Setting elements of this array will
  ///    immediately take effect.
  ///
  ///Return type: np.ndarray
  void getPoints(double** np_view2, int* m, int* n);
  ///Sets all points to the given nx3 Numpy array
  void setPoints(double* np_array2, int m, int n);
  ///Adds a point
  void addPoint(const double pt[3]);
  ///Translates all the vertices by v=v+t
  void translate(const double t[3]);
  ///Transforms all the vertices by the rigid transform v=R*v+t
  void _transform(const double R[9],const double t[3]);

  void* dataPtr;
  bool isStandalone;
};

/** @brief A 3D point cloud class.  
 *
 * Attributes:
 * 
 *     vertices (numpy array): a n x 3 array of vertices.
 *     properties (numpy array): a n x k array of vertex properties.  The
 *        name of each property is either anonymous or retrieved by
 *        `getPropertyName`.
 *
 * 
 * Property names are usually lowercase but follow PCL naming convention, and 
 * often include:
 *
 * - ``normal_x``, ``normal_y``, ``normal_z``: the outward normal 
 * - ``rgb``, ``rgba``: integer encoding of RGB (24 bit int, format 0xrrggbb)
 *   or RGBA color (32 bit int, format 0xaarrggbb) 
 * - ``opacity``: opacity, in range [0,1]
 * - ``c``: opacity, in range [0,255]
 * - ``r,g,b,a``: color channels, in range [0,1]
 * - ``u,v``: texture coordinate
 * - ``radius``: treats the point cloud as a collection of balls 
 * 
 * 
 * Settings are usually lowercase but follow PCL naming convention, and often
 * include:
 *
 * - ``version``: version of the PCL file, typically "0.7" 
 * - ``id``: integer id
 * - ``width``: the width (in pixels) of a structured point cloud
 * - ``height``: the height (in pixels) of a structured point cloud
 * - ``viewpoint``: Camera position and orientation in the form 
 *   ``ox oy oz qw qx qy qz``, with (ox,oy,oz) the focal point and 
 *   (qw,qx,qy,qz) the quaternion representation of the orientation (canonical
 *   representation, with X right, Y down, Z forward).
 *
 * 
 * Examples::
 * 
 *     pc = PointCloud()
 *     pc.addProperty('rgb')
 *     #add 1 point with coordinates (0,0,0) and color #000000 (black)
 *     pc.addPoint((0,0,0))
 *     pc.setProperty(0,0)
 *     print(len(pc.points)) # prints 1
 *     #add another point with coordinates (1,2,3)
 *     pc.addPoint([1,2,3])
 *     #this prints 2
 *     print(len(pc.points))
 *     print(pc.points)   #prints [[0,0,0],[1,2,3]]
 *     #this prints (2,1), because there is 1 property category x 2 points
 *     print(pc.properties.shape)
 *     #this prints 0; this is the default value added when addPoint was called
 *     print(pc.getProperty(1,0) )
 *
 */
struct PointCloud
{
  PointCloud();
  ~PointCloud();
  PointCloud(const PointCloud& rhs);
  ///Creates a standalone object that is a copy of this 
  PointCloud copy() const;
  ///Used internally 
  void operator = (const PointCloud& rhs);
  ///Copies the data of the argument into this.
  void set(const PointCloud&);

  ///Returns a view of the points.
  ///
  ///Returns:
  ///
  ///    ndarray: an nx3 Numpy array. Setting elements of this array will
  ///    change the points.
  ///
  ///Return type: np.ndarray
  void getPoints(double** np_view2, int* m, int* n);
  ///Sets all the points to the given nx3 Numpy array
  void setPoints(double* np_array2, int m, int n);
  ///Adds a point. Sets all its properties to 0.  
  ///
  ///Slow if properties are already set.  Setting the points and properties as
  ///matrices is faster.
  ///
  ///Returns the point's index.
  int addPoint(const double p[3]);
  ///Sets the position of the point at the given index to p
  void setPoint(int index,const double p[3]);
  ///Returns the position of the point at the given index
  ///
  ///Return type: Vector3
  void getPoint(int index,double out[3]) const;
  ///Returns the number of properties
  int numProperties() const;
  ///Sets all the points and m properties from the given n x (3+k) array
  void setPointsAndProperties(double* np_array2, int m,int n);
  ///Sets all the properties of all points to the given nxk array 
  void setProperties(double* np_array2, int m, int n);
  ///Returns all the properties of all points as an array view.
  ///
  ///Returns:
  ///
  ///    ndarray: an nxk Numpy array. Setting elements of this array will
  ///    change the vertices.
  ///
  ///Return type: np.ndarray
  void getProperties(double** np_view2, int* m, int* n);
  ///Adds a new property.  All values for this property are set to 0.
  int addProperty(const std::string& pname);
  ///Adds a new property with name pname, and sets values for this property to the given length-n array
  int addProperty(const std::string& pname,double* np_array,int m);
  /// Sets the name of a given property
  void setPropertyName(int pindex,const std::string& pname);
  /// Returns the name of a given property 
  std::string getPropertyName(int pindex) const;
  /// Returns the index of a named property or -1 if it does not exist 
  int propertyIndex(const std::string& pname) const;
  ///Sets property pindex of point index to the given value
  void setProperty(int index,int pindex,double value);
  ///Sets the property named pname of point index to the given value
  void setProperty(int index,const std::string& pname,double value);
  ///Returns property pindex of point index 
  double getProperty(int index,int pindex) const;
  ///Returns the property named pname of point index
  double getProperty(int index,const std::string& pname) const;
  ///Translates all the points by v=v+t
  void translate(const double t[3]);
  ///Transforms all the points by the rigid transform v=R*v+t
  void _transform(const double R[9],const double t[3]);
  ///Adds the given point cloud to this one.  They must share the same
  ///properties or else an exception is raised
  void join(const PointCloud& pc);
  ///Sets the given setting
  void setSetting(const std::string& key,const std::string& value);
  ///Returns the given setting
  std::string getSetting(const std::string& key) const;
  ///Sets a structured point cloud from a depth image.  [fx,fy,cx,cy] are the intrinsics parameters.  The depth is given as a size hxw array, top to bottom.
  void setDepthImage_d(const double intrinsics[4],double* np_array2,int m,int n,double depth_scale);
  ///Sets a structured point cloud from a depth image.  [fx,fy,cx,cy] are the intrinsics parameters.  The depth is given as a size hxw array, top to bottom.
  void setDepthImage_f(const double intrinsics[4],float* np_depth2,int m2,int n2,double depth_scale);
  ///Sets a structured point cloud from a depth image.  [fx,fy,cx,cy] are the intrinsics parameters.  The depth is given as a size hxw array, top to bottom.
  void setDepthImage_s(const double intrinsics[4],unsigned short* np_depth2,int m2,int n2,double depth_scale);
  ///Sets a structured point cloud from an RGBD (color,depth) image pair.  [fx,fy,cx,cy] are the intrinsics parameters.  The RGB colors are packed in 0xrrggbb order, size hxw, top to bottom.
  void setRGBDImages_i_d(const double intrinsics[4],unsigned int* np_array2,int m,int n,double* np_depth2,int m2,int n2,double depth_scale);
  ///Sets a structured point cloud from an RGBD (color,depth) image pair.  [fx,fy,cx,cy] are the intrinsics parameters.  The RGB colors are packed in 0xrrggbb order, size hxw, top to bottom.
  void setRGBDImages_i_f(const double intrinsics[4],unsigned int* np_array2,int m,int n,float* np_depth2,int m2,int n2,double depth_scale);
  ///Sets a structured point cloud from an RGBD (color,depth) image pair.  [fx,fy,cx,cy] are the intrinsics parameters.  The RGB colors are packed in 0xrrggbb order, size hxw, top to bottom.
  void setRGBDImages_i_s(const double intrinsics[4],unsigned int* np_array2,int m,int n,unsigned short* np_depth2,int m2,int n2,double depth_scale);
  ///Sets a structured point cloud from an RGBD (color,depth) image pair.  [fx,fy,cx,cy] are the intrinsics parameters.  The RGB colors are packed in 0xrrggbb order, size hxw, top to bottom.
  void setRGBDImages_b_d(const double intrinsics[4],unsigned char* np_array3,int m,int n,int p,double* np_depth2,int m2,int n2,double depth_scale);
  ///Sets a structured point cloud from an RGBD (color,depth) image pair.  [fx,fy,cx,cy] are the intrinsics parameters.  The RGB colors are an h x w x 3 array, top to bottom.
  void setRGBDImages_b_f(const double intrinsics[4],unsigned char* np_array3,int m,int n,int p,float* np_depth2,int m2,int n2,double depth_scale);
  ///Sets a structured point cloud from an RGBD (color,depth) image pair.  [fx,fy,cx,cy] are the intrinsics parameters.  The RGB colors are an h x w x 3 array, top to bottom.
  void setRGBDImages_b_s(const double intrinsics[4],unsigned char* np_array3,int m,int n,int p,unsigned short* np_depth2,int m2,int n2,double depth_scale);

  void* dataPtr;
  bool isStandalone;
};

/** @brief A geometric primitive.  So far only points, spheres, segments,
 * and AABBs can be constructed manually in the Python API. 
 *
 * Attributes:
 *
 *     type (str): Can be "Point", "Sphere", "Segment", "Triangle", 
 *         "Polygon", "AABB", and "Box".  Semi-supported types include 
 *         "Ellipsoid", and "Cylinder".
 *     properties (numpy array): a list of parameters defining the 
 *         primitive. The interpretation is type-specific.
 *
 */
struct GeometricPrimitive
{
  GeometricPrimitive();
  ~GeometricPrimitive();
  GeometricPrimitive(const GeometricPrimitive& rhs);
  ///Creates a standalone object that is a copy of this
  GeometricPrimitive copy() const;
  ///Used internally 
  void operator = (const GeometricPrimitive& rhs);
  ///Copies the data of the argument into this.
  void set(const GeometricPrimitive&);

  void setPoint(const double pt[3]);
  void setSphere(const double c[3],double r);
  void setSegment(const double a[3],const double b[3]);
  void setTriangle(const double a[3],const double b[3],const double c[3]);
  void setPolygon(const std::vector<double>& verts);
  void setAABB(const double bmin[3],const double bmax[3]);
  void setBox(const double ori[3],const double R[9],const double dims[3]);
  std::string getType() const;
  void getProperties(double** np_out, int* m) const;
  void setProperties(double* np_array, int m);
  bool loadString(const char* str);
  std::string saveString() const;

  void* dataPtr;
  bool isStandalone;
};

/** @brief An axis-aligned volumetric grid representing a signed distance
 * transform with > 0 indicating outside and < 0 indicating inside. 
 * 
 * In general, values are associated with cells rather than vertices.
 * 
 * Cell (i,j,k) contains a value, and has size
 * (w,d,h) = ((bmax[0]-bmin[0])/dims[0], (bmax[1]-bmin[1])/dims[1], (bmax[2]-bmin[2])/dims[2]).
 * It ranges over the box [w*i,w*(i+1)) x [d*j,d*(j+1)) x [h*k,h*(k+1)).
 * 
 * The field should be assumed sampled at the **centers** of cells, i.e.,
 * at (w*(i+1/2),d*(j+1/2),h*(k+1/2)).
 *
 * Attributes:
 *
 *     bmin (array of 3 doubles): contains the minimum bounds.
 * 
 *     bmax (array of 3 doubles): contains the maximum bounds.
 * 
 *     values (numpy array): contains a 3D array of
 *          ``dims[0] x dims[1] x dims[2]`` values. 
 * 
 *     truncationDistance (float): inf for SDFs, and the truncation distance
 *          for TSDFs. Cells whose values are >= d are considered "sufficiently
 *          far" and distance / tolerance queries outside of this range are
 *          usually not meaningful.
 * 
 */
class ImplicitSurface
{
public:
  ImplicitSurface();
  ~ImplicitSurface();
  ImplicitSurface(const ImplicitSurface& rhs);
  ///Creates a standalone object that is a copy of this
  ImplicitSurface copy() const;
  ///Used internally 
  void operator = (const ImplicitSurface& rhs);
  ///Copies the data of the argument into this.
  void set(const ImplicitSurface&);

  void getBmin(double out[3]) const;
  void setBmin(const double bmin[3]);
  void getBmax(double out[3]) const;
  void setBmax(const double bmax[3]);
  /// Resizes the x, y, and z dimensions of the grid 
  void resize(int sx,int sy,int sz);
  ///Sets all elements to a uniform value (e.g., 0)
  void set(double value);
  ///Sets a specific element of a cell
  void set(int i,int j,int k,double value);
  ///Gets a specific element of a cell
  double get(int i,int j,int k);
  ///Shifts the value uniformly 
  void shift(double dv);
  ///Scales the value uniformly 
  void scale(double cv);
  ///Returns a 3D Numpy array view of the values.  Changes to this 
  ///array will immediately take effect.
  ///
  ///Return type: np.ndarray
  void getValues(double** np_view3, int* m, int* n, int* p);
  ///Sets the values to a 3D numpy array
  void setValues(double* np_array3, int m, int n, int p);
  ///Sets the truncation distance for TSDFs. 
  void setTruncationDistance(double d);
  ///Retrieves the truncation distance for TSDFs. 
  double getTruncationDistance() const;

  void* dataPtr;
  bool isStandalone;
};

/** @brief An occupancy grid with 1 indicating inside and 0
 * indicating outside.  Can also be a fuzzy (probabilistic / density)
 * grid.
 *  
 * In general, values are associated with cells rather than vertices.
 * 
 * Cell (i,j,k) contains an occupancy / density value, and has size
 * (w,d,h) = ((bmax[0]-bmin[0])/dims[0], (bmax[1]-bmin[1])/dims[1], (bmax[2]-bmin[2])/dims[2]).
 * It ranges over the box [w*i,w*(i+1)) x [d*j,d*(j+1)) x [h*k,h*(k+1)).
 *
 * Attributes:
 *
 *     bmin (array of 3 doubles): contains the minimum bounds.
 * 
 *     bmax (array of 3 doubles): contains the maximum bounds.
 * 
 *     values (numpy array): contains a 3D array of
 *          ``dims[0] x dims[1] x dims[2]`` values. 
 * 
 *     occupancyThreshold (float): set to 0.5 by default. Collision
 *         detection treats all cells whose values are greater than this
 *         value as occupied.
 * 
 */
class OccupancyGrid
{
public:
  OccupancyGrid();
  ~OccupancyGrid();
  OccupancyGrid(const OccupancyGrid& rhs);
  ///Creates a standalone object that is a copy of this
  OccupancyGrid copy() const;
  ///Used internally 
  void operator = (const OccupancyGrid& rhs);
  ///Copies the data of the argument into this.
  void set(const OccupancyGrid&);

  void getBmin(double out[3]) const;
  void setBmin(const double bmin[3]);
  void getBmax(double out[3]) const;
  void setBmax(const double bmax[3]);
  /// Resizes the x, y, and z dimensions of the grid 
  void resize(int sx,int sy,int sz);
  ///Sets all elements to a uniform value (e.g., 0)
  void set(double value);
  ///Sets a specific element of a cell
  void set(int i,int j,int k,double value);
  ///Gets a specific element of a cell
  double get(int i,int j,int k);
  ///Shifts the value uniformly 
  void shift(double dv);
  ///Scales the value uniformly 
  void scale(double cv);
  ///Returns a 3D Numpy array view of the values.   Changes to this 
  ///array will immediately take effect.
  ///
  ///Return type: np.ndarray
  void getValues(double** np_view3, int* m, int* n, int* p);
  ///Sets the values to a 3D numpy array
  void setValues(double* np_array3, int m, int n, int p);
  /// Sets the threshold for collision detection
  void setOccupancyThreshold(double threshold);
  /// Gets the threshold for collision detection
  double getOccupancyThreshold() const;

  void* dataPtr;
  bool isStandalone;
};

/** @brief A height (elevation) map or a depth map.
 * 
 * In elevation-map form (`viewport.perspective=false`), the values are the
 * z-height of the terrain at each grid point.  In depth-map form
 * (`viewport.perspective=true`), the values are the depths of each
 * grid point (not distance) from the origin in the +z direction.
 * 
 * Note that unlike volume grid types, each grid entry is defined at a
 * vertex, not a cell.  In elevation map form, the (i,j) cell is associated
 * with the vertex ((i-cx)/fx,(n-1-j-cy)/fy,heights[i,j]) where n is
 * `heights.shape[1]`.  Note that the y-axis is flipped in the heightmap
 * compared to the viewport such that the image is given in standard
 * top-down convention.
 * 
 * In depth map form, the (i,j) cell is associated with the vertex
 * (d*(i-cx)/fx,d*(j-cy)/fy,d) where d=heights[i,j].  Note that the y-axis
 * is not flipped in this convention.
 *
 * Attributes:
 *
 *     viewport (Viewport): contains the size (w,h), projection (perspective)
 *          intrinsics (fx,fy,cx,cy), and pose (pose) of the heightmap
 *          reference coordinate system.  Note that to change the viewport, you
 *          will need to use `vp = hm.viewport; vp.w = ...; hm.viewport = vp`.
 * 
 *     heights (np.ndarray): contains a 2D array of (w,h) values.
 * 
 *     colorImage (np.ndarray): contains a 3D image array of colors in
 *          grayscale (h,w), RGB (h,w,3), or RGBA (h,w,4) format.
 * 
 */
class Heightmap
{
public:
  Heightmap();
  ~Heightmap();
  Heightmap(const Heightmap& rhs);
  ///Creates a standalone object that is a copy of this 
  Heightmap copy() const;
  ///Used internally 
  void operator = (const Heightmap& rhs);
  ///Copies the data of the argument into this.
  void set(const Heightmap&);

  /// Resizes the height map 
  void resize(int w,int h);
  ///Sets an orthographic projection (elevation map) with the given width and height
  void setSize(double width, double height);
  /// Sets an perspective projection (depth map) with the given x and y fields of view 
  /// and centered focal point.  If fovy=-1, then it will be set so that pixels
  /// are square.
  void setFOV(double fovx,double fovy=-1);
  /// Sets an perspective projection (depth map) with the given intrinsics fx, fy, cx, cy.
  /// If cx or cy are negative, then cx = (w-1)/2, cy = (h-1)/2.
  void setIntrinsics(double fx,double fy,double cx=-1,double cy=-1);
  /// Returns true if the heightmaps is in perspective (depth map) mode 
  bool isPerspective() const;
  /// Returns true if the heightmaps is in orthographic (elevation map) mode
  bool isOrthographic() const { return !isPerspective(); }
  /// Retrieves the viewport, which defines how the heightmap data is mapped to
  /// spatial coordinates.  Note that this is returned by value. If you change an
  /// attribute of the viewport, you should call setViewport (or viewport=...) to
  /// update the heightmap's viewport.
  Viewport getViewport() const;
  /// Sets the viewport
  void setViewport(const Viewport& viewport);
  ///Sets all elements to a uniform value (e.g., 0)
  void set(double value);
  ///Sets the height of a vertex (note, indices are x and y units, which is reversed from image convention)
  void set(int i,int j,double value);
  ///Gets the height of a vertex (note, indices are x and y units, which is reversed from image convention)
  double get(int i,int j);
  ///Gets the coordinates of a vertex (note, indices are x and y units, which is reversed from image convention)
  void getVertex(int i,int j,double out[3]) const;
  ///Returns the width/height of the heightmap in real coordinates.
  ///(only directly interpretable in orthographic mode; in perspective mode the
  ///returned value is the horizontal / vertical FOV in radians.)
  void getSize(double out[2]) const;
  ///Returns the width/height of each cell of the heightmap in real coordinates.
  ///(only directly interpretable in orthographic mode; in perspective mode the
  ///returned value is the resolution at 1 unit depth.)  
  void getResolution(double out[2]) const;
  /// Shifts the height uniformly 
  void shift(double dh);
  /// Scales the height uniformly 
  void scale(double c);
  ///Returns a 2D Numpy array view of the values.  Result has shape w x h and 
  ///has float32 dtype.  Modifications to the array are immediately reflected
  ///in the heightmap
  ///
  ///Return type: np.ndarray
  void getHeights(float** np_view2, int* m, int* n);
  ///Sets the values to a 2D numpy array of shape w x h
  void setHeights_f(float* np_array2, int m, int n);
  ///Returns true if colors are present
  bool hasColors() const;
  ///Erases all colors
  void clearColors();
  ///Sets a uniform grayscale color.  Call this first if you want to start setting colors.
  void setColor(double intensity);
  ///Sets a uniform color.  Call this first if you want to start setting colors.
  void setColor(const double rgba[4]);
  ///Gets the grayscale color of a vertex (note, indices are x and y units, which is reversed from image convention)
  void setColor(int i,int j,double intensity);
  ///Gets the RGBA color of a vertex (note, indices are x and y units, which is reversed from image convention)
  void setColor(int i,int j,const double rgba[4]);
  ///Gets the RGBA color of a vertex (note, indices are x and y units, which is reversed from image convention)
  ///
  ///Return type: Tuple[float,float,float,float]
  void getColor(int i,int j,double out[4]);
  ///Returns a 3D Numpy array view of the color image (h x w x (1, 3, or 4)), with rows ordered top to bottom
  ///
  ///Return type: np.ndarray
  void getColorImage(unsigned char** np_view3, int* m, int* n, int* p);
  ///Sets the values to a 3D numpy array (h x w x 1, 3, or (1, 3, or 4)), with rows ordered top to bottom
  void setColorImage_b(unsigned char* np_array3, int m, int n, int p);
  /// Sets colors to a 32-bit RGBA image (size h x w) with rows ordered top to bottom
  void setColorImage_i(unsigned int* np_array2, int m, int n);
  /// Retrieves a 32-bit RGBA image of the heightmap's colors (h x w)
  ///
  /// Return type: np.ndarray
  void getColorImage_i(unsigned int** np_out2, int* m, int* n);
  /// Retrieves a floating point RGB, RGBA, or L image (h x w x (1, 3, or 4)) with rows ordered from top to bottom. 
  ///
  /// Return type: np.ndarray
  void getColorImage_d(double** np_out3, int* m, int* n, int* p);
  /// Returns the number of properties 
  int numProperties() const;
  /// Adds a new property and sets it to 0
  int addProperty(const std::string& pname);
  /// Adds a new property and sets it to an array of size (w x h)
  int addProperty(const std::string& pname,double* np_array2,int m,int n);
  /// Retrieves the index associated with a property name 
  int propertyIndex(const std::string& pname) const;
  /// Sets an individual pixel's property vector 
  void setProperty(int i,int j,double* np_array,int m);
  /// Retrieves an individual pixel's property vector 
  ///
  ///Return type: np.ndarray
  void getProperty(int i,int j,double** np_out,int* m);
  /// Sets a property to an array of size (w x h)
  void setProperties(int pindex,double* np_array2,int m,int n);
  /// Retrieves a view of the property of size (w x h)
  ///
  ///Return type: np.ndarray
  void getProperties(int pindex,float** np_out2,int* m,int* n);

  void* dataPtr;
  bool isStandalone;
};

/** @brief Configures the _ext distance queries of
 * :class:`~klampt.Geometry3D`.
 *
 * The calculated result satisfies :math:`Dcalc \leq D(1+relErr) + absErr`
 * unless :math:`D \geq upperBound`, in which case Dcalc=upperBound may 
 * be returned.
 *
 * Attributes:
 * 
 *     relErr (float, optional): Allows a relative error in the reported
 *         distance to speed up computation.  Default 0.
 *     absErr (float, optional): Allows an absolute error in the reported
 *         distance to speed up computation.  Default 0.
 *     upperBound (float, optional): The calculation may branch if D exceeds
 *         this bound.
 *
 */
class DistanceQuerySettings
{
public:
  DistanceQuerySettings();
  double relErr;
  double absErr;
  double upperBound;
};

/** @brief The result from a "fancy" distance query of 
 * :class:`~klampt.Geometry3D`.
 *
 * Attributes:
 *
 *     d (float): The calculated distance, with negative values indicating
 *         penetration.  Can also be upperBound if the branch was hit.
 *     hasClosestPoints (bool):  If true, the closest point information is
 *         given in cp0 and cp1, and elem1 and elem2
 *     hasGradients (bool):  f true, distance gradient information is given
 *         in grad0 and grad1.
 *     cp1, cp2 (list of 3 floats, optional): closest points on self vs other,
 *         both given in world coordinates
 *     grad1, grad2 (list of 3 floats, optional): the gradients of the
 *         objects' signed distance fields at the closest points.  Given in
 *         world coordinates. 
 *         
 *         I.e., to move object1 to touch object2, move it in direction
 *         grad1 by distance -d.  Note that grad2 is always -grad1.
 *     elems1, elems2 (int): for compound objects, these are the
 *         element indices corresponding to the closest points.
 *
 */
class DistanceQueryResult
{
public:
  DistanceQueryResult();
  double d;
  bool hasClosestPoints;
  bool hasGradients;
  std::vector<double> cp1,cp2;
  std::vector<double> grad1,grad2;
  int elem1,elem2;
};

/** @brief The result from a contact query of :class:`~klampt.Geometry3D`.
 * The number of contacts n is variable.
 *
 * Attributes:
 *
 *     depths (list of n floats): penetration depths for each contact point. 
 *         The depth is measured with respect to the padded geometry, and must
 *         be nonnegative. A value of 0 indicates that depth cannot be 
 *         determined accurately.
 *     points1, points2 (list of n lists of floats): contact points on self vs 
 *         other,  The top level list has n entries, and each entry is a
 *         3-list expressed in world coordinates.  If an object is padded,
 *         these points are on the surface of the padded geometry.
 *     normals (list of n lists of floats): the outward-facing contact normal
 *         from this to other at each contact point, given in world
 *         coordinates.  Each entry is a 3-list, and can be a unit vector,
 *         or [0,0,0] if the the normal cannot be computed properly.
 *     elems1, elems2 (list of n ints): for compound objects, these are the
 *         element indices corresponding to each contact.
 * 
 */
class ContactQueryResult
{
public:
  ContactQueryResult();
  std::vector<double> depths;
  std::vector<std::vector<double> > points1,points2;
  std::vector<std::vector<double> > normals;
  std::vector<int> elems1,elems2;
};

/** @brief The three-D geometry container used throughout Klampt.  
 *
 * There are eight currently supported types of geometry:
 *
 * - primitives (:class:`GeometricPrimitive`)
 * - convex hulls (:class:`ConvexHull`)
 * - triangle meshes (:class:`TriangleMesh`)
 * - point clouds (:class:`PointCloud`)
 * - implicit surfaces (:class:`ImplicitSurface`)
 * - occupancy grids (:class:`OccupancyGrid`)
 * - heightmaps (:class:`Heightmap`)
 * - groups ("Group" type)
 * 
 * For now we also support the "VolumeGrid" identifier which is treated
 * as an alias for "ImplicitSurface".  This will be deprecated in a
 * future version
 * 
 * This class acts as a uniform container of all of these types.
 *
 * There are two modes in which a Geometry3D can be used.  It can be a
 * standalone geometry, which means it is a container of geometry data,
 * or it can be a reference to a world item's geometry.  For references,
 * modifiers change the world item's geometry.
 *
 * **Current transform**
 *
 * Each geometry stores a "current" transform, which is automatically
 * updated for world items' geometries.  Proximity queries are then
 * performed *with respect to the transformed geometries*.  Crucially, the
 * underlying geometry is not changed, because that could be computationally
 * expensive. 
 *
 * **Creating / modifying the geometry**
 *
 * Use the constructor, the :meth:`set`, or the set[TYPE]() methods to
 * completely change the geometry's data.
 * 
 * Note: if you want to set a world item's geometry to have the same contents as
 * a standalone geometry, use the set(rhs) function rather than the assignment (=)
 * operator.  ``object.geometry() = rhs`` does not work.
 *
 * Modifiers include:
 * 
 * - :meth:`setCurrentTransform`: updates the current transform.  (This call is
 *   very fast.)
 * - :meth:`translate`, :meth:`scale`, :meth:`rotate`, and :meth:`transform`
 *   transform the underlying geometry.  Any collision data structures will
 *   be recomputed after transformation. 
 * - :meth:`loadFile`: load from OFF, OBJ, STL, PCD, etc.  Also supports native
 *   Klamp't types .geom and .vol.
 * 
 * .. note::
 *
 *     Avoid the use of translate, rotate, and transform to represent object
 *     movement.  Use setCurrentTransform instead.
 *
 * **Proximity queries**
 * 
 * - :meth:`collides`: boolean collision query.
 * - :meth:`withinDistance`: boolean proximity query.
 * - :meth:`distance` and :meth:`distance_ext`: numeric-valued distance query.
 *   The distance may be negative to indicate signed distance, available for
 *   certain geometry types. Also returns closest points for certain geometry
 *   types.
 * - :meth:`distance_point` and :meth:`distance_point_ext`: numeric valued
 *   distance-to-point queries.
 * - :meth:`contacts`: estimates the contact region between two objects.
 * - :meth:`rayCast` and :meth:`rayCast_ext`: ray-cast queries.
 *
 * For most geometry types (TriangleMesh, PointCloud, ConvexHull), the
 * first time you perform a query, some collision detection data structures
 * will be initialized.  This preprocessing step can take some time for complex
 * geometries.  If you want to do this at a specific time, you can call
 * :meth:`refreshCollider` to initialize the data structures.
 * 
 * Note: Modifying the underlying geometry data (such as ``getPointCloud().points = X``)
 * will NOT update existing collision checking data structures associated with this
 * geometry.  If you had prior data and the collision checking data structures
 * were initialized, you will need to call :meth:`refreshCollider` to update
 * them after modification.  
 * 
 * **Collision margins**
 * 
 * Each object also has a "collision margin" which may virtually fatten the
 * object, as far as proximity queries are concerned. This is useful
 * for setting collision avoidance margins in motion planning.  Use the
 * :meth:`setCollisionMargin` and :meth:`getCollisionMargin` methods to access
 * the margin. By default the margin is zero. 
 *
 * .. note::
 *
 *     The geometry margin is NOT the same thing as simulation body collision
 *     padding!  All proximity queries are affected by the collision padding,
 *     inside or outside of simulation.
 *
 * **Conversions**
 *
 * Many geometry types can be converted to and from one another using the
 * :meth:`convert` method.  This can also be used to remesh TriangleMesh,
 * PointCloud, ImplicitSurface, OccupancyGrid, and Heightmap objects.
 *
 * For more information, please consult the
 * `geometry manual <https://github.com/krishauser/Klampt/blob/master/Cpp/docs/Manual-Geometry.md>`__ .
 */
class Geometry3D
{
 public:
  Geometry3D();
  Geometry3D(const Geometry3D&);
  Geometry3D(const GeometricPrimitive&);
  Geometry3D(const ConvexHull&);
  Geometry3D(const TriangleMesh&);
  Geometry3D(const PointCloud&);
  Geometry3D(const ImplicitSurface&);
  Geometry3D(const OccupancyGrid&);
  Geometry3D(const Heightmap&);
  Geometry3D(const char* fn);
  ~Geometry3D();
  const Geometry3D& operator = (const Geometry3D& rhs);
  ///Creates a standalone geometry from this geometry
  Geometry3D copy();
  ///Copies the geometry of the argument into this geometry.
  void set(const Geometry3D&);
  ///Returns True if this is a standalone geometry
  bool isStandalone();
  ///Frees the data associated with this geometry, if standalone 
  void free();
  ///Returns the type of geometry: GeometricPrimitive, ConvexHull, TriangleMesh,
  ///PointCloud, ImplicitSurface, OccupancyGrid, Heightmap, or Group
  std::string type();
  ///Returns True if this has not been set to a type (not the same as numElements()==0)
  bool empty();
  ///Initializes / refreshes the collision data structures for the current
  ///geometry content.
  void refreshCollider();
  ///Returns a TriangleMesh if this geometry is of type TriangleMesh
  TriangleMesh getTriangleMesh();
  ///Returns a PointCloud if this geometry is of type PointCloud
  PointCloud getPointCloud();
  ///Returns a GeometricPrimitive if this geometry is of type GeometricPrimitive
  GeometricPrimitive getGeometricPrimitive();
  ///Returns a ConvexHull if this geometry is of type ConvexHull
  ConvexHull getConvexHull();
  ///Returns the ImplicitSurface if this geometry is of type ImplicitSurface
  ImplicitSurface getImplicitSurface();
  ///Returns the OccupancyGrid if this geometry is of type OccupancyGrid
  OccupancyGrid getOccupancyGrid();
  ///Returns the Heightmap if this geometry is of type Heightmap
  Heightmap getHeightmap();
  
  ///Sets this Geometry3D to a TriangleMesh
  void setTriangleMesh(const TriangleMesh&);
  ///Sets this Geometry3D to a PointCloud
  void setPointCloud(const PointCloud&);
  ///Sets this Geometry3D to a GeometricPrimitive
  void setGeometricPrimitive(const GeometricPrimitive&);
  ///Sets this Geometry3D to a ConvexHull
  void setConvexHull(const ConvexHull&);
  ///Sets this Geometry3D to be a convex hull of two geometries.  Note: the relative
  ///transform of these two objects is frozen in place; i.e., setting the current
  ///transform of g2 doesn't do anything to this object.
  void setConvexHullGroup(const Geometry3D& g1, const Geometry3D & g2);
  ///Sets this Geometry3D to an ImplicitSurface.
  void setImplicitSurface(const ImplicitSurface& grid);
  ///Sets this Geometry3D to an OccupancyGrid.
  void setOccupancyGrid(const OccupancyGrid& grid);
  ///Sets this Geometry3D to a Heightmap.
  void setHeightmap(const Heightmap& hm);
  ///Sets this Geometry3D to a group geometry.  To add sub-geometries, 
  ///repeatedly call setElement() with increasing indices.
  void setGroup();
  ///Returns an element of the Geometry3D if it is a Group, TriangleMesh, or 
  ///PointCloud.  Raises an error if this is of any other type.  
  ///
  ///The element will be in local coordinates.
  Geometry3D getElement(int element);
  ///Sets an element of the Geometry3D if it is a Group, TriangleMesh, or
  ///PointCloud. The element will be in local coordinates.
  ///Raises an error if this is of any other type.  
  void setElement(int element,const Geometry3D& data);
  ///Returns the number of sub-elements in this geometry
  int numElements();

  ///Loads from file.  Standard mesh types, PCD files, and .geom files are
  ///supported.
  ///
  ///Returns:
  ///
  ///    True on success, False on failure
  ///
  bool loadFile(const char* fn);
  ///Saves to file.  Standard mesh types, PCD files, and .geom files are
  ///supported.
  bool saveFile(const char* fn);
  ///Sets the current transformation (not modifying the underlying data)
  void _setCurrentTransform(const double R[9],const double t[3]);
  ///Gets the current transformation 
  ///
  ///Return type: RigidTransform
  void getCurrentTransform(double out[9],double out2[3]);
  ///Translates the geometry data.
  ///Permanently modifies the data and resets any collision data structures.
  void translate(const double t[3]);
  ///Scales the geometry data uniformly.
  ///Permanently modifies the data and resets any collision data structures.
  void scale(double s);
  ///Scales the geometry data with different factors on each axis.
  ///Permanently modifies the data and resets any collision data structures.
  void scale(double sx,double sy,double sz);
  ///Rotates the geometry data.
  ///Permanently modifies the data and resets any collision data structures.
  void rotate(const double R[9]);
  ///Translates/rotates/scales the geometry data.
  ///Modifies the underlying data and resets any collision data structures.
  void _transform(const double R[9],const double t[3]);
  ///Attaches appearance data to the geometry.  This is only supported by
  ///triangle meshes.
  void setAppearance(const Appearance& appearance);
  ///Retrieves any appearance data attached to the geometry.
  ///If no appearance data is attached, returns an empty Appearance. 
  ///This is only supported by triangle meshes.
  Appearance getAppearance() const;
  ///Sets a padding around the base geometry which affects the results of
  ///proximity queries
  void setCollisionMargin(double margin);
  ///Returns the padding around the base geometry.  Default 0
  double getCollisionMargin();
  ///Returns an axis-aligned bounding box of the object as a tuple (bmin,bmax). 
  ///
  ///Note: O(1) time, but may not be tight
  ///
  ///Return type: Tuple[Vector3,Vector3]
  void getBB(double out[3],double out2[3]);
  ///Computes a tighter axis-aligned bounding box of the object than
  ///:meth:`Geometry3D.getBB`. Worst case O(n) time.
  ///
  ///Return type: Tuple[Vector3,Vector3]
  void getBBTight(double out[3],double out2[3]);
  /** @brief Converts a geometry to another type, if a conversion is
   * available.  The interpretation of param depends on the type of
   * conversion, with 0 being a reasonable default.
   *
   * Interpretations of the parameter are given as follows:
   * 
   *   - GeometricPrimitive -> anything.  param determines the desired
   *        resolution, with default constructing a 20x20x20 grid.
   *   - ConvexHull -> TriangleMesh.  param ignored.
   *   - ConvexHull -> PointCloud.  param is the desired dispersion of the
   *         points.  Equivalent to ConvexHull -> TriangleMesh -> PointCloud
   *   - ConvexHull -> ImplicitSurface.  param is the grid resolution, by
   *         default max(bmax-bmin)/20.
   *   - ConvexHull -> OccupancyGrid.  param is the grid resolution, by
   *         default max(bmax-bmin)/20.
   *   - TriangleMesh -> PointCloud.  param is the desired dispersion of
   *        the points, by default set to the average triangle diameter. 
   *        At least all of the mesh's vertices will be returned.
   *   - TriangleMesh -> ImplicitSurface.  Converted using the fast marching
   *        method, with good results only if the mesh is watertight.
   *        param is the grid resolution, by default set to the average
   *        triangle diameter.
   *   - TriangleMesh -> OccupancyGrid.  Converted using rasterization.
   *        param is the grid resolution, by default set to the average
   *        triangle diameter.
   *   - TriangleMesh -> ConvexHull.  If param==0, just calculates a convex
   *        hull. Otherwise, uses convex decomposition with the HACD library.
   *   - TriangleMesh -> Heightmap.  Converted using rasterization.  param
   *        is the grid resolution, by default set to max(bmax-bmin) / 256.
   *   - PointCloud -> TriangleMesh. Available if the point cloud is
   *        structured. param is the threshold for splitting triangles
   *        by depth discontinuity. param is by default infinity.
   *   - PointCloud -> OccupancyGrid.  param is the grid resolution, by default
   *        some reasonable number.
   *   - PointCloud -> ConvexHull.  Converted using SOLID / Qhull.
   *   - PointCloud -> Heightmap.  param is the grid resolution, by default
   *        set to max(bmax-bmin) / 256.
   *   - ImplicitSurface -> ConvexHull.  Equivalent to ImplicitSurface -> TriangleMesh
   *        -> ConvexHull.
   *   - ImplicitSurface -> TriangleMesh.  param determines the level set for
   *         the marching cubes algorithm.
   *   - ImplicitSurface -> PointCloud.  param determines the level set.
   *   - ImplicitSurface -> OccupancyGrid. param ignored, result matches this resolution.
   *   - ImplicitSurface -> Heightmap. param ignored, result matches this resolution.
   *   - OccupancyGrid -> ConvexHull.  Equivalent to OccupancyGrid -> TriangleMesh
   *        -> ConvexHull.
   *   - OccupancyGrid -> TriangleMesh.  Creates a mesh around each block.
   *   - OccupancyGrid -> PointCloud.  Outputs a point at the center of each block.
   *   - OccupancyGrid -> Heightmap.  param ignored, result matches this resolution.
   *   - Heightmap -> ConvexHull.  Equivalent to Heightmap -> TriangleMesh -> ConvexHull.
   *   - Heightmap -> TriangleMesh.  param ignored, result matches this resolution.
   *   - Heightmap -> PointCloud.  param ignored, result matches this resolution.
   *   - Heightmap -> ImplicitSurface.  param is the resolution in the z direction, by
   *        default set to heightmap range / 128.
   *   - Heightmap -> OccupancyGrid.  param is the resolution in the z direction, by
   *        default set to heightmap range / 128.
   *
   *  Available conversions are listed in the
   * `geometry manual <https://github.com/krishauser/Klampt/blob/master/Cpp/docs/Manual-Geometry.md>`__ .
   *
   */
  Geometry3D convert(const char* type,double param=0);
  /** @brief Returns true if this geometry contains the given point.
   * 
   * An approximate method is used for TriangleMesh.  For PointCloud, the
   * point is considered to be contained if it is one of the points in the
   * cloud, or if points have a radius attribute, within the given radius.
   */
  bool contains_point(const double pt[3]);
  /** @brief Returns true if this geometry collides with the other
   *
   * Unsupported types:
   *
   * - ImplicitSurface - GeometricPrimitive [aabb, box, triangle, polygon]
   * - ImplicitSurface - ConvexHull
   */
  bool collides(const Geometry3D& other);
  /// @brief Same as collide, but will also return the elements of each
  /// geometry that collide.  
  ///
  /// Returns: (elem1, elem2) where elem1 and elem2 are the indices of
  /// the elements that collide.  If len(elem1) == 0, then there is no
  /// detected collision.
  ///
  ///Return type: Tuple[list,list]
  void collides_ext(const Geometry3D& other,int maxContacts,std::vector<int>& out,std::vector<int>& out2);
  ///Returns true if this geometry is within distance ``tol`` to other
  bool withinDistance(const Geometry3D& other,double tol);
  /// @brief Same as withinDistance, but will also return the elements of each
  /// geometry that are within distance tol.
  ///
  /// Returns: (elem1, elem2) where elem1 and elem2 are the indices of
  /// the elements that are within distance tol.  If len(elem1) == 0, then
  /// there is no detected proximity.
  ///
  ///Return type: Tuple[list,list]
  void withinDistance_ext(const Geometry3D& other,double tol,int maxContacts,std::vector<int>& out,std::vector<int>& out2);
  ///Returns the distance from this geometry to the other.  If either geometry 
  ///contains volume information, this value may be negative to indicate
  ///penetration.  See :meth:`Geometry3D.distance` for more information.
  double distance_simple(const Geometry3D& other,double relErr=0,double absErr=0);
  ///Returns the the distance and closest point to the input point, given in 
  ///world coordinates.  An exception is raised if this operation is not 
  ///supported with the given geometry type.
  ///
  ///The return value contains the distance, closest points, and gradients if
  ///available.
  ///
  ///For some geometry types, the signed distance is returned.  The signed
  ///distance returns the negative penetration depth if pt is within this.
  ///The following geometry types return signed distances:
  ///
  ///- GeometricPrimitive
  ///- PointCloud
  ///- ImplictSurface
  ///- Heightmap (approximate, only accurate in the viewing direction)
  ///- ConvexHull
  ///
  ///For other types, a signed distance will be returned if the geometry has
  ///a positive collision margin, and the point penetrates less than this margin.
  DistanceQueryResult distance_point(const double pt[3]);
  ///A customizable version of :meth:`Geometry3D.distance_point`.
  ///The settings for the calculation can be customized with relErr, absErr, 
  ///and upperBound, e.g., to break if the closest points are at least
  ///upperBound distance from one another.
  DistanceQueryResult distance_point_ext(const double pt[3],const DistanceQuerySettings& settings);
  ///Returns the the distance and closest points between the given geometries.
  ///This may be either the normal distance or the signed distance, depending on
  ///the geometry type. 
  ///
  ///The normal distance returns 0 if the two objects are touching
  ///(this.collides(other)=True).
  ///
  ///The signed distance returns the negative penetration depth if the objects
  ///are touching.  Only the following combinations of geometry types return
  ///signed distances:
  ///
  ///- GeometricPrimitive-GeometricPrimitive (missing some for boxes, segments, and tris)
  ///- GeometricPrimitive-TriangleMesh (surface only)
  ///- GeometricPrimitive-PointCloud
  ///- GeometricPrimitive-ImplicitSurface
  ///- TriangleMesh (surface only)-GeometricPrimitive
  ///- PointCloud-ImplicitSurface
  ///- PointCloud-ConvexHull
  ///- ConvexHull-ConvexHull
  ///- ConvexHull-GeometricPrimitive
  ///
  ///If penetration is supported, a negative distance is returned and cp1,cp2
  ///are the deepest penetrating points.
  ///
  ///Unsupported types:
  ///
  ///- PointCloud-PointCloud
  ///- ImplicitSurface-TriangleMesh
  ///- ImplicitSurface-ImplicitSurface
  ///- OccupancyGrid - anything
  ///- ConvexHull - anything else besides ConvexHull
  ///
  ///See the comments of the distance_point function
  DistanceQueryResult distance(const Geometry3D& other);
  ///A customizable version of :meth:`Geometry3D.distance`.
  ///The settings for the calculation can be customized with relErr, absErr,
  ///and upperBound, e.g., to break if the closest points are at least
  ///upperBound distance from one another.  
  DistanceQueryResult distance_ext(const Geometry3D& other,const DistanceQuerySettings& settings);
  ///Performs a ray cast.
  ///
  ///All types supported, but PointCloud needs a positive collision
  ///margin, or points need to have a 'radius' property assigned)
  ///
  ///Returns:
  ///
  ///    (hit,pt) where hit is true if the ray starting at s and pointing
  ///    in direction d hits the geometry (given in world coordinates); pt is
  ///    the hit point, in world coordinates.
  ///
  ///Return type: Tuple[bool,Vector3]
  bool rayCast(const double s[3],const double d[3],double out[3]);
  ///A more sophisticated ray cast. 
  ///
  ///
  ///All types supported, but PointCloud needs a positive collision
  ///
  ///margin, or points need to have a 'radius' property assigned)
  ///Returns:
  ///
  ///    (hit_element,pt) where hit_element is >= 0 if ray starting at 
  ///    s and pointing in direction d hits the geometry (given in world 
  ///    coordinates).  
  ///
  ///    - hit_element is -1 if the object is not hit, otherwise it gives the
  ///      index of the element (triangle, point, sub-object) that was hit.  
  ///      For geometric primitives, this will be 0.
  ///    - pt is the hit point, in world coordinates.
  ///
  ///Return type: Tuple[int,Vector3]
  int rayCast_ext(const double s[3],const double d[3],double out[3]);
 
  ///Returns the set of contact points between this and other.  This set
  ///is a discrete representation of the region of surface overlap, which
  ///is defined as all pairs of points within distance
  ///self.collisionMargin + other.collisionMargin + padding1 + padding2.
  ///
  ///Relatively few geometry types are supported.
  ///
  ///For some geometry types (TriangleMesh-TriangleMesh,
  ///TriangleMesh-PointCloud, PointCloud-PointCloud) padding must be positive
  ///to get meaningful contact poitns and normals.
  ///
  ///If maxContacts != 0  a clustering postprocessing step is performed.
  ///
  ContactQueryResult contacts(const Geometry3D& other,double padding1,double padding2,int maxContacts=0);
  ///Calculates the furthest point on this geometry in the direction dir.
  ///
  ///Supported types:
  ///
  ///- ConvexHull
  ///- GeometricPrimitive
  ///- PointCloud
  ///- TriangleMesh
  ///- OccupancyGrid
  ///
  ///Return type: Vector3
  void support(const double dir[3], double out[3]);
  ///Calculates a 2D slice through the data. The slice is given by the local X-Y plane of a 
  ///transform (R,T) with orientation R and translation t.  The return Geometry's data is in
  ///the local frame of (R,t), and (R,t) is set as its current transform. 
  ///
  ///The geometry's current transform is respected.
  ///
  ///O(N) time.
  ///
  ///Supported types:
  ///
  ///- PointCloud.  Needs tol > 0.  A PointCloud is returned.
  ///- TriangleMesh. tol is ignored. A Group of GeometricPrimitives (segments) is returned.
  ///
  Geometry3D slice(const double R[9],const double t[3],double tol);
  ///Calculates a region of interest of the data for the bounding box [bmin,bmax]. 
  ///The geometry's current transform is respected.
  ///
  ///`query` can be "intersect", "touching", or "within". If "intersect", this tries to get a
  ///representation of the geometry intersecting the box.  If "touching", all elements touching
  ///the box are returned.  If "within", only elements entirely inside the box are returned.
  ///
  ///`query` can also be prefaced with a '~' which indicates that the ROI should be inverted,
  ///i.e. select everything that does NOT intersect with a box.
  ///
  ///O(N) time.
  ///
  ///Supported types:
  ///
  ///- PointCloud
  ///- TriangleMesh
  ///
  Geometry3D roi(const char* query,const double bmin[3],const double bmax[3]);
  ///Merges another geometry into this geometry.  The result is stored
  ///inplace and the type of the result is the same as this geometry.  This can be used
  ///to calculate the union of PointClouds, TriangleMeshes, ConvexPolytopes, and
  ///ImplicitSurfaces, OccupancyGrids, and Heightmaps. 
  ///
  ///ImplicitSurface, OccupancyGrid, and Heightmap merges preserve the domain of the
  ///current grid.  They can also be merged with many other geometries.
  void merge(const Geometry3D& other);

  int world;
  int id;
  void* geomPtr;
};

#endif
