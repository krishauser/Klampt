# Klamp't Manual: File types

* [Primitive data file format](#primitive-data-file-format)
* [Geometry file formats](#geometry-file-formats)
* [World (.xml) file format](#world--xml--file-format)
* [Robot (.rob) files](#robot--rob--files)
* [URDF files (.urdf) with Klamp't-specific elements](#urdf-files--urdf--with-klamp-t-specific-elements)
* [Piecewise Linear Path (.path) files](#piecewise-linear-path--path--files)

Most of the object types in Klamp't can be saved and loaded from disk in a manner that is compatible with the RobotPose app, the I/O mechanisms, and resource managment mechanisms.  The following custom file types are used in Klamp't:

- World files (.xml)
- Robot files (.rob)
- Robot URDF files with Klamp't-specific elements (.urdf)
- Geometric primitive files (.geom)
- Rigid object files (.obj)
- Configuration files (.config)
- Configuration sequence files (.configs)
- Linear path files (.path)
- Multipath files (.xml)
- Hold files (.hold)
- Stance files (.stance)
- Grasp files (.xml)

Klamp't also supports the following standard file types:
- Triangle mesh files (.off, and with many other types with Assimp) 
- Point cloud data files (.pcd)

## Primitive data file format

A Config (robot configuration) file is serialized to a whitespace-delimited string in the format `N q1 ... qN` where N is the number of DOF in the robot.

A Vector3 or Vector4 is serialized to a whitespace-delimited string in the form `x y z` or `x y z w` respectively.

A Matrix3 or Matrix4 is serialized to a whitespace-delimited string in row-major order, unless otherwise specified.  I.e., `r11 r12 r13 r21 r22 r23 r31 r32 r33`

A RigidTransform is serialized to a whitespace-delimited string with the rotation matrix in row-major order, followed by the translation.  I.e., `r11 r12 r13 r21 r22 r23 r31 r32 r33 t1 t2 t3`

## Geometry file formats

Geometries can be loaded from a variety of file formats.  The native triangle mesh format is Object File Format (OFF), which is a simple ASCII file format.  Klamp't also natively supports OBJ file format.  If Klamp't is compiled with Assimp support, it can also load a variety of other formats including STL, DAE, VMRL, etc.  Point clouds can be loaded from PCD files (v0.7), as specified by the Point Cloud Library (PCL).

Geometric primitives are stored in a string format (.geom).  TODO: document me

Geometry groups are stored in a string format (.group).   TODO: document me

## World (.xml) file format

Structure: an XML v1.0 file, containing robots, rigid objects, and terrains, as well as simulation parameters. Follows the following schema.

- `<world>`: top level element.
  - _Attributes_
    - `background` (`Vector4`, default light blue): sets the RGBA background color of the world. Each channel has the range [0,1]
  - _Children_
    - `<robot>`: adds a robot to the world.
      - Attributes
        - `name` (string, optional, default "Robot"): a string to be used as an identifier.
        - `file` (string): the Robot (.rob) file to be loaded. May be relative path, absolute path, or URL.
        - `config` (`Config`, optional): an initial configuration. Format: `N q1 ... qN` where N is the number of DOF in the robot.
    - `<rigidObject>`: adds a rigid object to the world. If the `file` attribute is not given, then the `geometry` child must be specified. Note: rotation attributes are applied in sequence.
      - _Attributes_
        - `file` (string, optional): the Rigid object (.obj) file to be loaded. May be relative path, absolute path, or URL.
        - `position` (`Vector3`, optional, default (0,0,0)): the position of the object center 
        - `rotateRPY` (`Vector3`, optional): rotates the object about the given roll-pitch-yaw entries.
        - `rotateX` (float, optional): rotates the object about the x axis.
        - `rotateY` (float, optional): rotates the object about the y axis.
        - `rotateZ` (float, optional): rotates the object about the z axis.
        - `rotateMoment` (`Vector3`, optional): rotates the object with a rotation matrix derived from the given exponential map representation.
      - _Children_
         - `<display>` or `<appearance>` (optional): configures the visualization of the object.  Default color is blue.
          - _Attributes_
            - `color` (`Vector3` or `Vector4`, optional): sets the RGB or RGBA color of the object.
            - `faceColor` (`Vector3` or `Vector4`, optional): sets the RGB or RGBA color of the object's faces.
            - `vertexColor` (`Vector3` or `Vector4`, optional): sets the RGB or RGBA color of the object's vertices (default not drawn, except for point clouds).
            - `vertexSize` or `pointSize` (float, optional): sets size of the points (in pixels) drawn at the object's vertices (default 3, for point clouds).
            - `edgeColor` (`Vector3` or `Vector4`, optional): sets the RGB or RGBA color of the object's edges (default not drawn).
            - `edgeSize` (float, optional): sets the width of the drawn edges.
            - `silhouette` (1, 4, or 5 floats, optional): configures the silhouette using a string of the form "width [r g b] [a]".  Default value is "0.0025 0 0 0 1".
            - `texture` (string, optional): sets a texture.  Can be an image file name, or "noise", "checker", "gradient", "colorgradient".
            - `texture_projection` (string, optional): sets a texture projection.  Can be "xy", "z", or "conformal" at the moment.
        - `<geometry>`: sets the object's geometry (optional).
          - _Attributes_
            - `file` or `mesh` (string): the geometry file (.off, other mesh, or .pcd).  May be relative path, absolute path, or URL  (Note: "mesh" is a misnomer, this works with any type of geometry file)
            - `scale` (float or `Vector3`, optional): a scale factor for the mesh.  If 3 elements are given, then this scales the mesh separately along each axis.
            - `translate` (`Vector3`, optional): a translation for the mesh.
            - `margin` (float, optional, default 0): the collision boundary layer width.
        - `<physics>`: sets the physics parameters of the object.
          - _Attributes_
            - `mass` (Real, optional, default 1): the object’s mass.
            - `com` (Vector3, optional, default (0,0,0)): the object’s center of mass, relative to the origin of its coordinate frame.
            - `inertia` (Matrix3, optional, default 0): the object’s inertia matrix.
            - `automass` (value "0" or "1", optional): the object’s COM and inertia matrix will be set automatically from the geometry.
            - `automassSurfaceFraction` (float, optional): if automass = 1, the COM and inertia will imagine this fraction of the volume of the geometry to be concentrated at the surface.
            - `kRestitution`, `kFriction`, `kStiffness`, `kDamping` (Reals, optional, defaults 0.5, 0.5, inf, inf): set the constitutive parameters of the object.
    - `<terrain>`: adds a terrain to the world.
      - _Attributes_
        - `file`: see `<world><rigidObject><geometry mesh>`
        - `scale`: see `<world><rigidObject><geometry scale>`
        - `margin`: see `<world><rigidObject><geometry scale>`
        - `translation`, `position`: see `<world><rigidObject position>`.
        - `rotate*`: see `<world><rigidObject><rotate*>`.
        - `kFriction`: see `<world><rigidObject><physics kFriction>`.
      - _Children_
        - `<display>` or `<appearance>` (optional): configures the visualization of the terrain (see `<rigidObject><display>`).  Default color is light brown.
    - `<simulation>` (optional): configures the simulation model.
      - _Children_
        - `<globals>` (optional): global ODE simulation parameters.
          - _Attributes_
            - `gravity` (`Vector3`, optional, default (0,0,-9.8)): sets the gravity vector
            - `CFM`: ODE's constraint force mixing parameter.
            - `ERP`: ODE's error reduction parameter.
            - `maxContacts` (int, optional, default 20): sets a maximum number of contacts per body-body contact.
            - `boundaryLayer` (bool, optional, default 1): activates boundary layer collision detection.
            - `rigidObjectCollisions` (bool, optional, default 1): activates object to object collision detection.
            - `robotSelfCollisions` (bool, optional, default 0): activates robot self-collision detection.
            - `robotRobotCollisions` (bool, optional, default 0): activates robot to robot collision detection.
        - `<terrain>` (optional): terrain configuration.
          - _Attributes_
            - `index` (int): the terrain index.  Either index or name must be specified.
            - `name` (str): the terrain name.  Either index or name must be specified.
          - _Children_
            - `<geometry>`: sets up the geometry and constitutive parameters
              - _Attributes_
                - `padding` (float, optional, default 0 for terrains, 0.0025 for everything else): sets the boundary layer thickness.
                - `kRestitution`, `kFriction`, `kStiffness`, `kDamping`: see `<world><rigidObject><physics k*>`
        - `<rigidObject>` (optional): rigid object configuration.  Also can be referred to by `<object>`
          - _Attributes_
            - `index` (int): the rigid object index.  Either index or name must be specified.
            - `name` (str): the rigid object name.  Either index or name must be specified.
          - _Children_
            - `<geometry>`: see `<world><simulation><terrain><geometry>`.
        - `<robot>` (optional): robot configuration
          - _Attributes_
            - `index` (int): the robot index.  Either index or name must be specified.
            - `name` (str): the robot name.  Either index or name must be specified.
            - `body` (int, optional, default -1): the link index. -1 applies the settings to the entire robot.
          - _Children_
            - `<geometry>`: see `<world><simulation><terrain><geometry>`.
            - `<controller>`: configures the robot's controller. Each controller type has a certain set of optional attributes that can be set here.
              - _Attributes_
                - `type` (string): the controller type. See the [controller documentation](Manual-Control.md#controllers) for more details.
                - `rate` (float, optional, default 100): rate at which the controller runs, in Hz.
                - `timeStep` (float, optional, default 0.01): 1/rate.
            - `<sensors>`: configures the robot's sensors.
              - _Children:_ Any of the sensor types listed in the [sensor documentation](Manual-Control.md#sensors)
        - `<joint>` (optional): adds a custom joint to the simulator.
          - _Attributes_
            - `type` (str): the joint type, either "fixed", "hinge", or "slider"
            - `axis` (3 floats): world space axis for "hinge" and "slider" joints 
            - `point` (3 floats): world space position for "hinge" joints
          - _Children_: one or two bodies to which the joint should be attached. If one body is specified, the body is attached to the world frame.
            - `<robot>` or `<rigidObject>` or `<terrain>`: an object to which the joint should be attached.  See `<world><simulation><robot>` or `<rigidObject>` or `<terrain>`.
      - `<state>`: resumes the simulator from some other initial state.
        - _Attributes_
          - `data` (string): Base64 encoded data from a prior `WorldSimulator.WriteState` call. Other than simulation state, the world file must be otherwise identical to the one that produced this data.


## Robot (.rob) files

**Structure**: a series of lines, separated by newlines.  Comments start with #, may appear anywhere on a line, and comments continue until the end of the line.  Lines can be continued to the next line using the backslash \\.

A robot has N links, and D drivers.  Elements of each line are whitespace-separated. Indices are zero-based.  inf indicates infinity.  Some items are optional, indicated by default values.

**Kinematic specification items**:

- `links LinkName[0] ... LinkName[N-1]`: link names, names with spaces can be enclosed in quotes.
- `parents parent[0] ... parent[N-1]`: link parent indices.  -1 indicates that a link's parent is the world frame.
- `jointtype v[0] ... v[N-1]`: DOF motion type, can be r for revolute or p for prismatic.
- `tparent T[0] ... T[N-1]`: relative rigid transforms between each link and its parent.  Each T[i] is a row-major list of entries of the rotation matrix, followed by the translation (12 values for each T).
- `{alpha, a, d, theta} v[0] ... v[N-1]`: Denavit-Hartenberg parameters. Either tparent or D-H parameters must be specified.  `alphadeg` is equivalent to `alpha` and `thetadeg` is equivalent to `theta`, but in degrees.
- `axis a[0] ... a[N-1]`: DOF axes, in the local frame of the link (3 values for each a).  Default: z axis (0,0,1).
- `qmin v[0] ... v[N-1]`: configuration lower limits, in radians.  `qmindeg` is equivalent, but in degrees. Default: -inf.
- `qmax v[0] ... v[N-1]`: configuration upper limits, in radians. `qmaxdeg` is equivalent, but in degrees. Default: inf.
- `q v[0] ... v[N-1]`: initial configuration values, in radians. `qdeg` is equivalent, but in degrees. Default: 0.
- `translation`: a shift of link 0. Default: (0, 0, 0).
- `rotation`: a rotation of link 0, given by a row-major list of entries of a 3x3 rotation matrix.  Default: identity.
- `scale`: scales the entire robot model.
- `mount link fn [optional transform T] [optional "as X"]`: mounts the sub-robot file in `fn` as a child of link `link`. 

  If `T` is provided, this is the relative transform of the sub-robot given by a row-major list of entries of rotation matrix followed by the translation (12 values in `T`).


  If `as X` is provided, with X a string, then all link names of the mounted sub-robot will be prefixed with "X:".

**Dynamic specification items**:

- `mass v[0] ... v[N-1]`: link masses.
- `automass`: set the link centers of mass and inertia matrices automatically from the link geometry.  Can also give `automass surfaceFraction` to specify that `surfaceFraction` fraction of the mass is concentrated at the geometry's surface.
- `com v[0] ... v[N-1]`: link centers of mass, given in local (x,y,z) coordinates (3 values for each v).  May be omitted if automass is included.
- `inertiadiag v[0] ... v[N-1]`:  link inertia matrix diagonals (Ixx, Iyy, Izz), assuming off-diagonal elements are all zero (3 values for each v).  May be omitted if `inertia` or `automass` is included.
- `inertia v[0] ... v[N-1]`: link 3x3 inertia matrices (9 items for each `v`).  May be omitted if inertiadiag or automass is included.
- `velmin v[0] ... v[N-1]`: configuration velocity lower limits, in radians. `velmindeg` is equivalent, but in degrees. Default: -inf.
- `velmax v[0] ... v[N-1]`: configuration velocity upper limits, in radians. `velmaxdeg` is equivalent, but in degrees. Default: inf.
- `accmax v[0] ... v[N-1]`: configuration acceleration absolute value limits, in radians. `accmaxdeg` is equivalent, but in degrees. Default: inf.
- `torquemax v[0] ... v[N-1]`: DOF torque absolute value limits, in Nm (revolute) or N (prismatic). Default: inf.
- `powermax v[0] ... v[N-1]`: DOF power (torque\*velocity) absolute value limits. Default: inf.
- `autotorque`: set the torquemax values according to an approximation: acceleration maxima \* masses \* radii of descendent links.


**Geometric items**:

- `geometry fn[0] ... fn[N-1]`: geometry files for each link. File names can be either absolute paths, relative paths, or URLs. Files with spaces can be enclosed in quotes.  Empty geometries can be specified using &quot;&quot;.
- `geomscale v[0] ... v[N-1]`: scales the link geometry.  Default: no scaling.
- `geomtransform index m11 m12 m13 m14 m21 m22 m23 m24 m31 m32 m33 m34 m41 m42 m43 m44`: transforms the link geometry with a 4x4 transformation matrix m with entries given in row-major order.
- `geommargin v[0] ... v[N-1]`: sets the collision geometry to have this virtual margin around each geometric mesh.  Default: 0.
- `noselfcollision i[0] j[0] ... i[k] j[k]`: turn off self-collisions between the indicated link pairs.  Each item may be a link index in the range 0,...,N-1 or a link name.
- `selfcollision i[0] j[0] ... i[k] j[k]`: turn on self-collisions between the indicated link pairs.  Each item may be a link index in the range 0,...,N-1 or a link name.  Default: all self-collisions enabled, except for link vs parent.

**Joint items**:

- `joint type index [optional baseindex]`: indicates how a group of link DOFs associated with link `index` should be interpreted.  If `baseindex` is specified, this indicates that the joint operates on a group of DOFs ranging from `baseindex` to `index`.  `type` indicates the type of joint, and can be
    - `normal` (1DOF interval)
    - `spin` (1DOF wrapping around from 0 to 2pi)
    - `weld` (0DOF)
    - `floating` (6DOF with 3 translational 1 rotational, `baseindex` must be specified)
    - `floatingplanar` (3DOF with 2 translational 1 rotational, `baseindex` must be specified)
    - `ballandsocket` (3DOF rotational, `baseindex` must be specified).

**Driver items**:

- `driver type [params]`: TODO: describe driver types `normal`, `affine`, `translation`, `rotation`.
- `servoP`: driver position gains.
- `servoI`: driver integral gains.
- `servoD`: driver derivative gains.
- `dryFriction`: driver dry friction coefficients.
- `viscousFriction`: driver viscous friction coefficients.

**Properties**:

- `property sensors [file or XML string]`: defines the robot's sensors either in an XML file or string. See the World XML format above or the [sensor documentation](Manual-Control.md#sensors) for more details on the XML format of this element.
- `property controller [file or XML string]`: defines the robot's controller either in an XML file or string. See the World XML format above or the [controller documentation](Manual-Control.md#controllers) for more details on the XML format of this element.



## URDF files (.urdf) with Klamp't-specific elements

URDF (Unified Robot Description Format) is a widely used XML-based robot format found in ROS and other packages. Klamp't has always been able to convert URDF files to .rob files, which can be edited to introduce Klamp't-specific attributes, like motor simulation parameters and ignoring certain self-collision pairs.  Starting in version 0.6, Klamp't can now read those attributes from URDF files with an extra `<klampt>` XML element.  The schema for defining this element is as follows:

- `<robot>`: top level element. Follows URDF format as usual.
  - _Children_
    - `<klampt>`: specifies Klamp't-specific parameters
      - _Attributes_
        - `use_vis_geom` (bool, optional, default false): use visualization geometry in imported model.
        - `flip_yz` (bool, optional, default true): flip the Y-Z axes of imported link geometries.
        - `package_root` (string, optional, default ".""): describe the path of the package described in any "package://" URI strings, relative to the URDF file.
        - `world_frame` (string, optional, default "world"): the name of the fixed world frame.
        - `freeze_root_link` (bool, optional, default false): if true, the root link is frozen in space (useful for debugging)
        - `default_mass` (float, optional, default 1e-8): default mass assigned to links not given mass parameters.
        - `default_inertia` (float, `Vector3`, or `Matrix3`, optional, default 1e-8): default inertia matrix assigned to links not given mass parameters.
      - _Children_
        - `<link>`: describes link parameters.
          - _Attributes_
            - `name` (string): identifies the link.
            - `physical` (bool, optional, default true): if set to 0, this is a virtual link with no mass.
            - `accMax` (float, optional, default inf): sets the acceleration maximum for this link.
            - `servoP`, `servoI`, `servoD` (float, optional, defaults 10, 0, 1): sets the PID gains of this joint (note: must be a normally driven link).
            - `dryFriction`, `viscousFriction` (float, optional, default 0): sets the friction constants for this joint.
        - `<noselfcollision>`: turns off self collisions.
          - _Attributes_
            - `pairs` (string, optional): identifies one or more pairs of links for which self-collision should be turned off.  Whitespace-separated. Each item can be an index or a link name.
            - `group1`,`group2` (string, optional): if `group1` and group2 are specified, collisions between all of the links in group 1 (a whitespace separated list of link indices or names) will be turned off.  Either `pairs` or both `group1` and `group2` must be present in the element.
        - `<selfcollision>`: turns on certain self collisions.  Note: if this item is present, default self collisions are not used.  _Same attributes as `<noselfcollisions>`._
        - `<sensors>`: specifies sensors to be attached to the robot. See the World XML format above or the [sensor documentation](Manual-Control.md#sensors) for more details on the XML format of this element.
        - <mount>`: mounts a geometry or another robot to a link.
          - _Attributes_
            - `link` (string): the name or integer index of the link.
            - `file` (string): the absolute path / relative path / URL of a geometry file (OFF, OBJ, STL, etc) or other robot file (.urdf or .rob).
            - `transform` (12 floats, optional): if provided, the relative transform of the sub-robot given by a row-major list of entries of rotation matrix followed by the translation (12 values in `T`).
            - `prefix` or `as` (string, optional): an alternative identifier X for the sub-robot.  If provided, then "X:" is prepended to all of its link names. (`as` added in 0.8.6)

## Piecewise Linear Path (.path) files

A piecewise linear path file has the following format:

```
t1   N q11 q12 ... q1N
t2   N q21 q22 ... q2N
...
tM   N qM1 qM2 ... qMN
```

Where the path is given by M points in time corresponding to M milestones, each of which is a `Config` of length N.  Each row consists of a time and a milestone.  It is assumed that t[k+1] >= t[k] for all k, and typically it is assumed that t1=0.
