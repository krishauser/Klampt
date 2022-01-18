"""A module for converting between Klamp't objects and ROS messages.
Tested on ROS Kinetic.

The easiest way to use this is to use the toMsg(klampt_obj) and
fromMsg(ros_obj) functions.  However, certain objects, like
JointState and JointTrajectory, have several configuration options
that may need to be specified.

Another simple way to use this is to use the publisher, subscriber,
object_publisher, object_subscriber, broadcast_tf, and listen_tf functions.

The C++ bindings allow you to subscribe to ROS PointCloud messages
natively (Geometry3D.loadFile("ros://...") or SubscribeToStream()).
It has not been determined whether using the functions in this module
with rospy interferes with those bindings.
"""

import rospy

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3,Point,Quaternion,Pose,Transform,PoseStamped,WrenchStamped
from trajectory_msgs.msg import JointTrajectory
from shape_msgs.msg import Mesh
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import LaserScan
from klampt.math import so3,se3
import math
import warnings

def from_Vector3(ros_v):
    """From ROS Vector3 to Klamp't point"""
    return [ros_v.x,ros_v.y,ros_v.z]

def to_Vector3(klampt_pt):
    """From Klamp't point to ROS Vector3"""
    return Vector3(klampt_pt[0],klampt_pt[1],klampt_pt[2])

def from_Point(ros_pt):
    """From ROS Point to Klamp't point"""
    return [ros_pt.x,ros_pt.y,ros_pt.z]

def to_Point(klampt_pt):
    """From Klamp't point to ROS Point"""
    return Point(klampt_pt[0],klampt_pt[1],klampt_pt[2])

def from_Quaternion(ros_q):
    """From ROS Quaternion to Klamp't so3 element"""
    q = [ros_q.w,ros_q.x,ros_q.y,ros_q.z]
    return so3.from_quaternion(q)

def to_Quaternion(klampt_so3):
    """From Klamp't so3 element to ROS Quaternion"""
    q = so3.quaternion(klampt_so3)
    ros_q = Quaternion()
    ros_q.w,ros_q.x,ros_q.y,ros_q.z = q
    return ros_q

def from_Pose(ros_pose):
    """From ROS Pose to Klamp't se3 element"""
    return (from_Quaternion(ros_pose.orientation),from_Point(ros_pose.position))

def to_Pose(klampt_se3):
    """From Klamp't se3 element to ROS Pose"""
    ros_pose = Pose()
    ros_pose.orientation = to_Quaternion(klampt_se3[0])
    ros_pose.position = to_Point(klampt_se3[1])
    return ros_pose

def from_Transform(ros_transform):
    """From tf.Transform to Klampt se3 element"""
    return (from_Quaternion(ros_transform.orientation),from_Point(ros_transform.translation))

def to_Transform(klampt_se3):
    """From Klampt se3 element to ROS Transform """
    ros_pose = Transform()
    ros_pose.orientation = to_Quaternion(klampt_se3[0])
    ros_pose.translation.x,ros_pose.translation.y,ros_pose.translation.z = klampt_se3[1]
    return ros_pose

def from_PoseStamped(ros_pose):
    """From ROS PoseStamped to Klamp't se3 element"""
    return (from_Quaternion(ros_pose.orientation),from_Point(ros_pose.position))

def to_PoseStamped(klampt_se3,stamp='now'):
    """From Klamp't se3 element to ROS PoseStamped
    """
    if stamp=='now':
        stamp = rospy.Time.now()
    elif isinstance(stamp,(int,float)):
        stamp = rospy.Time(stamp)
    ros_pose = PoseStamped()
    ros_pose.header.stamp = stamp
    ros_pose.orientation = to_Quaternion(klampt_se3[0])
    ros_pose.position = to_Point(klampt_se3[0])
    return ros_pose

def from_JointState(ros_js,robot,joint_link_indices=None):
    """From ROS JointState to Klamp't robot configuration.
    The Klamp't robot must be given, and any indices not specified
    in ros_js are kept at their current values

    Args:
        ros_js (sensor_msgs.msg.JointState): the JointState object
        robot (RobotModel): the robot
        joint_link_indices (dict, optional): if given, maps ROS joint names to
            Klampt link indices.  Default uses the Klamp't link names.

    Returns:
        tuple: a pair (config,velocity,effort).  Each item may be None if the
        JointState message does not contain position, velocity, or effort
        info.
    """
    if joint_link_indices is None:
        joint_link_indices = dict()
        for i in range(robot.numLinks()):
            joint_link_indices[robot.link(i).getName()] = i
    q = robot.getConfig() if len(ros_js.position) > 0 else None
    dq = robot.getVelocity() if len(ros_js.velocity) > 0 else None
    effort = [0.0]*robot.numLinks() if len(ros_js.effort) > 0 else None
    for i,name in enumerate(ros_js.name):
        if name not in joint_link_indices:
            raise ValueError("ROS JointState message has invalid name "+name)
        if len(ros_js.position) > 0:
            q[joint_link_indices[name]] = ros_js.position[i]
        if len(ros_js.velocity) > 0:
            dq[joint_link_indices[name]] = ros_js.velocity[i]
        if len(ros_js.effort) > 0:
            effort[joint_link_indices[name]] = ros_js.effort[i]
    return (q,dq,effort)

def to_JointState(robot,q='current',dq='current',effort=None,indices='auto',link_joint_names=None):
    """Returns a ROS JointState message for a Klamp't robot or controller.

    Args:
        robot (RobotModel or SimRobotController): the robot
        q (str or Config, optional): either 'current', 'commanded', 'sensed',
            'actual', None, or a configuration of size robot.numLinks(). 

            'commanded', 'sensed', and 'actual' are available only when robot
            is a SimRobotController.

        dq (str or Config, optional): either 'current', 'commanded', 'sensed',
            'actual', None, or a joint velocity vector of size robot.numLinks()

            'commanded', 'sensed', and 'actual' are available only when robot
            is a SimRobotController.

        effort (str or Config, optional): either 'commanded', sensed',
            'actual', None, or a torque vector of size robot.numDrivers().

        indices (str or list of int): if 'auto', all elements are set.
            Otherwise, only these indices are set.

        link_joint_names (list of str, optional): if given, the i'th link is
            mapped to the ROS joint name link_joint_names[i].
    """
    from klampt.robotsim import SimRobotController
    if q == 'current':
        if isinstance(robot,SimRobotController):
            q = robot.getSensedConfig()
        else:
            q = robot.getConfig()
    elif q == 'commanded':
        q = robot.getCommandedConfig()
    elif q == 'sensed':
        q = robot.getSensedConfig()
    elif q == 'actual':
        q = robot.sim.getActualConfig(robot.index)
    if dq == 'current':
        if isinstance(robot,SimRobotController):
            dq = robot.getSensedVelocity()
        else:
            dq = robot.getVelocity()
    elif dq == 'commanded':
        dq = robot.getCommandedVelocity()
    elif dq == 'sensed':
        dq = robot.getSensedVelocity()
    elif dq == 'actual':
        dq = robot.sim.getActualVelocity(robot.index)
    if effort == 'commanded':
        effort = robot.getCommandedTorque()
    elif effort == 'sensed':
        effort = robot.getSensedTorque()
    elif effort == 'actual':
        effort = robot.sim.getActualTorque(robot.index)

    if isinstance(robot,SimRobotController):
        robot = robot.model()

    #convert driver-level efforts to link-level efforts
    if q is not None and len(q) != robot.numLinks():
        raise ValueError("Invalid size of q vector")
    if dq is not None and len(dq) != robot.numLinks():
        raise ValueError("Invalid size of dq vector")
    if effort is not None and len(effort) != robot.numLinks():
        if len(effort) != robot.numDrivers():
            raise ValueError("Invalid size of effort vector")
        linkeffort = [0.0]*robot.numLinks()
        k = 0
        for i in range(robot.numDrivers()):
            d = robot.driver(i)
            coefs = None if d.getType() != 'affine' else d.getAffineCoeffs()[0]
            for j,l in enumerate(d.getAffectedLinks()):
                if coefs is None:
                    linkeffort[l] = effort[k+j]
                else:
                    linkeffort[l] = coefs[j]*effort[k+j]
            k += len(d.getAffectedLinks())
        effort = linkeffort

    if indices == 'auto':
        indices = list(range(robot.numLinks()))
    js = JointState()
    for i,link in enumerate(indices):
        if link_joint_names is not None:
            name = link_joint_names[link]
        else:
            name = robot.link(link).getName()
        js.name.append(name)
        if q is not None:
            js.position.append(q[link])
        if dq is not None:
            js.velocity.append(dq[link])
        if effort is not None:
            js.effort.append(effort[link])
    return js

def from_Float32MultiArray(ros_msg):
    """From ROS Float32MultiArray to Klamp't Config"""
    return [v for v in ros_msg.data]

def to_Float32MultiArray(klampt_config):
    res = Float32MultiArray()
    for v in klampt_config:
        res.data.append(v)
    return res

def from_JointTrajectory(ros_traj,robot=None,joint_link_indices=None):
    """Returns a Klamp't Trajectory or RobotTrajectory for a JointTrajectory
    message.

    Args:
        ros_traj (sensor_msgs.msg.JointTrajectory): the JointTrajectory object
        robot (RobotModel): the robot, optional
        joint_link_indices (dict, optional): if given, maps ROS joint names to
            Klampt link indices.  Default uses the Klamp't link names.

    """
    from klampt.model.trajectory import Trajectory,RobotTrajectory
    if robot is None:
        res = Trajectory()
        for i in range(len(ros_traj.points)):
            res.times.append(ros_traj.points[i].time_from_start.toSec())
            res.milestones.append([v for v in ros_traj.points[i].positions])
        return res
    else:
        res = RobotTrajectory(robot)
        q0 = robot.getConfig()
        if joint_link_indices is None:
            joint_link_indices = dict()
            for i in range(robot.numLinks()):
                joint_link_indices[robot.link(i).getName()] = i
        indices = []
        if len(ros_traj.joint_names) > 0:
            for n in ros_traj.joint_names:
                if n not in joint_link_indices:
                    raise ValueError("ROS joint name %s is invalid"%(n,))
                indices = joint_link_indices[n]
        for i in range(len(ros_traj.points)):
            res.times.append(ros_traj.points[i].time_from_start.toSec())
            for j,k in enumerate(indices):
                q0[k] = ros_traj.points[i].positions[j]
            res.milestones.append([v for v in q0])
        return res

def to_JointTrajectory(klampt_traj,indices='auto',link_joint_names=None):
    """Returns a ROS JointTrajectory message for a Klamp't Trajectory or
    RobotTrajectory.

    Args:
        klampt_traj (Trajectory or RobotTrajectory): the trajectory
        indices (str or list of ints): the indices to send (only valid with
            RobotTrajectory)
        link_joint_names (list of str, optional): if given, the i'th link is
            mapped to the ROS joint name link_joint_names[i].
    """
    from trajectory_msgs.msg import JointTrajectoryPoint
    res = JointTrajectory()
    if len(klampt_traj.milestones) == 0:
        res.joint_names = []
        res.points = []
        return res
    if not hasattr(klampt_traj,'robot'):
        if link_joint_names is None:
            for i in range(len(klampt_traj.milestones[0])):
                res.joint_names.append(chr(ord('0')+i))
        else:
            assert len(link_joint_names) == len(klampt_traj.milestones[0])
            for i in range(len(klampt_traj.milestones[0])):
                res.joint_names.append(link_joint_names)
        for i,q in enumerate(klampt_traj.milestones):
            res.points.append(JointTrajectoryPoint())
            res.points[-1].time_from_start = rospy.Duration(klampt_traj.times[i])
            res.points[-1].positions = q
        return res
    else:
        #TODO: hermite trajectories?
        if indices == 'auto':
            indices = list(range(klampt_traj.robot.numLinks()))
        if link_joint_names is None:
            link_joint_names = [klampt_traj.robot.link(i).getName() for i in range(klampt_traj.robot.numLinks())]
        for i,link in enumerate(indices):
            res.joint_names.append(link_joint_names[link])
        if len(indices) != len(klampt_traj.milestones[0]):
            raise ValueError("Path doesn't have same number of milestones as the robot")
        for i,q in enumerate(klampt_traj.milestones):
            res.points.append(JointTrajectoryPoint())
            res.points[-1].time_from_start = rospy.Duration(klampt_traj.times[i])
            for link in indices:
                res.points[-1].positions.append = klampt_traj.milestones[i][link]
        return res

def to_Path(klampt_path,start_time='now',frame='1'):
    """Converts a Klampt SE3Trajectory, SO3Trajectory, or 2D or 3D Trajectory
    to a ROS Path.

    start_time can be 'now' or a rospy.Time object.

    frame is the frame_id in the object's headers (1 means global frame).
    """
    from klampt.model.trajectory import SO3Trajectory,SE3Trajectory
    rospath = Path()
    if start_time == 'now':
        start_time = rospy.Time.now()
    elif isinstance(start_time,(int,float)):
        start_time = rospy.Time(start_time)
    rospath.header.stamp = start_time
    rospath.header.frame_id = frame
    if len(klampt_path.milestones) == 0:
        return rospath
    milestone_to_se3 = None
    if isinstance(klampt_path,SE3Trajectory):
        milestone_to_se3 = lambda x:klampt_path.to_se3(x)
    elif isinstance(klampt_path,SO3Trajectory):
        milestone_to_se3 = lambda x:(x,[0,0,0])
    else:
        if len(klampt_path.milestones[0]) not in [2,3]:
            raise ValueError("Only 3D trajectories are valid")
        ident = so3.identity()
        if len(klampt_path.milestones[0]) == 2:
            milestone_to_se3 = lambda x:(ident,[x[0],x[1],0.0])
        else:
            milestone_to_se3 = lambda x:(ident,x)
    for t,x in zip(klampt_path.times,klampt_path.milestones):
        T = milestone_to_se3(x)
        Ps = PoseStamped()
        Ps.pose = to_Pose(T)
        Ps.header.seq = len(rospath.poses)
        Ps.header.stamp = start_time + t
        Ps.header.frame_id = frame
        rospath.poses.append(Ps)
    return rospath

def from_Path(ros_path):
    """Converts a ROS Path to a Klamp't SE3Trajectory.

    Times are assigned relative to header's time stamp.
    """
    from klampt.model.trajectory import SE3Trajectory
    times = []
    milestones = []
    for p in ros_path.poses:
        times.append(float(p.header.stamp - ros_path.header.stamp))
        milestones.append(from_Pose(p.pose))
    return SE3Trajectory(times,milestones)

def from_Mesh(ros_mesh):
    """From a ROS Mesh to a Klampt TriangleMesh"""
    from klampt import TriangleMesh
    mesh = TriangleMesh()
    mesh.vertices.resize(len(ros_mesh.vertices)*3)
    mesh.indices.resize(len(ros_mesh.triangles)*3)
    for i,v in enumerate(ros_mesh.vertices):
        k=i*3
        mesh.vertices[k] = v.x
        mesh.vertices[k+1] = v.y
        mesh.vertices[k+2] = v.z
    for i,t in enumerate(ros_mesh.triangles):
        k=i*3
        mesh.indices[k] = t.vertex_indices[0]
        mesh.indices[k+1] = t.vertex_indices[1]
        mesh.indices[k+2] = t.vertex_indices[2]
    return mesh

def to_Mesh(klampt_mesh):
    """From a Klampt Geometry3D or TriangleMesh to a ROS Mesh"""
    from klampt import Geometry3D,TriangleMesh
    from shape_msgs.msg import MeshTriangle
    if isinstance(klampt_mesh,Geometry3D):
        mesh = klampt_mesh.getTriangleMesh()
        T = klampt_mesh.getCurrentTransform()
        if T != se3.identity():
            mesh.transform(*T)
        klampt_mesh = mesh
    ros_mesh = Mesh()
    for i in range(len(klampt_mesh.vertices)//3):
        k = i*3
        ros_mesh.vertices.append(Vector3(klampt_mesh.vertices[k],klampt_mesh.vertices[k+1],klampt_mesh.vertices[k+2]))
    for i in range(len(klampt_mesh.indices)//3):
        k = i*3
        ros_mesh.triangles.append(MeshTriangle([klampt_mesh.indices[k],klampt_mesh.indices[k+1],klampt_mesh.indices[k+2]]))
    return ros_mesh

def from_PointCloud2(ros_pc):
    """From a ROS PointCloud2 to a Klampt PointCloud"""
    from klampt import PointCloud
    from sensor_msgs.msg import PointField
    from sensor_msgs import point_cloud2
    pc = PointCloud()
    if ros_pc.header.seq > 0:
        pc.setSetting("id",str(ros_pc.header.seq))
    if ros_pc.fields[0].name != 'x' or ros_pc.fields[1].name != 'y' or ros_pc.fields[2].name != 'z':
        raise ValueError("PointCloud2 message doesn't have x,y,z fields?")
    for i in range(3,len(ros_pc.fields)):
        pc.addProperty(ros_pc.fields[i].name)
    N = ros_pc.height*ros_pc.width
    plist = []
    proplist = []
    if ros_pc.height > 1:
        structured = True
        pc.settings.set("width",str(pc.width));
        pc.settings.set("height",str(pc.height));
    for p in point_cloud2.read_points(ros_pc):
        plist += [p[0],p[1],p[2]]
        proplist += [float(v) for v in p[3:]]
    pc.setPoints(len(plist)//3,plist)
    pc.setProperties(proplist)
    return pc

def to_PointCloud2(klampt_pc,frame='0',stamp='now'):
    """Converts a Klampt Geometry3D or PointCloud to a ROS PointCloud2 message.

    If it's a Geometry3D, the points are first transformed by the current
    transform.

    frame is the ROS frame in the header.  Default '0' indicates no frame.

    stamp is the ROS timestamp, or 'now', or a numeric value.
    """
    from klampt import Geometry3D, PointCloud
    from sensor_msgs.msg import PointField
    from sensor_msgs import point_cloud2
    from std_msgs.msg import Header
    if stamp=='now':
        stamp = rospy.Time.now()
    elif isinstance(stamp,(int,float)):
        stamp = rospy.Time(stamp)
    if isinstance(klampt_pc,Geometry3D):
        pc = klampt_pc.getPointCloud()
        T = klampt_pc.getCurrentTransform()
        if T != se3.identity():
            pc.transform(*T)
        klampt_pc = pc
    if not isinstance(klampt_pc,PointCloud):
        raise TypeError("Must provide a klampt.PointCloud object")
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
    for i in range(len(klampt_pc.propertyNames)):
        fields.append(PointField(klampt_pc.propertyNames[i],12+i*4,PointField.FLOAT32,1))
    import numpy
    points = numpy.array(klampt_pc.vertices).reshape((klampt_pc.numPoints(),3))
    alldata = points
    if len(fields) > 3:
        properties = numpy.array(klampt_pc.properties).reshape((klampt_pc.numPoints(),len(klampt_pc.propertyNames)))
        alldata = numpy.hstack((points,properties))
    seq_no = 0
    try:
        seq_no = klampt_pc.getSetting('id')
    except Exception:
        pass
    ros_pc = point_cloud2.create_cloud(Header(stamp=stamp,seq=seq_no,frame_id=frame),fields,alldata)
    try:
        w = klampt_pc.getSetting('width')
        h = klampt_pc.getSetting('height')
        ros_pc.height = int(w)
        ros_pc.height = int(h)
    except:
        ros_pc.height = 1
        ros_pc.width = klampt_pc.numPoints()
    return ros_pc


def to_CameraInfo(klampt_obj):
    """Converts a Klampt SimRobotSensor, GLViewport, or Viewport to a ROS
    CameraInfo message.
    """
    from sensor_msgs.msg import CameraInfo
    from klampt import SimRobotSensor,Viewport
    msg = CameraInfo()
    msg.distortion_model = "plumb_bob";
    msg.D = [0.0]*5
    if isinstance(klampt_obj,SimRobotSensor):
        if klampt_obj.type() != 'CameraSensor':
            raise ValueError("Can't publish CameraInfo with a non-camera sensor")
        msg.width = klampt_obj.getSetting('xres');
        msg.height = klampt_obj.getSetting('yres');    
        fx = 0.5*klampt_obj.getSetting('xres')/math.tan(klampt_obj.getSetting('xfov')*0.5);
        fy = 0.5*klampt_obj.getSetting('yres')/math.tan(klampt_obj.getSetting('yfov')*0.5);
    elif isinstance(klampt_obj,Viewport):
        msg.width = klampt_obj.w
        msg.height = klampt_obj.h
        fx = klampt_obj.scale
        fy = klampt_obj.scale
    elif hasattr(klampt_obj,'to_viewport'):
        vp = klampt_obj.to_viewport()
        msg.width = vp.w
        msg.height = vp.h
        fx = vp.scale
        fy = vp.scale
    else:
        raise ValueError("Invalid object type: "+klampt_obj.__class__.__name__)
    cx = 0.5*msg.width
    cy = 0.5*msg.height
    msg.K[0] = fx;
    msg.K[4] = fy;
    msg.K[8] = 1;
    msg.K[3] = cx;
    msg.K[7] = cy;
    msg.R[0] = 1;
    msg.R[4] = 1;
    msg.R[8] = 1;
    msg.P[0] = fx;
    msg.P[5] = fy;
    msg.P[10] = 1;
    msg.P[3] = cx;
    msg.P[8] = cy;
    return msg

def from_CameraInfo(ros_ci,klampt_obj):
    """Fills in some information about a Klampt SimRobotSensor, GLViewport,
    or Viewport using a ROS CameraInfo message.  Modifies the object in-place.
    """
    from klampt import SimRobotSensor,Viewport
    fx = ros_ci.K[0]
    fy = ros_ci.K[4]
    w = ros_ci.width
    h = ros_ci.height
    x = 0
    y = 0
    if ros_ci.roi.x_offset != 0 or ros_ci.roi.y_offset != 0:
        raise NotImplementedError("TODO: set viewport with an ROI")
    if isinstance(klampt_obj,SimRobotSensor):
        if klampt_obj.type() != 'CameraSensor':
            raise ValueError("Can't publish CameraInfo with a non-camera sensor")
        klampt_obj.setSetting('xres',str(w));
        klampt_obj.setSetting('yres',str(h)); 
        klampt_obj.setSetting('xfov',str(2*math.atan(0.5*w/fx)))
        klampt_obj.setSetting('yfov',str(2*math.atan(0.5*h/fy)))
    elif isinstance(klampt_obj,Viewport):
        klampt_obj.x = x
        klampt_obj.y = y
        klampt_obj.w = w
        klampt_obj.h = h
        if fx != fy:
            warnings.warn("from_CameraInfo: can't handle non-square pixels in Viewport")
        klampt_obj.scale = fx
    elif hasattr(klampt_obj,'to_viewport'):
        klampt_obj.x = x
        klampt_obj.y = y
        klampt_obj.w = w
        klampt_obj.h = h
        if fx != fy:
            warnings.warn("from_CameraInfo: can't handle non-square pixels in GLViewport")
        klampt_obj.xfov = 2*math.atan(0.5*w/fx)
    else:
        raise ValueError("Invalid object type: "+klampt_obj.__class__.__name__)
    return klampt_obj

def to_SensorMsg(klampt_sensor,frame=None,frame_prefix='klampt',stamp='now'):
    """Converts a sensor's measurements to a ROS message(s).

    Special types are CameraSensor, ForceTorqueSensor, and LaserRangeSensor.

    * CameraSensor is converted to up to three messages: CameraInfo,
      Image (rgb, optional), and Image (depth, optional).
    * ForceTorqueSensor is converted to a WrenchStamped.

    Generic sensors are converted to a Float32MultiArray.

    Args:
        klampt_sensor (SimRobotSensor): the sensor
        frame (str, optional): if given, this is the frame_id used in the 
            ROS message(s).  Otherwise, the id is determined automatically.
        frame_prefix (str, optional): if frame is not given, this is the
            prefix used in the automatic frame_id assignment (see below)
        stamp (str, float, or rospy.Time, optional): can be 'now', a float,
            or a rospy.Time.  Will be set in the ROS message header.

    The ROS ``frame_id`` is set as follows:
    
    - If frame!=None, then ``frame_id=frame``.
    - If frame==None and frame_prefix==None, then
      ``frame_id = [robot_name]/[sensor_name]``.
    - Otherwise,
      ``frame_id = [frame_prefix]/[robot_name]/[sensor_name]``.

    """
    if frame is None:
        frame = ""
        if frame_prefix is not None:
            frame = frame_prefix + '/'
        else:
            frame = '.'
        frame += klampt_sensor.robot().getName()+'/' + klampt_sensor.name()

    stype = klampt_sensor.type()
    if stype == 'CameraSensor':
        import numpy 
        import sys

        camera = klampt_sensor
        
        ci = to_CameraInfo(camera)
        ci.frame_id = frame

        msgs = [ci]
        measurements = camera.getMeasurements()
        if len(measurements) == 0:
            return msgs
        
        sys_bigendian = (sys.byteorder == 'big')
        if int(camera.getSetting('rgb')):
            msg = Image()
            msg.header.frame_id = frame
            w = msg.width = int(camera.getSetting('xres'));
            h = msg.height = int(camera.getSetting('yres'));
            msg.encoding = "rgb8";
            msg.is_bigendian = (numpy.uint32.byteorder == '>' or (numpy.uint32.byteorder == '=' and sys_bigendian))
            msg.step = msg.width*3
            abgr = numpy.array(measurements[0:w*h]).reshape(h,w).astype(numpy.uint32)
            msg.data = abgr.tostring()
            msgs.append(msg)
        if int(camera.getSetting('depth')):
            msg = Image()
            msg.header.frame_id = frame
            msg.width = int(camera.getSetting('xres'));
            msg.height = int(camera.getSetting('yres'));
            msg.encoding = "32FC1";
            msg.is_bigendian = (numpy.uint32.byteorder == '>' or (numpy.float32.byteorder == '=' and sys_bigendian))
            msg.step = msg.width*4
            ofs = 0
            w = int(camera.getSetting('xres'))
            h = int(camera.getSetting('yres'))
            if int(camera.getSetting('rgb')):
                ofs = w*h
            abgr = numpy.array(measurements[ofs:ofs+w*h]).reshape(h,w).astype(numpy.float32)
            msg.data = abgr.tostring()
            msgs.append(msg)
        return msgs
    elif stype == 'ForceTorqueSensor':
        measurements = klampt_sensor.getMeasurements()
        if len(measurements)==0:
            measurements = [0.0]*6
        assert (measurements.size() == 6),"Error, F/T sensor isn't returning 6 measurements?"
        res = WrenchStamped()
        if stamp=='now':
            stamp = rospy.Time.now()
        elif isinstance(stamp,(int,float)):
            stamp = rospy.Time(stamp)
        res.header.frame_id = frame
        res.header.stamp = stamp
        res.wrench.force.x = measurements[0]
        res.wrench.force.y = measurements[1]
        res.wrench.force.z = measurements[2]
        res.wrench.torque.x = measurements[3]
        res.wrench.torque.y = measurements[4]
        res.wrench.torque.z = measurements[5]
        return res
    elif stype == 'LaserRangeSensor':
        measurements = klampt_sensor.getMeasurements()
        res = LaserScan()
        if stamp=='now':
            stamp = rospy.Time.now()
        elif isinstance(stamp,(int,float)):
            stamp = rospy.Time(stamp)
        res.header.frame_id = frame
        res.header.stamp = stamp
        res.angle_max = float(klampt_sensor.getSetting("xSweepMagnitude"))
        res.angle_min = -1.0 * res.angle_max
        measurement_count = float(klampt_sensor.getSetting("measurementCount"))
        res.angle_increment = (res.angle_max - res.angle_min)/measurement_count
        res.time_increment = float(klampt_sensor.getSetting("xSweepPeriod"))
        res.range_min = float(klampt_sensor.getSetting("depthMinimum"))
        res.range_max = float(klampt_sensor.getSetting("depthMaximum"))
        res.ranges = measurements
        res.intensities = []
        return res
    else:
        measurements = klampt_sensor.getMeasurements()
        return to_Float32MultiArray(measurements)

def to_ShapeMsg(klampt_geom):
    if klampt_geom.type() == 'PointCloud':
        return to_PointCloud2(klampt_geom)
    elif klampt_geom.type() == 'TriangleMesh':
        return to_Mesh(klampt_geom)
    else:
        raise ValueError("Implicit surfaces, geometric primitives, and groups not supported yet")

SUPPORTED_KLAMPT_TYPES = {
    'Vector3':'Vector3',
    'Point':'Point',
    'Matrix3':'Quaternion',
    'Rotation':'Quaternion',
    'Config':'Float32MultiArray',
    'RigidTransform':'Pose',
    'RobotModel':'JointState',
    'SimRobotController':'JointState',
    'Trajectory':'JointTrajectory',
    'RobotTrajectory':'JointTrajectory',
    'PointCloud':'PointCloud2',
    'TriangleMesh':'Mesh',
    'Geometry3D':'ShapeMsg',
    'SimRobotSensor':'SensorMsg',
    'Viewport':'CameraInfo',
    'GLViewport':'CameraInfo',
}

def _compatibleKlamptType(klampt_obj):
    from ..model import types
    if klampt_obj.__class__.__name__ in SUPPORTED_KLAMPT_TYPES:
        return klampt_obj.__class__.__name__
    otype = types.object_to_type(klampt_obj,SUPPORTED_KLAMPT_TYPES)
    if otype is None:
        raise ValueError("Don't know how to convert Klampt object of type "+",".join(types.object_to_types(klampt_obj)))
    if otype not in SUPPORTED_KLAMPT_TYPES:
        raise ValueError("Don't know how to convert Klampt object of type "+otype)
    return otype

def _from_converter(ros_obj):
    converter = 'from_'+ros_obj.__class__.__name__
    if converter in globals():
        return globals()[converter]
    raise ValueError("Don't know how to convert ROS messages of type "+ros_obj.__class__.__name__)

def _from_converter2(klampt_obj):
    type = _compatibleKlamptType(klampt_obj)
    converter = 'from_' + SUPPORTED_KLAMPT_TYPES[type]
    assert converter in globals(),"Can't convert from ROS message type "+SUPPORTED_KLAMPT_TYPES[type]
    return globals()[converter]

def _to_converter(klampt_obj):
    type = _compatibleKlamptType(klampt_obj)
    converter = 'to_' + SUPPORTED_KLAMPT_TYPES[type]
    assert converter in globals()
    return globals()[converter]

def fromMsg(ros_obj,*args,**kwargs):
    """General conversion from ROS messages to corresponding Klamp't objects."""
    _from_converter(ros_obj)(ros_obj,*args,**kwargs)

def toMsg(klampt_obj,*args,**kwargs):
    """General conversion from Klamp't objects to corresponding ROS messages."""
    return _to_converter(klampt_obj)(klampt_obj,*args,**kwargs)

class KlamptROSPublisher(rospy.Publisher):
    def __init__(self,converter,*args,**kwargs):
        """You shouldn't need to use this explicitly. Use :meth:`publisher` or
        :meth:`object_publisher` instead.
        """
        self.converter=converter
        self.seq_no = 0
        rospy.Publisher.__init__(self,*args,**kwargs)

    def publish(self,klampt_obj,header=None):
        """Publishes a Klamp't object to the advertised topic.

        Args:
            klampt_obj: A Klamp't object of the appropriate type
            header (Header, optional): a ROS header that overrides the
                default which stamps with the current time.
        """
        self.seq_no += 1
        rosobj = self.converter(klampt_obj)
        if hasattr(rosobj,'header'):
            if header is not None:
                rosobj.header = header
            else:
                now = rospy.Time.now()
                rosobj.header.seq = self.seq_no
                if rosobj.header.stamp == 0:
                    rosobj.header.stamp = now
                if len(rosobj.header.frame_id)==0:
                    rosobj.header.frame_id = '0'
        rospy.Publisher.publish(self,rosobj)

class KlamptROSCameraPublisher:
    def __init__(self,topic,frame_id,frame_prefix,*args,**kwargs):
        """You shouldn't need to use this explicitly. Use 
        :meth:`object_publisher` instead.
        """
        self.topic = topic
        self.frame_prefix = frame_prefix
        self.frame_id = frame_id
        self.rgbpubinfo = None
        self.rgbpub = None
        self.dpubinfo = None
        self.dpub = None
        self.pubargs = args
        self.pubkwargs = kwargs
        self.num_msgs = 0

    def publish(self,camera):
        """Publishes a Klamp't camera data to the topics:

        - [topic]/rgb/camera_info
        - [topic]/rgb/image_rect_color
        - [topic]/depth_registered/camera_info
        - [topic]/depth_registered/image_rect

        Args:
            camera (SimRobotSensor): an updated sensor of 'CameraSensor' type.
        """
        
        assert camera.type() == 'CameraSensor'
        if self.frame_id is None:
            frame = ""
            if self.frame_prefix is not None:
                frame = self.frame_prefix + '/'
            frame = frame + camera.robot.getName()
            link = int(camera.getSetting("link"))
            frame += '/' + camera.robot.link(link).getName()
            self.frame_id = frame

        msgs = to_SensorMsg(camera,frame=self.frame_id)
        if len(msgs) <= 1:
            #no measurements
            return
        self.num_msgs += 1
        for i in range(1,len(msgs)):
            msgs[i].header.stamp = rospy.Time.now()
            msgs[i].header.seq = self.num_msgs
        
        offset = 1
        if int(camera.getSetting('rgb')):
            if self.rgbpub is None:
                self.rgbpubinfo = rospy.Publisher(self.topic+'/rgb/camera_info',CameraInfo,*self.pubargs,**self.pubkwargs)
                self.rgbpub = rospy.Publisher(self.topic+'/rgb/image_rect_color',Image,*self.pubargs,**self.pubkwargs)
            self.rgbpubinfo.publish(msgs[0])
            self.rgbpub.publish(msgs[1])
            offset = 2
        if int(camera.getSetting('depth')):
            if self.dpub is None:
                self.dpubinfo = rospy.Publisher(self.topic+'/depth_registered/camera_info',CameraInfo,*self.pubargs,**self.pubkwargs)
                self.dpub = rospy.Publisher(self.topic+'/depth_registered/image_rect',Image,*self.pubargs,**self.pubkwargs)
            self.dpubinfo.publish(msgs[0])
            self.dpub.publish(msgs[offset])
            

def publisher_SimRobotSensor(topic,klampt_sensor,convert_kwargs=None,**kwargs):
    stype = klampt_sensor.type()
    if stype == 'CameraSensor':
        frame_id = None
        frame_prefix = None
        if convert_kwargs is not None:
            frame_id = convert_kwargs.get('frame',None)  
            frame_prefix = convert_kwargs.get('frame_prefix',None)
        return KlamptROSCameraPublisher(topic,frame_id,frame_prefix,**kwargs)
    elif stype == 'ForceTorqueSensor':
        ros_type = 'WrenchStamped'
    elif stype == 'LaserRangeSensor':
        ros_type = 'LaserScan'
    else:
        ros_type = 'Float32MultiArray'
    converter = to_SensorMsg
    ros_msg_class = globals()[ros_type]
    return KlamptROSPublisher((lambda rosobj,converter=converter,kwargs=convert_kwargs:converter(rosobj,**kwargs)),
        topic,ros_msg_class,**kwargs)

def publisher(topic,klampt_type,convert_kwargs=None,ros_type=None,**kwargs):
    """Convenience function. The publisher can be called in the form
    pub.publish(klampt_obj), which will convert a klampt_obj to a ROS message
    before publishing to a topic.

    Returns:
        KlamptROSPublisher
    """
    if convert_kwargs is None:
        convert_kwargs = dict()
    if not isinstance(klampt_type,str):
        klampt_type = klampt_type.__class__.__name__
    if ros_type is None:
        ros_type = SUPPORTED_KLAMPT_TYPES[klampt_type]
    if ros_type in ['SensorMsg','ShapeMsg']:
        raise ValueError("Klamp't object is ambiguous, need to specify a ROS type")
    converter = 'to_' + ros_type
    assert converter in globals(),"Can't convert from ROS message type "+ros_type
    converter = globals()[converter]
    ros_msg_class = globals()[ros_type]
    return KlamptROSPublisher((lambda rosobj,converter=converter,kwargs=convert_kwargs:converter(rosobj,**kwargs)),
        topic,ros_msg_class,**kwargs)

def object_publisher(topic,klampt_object,convert_kwargs=None,**kwargs):
    """Convenience function.  If klampt_object is an object, this sets up
    the publisher to be of the correct type.  You can then call publish on
    the returned publisher  using the same klampt_object (or another
    compatible Klampt objects).

    SimRobotSensors (particularly, cameras) can be published to multiple
    topics of the form [topic]/[subtopic].

    Returns:
        KlamptROSPublisher
    """
    from klampt import SimRobotSensor,Geometry3D
    if isinstance(klampt_object,SimRobotSensor):
        return publisher_SimRobotSensor(topic,klampt_object,convert_kwargs,**kwargs)
    ros_type = None
    if isinstance(klampt_object,Geometry3D):
        if klampt_object.type() == 'PointCloud':
            ros_type = 'PointCloud2'
        elif klampt_object.type() == 'TriangleMesh':
            ros_type = 'TriangleMesh'
        else:
            raise ValueError("Implicit surfaces, geometric primitives, and groups not supported yet")        
    return publisher(topic,_compatibleKlamptType(klampt_object),convert_kwargs,ros_type,**kwargs)
    
def subscriber(topic,klampt_type,callback,convert_kwargs=None,**kwargs):
    """Convenience function.  The subscriber automatically converts ROS
    messages to a Klampt type to be passed to the callback.

    Returns:
        rospy.Subscriber
    """
    if convert_kwargs is None:
        convert_kwargs = dict()
    if klampt_type not in SUPPORTED_KLAMPT_TYPES:
        raise ValueError("Don't know how to convert ROS messages to Klampt type "+klampt_type)
    ros_type = SUPPORTED_KLAMPT_TYPES[klampt_type]
    converter = 'from_'+ros_type
    assert converter in globals(),"No converter for Klampt type "+klampt_type
    convert_fn = globals()[converter]
    def shim(ros_obj,callback=callback,convert_fn=convert_fn,convert_kwargs=convert_kwargs):
        klampt_obj = convert_fn(ros_obj,**convert_kwargs)
        callback(klampt_obj)
    ros_msg_class = globals()[ros_type]
    return rospy.Subscriber(topic,ros_msg_class,callback=shim,**kwargs)

def object_subscriber(topic,klampt_obj,convert_kwargs=None,**kwargs):
    """Convenience function.  Every time a message arrives on the ROS topic,
    the given klampt object is updated. 

    Only valid for:

        * points / transforms provided as lists
        * Config (Float32MultiArray or JointState messages)
        * RobotModel (JointState messages)
        * SimRobotController (JointState messages)
        * Trajectory  (JointTrajectory messages)

    Returns:
        rospy.Subscriber
    """
    if convert_kwargs is None:
        convert_kwargs = dict()
    type = _compatibleKlamptType(klampt_obj)
    copy_fn = None
    if type in ['Config','Point','Vector3','Matrix3','Rotation','RigidTranform']:
        def default_copy(vin,out):
            assert isinstance(out,list),"Can only copy into lists, not tuples"
            assert len(vin) == len(out)
            for i in range(len(vin)):
                out[i] = vin[i]
        copy_fn = default_copy
    elif type == 'RobotModel':
        def copyrobot(jointstate_res,robot=klampt_obj):
            robot.setConfig(jointstate_res[0])
        copy_fn = copyrobot
        convert_kwargs['robot'] = klampt_obj
    elif type == 'SimRobotController':
        def copycontroller(jointstate_res,controller=klampt_obj):
            q,dq,effort = jointstate_res
            if dq is None and q is not None:
                dq = [0.0]*len(q)
            if effort is not None:
                if q is None:
                    controller.setTorque(effort)
                else:
                    controller.setPIDCommand(q,dq,effort)
            elif q is not None:
                controller.setPIDCommand(q,dq)
        copy_fn = copycontroller
        convert_kwargs['robot'] = klampt_obj.model()
    elif type == 'Trajectory':
        def copytraj(jointtrajectory_res,traj=klampt_obj):
            traj.times,traj.milestones = jointtrajectory_res.times,jointtrajectory_res.milestones
        copy_fn = copytraj
    else:
        raise ValueError("Hmm... not clear how to update that Klamp't object")

    def shim(new_klampt_obj,klampt_obj_ptr=klampt_obj):
        copy_fn(new_klampt_obj,klampt_obj_ptr)
        
    return subscriber(topic,type,shim,convert_kwargs,**kwargs)


def broadcast_tf(broadcaster,klampt_obj,frameprefix="klampt",root="world",stamp=0.):
    """Broadcasts Klampt frames to the ROS tf module.

    Args:
        broadcaster (tf.TransformBroadcaster): the tf broadcaster
        klampt_obj: the object to publish.  Can be WorldModel, Simulator,
            RobotModel, SimRobotController, anything with a getTransform
            or getCurrentTransform method, or se3 element
        frameprefix (str): the name of the base frame for this object
        root (str): the name of the TF world frame.

    Note:
        If klampt_obj is a Simulator or SimRobotController, then its
        corresponding model will be updated.
    """
    from klampt import WorldModel,Simulator,RobotModel,SimRobotController,SimRobotSensor
    if stamp == 'now':
        stamp = rospy.Time.now()
    elif isinstance(stamp,(int,float)):
        stamp = rospy.Time(stamp)
    world = None
    if isinstance(klampt_obj,WorldModel):
        world = klampt_obj
    elif isinstance(klampt_obj,Simulator):
        world = klampt_obj.model()
        klampt_obj.updateModel()
    if world is not None:
        prefix = frameprefix
        for i in range(world.numRigidObjects()):
            T = world.rigidObject(i).getTransform()
            q = so3.quaternion(T[0])
            broadcaster.sendTransform(T[1],(q[1],q[2],q[3],q[0]),stamp,prefix+"/"+world.rigidObject(i).getName(),root)
        for i in range(world.numRobots()):
            robot = world.robot(i)
            rprefix = prefix+"/"+robot.getName()
            broadcast_tf(broadcaster,robot,rprefix,root)
        return
    robot = None
    if isinstance(klampt_obj,RobotModel):
        robot = klampt_obj
    elif isinstance(klampt_obj,SimRobotController):
        robot = klampt_obj.model()
        robot.setConfig(klampt_obj.getSensedConfig())
    if robot is not None:
        rprefix = frameprefix
        for j in range(robot.numLinks()):
            p = robot.link(j).getParent()
            if p < 0:
                T = robot.link(j).getTransform()
                q = so3.quaternion(T[0])
                broadcaster.sendTransform(T[1],(q[1],q[2],q[3],q[0]),stamp,rprefix+"/"+robot.link(j).getName(),root)
            else:
                Tparent = se3.mul(se3.inv(robot.link(p).getTransform()),robot.link(j).getTransform())
                q = so3.quaternion(Tparent[0])
                broadcaster.sendTransform(Tparent[1],(q[1],q[2],q[3],q[0]),stamp,rprefix+"/"+robot.link(j).getName(),rprefix+"/"+robot.link(p).getName())
        return
    if isinstance(klampt_obj,SimRobotSensor):
        from ..model import sensing
        Tsensor_link = sensing.get_sensor_xform(klampt_obj)
        link = int(klampt_obj.get_setting('link'))
        if klampt_obj.type() == 'LaserRangeSensor': #the convention between Klampt and ROS is different
            klampt_to_ros_lidar = so3.from_matrix([[0,1,0],
                                                   [0,0,1],
                                                   [1,0,0]])
            Tsensor_link = (so3.mul(Tsensor_link[0],klampt_to_ros_lidar),Tsensor_link[1])
        q = so3.quaternion(Tsensor_link[0])
        robot = klampt_obj.robot()
        broadcaster.sendTransform(Tsensor_link[1],(q[1],q[2],q[3],q[0]),stamp,frameprefix+"/"+klampt_obj.name(),frameprefix+"/"+robot.link(link).getName())
        return
    transform = None
    if isinstance(klampt_obj,(list,tuple)):
        #assume to be an SE3 element.
        transform = klampt_obj
    elif hasattr(klampt_obj,'getTransform'):
        transform = klampt_obj.getTransform()
    elif hasattr(klampt_obj,'getCurrentTransform'):
        transform = klampt_obj.getCurrentTransform()
    else:
        raise ValueError("Invalid type given to broadcast_tf: ",klampt_obj.__class__.__name__)

    q = so3.quaternion(transform[0])
    broadcaster.sendTransform(transform[1],(q[1],q[2],q[3],q[0]),stamp,frameprefix,root)
    return

def listen_tf(listener,klampt_obj,frameprefix="klampt",root="world",onerror=None):
    """Reads Klampt frames from the ROS tf module.

    Args:
        listener (tf.TransformListener): the tf listener
        klampt_obj: the object to update. Can be WorldModel, RobotModel,
            anything with a setTransform or setCurrentTransform method,
            or None (in the latter case, a se3 object will be returned).

            Note:
            
                RobotModel configurations will not be changed, just
                the link transforms.

        frameprefix (str): the name of the base frame for this object
        root (str): the name of the TF world frame.
        onerror (str or None): either 'raise' in which case a tf exception
            is raised, 'print', in which case the error is printed, or None,
            in which case any exception is silently ignored.
    """
    from klampt import WorldModel,RobotModel
    import tf
    def do_lookup(frame,parent):
        try:
            (trans,rot) = listener.lookupTransform(frame, parent, rospy.Time(0))
            return (so3.from_quaternion((rot[3],rot[0],rot[1],rot[2])),trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            if onerror == 'print':
                print("listen_tf: Error looking up frame",frame)
            elif onerror == 'raise':
                raise
        return None          
    if isinstance(klampt_obj,WorldModel):
        world = klampt_obj
        prefix = frameprefix
        for i in range(world.numRigidObjects()):
            T = do_lookup(prefix+"/"+world.rigidObject(i).getName(),root)
            if T:
                world.rigidObject(i).setTransform(*T)
        for i in range(world.numRobots()):
            robot = world.robot(i)
            rprefix = prefix+"/"+robot.getName()
            listen_tf(listener,robot,rprefix,root,onerror)
        return
    elif isinstance(klampt_obj,RobotModel):
        robot = klampt_obj
        rprefix = frameprefix
        for j in range(robot.numLinks()):
            p = robot.link(j).getParent()
            if p < 0:
                T = do_lookup(rprefix+"/"+robot.link(j).getName(),root)
                if T:
                    robot.link(j).setTransform(*T)
            else:
                T = do_lookup(rprefix+"/"+robot.link(j).getName(),rprefix+"/"+robot.link(p).getName())
                if T:
                    robot.link(j).setTransform(*se3.mul(robot.link(p).getTransform(),T))
        return
    elif hasattr(klampt_obj,'setTransform'):
        T = do_lookup(frameprefix,root)
        if T:
            klampt_obj.setTransform(*T)
    elif hasattr(klampt_obj,'setCurrentTransform'):
        T = do_lookup(frameprefix,root)
        if T:
            klampt_obj.setCurrentTransform(*T)
    elif klampt_obj is None:
        return do_lookup(frameprefix,root)
    else:
        raise ValueError("Invalid type given to listen_tf: ",klampt_obj.__class__.__name__)



