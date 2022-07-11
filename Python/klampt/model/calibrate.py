"""Functions for extrinsic calibration of cameras.

.. versionadded:: 0.9
"""

from klampt import *
from klampt.math import vectorops,so3,se3
from klampt.io import resource
from klampt.model.trajectory import RobotTrajectory
import math
from klampt.control.robotinterface import RobotInterfaceBase
from klampt.model import sensing
from typing import Optional,Union,Sequence,List,Tuple,Dict,Any,Callable
from .typing import Vector2,Vector3,RigidTransform,Vector
import random

Key = Union[int,str]

class PointMarker:
    """A physical point marker that can be detected in an image.

    Attributes:
        link (int or str, optional): link on which this is located. None means
            attached to the world frame.
        size (float): used only for visual debugging, default 0.01 (1cm)
        local_coordinates (3-vector, optional): position relative to link
        variable (bool): whether this should be optimized

    """
    def __init__(self,link=None,local_coordinates=None):
        self.link = link                             # type: Optional[Key]
        self.size = 0.01                             # type: float
        self.local_coordinates = local_coordinates   # type: Vector3
        self.variable = True                         # type: bool
    

class TransformMarker:
    """A physical marker with some orientation, and either the transform
    or points of features on the marker can be detected in an image.

    Attributes:
        link (int or str, optional): link on which this is located. None means
            attached to the world frame.
        size (float): used only for visual debugging, default 0.01 (1cm)
        local_coordinates (se3 element, optional): pose relative to link
        local_features (list of 3-vectors): feature points relative to marker
        variable (bool): whether this should be optimized
        
    """
    def __init__(self,link=None,local_coordinates=None,local_features=None):
        self.link = link                            # type: Optional[Key]
        self.size = 0.1                             # type: float
        self.local_coordinates = local_coordinates  # type: RigidTransform
        self.local_features = ([] if local_features is None else local_features)  # type: List[Vector3]
        self.variable = True                        # type: bool


class CameraInfo:
    """Stores info for a camera in a calibration.

    Attributes:
        link (int or str, optional): link on which this is located. None means
            attached to the world frame.
        intrinsics (dict): dict with keys 'fx', 'fy', 'cx', 'cy' giving the
            camera intrinsics.
        local_coordinates (se3 element, optional): pose relative to link
        variable (bool): whether this should be optimized

    """
    def __init__(self,link,intrinsics=None,local_coordinates=None):
        self.link = link                            # type: Optional[Key]
        self.intrinsics = intrinsics                # type: dict
        self.local_coordinates = local_coordinates  # type: RigidTransform
        self.variable = True                        # type: bool


class PointObservation:
    """A 3D point measurement in the frame of a camera.

    X is to the right, Y is down, Z is forward.  Units are in m.
    """
    def __init__(self,value,camera_id,frame_id,marker_id,feature_id=None):
        self.camera_id = camera_id
        self.frame_id = frame_id
        self.marker_id = marker_id
        self.feature_id = feature_id
        self.value = value
        self.error = None
        self.hand_eye_transform = None


class TransformObservation:
    """A 3D SE(3) transform measurement in the frame of a camera.

    X is to the right, Y is down, Z is forward.  Translation units are in m.
    """
    def __init__(self,value,camera_id,frame_id,marker_id):
        self.camera_id = camera_id
        self.frame_id = frame_id
        self.marker_id = marker_id
        self.value = value
        self.error = None
        self.hand_eye_transform = None


class PixelObservation:
    """A 2D pixel measurement in the frame of a camera.

    X is to the right, Y is down.  Units are in pixels.
    """
    def __init__(self,value,camera_id,frame_id,marker_id,feature_id=None):
        self.camera_id = camera_id
        self.frame_id = frame_id
        self.marker_id = marker_id
        self.feature_id = feature_id
        self.value = value
        self.error = None
        self.hand_eye_transform = None


class RobotExtrinsicCalibration:
    """Stores a calibration dataset and helps perform extrinsic (hand-eye)
    calibration.

    Usage for a world-attached camera, Aruco marker on end effector::

        calib = RobotExtrinsicCalibration()
        calib.robot = #set the robot model here
        calib.cameras[0] = CameraInfo(link=None,intrinsics=intrinsics)
        ee_link = # set the ee link index or name here
        calib.markers[tag_id] = TransformMarker(ee_link)

        calib.visualize()         #this will show a visualization of the markers and camera

        calib.editTrajectory()    #this will pop up an editing window

        controller = MyRobotInterface()  #setup the robot controller here
        controller.initialize()

        def take_camera_picture():
            #TODO: take the picture from the camera here
            return image

        #TODO: setup aruco_detector... see Aruco documentation for more details
        camparam = aruco.CameraParameters()
        camparam.readFromXMLFile('cam_params.yml')
        detector = aruco.MarkerDetector()
        detector.setDictionary("my_markers.dict")
        
        def detect_aruco_marker(image):
            #return all detected markers
            markers = aruco_detector.detect(image)
            results = []
            for marker in markers:
                marker.calculateExtrinsics(0.15, camparam)
                mtx = marker.getTransformMatrix()
                Tmarker = se3.from_homogeneous(mtx)  #convert to klampt.se3
                results.append((marker.id,Tmarker))
            return results

        #if you just want to run detection on the pictures offline, use
        #take_camera_picture = None and detect_aruco_marker = None
        calib.executeTrajectory(controller,take_camera_picture,detect_aruco_marker)

        calib.editCalibration()     #visual editing of initial guesses
        calib.calibrate()  #optimize the calibration
        calib.save('my_calibration.json')
    
    To use reprojection error instead::
    
        intrinsics = {'fx':fx,'fy':fy,'cx':cx,'cy':cy}  #intrinsics are needed if you are using pixels
        
    
    If you want to load a set of configurations and marker transforms from
    disk manually::

        ... do calib robot and marker setup ...
        configs = klampt.io.loader.load('calibration.configs')
        transforms = []
        with open('marker_transforms.txt','r') as f:
            for line in f.readlines():
                if len(line.strip()) > 0:
                    T = klampt.io.loader.read(line,'RigidTransform')
                    transforms.append(T)
        for i,(q,T) in enumerate(zip(configs,transforms)):
            calib.frames.append(None)
            calib.configurations.append(q)
            calib.addDetection(T,frame_id=i,marker_id=0)

        calib.editCalibration()     #visual editing of initial guesses
        calib.calibration()  #optimize the calibration
        calib.save('my_calibration.json')

    To visualize the calibration::

        ... do calib robot and marker setup ...
        calib.load('my_calibration.json')
        calib.visualize()
    
    To fix the marker transform during optimization, e.g. at local coordinates
    [x,y,z]::

        calib.markers[0].local_coordinates = (so3.identity(),[x,y,z])
        calib.markers[0].variable = False
        calib.calibrate()
    
    A similar flag can be used for fixing the camera.
    
    """
    def __init__(self) -> None:
        self.robot = None                 # type: Optional[RobotModel]
        self.world = None                 # type: Optional[WorldModel]
        self.trajectory = None            # type: Optional[RobotTrajectory]
        self.cameras = dict()             # type: Dict[Key,CameraInfo]
        self.camera_constraints = dict()  #TODO: for bundle adjustment
        self.markers = dict()             # type: Dict[Key,Union[PointMarker,TransformMarker]]
        self.frames = []
        self.frame_camera_ids = []        # type: List[Key]
        self.configurations = []          # type: List[Vector]
        self.observations = []            # type: List[Union[PointObservation,TransformObservation,PixelObservation]]

    def cameraFromRobot(self,sensors=None) -> None:
        """Sets up the camera(s) from the robot model. 
        
        Args:
            sensors (str, int, list of str, or list of int, optional):
                specifies one or more sensors to use. If None, just uses the
                first camera.  If 'all', uses all cameras.
        """
        self.cameras = dict()
        simsensors = []
        if sensors is None or sensors=='all':
            sindex = 0
            while True:
                s = self.robot.sensor(sindex)
                if s.name() == '':
                    break
                if s.type() == 'CameraSensor':
                    simsensors.append(s)
                    if sensors is None:
                        break
                sindex += 1
        else:
            if not isinstance(sensors,(list,tuple)):
                sensors = [sensors]
            for sindex in sensors:
                s = self.robot.sensor(sindex)
                if s.name() == '':
                    raise ValueError("Invalid sensor {}, not in robot model".format(sindex))
                if s.type() != 'CameraSensor':
                    raise ValueError("Invalid sensor {}, not a CameraSensor".format(sindex))
                simsensors.append(s)
        for s in simsensors:
            caminfo = CameraInfo(int(s.getSetting('link')))
            caminfo.intrinsics = sensing.camera_to_intrinsics(s,'json')
            caminfo.local_coordinates = sensing.get_sensor_xform(s)
            self.cameras[s.name()] = caminfo

    def editTrajectory(self, world : WorldModel = None, name='calibration_trajectory') -> RobotTrajectory:
        """Returns a RobotTrajectory that passes through all calibration
        configurations.

        Note: the executed trajectory stops at all milestones. 
        """
        if world is None:
            if self.world is not None:
                world = self.world
            else:
                if self.robot is None:
                    raise ValueError("Can't edit trajectory without a robot model")
                world = WorldModel()
                world.add('temp',self.robot)
        if self.trajectory is None and len(self.configurations) > 0:
            self.trajectory = RobotTrajectory(self.robot, list(range(len(self.configurations))), self.configurations)
        save,newtraj = resource.edit(name,self.trajectory, 'Trajectory', world=world, doedit=True)
        if save:
            self.trajectory = newtraj
            return newtraj
        return self.trajectory
    
    def executeTrajectory(self,
                        controller : RobotInterfaceBase,
                        traj : RobotTrajectory,
                        camera_fn : Callable,
                        detect_fn : Callable=None,
                        speed=1.0,
                        wait=0.0) -> None:
        """Executes a trajectory on a controller, optionally taking images 
        along the way.

        TODO: specify times to actually stop.
        """
        import time
        def wait_for_move():
            moving = True
            while moving:
                controller.beginStep()
                moving = controller.isMoving()
                controller.endStep()
                time.sleep(0.1)

        for i in range(len(traj.milestones)):
            print("Moving to milestone",i)
            controller.beginStep()
            controller.moveToPosition(traj.milestones[0],speed)
            controller.endStep()
            wait_for_move()

            if wait != 0:
                print("Waiting for",wait,"s")
            time.sleep(wait*0.5)
            if camera_fn:
                print("Taking pictures")
                if isinstance(camera_fn,dict):
                    for (k,fn) in camera_fn.items():
                        image = fn()
                        self.add_frame(image,controller.sensedPosition(),camera_id=k,detect_fn=detect_fn)
                else:
                    image = camera_fn()
                    self.add_frame(image,controller.sensedPosition(),detect_fn=detect_fn)
            time.sleep(wait*0.5)

    def visualize(self, world : WorldModel=None, edit=False) -> None:
        """Pops up a visualization window showing the calibration setup.
        """
        from klampt import vis
        from klampt.model import sensing
        if world is None:
            if self.world is not None:
                world = self.world
            else:
                if self.robot is None:
                    raise ValueError("Can't edit trajectory without a robot model")
                world = WorldModel()
                world.add('temp',self.robot)
        vis.add('world',world)
        camera_widgets = dict()
        marker_widgets = dict()
        sensors = dict()
        for i,c in self.cameras.items():
            try:
                s = self.robot.sensor(i)
                if s.name()=='':
                    raise ValueError()
            except Exception:
                #create a new camera on the robot
                s = self.robot.addSensor(str(i),'CameraSensor')
                sensing.intrinsics_to_camera(c.intrinsics,s,'json')
                s.setSetting('zmin','0.1')
                s.setSetting('zmax','10')
                s.setSetting('link',str(_linkIndex(c.link,self.robot)))
                sensing.set_sensor_xform(s,c.local_coordinates)
            #for k in s.settings():
            #    print(k,s.getSetting(k))
            sensors[i] = s
            vis.add('Camera_widget {}'.format(i),s)
            vis.add('Camera {}'.format(i),sensing.get_sensor_xform(s,self.robot))
            if edit:
                camera_widgets[i] = vis.edit('Camera {}'.format(i))
        for i,m in self.markers.items():
            if isinstance(m,PointMarker):
                coords = _worldPosition(m.link,self.robot,m.local_coordinates)
                vis.add('Marker {}'.format(i),coords,size=m.size)
                if edit:
                    marker_widgets[i] = vis.edit('Marker {}'.format(i))
            else:
                coords = _worldTransform(m.link,self.robot,m.local_coordinates)
                vis.add('Marker {}'.format(i),coords,length=m.size)
                if edit:
                    marker_widgets[i] = vis.edit('Marker {}'.format(i))

        import time
        vis.add('calibration_configs',self.configurations,color=(0,1,0,0.3))
        vis.show()
        re_read_cameras = []
        re_read_markers = []
        while vis.shown():
            #TODO: spin between calibration configurations and show sensed observations + errors for each frame
            #read edited camera / marker transforms
            for k in re_read_cameras: #make sure to get released configuration
                Tc = vis.getItemConfig('Camera {}'.format(k))
                Tc = (Tc[:9],Tc[9:])
                c = self.cameras[k]
                c.local_coordinates = _localTransform(c.link,self.robot,Tc)
                sensing.set_sensor_xform(sensors[k],c.local_coordinates)
            re_read_cameras = []
            for k,cw in camera_widgets.items():
                if cw.hasFocus():
                    Tc = vis.getItemConfig('Camera {}'.format(k))
                    Tc = (Tc[:9],Tc[9:])
                    c = self.cameras[k]
                    c.local_coordinates = _localTransform(c.link,self.robot,Tc)
                    sensing.set_sensor_xform(sensors[k],c.local_coordinates)
                    re_read_cameras.append(k)
            for k in re_read_markers: #make sure to get released configuration
                Tm = vis.getItemConfig('Marker {}'.format(k))
                m = self.markers[k]
                if isinstance(m,PointMarker):
                    m.local_coordinates = _localPosition(m.link,self.robot,Tm)
                else:
                    Tm = (Tm[:9],Tm[9:])
                    m.local_coordinates = _localTransform(m.link,self.robot,Tm)
            re_read_cameras = []
            for k,mw in marker_widgets.items():
                if mw.hasFocus():
                    Tm = vis.getItemConfig('Marker {}'.format(k))
                    m = self.markers[k]
                    if isinstance(m,PointMarker):
                        m.local_coordinates = _localPosition(m.link,self.robot,Tm)
                    else:
                        Tm = (Tm[:9],Tm[9:])
                        m.local_coordinates = _localTransform(m.link,self.robot,Tm)
                    re_read_markers.append(k)
            time.sleep(0.05)

    def editCalibration(self, world : WorldModel = None) -> None:
        for i,c in self.cameras.items():
            if c.local_coordinates is None:
                c.local_coordinates = se3.identity()
        for i,m in self.markers.items():
            if m.local_coordinates is None:
                if isinstance(m,TransformMarker):
                    m.local_coordinates = se3.identity()
                else:
                    m.local_coordinates = [0.0]*3
        self.visualize(world,True)

    def addFrame(self, frame, q : Vector=None, camera_id : Key=None, detect_fn:Callable=None) -> None:
        """
        Add a frame and configuration, automatically using forward kinematics
        to determine camera / marker transforms. This is the easiest way to add
        frames and detections.

        Args:
            frame (Image): the image for the frame.
            q (configuration, optional): the robot's configuration. If not
                given, the observations' `hand_eye_transform` must be filled
                in manually.
            camera_id (int or str, optional): the camera ID, 0 by default.
            detect_fn (callable, optional): if given, a function
                `detect(frame)` which returns a list of detected observations.
                Each observation can be a PixelObservation, PointObservation,
                TransformObservation, or a (marker,value) pair.  To indicate
                a detected feature on a marker, a `((marker,feature),value)`
                pair is returned.
        
        """
        if q is not None:
            if self.robot is None:
                raise ValueError("Need to set robot first")
            if len(q) != self.robot.numLinks():
                raise ValueError("Invalid length of configuration")
        frame_id = len(self.frames)
        self.frames.append(frame)
        self.configurations.append(q)
        if camera_id is None:
            camera_id = list(self.cameras.keys())[0]
        self.frame_camera_ids.append(camera_id)
        if detect_fn is not None:
            detections = detect_fn(frame)
            for det in detections:
                if isinstance(det,(PixelObservation,PointObservation,TransformObservation)):
                    det.camera_id = camera_id
                    det.frame_id = frame_id
                    self.robot.setConfig(q)
                    camera_link = self.cameras[camera_id].link
                    marker_link = self.markers[det.marker_id].link
                    camera_link_transform = _linkTransform(camera_link,self.robot)
                    marker_link_transform = _linkTransform(marker_link,self.robot)
                    det.hand_eye_transform = se3.mul(se3.inv(camera_link_transform),marker_link_transform)
                    self.observations.append(det)
                else:
                    marker,value = det
                    feature = None
                    if hasattr(marker,'__iter__'):
                        marker,feature = marker
                    self.addDetection(frame_id,marker,feature,value,camera_id)
    
    def addDetection(self, value : Union[Vector2,Vector3,RigidTransform],
                        frame_id : int,
                        marker_id : Key,
                        feature_id : int=None,
                        camera_id : Key=None,
                        error : Union[float,Sequence[float]]=None) -> Union[PixelObservation,PointObservation,TransformObservation]:
        """Manually adds a detection for a given camera / frame / marker / 
        feature.  Automatically determines a `hand_eye_transform` for the
        observation if a robot and configuration are already provided for
        this frame.

        Args:
            value (2-list, 3-list, or SE3 element): the pixel, point, or
                transform of the detection, given in camera coordinates.
            frame_id (int): the index of the frame
            marker_id (int or str): the index of the marker
            feature_id (int, optional): the index of the feature on the marker.
            camera_id (int or str, optional): the camera that observed this
                feature (default 0)
            error (float or list, optional): the standard deviation of the
                observation error.  If a float is given, noise is assumed
                to be isotropic.
        
        Returns:
            The configured detection object.
        """
        if frame_id < 0 or frame_id >= len(self.frames):
            raise ValueError("Invalid frame")
        if marker_id not in self.markers:
            raise ValueError("Invalid marker")
        if camera_id is None:
            camera_id = list(self.cameras.keys())[0]
        if camera_id not in self.cameras:
            raise ValueError("Invalid camera")
        marker = self.markers[marker_id]
        if feature_id is not None:
            if not hasattr(marker,'local_features'):
                raise ValueError("Invalid detection, references a feature in a point marker")
            if feature_id < 0 or feature_id >= len(marker.local_features):
                raise ValueError("Invalid detection, references a non-existent feature")
        if len(value)==2 and not hasattr(value[0],'__iter__'):
            self.observations.append(PixelObservation(value,camera_id,frame_id,marker_id,feature_id))
        elif isinstance(marker,TransformMarker):
            self.observations.append(TransformObservation(value,camera_id,frame_id,marker_id))
        else:
            self.observations.append(PointObservation(value,camera_id,frame_id,marker_id,feature_id))
        self.observations[-1].error = error
        if self.robot is not None and frame_id < len(self.configurations) and self.configurations[frame_id] is not None:
            self.robot.setConfig(self.configurations[frame_id])
            camera_link = self.cameras[camera_id].link
            marker_link = marker.link
            camera_link_transform = _linkTransform(camera_link,self.robot)
            marker_link_transform = _linkTransform(marker_link,self.robot)
            self.observations[-1].hand_eye_transform = se3.mul(se3.inv(camera_link_transform),marker_link_transform)
        return self.observations[-1]
    
    def cameraTransforms(self,world=False) -> Dict[Key,RigidTransform]:
        """Returns a dict of camera transforms, in either world or link-
        local coordinates.  World coordinates use the robot's current
        configuration.
        """
        if world:
            res = dict()
            for i,c in self.cameras.items():
                res[i] = _worldTransform(c.link,self.robot,c.local_coordinates)
            return res
        else:
            return dict((i,(c.local_coordinates if c is not None else se3.identity())) for i,c in self.cameras.items())

    def markerTransforms(self,world=False) -> Dict[int,RigidTransform]:
        """Returns a dict of marker transforms, in either world or link-
        local coordinates.  World coordinates use the robot's current
        configuration.
        """
        if world:
            res = dict()
            for i,m in self.markers.items():
                if isinstance(m,PointMarker):
                    res[i] = _worldPosition(m.link,self.robot,m.local_coordinates)
                else:
                    res[i] = _worldTransform(m.link,self.robot,m.local_coordinates)
            return res
        else:
            res = dict()
            for i,m in self.markers.items():
                if isinstance(m,PointMarker):
                    res[i] = (m.local_coordinates if m is not None else [0.0]*3)
                else:
                    res[i] = (m.local_coordinates if m is not None else se3.identity())
            return res

    def setCameraTransform(self, camera : Key, T : RigidTransform, world=False):
        """Sets the camera transform for the specified camera.
        World coordinates use the robot's current configuration.
        """
        c = self.cameras[camera]
        if world:
            T = _localTransform(c.link,self.robot,T)
        c.local_coordinates = T
    
    def setMarkerTransform(self, marker : Key, T : RigidTransform, world=False):
        """Sets the transform / coordinates for the specified marker.
        World coordinates use the robot's current configuration.
        """
        m = self.markers[marker]
        if world:
            if isinstance(m,TransformMarker):
                T = _localTransform(m.link,self.robot,T)
            else:
                T = _localPosition(m.link,self.robot,T)
        m.local_coordinates = T

    def predictedObservations(self,camera_transforms=None,marker_transforms=None,noise=False) -> List[Union[PointObservation,PixelObservation,TransformObservation]]:
        """Returns a list of predicted observations for a given set of camera 
        and marker transforms.

        Args:
            camera_transforms (dict key->se3 elements, optional): keys matching
                `self.cameras`. If None, then it is assumed that all cameras
                have transforms (`local_coordinates`) specified.
            marker_transforms (dict key->se3 elements, optional): keys matching
                `self.markers`. If None, then it is assumed that all markers
                have transforms (`local_coordinates`) specified.
            noise (bool or float): if not False, then noise is injected into
                the prediction. This is useful for generating synthetic data
                and understanding the observation error model.
        
        Returns:
            A list of configured observation objects, corresponding to the 
            values in `self.observations`.
        """
        if camera_transforms is None:
            camera_transforms = dict()
        if not isinstance(camera_transforms,dict):
            if len(self.cameras)==1:
                #be tolerant
                camera_transforms = {list(self.cameras.keys())[0]:camera_transforms}
        if any(i not in camera_transforms for i,c in self.cameras.items()):
            camera_transforms = camera_transforms.copy()
            for i,c in self.cameras.items():
                if i not in camera_transforms:
                    camera_transforms[i] = c.local_coordinates
                    if c.local_coordinates is None:
                        raise ValueError("No transform specified for camera {}".format(i))
        if marker_transforms is None:
            marker_transforms = dict()
        if not isinstance(marker_transforms,dict):
            if len(self.markers)==1:
                #be tolerant
                marker_transforms = {list(self.markers.keys())[0]:marker_transforms}
        if any(i not in marker_transforms for i,m in self.markers.items()):
            marker_transforms = marker_transforms.copy()
            for i,m in self.markers.items():
                if i not in marker_transforms:
                    marker_transforms[i] = m.local_coordinates
                    if m.local_coordinates is None:
                        raise ValueError("No transform specified for marker {}".format(i))
        if noise is False:
            noise = 0
        def _add_noise(value,error):
            if noise == 0:
                return
            if error is None:
                for i in range(len(value)):
                    value[i] += random.gauss(0,noise)
            elif hasattr(error,'__iter__'):
                for i in range(len(value)):
                    value[i] += random.gauss(0,noise*error[i])
            else:
                for i in range(len(value)):
                    value[i] += random.gauss(0,noise*error)

        obs = []
        for o in self.observations:
            cam = self.cameras[o.camera_id]
            if cam.intrinsics is not None:
                fx,fy,cx,cy = cam.intrinsics['fx'],cam.intrinsics['fy'],cam.intrinsics['cx'],cam.intrinsics['cy']
            marker = self.markers[o.marker_id]
            Tm_local = marker_transforms[o.marker_id]
            Tc_local = camera_transforms[o.camera_id]
            if isinstance(marker,TransformMarker):
                Tm_c = se3.mul(se3.inv(Tc_local),se3.mul(o.hand_eye_transform,Tm_local))
                if isinstance(o,PixelObservation):
                    #projection
                    xyz = se3.apply(Tm_c,marker.local_features[o.feature_id])
                    pixel = [fx*xyz[0]/xyz[2] + cx,fy*xyz[1]/xyz[2] + cy]
                    _add_noise(pixel,o.error)
                    obs.append(PixelObservation(pixel,o.camera_id,o.frame_id,o.marker_id,o.feature_id))
                elif isinstance(o,PointObservation):
                    #point
                    feature = o.feature_id
                    xyz = se3.apply(Tm_c,marker.local_features[o.feature_id])
                    _add_noise(xyz,o.error)
                    obs.append(PointObservation(xyz,o.camera_id,o.frame_id,o.marker_id,o.feature_id))
                else:
                    #transform
                    if noise:
                        disturbance = [0]*6
                        _add_noise(disturbance,o.error)
                        Tdist = so3.from_moment(disturbance[:3]),disturbance[3:]
                        Tm_c = se3.mul(Tdist,Tm_c)
                    obs.append(TransformObservation(Tm_c,o.camera_id,o.frame_id,o.marker_id))
            else:
                xyz = se3.apply(se3.inv(Tc_local),se3.apply(o.hand_eye_transform,Tm_local))
                if isinstance(o,PixelObservation):
                    #projection
                    pixel = [fx*xyz[0]/xyz[2] + cx,fy*xyz[1]/xyz[2] + cy]
                    _add_noise(pixel,o.error)
                    obs.append(PixelObservation(pixel,o.camera_id,o.frame_id,o.marker_id,o.feature_id))
                elif isinstance(o,PointObservation):
                    #point
                    _add_noise(xyz,o.error)
                    obs.append(PointObservation(xyz,o.camera_id,o.frame_id,o.marker_id,o.feature_id))
                else:
                    raise ValueError("Uh.. can't get a TransformObservation for a PointMarker")
        return obs

    def predictedResiduals(self,camera_transforms=None,marker_transforms=None) -> List[Vector]:
        """Returns a list of predicted residuals for a given set of camera 
        and marker transforms.

        Args:
            camera_transforms (list of se3 elements, optional): length
                `len(self.cameras)`. If None, then it is assumed that all
                cameras have transforms (`local_coordinates`) specified.
            marker_transforms (list of se3 elements, optional): length
                `len(self.markers)`. If None, then it is assumed that all
                markers have transforms (`local_coordinates`) specified.
        
        Returns:
            A list of residuals (prediction - observed value)
            corresponding to the values in `self.observations`.
        """
        if camera_transforms is None:
            camera_transforms = dict()
        if not isinstance(camera_transforms,dict):
            if len(self.cameras)==1:
                #be tolerant
                camera_transforms = {list(self.cameras.keys())[0]:camera_transforms}
        if any(i not in camera_transforms for i,c in self.cameras.items()):
            camera_transforms = camera_transforms.copy()
            for i,c in self.cameras.items():
                if i not in camera_transforms:
                    print("Reading camera transform",i)
                    camera_transforms[i] = c.local_coordinates
                    if c.local_coordinates is None:
                        raise ValueError("No transform specified for camera {}".format(i))
        if marker_transforms is None:
            marker_transforms = dict()
        if not isinstance(marker_transforms,dict):
            if len(self.markers)==1:
                #be tolerant
                marker_transforms = {list(self.markers.keys())[0]:marker_transforms}
        if any(i not in marker_transforms for i,m in self.markers.items()):
            marker_transforms = marker_transforms.copy()
            for i,m in self.markers.items():
                if i not in marker_transforms:
                    print("Reading marker transform",i)
                    marker_transforms[i] = m.local_coordinates
                    if m.local_coordinates is None:
                        raise ValueError("No transform specified for marker {}".format(i))
            
        res = []
        for o in self.observations:
            cam = self.cameras[o.camera_id]
            if cam.intrinsics is not None:
                fx,fy,cx,cy = cam.intrinsics['fx'],cam.intrinsics['fy'],cam.intrinsics['cx'],cam.intrinsics['cy']
            marker = self.markers[o.marker_id]
            Tm_local = marker_transforms[o.marker_id]
            Tc_local = camera_transforms[o.camera_id]
            if isinstance(marker,TransformMarker):
                Tm_c = se3.mul(se3.inv(Tc_local),se3.mul(o.hand_eye_transform,Tm_local))
                if isinstance(o,PixelObservation):
                    #projection
                    xyz = se3.apply(Tm_c,marker.local_features[o.feature_id])
                    pixel = [fx*xyz[0]/xyz[2] + cx,fy*xyz[1]/xyz[2] + cy]
                    res.append(vectorops.sub(pixel,o.value))
                elif isinstance(o,PointObservation):
                    #point
                    feature = o.feature_id
                    xyz = se3.apply(Tm_c,marker.local_features[o.feature_id])
                    res.append(vectorops.sub(xyz,o.value))
                else:
                    #transform
                    res.append(se3.error(Tm_c,o.value))
            else:
                xyz = se3.apply(se3.inv(Tc_local),se3.apply(o.hand_eye_transform,Tm_local))
                if isinstance(o,PixelObservation):
                    #projection
                    pixel = [fx*xyz[0]/xyz[2] + cx,fy*xyz[1]/xyz[2] + cy]
                    res.append(vectorops.sub(pixel,o.value))
                elif isinstance(o,PointObservation):
                    #point
                    res.append(vectorops.sub(xyz,o.value))
                else:
                    raise ValueError("Uh.. can't get a TransformObservation for a PointMarker")
        return res

    def predictedLogLikelihoods(self,camera_transforms=None,marker_transforms=None) -> List[float]:
        """Returns a list of predicted observation log-likelihoods for a
        given set of camera and marker transforms.

        Args:
            camera_transforms (list of se3 elements, optional): length
                `len(self.cameras)`. If None, then it is assumed that all
                cameras have transforms (`local_coordinates`) specified.
            marker_transforms (list of se3 elements, optional): length
                `len(self.markers)`. If None, then it is assumed that all
                markers have transforms (`local_coordinates`) specified.
        
        Returns:
            A list of log likelihoods of residuals corresponding to the
            values in `self.observations`.
        """
        residuals = self.predictedResiduals(camera_transforms,marker_transforms)
        log2pi = math.log(math.pi*2)
        ll = 0
        for res,obs in zip(residuals,self.observations):
            if obs.error is None:
                ll += -vectorops.normSquared(res)*0.5 - len(res)*log2pi
            elif hasattr(obs.error):
                ll += -vectorops.normSquared(vectorops.div(res,obs.error))*0.5 - len(res)*log2pi - sum(math.log(e) for e in obs.error)
            else:
                ll += -vectorops.normSquared(vectorops.div(res,obs.error))*0.5 - len(res)*(log2pi+math.log(obs.error))
        return ll

    def optimize(self,maxIters=100,tol=1e-7,regularizationFactor=0.0,store=True) -> Tuple[float,Dict[Key,RigidTransform],Dict[Key,RigidTransform]]:
        """Optimizes the calibrated transforms.

        Requires scipy.

        Returns:
            A tuple of the RMSE, camera transform dictionary, and
            marker transform dictionary.
        """
        import numpy as np
        import scipy.optimize
        observation_weights = []
        for o in self.observations:
            if o.error is None:
                observation_weights.append(1)
            elif hasattr(o.error,'__iter__'):
                observation_weights.append(np.divide(1,o.error))
            else:
                observation_weights.append(1.0/o.error)
        def pack_variables(camera_transforms,marker_transforms):
            xs = []
            for i,c in self.cameras.items():
                T = camera_transforms[i]
                if c.variable:
                    xs.append(so3.moment(T[0]))
                    xs.append(T[1])
            for i,m in self.markers.items():
                T = marker_transforms[i]
                if m.variable:
                    if isinstance(m,TransformMarker):
                        xs.append(so3.moment(T[0]))
                        xs.append(T[1])
                    else:
                        xs.append(T)
            return np.hstack(xs)
        def unpack_variables(x):
            #unpack variables from x
            i = 0
            camera_transforms = dict()
            for k,c in self.cameras.items():
                if c.variable:
                    camera_transforms[k] =(so3.from_moment(x[i:i+3]),x[i+3:i+6])
                    i += 6
                else:
                    camera_transforms[k] = c.local_coordinates
            marker_transforms = dict()
            for k,m in self.markers.items():
                if isinstance(m,TransformMarker):
                    marker_transforms[k] = (so3.from_moment(x[i:i+3]),x[i+3:i+6])
                    i+=6
                else:
                    marker_transforms[k] = x[i:i+3]
                    i+=3
            return camera_transforms,marker_transforms
        def error_fn(x):
            camera_transforms,marker_transforms = unpack_variables(x)
            error = 0
            for (e,w) in zip(self.predictedResiduals(camera_transforms,marker_transforms),observation_weights):
                ew = np.multiply(e,w)
                error += ew.dot(ew)
            #print("Sqrt weighted error",math.sqrt(error))
            if regularizationFactor != 0:
                return error + regularizationFactor*(x-x0).dot(x-x0)
            return error
        camera_transforms0,marker_transforms0 = self.cameraTransforms(),self.markerTransforms()
        x = pack_variables(camera_transforms0,marker_transforms0)
        if regularizationFactor != 0.0:
            x0 = x.copy()
        print("Initial RMSE:",math.sqrt(sum(vectorops.normSquared(e) for e in self.predictedResiduals(camera_transforms0,marker_transforms0))))
        #res = scipy.optimize.minimize(error_fn,x,method='Nelder-Mead',options={'maxiter':maxIters})
        res = scipy.optimize.minimize(error_fn,x,method='BFGS',options={'maxiter':maxIters})
        x = res.x
        camera_transforms,marker_transforms = unpack_variables(x)
        rmse = math.sqrt(sum(vectorops.normSquared(e) for e in self.predictedResiduals(camera_transforms,marker_transforms)))
        print("Final RMSE:",rmse)
        try:
            import numdifftools as nd
            Hf = nd.Hessian(error_fn)
            H = Hf(x)
            Hinv = np.linalg.inv(H)
            Hinv_diag = np.sqrt(np.diag(Hinv)/len(self.observations))
            i = 0
            print()
            print("Cramer-Rao estimate of standard errors for cameras")
            for k,c in self.cameras.items():
                if c.variable:
                    print(k,":",Hinv_diag[i:i+3],"(r)",Hinv_diag[i+3:i+6],'(t)')
                    i += 6
            print("Cramer-Rao estimate of standard errors for markers")
            for k,m in self.markers.items():
                if m.variable:
                    if isinstance(m,PointMarker):
                        print(k,":",Hinv_diag[i:i+3])
                        i += 3
                    else:
                        print(k,":",Hinv_diag[i:i+3],"(r)",Hinv_diag[i+3:i+6],'(t)')
                        i += 6
        except ImportError:
            pass

        if store:
            for k,c in self.cameras.items():
                if c.variable:
                    T = camera_transforms[k]
                    c.local_coordinates = T
            for k,m in self.markers.items():
                if m.variable:
                    T = marker_transforms[k]
                    m.local_coordinates = T
        return rmse,camera_transforms,marker_transforms

    def updateRobotSensors(self) -> None:
        """Uses the current camera transforms to update the robot model's
        sensors.
        """
        for i,c in self.cameras.items():
            try:
                s = self.robot.sensor(i)
                if s.name()=='':
                    raise ValueError()
            except Exception:
                #create a new camera on the robot
                s = self.robot.addSensor(str(i),'CameraSensor')
                sensing.intrinsics_to_camera(c.intrinsics,s,'json')
                s.setSetting('zmin','0.1')
                s.setSetting('zmax','10')
                s.setSetting('link',str(_linkIndex(c.link,self.robot)))
            sensing.set_sensor_xform(s,c.local_coordinates)

    def save(self,fn) -> None:
        """Saves to a JSON file on disk. 
        
        .. warning::

            Images will not be saved.
        
        Args:
            fn (str or file): the file to save to.
        """
        if isinstance(fn,str):
            with open(fn,'w') as f:
                return self.save(f)
        import json
        jsonobj = {}
        if self.robot is not None:
            jsonobj['robot'] = self.robot.getName()
        jsonobj['cameras'] = dict((k,_CameraIO.toJson(c)) for k,c in self.cameras.items())
        jsonobj['frames'] = [None for f in self.frames]
        jsonobj['frame_camera_ids'] = self.frame_camera_ids
        jsonobj['markers'] = dict((k,_MarkerIO.toJson(m)) for k,m in self.markers.items())
        jsonobj['configurations'] = self.configurations
        jsonobj['observations'] = [_ObservationIO.toJson(c) for c in self.observations]
        json.dump(jsonobj)

    def load(self,fn) -> None:
        """Loads from a JSON file on disk.
        
        Args:
            fn (str or file): the file to load from
        """
        if isinstance(fn,str):
            with open(fn,'r') as f:
                return self.load(f)
        import json
        jsonobj = json.load(fn)
        if 'robot' in jsonobj:
            robotname = jsonobj['robot']
            if self.robot is not None:
                if self.robot.getName() != robotname:
                    import warnings
                    warnings.warn("Robot name in calibration file does not match robot object")
        self.cameras = dict((k,_CameraIO.fromJson(c)) for k,c in jsonobj['cameras'].items())
        self.frames = jsonobj['frames']
        self.frame_camera_ids = jsonobj['frame_camera_ids']
        self.configurations = jsonobj['configurations']
        self.markers = dict((k,_MarkerIO.fromJson(c)) for k,c in jsonobj['markers'].items())
        self.observations = [_ObservationIO.fromJson(c) for c in jsonobj['observations']]


def _linkIndex(link,robot):
    if link is None:
        return -1
    if isinstance(link,int):
        return link
    else:
        return robot.link(link).index

def _linkTransform(link,robot):
    link = _linkIndex(link,robot)
    if link < 0:
        return se3.identity()
    return robot.link(link).getTransform()

def _worldTransform(link,robot,localTransform):
    link = _linkIndex(link,robot)
    if localTransform is None:
        localTransform = se3.identity()
    if link < 0:
        return localTransform
    return se3.mul(robot.link(link).getTransform(),localTransform)

def _worldPosition(link,robot,localPosition):
    link = _linkIndex(link,robot)
    if localPosition is None:
        localPosition = [0.0]*3
    if link < 0:
        return localPosition
    return se3.mul(robot.link(link).getTransform(),localPosition)

def _localTransform(link,robot,worldTransform):
    link = _linkIndex(link,robot)
    if worldTransform is None:
        worldTransform = se3.identity()
    if link < 0:
        return worldTransform
    return se3.mul(se3.inv(robot.link(link).getTransform()),worldTransform)

def _localPosition(link,robot,worldPosition):
    link = _linkIndex(link,robot)
    if worldPosition is None:
        worldPosition = [0.0]*3
    if link < 0:
        return worldPosition
    return se3.mul(se3.inv(robot.link(link).getTransform()),worldPosition)

class _MarkerIO:
    @staticmethod
    def toJson(obj):
        if isinstance(obj,PointMarker):
            return {'type':'Point','link':obj.link,'local_coordinates':obj.local_coordinates,'variable':obj.variable}
        else:
            return {'type':'Transform','link':obj.link,'local_coordinates':obj.local_coordinates,'local_features':obj.local_features,'variable':obj.variable}

    @staticmethod
    def fromJson(obj):
        if obj['type'] == 'Point':
            res = PointMarker(obj['link'],obj['local_coordinates'])
            res.variable = obj['variable']
        else:
            res = TransformMarker(obj['link'],obj['local_coordinates'],obj['local_features'])
            res.variable = obj['variable']
        return res

class _CameraIO:
    @staticmethod
    def toJson(obj):
        return {'link':obj.link,'intrinsics':obj.intrinsics,'local_coordinates':obj.local_coordinates,'variable':obj.variable}

    @staticmethod
    def fromJson(obj):
        res = CameraInfo(obj['link'],obj['intrinsics'],obj['local_coordinates'])
        res.variable = obj['variable']
        return res


class _ObservationIO:
    @staticmethod
    def toJson(obj):
        if isinstance(obj,PixelObservation):
            return {'type':'Pixel','value':obj.value,'camera_id':obj.camera_id,'frame_id':obj.frame_id,'marker_id':obj.marker_id,'feature_id':obj['feature_id']}
        elif isinstance(obj,PointObservation):
            return {'type':'Point','value':obj.value,'camera_id':obj.camera_id,'frame_id':obj.frame_id,'marker_id':obj.marker_id,'feature_id':obj['feature_id']}
        else:
            return {'type':'Transform','value':obj.value,'camera_id':obj.camera_id,'frame_id':obj.frame_id,'marker_id':obj.marker_id}

    @staticmethod
    def fromJson(obj):
        if obj['type'] == 'Pixel':
            return PixelObservation(obj['value'],obj['camera_id'],obj['frame_id'],obj['marker_id'],obj['feature_id'])
        elif obj['type'] == 'Point':
            return PointObservation(obj['value'],obj['camera_id'],obj['frame_id'],obj['marker_id'],obj['feature_id'])
        elif obj['type'] == 'Transform':
            return TransformObservation(obj['value'],obj['camera_id'],obj['frame_id'],obj['marker_id'])
    