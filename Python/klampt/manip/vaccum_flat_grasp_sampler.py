from ..model.gripperinfo import GripperInfo
from .grasp import Grasp
from .grasp_sampler import GraspSamplerBase
from .. import Geometry3D,PointCloud,RigidObjectModel,IKObjective
from ..math import vectorops,so3,se3
from ..model.geometry import fit_plane,fit_plane_centroid
import math
import numpy as np
from typing import Union,Optional,Callable


class FlatAreaGraspSampler(GraspSamplerBase):
    """A GraspSamplerBase subclass that will find flat areas in a
    point cloud for a vacuum gripper.

    There are two objectives for scoring points.  The first is roughness,
    which is the variance of the distance of points to the plane fitted
    to the flat area.  The second is verticality, which is the angle of the
    plane's normal with respect to the gripper's primary axis.  The
    verticality is penalized by the vertical_penalty parameter, which can
    be a constant or a function of the angle.  The roughness is penalized
    by the roughness_penalty parameter, which can be a constant or a function
    of the variance.  The score is the sum of the verticality and roughness
    penalties, and the "likelihood of success" is given by exp(-score).

    All the calculation is done in the init() method, so next() just
    iterates through the flat areas found.
    """
    def __init__(self, gripper : GripperInfo, roughness_penalty : Union[float,Callable], vertical_penalty : Union[float,Callable]):
        self._gripper = gripper
        assert gripper.primaryAxis is not None,"Gripper needs a primary axis"
        assert gripper.maximumSpan is not None,"Gripper needs an opening span"
        self.roughness_penalty = roughness_penalty
        self.vertical_penalty = vertical_penalty
        if not callable(roughness_penalty):
            self.roughness_penalty = lambda var:var*roughness_penalty
        if not callable(vertical_penalty):
            self.vertical_penalty = lambda angle:angle*vertical_penalty
        self.pc = None
        self.pc_xform = None
        self.options = None
        self.index = None

    def gripper(self):
        return self._gripper

    def init(self,scene,object:Union[RigidObjectModel,Geometry3D,PointCloud],hints=None) -> bool:
        """Needs object to contain a structured PointCloud."""
        if not isinstance(object,(RigidObjectModel,Geometry3D,PointCloud)):
            print("Need to pass an object as a RigidObjectModel, Geometry3D, or PointCloud")
            return False
        if isinstance(object,RigidObjectModel):
            return self.init(scene,object.geometry(),hints)
        pc = None
        xform = None
        if isinstance(object,Geometry3D):
            pc = object.getPointCloud()
            xform = object.getCurrentTransform()
        else:
            pc = object
            xform = se3.identity()
        self.pc = pc
        self.pc_xform = xform

        #now look through PC and find flat parts
        #do a spatial hash
        from collections import defaultdict
        estimation_knn = 6
        pts = pc.points
        N = pts.shape[0]
        positions = pts[:,:3]
        normals = np.zeros((N,3))
        indices = (positions * (1.0/self._gripper.maximumSpan)).astype(int)
        pt_hash = defaultdict(list)
        for i,(ind,p) in enumerate(zip(indices,positions)):
            pt_hash[tuple(ind)].append((i,p))
        options = []
        for (ind,iplist) in pt_hash.items():
            if len(iplist) < estimation_knn:
                pass
            else:
                pindices = [ip[0] for ip in iplist]
                pts = [ip[1] for ip in iplist]
                try:
                    c,n = fit_plane_centroid(pts)
                except ValueError:
                    # print("Have less than 3 points to fit a plane at index",ind,":",len(pts))
                    continue  # not enough points to fit a plane
                cw = se3.apply(xform,c)
                nw = so3.apply(xform[0],n) 
                if nw[2] < 0:
                    nw = vectorops.mul(nw,-1)
                verticality = self.vertical_penalty(math.acos(nw[2]))
                var = sum(vectorops.dot(vectorops.sub(p,c),n)**2 for p in pts)
                roughness = self.roughness_penalty(var)
                options.append((cw,nw,verticality + roughness))
        if len(options) == 0:
            return False
        self.options = sorted(options,key=lambda x:x[2])
        self.index = 0
        return True

    def next(self) -> Optional[Grasp]:
        """Returns the next Grasp from the database."""
        if self.options is None:
            return None
        
        if self.index >= len(self.options):
            self.options = None
            return None

        cworld,nworld,score = self.options[self.index]
        self.index += 1
        # cworld = se3.apply(self.pc_xform,c)
        # nworld = so3.apply(self.pc_xform[0],n)
        objective = IKObjective()
        objective.setLinks(self._gripper.baseLink)
        objective.setFixedPosConstraint(self._gripper.center,cworld)
        objective.setAxialRotConstraint(self._gripper.primaryAxis,vectorops.mul(nworld,-1))
        score = math.exp(-score)
        return Grasp(objective,score=score)
    
    def score(self):
        """Returns the top score.
        """
        if self.options is None: return 0
        if self.index >= len(self.options):
            return 0
        return math.exp(-self.options[self.index][2])

    def visDebug(self):
        """Visualizes the flat areas found."""
        from klampt import vis
        if self.options is None: return
        if self.index >= len(self.options):
            return
        for i,(cworld,nworld,score) in enumerate(self.options[self.index:]):
            # cworld = se3.apply(self.pc_xform,c)
            # nworld = so3.apply(self.pc_xform[0],n)
            score = math.exp(-score)
            opacity = score
            #vis.add(f"flat_contact_{i+self.index}",cworld,color=(1,0,0,opacity),size=9,hide_label=True)
            vis.add(f"flat_contact_n_{i+self.index}",[cworld,vectorops.add(cworld,vectorops.mul(nworld,0.05))],color=(1,0,0,opacity),hide_label=True)