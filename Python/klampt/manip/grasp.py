from __future__ import annotations
from ..model.robotinfo import GripperInfo
from ..model.contact import ContactPoint
from ..model import ik
from .. import io
from .. import IKObjective, IKSolver, RobotModel, RobotModelLink
from ..math import se3,vectorops
import copy
from ..model.typing import Config, RigidTransform
from typing import List,Optional,Union

class Grasp:
    """A "fully-qualified" grasp for an arbitrary gripper. A grasp specifies
    an IK constraint and finger configuration.

    Optional estimated contacts may be given, and these are specified in world
    space.
    """
    def __init__(self, ikConstraint:Optional[IKObjective]=None,
                 fingerLinks:Optional[List[int]]=None,
                 fingerConfig:Optional[List[float]]=None,
                 contacts:Optional[List[ContactPoint]]=None,
                 score:Optional[float]=None):
        self.ikConstraint = ikConstraint
        self.fingerLinks = fingerLinks if fingerLinks is not None else []
        self.fingerConfig = fingerConfig if fingerConfig is not None else []
        self.contacts = contacts if contacts is not None else []
        if ikConstraint is not None:
            if not isinstance(ikConstraint,IKObjective):
                raise TypeError("ikConstraint must be an IKObjective")
        for c in self.contacts:
            if not isinstance(c,ContactPoint):
                raise TypeError("contacts must be a list of ContactPoint")
        self.score = score
    
    def __str__(self):
        if self.score is not None:
            return "Grasp link={} at {}, fingers={} score={}".format(self.ikConstraint.link(),self.ikConstraint.getPosition()[1],
                ' '.join('%d=%.2f'%(i,v) for (i,v) in zip(self.fingerLinks,self.fingerConfig)),self.score)
        return "Grasp link={} at {}, fingers={}".format(self.ikConstraint.link(),self.ikConstraint.getPosition()[1],
            ' '.join('%d=%.2f'%(i,v) for (i,v) in zip(self.fingerLinks,self.fingerConfig)))

    @staticmethod
    def fixedFromCurrent(robot: RobotModel,
                         link : Union[int,str,RobotModelLink],
                         gripper_info : Optional[GripperInfo] = None) -> Grasp: 
        """Creates a fixed Grasp from the current robot configuration.  The
        given link is the link to be fixed.  If gripper_info is given, then
        the finger links will be taken from that, otherwise they will
        be auto-detected as all descendant links of the given link."""
        if not isinstance(link,RobotModelLink):
            link = robot.link(link)
        constraint = ik.fixed_objective(link)
        if gripper_info is not None:
            finger_links = gripper_info.fingerLinks
        else:
            #auto-detect finger links as descendants of the given link
            finger_links = set()
            finger_links.add(link.index)
            for i in range(robot.numLinks()):
                if robot.link(i).getParentIndex() in finger_links:
                    finger_links.add(i)
            finger_links = list(sorted(finger_links))
        q = robot.getConfig()
        finger_config = [q[i] for i in finger_links]
        return Grasp(constraint,finger_links,finger_config,[],None)

    def gripperLink(self) -> Optional[int]:
        """Returns the link that is fixed by the IK constraint"""
        if self.ikConstraint is None: return None
        return self.ikConstraint.link()

    def setFingerConfig(self, q : Config) -> Config:
        """Given a full robot config q, returns a config but with the finger
        degrees of freedom matching those of this grasp.
        """
        qf = [v for v in q]
        for (i,v) in zip(self.fingerLinks,self.fingerConfig):
            qf[i] = v
        return qf

    def getFingerConfig(self, q : Config) -> Config:
        """Given a full robot config q, returns the finger
        degrees of freedom matching those of this grasp.
        """
        return [q[i] for i in self.fingerLinks]

    def getIKSolver(self, robot : RobotModel) -> IKSolver:
        """Returns a configured IK solver that will try to achieve the
        specified IK constraint while keeping the finger configurations
        fixed.
        """
        obj = self.ikConstraint.copy()
        obj.robot = robot
        solver = ik.solver(obj)
        q = robot.getConfig()
        robot.setConfig(self.setFingerConfig(q))
        active = solver.getActiveDofs()
        marked_active = [False]*robot.numLinks()
        for a in active:
            marked_active[a] = True
        for l in self.fingerLinks:
            marked_active[l] = False
        active = [i for i,a in enumerate(marked_active) if a]
        solver.setActiveDofs(active)
        return solver

    def getTransformed(self, xform : RigidTransform) -> Grasp:
        """Returns a copy of self, transformed by xform.
        """
        obj = self.ikConstraint.copy()
        obj.transform(*xform)
        world_contacts = [copy.copy(c) for c in self.contacts]
        for c in world_contacts:
            c.transform(xform)
        return Grasp(obj,self.fingerLinks,self.fingerConfig,world_contacts,self.score)

    def asFixedGrasp(self,Tref:Optional[RigidTransform]=None)->Grasp:
        """For non-fixed grasps, returns a Grasp with a fully specified base
        transform.  To sample fixed grasps, pass random values for Tref (e.g.,
        `(so3.sample(),[0,0,0]))` to randomize orientation."""
        if Tref is None:
            Tref = se3.identity()
        Tfixed = self.ikConstraint.closestMatch(*Tref)
        ik2 = self.ikConstraint.copy()
        ik2.setFixedTransform(self.ikConstraint.link(),*Tfixed)
        return Grasp(ik2,self.fingerLinks,self.fingerConfig,self.contacts,self.score)

    def transfer(self,gripper_source : GripperInfo, gripper_dest : GripperInfo) -> Grasp:
        """Creates a copy of this Grasp so that it can be used for another
        gripper.  self must match gripper_source, and the number of finger
        DOFs in gripper_source and gripper_dest must match.
        """
        if gripper_source.baseLink != self.ikConstraint.link():
            raise ValueError("Invalid gripper source? base_link doesn't match")
        if self.fingerLinks != gripper_source.fingerLinks:
            raise ValueError("Invalid gripper source? fingerLinks doesn't match")
        if len(gripper_dest.fingerLinks) != len(gripper_source.fingerLinks):
            raise ValueError("Invalid gripper destination? fingerLinks doens't match")
        obj = self.ikConstraint.copy()
        obj.setLinks(gripper_dest.baseLink,obj.destLink())
        return Grasp(obj,gripper_dest.fingerLinks,self.fingerConfig,self.contacts,self.score)

    def toJson(self) -> dict:
        """Returns a JSON-compatible object storing all of the grasp data."""
        return {'ikConstraint':io.to_json(self.ikConstraint),'fingerLinks':self.fingerLinks,'fingerConfig':self.fingerConfig,'score':self.score}

    @staticmethod
    def fromJson(jsonObj : dict) -> Grasp:
        """Creates the grasp from a JSON-compatible object previously saved by
        toJson.
        """
        constraint = io.from_json(jsonObj['ikConstraint'])
        links = jsonObj['fingerLinks']
        config = jsonObj['fingerConfig']
        score = jsonObj.get('score',None)
        return Grasp(constraint,links,config,None,score)

    def addToVis(self, prefix:str, gripper:Optional[GripperInfo]=None, hide_label=True):
        from klampt import vis
        for i,c in enumerate(self.contacts):
            vis.add(prefix+"_c"+str(i),c,hide_label=hide_label)
        if gripper is not None and gripper.type == 'parallel':
            #add a 'T' corresponding to the grasp width
            opening_amount = gripper.configToOpening(self.fingerConfig)
            w = gripper.openingToWidth(opening_amount)
            T = self.asFixedGrasp().ikConstraint.getTransform()
            if gripper.primaryAxis is not None and gripper.secondaryAxis is not None:
                print("Drawing parallel gripper grasp with width",w)
                outer_point = vectorops.madd(gripper.center,gripper.primaryAxis,gripper.fingerLength)
                top = [vectorops.madd(outer_point,gripper.secondaryAxis,-w/2), vectorops.madd(outer_point,gripper.secondaryAxis,w/2)]
                middle = [gripper.center, outer_point]
                top = [se3.apply(T,p) for p in top]
                middle = [se3.apply(T,p) for p in middle]
                vis.add(prefix+"_primary",middle,hide_label=hide_label,color=(1,0,0,1))
                vis.add(prefix+"_secondary",top,hide_label=hide_label,color=(0,1,0,1))
            else:
                vis.add(prefix+"_ik",self.ikConstraint,hide_label=hide_label)
        elif gripper is not None and gripper.type == 'vacuum':
            T = self.asFixedGrasp().ikConstraint.getTransform()
            if gripper.primaryAxis is not None:
                outer_point = vectorops.madd(gripper.center,gripper.primaryAxis,gripper.fingerLength)
                vis.add(prefix+"_primary",[se3.apply(T,gripper.center),se3.apply(T,outer_point)],hide_label=hide_label,color=(1,0,0,1))
            else:
                vis.add(prefix+"_ik",self.ikConstraint,hide_label=hide_label)
        else:
            vis.add(prefix+"_ik",self.ikConstraint,hide_label=hide_label)

    def removeFromVis(self, prefix:str, gripper:Optional[GripperInfo]=None):
        from klampt import vis
        vis.remove(prefix+"_ik")
        for i,c in enumerate(self.contacts):
            vis.remove(prefix+"_c"+str(i))



