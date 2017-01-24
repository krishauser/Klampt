from robotsim import *
import atexit
atexit.register(destroy)

__all__ = ['WorldModel','RobotModel','RobotModelLink','RigidObjectModel','TerrainModel','Mass','ContactParameters',
           'SimRobotController','SimBody','Simulator',
           'Geometry3D','Appearance','TriangleMesh','PointCloud','GeometricPrimitive',
           'IKObjective','IKSolver','GeneralizedIKObjective','GeneralizedIKSolver',
           'model','math','io','plan','sim']


