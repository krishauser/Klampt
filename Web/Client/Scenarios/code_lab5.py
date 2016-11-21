import math
from klampt import vectorops,so2

#CONFIGURATION VARIABLES

environment = 'normal'
#environment = 'fancy'

method = 'accumulator'
#method = 'occupancy_grid'
#method = 'robust_occupancy_grid'
#method = 'probabilistic_occupancy_grid'

noisy_sensor = False

def init():
    pass

def depthImageToPointCloud(scan):
    """In:
    - scan: a 1D array of quantized depth values
    Out:
    - a list of 2D points giving the point cloud in the
      sensor-local frame.  y axis is forward, x axis is to the right.
      units are in meters.
    """
    #TODO: implement me.  This return array is incorrect
    return [(1,float(s)/100) for s in scan if s != 0]

def localToWorld(localPC,robotState,cameraExtrinsics):
    """Convert a local point cloud to world coordinates.
    In:
    - localPC: a point cloud in camera-local coordinates
      with x axis to the right, y axis forward.
    - robotState: an (x,y,theta) tuple
    - cameraExtrinsics: the position of the camera, in robot-
      local coordinates.
    Out:
    - a point cloud in world x-y coordinates.
    """
    #TODO: implement me.  This return value is incorrect, it just
    #adds on the local camera displacement
    worldPC = []
    for p in localPC:
        worldPC.append((p[0]+cameraExtrinsics[0],p[1]+cameraExtrinsics[1]))
    return worldPC


class PointCloudMap:
    """A 2D map consisting of a list of points"""
    
    def __init__(self,points):
        self.points = points

class GridMap:
    """A 2D map consisting of a grid of occupancy levels.
    - array: a 2D array of numeric values, taking the range from 0 (unoccupied) to 1
      (occupied).  The array is laid out so array[i][j] is in the i'th cell in the
      x-direction and the j'th cell in the y-direction
    - bounds: a pair of tuples [(xmin,ymin),(xmax,ymax)] giving the rectangle
      holding the map
    - vmin, vmax: the minimum and maximum values in the array. Automatically computed
      (usually 0 and 1).
    """
    
    def __init__(self,array,bounds):
        self.array = array
        self.bounds = bounds
        self.vmin = 0
        self.vmax = 1
        for row in array:
            for v in row:
                self.vmin = min(self.vmin,v)
                self.vmax = max(self.vmax,v)
        
class MapperBase:
    """By default, just keeps the last scans as a point cloud. """
    def __init__(self):
        self.points = []
    def addScan(self,Tsensor,pointList):
        """Adds the point list to the map.
        - Tsensor: a rigid transform of the sensor in world coordinates
          (Y axis points forward, X axis points to the right, Z points up)
        - pointList: a list of 2-tuples giving the point cloud in world
          coordinates
        """
        self.points = pointList
    def getMap(self):
        """Returns a PointCloudMap or a GridMap describing the world."""
        return PointCloudMap(self.points)

class AccumulatorMapper(MapperBase):
    """Simply adds new scans to a point cloud. """
    def __init__(self):
        self.points = []
    def addScan(self,Tsensor,pointList):
        self.points += pointList
    def getMap(self):
        return PointCloudMap(self.points)

class OccupancyGridMapper(MapperBase):
    """Adds new scans to an occupancy grid.  You will need to implement
    the addScan() method.

    Attributes:
    - bounds: a pair [bmin,bmax] where bmin is the lower left corner and
      bmax is the upper right corner of the range of the map.
    - res: a pair (xres,yres) giving the resolution of the grid in the x and
      y directions.
    - grid: a 2D array (list of lists) containing occupancies (0 for empty,
      1 for filled)
    """
    def __init__(self,width,height,xres,yres):
        self.bounds = [(-0.5*width,-0.5*height),(0.5*width,0.5*height)]
        self.res = (xres,yres)
        self.grid = [[0]*yres for i in range(xres)]
    def addScan(self,Tsensor,pointList):
        #TODO: implement me
        pass
    def getMap(self):
        return GridMap(self.grid,self.bounds)

class RobustOccupancyGridMapper(MapperBase):
    """Adds new scans to an occupancy grid.  You will need to implement
    the addScan() method.

    Attributes:
    - bounds: a pair [bmin,bmax] where bmin is the lower left corner and
      bmax is the upper right corner of the range of the map.
    - res: a pair (xres,yres) giving the resolution of the grid in the x and
      y directions.
    - grid: a 2D array (list of lists) containing *counts*.
    """
    def __init__(self,width,height,xres,yres):
        self.bounds = [(-0.5*width,-0.5*height),(0.5*width,0.5*height)]
        self.res = (xres,yres)
        self.grid = [[0]*yres for i in range(xres)]
        #TODO: tune me
        self.threshold = 0.2
        self.minPoints = 0
    def addScan(self,Tsensor,pointList):
        #TODO: implement me
        pass
    def getMap(self):
        #TODO: you may wish to produce a new grid rather than the grid of
        #counts
        return GridMap(self.grid,self.bounds)


class ProbabilisticOccupancyGridMapper(MapperBase):
    """Adds new scans to an occupancy grid.  You will need to implement
    the addScan() method.

    Attributes:
    - bounds: a pair [bmin,bmax] where bmin is the lower left corner and
      bmax is the upper right corner of the range of the map.
    - res: a pair (xres,yres) giving the resolution of the grid in the x and
      y directions.
    - occupancyCounts, freeCounts: a 2D array (list of lists) containing
      *counts*.
    """
    def __init__(self,width,height,xres,yres):
        self.bounds = [(-0.5*width,-0.5*height),(0.5*width,0.5*height)]
        self.res = (xres,yres)
        occupancyPrior = 2
        freePrior = 3
        self.occupancyCounts = [[occupancyPrior]*yres for i in range(xres)]
        self.freeCounts = [[freePrior]*yres for i in range(xres)]
    def addScan(self,Tsensor,pointList):
        #TODO: implement me
        pass
    def getMap(self):
        #TODO: you may wish to produce a new grid rather than the grid of
        #counts
        return GridMap(self.occupancyCounts,self.bounds)


def get_control(t):
    """Used by the server to determine how the robot should drive. Return value is
    a tuple (vfwd,vleft,turnrate)"""
    if t % 2 < 1.5:
        return (1,0,3)
    else:
        return (1,0,-3)

