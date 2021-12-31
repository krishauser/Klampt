#all of the classes defined in the package that you'd like to cross-reference
classes = {'RobotModel','WorldModel','RobotModelLink','RobotModelDriver','TerrainModel','RigidObjectModel',
    'IKObjective','IKSolver',
    'Geometry3D','Appearance','Viewport','Widget','TriangleMesh','PointCloud','VolumeGrid','GeometricPrimitive','ConvexHull',
    'ContactParameters','Mass','DistanceQueryResult','DistanceQuerySettings','ContactQueryResult',
    'SimRobotSensor','SimRobotController','SimBody','Simulator',
    'CSpaceInterface','MotionPlanInterface',
    'ContactPoint','Trajectory','RobotTrajectory','HermiteTrajectory','MultiPath',
    'CSpace','MotionPlan'
    }

#default prefix for class cross-references
default_class_prefix = 'klampt'

#other prefixes for class cross-references
class_prefix = {
    'ContactPoint':'klampt.model.contact',
    'Trajectory':'klampt.model.trajectory',
    'RobotTrajectory':'klampt.model.trajectory',
    'HermiteTrajectory':'klampt.model.trajectory',
    'MultiPath':'klampt.model.multipath',
    'CSpace':'klampt.plan.cspace',
    'MotionPlan':'klampt.plan.cspace',
}

basic_types = {'int','float','str','bytes','bool'}

#location to inject typing string
typing_inject_location = 'import __builtin__'
typing_header = """
from typing import Sequence,Tuple,Iterator
from klampt.model.typing import IntArray,Vector,Vector3,Point,Rotation
"""

#conversions from SWIG docstrings to RST docstrings
to_python_types = {
    'char *':'str',
    'char':'str',
    'std::string':'str',
    'double':'float',
    'doubleVector': "list of floats",
    'doubleArray': "list of floats",
    'doubleMatrix': "2D matrix, list of list of floats",
    'floatVector': "list of floats",
    'floatArray': "list of floats",
    'intVector': "list of int",
    'intArray': "list of int",
    'stringArray': "list of str",
    'double const [3]': "list of 3 floats",
    'double [3]': "list of 3 floats",
    'double const [9]': "list of 9 floats (so3 element)",
    'double [9]': "list of 9 floats (so3 element)",
    'std::vector< unsigned char,std::allocator< unsigned char > > const':'bytes',
    'std::vector<(float,std::allocator<(float)>)>':'list of floats',
    'std::vector<(double,std::allocator<(double)>)>':'list of floats',
    'unsigned_char * np_array':'1D Numpy array of np.uint8',
    'unsigned_char * np_array2':'2D Numpy array of np.uint8',
    'int * np_array':'1D Numpy array of ints',
    'int * np_array2':'2D Numpy array of ints',
    'float * np_array':'1D Numpy array of np.float32',
    'float * np_array2':'2D Numpy array of np.float32',
    'float * np_array3':'3D Numpy array of np.float32',
    'double * np_array':'1D Numpy array of floats',
    'double * np_array2':'2D Numpy array of floats',
    'double * np_array3':'3D Numpy array of floats',
    'PyObject': "object",
    'PyObject *': "object"
}

to_python_type_hints = {
    'void' : 'None',
    'bool' : 'bool',
    'int' : 'int',
    'float' : 'float',
    'size_t' : 'int',
    'ptrdiff_t' : 'int',
    'SwigPyIterator': 'Iterator',
    'swig::SwigPyIterator': 'Iterator',
    'swig::SwigPyIterator *': 'Iterator',
    'intArray *': '"intArray"',
    'floatArray *': '"floatArray"',
    'doubleArray *': '"doubleArray"',
    'intVector *': '"intVector"',
    'floatVector *': '"floatVector"',
    'doubleVector *': '"doubleVector"',
    'stringVector *': '"stringVector"',
    "std::vector< int,std::allocator< int > > *": '"intVector"',
    "std::vector< float,std::allocator< float > > *": '"floatVector"',
    "std::vector< double,std::allocator< double > > *": '"doubleVector"',
    "std::vector< std::string,std::allocator< std::string > > *": '"stringVector"',
    "std::vector< std::vector< double,std::allocator< double > >,std::allocator< std::vector< double,std::allocator< double > > > > *": '"doubleMatrix"',
    "std::vector< int >::value_type" : "int",
    "std::vector< float >::value_type" : "float",
    "std::vector< double >::value_type" : "float",
    "std::vector< std::string >::value_type" : "str",
    "std::vector< std::vector< double > >::value_type" : "Vector",
    "std::map< std::string,std::string >::key_type" : 'str',
    "std::map< std::string,std::string >::mapped_type" : 'str',
    "std::vector< std::string,std::allocator< std::string > >" : "Sequence[str]",
    'unsigned char *':'"ndarray"',
    'unsigned short *':'"ndarray"',
    'unsigned int *':'"ndarray"',
    'int *' : 'IntArray',
    'float *' : 'Vector',
    'double *' : 'Vector',
    'intArray' : 'IntArray',
    'intVector' : 'IntArray',
    'floatArray' : 'Vector',
    'floatVector' : 'Vector',
    'doubleArray' : 'Vector',
    'doubleVector' : 'Vector',
    'doubleMatrix' : 'Sequence[Sequence[float]]',
    "double [9]" : 'Rotation',
    "double [3]" : 'Point',
    "double const [9]" : 'Rotation',
    "double const [3]" : 'Point',
    "double [4]" : "Sequence[float]",
    "double [16]" : "Sequence[float]",
    "std::vector< double > &, std::vector< double > &, std::vector< double > &": "Tuple[Vector,Vector,Vector]",
    "std::vector< double > &, std::vector< std::vector< double > > &, std::vector< std::vector< double > > &": "Tuple[Vector,Sequence[Vector],Sequence[Vector]]",
    "std::vector< std::vector< double > > &, std::vector< std::vector< double > > &, std::vector< std::vector< double > > &": "Tuple[Sequence[Vector],Sequence[Vector],Sequence[Vector]]",
    "Klampt::SensorBase *" : None,
}

"""SWIG's bindings of STL has a bunch of garbage"""
stl_to_python_type_hints = {
    "::size_type" : 'int',
    "::difference_type" : 'int',
    "::iterator" : "Iterator",
    "::reverse_iterator" : "Iterator",
    "::allocator_type" : None,
}

to_python_defaults = {'NULL':"None"}
