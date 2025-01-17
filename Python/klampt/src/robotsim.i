%module(docstring="Klamp't Core Python bindings \n------------------------------") robotsim
%{
  #define SWIG_FILE_WITH_INIT
	#include <exception>
	#include <vector>
	#include "pyerr.h"
	#include "geometry.h"
	#include "appearance.h"
	#include "widget.h"
	#include "viewport.h"
	#include "robotmodel.h"
	#include "robotik.h"
	#include "robotsim.h"
  #include "robotio.h"
  #include "stability.h"
%}
%include "carrays.i"
%include "std_string.i"
%include "std_vector.i"
%include "std_map.i"
%include "numpy.i"


%init %{
import_array();
%}

%array_class(double, doubleArray);
%array_class(float, floatArray);
%array_class(int, intArray);

namespace std {
   %template(stringVector) vector<string>;
   %template(doubleVector) vector<double>;
   %template(floatVector) vector<float>;
   %template(intVector) vector<int>;
   %template(doubleMatrix) vector<vector<double> >;
   %template(stringMap) map<string,string>;
};

%exception {
	try {
		$action
	}
	catch(PyException& e) {
		e.setPyErr();
		return NULL;
	}
	catch(std::exception& e) {
		PyErr_SetString(PyExc_RuntimeError, const_cast<char*>(e.what()));
		return NULL;
	}
}

%{
static int convert_iarray(PyObject *input, int *ptr, int size) {
  int i;
  if (!PySequence_Check(input)) {
      PyErr_SetString(PyExc_TypeError,"Expecting a sequence");
      return 0;
  }
  if (PyObject_Length(input) != size) {
      PyErr_SetString(PyExc_ValueError,"Sequence size mismatch");
      return 0;
  }
  for (i =0; i < size; i++) {
      PyObject *o = PySequence_GetItem(input,i);
      if (!PyInt_Check(o)) {
        Py_XDECREF(o);
        PyErr_SetString(PyExc_ValueError,"Expecting a sequence of ints");
        return 0;
      }
      ptr[i] = PyInt_AsLong(o);
      Py_DECREF(o);
  }
  return 1;
}

static int convert_farray(PyObject *input, float *ptr, int size) {
  int i;
  if (!PySequence_Check(input)) {
      PyErr_SetString(PyExc_TypeError,"Expecting a sequence");
      return 0;
  }
  if (PyObject_Length(input) != size) {
      PyErr_SetString(PyExc_ValueError,"Sequence size mismatch");
      return 0;
  }
  for (i =0; i < size; i++) {
      PyObject *o = PySequence_GetItem(input,i);
      if (!PyNumber_Check(o)) {
        Py_XDECREF(o);
        PyErr_SetString(PyExc_ValueError,"Expecting a sequence of floats");
        return 0;
      }
      ptr[i] = (float)PyFloat_AsDouble(o);
      Py_DECREF(o);
  }
  return 1;
}

static int convert_darray(PyObject *input, double *ptr, int size) {
  int i;
  if (!PySequence_Check(input)) {
      PyErr_SetString(PyExc_TypeError,"Expecting a sequence");
      return 0;
  }
  if (PyObject_Length(input) != size) {
      PyErr_SetString(PyExc_ValueError,"Sequence size mismatch");
      return 0;
  }
  for (i =0; i < size; i++) {
      PyObject *o = PySequence_GetItem(input,i);
      if (!PyNumber_Check(o)) {
         Py_XDECREF(o);
         PyErr_SetString(PyExc_ValueError,"Expecting a sequence of floats");
         return 0;
      }
      else
        ptr[i] = PyFloat_AsDouble(o);
      Py_DECREF(o);
  }
  return 1;
}

static PyObject* convert_iarray_obj(const int *ptr, int size) {
  int i;
  PyObject* res = PyList_New(size);
  if(!res) {
      PyErr_SetString(PyExc_RuntimeError,"Couldn't allocate list of requested size");
      return NULL;
  }
  for (i=0; i < size; i++) {
      PyList_SetItem(res,i,PyInt_FromLong(ptr[i]));
  }
  return res;
}

static PyObject* convert_farray_obj(const float *ptr, int size) {
  int i;
  PyObject* res = PyList_New(size);
  if(!res) {
      PyErr_SetString(PyExc_RuntimeError,"Couldn't allocate list of requested size");
      return NULL;
  }
  for (i=0; i < size; i++) {
      PyList_SetItem(res,i,PyFloat_FromDouble((double)ptr[i]));
  }
  return res;
}

static PyObject* convert_darray_obj(const double *ptr, int size) {
  int i;
  PyObject* res = PyList_New(size);
  if(!res) {
      PyErr_SetString(PyExc_RuntimeError,"Couldn't allocate list of requested size");
      return NULL;
  }
  for (i=0; i < size; i++) {
      PyList_SetItem(res,i,PyFloat_FromDouble(ptr[i]));
  }
  return res;
}

static PyObject* convert_dmatrix_obj(const std::vector<std::vector<double> >& mat) {
  size_t i;
  PyObject* res = PyList_New(mat.size());
  if(!res) {
      PyErr_SetString(PyExc_RuntimeError,"Couldn't allocate list of requested size");
      return NULL;
  }
  for (i=0; i < mat.size(); i++) {
      PyObject* resi = convert_darray_obj(&(mat[i][0]),mat[i].size());
      if(!resi) {
  Py_XDECREF(res);
  return NULL;
      }
      PyList_SetItem(res,i,resi);
  }
  return res;
}



%}

%typemap(in) const int [ANY](int temp[$1_dim0]) {
   if (!convert_iarray($input,temp,$1_dim0)) {
      return NULL;
   }
   $1 = &temp[0];
}

%typemap(in) const float [ANY](float temp[$1_dim0]) {
   if (!convert_farray($input,temp,$1_dim0)) {
      return NULL;
   }
   $1 = &temp[0];
}

%typemap(in, numinputs=0) float out[ANY] (float temp[$1_dim0]) {
   $1 = &temp[0];
}

%typemap(in) const double [ANY](double temp[$1_dim0]) {
   if (!convert_darray($input,temp,$1_dim0)) {
      return NULL;
   }
   $1 = &temp[0];
}

//TODO Klampt 0.9: do we want to pass in tuples to setTransform?
/*
%typemap(in) (const double R[9],const double t[3]) (double Rtemp[9],double ttemp[3]) {
   if(!PySequence_Check($input)) {
      throw PyException("Must pass a 2-tuple as transform");
   }
   if(PySequence_Size($input) != 2) {
      throw PyException("Must pass a 2-tuple as transform");
   }
   PyObject* Rref = PySequence_GetItem($input,0);
   if (!convert_darray(Rref,Rtemp,9)) {
      Py_DECREF(Rref);
      throw PyException("First element must be a 9-vector");
   }
   PyObject* tref = PySequence_GetItem($input,1);
   if (!convert_darray(tref,ttemp,3)) {
      Py_DECREF(Rref);
      Py_DECREF(tref);
      throw PyException("Second element must be a 3-vector");
   }
   Py_DECREF(Rref);
   Py_DECREF(tref);
   $1 = Rtemp;
   $2 = ttemp;
}
*/

%typemap(in, numinputs=0) double out[ANY] (double temp[$1_dim0]) {
   $1 = &temp[0];
}

%typemap(in, numinputs=0) double out2[ANY] (double temp2[$1_dim0]) {
   $1 = &temp2[0];
}

%typemap(in, numinputs=0) std::vector<int>& out (std::vector<int> temp) {
   $1 = &temp;
}

%typemap(in, numinputs=0) std::vector<int>& out2 (std::vector<int> temp2) {
   $1 = &temp2;
}

%typemap(in, numinputs=0) std::vector<double>& out (std::vector<double> temp) {
   $1 = &temp;
}

%typemap(in, numinputs=0) std::vector<double>& out2 (std::vector<double> temp2) {
   $1 = &temp2;
}

%typemap(in, numinputs=0) std::vector<std::vector<double> >& out (std::vector<std::vector<double> > temp) {
   $1 = &temp;
}

%typemap(in, numinputs=0) std::vector<std::vector<double> >& out2 (std::vector<std::vector<double> > temp2) {
   $1 = &temp2;
}

%typemap(in, numinputs=0) std::vector<std::vector<double> >& out3 (std::vector<std::vector<double> > temp3) {
   $1 = &temp3;
}

%typemap(argout) float out[ANY] {
    PyObject *o, *o2, *o3;
    o = convert_farray_obj($1,$1_dim0);
    if ((!$result) || ($result == Py_None)) {
        $result = o;
    } else {
        if (!PyTuple_Check($result)) {
            PyObject *o2 = $result;
            $result = PyTuple_New(1);
            PyTuple_SetItem($result,0,o2);
        }
        o3 = PyTuple_New(1);
        PyTuple_SetItem(o3,0,o);
        o2 = $result;
        $result = PySequence_Concat(o2,o3);
        Py_DECREF(o2);
        Py_DECREF(o3);
    }
}

%typemap(argout) double out[ANY] {
    PyObject *o, *o2, *o3;
    o = convert_darray_obj($1,$1_dim0);
    if ((!$result) || ($result == Py_None)) {
        $result = o;
    } else {
        if (!PyTuple_Check($result)) {
            PyObject *o2 = $result;
            $result = PyTuple_New(1);
            PyTuple_SetItem($result,0,o2);
        }
        o3 = PyTuple_New(1);
        PyTuple_SetItem(o3,0,o);
        o2 = $result;
        $result = PySequence_Concat(o2,o3);
        Py_DECREF(o2);
        Py_DECREF(o3);
    }
}

%apply double out[ANY] { double out2[ANY] };

%typemap(argout) std::vector<int>& out {
    PyObject *o, *o2, *o3;
    o = convert_iarray_obj(&(*$1)[0],(int)$1->size());
    if ((!$result) || ($result == Py_None)) {
        $result = o;
    } else {
        if (!PyTuple_Check($result)) {
            PyObject *o2 = $result;
            $result = PyTuple_New(1);
            PyTuple_SetItem($result,0,o2);
        }
        o3 = PyTuple_New(1);
        PyTuple_SetItem(o3,0,o);
        o2 = $result;
        $result = PySequence_Concat(o2,o3);
        Py_DECREF(o2);
        Py_DECREF(o3);
    }
}

%apply std::vector<int>& out { std::vector<int>& out2 };

%typemap(argout) std::vector<double>& out {
    PyObject *o, *o2, *o3;
    o = convert_darray_obj(&(*$1)[0],(int)$1->size());
    if ((!$result) || ($result == Py_None)) {
        $result = o;
    } else {
        if (!PyTuple_Check($result)) {
            PyObject *o2 = $result;
            $result = PyTuple_New(1);
            PyTuple_SetItem($result,0,o2);
        }
        o3 = PyTuple_New(1);
        PyTuple_SetItem(o3,0,o);
        o2 = $result;
        $result = PySequence_Concat(o2,o3);
        Py_DECREF(o2);
        Py_DECREF(o3);
    }
}

%apply std::vector<double>& out { std::vector<double>& out2 };

%apply std::vector<double>& out { std::vector<double>& kPout };

%apply std::vector<double>& out { std::vector<double>& kIout };

%apply std::vector<double>& out { std::vector<double>& kDout };

%typemap(argout) std::vector<std::vector<double> >& out {
    PyObject *o, *o2, *o3;
    o = convert_dmatrix_obj((*$1));
    if ((!$result) || ($result == Py_None)) {
        $result = o;
    } else {
        if (!PyTuple_Check($result)) {
            PyObject *o2 = $result;
            $result = PyTuple_New(1);
            PyTuple_SetItem($result,0,o2);
        }
        o3 = PyTuple_New(1);
        PyTuple_SetItem(o3,0,o);
        o2 = $result;
        $result = PySequence_Concat(o2,o3);
        Py_DECREF(o2);
        Py_DECREF(o3);
    }
}

%apply std::vector<std::vector<double> >& out { std::vector<std::vector<double> >& out2 };

%apply std::vector<std::vector<double> >& out { std::vector<std::vector<double> >& out3 };

%typemap(argout) std::vector<std::string> {
  int size = $1.size();
  $result = PyList_New(size);
  if(!res) {
      PyErr_SetString(PyExc_RuntimeError,"Couldn't allocate list of requested size");
  }
  else {
    for (i=0; i < size; i++) {
      PyList_SetItem($result,i,PyString_FromString($1[i].c_str()));
    }
  }
}

%apply (unsigned char* IN_ARRAY1,int DIM1) {(unsigned char* np_array,int m)};
%apply (unsigned char* IN_ARRAY2,int DIM1,int DIM2) {(unsigned char* np_array2, int m, int n)};
%apply (unsigned char* IN_ARRAY3,int DIM1,int DIM2,int DIM3) {(unsigned char* np_array3, int m, int n,int p)};
%apply (unsigned char** ARGOUTVIEW_ARRAY2,int* DIM1,int* DIM2) {(unsigned char** np_view2,int* m, int *n)};
%apply (unsigned char** ARGOUTVIEW_ARRAY3,int* DIM1,int* DIM2,int* DIM3) {(unsigned char** np_view3,int* m, int *n, int* p)};
%apply (unsigned short* IN_ARRAY2,int DIM1,int DIM2) {(unsigned short* np_array2, int m, int n)};
%apply (unsigned int* IN_ARRAY1,int DIM1) {(unsigned int* np_array, int m)};
%apply (unsigned int* IN_ARRAY2,int DIM1,int DIM2) {(unsigned int* np_array2, int m, int n)};
%apply (int* IN_ARRAY2,int DIM1,int DIM2) {(int* np_array2, int m, int n)};
%apply (int** ARGOUTVIEW_ARRAY2,int* DIM1,int* DIM2) {(int** np_view2,int* m, int *n)};
%apply (float* IN_ARRAY1, int DIM1) {(float* np_array,int m)};
%apply (float* IN_ARRAY2,int DIM1,int DIM2) {(float* np_array2, int m, int n)};
%apply (float* IN_ARRAY2,int DIM1,int DIM2) {(float* contacts, int m, int n)};
%apply (double* IN_ARRAY1, int DIM1) {(double* np_array,int m)};
%apply (double* IN_ARRAY2,int DIM1,int DIM2) {(double* np_array2, int m, int n)};
%apply (double* IN_ARRAY3,int DIM1,int DIM2, int DIM3) {(double* np_array3, int m, int n, int p)};
%apply (double** ARGOUTVIEW_ARRAY1,int* DIM1) {(double** np_view,int* m)};
%apply (double** ARGOUTVIEW_ARRAY2,int* DIM1,int* DIM2) {(double** np_view2,int* m, int *n)};
%apply (double** ARGOUTVIEW_ARRAY3,int* DIM1,int* DIM2,int* DIM3) {(double** np_view3,int* m, int *n, int* p)};
%apply (float** ARGOUTVIEWM_ARRAY1,int* DIM1) {(float** np_out,int* m)};
%apply (float** ARGOUTVIEWM_ARRAY2,int* DIM1,int* DIM2) {(float** np_out2,int* m, int *n)};
%apply (float** ARGOUTVIEWM_ARRAY3,int* DIM1,int* DIM2,int* DIM3) {(float** np_out3,int* m, int *n, int* p)};
%apply (double** ARGOUTVIEWM_ARRAY1,int* DIM1) {(double** np_out,int* m)};
%apply (double** ARGOUTVIEWM_ARRAY2,int* DIM1,int* DIM2) {(double** np_out2,int* m, int *n)};
%apply (double** ARGOUTVIEWM_ARRAY3,int* DIM1,int* DIM2,int* DIM3) {(double** np_out3,int* m, int *n, int* p)};

//typemaps specifically for PointCloud
%apply (unsigned short* IN_ARRAY2,int DIM1,int DIM2) {(unsigned short* np_depth2, int m2, int n2)};
%apply (float* IN_ARRAY2,int DIM1,int DIM2) {(float* np_depth2, int m2, int n2)};
%apply (double* IN_ARRAY2,int DIM1,int DIM2) {(double* np_depth2, int m2, int n2)};

%feature("python:annotations", "python");
%feature("autodoc","1");
%include "docs/docs.i"

%extend Mass { 
%pythoncode {
     com = property(getCom, setCom)
     """The object's center of mass in local coordinates (3-list)"""
     inertia = property(getInertia, setInertia)
     """The object's inertia in local coordinates (9-list)"""
}
}

%extend RobotModelLink { 
%pythoncode {
     name = property(getName, setName)
     parent = property(getParent, setParent)
     mass = property(getMass, setMass)
     parentTransform = property(getParentTransform)
     axis = property(getAxis,setAxis)
     prismatic = property(isPrismatic,setPrismatic)
     transform = property(getTransform)
}
}

%extend RobotModelDriver { 
%pythoncode {
     name = property(getName, setName)
     type = property(getType)
     affectedLink = property(getAffectedLink)
     affectedLinks = property(getAffectedLinks)
     value = property(getValue, setValue)
     velocity = property(getVelocity, setVelocity)
}
}

%extend RobotModel { 
%pythoncode {
     name = property(getName, setName)
     id = property(getID)
     config = property(getConfig,setConfig)
     velocity = property(getVelocity,setVelocity)

}
}

%extend RigidObjectModel { 
%pythoncode {
     name = property(getName, setName)
     id = property(getID)
     mass = property(getMass, setMass)
     transform = property(getTransform)
}
}

%extend TerrainModel { 
%pythoncode {
     name = property(getName, setName)
     id = property(getID)
}
}

%extend IKObjective {
%pythoncode {
    def __reduce__(self):
        from klampt.io import loader
        jsonobj = loader.to_json(self,'IKObjective')
        return (loader.from_json,(jsonobj,'IKObjective'))
}
}

%extend Appearance {
%pythoncode {
    def setTexture1D(self,format,array):
        """Sets a 1D texture.

        Args:
            format (str): describes how the array is specified.
                Valid values include:

                - '': turn off texture mapping
                - 'rgb8': unsigned byte RGB colors with red in the 1st
                    column, green in the 2nd, blue in the 3rd.
                - 'bgr8': unsigned byte RGB colors with blue in the 1st
                    column, green in the 2nd, green in the 3rd
                - 'rgba8': unsigned byte RGBA colors with red in the 1st
                    column and alpha in the 4th
                - 'bgra8': unsigned byte RGBA colors with blue in the 1st
                    column and alpha in the 4th
                - 'l8': unsigned byte grayscale colors, one channel

            array (np.ndarray): a 1D or 2D array, of size w or w x c
                where w is the width and c is the number of channels.
                
                Datatype is of type uint8, or for rgba8 / bgra8, can
                also be packed into uint32 elements.  In this case, the pixel
                format is 0xaarrggbb or 0xaabbggrr, respectively.
        """
        import numpy
        array = numpy.asarray(array)
        if len(array.shape) == 1:
            if array.dtype == numpy.uint8:
                return self.setTexture1D_b(format,array)
            else:
                return self.setTexture1D_i(format,array)
        elif len(array.shape) == 2:
            return self.setTexture1D_channels(format,array)
        else:
            raise ValueError("Can only pass a 1D or 2D array to setTexture1D")

    def setTexture2D(self,format,array):
        """Sets a 2D texture.

        Args:
            format (str): describes how the array is specified.
                Valid values include:

                - '': turn off texture mapping
                - 'rgb8': unsigned byte RGB colors with red in the 1st
                    column, green in the 2nd, blue in the 3rd.
                - 'bgr8': unsigned byte RGB colors with blue in the 1st
                    column, green in the 2nd, green in the 3rd
                - 'rgba8': unsigned byte RGBA colors with red in the 1st
                    column and alpha in the 4th
                - 'bgra8': unsigned byte RGBA colors with blue in the 1st
                    column and alpha in the 4th
                - 'l8': unsigned byte grayscale colors, one channel
                
            array (np.ndarray): a 2D or 3D array, of size h x w or h x w x c
                where h is the height, w is the width, and c is the number of
                channels.
                
                Datatype is of type uint8, or for rgba8 / bgra8, can
                also be packed into uint32 elements.  In this case, the pixel
                format is 0xaarrggbb or 0xaabbggrr, respectively.
        """
        
        import numpy
        array = numpy.asarray(array)
        if len(array.shape) == 2:
            if array.dtype == numpy.uint8:
                return self.setTexture2D_b(format,array)
            else:
                return self.setTexture2D_i(format,array)
        elif len(array.shape) == 3:
            return self.setTexture2D_channels(format,array)
        else:
            raise ValueError("Can only pass a 2D or 3D array to setTexture2D")

    def setTexcoords(self,array):
        """Sets texture coordinates for the mesh.

        Args:
            array (np.ndarray): a 1D or 2D array, of size N or Nx2, where N is
                the number of vertices in the mesh.
        """
        import numpy
        array = numpy.asarray(array)
        if len(array.shape) == 1:
            return self.setTexcoords1D(array)
        elif len(array.shape) == 2:
            return self.setTexcoords2D(array)
        else:
            raise ValueError("Must provide either a 1D or 2D array")
}
}

%extend TriangleMesh {
%pythoncode {
    vertices = property(getVertices, setVertices)
    """The vertices of the mesh."""

    indices = property(getIndices, setIndices)
    """The triangles of the mesh, given as indices into the vertices array."""

    def triangle(self, i) -> Tuple[Tuple[float,float,float],Tuple[float,float,float],Tuple[float,float,float]]:
        """Returns the i'th triangle of the mesh as a tuple of 3 3-tuples."""
        a,b,c = self.indices[i]
        v = self.vertices
        return (v[a],v[b],v[c])

    def __reduce__(self):
        from klampt.io import loader
        jsonobj = loader.to_json(self,'TriangleMesh')
        return (loader.from_json,(jsonobj,'TriangleMesh'))
}
}

%extend PointCloud {
%pythoncode {
    points = property(getPoints, setPoints)
    """The points of the point cloud."""

    properties = property(getProperties, setProperties)
    """The properties of the point cloud."""

    def getPropertyNames(self) -> List[str]:
        """Returns the names of the properties."""
        return [self.getPropertyName(i) for i in range(self.numProperties())]

    def __reduce__(self):
        from klampt.io import loader
        jsonobj = loader.to_json(self,'PointCloud')
        return (loader.from_json,(jsonobj,'PointCloud'))

    def setDepthImage(self,intrinsics,depth,depth_scale=1.0):
        """
        Sets a structured point cloud from a depth image.

        Args:
            intrinsics (4-list): the intrinsics parameters [fx,fy,cx,cy].
            depth (np.ndarray): the depth values, of shape (h,w).  Should have
                dtype float, np.float32, or np.uint16 for best performance.
            depth_scale (float, optional): converts depth image values to real
                depth units.
        """
        import numpy as np
        if len(intrinsics) != 4:
            raise ValueError("Invalid value for the intrinsics parameters")
        if depth.dtype == float:
            return self.setDepthImage_d(intrinsics,depth,depth_scale)
        elif depth.dtype == np.float32:
            return self.setDepthImage_f(intrinsics,depth,depth_scale)
        elif depth.dtype == np.uint16:
            return self.setDepthImage_s(intrinsics,depth,depth_scale)
        else:
            return self.setDepthImage_d(intrinsics,depth,depth_scale)

    def setRGBDImages(self,intrinsics,color,depth,depth_scale=1.0):
        """
        Sets a structured point cloud from a color,depth image pair.

        Args:
            intrinsics (4-list): the intrinsics parameters [fx,fy,cx,cy].
            color (np.ndarray): the color values, of shape (h,w) or (h,w,3).
                In first case, must have dtype np.uint32 with r,g,b values
                packed in 0xrrggbb order.  In second case, if dtype is
                np.uint8, min and max are [0,255].  If dtype is float or
                np.float32, min and max are [0,1].
            depth (np.ndarray): the depth values, of shape (h,w).  Should have
                dtype float, np.float32, or np.uint16 for best performance.
            depth_scale (float, optional): converts depth image values to real
                depth units.
        """
        import numpy as np
        if len(intrinsics) != 4:
            raise ValueError("Invalid value for the intrinsics parameters")
        if color.shape[0] != depth.shape[0] or color.shape[1] != depth.shape[1]:
            raise ValueError("Color and depth images need to have matching dimensions")
        if len(color.shape)==3:
            if color.shape[2] != 3:
                raise ValueError("Color image can only have 3 channels")
            if color.dtype != np.uint8:
                color = (color*255.0).astype(np.uint8)
            if depth.dtype == float:
                return self.setRGBDImages_b_d(intrinsics,color,depth,depth_scale)
            elif depth.dtype == np.float32:
                return self.setRGBDImages_b_f(intrinsics,color,depth,depth_scale)
            elif depth.dtype == np.uint16:
                return self.setRGBDImages_b_s(intrinsics,color,depth,depth_scale)
            else:
                return self.setRGBDImages_b_d(intrinsics,color,depth,depth_scale)
        else:
            if depth.dtype == float:
                return self.setRGBDImages_i_d(intrinsics,color,depth,depth_scale)
            elif depth.dtype == np.float32:
                return self.setRGBDImages_i_f(intrinsics,color,depth,depth_scale)
            elif depth.dtype == np.uint16:
                return self.setRGBDImages_i_s(intrinsics,color,depth,depth_scale)
            else:
                return self.setRGBDImages_i_d(intrinsics,color,depth,depth_scale)

}
}

%extend ConvexHull {
%pythoncode {
    points = property(getPoints, setPoints)
    """The points of the convex hull."""

    def __reduce__(self):
        from klampt.io import loader
        jsonobj = loader.to_json(self,'ConvexHull')
        return (loader.from_json,(jsonobj,'ConvexHull'))
}
}

%extend GeometricPrimitive {
%pythoncode {
    type = property(getType)
    """The type of the geometric primitive."""

    properties = property(getProperties, setProperties)
    """The properties of the geometric primitive.  Type dependent."""

    def __reduce__(self):
        from klampt.io import loader
        jsonobj = loader.to_json(self,'GeometricPrimitive')
        return (loader.from_json,(jsonobj,'GeometricPrimitive'))
}
}

%extend VolumeGrid { 
%pythoncode {
    bmin = property(getBmin, setBmin)
    """The lower bound of the domain."""

    bmax = property(getBmax, setBmax)
    """The upper bound of the domain."""

    def setBounds(self, bounds):
        """@deprecated
        
        Provided for backwards compatibility
        """
        import warnings
        warnings.warn("VolumeGrid. setBounds will be deprecated in favor of bmin, bmax attributes in a future version of Klampt",DeprecationWarning)
        self.bmin = bounds[0:3]
        self.bmax = bounds[3:6]
    
    def getBounds(self):
        """@deprecated
        
        Provided for backwards compatibility
        """
        import warnings
        warnings.warn("VolumeGrid. getBounds will be deprecated in favor of bmin, bmax attributes in a future version of Klampt",DeprecationWarning)
        return list(self.bmin) + list(self.bmax)
    
    bounds = property(getBounds, setBounds)
    """Klampt 0.9 backwards compatibility accessor for the (bmin, bmax) pair."""

    values = property(getValues, setValues)
    """The 3D array of values in the grid (numpy.ndarray)"""

    def __reduce__(self):
        from klampt.io import loader
        jsonobj = loader.to_json(self,'VolumeGrid')
        return (loader.from_json,(jsonobj,'VolumeGrid'))

}
}

%extend Heightmap {
%pythoncode {
    def __reduce__(self):
        from klampt.io import loader
        jsonobj = loader.to_json(self,'Heightmap')
        return (loader.from_json,(jsonobj,'Heightmap'))

    def setHeightImage(self, img, height_scale : float = 1.0):
        """
        Sets heights from a height image.

        Args:
            img (np.ndarray): the height values, of shape (h,w).  Should have
                dtype float, np.float32, or np.uint16 for best performance.
            height_scale (float, optional): converts depth image values to real
                depth units.
        """
        import numpy as np
        if len(img.shape) != 2:
            raise ValueError("Invalid shape for the height image")
        if img.dtype == float:
            return self.setHeightImage_d(img,height_scale)
        elif img.dtype == np.float32:
            return self.setHeightImage_f(img,height_scale)
        elif img.dtype == np.uint16:
            return self.setHeightImage_s(img,height_scale)
        elif img.dtype == np.uint8:
            return self.setHeightImage_b(img,height_scale)
        else:
            raise ValueError("Invalid dtype for the height image, can use float, np.float32, np.uint16, or np.uint8")

    def setColorImage(self, img):
        """
        Sets colors from a color image.

        Args:
            img (np.ndarray): the color values, of shape (h,w) or (h,w,3) or (h,w,4).
                Should have dtype float, np.float32, np.uint32 (RGBA 32-bit) or np.uint8.
        """
        import numpy as np
        if len(img.shape) != 2 and len(img.shape) != 3:
            raise ValueError("Invalid shape for the color image")
        if len(img.shape) == 2:
            if img.dtype == np.uint32:
                return self.setColorImage_i(img)
            elif img.dtype == np.uint8:
                return self.setColorImage_b(img)
            elif img.dtype == float:
                return self.setColors(img.reshape(img.shape[0],img.shape[1],1))
            else:
                raise ValueError("Invalid dtype for the color image, can use np.uint32, np.uint8, or float")
        else:
            if img.shape[2] != 3 and img.shape[2] != 4:
                raise ValueError("Invalid shape for the color image")
            if img.dtype == np.uint8:
                return self.setColorImage_b(img)
            elif img.dtype == float:
                return self.setColors(img)
            else:
                raise ValueError("Invalid dtype for the height image, can use float or np.uint8")
}
}


%extend Geometry3D {
%pythoncode {        
    def __reduce__(self):
        from klampt.io import loader
        jsonobj = loader.to_json(self,'Geometry3D')
        return (loader.from_json,(jsonobj,'Geometry3D'))
}
}

%extend Viewport { 
%pythoncode {
    def setClippingPlanes(self, cp):
        """Provided for backwards compatibility"""
        import warnings
        warnings.warn("Viewport. clippingPlanes will be deprecated in favor of n,f attributes in a future version of Klampt",DeprecationWarning)
        self.n, self.f = cp
    
    def getClippingPlanes(self):
        """Provided for backwards compatibility"""
        import warnings
        warnings.warn("Viewport. clippingPlanes will be deprecated in favor of n,f attributes in a future version of Klampt",DeprecationWarning)
        return (self.n, self.f)

    fov = property(getFOV, setFOV)
    """Convenience accessor for the field of view, in radians."""

    clippingPlanes = property(getClippingPlanes, setClippingPlanes)
    """Klampt 0.9 backwards compatibility accessor for the (n, f) pair."""
}
}


%include "geometry.h"
%include "appearance.h"
%include "widget.h"
%include "viewport.h"
%include "robotmodel.h"
%include "robotik.h"
%include "robotsim.h"
%include "robotio.h"
%include "stability.h"


//PEP8 deprecations
%pythoncode {
    import warnings

    def _deprecated_func(oldName,newName):
        import sys
        mod = sys.modules[__name__]
        f = getattr(mod,newName)
        def depf(*args,**kwargs):
            warnings.warn("{} will be deprecated in favor of {} in a future version of Klampt".format(oldName,newName),DeprecationWarning)
            return f(*args,**kwargs)
        depf.__doc__ = 'Deprecated in a future version of Klampt. Use {} instead'.format(newName)
        setattr(mod,oldName,depf)

}
// add deprecations above, e.g.,
// _deprecated_func('SubscribeToStream','subscribe_to_stream')
