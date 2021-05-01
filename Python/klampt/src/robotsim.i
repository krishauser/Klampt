%module(docstring="Klamp't Core Python bindings \n------------------------------") robotsim
%{
  #define SWIG_FILE_WITH_INIT
	#include <exception>
	#include <vector>
	#include "pyerr.h"
	#include "geometry.h"
	#include "appearance.h"
	#include "widget.h"
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
%array_class(int, intArray);

namespace std {
   %template(stringVector) vector<string>;
   %template(doubleVector) vector<double>;
   %template(floatVector) vector<float>;
   %template(intVector) vector<int>;
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


%}

%typemap(in) const int [ANY](int temp[$1_dim0]) {
   if (!convert_iarray($input,temp,$1_dim0)) {
      return NULL;
   }
   $1 = &temp[0];
}

%apply (unsigned char* IN_ARRAY1, int DIM1) {(unsigned char* np_array,int m)};
%apply (unsigned char* IN_ARRAY2,int DIM1,int DIM2) {(unsigned char* np_array2, int m, int n)};
%apply (float ARGOUT_ARRAY1[ANY]) {(float out[ANY])};
%apply (float ARGOUT_ARRAY1[ANY]) {(float out2[ANY])};
%apply (float IN_ARRAY1[ANY]) {const float [ANY]};
%apply (float* IN_ARRAY1, int DIM1) {(float* np_array,int m)};
%apply (float IN_ARRAY2[ANY][ANY]) {const float [ANY][ANY]};
%apply (float* IN_ARRAY2,int DIM1,int DIM2) {(float* np_array2, int m, int n)};
%apply (double ARGOUT_ARRAY1[ANY]) {(double out[ANY])};
%apply (double ARGOUT_ARRAY1[ANY]) {(double out2[ANY])};
%apply (double* ARGOUT_ARRAY1,int DIM1) {(double* out,int m)};
%apply (double* ARGOUT_ARRAY1,int DIM1) {(double* out2,int m)};
%apply (double ARGOUT_ARRAY2[ANY][ANY]) {(double out[ANY][ANY])};
%apply (double IN_ARRAY1[ANY]) {const double [ANY]};
%apply (double IN_ARRAY2[ANY][ANY]) {const double [ANY][ANY]};
%apply (double* IN_ARRAY2,int DIM1,int DIM2) {(double* np_array2, int m, int n)};
%apply (double** ARGOUTVIEW_ARRAY2,int* DIM1,int* DIM2) {(double** np_view2,int* m, int *n)};
%apply (double** ARGOUTVIEWM_ARRAY2,int* DIM1,int* DIM2) {(double** np_out2,int* m, int *n)};
%apply (double* IN_ARRAY3,int DIM1,int DIM2, int DIM3) {(double* np_array3, int m, int n, int p)};
%apply (double** ARGOUTVIEW_ARRAY3,int* DIM1,int* DIM2,int* DIM3) {(double** np_view3,int* m, int *n, int* p)};
%apply (double** ARGOUTVIEWM_ARRAY3,int* DIM1,int* DIM2,int* DIM3) {(double** np_out3,int* m, int *n, int* p)};

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

%feature("autodoc","1");
%include "docs/docs.i"

%extend Mass { 
%pythoncode {
     com = property(getCom, setCom)
     inertia = property(getInertia, setInertia)
}
}

%extend VolumeGrid { 
%pythoncode {
     values = property(getValues, setValues)
}
}

%extend IKObjective {
%pythoncode {
    def __reduce__(self):
        from klampt.io import loader
        jsonobj = loader.toJson(self,'IKObjective')
        return (loader.fromJson,(jsonobj,'IKObjective'))
}
}

%extend Geometry3D {
%pythoncode {
    def __reduce__(self):
        from klampt.io import loader
        jsonobj = loader.toJson(self,'Geometry3D')
        return (loader.fromJson,(jsonobj,'Geometry3D'))
}
}


%extend TriangleMesh {
%pythoncode {
    def __reduce__(self):
        from klampt.io import loader
        jsonobj = loader.toJson(self,'TriangleMesh')
        return (loader.fromJson,(jsonobj,'TriangleMesh'))
}
}

%extend PointCloud {
%pythoncode {
    def __reduce__(self):
        from klampt.io import loader
        jsonobj = loader.toJson(self,'PointCloud')
        return (loader.fromJson,(jsonobj,'PointCloud'))
}
}

%extend VolumeGrid {
%pythoncode {
    def __reduce__(self):
        from klampt.io import loader
        jsonobj = loader.toJson(self,'VolumeGrid')
        return (loader.fromJson,(jsonobj,'VolumeGrid'))
}
}

%extend ConvexHull {
%pythoncode {
    def __reduce__(self):
        from klampt.io import loader
        jsonobj = loader.toJson(self,'ConvexHull')
        return (loader.fromJson,(jsonobj,'ConvexHull'))
}
}

%extend GeometricPrimitive {
%pythoncode {
    def __reduce__(self):
        from klampt.io import loader
        jsonobj = loader.toJson(self,'GeometricPrimitive')
        return (loader.fromJson,(jsonobj,'GeometricPrimitive'))
}
}

%include "geometry.h"
%include "appearance.h"
%include "widget.h"
%include "robotmodel.h"
%include "robotik.h"
%include "robotsim.h"
%include "robotio.h"
%include "stability.h"
