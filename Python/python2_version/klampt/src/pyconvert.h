#ifndef PYTHON_KLAMPT_CONVERSIONS_H
#define PYTHON_KLAMPT_CONVERSIONS_H

#include <Python.h>
#if PY_MAJOR_VERSION >= 3
#define IS_PY3K
#define PyInt_FromLong PyLong_FromLong
#define PyInt_AsLong PyLong_AsLong 
#define PyString_FromString PyUnicode_FromString
#endif //PY_MAJOR_VERSION >= 3
#include <string>
#include <vector>
#include <KrisLibrary/math/vector.h>
#include <KrisLibrary/math3d/primitives.h>

inline PyObject* ToPy(int x) { return PyInt_FromLong(x); }
inline PyObject* ToPy(double x) { return PyFloat_FromDouble(x); }
inline PyObject* ToPy(const std::string& x) { return PyString_FromString(x.c_str()); }

inline bool FromPy(PyObject* obj,int& x) { x = PyInt_AsLong(obj); return !PyErr_Occurred(); }
inline bool FromPy(PyObject* obj,double& x) { x = PyFloat_AsDouble(obj); return !PyErr_Occurred(); }
//inline bool FromPy(PyObject* obj,std::string& x) { return PyString_FromString(x.c_str()); }

template <class T>
PyObject* ToPy_VectorLike(const T& x,size_t n) {
  PyObject* ls = PyList_New(n);
  PyObject* pItem;
  if(ls == NULL) {
    goto fail;
  }
  
  for(size_t i = 0; i < n; i++) {
    pItem = ::ToPy(x[i]);
    if(pItem == NULL)
      goto fail;
    PyList_SetItem(ls, i, pItem);
  }
  
  return ls;
  
 fail:
  Py_XDECREF(ls);
  throw PyException("Failure during ToPy");
  return NULL;
}

template <class T>
bool FromPy_VectorLike(PyObject* seq,T& x)
{
  if(!PySequence_Check(seq))
    return false;
  
  x.resize(PySequence_Size(seq));
  for(Py_ssize_t i = 0; i < PySequence_Size(seq); i++) {
    PyObject* v=PySequence_GetItem(seq, i);
    assert(v != NULL);
    bool res = FromPy(v,x[(int)i]);
    Py_XDECREF(v);
    if(!res || PyErr_Occurred()) return false;
  }
  return true;
}

template <class T>
bool FromPy_VectorLike_Fixed(PyObject* seq,int n,T& x)
{
  if(!PySequence_Check(seq))
    return false;
  if(PySequence_Size(seq) != n)
    return false;
  for(Py_ssize_t i = 0; i < PySequence_Size(seq); i++) {
    PyObject* v=PySequence_GetItem(seq, i);
    assert(v != NULL);
    bool res = FromPy(v,x[(int)i]);
    Py_XDECREF(v);
    if(!res || PyErr_Occurred()) return false;
  }
  return true;
}



inline PyObject* ToPy(const Math3D::Vector2& x) 
{
  return ToPy_VectorLike(x,2);
}

inline bool FromPy(PyObject* obj,Math3D::Vector2& x) 
{
  return FromPy_VectorLike_Fixed(obj,2,x);
}

inline PyObject* ToPy(const Math3D::Vector3& x)
{
  return ToPy_VectorLike(x,3);
}

inline bool FromPy(PyObject* obj,Math3D::Vector3& x) 
{
  return FromPy_VectorLike_Fixed(obj,3,x);
}

inline PyObject* ToPy(const Math3D::Vector4& x)
{
  return ToPy_VectorLike(x,4);
}

inline bool FromPy(PyObject* obj,Math3D::Vector4& x) 
{
  return FromPy_VectorLike_Fixed(obj,4,x);
}

template <class T>
PyObject* ToPy(const std::vector<T>& x)
{
  return ToPy_VectorLike(x,x.size());
}

template <class T>
bool FromPy(PyObject* obj,std::vector<T>& x)
{
  return FromPy_VectorLike(obj,x);
}

inline PyObject* ToPy(const Math::Vector& x) 
{
  return ToPy_VectorLike(x,x.n);
}

inline bool FromPy(PyObject* obj,Math::Vector& x) 
{
  return FromPy_VectorLike(obj,x);
}

inline PyObject* PyListFromVector(const std::vector<double>& x)
{
  return ToPy(x);
}

inline bool PyListToVector(PyObject* seq,std::vector<double>& x)
{
  return FromPy_VectorLike(seq,x);
}

inline PyObject* PyListFromConfig(const Math::Vector& x)
{
  return ToPy_VectorLike(x,x.n);
}

inline bool PyListToConfig(PyObject* seq,Math::Vector& x)
{
  return FromPy_VectorLike(seq,x);
}


#endif 
