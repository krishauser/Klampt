/*********************************
*
* pyvectorfield.cpp
* 
* A C++ wrapper for Python code implementing a vector-field system (presently 
* the Python proxy object VectorFieldFunction, defined in vectorfield.py).
*
* Written by Mark Wilson (mw54@cs.indiana.edu)
* Last updated 6/9/2010
*
* Some notes on implementation:
* -The vast majority of this code is error handling.  Most functions that call
*	into the Python API have to sequentially initialize or retrieve a lot of
*	Python resources.  I've opted to employ a system in which the new resource
*	is checked for failure, an exception is set, and we jump to a block that
*	tears down all initialized resources, then throws the previously-set
*	exception:
*		PyObject* pyObj = Py_SomeFunction();
*		if(pyObj == NULL) {
*			exception = SomeException;
*			goto fail;
*		}
*		...
*		tear down all Python resources
*		return success;
*		fail:
*			conditional teardown of Python resources
*			throw exception;
*
*	This uses the dreaded "goto" but avoids the dreaded "septuply-nested if",
*	as well as putting all my error-recovery teardown code in one place,
*	which I consider to be a better-than-fair trade in this instance.
*
* -Exceptions are used because it is necessary to signal to SWIG that an
*	anomalous condition occurred or it will eat the Python exception and
*	continue on its merry way.  This can be done via return values, but
*	if an anomaly occurs there is no way to signal that to the Newton
*	solver, so throwing an exception bypasses it and goes back up to the Python
*	system.
*
* -A frequent error-handling system looks roughly like:
*		if(Py_Function_Failed) {
*			if(!PyErr_Occurred()) {
*				except = PyException("Some error");
*			}
*			goto fail;
*		}
*	If PyErr_Occurred() returns true, the failure block will create a
*	PyPyErrorException (a C++ wrapper around a Python exception condition) and
*	throw that instead, so all we need to do is be ready with a backup 
*	exception in the case that no Python error occurred.
*
*
*
**********************************/

#include "pyvectorfield.h"
#include "pyerr.h"
#include "pyconvert.h"
#include <KrisLibrary/math/matrix.h>
#include <Python.h>

#include <iostream>
using std::cout;
using std::endl;

#if PY_VERSION_HEX < 0x02050000
typedef int Py_ssize_t;
#endif
#if PY_MAJOR_VERSION >= 3
#define PyInt_Check PyLong_Check
#endif //PY_MAJOR_VERSION >= 3

namespace PyPlanner {


PyObject* PyTupleFromVector(const Vector& x)
{
  PyObject* ls = PyTuple_New(x.n);
  PyObject* pItem;
  if(ls == NULL) {
    goto fail;
  }
	
  for(int i = 0; i < x.n; i++) {
    pItem = PyFloat_FromDouble(x[i]);
    if(pItem == NULL)
      goto fail;
    PyTuple_SetItem(ls, (Py_ssize_t)i, pItem);
  }
  
  return ls;
  
 fail:
  Py_XDECREF(ls);
  return NULL;
}



	

PyVectorFieldFunction::PyVectorFieldFunction(PyObject* _pVF) {
	pVFObj = NULL;
	pXTemp = NULL;
	PyObject* pModule = NULL;
	PyObject* pVFClass = NULL;
	PyObject* pMethodName = NULL;
	PyObject* pResult = NULL;
	PyException except("Unknown error in "\
					"PyVectorFieldFunction::PyVectorFieldFunction");
	PyPyErrorException pyExcept;
	
	// Load up the module containing the pure-Python proxy class to 
	// VectorFieldFunction
	/*
	pModule = PyImport_ImportModule(PY_PROXY_MODULE);
	if(pModule == NULL) {
		if(!PyErr_Occurred()) {
			except = PyException("PyVectorFieldFunction: Couldn't import " \
						"VectorFieldFunction proxy module.", Import);
		}
		goto fail;
	}
	
	// Get the class itself
	pVFClass = PyObject_GetAttrString(pModule, PY_PROXY_CLASS);
	if(pVFClass == NULL) {
		if(!PyErr_Occurred()) {
			except = PyException("PyVectorFieldFunction: Couldn't access " \
						"VectorFieldFunction proxy class.", Attribute);
		}
		goto fail;
	}
	
	// Make sure the PyObject we were handed is an instance of the proxy class
	if(!PyObject_IsInstance(_pVF, pVFClass)) {
		except = PyException("PyVectorFieldFunction: Vector-field function " \
					"object must be of type VectorFieldFunction or subclass.",
					Type);
		goto fail;
	}
	*/
	
	// Own new reference to the object
	Py_INCREF(_pVF);
	pVFObj = _pVF;
	// Give away the module and class
	//Py_DECREF(pVFClass);
	//Py_DECREF(pModule);
	//pVFClass = pModule = NULL;
	
	// Get the method name to retrieve the number of variables defining the
	// field
	pMethodName = PyString_FromString(PY_VAR_FN);
	if(pMethodName == NULL) {
		if(!PyErr_Occurred()) {
			except = PyException(
						"PyVectorFieldFunction::PyVectorFieldFunction: "\
						"Couldn't retrieve variable-count method name.");
		}
		goto fail;
	}
	// Call the method
	pResult = PyObject_CallMethodObjArgs(pVFObj, pMethodName, NULL);
	// Give away the method name
	Py_DECREF(pMethodName);
	pMethodName = NULL;
	// Get the number of variables
	if(pResult == NULL) {
		if(!PyErr_Occurred()) {
			except = PyException(
						"PyVectorFieldFunction::PyVectorFieldFunction: "\
						"Couldn't query vector-field function for number "\
						"of variables.");
		}
		goto fail;
	}
	if(!PyInt_Check(pResult)) {
		except = PyException("PyVectorFieldFunction::PyVectorFieldFunction: "\
					"VectorFieldFunction.numVars() must return an integer.",
					Type);
		goto fail;
	}
	n = (int)PyInt_AsLong(pResult);
	
	// TODO: bounds checking on the number of variables
	
	// Do the same but for the number of functions in the field
	pMethodName = PyString_FromString(PY_FUN_FN);
	if(pMethodName == NULL) {
		if(!PyErr_Occurred()) {
			except = PyException(
						"PyVectorFieldFunction::PyVectorFieldFunction: "\
						"Couldn't retrieve function-count method name.");
		}
		goto fail;
	}
	pResult = PyObject_CallMethodObjArgs(pVFObj, pMethodName, NULL);
	Py_DECREF(pMethodName);
	pMethodName = NULL;
	if(pResult == NULL) {
		if(!PyErr_Occurred()) {
			except = PyException(
						"PyVectorFieldFunction::PyVectorFieldFunction: "\
						"Couldn't query vector-field function for number "\
						"of functions.");
		}
		goto fail;
	}
	if(!PyInt_Check(pResult)) {
		except = PyException("PyVectorFieldFunction::PyVectorFieldFunction: "\
					"VectorFieldFunction.numFns() must return an integer.",
					Type);
		goto fail;
	}
	m = (int)PyInt_AsLong(pResult);
	
	// TODO: bounds checking on the number of functions
	
	return;
	
	fail:
		Py_XDECREF(pVFObj);
		// Make sure the object is null so we know we weren't initialized
		// correctly
		pVFObj = NULL;
		//Py_XDECREF(pModule);
		//Py_XDECREF(pVFClass);
		Py_XDECREF(pResult);
		Py_XDECREF(pMethodName);
		if(PyErr_Occurred()) {
			pyExcept = PyPyErrorException();
			throw pyExcept;
		}
		throw except;
}

PyVectorFieldFunction::~PyVectorFieldFunction() {
	if(is_init()) {
		Py_DECREF(pVFObj);
		Py_XDECREF(pXTemp);
	}
}

void PyVectorFieldFunction::PreEval(const Vector& x)
{
  if(x.n != n) {
    throw PyException("PyVectorFieldFunction::PreEval: Number of "	\
		      "variables in arg must match number of variables " \
		      "in vector field.");
  }

  Py_XDECREF(pXTemp);
  pXTemp = PyTupleFromVector(x);	
  if(pXTemp == NULL) {
    if(!PyErr_Occurred()) {
      throw PyException("PyVectorFieldFunction::PreEval: Couldn't "	\
			"build variable-value tuple.");
    }
  }
}

void PyVectorFieldFunction::Eval(const Vector& x, Vector& v)
{
  if(!is_init()) {
    throw PyException("PyVectorFieldFunction::Eval: object is "	\
		      "uninitialized [did you remember to call setVectorField() " \
		      "before findRoots()?]");
  }
  if(!pXTemp) {
    throw PyException("PyVectorFieldFunction::Eval: object is "	\
		      "uninitialized [did you remember to call PreEval()?]");
  }

  v.resize(m);

  //If eval_i is the only method available
  if(!PyObject_HasAttrString(pVFObj,PY_EVAL_FN)) {
    if(!PyObject_HasAttrString(pVFObj,PY_EVAL_I_FN)) {
      throw PyException("PyVectorFieldFunction::Eval: object is does not contain eval() or eval_i() methods");
    }

    for(int i = 0; i < m; i++) {
      v[i] = Eval_i(x, i);
    }
    return;
  }

  //Has the eval() method
  // Get the method name for the eval_i proxy
  PyObject* pMethodName = PyString_FromString(PY_EVAL_FN);
  PyObject* pResult = NULL;
  PyException except("PyVectorFieldFunction::Eval: Unknown error.");
  PyPyErrorException pyExcept;

  if(pMethodName == NULL) {	
    if(!PyErr_Occurred()) {
      except = PyException("PyVectorFieldFunction::Eval: Couldn't "	\
			   "retrieve eval method name.");
    }
    goto fail;
  }
	
  // Call the proxy method
  pResult = PyObject_CallMethodObjArgs(pVFObj, pMethodName, pXTemp, NULL);
  // Give away the method name, the variable tuple, and the index int
  Py_DECREF(pMethodName);
  pMethodName = NULL;
  if(pResult == NULL) {
    if(!PyErr_Occurred()) {
      except = PyException("PyVectorFieldFunction::Eval: Unsuccessful " \
			   "call to Python VectorFieldFunction.eval().");
    }
    goto fail;
  }  // Result should be either a float, int, or sequence
  if(!PyFloat_Check(pResult) && !PyInt_Check(pResult) && !PySequence_Check(pResult)) {
    except = PyException("PyVectorFieldFunction::Eval: "		\
			 "VectorFieldFunction.eval() must return an int,  float, or sequence.",
			 Type);
    goto fail;
  }
	
  // Get result
  if(PyFloat_Check(pResult) || PyInt_Check(pResult)) {
    if(v.n != 1) {
      except = PyException("PyVectorFieldFunction::Eval: "		\
			   "VectorFieldFunction.eval() returned a float, but need a sequence.",
			   Type);
      goto fail;
    }
    v(0) = PyFloat_AsDouble(pResult);
    assert(!PyErr_Occurred());
  }
  else {
    if(PySequence_Size(pResult) != (Py_ssize_t)v.n) {
      except = PyException("PyVectorFieldFunction::Eval: "		\
			   "VectorFieldFunction.eval() returned a list of incorrect size.",
			   Type);
      goto fail;
    }
    if(!PyListToConfig(pResult,v)) {
      except = PyException("PyVectorFieldFunction::Eval: "		\
			   "VectorFieldFunction.eval() could not convert result to a vector.",
			   Type);
      goto fail;
    }
  }
	
  // Give away the Python result object
  Py_DECREF(pResult);
  pResult = NULL;
  
  //All done!
  return;

 fail:
  Py_XDECREF(pMethodName);
  Py_XDECREF(pResult);
  if(PyErr_Occurred()) {
    pyExcept = PyPyErrorException();
    throw pyExcept;
  }
  throw except;
}

Real PyVectorFieldFunction::Eval_i(const Vector& x, int i) {
  if(!is_init()) {
    throw PyException("PyVectorFieldFunction::Eval: object is "	\
		      "uninitialized [did you remember to call setVectorField() " \
		      "before findRoots()?]");
  }

  if(!pXTemp) {
    throw PyException("PyVectorFieldFunction::Eval_i: object is "	\
		      "uninitialized [did you remember to call PreEval()?]");
  }


  // Tuple to hold variable values we want to test
  // Get the method name for the eval_i proxy
  PyObject* pMethodName = PyString_FromString(PY_EVAL_I_FN);
  PyObject* pIndex = NULL;
  PyObject* pResult = NULL;
  PyException except("PyVectorFieldFunction::Eval_i: Unknown error.");
  PyPyErrorException pyExcept;
  Real result;
    	
	if(pMethodName == NULL) {	
		if(!PyErr_Occurred()) {
			except = PyException("PyVectorFieldFunction::Eval_i: Couldn't "\
						"retrieve eval_i method name.");
		}
		goto fail;
	}
	
	pIndex = PyInt_FromLong(i);
	if(pIndex == NULL) {
		if(!PyErr_Occurred()) {
			except = PyException("PyVectorFieldFunction::Eval_i: Couldn't "\
						"build index.");
		}
		goto fail;
	}
	
	// Call the proxy method
	pResult = PyObject_CallMethodObjArgs(pVFObj, pMethodName, pXTemp, pIndex, 
				NULL);
	// Give away the method name, the variable tuple, and the index int
	Py_DECREF(pMethodName);
	Py_DECREF(pIndex);
	pMethodName = pIndex = NULL;
	if(pResult == NULL) {
		if(!PyErr_Occurred()) {
			except = PyException("PyVectorFieldFunction::Eval_i: Unsuccessful "\
						"call to Python VectorFieldFunction.eval_i().");
		}
		goto fail;
	}
	// Result should be either a float or an int
	if(!PyFloat_Check(pResult) && !PyInt_Check(pResult)) {
		except = PyException("PyVectorFieldFunction::Eval_i: "\
					"VectorFieldFunction.eval_i() must return an int or float.",
					Type);
		goto fail;
	}
	
	// Get result
	if(PyFloat_Check(pResult)) {
		result = PyFloat_AsDouble(pResult);
	}
	else {
		result = PyInt_AsLong(pResult);
	}
	// Give away the Python result object
	Py_DECREF(pResult);
	pResult = NULL;
	
	return result;
	
	fail:
		Py_XDECREF(pMethodName);
		Py_XDECREF(pIndex);
		Py_XDECREF(pResult);
		if(PyErr_Occurred()) {
			pyExcept = PyPyErrorException();
			throw pyExcept;
		}
		throw except;
}

void PyVectorFieldFunction::Jacobian(const Vector& x, Matrix& J)
{
  if(!is_init()) {
    throw PyException("PyVectorFieldFunction::Jacobian: object is "	\
		      "uninitialized [did you remember to call setVectorField() " \
		      "before findRoots()?]");
  }

  if(!pXTemp) {
    throw PyException("PyVectorFieldFunction::Jacobian: object is "	\
		      "uninitialized [did you remember to call PreEval()?]");
  }


	PyObject* elem, *val;
	int i,j;
  J.resize(m,n);

  //If eval_i is the only method available
  if(!PyObject_HasAttrString(pVFObj,PY_JBN_FN)) {
    if(!PyObject_HasAttrString(pVFObj,PY_JBN_IJ_FN)) {
      throw PyException("PyVectorFieldFunction::Jacobian: object is does not contain jacobian() or jacobian_ij() methods");
    }

    for(i = 0; i < m; i++) {
      for(j = 0; j < n; j++) {
	J(i,j) = Jacobian_ij(x, i, j);
      }
    }
    return;
  }

  //Has the jacobian() method
  // Tuple to hold variable values we want to test
  // Get the method name for the jacobian proxy
  PyObject* pMethodName = PyString_FromString(PY_JBN_FN);
  PyObject* pResult = NULL;
  PyException except("PyVectorFieldFunction::Jacobian: Unknown error.");
  PyPyErrorException pyExcept;

  if(pMethodName == NULL) {	
    if(!PyErr_Occurred()) {
      except = PyException("PyVectorFieldFunction::Jacobian: Couldn't "	\
			   "retrieve jacobian method name.");
    }
    goto fail;
  }
	
  // Call the proxy method
  pResult = PyObject_CallMethodObjArgs(pVFObj, pMethodName, pXTemp, NULL);
  // Give away the method name, the variable tuple, and the index int
  Py_DECREF(pMethodName);
  pMethodName = NULL;
  if(pResult == NULL) {
    if(!PyErr_Occurred()) {
      except = PyException("PyVectorFieldFunction::Jacobian: Unsuccessful " \
			   "call to Python VectorFieldFunction.jacobian().");
    }
    goto fail;
  }
  // Result should be a sequence
  if(!PySequence_Check(pResult)) {
    except = PyException("PyVectorFieldFunction::Jacobian: "		\
			 "VectorFieldFunction.jacobian() must return a sequence.",
			 Type);
    goto fail;
  }
	
  if(PySequence_Size(pResult) != (Py_ssize_t)m) {
    printf("Sequence size: %d != %d\n",(int)PySequence_Size(pResult),m);
    except = PyException("PyVectorFieldFunction::Jacobian: "		\
			 "VectorFieldFunction.jacobian() returned a list of incorrect size.",
			 Type);
    goto fail;
  }
  for(i=0;i<m;i++) {
    elem = PySequence_GetItem(pResult, Py_ssize_t(i));    
    if(!PySequence_Check(elem) || PySequence_Size(elem) != (Py_ssize_t)n) {
	except = PyException("PyVectorFieldFunction::Jacobian: "		\
			     "VectorFieldFunction.jacobian() is not a list of lists of the right size.",
			     Type);
      Py_XDECREF(elem);
	goto fail;
    }
    for(j=0;j<n;j++) {
    	val = PySequence_GetItem(elem,(Py_ssize_t)j);
	    J(i,j) = PyFloat_AsDouble(val);
	    Py_XDECREF(val);
	    if(PyErr_Occurred()) {
	except = PyException("PyVectorFieldFunction::Jacobian: "		\
			     "VectorFieldFunction.jacobian() element couldn't be cast to double",
			     Type);
      Py_XDECREF(elem);
	goto fail;
      }
    }
    Py_XDECREF(elem);
  }
	
  // Give away the Python result object
  Py_DECREF(pResult);
  pResult = NULL;
  
  //All done!
  return;
  
 fail:
  Py_XDECREF(pMethodName);
  Py_XDECREF(pResult);
  if(PyErr_Occurred()) {
    pyExcept = PyPyErrorException();
    throw pyExcept;
  }
  throw except;
}

Real PyVectorFieldFunction::Jacobian_ij(const Vector& x, int i, int j) {
  if(!is_init()) {
    throw PyException("PyVectorFieldFunction::Jacobian_ij: object is "	\
		      "uninitialized [did you remember to call setVectorField() " \
		      "before findRoots()?]");
  }

  if(!pXTemp) {
    throw PyException("PyVectorFieldFunction::Jacobian_ij: object is "	\
		      "uninitialized [did you remember to call PreEval()?]");
  }

  // Method name for the Jacobian_ij proxy
  PyObject* pMethodName = PyString_FromString(PY_JBN_IJ_FN);
  PyObject* pI = NULL;
  PyObject* pJ = NULL;
  PyObject* pResult = NULL;
  PyException except("PyVectorFieldFunction::Jacobian_ij: Unknown error.");
  PyPyErrorException pyExcept;
  Real result;
	
	if(pMethodName == NULL) {	
		if(!PyErr_Occurred()) {
			except = PyException("PyVectorFieldFunction::Jacobian_ij: "\
						"Couldn't retrieve eval_i method name.");
		}
		goto fail;
	}
		
	// Index values to Python
	pI = PyInt_FromLong(i);
	pJ = PyInt_FromLong(j);
	if(pI == NULL || pJ == NULL) {
		if(!PyErr_Occurred()) {
			except = PyException("PyVectorFieldFunction::Jacobian_ij: "\
						"Couldn't build index.");
		}
		goto fail;
	}
	
	// Call the proxy method
	pResult = PyObject_CallMethodObjArgs(pVFObj, pMethodName, pXTemp, pI, pJ,
				NULL);
	// Give away the method name, variable tuple, and index ints
	Py_DECREF(pMethodName);
	Py_DECREF(pI);
	Py_DECREF(pJ);
	pMethodName = pI = pJ = NULL;
	if(pResult == NULL) {
		if(!PyErr_Occurred()) {
			except = PyException("PyVectorFieldFunction::Jacobian_ij: "\
						"Unsuccessful call to Python "\
						"VectorFieldFunction.jacobian_ij().");
		}
		goto fail;
	}
	// Result should be either a float or an int
	result = PyFloat_AsDouble(pResult);
	if(PyErr_Occurred()) {
		except = PyException("PyVectorFieldFunction::Jacobian_ij: "\
					"VectorFieldFunction.jacobian_ij() must return an "\
					"int or float.", Type);
		goto fail;
	}
	
	// Give away the Python result value
	Py_DECREF(pResult);
	pResult = NULL;
	
	return result;
	
	fail:
		Py_XDECREF(pMethodName);
		Py_XDECREF(pI);
		Py_XDECREF(pJ);
		Py_XDECREF(pResult);
		if(PyErr_Occurred()) {
			pyExcept = PyPyErrorException();
			throw pyExcept;
		}
		throw except;
}

}
