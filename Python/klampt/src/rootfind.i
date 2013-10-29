%module(docstring="Python interface to KrisLibrary nonlinear, multidimensional root finding routines") rootfind
%{
	#include <exception>
	#include "pyerr.h"
	#include "rootfind.h"
%}
	
%exception {
	try {
		$action
	}
	catch(PyException& e) {
		e.setPyErr();
		destroy();
		return NULL;
	}
	catch(std::exception& e) {
		PyErr_SetString(PyExc_RuntimeError, const_cast<char*>(e.what()));
		destroy();
		return NULL;
	}
}

void setFTolerance(double tolf);
void setXTolerance(double tolx);
int setVectorField(PyObject* pVFObj);
PyObject* findRoots(PyObject* startVals, int iter);
PyObject* findRootsBounded(PyObject* startVals, PyObject* boundVals, int iter);
void destroy();
