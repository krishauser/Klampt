%module(docstring="Python interface to KrisLibrary motion planing routines") motionplanning
%{
	#include <exception>
	#include "pyerr.h"
	#include "motionplanning.h"
%}
	
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


%feature("autodoc","1");
%include "docs/docs.i"

%include "motionplanning.h"
