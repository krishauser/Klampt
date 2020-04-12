%module(docstring="Python interface to C++ motion planing routines") motionplanning
%{
	#include <exception>
	#include "pyerr.h"
	#include "motionplanning.h"
%}

%include "std_string.i"
%include "std_vector.i"
	
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
