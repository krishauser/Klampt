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
		destroy();
		return NULL;
	}
	catch(std::exception& e) {
		PyErr_SetString(PyExc_RuntimeError, const_cast<char*>(e.what()));
		destroy();
		return NULL;
	}
}

%include "motionplanning.h"