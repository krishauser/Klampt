#include "pyerr.h"

#include <Python.h>


/*****************************************
* PyException
*****************************************/

PyException& PyException::operator=(const PyException& e) throw() {
	exception::operator=(e);
	msg = e.msg;
	type = e.type;
	return *this;
}

void PyException::setPyErr() {
	PyObject* errType;
	switch(type) {
		case Type:
			errType = PyExc_TypeError;
			break;
		case Import:
			errType = PyExc_ImportError;
			break;
		case Attribute:
			errType = PyExc_AttributeError;
			break;
		case Other:
		default:
			errType = PyExc_RuntimeError;
			break;
	}
	PyErr_SetString(errType, const_cast<char*>(what()));
}


/******************************************
* PyPyErrorException
******************************************/


PyPyErrorException::PyPyErrorException() throw() 
		: PyException("Temporarily saved Python exception", Type) {
	PyErr_Fetch(&pType, &pVal, &pTrace);
}

PyPyErrorException::PyPyErrorException(const string& _msg) throw() 
		: PyException(_msg, Type) {
	PyErr_Fetch(&pType, &pVal, &pTrace);
}

PyPyErrorException::PyPyErrorException(const PyPyErrorException& e) throw() 
		: PyException(e) {
	pType = e.pType;
	pVal = e.pVal;
	pTrace = e.pTrace;
	if(pType != NULL) {
		Py_INCREF(pType);
	}
	if(pVal != NULL) {
		Py_INCREF(pVal);
	}
	if(pTrace != NULL) {
		Py_INCREF(pTrace);
	}
}

PyPyErrorException::~PyPyErrorException() throw() {
	Py_XDECREF(pType);
	Py_XDECREF(pVal);
	Py_XDECREF(pTrace);
}

PyPyErrorException& PyPyErrorException::operator=(const PyPyErrorException& e) 
		throw() {
	PyException::operator=(e);
	pType = e.pType;
	pVal = e.pVal;
	pTrace = e.pTrace;
	if(pType != NULL) {
		Py_INCREF(pType);
	}
	if(pVal != NULL) {
		Py_INCREF(pVal);
	}
	if(pTrace != NULL) {
		Py_INCREF(pTrace);
	}
	return *this;
}

void PyPyErrorException::setPyErr() {
	if(pType != NULL) {
		// Restore steals refs, so set them to NULL to avoid double decref
		PyErr_Restore(pType, pVal, pTrace);
		pType = pVal = pTrace = NULL;
	}
}
