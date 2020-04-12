#ifndef PYERR_H
#define PYERR_H

#include <exception>
#include <string>

// Forward declaration of C-type PyObject
struct _object;
typedef _object PyObject;

using std::exception;
using std::string;

enum PyExceptionType {
	Type, Import, Attribute, Saved, Other
};

class PyException : public exception {
	private:
		PyExceptionType type;
		string msg;
	public:
		PyException() throw() : exception() {
			msg = "Unknown planner error";
			type = Other;
		}
		PyException(const PyException& e) throw() : exception(e) {
			msg = e.msg;
			type = e.type;
		}
		PyException(const string& _msg, const PyExceptionType& _type=Other) 
				throw() : exception() {
			msg = _msg;
			type = _type;
		}
		~PyException() throw() { }
		
		PyException& operator=(const PyException& e) throw();
		
		virtual const char* what() const throw() {
			return msg.c_str();
		}
		PyExceptionType exceptionType() const {
			return type;
		}
		
		virtual void setPyErr();
};


class PyPyErrorException : public PyException {
	private:
		PyObject* pType;
		PyObject* pVal;
		PyObject* pTrace;
	public:
		PyPyErrorException() throw();
		PyPyErrorException(const string& _msg) throw();
		PyPyErrorException(const PyPyErrorException& e) throw();
		PyPyErrorException& operator=(const PyPyErrorException& e) throw();
		~PyPyErrorException() throw();
		
		virtual void setPyErr();
};



#endif
