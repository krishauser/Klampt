#ifndef PYVECTORFIELD_H
#define PYVECTORFIELD_H

#include <string>

// Forward declaration of C-type PyObject
struct _object;
typedef _object PyObject;

#include <KrisLibrary/math/function.h>

// Useful constants for interfacing with Python
//#define PY_PROXY_MODULE "vectorfield"
//#define PY_PROXY_CLASS "VectorFieldFunction"
#define PY_VAR_FN "num_vars"
#define PY_FUN_FN "num_fns"
#define PY_EVAL_FN "eval"
#define PY_EVAL_I_FN "eval_i"
#define PY_JBN_FN "jacobian"
#define PY_JBN_IJ_FN "jacobian_ij"

namespace PyPlanner {

using Math::VectorFieldFunction;
using Math::Vector;
using Math::Matrix;
using Math::Real;

using std::string;

class PyVectorFieldFunction : public VectorFieldFunction {
	private:
		PyVectorFieldFunction();
		int n, m;
		PyObject* pVFObj;
		PyObject* pXTemp;
		
	public:
		PyVectorFieldFunction(PyObject* _pVF);
		~PyVectorFieldFunction();
		
		bool is_init() const { return (pVFObj != NULL); }
		
		string Label() const { return "Vector Field Function interface "\
								"to Python"; }
		int NumDimensions() const { return m; }
		int NumVariables() const { return n; }
		void PreEval(const Vector& x);
		void Eval(const Vector& x, Vector& v);
		Real Eval_i(const Vector& x, int i);
		void Jacobian(const Vector& x, Matrix& J);
		Real Jacobian_ij(const Vector& x, int i, int j);
		
};

}


#endif
