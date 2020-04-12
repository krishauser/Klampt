#ifndef ROOTFIND_H
#define ROOTFIND_H

// Forward declaration of C-type PyObject
struct _object;
typedef _object PyObject;

/// Sets the termination threshold for the change in f
void setFTolerance(double tolf);

/// Sets the termination threshold for the change in x
void setXTolerance(double tolx);

/// Sets the vector field object.
///
/// Returns:
/// 
///     status (int): 0 if pVFObj = NULL, 1 otherwise.
///
/// See vectorfield.py for an abstract base class that can be overridden to
/// produce one of these objects.
int setVectorField(PyObject* pVFObj);

/// Sets the function object.
///
/// Returns:
///
///     status (int): 0 if pVFObj = NULL, 1 otherwise.
///
/// See vectorfield.py for an abstract base class that can be overridden to
/// produce one of these objects.
///
/// Equivalent to setVectorField; just a more intuitive name.
int setFunction(PyObject* pVFObj);

/**
 * Performs unconstrained root finding for up to iter iterations
 * 
 * Returns:
 * 
 *     status,x,n (tuple of int, list of floats, int): where status indicates 
 *         the return code, as follows: 
 *
 *             - 0: convergence reached in x
 *             - 1: convergence reached in f
 *             - 2: divergence
 *             - 3: degeneration of gradient (local extremum or saddle point)
 *             - 4: maximum iterations reached
 *             - 5: numerical error occurred
 *
 *         and x is the final point and n is the number of iterations used
 */
PyObject* findRoots(PyObject* startVals, int iter);

/// Same as findRoots, but with given bounds (xmin,xmax)
PyObject* findRootsBounded(PyObject* startVals, PyObject* boundVals, int iter);

/// destroys internal data structures
void destroy();


#endif
