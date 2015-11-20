#ifndef ROOTFIND_H
#define ROOTFIND_H

// Forward declaration of C-type PyObject
struct _object;
typedef _object PyObject;

/// Sets the termination threshold for the change in f
void setFTolerance(double tolf);

/// Sets the termination threshold for the change in x
void setXTolerance(double tolx);

/// Sets the vector field object, returns 0 if pVFObj = NULL, 1 otherwise.
/// See vectorfield.py for an abstract base class that can be overridden to
/// produce one of these objects.
int setVectorField(PyObject* pVFObj);

/// Sets the function object, returns 0 if pVFObj = NULL, 1 otherwise.
/// See vectorfield.py for an abstract base class that can be overridden to
/// produce one of these objects.
/// Equivalent to setVectorField; just a more intuitive name.
int setFunction(PyObject* pVFObj);

/**
 * Performs unconstrained root finding for up to iter iterations
 * Return values is a tuple indicating:
 *     - (0,x,n) : convergence reached in x
 *     - (1,x,n) : convergence reached in f
 *     - (2,x,n) : divergence
 *     - (3,x,n) : degeneration of gradient (local extremum or saddle point)
 *     - (4,x,n) : maximum iterations reached
 *     - (5,x,n) : numerical error occurred
 * where x is the final point and n is the number of iterations used
 */
PyObject* findRoots(PyObject* startVals, int iter);

/// Same as findRoots, but with given bounds (xmin,xmax)
PyObject* findRootsBounded(PyObject* startVals, PyObject* boundVals, int iter);

/// destroys internal data structures
void destroy();


#endif
