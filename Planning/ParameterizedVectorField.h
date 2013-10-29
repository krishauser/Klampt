#ifndef PARAMETERIZED_VECTOR_FIELD_H
#define PARAMETERIZED_VECTOR_FIELD_H

#include <math/function.h>
using namespace Math;

/** @brief A VectorFieldFunction that depends on a parameter theta
 */
class ParameterizedVectorFieldFunction : public VectorFieldFunction
{
 public:
  virtual ~ParameterizedVectorFieldFunction() {}
  virtual int NumParameters() const { return 0; }
  virtual void SetParameter(const Vector& theta) {}
};

#endif

