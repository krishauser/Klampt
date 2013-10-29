#ifndef ROBOT_RESOURCE_H
#define ROBOT_RESOURCE_H

/* TODO: add support for this */

#include <utils/ResourceLibrary.h>

class Resource
{
 public:
  bool load(const char* fn);
  bool save(const char* fn);
  const char* type() const;
  const char* name() const;
  const char* file() const;
  PyObject* object() const;
  Resource cast(const char* type) const;
  vector<Resource> extract(const char* type) const;

  ResourcePtr res;
};

#endif
