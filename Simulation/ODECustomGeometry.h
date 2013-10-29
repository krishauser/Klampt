#ifndef ODE_CUSTOM_MESH_H
#define ODE_CUSTOM_MESH_H

#include <geometry/AnyGeometry.h>
#include <ode/common.h>
using namespace Geometry;

struct CustomGeometryData
{
  AnyCollisionGeometry3D* geometry;
  Real outerMargin;
};


dGeomID dCreateCustomGeometry(AnyCollisionGeometry3D* geom,Real outerMargin=0);
CustomGeometryData* dGetCustomGeometryData(dGeomID o);
void InitODECustomGeometry();

#endif

