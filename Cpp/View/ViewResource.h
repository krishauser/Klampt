#ifndef VIEW_RESOURCE_H
#define VIEW_RESOURCE_H

#include <Klampt/Modeling/Resources.h>
#include "ViewRobot.h"
#include "ViewStance.h"
#include "ViewGrasp.h"

namespace Klampt {

class ViewResource
{
 public:
  ViewResource(RobotModel* robot=NULL);
  void SetRobot(RobotModel* robot);
  void SetAnimTime(Real time);
  void DrawGL(const ResourcePtr& r);

  //helpers
  void RenderLinearPath(const LinearPathResource* rc,Real pathTime);
  void RenderMultiPath(const MultiPathResource* path,Real pathTime);
  void GetAnimConfig(const ResourcePtr& r,Config& q);
  void GetLinearPathConfig(const LinearPathResource* rc,Real pathTime,Config& q);
  void GetMultiPathConfig(const MultiPathResource* path,Real pathTime,Config& q);


  ViewRobot configViewer,configsViewer;
  ViewRobot pathViewer; Real pathTime;
  Real pathIKResolution;
  ViewHold holdViewer;
  ViewStance stanceViewer;
  ViewGrasp graspViewer;
  //custom draw
  vector<const void*> items;
  vector<shared_ptr<Geometry::AnyGeometry3D> > geometries;
  vector<GLDraw::GeometryAppearance> appearances;
};

} //namespace Klampt

#endif 

