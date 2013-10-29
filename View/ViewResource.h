#ifndef VIEW_RESOURCE_H
#define VIEW_RESOURCE_H

#include "Modeling/Resources.h"
#include "View/ViewRobot.h"
#include "View/ViewStance.h"
#include "View/ViewGrasp.h"

class ViewResource
{
 public:
  ViewResource(Robot* robot=NULL);
  void SetRobot(Robot* robot);
  void SetAnimTime(Real time);
  void DrawGL(const ResourcePtr& r);

  //helpers
  void RenderLinearPath(const LinearPathResource* rc,Real pathTime);
  void RenderMultiPath(const MultiPathResource* path,Real pathTime);

  ViewRobot configViewer,configsViewer;
  ViewRobot pathViewer; Real pathTime;
  Real pathIKResolution;
  ViewHold holdViewer;
  ViewStance stanceViewer;
  ViewGrasp graspViewer;
};

#endif 

