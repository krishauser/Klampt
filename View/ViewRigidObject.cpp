#include "ViewRigidObject.h"
#include <errors.h>
#include <GLdraw/GL.h>
#include <GLdraw/drawextra.h>
#include <GLdraw/drawMesh.h>
#include <GLdraw/GLColor.h>
#include <GLdraw/GLError.h>
using namespace Math;
using namespace Meshing;

ViewRigidObject::ViewRigidObject()
{
  appearance.faceColor.set(0.4,0.2,0.8);
  obj = NULL;
}

ViewRigidObject::~ViewRigidObject()
{
}
  
void ViewRigidObject::Draw()
{
  if(!obj) return;
  if(appearance.geom == NULL)
    appearance.Set(obj->geometry);

  glDisable(GL_CULL_FACE);
  glPushMatrix();
  glMultMatrix(Matrix4(obj->T));

  appearance.DrawGL();

  glPopMatrix();
  glEnable(GL_CULL_FACE);
}
