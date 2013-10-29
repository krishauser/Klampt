#include "ViewEnvironment.h"
#include "ViewTextures.h"
#include <errors.h>
#include <GLdraw/GL.h>
#include <GLdraw/drawextra.h>
#include <GLdraw/drawMesh.h>
#include <GLdraw/GLColor.h>
#include <GLdraw/GLError.h>
#include <math/random.h>
#include <meshing/LSConformalMapping.h>
using namespace Math;
using namespace Meshing;

void SetupTextureCoordinates(TriMeshWithTopology& mesh,vector<Vector2>& texcoords)
{
  TriMeshChart chart(mesh);
  LSConformalMapping mapping(mesh,chart);
  if(mapping.Calculate()) {
    texcoords = chart.coordinates;
  }
  else {
    cerr<<"Error generating texcoords"<<endl;
  }
}

ViewEnvironment::ViewEnvironment()
{
  appearance.faceColor.set(0.4,0.3,0.1);
  texture = NoiseTexture;
  texture = CheckerTexture;
  texCoords = XYTexCoords;
  texDivs = 1;
}

ViewEnvironment::~ViewEnvironment()
{
}
  
void ViewEnvironment::Draw()
{
  if(!appearance.geom) {
    appearance.Set(env->geometry);

    //set up the texture coordinates
    if(texture != NoTexture) {
      ViewTextures::Initialize();

      appearance.faceColor.rgba[0]*=2;
      appearance.faceColor.rgba[1]*=2;
      appearance.faceColor.rgba[2]*=2;

      switch(texture) {
      case NoiseTexture:
	appearance.tex2D = new GLTexture2D(ViewTextures::noise);
	break;
      case CheckerTexture:
	appearance.tex2D = new GLTexture2D(ViewTextures::checker);
	break;
      case GradientTexture:
	appearance.tex1D = new GLTexture1D(ViewTextures::grayscaleGradient);
	break;
      case ColorGradientTexture:
	appearance.tex1D = new GLTexture1D(ViewTextures::rainbowGradient);
	appearance.faceColor.set(0.5,0.5,0.5);
	break;
      }
    }

    switch(texCoords) {
    case ParameterizedTexCoord:
      SetupTextureCoordinates(*AnyCast<Geometry::CollisionMesh>(&env->geometry.collisionData),appearance.texcoords);
      break;
    case XYTexCoords:
      {
	appearance.texgen.resize(2);
	appearance.texgen[0] = Vector4(texDivs,0,0,0);
	appearance.texgen[1] = Vector4(0,texDivs,0,0);
      }
      break;
    case ZTexCoord:
      {
	Real minz,maxz;
	AABB3D bb = env->geometry.GetAABB();
	minz = bb.bmin.z;
	maxz = bb.bmax.z;
	Real scale,offset;
	if(maxz == minz) {
	  scale=1;
	  offset=-minz+1;
	}
	else {
	  //scale = texDivs/(maxz-minz);
	  scale = (texDivs - 2.0/64.0)/(maxz-minz);
	  offset = -(scale+1.0/64.0)*minz;
	}
	appearance.texgen.resize(1);
	appearance.texgen[0] = Vector4(0,0,scale,offset);
      }
    }   
  }
  appearance.DrawGL();
  /*
  if(edges) {
    glDisable(GL_LIGHTING);
    glDepthFunc(GL_LEQUAL);
    glColor4fv(edgeColor.rgba);
    glBegin(GL_LINES);
    if(env->mesh.triNeighbors.empty())
      env->mesh.CalcTriNeighbors();
    Vector3 ni;
    const static Real creaseAngle = Pi/6.0;
    Real cosCreaseAngle = Cos(creaseAngle);
    for(size_t i=0;i<env->mesh.tris.size();i++) {
      ni = env->mesh.TriangleNormal(i);
      for(int k=0;k<3;k++) {
	int j=env->mesh.triNeighbors[i][k];
	if(j<0) continue;
	Vector3 nj=env->mesh.TriangleNormal(j);
	if(dot(ni,nj) < cosCreaseAngle) {
	  int v1,v2;
	  env->mesh.tris[i].getCompliment(k,v1,v2);
	  Assert(v1 >= 0 && v1 < (int)env->mesh.verts.size());
	  Assert(v2 >= 0 && v2 < (int)env->mesh.verts.size());
	  glVertex3v(env->mesh.verts[v1]);
	  glVertex3v(env->mesh.verts[v2]);
	}
      }
    }
    glEnd();
    glDepthFunc(GL_LESS);
  }
  */
}
