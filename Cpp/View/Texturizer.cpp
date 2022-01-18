#include "Texturizer.h"
#include "ViewTextures.h"
#include <KrisLibrary/errors.h>
#include <KrisLibrary/GLdraw/GLColor.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/meshing/LSConformalMapping.h>
#include <map>
#include <iostream>
using namespace std;
using namespace Math;
using namespace Meshing;
using namespace GLDraw;
using namespace Klampt;

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

Texturizer::Texturizer()
{
  texture = "checker";
  texCoords = XYTexCoords;
  texCoordAutoScale = false;
  texDivs = 1;
}
  
bool Texturizer::Set(ManagedGeometry& geom)
{
  GLDraw::GeometryAppearance* app = geom.Appearance().get();
  Assert(app != NULL);
  if(!app->geom) 
    app->Set(*geom);

  //set up the texture coordinates
  if(texture.empty()) {
    app->tex1D = NULL;
    app->tex2D = NULL;
  }
  else {
    ViewTextures::Initialize();

    app->tex2D = ViewTextures::Load(texture.c_str());
    if(!app->tex2D) {
      fprintf(stderr,"Texture image %s couldn't be loaded\n",texture.c_str());
      return false;
    }
    if(app->tex2D->h == 1) {
      app->tex1D = app->tex2D;
      app->tex2D = NULL;
    }
  }

  switch(texCoords) {
  case ParameterizedTexCoord:
    if(geom->type != Geometry::AnyGeometry3D::TriangleMesh)
      fprintf(stderr,"Can't wrap texture coordinates around non-triangle mesh geometry\n");
    else
      SetupTextureCoordinates(geom->TriangleMeshCollisionData(),app->texcoords);
    break;
  case XYTexCoords:
    {
      app->texgen.resize(2);
      if(texCoordAutoScale == false) {
	app->texgen[0] = Vector4(texDivs,0,0,0);
	app->texgen[1] = Vector4(0,texDivs,0,0);
      }
      else {
	AABB3D bb = geom->GetAABB();
	Real scale,offset;
	if(bb.bmin.x == bb.bmax.x) {
	  app->texgen[0] = Vector4(1,0,0,-bb.bmin.x+1);
	}
	else {
	  Real width = Inf;
	  if(app->tex2D) width = app->tex2D->w;
	  scale = (texDivs - 2.0/width)/(bb.bmax.x-bb.bmin.x);
	  offset = -(scale+1.0/width)*bb.bmin.x;
	  app->texgen[0] = Vector4(scale,0,0,offset);
	}
	if(bb.bmin.x == bb.bmax.x) {
	  app->texgen[1] = Vector4(0,1,0,-bb.bmin.x+1);
	}
	else {
	  Real height = Inf;
	  if(app->tex2D) height = app->tex2D->h;
	  scale = (texDivs - 2.0/height)/(bb.bmax.y-bb.bmin.y);
	  offset = -(scale+1.0/height)*bb.bmin.y;
	  app->texgen[1] = Vector4(0,scale,0,offset);
	}
      }
    }
    break;
  case ZTexCoord:
    {
      app->texgen.resize(1);
      if(texCoordAutoScale == false) {
	app->texgen[0] = Vector4(0,0,texDivs,0);
      }
      else {
	Real minz,maxz;
	AABB3D bb = geom->GetAABB();
	minz = bb.bmin.z;
	maxz = bb.bmax.z;
	Real scale,offset;
	if(maxz == minz) {
	  scale=1;
	  offset=-minz+1;
	}
	else {
	  Real width = Inf;
	  if(app->tex1D) width = app->tex1D->w;
	  //scale = texDivs/(maxz-minz);
	  scale = (texDivs - 2.0/width)/(maxz-minz);
	  offset = -(scale+1.0/width)*minz;
	}
	app->texgen[0] = Vector4(0,0,scale,offset);
      }
    }
  }   
  return true;
}
