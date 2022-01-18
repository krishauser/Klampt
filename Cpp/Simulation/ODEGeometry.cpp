#include "ODEGeometry.h"
#include "ODECommon.h"
#include "ODECustomGeometry.h"
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/meshing/TriMeshOperators.h>
#include <KrisLibrary/meshing/VolumeGrid.h>
#include <ode/ode.h>
#include <ode/common.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/errors.h>
using namespace Meshing;

#define USING_GIMPACT 0

namespace Klampt {

ODEGeometry::ODEGeometry()
  :triMeshDataID(0),geomID(0),verts(NULL),indices(NULL),normals(NULL),numVerts(0),numTris(0),geometrySelfAllocated(false)
{
  surface.kRestitution = 0;
  surface.kFriction = 0;
  surface.kStiffness = Inf;
  surface.kDamping = Inf;

#if USING_GIMPACT
  numVertComponents = 3;
#else
  numVertComponents = 4;
#endif
}

ODEGeometry::~ODEGeometry()
{
  Clear();
}

void ODEGeometry::Create(AnyCollisionGeometry3D* geom,dSpaceID space,Vector3 offset,bool useCustomMesh)
{
  //printf("ODEGeometry: Collision detection method: %s\n",(useCustomMesh?"custom":"GIMPACT"));
  Clear();
  if(!useCustomMesh) {
    Assert(geom->type == AnyGeometry3D::TriangleMesh);
	const TriMesh* meshp = AnyCast<TriMesh>(&geom->data);
    if(!meshp) FatalError("Geometry is not a triangle mesh");
    const TriMesh& mesh = *AnyCast<TriMesh>(&geom->data);
    Assert(numVertComponents == 3 || numVertComponents == 4);
#if USING_GIMPACT
    //GIMPACT needs this
    Assert(numVertComponents == 3);
#endif
    
    numVerts = (int)mesh.verts.size();
    verts = new dReal[mesh.verts.size()*numVertComponents];
    for(size_t i=0;i<mesh.verts.size();i++) {
      if(numVertComponents == 3)
	CopyVector3(&verts[i*numVertComponents],mesh.verts[i]+offset);
      else {
	CopyVector(&verts[i*numVertComponents],mesh.verts[i]+offset);
	verts[i*numVertComponents+3] = 1.0;
      }
    }
    
    numTris = (int)mesh.tris.size();
    indices = new int[mesh.tris.size()*3];
    normals = new dReal[mesh.tris.size()*3];
    for(size_t i=0;i<mesh.tris.size();i++) {
      indices[i*3] = mesh.tris[i].a;
      indices[i*3+1] = mesh.tris[i].b;
      indices[i*3+2] = mesh.tris[i].c;
      CopyVector3(&normals[i*3],mesh.TriangleNormal(i));
    }
    
    triMeshDataID = dGeomTriMeshDataCreate();
    //for some reason, ODE behaves better when it calculates its own normals
#if defined(dDOUBLE)
    if(USING_GIMPACT)
      FatalError("GIMPACT doesn't work with doubles, recompile with dSINGLE");
    //dGeomTriMeshDataBuildDouble1(triMeshDataID,verts,sizeof(dReal)*numVertComponents,numVerts,indices,numTris*3,sizeof(int)*3,normals);
    dGeomTriMeshDataBuildDouble(triMeshDataID,verts,sizeof(dReal)*numVertComponents,numVerts,indices,numTris*3,sizeof(int)*3);
#else
    //dGeomTriMeshDataBuildSingle1(triMeshDataID,verts,sizeof(dReal)*numVertComponents,numVerts,indices,numTris*3,sizeof(int)*3,normals);
    dGeomTriMeshDataBuildSingle(triMeshDataID,verts,sizeof(dReal)*numVertComponents,numVerts,indices,numTris*3,sizeof(int)*3);
#endif
    geomID = dCreateTriMesh(space, triMeshDataID, 0, 0, 0);

  /* Sanity check!
  for(size_t i=0;i<mesh.tris.size();i++) {
    dVector3 v0,v1,v2;
    dVector3 x0,x1,x2;
    dGeomTriMeshGetTriangle(geomID,i,&v0,&v1,&v2);
    Triangle3D tri;
    mesh.GetTriangle(i,tri);
    tri.a += offset;
    tri.b += offset;
    tri.c += offset;
    CopyVector(x0,tri.a);
    CopyVector(x1,tri.b);
    CopyVector(x2,tri.c);
    printf("triangle %d:\n",i);
    printf("  (%g,%g,%g):\n",x0[0],x0[1],x0[2]);
    printf("  (%g,%g,%g):\n",x1[0],x1[1],x1[2]);
    printf("  (%g,%g,%g):\n",x2[0],x2[1],x2[2]);
    printf("ret:\n");
    printf("  (%g,%g,%g):\n",v0[0],v0[1],v0[2]);
    printf("  (%g,%g,%g):\n",v1[0],v1[1],v1[2]);
    printf("  (%g,%g,%g):\n",v2[0],v2[1],v2[2]);
    for(int k=0;k<3;k++) {
      Assert(FuzzyEquals(x0[k],v0[k]));
      Assert(FuzzyEquals(x2[k],v1[k]));
      Assert(FuzzyEquals(x1[k],v2[k]));
    }
  }
  */
  }
  else {
    Timer timer;
    geom->InitCollisionData();
    double t = timer.ElapsedTime();
    if(t > 0.1) printf("ODEGeometry: initializing collision data took time %gs\n",t);

    //add offsets
    collisionGeometry = geom;
    geomID = dCreateCustomGeometry(collisionGeometry,0.0);
    dGetCustomGeometryData(geomID)->odeOffset = offset;
    dSpaceAdd(space,geomID);
  }
}

void ODEGeometry::Clear()
{
  SafeDeleteProc(geomID,dGeomDestroy);
  SafeDeleteProc(triMeshDataID,dGeomTriMeshDataDestroy);
  SafeArrayDelete(verts);
  SafeArrayDelete(indices);
  SafeArrayDelete(normals);
  numTris = numVerts = 0;
  if(geometrySelfAllocated) {
    geometrySelfAllocated = false;
    delete collisionGeometry;
  }
  collisionGeometry = NULL;
}

void ODEGeometry::DrawGL()
{
  if(!verts) return;

  glColor3f(1,1,0);
  glPointSize(3.0);
  glBegin(GL_POINTS);
  for(int i=0;i<numVerts;i++) {
#if defined(dDOUBLE)
    glVertex3dv(&verts[i*numVertComponents]);
#else
    glVertex3fv(&verts[i*numVertComponents]);
#endif
  }
  glEnd();
  glPointSize(1.0);

  glColor3f(1,0.5,0);
  glBegin(GL_LINES);
  Real len=0.1;
  for(int i=0;i<numTris;i++) {
    int a=indices[i*3];
    int b=indices[i*3+1];
    int c=indices[i*3+2];
    dReal centroid[3]={0,0,0};
    for(int k=0;k<3;k++)
      centroid[k] = (verts[a*numVertComponents+k]+verts[b*numVertComponents+k]+verts[c*numVertComponents+k])/3.0;
#if defined(dDOUBLE)
    glVertex3dv(centroid);
    glVertex3d(centroid[0]+len*normals[i*3],
	       centroid[1]+len*normals[i*3+1],
	       centroid[2]+len*normals[i*3+2]);
#else
    glVertex3fv(centroid);
    glVertex3f(centroid[0]+len*normals[i*3],
	       centroid[1]+len*normals[i*3+1],
	       centroid[2]+len*normals[i*3+2]);
#endif //dDOUBLE
  }
  glEnd();
}

void ODEGeometry::SetPadding(Real padding)
{
  if(collisionGeometry) {
    //printf("Setting padding %g\n",padding);
    dGetCustomGeometryData(geom())->outerMargin = padding;
  }
  else {
    //fprintf(stderr,"Not using boundary layer, setting padding %g has no effect\n",padding);
  }
}

Real ODEGeometry::GetPadding()
{
  if(collisionGeometry)
    return dGetCustomGeometryData(geom())->outerMargin;
  else
    return 0;
}

AnyCollisionGeometry3D* _Preshrink(AnyCollisionGeometry3D* geom,Real padding) {
  if(padding==0) return geom;
  switch(geom->type) {
  case AnyCollisionGeometry3D::Primitive:
    if(geom->AsPrimitive().type == GeometricPrimitive3D::Sphere) {
      Sphere3D* s = AnyCast<Sphere3D>(&geom->AsPrimitive().data);
      Sphere3D s2 = *s;
      s2.radius -= padding;
      AnyCollisionGeometry3D* res = new AnyCollisionGeometry3D(GeometricPrimitive3D(s2));
      return res;
    }
    fprintf(stderr,"SetPaddingWithPreshink: Cannot shrink geometric primitives\n");
    return geom;
  case AnyCollisionGeometry3D::PointCloud:
    fprintf(stderr,"SetPaddingWithPreshink: Cannot shrink point clouds\n");
    return geom;
  case AnyCollisionGeometry3D::TriangleMesh:
    {
      const Meshing::TriMesh& morig = geom->AsTriangleMesh();
      Meshing::TriMeshWithTopology mnew;
      mnew.verts = morig.verts;
      mnew.tris = morig.tris;
      int numflips = Meshing::ApproximateShrink(mnew,padding);
      if(numflips > 0) {
	fprintf(stderr,"SetPaddingWithPreshink: Warning, mesh shrinkage by amount %g created %d triangle flips\n",padding,numflips);
      }
      AnyCollisionGeometry3D* res = new AnyCollisionGeometry3D(mnew);
      res->margin = geom->margin;
      res->InitCollisionData();
      return res;
    }
    break;
  case AnyCollisionGeometry3D::ImplicitSurface:
    {
      //this doesn't really make sense to do, but if the user wants it...
      const Meshing::VolumeGrid& vgrid = geom->AsImplicitSurface();
      Meshing::VolumeGrid vgridnew = vgrid;
      vgridnew.Add(-padding);
      AnyCollisionGeometry3D* res = new AnyCollisionGeometry3D(vgridnew);
      res->margin = geom->margin;
      return res;
    }
  case AnyCollisionGeometry3D::Group:
    {
      fprintf(stderr,"TODO: Can't do preshrink for group geometries yet\n");
      return geom;
    }
    break;
  default: 
    FatalError("Invalid geometry type %s\n",geom->TypeName());
    return geom;
  }
}


AnyCollisionGeometry3D* ODEGeometry::SetPaddingWithPreshrink(Real padding,bool inplace)
{
  if(collisionGeometry) {
    printf("ODEGeometry::SetPaddingWithPreshrink: Working...");
    fflush(stdout);
    AnyCollisionGeometry3D* newgeom = _Preshrink(collisionGeometry,padding);
    printf(" Done.\n");
    if(collisionGeometry != newgeom) {
      if(inplace) {
	//modify original geometry
	collisionGeometry->data = newgeom->data;
	collisionGeometry->collisionData = newgeom->collisionData;
      }
      else {
	//changing geometry without touching original pointer, need to re-add to the geom's space
	collisionGeometry = newgeom;
	geometrySelfAllocated = true;
	//get old data
	Vector3 offset = dGetCustomGeometryData(geomID)->odeOffset;
	dSpaceID space = dGeomGetSpace(geomID);
	dBodyID body = dGeomGetBody(geomID);
	void* data = dGeomGetData(geomID);
	dSpaceRemove(space,geomID);
	//deallocate old geometry
	dGeomDestroy(geomID);
	if(collisionGeometry != NULL) {
	  geomID = dCreateCustomGeometry(collisionGeometry,0.0);
	  dGetCustomGeometryData(geomID)->odeOffset = offset;
	  dGeomSetBody(geomID,body);
	  dGeomSetData(geomID,data);
	  dSpaceAdd(space,geomID);
	}
      }
    }
    SetPadding(padding);
    return collisionGeometry;
  }
  else {
    fprintf(stderr,"ODEGeometry::SetPaddingWithPreshrink: Not using boundary layer, setting padding/preshrink %g has no effect\n",padding);
    return NULL;
  }
}

} //namespace Klampt