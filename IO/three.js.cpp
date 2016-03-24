#include "three.js.h"
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <sstream>
using namespace std;

unsigned char FloatColorChar(float v)
{
  return Max(Min(int(v*255.0),255),0);
}
int ToRGB32(const GLDraw::GLColor& col)
{
  unsigned char r=FloatColorChar(col.rgba[0]);
  unsigned char g=FloatColorChar(col.rgba[1]);
  unsigned char b=FloatColorChar(col.rgba[2]);
  return r << 16 | g << 8 | b;
}

string MakeRandomUUID()
{
  stringstream ss;
  boost::uuids::uuid uuid = boost::uuids::random_generator()();
  ss << uuid;
  return ss.str();
}

struct ThreeJSCache
{
  bool HasUUID(const Geometry::AnyCollisionGeometry3D& geom) const {
    return geometryUUIDs.count(&geom) != 0;
  }
  bool HasUUID(const GLDraw::GeometryAppearance& app) const {
    return materialUUIDs.count(&app) != 0;
  }
  string GetUUID(const Geometry::AnyCollisionGeometry3D& geom) {
    if(geometryUUIDs.count(&geom) == 0) {
      string res = MakeRandomUUID();
      geometryUUIDs[&geom] = res;
      return res;
    }
    return geometryUUIDs[&geom];
  }
  string GetUUID(const GLDraw::GeometryAppearance& app) {
    if(materialUUIDs.count(&app) == 0) {
      string res = MakeRandomUUID();
      materialUUIDs[&app] = res;
      return res;
    }
    return materialUUIDs[&app];
  }
  map<const Geometry::AnyCollisionGeometry3D*,string> geometryUUIDs;
  map<const GLDraw::GeometryAppearance*,string> materialUUIDs;
};

void ThreeJSExport(const RobotWorld& world,AnyCollection& out,ThreeJSCache& cache);
void ThreeJSExport(WorldSimulation& sim,AnyCollection& out,ThreeJSCache& cache);
void ThreeJSExport(const Robot& robot,AnyCollection& out,ThreeJSCache& cache);
void ThreeJSExport(const RigidObject& object,AnyCollection& out,ThreeJSCache& cache);
void ThreeJSExport(const Terrain& terrain,AnyCollection& out,ThreeJSCache& cache);
void ThreeJSExportGeometry(const ManagedGeometry& geom,AnyCollection& out,ThreeJSCache& cache);
void ThreeJSExportAppearance(const ManagedGeometry& geom,AnyCollection& out,ThreeJSCache& cache);
void ThreeJSExport(const Geometry::AnyCollisionGeometry3D& geom,AnyCollection& out,ThreeJSCache& cache);
void ThreeJSExport(const GLDraw::GeometryAppearance& app,AnyCollection& out,ThreeJSCache& cache);

void ThreeJSExport(const RigidTransform& T,AnyCollection& out)
{
  Matrix4 mat;
  T.get(mat);
  out.resize(16);
  //column major
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      out[j*4+i] = mat(i,j);
}

void ThreeJSExport(const RobotWorld& world,AnyCollection& out,ThreeJSCache& cache)
{
  out["metadata"]["version"] = 4.4;
  out["metadata"]["type"] = "Object";
  out["metadata"]["generator"] = "Klampt three.js export";
  AnyCollection& geometries = out["geometries"];
  AnyCollection& materials = out["materials"];
  //first, loop through all geometries and materials
  for(size_t i=0;i<world.robots.size();i++) {
    for(size_t j=0;j<world.robots[i]->links.size();j++) {
      AnyCollection geom,mat;
      ThreeJSExportGeometry(world.robots[i]->geomManagers[j],geom,cache);
      ThreeJSExportAppearance(world.robots[i]->geomManagers[j],mat,cache);
      if(geom.ismap()) 
	geometries[geometries.size()] = geom;
      if(mat.ismap()) 
	materials[materials.size()] = mat;
    }
  }
  for(size_t i=0;i<world.rigidObjects.size();i++) {
    AnyCollection geom,mat;
    ThreeJSExportGeometry(world.rigidObjects[i]->geometry,geom,cache);
    ThreeJSExportAppearance(world.rigidObjects[i]->geometry,mat,cache);
    if(geom.ismap()) 
      geometries[geometries.size()] = geom;
    if(mat.ismap()) 
      materials[materials.size()] = mat;
  }
  for(size_t i=0;i<world.terrains.size();i++) {
    AnyCollection geom,mat;
    ThreeJSExportGeometry(world.terrains[i]->geometry,geom,cache);
    ThreeJSExportAppearance(world.terrains[i]->geometry,mat,cache);
    if(geom.ismap()) 
      geometries[geometries.size()] = geom;
    if(mat.ismap()) 
      materials[materials.size()] = mat;
  }
  //now loop through all objects
  AnyCollection& rootobject = out["object"];
  rootobject["name"] = "world";
  rootobject["uuid"] = MakeRandomUUID();
  rootobject["type"] = "Scene";
  RigidTransform T; T.setIdentity();
  ThreeJSExport(T,rootobject["matrix"]);
  AnyCollection& clist = rootobject["children"];
  for(size_t i=0;i<world.robots.size();i++) 
    ThreeJSExport(*world.robots[i],clist[clist.size()],cache);
  for(size_t i=0;i<world.rigidObjects.size();i++) 
    ThreeJSExport(*world.rigidObjects[i],clist[clist.size()],cache);
  for(size_t i=0;i<world.terrains.size();i++) 
    ThreeJSExport(*world.terrains[i],clist[clist.size()],cache);
}

void ThreeJSExport(WorldSimulation& sim,AnyCollection& out,ThreeJSCache& cache)
{
  sim.UpdateModel();
  ThreeJSExport(*sim.world,out,cache);
  //TODO: export commanded config
}

void ThreeJSExport(const Robot& robot,AnyCollection& out,ThreeJSCache& cache)
{
  out["uuid"] = MakeRandomUUID();
  out["name"] = robot.name;
  out["type"] = "Group";
  out["children"].resize(0);
  RigidTransform T; T.setIdentity();
  ThreeJSExport(T,out["matrix"]);
  vector<AnyCollection*> llists(robot.links.size(),NULL);
  vector<AnyCollection*> clists(robot.links.size(),NULL);
  for(size_t i=0;i<robot.links.size();i++) {
    AnyCollection* clist = NULL;
    if(robot.parents[i] < 0) 
      clist = &out["children"];
    else {
      if(!clists[robot.parents[i]])
	//add children element
	clists[robot.parents[i]] = &(*llists[robot.parents[i]])["children"];
      clist = clists[robot.parents[i]];
    }
    //this accessor automatically appends to the array
    AnyCollection& me = (*clist)[clist->size()];
    llists[i] = &me;
    me["uuid"] = MakeRandomUUID();
    me["name"] = robot.LinkName(i);
    if(robot.geomManagers[i].Empty()) {
      me["type"] = "Group";
    }
    else {
      me["type"] = "Mesh";
      ThreeJSExportGeometry(robot.geomManagers[i],me["geometry"],cache);
      ThreeJSExportAppearance(robot.geomManagers[i],me["material"],cache);
    }
    RigidTransform Tparent,Trel;
    if(robot.parents[i] < 0) Tparent.setIdentity();
    else Tparent = robot.links[robot.parents[i]].T_World;
    Trel.mulInverseA(Tparent,robot.links[i].T_World);
    ThreeJSExport(Trel,me["matrix"]);
  }
}
void ThreeJSExport(const RigidObject& object,AnyCollection& out,ThreeJSCache& cache)
{
  out["uuid"] = MakeRandomUUID();
  out["name"] = object.name;
  if(object.geometry.Empty()) {
    out["type"] = "Group";
  }
  else {
    out["type"] = "Mesh";
    ThreeJSExportGeometry(object.geometry,out["geometry"],cache);
    ThreeJSExportAppearance(object.geometry,out["material"],cache);
  }
  ThreeJSExport(object.T,out["matrix"]);
}
void ThreeJSExport(const Terrain& terrain,AnyCollection& out,ThreeJSCache& cache)
{
  out["uuid"] = MakeRandomUUID();
  out["name"] = terrain.name;
  if(terrain.geometry.Empty()) {
    out["type"] = "Group";
  }
  else {
    out["type"] = "Mesh";
    ThreeJSExportGeometry(terrain.geometry,out["geometry"],cache);
    ThreeJSExportAppearance(terrain.geometry,out["material"],cache);
  }
  RigidTransform T; T.setIdentity();
  ThreeJSExport(T,out["matrix"]);
}

void ThreeJSExportGeometry(const ManagedGeometry& geom,AnyCollection& out,ThreeJSCache& cache)
{
  if(!geom.Empty())
    ThreeJSExport(*geom,out,cache);
}
void ThreeJSExportAppearance(const ManagedGeometry& geom,AnyCollection& out,ThreeJSCache& cache)
{
  if(!geom.Empty())
    ThreeJSExport(*geom.Appearance(),out,cache);
}
void ThreeJSExport(const Geometry::AnyCollisionGeometry3D& geom,AnyCollection& out,ThreeJSCache& cache)
{
  if(geom.Empty()) return;
  if(cache.HasUUID(geom)) {
    out = cache.GetUUID(geom);
    return;
  }
  if(geom.type == Geometry::AnyCollisionGeometry3D::Primitive) {
    const GeometricPrimitive3D& prim = geom.AsPrimitive();
    //TODO: save primitive
    //out["uuid"] = cache.GetUUID(geom);
    //if(prim.type == GeometricPrimitive3D::Sphere) 
  }
  else if(geom.type == Geometry::AnyCollisionGeometry3D::TriangleMesh) {
    const Meshing::TriMesh& mesh = geom.AsTriangleMesh();
    out["uuid"] = cache.GetUUID(geom);
    out["type"] = "Geometry";
    AnyCollection vertices, faces;
    vertices.resize(mesh.verts.size()*3);
    for(size_t i=0;i<mesh.verts.size();i++) {
      vertices[int(i*3)] = float(mesh.verts[i].x);
      vertices[int(i*3+1)] = float(mesh.verts[i].y);
      vertices[int(i*3+2)] = float(mesh.verts[i].z);
    }
    faces.resize(mesh.tris.size()*3);
    for(size_t i=0;i<mesh.tris.size();i++) {
      faces[int(i*3)] = mesh.tris[i].a;
      faces[int(i*3+1)] = mesh.tris[i].b;
      faces[int(i*3+2)] = mesh.tris[i].c;
    }
    out["vertices"] = vertices;
    out["faces"] = faces;
  }
  else {
    //can't export files of that type
  }
}
void ThreeJSExport(const GLDraw::GeometryAppearance& app,AnyCollection& out,ThreeJSCache& cache)
{
  if(cache.HasUUID(app)) { 
    //just a string
    out = cache.GetUUID(app);
    return;
  }
  else {
    //save material
    out["uuid"] = cache.GetUUID(app);
    out["type"] = "MeshStandardMaterial";
    int rgb = ToRGB32(app.faceColor);
    out["color"] = rgb;
    out["emissive"] = 0;
    if(app.faceColor.rgba[3] != 1.0) {
      out["transparent"] = true;
      out["opacity"] = app.faceColor.rgba[3];
    }
  }
}

///Exports a world to a JSON object that can be used in the three.js editor.
///Contains metadata, geometries, materials, and object items.
void ThreeJSExport(const RobotWorld& world,AnyCollection& out)
{
  ThreeJSCache cache;
  ThreeJSExport(world,out,cache);
}
///Exports a simulation to a JSON object that can be used in the three.js
///editor.  Contains metadata, geometries, materials, and object items.
///Draws the simulation world in natural color and the commanded
///world in transparent green.
void ThreeJSExport(WorldSimulation& sim,AnyCollection& out)
{
  ThreeJSCache cache;
  ThreeJSExport(sim,out,cache);
}
///Exports a robot to a JSON object that can be used in the three.js editor.
///The result is a hierarchical set of Mesh or Group objects.
void ThreeJSExport(const Robot& robot,AnyCollection& out)
{
  ThreeJSCache cache;
  ThreeJSExport(robot,out,cache);
}
///Exports a rigid object to a JSON object that can be used in the three.js
///editor. The result is a Mesh object (or Group if the geometry is empty).
void ThreeJSExport(const RigidObject& object,AnyCollection& out)
{
  ThreeJSCache cache;
  ThreeJSExport(object,out,cache);
}
///Exports a rigid object to a JSON object that can be used in the three.js
///editor. The result is a Mesh object (or Group if the geometry is empty).
void ThreeJSExport(const Terrain& terrain,AnyCollection& out)
{
  ThreeJSCache cache;
  ThreeJSExport(terrain,out,cache);
}
///Exports geometry to a three.js scene Geometry instance.  The "uuid"
///element of the output gives the unique ID number that can be used elsewhere
void ThreeJSExportGeometry(const ManagedGeometry& geom,AnyCollection& out)
{
  ThreeJSCache cache;
  ThreeJSExportGeometry(geom,out,cache);
}
///Exports appearance to a three.js scene Material instance.  The "uuid"
///element of the output gives the unique ID number that can be used elsewhere
void ThreeJSExportAppearance(const ManagedGeometry& geom,AnyCollection& out)
{
  ThreeJSCache cache;
  ThreeJSExportAppearance(geom,out,cache);
}
///Exports to a three.js scene Geometry instance
void ThreeJSExport(const Geometry::AnyCollisionGeometry3D& geom,AnyCollection& out)
{
  ThreeJSCache cache;
  ThreeJSExport(geom,out,cache);
}
///Exports to a three.js scene Material instance
void ThreeJSExport(const GLDraw::GeometryAppearance& app,AnyCollection& out)
{
  ThreeJSCache cache;
  ThreeJSExport(app,out,cache);
}
