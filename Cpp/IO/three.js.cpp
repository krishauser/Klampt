#include "three.js.h"
#include <KrisLibrary/meshing/MeshPrimitives.h>
#include <KrisLibrary/meshing/PointCloud.h>
#include <KrisLibrary/math/random.h>
//#include <boost/uuid/uuid.hpp>            // uuid class
//#include <boost/uuid/uuid_generators.hpp> // generators
//#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <sstream>
using namespace std;

#define THREE_JS_OLD_VERSION 0

namespace Klampt {

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
  char str[33];
  str[32]=0;
  char alphaNumeric[] = {'a','b','c','d','e','f','g','0','1','2','3','4','5','6','7','8','9'};
  //make a 32 char long string
  
  //boost::uuids::uuid uuid = boost::uuids::random_generator()();
  for(int i = 0; i<32; i++){
    str[i] = alphaNumeric[Math::RandInt(16)];
  }
  //ss << uuid;
  return string(str);
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

void ThreeJSExport(const WorldModel& world,AnyCollection& out,ThreeJSCache& cache);
void ThreeJSExport(Simulator& sim,AnyCollection& out,ThreeJSCache& cache);
void ThreeJSExport(const RobotModel& robot,AnyCollection& out,ThreeJSCache& cache);
void ThreeJSExport(const RigidObjectModel& object,AnyCollection& out,ThreeJSCache& cache);
void ThreeJSExport(const TerrainModel& terrain,AnyCollection& out,ThreeJSCache& cache);
void ThreeJSExport(const ManagedGeometry& geom,AnyCollection& out,ThreeJSCache& cache);
void ThreeJSExportGeometry(const ManagedGeometry& geom,AnyCollection& out,ThreeJSCache& cache);
void ThreeJSExportAppearance(const ManagedGeometry& geom,AnyCollection& out,ThreeJSCache& cache);
void ThreeJSExport(const Geometry::AnyCollisionGeometry3D& geom,AnyCollection& out,ThreeJSCache& cache);
void ThreeJSExport(const GLDraw::GeometryAppearance& app,const Geometry::AnyCollisionGeometry3D& geom,AnyCollection& out,ThreeJSCache& cache);

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




void ThreeJSExport(const WorldModel& world,AnyCollection& out,ThreeJSCache& cache)
{
  out["metadata"]["version"] = 4.4;
  out["metadata"]["type"] = "Object";
  out["metadata"]["fullscene"] = true;
  out["metadata"]["generator"] = "Klampt three.js export";
  AnyCollection& geometries = out["geometries"];
  AnyCollection& materials = out["materials"];
  //even if the world is empty, this should be set to an array
  geometries.resize(0);
  materials.resize(0);
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
  //ThreeJSExport(T,rootobject["matrix"]);
  AnyCollection& clist = rootobject["children"];
  for(size_t i=0;i<world.robots.size();i++) 
    ThreeJSExport(*world.robots[i],clist[clist.size()],cache);
  for(size_t i=0;i<world.rigidObjects.size();i++) 
    ThreeJSExport(*world.rigidObjects[i],clist[clist.size()],cache);
  for(size_t i=0;i<world.terrains.size();i++) 
    ThreeJSExport(*world.terrains[i],clist[clist.size()],cache);

  vector<GLDraw::GLLight> lights = world.lights;
  if(lights.empty()) {
    lights.resize(1);
    lights[0].setColor(GLDraw::GLColor(1,1,1));
    lights[0].setDirectionalLight(Vector3(0.2,0.4,-1));
  }
  for(size_t i=0;i<lights.size();i++) {
    AnyCollection light;
    light["uuid"] = MakeRandomUUID();
    stringstream ss;
    ss<<"Light "<<i+1;
    light["name"] = ss.str();
    light["color"] = ToRGB32(lights[i].diffuse);
    light["intensity"]=1;
    if(lights[i].position[3] == 0) {
      light["type"] = "DirectionalLight";
      AnyCollection pos;
      pos.resize(3);
      pos[0] = -lights[i].position[0]*20;
      pos[1] = -lights[i].position[1]*20;
      pos[2] = -lights[i].position[2]*20;
      AnyCollection zero;
      zero.resize(3);
      zero[0] = 0;
      zero[1] = 0;
      zero[2] = 0;
      light["position"] = pos;
      light["target"]["position"] = zero;
    }
    else if(lights[i].spot_exponent > 0) {
      light["type"] = "SpotLight";
      AnyCollection pos;
      pos.resize(3);
      pos[0] = lights[i].position[0];
      pos[1] = lights[i].position[1];
      pos[2] = lights[i].position[2];
      AnyCollection zero;
      zero.resize(3);
      zero[0] = lights[i].position[0] + lights[i].spot_direction[0]*20;
      zero[1] = lights[i].position[1] + lights[i].spot_direction[1]*20;
      zero[2] = lights[i].position[2] + lights[i].spot_direction[2]*20;
      light["position"] = pos;
      light["angle"] = lights[i].spot_cutoff;
      light["penumbra"] = 1.0-Exp(-lights[i].spot_exponent);
      light["target"]["position"] = zero;
    }
    else {
      light["type"] = "PointLight";
      AnyCollection pos;
      pos.resize(3);
      pos[0] = lights[i].position[0];
      pos[1] = lights[i].position[1];
      pos[2] = lights[i].position[2];
      AnyCollection zero;
      zero.resize(3);
      zero[0] = 0;
      zero[1] = 0;
      zero[2] = 0;
      light["position"] = pos;
      light["target"]["position"] = zero;
    }
      /*
      AnyCollection lightMatrix;
      lightMatrix[0]=1;
      lightMatrix[1]=0;
      lightMatrix[2]=0;
      lightMatrix[3]=0;
      lightMatrix[4]=0;
      lightMatrix[5]=1;
      lightMatrix[6]=0;
      lightMatrix[7]=0;
      lightMatrix[8]=0;
      lightMatrix[9]=0;
      lightMatrix[10]=1;
      lightMatrix[11]=0;
      lightMatrix[12]=2;
      lightMatrix[13]=0;
      lightMatrix[14]=9;
      lightMatrix[15]=1;

      light["matrix"]=lightMatrix;
      */
    clist[clist.size()]=light;
  }

  AnyCollection sceneMatrix; //adjust to match view in three.js
  sceneMatrix[0]=1;
  sceneMatrix[1]=0;
  sceneMatrix[2]=0;
  sceneMatrix[3]=0;
  sceneMatrix[4]=0;
  sceneMatrix[5]=0;
  sceneMatrix[6]=-1;
  sceneMatrix[7]=0;
  sceneMatrix[8]=0;
  sceneMatrix[9]=1;
  sceneMatrix[10]=0;
  sceneMatrix[11]=0;
  sceneMatrix[12]=0;
  sceneMatrix[13]=0;
  sceneMatrix[14]=0;
  sceneMatrix[15]=1;

  rootobject["matrix"]=sceneMatrix;
}

void ThreeJSExportTransforms(const WorldModel& world,AnyCollection& out)
{
  out["metadata"]["version"] = 4.4;
  out["metadata"]["type"] = "Object";
  out["metadata"]["fullscene"] = false;
  out["metadata"]["generator"] = "Klampt three.js export";

  //now loop through all objects
  AnyCollection& clist = out["object"];

  //AnyCollection& clist = rootobject["object"];

  AnyCollection sceneMatrix; //adjust to match view in three.js
  sceneMatrix[0]=1;
  sceneMatrix[1]=0;
  sceneMatrix[2]=0;
  sceneMatrix[3]=0;
  sceneMatrix[4]=0;
  sceneMatrix[5]=0;
  sceneMatrix[6]=-1;
  sceneMatrix[7]=0;
  sceneMatrix[8]=0;
  sceneMatrix[9]=1;
  sceneMatrix[10]=0;
  sceneMatrix[11]=0;
  sceneMatrix[12]=0;
  sceneMatrix[13]=0;
  sceneMatrix[14]=0;
  sceneMatrix[15]=1;

  AnyCollection worldUpdate;
  worldUpdate["name"]="world";
  worldUpdate["matrix"]=sceneMatrix;

  clist[clist.size()]=worldUpdate;

  //AnyCollection& clist = rootobject["children"];
  for(size_t i=0;i<world.robots.size();i++) 
    ThreeJSExportTransforms(*world.robots[i],clist);
  for(size_t i=0;i<world.rigidObjects.size();i++) 
    ThreeJSExportTransforms(*world.rigidObjects[i],clist[clist.size()]);
  for(size_t i=0;i<world.terrains.size();i++) 
    ThreeJSExportTransforms(*world.terrains[i],clist[clist.size()]);

  AnyCollection lightUpdate;
  
  /*
  AnyCollection lightMatrix;
  lightMatrix[0]=1;
  lightMatrix[1]=0;
  lightMatrix[2]=0;
  lightMatrix[3]=0;
  lightMatrix[4]=0;
  lightMatrix[5]=1;
  lightMatrix[6]=0;
  lightMatrix[7]=0;
  lightMatrix[8]=0;
  lightMatrix[9]=0;
  lightMatrix[10]=1;
  lightMatrix[11]=0;
  lightMatrix[12]=2;
  lightMatrix[13]=0;
  lightMatrix[14]=9;
  lightMatrix[15]=1;

  lightUpdate["name"] = "DirectionalLight 1";
  lightUpdate["matrix"]=lightMatrix;
  clist[clist.size()]=lightUpdate;
  */

}

void ThreeJSExport(Simulator& sim,AnyCollection& out,ThreeJSCache& cache)
{
  sim.UpdateModel();
  ThreeJSExport(*sim.world,out,cache);
  //TODO: export commanded config
}

void ThreeJSExportTransforms(Simulator& sim,AnyCollection& out)
{
  sim.UpdateModel();
  ThreeJSExportTransforms(*sim.world,out);
  //TODO: export commanded config
}


void ThreeJSExport(const RobotModel& robot,AnyCollection& out,ThreeJSCache& cache)
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
    ThreeJSExport(robot.geomManagers[i],me,cache);
    RigidTransform Tparent,Trel;
    if(robot.parents[i] < 0) Tparent.setIdentity();
    else Tparent = robot.links[robot.parents[i]].T_World;
    Trel.mulInverseA(Tparent,robot.links[i].T_World);
    ThreeJSExport(Trel,me["matrix"]);
  }
}

void ThreeJSExportTransforms(const RobotModel& robot,AnyCollection& out)
{  
  AnyCollection robotUpdate;
  
  robotUpdate["name"] = robot.name;
  RigidTransform T; T.setIdentity();
  ThreeJSExport(T,robotUpdate["matrix"]);

  out[out.size()]=robotUpdate;

  vector<AnyCollection*> llists(robot.links.size(),NULL);
  vector<AnyCollection*> clists(robot.links.size(),NULL);

  for(size_t i=0;i<robot.links.size();i++) {
    //AnyCollection* clist = NULL;
    //if(robot.parents[i] < 0) 
      //clist = &out["children"];
    //else {
    //  if(!clists[robot.parents[i]])
  //add children element
    //clists[robot.parents[i]] = &(*llists[robot.parents[i]])["children"];
    //  clist = clists[robot.parents[i]];
    //}

    //this accessor automatically appends to the array
    //nyCollection& me = (*clist)[clist->size()];
    //llists[i] = &me;
    AnyCollection robotLinkUpdate;
   
    robotLinkUpdate["name"] = robot.LinkName(i);
    
    RigidTransform Tparent,Trel;
    if(robot.parents[i] < 0) Tparent.setIdentity();
    else Tparent = robot.links[robot.parents[i]].T_World;
    Trel.mulInverseA(Tparent,robot.links[i].T_World);
    ThreeJSExport(Trel,robotLinkUpdate["matrix"]);

    out[out.size()]=robotLinkUpdate;
  }
}

void ThreeJSExport(const RigidObjectModel& object,AnyCollection& out,ThreeJSCache& cache)
{
  out["uuid"] = MakeRandomUUID();
  out["name"] = object.name;
  ThreeJSExport(object.geometry,out,cache);
  ThreeJSExport(object.T,out["matrix"]);
}

void ThreeJSExportTransforms(const RigidObjectModel& object,AnyCollection& out)
{
  //out["uuid"] = MakeRandomUUID();
  out["name"] = object.name;
  ThreeJSExport(object.T,out["matrix"]);
}

void ThreeJSExport(const TerrainModel& terrain,AnyCollection& out,ThreeJSCache& cache)
{
  out["uuid"] = MakeRandomUUID();
  out["name"] = terrain.name;
  ThreeJSExport(terrain.geometry,out,cache);
  RigidTransform T; T.setIdentity();
  ThreeJSExport(T,out["matrix"]);
}

void ThreeJSExportTransforms(const TerrainModel& terrain,AnyCollection& out)
{
  //out["uuid"] = MakeRandomUUID();
  out["name"] = terrain.name;
  RigidTransform T; T.setIdentity();
  ThreeJSExport(T,out["matrix"]);
}


void ThreeJSExport(const ManagedGeometry& geom,AnyCollection& out,ThreeJSCache& cache)
{
  if(geom.Empty()) {
    out["type"] = "Group";
  }
  else {
    if(geom->type == Geometry::AnyGeometry3D::PointCloud)
      out["type"] = "Points";
    else
      out["type"] = "Mesh";
    ThreeJSExport(*geom,out["geometry"],cache);
    ThreeJSExport(*geom.Appearance(),*geom,out["material"],cache);
  }
}


void ThreeJSExportGeometry(const ManagedGeometry& geom,AnyCollection& out,ThreeJSCache& cache)
{
  if(!geom.Empty())
    ThreeJSExport(*geom,out,cache);
}
void ThreeJSExportAppearance(const ManagedGeometry& geom,AnyCollection& out,ThreeJSCache& cache)
{
  if(!geom.Empty())
    ThreeJSExport(*geom.Appearance(),*geom,out,cache);
}
void ThreeJSExport(const Meshing::TriMesh& mesh,AnyCollection& out)
{
  #if THREE_JS_OLD_VERSION
    out["type"] = "Geometry";
  #else
    out["type"] = "BufferGeometry";
    AnyCollection attributes;
  #endif //THREE_JS_OLD_VERSION
  AnyCollection vertices, faces;
  vertices.resize(mesh.verts.size()*3);
  for(size_t i=0;i<mesh.verts.size();i++) {
    vertices[int(i*3)] = float(mesh.verts[i].x);
    vertices[int(i*3+1)] = float(mesh.verts[i].y);
    vertices[int(i*3+2)] = float(mesh.verts[i].z);
  }
  #if THREE_JS_OLD_VERSION
    faces.resize(mesh.tris.size()*4);
    for(size_t i=0;i<mesh.tris.size();i++) {
      faces[int(i*4)] = 0;
      faces[int(i*4+1)] = mesh.tris[i].a;
      faces[int(i*4+2)] = mesh.tris[i].b;
      faces[int(i*4+3)] = mesh.tris[i].c;
    }
  #else
    faces.resize(mesh.tris.size()*3);
    for(size_t i=0;i<mesh.tris.size();i++) {
      faces[int(i*3)] = mesh.tris[i].a;
      faces[int(i*3+1)] = mesh.tris[i].b;
      faces[int(i*3+2)] = mesh.tris[i].c;
    }
  #endif //THREE_JS_OLD_VERSION
  
  #if THREE_JS_OLD_VERSION
    out["data"]["position"] = vertices;
    out["data"]["index"] = faces;
  #else
    /*
    AnyCollection normals;
    normals.resize(mesh.verts.size()*3);
    vector<Vector3> vnormals(mesh.verts.size(),Vector3(0.0));
    for(size_t i=0;i<mesh.tris.size();i++) {
      Vector3 n = mesh.TriangleNormal(i);
      vnormals[mesh.tris[i].a] += n;
      vnormals[mesh.tris[i].b] += n;
      vnormals[mesh.tris[i].c] += n;
    }
    int k=0;
    for(size_t i=0;i<vnormals.size();i++,k+=3) {
      Real len = vnormals[i].length();
      if(len > 0)
        vnormals[i] *= 1.0/len;
      normals[k] = int(vnormals[i].x*32767);
      normals[k+1] = int(vnormals[i].y*32767);
      normals[k+2] = int(vnormals[i].z*32767);
    }
    */

    AnyCollection vertarray,normalarray,facearray;
    vertarray["type"] = "Float32Array";
    vertarray["array"] = vertices;
    vertarray["itemSize"] = 3;
    /*
    normalarray["type"] = "Int16Array";
    normalarray["array"] = normals;
    normalarray["itemSize"] = 3;
    normalarray["normalized"] = true;
    */
    if(mesh.verts.size() > 0xffff)
      facearray["type"] = "Uint32Array";
    else
      facearray["type"] = "Uint16Array";
    facearray["array"] = faces;
    facearray["itemSize"] = 1;
    attributes["position"] = vertarray;
    //attributes["normal"] = normalarray;
    out["data"]["attributes"] = attributes;
    out["data"]["index"] = facearray;
  #endif //THREE_JS_OLD_VERSION
}
void ThreeJSExport(const Meshing::PointCloud3D& pc,AnyCollection& out)
{
  #if THREE_JS_OLD_VERSION
    out["type"] = "Geometry";
  #else
    out["type"] = "BufferGeometry";
    AnyCollection attributes;
  #endif
  vector<float> vertices;
  vertices.reserve(pc.points.size()*3);
  for(size_t i=0;i<pc.points.size();i++) {
    if(IsFinite(pc.points[i].x) && IsFinite(pc.points[i].y) && IsFinite(pc.points[i].z)) {
      vertices.push_back(float(pc.points[i].x));
      vertices.push_back(float(pc.points[i].y));
      vertices.push_back(float(pc.points[i].z));
    }
  }
  #if THREE_JS_OLD_VERSION
    out["data"]["position"] = AnyCollection(vertices);
  #else
    AnyCollection vertarray;
    vertarray["type"] = "Float32Array";
    vertarray["array"] = AnyCollection(vertices);
    vertarray["itemSize"] = 3;
    attributes["position"] = vertarray;
    out["data"]["position"] = vertarray;
  #endif //THREE_JS_OLD_VERSION
  vector<Vector4> rgba;
  if(pc.GetColors(rgba)) {
    AnyCollection colors;
    
    #if THREE_JS_OLD_VERSION
      colors.resize(vertices.size());
      int k=0;
      for(size_t i=0;i<pc.points.size();i++) {
        if(IsFinite(pc.points[i].x) && IsFinite(pc.points[i].y) && IsFinite(pc.points[i].z)) {
          colors[int(k*3)] = float(rgba[i].x);
          colors[int(k*3+1)] = float(rgba[i].y);
          colors[int(k*3+2)] = float(rgba[i].z);
          k ++;
        }
      }
      out["data"]["color"] = colors;
    #else
      //TODO: partially transparent point clouds?
      //colors.resize((vertices.size()/3)*4);
      colors.resize(vertices.size()/3);
      int k=0;
      for(size_t i=0;i<pc.points.size();i++) {
        if(IsFinite(pc.points[i].x) && IsFinite(pc.points[i].y) && IsFinite(pc.points[i].z)) {
          colors[int(k*3)] = int(rgba[i].x*255.0);
          colors[int(k*3+1)] = int(rgba[i].y*255.0);
          colors[int(k*3+2)] = int(rgba[i].z*255.0);
          k ++;
        }
      }
      AnyCollection colorarray;
      colorarray["type"] = "Uint8Array";
      colorarray["array"] = colors;
      //colorarray["itemSize"] = 4;
      colorarray["itemSize"] = 3;
      colorarray["normalized"] = true;
      attributes["color"] = colorarray;
    #endif //THREE_JS_OLD_VERSION
  }
  #if THREE_JS_OLD_VERSION
  #else
    out["data"]["attributes"] = attributes;
  #endif //THREE_JS_OLD_VERSION
}
void ThreeJSExport(const Geometry::AnyCollisionGeometry3D& geom,AnyCollection& out,ThreeJSCache& cache)
{
  if(geom.Empty()) {
    fprintf(stderr,"Unable to save empty geometry to three.js!\n");
    return;
  }
  if(cache.HasUUID(geom)) {
    //fprintf(stderr,"Cached geometry.\n");
    out = cache.GetUUID(geom);
    return;
  }
  if(geom.type == Geometry::AnyCollisionGeometry3D::Primitive) {
    const GeometricPrimitive3D& prim = geom.AsPrimitive();
    //save primitive as mesh
    out["uuid"] = cache.GetUUID(geom);
    if(prim.type == GeometricPrimitive3D::Sphere) {
      const Sphere3D* s = AnyCast_Raw<Sphere3D>(&prim.data);
      int numStacks = 20;
      if(s->radius < 0.05) numStacks = 6;
      else if(s->radius < 0.2) numStacks = 10;
      else if(s->radius > 1.0) numStacks = 40;
      int numSlices = numStacks*2;
      Meshing::TriMesh mesh;
      Meshing::MakeTriMesh(*s,numStacks,numStacks*2,mesh);
      ThreeJSExport(mesh,out);
    }
    else {
      AABB3D bb = geom.GetAABB();
      Real rad = (bb.bmax - bb.bmin).maxElement();
      int numDivs = 20;
      if(rad < 0.05) numDivs = 6;
      else if(rad < 0.2) numDivs = 10;
      else if(rad > 1.0) numDivs = 40;
      Meshing::TriMesh mesh;
      Meshing::MakeTriMesh(prim,mesh,numDivs);
      ThreeJSExport(mesh,out);
    }
  }
  else if(geom.type == Geometry::AnyCollisionGeometry3D::TriangleMesh) {
    //fprintf(stderr,"Triangle mesh geometry.\n");
    const Meshing::TriMesh& mesh = geom.AsTriangleMesh();
    out["uuid"] = cache.GetUUID(geom);
    ThreeJSExport(mesh,out);
  }
  else if(geom.type == Geometry::AnyCollisionGeometry3D::PointCloud) {
    //fprintf(stderr,"Triangle mesh geometry.\n");
    const Meshing::PointCloud3D& pc = geom.AsPointCloud();
    out["uuid"] = cache.GetUUID(geom);
    ThreeJSExport(pc,out);
  }
  else if(geom.type == Geometry::AnyCollisionGeometry3D::ConvexHull) {
    Geometry::AnyGeometry3D mesh;
    const AnyGeometry3D& ggeom = geom;
    if(!ggeom.Convert(Geometry::AnyGeometry3D::TriangleMesh,mesh))
      fprintf(stderr,"Unable to save geometries of type %s to three.js, problem exporting to TriangleMesh\n",geom.TypeName());
    else
      ThreeJSExport(mesh.AsTriangleMesh(),out);
  }
  else if(geom.type == Geometry::AnyCollisionGeometry3D::ImplicitSurface) {
    Geometry::AnyGeometry3D mesh;
    const AnyGeometry3D& ggeom = geom;
    if(!ggeom.Convert(Geometry::AnyGeometry3D::TriangleMesh,mesh))
      fprintf(stderr,"Unable to save geometries of type %s to three.js, problem exporting to TriangleMesh\n",geom.TypeName());
    else
      ThreeJSExport(mesh.AsTriangleMesh(),out);
  }
  else {
    //can't export files of that type
    fprintf(stderr,"Unable to save geometries of type %s to three.js!\n",geom.TypeName());
  }
}
void ThreeJSExport(const GLDraw::GeometryAppearance& app,const Geometry::AnyCollisionGeometry3D& geom,AnyCollection& out,ThreeJSCache& cache)
{
  if(cache.HasUUID(app)) { 
    //just a string
    out = cache.GetUUID(app);
    return;
  }
  else {
    //save material
    out["uuid"] = cache.GetUUID(app);
    if(geom.type != Geometry::AnyGeometry3D::PointCloud) {
      //out["type"] = "MeshStandardMaterial";
      out["type"] = "MeshPhongMaterial";
      out["flatShading"] = 1;
      int rgb = ToRGB32(app.faceColor);
      out["color"] = rgb;
      rgb = ToRGB32(app.emissiveColor);
      out["emissive"] = rgb;
      out["shininess"] = app.shininess;
      if(app.shininess > 0)
        rgb = ToRGB32(app.specularColor);
      else
        rgb = 0;
      out["specular"] = rgb;
      if(app.faceColor.rgba[3] != 1.0) {
        out["transparent"] = true;
        out["opacity"] = app.faceColor.rgba[3];
      }
    }
    else {
      out["type"] = "PointsMaterial";
      out["size"] = 0.01;
      if(geom.AsPointCloud().HasColor()) {
        out["vertexColors"] = true;
      }
      else {
        int rgb = ToRGB32(app.vertexColor);
        out["color"] = rgb;
        if(app.vertexColor.rgba[3] != 1.0) {
          out["transparent"] = true;
          out["opacity"] = app.vertexColor.rgba[3];
        }
      }
    }
  }
}

///Exports a world to a JSON object that can be used in the three.js editor.
///Contains metadata, geometries, materials, and object items.
void ThreeJSExport(const WorldModel& world,AnyCollection& out)
{
  ThreeJSCache cache;
  ThreeJSExport(world,out,cache);
}
///Exports a simulation to a JSON object that can be used in the three.js
///editor.  Contains metadata, geometries, materials, and object items.
///Draws the simulation world in natural color and the commanded
///world in transparent green.
void ThreeJSExport(Simulator& sim,AnyCollection& out)
{
  ThreeJSCache cache;
  ThreeJSExport(sim,out,cache);
}

//void ThreeJSExportTransforms(Simulator& sim,AnyCollection& out) //DJZ - not using cache for transforms, so don't need this function 
//{
//  ThreeJSCache cache;
//  ThreeJSExportTransforms(sim,out,cache);
//}

///Exports a robot to a JSON object that can be used in the three.js editor.
///The result is a hierarchical set of Mesh or Group objects.
void ThreeJSExport(const RobotModel& robot,AnyCollection& out)
{
  ThreeJSCache cache;
  ThreeJSExport(robot,out,cache);
}
///Exports a rigid object to a JSON object that can be used in the three.js
///editor. The result is a Mesh object (or Group if the geometry is empty).
void ThreeJSExport(const RigidObjectModel& object,AnyCollection& out)
{
  ThreeJSCache cache;
  ThreeJSExport(object,out,cache);
}
///Exports a rigid object to a JSON object that can be used in the three.js
///editor. The result is a Mesh object (or Group if the geometry is empty).
void ThreeJSExport(const TerrainModel& terrain,AnyCollection& out)
{
  ThreeJSCache cache;
  ThreeJSExport(terrain,out,cache);
}
///Exports geometry to a three.js scene Geometry instance.  The "uuid"
///element of the output gives the unique ID number that can be used elsewhere
void ThreeJSExport(const ManagedGeometry& geom,AnyCollection& out)
{
  ThreeJSCache cache;
  ThreeJSExport(geom,out,cache);
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
void ThreeJSExport(const GLDraw::GeometryAppearance& app,const Geometry::AnyCollisionGeometry3D& geom,AnyCollection& out)
{
  ThreeJSCache cache;
  ThreeJSExport(app,geom,out,cache);
}

} // namespace Klampt