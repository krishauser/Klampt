#include "XmlWorld.h"
#include "View/Texturizer.h"
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/utils/fileutils.h>
#include <fstream>

///defined in XmlODE.cpp
int SafeQueryFloat(TiXmlElement* e,const char* attr,double& out);

///reads a transformation matrix from attributes of an XML element
bool ReadTransform(TiXmlElement* e,RigidTransform& xform)
{
  Matrix3 R;
  R.setIdentity();
  Vector3 pos(Zero);
  bool transform=false;
  if(e->QueryValueAttribute("translation",&pos)==TIXML_SUCCESS) 
    transform=true;
  else if(e->QueryValueAttribute("position",&pos)==TIXML_SUCCESS) 
    transform=true;
  else pos.setZero();

  if(e->Attribute("rotateRPY")) {
    stringstream ss(e->Attribute("rotateRPY"));
    Vector3 xyz;
    EulerAngleRotation ea;
    ss>>xyz;
    //switch roll pitch yaw to ZYX order
    ea.set(xyz.z,xyz.y,xyz.x);
    ea.getMatrixZYX(R);
    transform=true;
  }
  if(e->Attribute("rotateMoment")) {
    stringstream ss(e->Attribute("rotateMoment"));
    MomentRotation m;
    ss>>m;
    m.getMatrix(R);
    transform=true;
  }
  if(e->Attribute("rotateX")) {
    Real val;
    stringstream ss(e->Attribute("rotateX"));
    ss >> val;
    Matrix3 temp;
    temp.setRotateX(val);
    R = temp*R;
    transform = true;
  }
  if(e->Attribute("rotateY")) {
    Real val;
    stringstream ss(e->Attribute("rotateY"));
    ss >> val;
    Matrix3 temp;
    temp.setRotateY(val);
    R = temp*R;
    transform = true;
  }
  if(e->Attribute("rotateZ")) {
    Real val;
    stringstream ss(e->Attribute("rotateZ"));
    ss >> val;
    Matrix3 temp;
    temp.setRotateZ(val);
    R = temp*R;
    transform = true;
  }
  if(transform) {
    xform.set(R,pos);
    return true;
  }
  xform.setIdentity();
  return false;
}

///reads a transformaton matrix from attributes of an XML element.  Allows scaling
bool ReadTransform(TiXmlElement* e,Matrix4& xform)
{
  //read the rigid transform
  bool transform = false;
  RigidTransform rigid;
  transform = ReadTransform(e,rigid);

  xform.set(rigid);
  bool scale = false;
  Real temp;
  Vector3 s(1,1,1);
  if(e->QueryValueAttribute("scale",&s) == TIXML_SUCCESS) {
    scale = true;
  }
  else if(e->QueryValueAttribute("scale",&temp) == TIXML_SUCCESS) {
    s.set(temp);
    scale  = true;
  }
  if(scale) {
    for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
	xform(i,j)*=s[j];
    return true;
  }
  return transform;
}


XmlRobot::XmlRobot(TiXmlElement* _element,string _path)
  :e(_element),path(_path)
{}

bool XmlRobot::GetRobot(Robot& robot)
{
  const char* fn = e->Attribute("file");
  if(!fn) {
    fprintf(stderr,"XmlRobot: element does not contain file attribute\n");
    return false;
  }
  string sfn = path + string(fn);
  if(!robot.Load(sfn.c_str())) {
    fprintf(stderr,"XmlRobot: error loading %s, trying absolute path\n",sfn.c_str());
    //try absolute path
    if(!robot.Load(fn)) {
      fprintf(stderr,"XmlRobot: error loading %s\n",sfn.c_str());
      return false;
    }
    else
      fprintf(stderr,"XmlRobot: absolute path succeeded\n");
  }
  Vector q;
  if(e->QueryValueAttribute("config",&q)==TIXML_SUCCESS) {
    if(q.n != robot.q.n) {
      fprintf(stderr,"%d!=%d\n",q.n,robot.q.n);
      fprintf(stderr,"XmlRobot: element's configuration doesnt match size with the robot\n");
      return false;
    }
    robot.UpdateConfig(q);
  }
  if(e->Attribute("configfile")!= NULL) {
    ifstream in (e->Attribute("configfile"),ios::in);
    if(!in) {
      fprintf(stderr,"XmlRobot: could not open robot config file %s\n",e->Attribute("configfile"));
      return false;
    }
    Vector q;
    in >> q;
    if(q.n != robot.q.n) {
      fprintf(stderr,"%d!=%d\n",q.n,robot.q.n);
      fprintf(stderr,"XmlRobot: configuration file %s vector  doesnt match size with the robot\n",e->Attribute("configfile"));
      return false;
    }
    robot.UpdateConfig(q);
  }

  TiXmlElement*es=e->FirstChildElement("sensors");
  if(es) {
    if(es->Attribute("file"))
      robot.properties["sensors"] = es->Attribute("file");
    else {
      stringstream ss;
      ss<<*es;
      robot.properties["sensors"] = ss.str();
    }
  }

  RigidTransform T;
  if(ReadTransform(e,T)) {
    for(size_t i=0;i<robot.links.size();i++)
      if(robot.parents[i] == -1) 
	robot.links[i].T0_Parent = T*robot.links[i].T0_Parent;
    robot.UpdateFrames();
  }

  return true;
}


XmlRigidObject::XmlRigidObject(TiXmlElement* _element,string _path)
  :e(_element),path(_path)
{}

bool XmlRigidObject::GetRigidObject(RigidObject& obj)
{
  obj.T.setIdentity();
  obj.mass=1.0;
  obj.com.setZero();
  obj.inertia.setZero();
  obj.kFriction = 0.5;
  obj.kRestitution = 0.5;
  obj.kStiffness=Inf;
  obj.kDamping=Inf;

  const char* fn = e->Attribute("file");
  if(fn) {
    string sfn = path + string(fn);
    if(!obj.Load(sfn.c_str())) {
      fprintf(stderr,"XmlRigidObject: error loading %s, trying absolute path\n",sfn.c_str());
      if(!obj.Load(fn)) {
	fprintf(stderr,"XmlRigidObject: error loading obj file %s\n",sfn.c_str());
	return false;
      }
      else
	fprintf(stderr,"XmlRigidObject: absolute path succeeded\n");
    }
  }
  TiXmlElement* geom=e->FirstChildElement("geometry");
  if(geom) {
    const char* fn = geom->Attribute("file");
    if(!fn)
      fn = geom->Attribute("mesh");
    if(fn) {
      obj.geomFile = fn;
      string sfn = path + obj.geomFile;
      if(!obj.LoadGeometry(sfn.c_str())) {
        fprintf(stderr,"XmlRigidObject: error loading geometry from %s\n",sfn.c_str());
        return false;
      }
    }
    Matrix4 xform;
    if(ReadTransform(geom,xform)) {
      obj.geometry.TransformGeometry(xform);
    }
    Real temp;
    if(geom->QueryValueAttribute("margin",&temp) == TIXML_SUCCESS) {
      obj.geometry->margin = temp;
    }
  }
  if(obj.geometry->Empty()) {
    fprintf(stderr,"XmlRigidObject: element does not contain geometry attribute\n");
    return false;
  }

  RigidTransform xform;
  if(ReadTransform(e,xform)) {
    obj.T = xform;
  }

  TiXmlElement* phys=e->FirstChildElement("physics");
  if(phys) {
    Real val;
    Vector3 com;
    Matrix3 inertia;
    if(phys->QueryValueAttribute("mass",&val)==TIXML_SUCCESS)
      obj.mass = val;
    if(phys->QueryValueAttribute("com",&com)==TIXML_SUCCESS)
      obj.com = com;
    if(phys->QueryValueAttribute("inertia",&inertia)==TIXML_SUCCESS)
      obj.inertia = inertia;

    int automass=0;
    if(phys->QueryValueAttribute("automass",&automass)==TIXML_SUCCESS)
      ;
    else automass=0;
    
    if(automass) {
      obj.SetMassFromGeometry(obj.mass);
    }
    
    SafeQueryFloat(phys,"kRestitution",obj.kRestitution);
    SafeQueryFloat(phys,"kFriction",obj.kFriction);
    SafeQueryFloat(phys,"kStiffness",obj.kStiffness);
    SafeQueryFloat(phys,"kDamping",obj.kDamping);
  }

  obj.UpdateGeometry();
  return true;
}

XmlTerrain::XmlTerrain(TiXmlElement* _element,string _path)
  :e(_element),path(_path)
{}

bool XmlTerrain::GetTerrain(Terrain& env)
{
  const char* fn = e->Attribute("file");
  if(!fn) {
    fprintf(stderr,"XmlTerrain: element does not contain file attribute\n");
    return false;
  }
  string sfn = path + string(fn);
  if(!env.Load(sfn.c_str())) {
    fprintf(stderr,"XmlTerrain: error loading %s, trying absolute path\n",sfn.c_str());
    if(!env.Load(fn)) {
      fprintf(stderr,"XmlTerrain: error loading %s\n",sfn.c_str());
      return false;
    }
    else
      fprintf(stderr,"XmlTerrain: absolute path succeeded\n");
  }

  Real kf;
  if(e->QueryValueAttribute("kFriction",&kf)==TIXML_SUCCESS) {
    env.SetUniformFriction(kf);
  }

  Matrix4 xform;
  if(ReadTransform(e,xform)) {
    env.geometry.TransformGeometry(xform);
  }
  Real margin;
  if(e->QueryValueAttribute("margin",&margin) == TIXML_SUCCESS) {
    env.geometry->margin = margin;
  }
  return true;
}

class XmlAppearance
{
 public:
  XmlAppearance(TiXmlElement* element,const string& _path) : e(element),path(_path) {}
  bool Get(ManagedGeometry& geom)
  {
    Texturizer tex;
    tex.texCoordAutoScale = false;
    geom.Appearance()->texWrap = true;
    if(e->Attribute("color")) {
      Vector3 rgb;
      stringstream ss(e->Attribute("color"));
      ss >> rgb;
      Real a=1.0;
      if(ss >> a) { }
      else a=1.0;
      tex.texture = "";
      geom.Appearance()->faceColor.set(rgb.x,rgb.y,rgb.z,a);
      geom.Appearance()->vertexColor.set(rgb.x,rgb.y,rgb.z,a);
    }
    if(e->Attribute("texture")) {
      tex.texture = e->Attribute("texture");
      if(0==strcmp(e->Attribute("texture"),"checker")) {
  tex.texCoords = Texturizer::XYTexCoords;
      }
      else if(0==strcmp(e->Attribute("texture"),"noise")) {
  tex.texCoords = Texturizer::XYTexCoords;
      }
      else if(0==strcmp(e->Attribute("texture"),"gradient")) {
  tex.texCoords = Texturizer::ZTexCoord;
  tex.texCoordAutoScale = true;
  geom.Appearance()->texWrap = false;
      }
      else if(0==strcmp(e->Attribute("texture"),"colorgradient")) {
  tex.texCoords = Texturizer::ZTexCoord;
  tex.texCoordAutoScale = true;
  geom.Appearance()->texWrap = false;
      }
      else {
  tex.texture = path+string(e->Attribute("texture"));
  tex.texCoordAutoScale = true;
      }
      if(e->Attribute("texture_projection")) {
        if(0==strcmp(e->Attribute("texture_projection"),"z")) {
    tex.texCoords = Texturizer::ZTexCoord;
        }
        else if(0==strcmp(e->Attribute("texture_projection"),"xy")) {
    tex.texCoords = Texturizer::XYTexCoords;
        }
        else if(0==strcmp(e->Attribute("texture_projection"),"conformal")) {
    tex.texCoords = Texturizer::ParameterizedTexCoord;
        }
        else {
    printf("Unsupported value for texture_projection: %s\n",e->Attribute("texture_projection"));
    tex.texCoords = Texturizer::XYTexCoords;
        }
      }
      tex.Set(geom);
    }
    return true;
  }
  bool Get(Terrain& terrain)
  {
    terrain.geometry.SetUniqueAppearance();
    terrain.geometry.Appearance()->faceColor.set(0.8,0.6,0.2);
    Texturizer tex;
    //checker by default
    tex.texture = "checker";
    tex.texCoordAutoScale = false;
    tex.Set(terrain.geometry);
    return Get(terrain.geometry);
  }

  TiXmlElement* e;
  string path;
};

void WriteAppearance(ManagedGeometry& geom,FILE* out,int indent=0)
{
  float* rgba = geom.Appearance()->faceColor;
  for(int i=0;i<indent;i++)
    fprintf(out," ");
  fprintf(out,"<display color=\"%f %f %f %f\"",rgba[0],rgba[1],rgba[2],rgba[3]);
  //TODO: any other display stuff?
  fprintf(out,"/>\n");
}


XmlWorld::XmlWorld()
  :elem(NULL)
{}

bool XmlWorld::Load(const string& fn)
{
  if(!doc.LoadFile(fn.c_str())) return false;
  return Load(doc.RootElement(),GetFilePath(fn));
}

bool XmlWorld::Load(TiXmlElement* e,string _path)
{
  elem = e;
  path = _path;
  if(0 != strcmp(e->Value(),"world")) return false;
  return true;
}

bool XmlWorld::GetWorld(RobotWorld& world)
{
  if(!elem) return false;
  string robot="robot";
  string object="rigidObject";
  string terrain="terrain";
  string display="display";
  string appearance="appearance";
  string goal="goal";
  TiXmlElement* e;
  //parse display
  e = GetElement(display);
  if(!e) e = GetElement(appearance);
  if(e) {
    Vector4 rgba;
    if(e->QueryValueAttribute("background",&rgba)==TIXML_SUCCESS)
      world.background.set(rgba.x,rgba.y,rgba.z,rgba.w);
    //TODO: lights?
  }

  //parse goal
  e = GetElement(goal);
  goalCount = 0;
  while(e) {
	  if(e->QueryValueAttribute("position",&goals[goalCount])==TIXML_SUCCESS)
		  goalCount++;
	  e=e->NextSiblingElement(goal);
  }
  //parse robots
  e = GetElement(robot);
  while(e) {
    const char* name = e->Attribute("name");
    string sname = "Robot";
    if(name) sname=name;
    Robot* r = new Robot;
    if(!XmlRobot(e,path).GetRobot(*r)) {
      printf("XmlWorld: Unable to load robot %s\n",sname.c_str());
      delete r;
      return false;
    }
    int i = world.AddRobot(sname,r);
    e = e->NextSiblingElement(robot);
  }
  //parse objects
  e = GetElement(object);
  while(e) {
    const char* name = e->Attribute("name");
    string sname = "Object";
    if(name) sname=name;
    RigidObject* o = new RigidObject;
    if(!XmlRigidObject(e,path).GetRigidObject(*o)) {
      printf("XmlWorld: Unable to load rigid object %s\n",sname.c_str());
      delete o;
      return false;
    }
    int i = world.AddRigidObject(sname,o);
    TiXmlElement* d = e->FirstChildElement(display);
    if(!d) d = e->FirstChildElement(appearance);
    if(d) {
      if(!XmlAppearance(d,path).Get(world.rigidObjects[i]->geometry)) {
	printf("XmlWorld: Warning, unable to load geometry appearance %s\n",sname.c_str());
      }
    }
    e = e->NextSiblingElement(object);
  }
  //parse objects
  e = GetElement(terrain);
  while(e) {
    const char* name = e->Attribute("name");
    string sname = "Terrain";
    Terrain* t = new Terrain;
    if(!XmlTerrain(e,path).GetTerrain(*t)) {
      printf("XmlWorld: Unable to load terrain %s\n",sname.c_str());
      delete t;
      return false;
    }
    if(name) sname=name;
    else {
      stringstream ss;
      ss<<"Terrain"<<world.terrains.size();
      sname = ss.str();
    }
    int i = world.AddTerrain(sname,t);
    TiXmlElement* d = e->FirstChildElement(display);
    if(!d) d = e->FirstChildElement(appearance);
    if(d) {
      if(!XmlAppearance(d,path).Get(*world.terrains[i])) {
	printf("XmlWorld: Warning, unable to load terrain appearance %s\n",sname.c_str());
      }
    }
    e = e->NextSiblingElement(terrain);
  }
  return true;
}

TiXmlElement* XmlWorld::GetElement(const string& name)
{
  if(!elem) return NULL;
  return elem->FirstChildElement(name);
}

TiXmlElement* XmlWorld::GetElement(const string& name,int index)
{
  TiXmlElement* e=elem->FirstChildElement(name);
  while(index > 0) {
    if(!e) return NULL;
    index--;
    e = e->NextSiblingElement(name);
  }
  return e;
}

string GetRelativeFilename(const std::string& filename,const std::string& path)
{
  //TODO: this doesn't work on windows
  if(filename[0] == '/' && path[0] != '/') {
    string cwd = FileUtils::GetWorkingDirectory();
    return GetRelativeFilename(filename,cwd+"/"+path);
  }
  if(filename[0] != '/' && path[0] == '/') {
    string cwd = FileUtils::GetWorkingDirectory();
    return GetRelativeFilename(cwd+"/"+filename,path);
  }
  std::vector<std::string> fcomponents,pcomponents;

  SplitPath(filename,fcomponents);
  SplitPath(path,pcomponents);
  size_t start=0;
  for(size_t i=0;i<Min(fcomponents.size(),pcomponents.size());i++) {
    if(fcomponents[i] != pcomponents[i]) break;
    start=i+1;
  }
  //path shared from start-1 and before. The relative path starts at start
  std::vector<std::string> res;
  for(size_t i=start;i<pcomponents.size();i++)
    res.push_back("..");
  for(size_t i=start;i<fcomponents.size();i++)
    res.push_back(fcomponents[i]);
  return JoinPath(res);
}

const char* DefaultFileExtension(const Geometry::AnyCollisionGeometry3D& geom)
{
  if(geom.type == Geometry::AnyGeometry3D::Primitive)
    return ".geom";
  else if(geom.type == Geometry::AnyGeometry3D::TriangleMesh)
    return ".off";
  else if(geom.type == Geometry::AnyGeometry3D::PointCloud)
    return ".pcd";
  else if(geom.type == Geometry::AnyGeometry3D::ImplicitSurface)
    return ".vol";
  else
    return ".unknown";
}

bool XmlWorld::Save(RobotWorld& world,const string& fn,string itempath)
{
  string filepath = GetFilePath(fn);
  string filename = GetFileName(fn);
  string relpath = itempath;
  if(itempath.length() == 0) {
    itempath = fn;
    StripExtension(itempath);
    relpath = filename;
    StripExtension(relpath);
    relpath = relpath + "/";
  }
  printf("World::Save(): Saving world item files to %s\n",itempath.c_str());
  if(!FileUtils::IsDirectory(itempath.c_str())) {
    if(!FileUtils::MakeDirectoryRecursive(itempath.c_str())) {
      printf("World::Save(): could not make directory %s for world items\n",itempath.c_str());
      return false;
    }
  }
  itempath = itempath + "/";

  //start saving world elements

  //get unique names
  vector<string> robotFileNames(world.robots.size());
  vector<string> objectFileNames(world.rigidObjects.size());
  vector<string> terrainFileNames(world.terrains.size());
  map<string,int> names;
  for(size_t i=0;i<world.robots.size();i++) {
    string rfn = world.robots[i]->name + ".rob";
    if(names.count(world.robots[i]->name) != 0) {
      names[world.robots[i]->name] += 1;
      char buf[32];
      sprintf(buf,"_%d",names[world.robots[i]->name]);
      rfn = world.robots[i]->name + buf + ".rob";
    }
    else names[world.robots[i]->name] = 0;
    robotFileNames[i] = rfn;
  }
  names.clear();
  for(size_t i=0;i<world.rigidObjects.size();i++) {
    string rfn = world.rigidObjects[i]->name + ".obj";
    if(names.count(world.rigidObjects[i]->name) != 0) {
      names[world.rigidObjects[i]->name] += 1;
      char buf[32];
      sprintf(buf,"_%d",names[world.rigidObjects[i]->name]);
      rfn = world.rigidObjects[i]->name + buf + ".obj";
    }
    else names[world.rigidObjects[i]->name] = 0;
    objectFileNames[i] = rfn;
  }
  names.clear();
  for(size_t i=0;i<world.terrains.size();i++) {
    string rfn = world.terrains[i]->name + ".env";
    if(names.count(world.terrains[i]->name) != 0) {
      names[world.terrains[i]->name] += 1;
      char buf[32];
      sprintf(buf,"_%d",names[world.terrains[i]->name]);
      rfn = world.terrains[i]->name + buf + ".env";
    }
    else names[world.terrains[i]->name] = 0;
    terrainFileNames[i] = rfn;
  }

  bool geomErrors = false;
  //first, deal with geometries and relative paths to old cached geomes
  for(size_t i=0;i<world.robots.size();i++) {
    for(size_t j=0;j<world.robots[i]->links.size();j++) {
      if(world.robots[i]->geomManagers[j].Empty()) continue;
      if(world.robots[i]->geomManagers[j].IsCached()) {
        //modify geomFiles[j] to point to path of geomfile *relative* to where the robot will be saved
        const string& geomfile = world.robots[i]->geomManagers[j].CachedFilename();
        string relfile = GetRelativeFilename(geomfile,itempath);
        world.robots[i]->geomFiles[j] = relfile;
      }
      else {
        //save the geometry to the item path too
        string geomdir = robotFileNames[i];
        StripExtension(geomdir);
        if(!FileUtils::IsDirectory(((relpath)+geomdir).c_str()))
          FileUtils::MakeDirectory((relpath+geomdir).c_str());
        world.robots[i]->geomFiles[j] = geomdir + "/" + FileUtils::SafeFileName(world.robots[i]->linkNames[j]) + DefaultFileExtension(*world.robots[i]->geomManagers[j]);
        printf("  Saving modified geometry for link %s to %s\n",world.robots[i]->linkNames[j].c_str(),(relpath + world.robots[i]->geomFiles[j]).c_str());
        world.robots[i]->geomManagers[j]->Save((relpath + world.robots[i]->geomFiles[j]).c_str());
      }
    }
  }
  for(size_t i=0;i<world.rigidObjects.size();i++) {
    if(world.rigidObjects[i]->geometry.Empty()) continue;
    if(world.rigidObjects[i]->geometry.IsCached()) {
      //modify geomFile to point to path of geomfile *relative* to where the robot will be saved
      const string& geomfile = world.rigidObjects[i]->geometry.CachedFilename();
      string relfile = GetRelativeFilename(geomfile,itempath);
      world.rigidObjects[i]->geomFile = relfile;
    }
    else {
      //save the geometry to the item path too
      string geomdir = objectFileNames[i];
      StripExtension(geomdir);
      if(!FileUtils::IsDirectory((relpath+geomdir).c_str()))
        FileUtils::MakeDirectory((relpath+geomdir).c_str());
      world.rigidObjects[i]->geomFile = geomdir + "/" + FileUtils::SafeFileName(world.rigidObjects[i]->name) + DefaultFileExtension(*world.rigidObjects[i]->geometry);
      printf("  Saving modified geometry for rigid object %s to %s\n",world.rigidObjects[i]->name.c_str(),(relpath + world.rigidObjects[i]->geomFile).c_str());
      world.rigidObjects[i]->geometry->Save((relpath + world.rigidObjects[i]->geomFile).c_str());
    }
  }
  for(size_t i=0;i<world.terrains.size();i++) {
    if(world.terrains[i]->geometry.Empty()) continue;
    if(world.terrains[i]->geometry.IsCached()) {
      //modify geomFile to point to path of geomfile *relative* to where the robot will be saved
      const string& geomfile = world.terrains[i]->geometry.CachedFilename();
      string relfile = GetRelativeFilename(geomfile,itempath);
      world.terrains[i]->geomFile = relfile;
    }
    else {
      //save the geometry to the item path too
      string geomdir = terrainFileNames[i];
      StripExtension(geomdir);
      if(!FileUtils::IsDirectory((relpath+geomdir).c_str()))
        FileUtils::MakeDirectory((relpath+geomdir).c_str());
      world.terrains[i]->geomFile = geomdir + "/" + FileUtils::SafeFileName(world.terrains[i]->name) + DefaultFileExtension(*world.terrains[i]->geometry);
      printf("  Saving modified geometry for terrain %s to %s\n",world.terrains[i]->name.c_str(),(relpath + world.terrains[i]->geomFile).c_str());
      world.terrains[i]->geometry->Save((relpath + world.terrains[i]->geomFile).c_str());
    }
  }

  for(size_t i=0;i<world.robots.size();i++) {
    printf("  Saving robot to %s\n",(itempath+robotFileNames[i]).c_str());
    if(!world.robots[i]->Save((itempath + robotFileNames[i]).c_str())) {
      printf("  Robot saving failed.\n");
      return false;
    }
  }
  for(size_t i=0;i<world.rigidObjects.size();i++) {
    printf("  Saving rigid object to %s\n",(itempath+objectFileNames[i]).c_str());
    if(!world.rigidObjects[i]->Save((itempath + objectFileNames[i]).c_str())) {
      printf("  Rigid object saving failed.\n");
      return false;
    }
  }
  for(size_t i=0;i<world.terrains.size();i++) {
    printf("  Saving terrain to %s\n",(itempath+terrainFileNames[i]).c_str());
    if(!world.terrains[i]->Save((itempath + terrainFileNames[i]).c_str())) {
      printf("  Terrain saving failed.\n");
      return false;
    }
  }
  FILE* out = fopen(fn.c_str(),"w");
  if(!out) return false;
  fprintf(out,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<world>\n");
  fprintf(out,"  <display background=\"%f %f %f %f\" />\n",world.background.rgba[0],world.background.rgba[1],world.background.rgba[2],world.background.rgba[3]);
  for(size_t i=0;i<world.robots.size();i++) {
    fprintf(out,"  <robot name=\"%s\" file=\"%s\" />\n",world.robots[i]->name.c_str(),(relpath+robotFileNames[i]).c_str());
  }
  for(size_t i=0;i<world.rigidObjects.size();i++) {
    fprintf(out,"  <rigidObject name=\"%s\" file=\"%s\" ",world.rigidObjects[i]->name.c_str(),(relpath+objectFileNames[i]).c_str());
    if(world.rigidObjects[i]->geometry->margin != 0)
      fprintf(out,"margin=\"%f\" ",world.rigidObjects[i]->geometry->margin);
    fprintf(out,">\n");
    WriteAppearance(world.rigidObjects[i]->geometry,out,4);
    fprintf(out,"  </rigidObject>\n");
  }
  for(size_t i=0;i<world.terrains.size();i++) {
    fprintf(out,"  <terrain name=\"%s\" file=\"%s\" ",world.terrains[i]->name.c_str(),(relpath+terrainFileNames[i]).c_str());
    if(world.terrains[i]->geometry->margin != 0)
      fprintf(out,"margin=\"%f\" ",world.terrains[i]->geometry->margin);
    fprintf(out,">\n");
    WriteAppearance(world.terrains[i]->geometry,out,4);
    fprintf(out,"  </terrain>\n");
  }
  fprintf(out,"</world>\n");
  fclose(out);
  if(geomErrors)
    printf("World::Save(): warning: geometry files may not be saved properly\n");
  return true; 
}