#include "XmlWorld.h"
#include "View/Texturizer.h"
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/utils/fileutils.h>
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/Logger.h>
#include <fstream>

DECLARE_LOGGER(XmlParser);

///defined in XmlODE.cpp
int SafeQueryFloat(TiXmlElement* e,const char* attr,double& out);

///returns a resolved path for a file referred to within another file.
///path is the path containing the referencing file, and fn is an internal file reference
string ResolveFileReference(const string& path,const string& fn)
{
  //cout<<"Trying to resolve file "<<fn<<" in context of path "<<path<<endl;
  string res;
  if(fn.empty()) return "";
  if(fn[0]=='/') {
    //root of filesystem
    //cout<<"  Root of filesystem"<<endl;
    return fn; 
  }
  if(fn.find("://") != string::npos) {  //URL
    //cout<<"  File is URL"<<endl;
    return fn;
  }
  if(path.find("://") != string::npos) { //source is a URL
    string relfile = ReducePath(JoinPath(path,fn));
    //cout<<"  Path is URL"<<endl;
    return relfile;
  }
  string relpath = JoinPath(path,fn);
  if(FileUtils::Exists(relpath.c_str())) {
    //cout<<"  Relative path "<<relpath<<" exists"<<endl;
    return relpath;
  }
  else {
    //if(!FileUtils::Exists(fn.c_str())) 
    //  cout<<"  Neither relative path "<<relpath<<" nor absolute path "<<fn<<" exist"<<endl;
  }
  return fn;
}

///Resolves URLs to a local file, if needed. "" is returned on failure.
string MakeURLLocal(const string& url,const char* url_resolution_path="klampt_downloads")
{
  if(url.find("://") == string::npos) return url;
  //use CURL and download to a local directory
  string tempfile = GetFileName(url);
  if(url_resolution_path) {
    FileUtils::MakeDirectory(url_resolution_path);
    tempfile = JoinPath(url_resolution_path,tempfile);
  }
  //LOG4CXX_INFO(GET_LOGGER(XmlParser),"Downloading "<<url<<" to "<<tempfile<<"...");
  if(!GetURLDownload(url.c_str(),tempfile.c_str())) {
    //LOG4CXX_INFO(GET_LOGGER(XmlParser),"Download of "<<url<<" failed.");
    return "";
  }
  return tempfile;
}

template <class T>
bool LoadObjectFile(T& obj,const string& path,const string& fn,const char* type)
{
  string sfn = ResolveFileReference(path,fn);
  if(sfn.empty()) {
    LOG4CXX_ERROR(GET_LOGGER(XmlParser),type<<": invalid file reference "<<fn);
    return false;
  }
  if(obj.Load(sfn.c_str())) 
    return true;
  LOG4CXX_ERROR(GET_LOGGER(XmlParser),type<<": error loading from file "<<sfn[0]);
  return false;
}

template <class T>
bool LoadObjectGeometryFile(T& obj,const string& path,const string& fn,const char* type)
{
  string sfn = ResolveFileReference(path,fn);
  if(sfn.empty()) {
    LOG4CXX_ERROR(GET_LOGGER(XmlParser),type<<": invalid file reference "<<fn);
    return false;
  }
  if(obj.LoadGeometry(sfn.c_str())) 
    return true;
  LOG4CXX_ERROR(GET_LOGGER(XmlParser),type<<": error loading from file "<<sfn[0]);
  return false;
}


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
    LOG4CXX_ERROR(GET_LOGGER(XmlParser),"XmlRobot: element does not contain file attribute");
    return false;
  }

  if(!LoadObjectFile(robot,path,fn,"XmlRobot")) return false;

  Vector q;
  if(e->QueryValueAttribute("config",&q)==TIXML_SUCCESS) {
    if(q.n != robot.q.n) {
      LOG4CXX_ERROR(GET_LOGGER(XmlParser),"XmlRobot: element's configuration doesnt match size with the robot");
      LOG4CXX_ERROR(GET_LOGGER(XmlParser),q.n<<"!="<<robot.q.n);
      return false;
    }
    robot.UpdateConfig(q);
  }
  if(e->Attribute("configfile")!= NULL) {
    const char* fn = e->Attribute("configfile");
    Vector q;
    auto str = ResolveFileReference(path,fn);
    auto localfile = MakeURLLocal(str);
    if(!localfile.empty()) {
      ifstream in (localfile.c_str(),ios::in);
      if(in) {
        in >> q;
      }
      else {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"XmlRobot: could not open robot config file "<<fn);
        return false;
      }
    }
    else {
      LOG4CXX_ERROR(GET_LOGGER(XmlParser),"XmlRobot: could not open robot config file "<<fn);
      return false;
    }
    if(q.n != robot.q.n) {
      LOG4CXX_ERROR(GET_LOGGER(XmlParser),"XmlRobot: configuration file "<<fn<<" vector doesnt match size with the robot");
      LOG4CXX_ERROR(GET_LOGGER(XmlParser),q.n<<"!="<<robot.q.n);
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
    if(!LoadObjectFile(obj,path,fn,"XmlRigidObject")) return false;
  }

  TiXmlElement* geom=e->FirstChildElement("geometry");
  if(geom) {
    const char* fn = geom->Attribute("file");
    if(!fn)
      fn = geom->Attribute("mesh");
    if(fn) {
      if(!LoadObjectGeometryFile(obj,path,fn,"XmlRigidObject")) return false;
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
    LOG4CXX_ERROR(GET_LOGGER(XmlParser),"XmlRigidObject: element does not contain geometry attribute");
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
    Real surfaceFraction=1.0;
    if(phys->QueryValueAttribute("automass",&automass)==TIXML_SUCCESS) {
      if(phys->QueryValueAttribute("automassSurfaceFraction",&surfaceFraction)==TIXML_SUCCESS)
        ;
    }
    else automass=0;
    
    if(automass) {
      obj.SetMassFromGeometry(obj.mass,surfaceFraction);
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
    LOG4CXX_ERROR(GET_LOGGER(XmlParser),"XmlTerrain: element does not contain file attribute");
    return false;
  }
  if(!LoadObjectFile(env,path,fn,"XmlTerrain")) return false;

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
    geom.SetUniqueAppearance();
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
    if(e->Attribute("vertexColor")) {
      Vector3 rgb;
      stringstream ss(e->Attribute("vertexColor"));
      ss >> rgb;
      Real a=1.0;
      if(ss >> a) { }
      else a=1.0;
      geom.Appearance()->vertexColor.set(rgb.x,rgb.y,rgb.z,a);
      if(a == 0)
        geom.Appearance()->drawVertices = false;
      else
        geom.Appearance()->drawVertices = true;
    }
    if(e->Attribute("edgeColor")) {
      Vector3 rgb;
      stringstream ss(e->Attribute("edgeColor"));
      ss >> rgb;
      Real a=1.0;
      if(ss >> a) { }
      else a=1.0;
      geom.Appearance()->edgeColor.set(rgb.x,rgb.y,rgb.z,a);
      if(a == 0)
        geom.Appearance()->drawEdges = false;
      else
        geom.Appearance()->drawEdges = true;
    }
    if(e->Attribute("faceColor")) {
      Vector3 rgb;
      stringstream ss(e->Attribute("faceColor"));
      ss >> rgb;
      Real a=1.0;
      if(ss >> a) { }
      else a=1.0;
      tex.texture = "";
      geom.Appearance()->faceColor.set(rgb.x,rgb.y,rgb.z,a);
      if(a == 0)
        geom.Appearance()->drawFaces = false;
      else
        geom.Appearance()->drawFaces = true;
    }
    if(e->Attribute("emissiveColor")) {
      Vector3 rgb;
      stringstream ss(e->Attribute("emissiveColor"));
      ss >> rgb;
      Real a=1.0;
      if(ss >> a) { }
      else a=1.0;
      geom.Appearance()->emissiveColor.set(rgb.x,rgb.y,rgb.z,a);
    }
    if(e->Attribute("shininess")) {
      stringstream ss(e->Attribute("shininess"));
      ss >> geom.Appearance()->shininess;
    }
    if(e->Attribute("specularColor")) {
      Vector3 rgb;
      stringstream ss(e->Attribute("specularColor"));
      ss >> rgb;
      Real a=1.0;
      if(ss >> a) { }
      else a=1.0;
      geom.Appearance()->specularColor.set(rgb.x,rgb.y,rgb.z,a);
    }
    if(e->Attribute("vertexSize")) {
      Real vertexSize;
      stringstream ss(e->Attribute("vertexSize"));
      ss >> vertexSize;
      geom.Appearance()->vertexSize = vertexSize;
      geom.Appearance()->drawVertices = true;
    }
    if(e->Attribute("pointSize")) {
      Real vertexSize;
      stringstream ss(e->Attribute("pointSize"));
      ss >> vertexSize;
      geom.Appearance()->vertexSize = vertexSize;
      geom.Appearance()->drawVertices = true;
    }
    if(e->Attribute("edgeSize")) {
      Real edgeSize;
      stringstream ss(e->Attribute("edgeSize"));
      ss >> edgeSize;
      geom.Appearance()->edgeSize = edgeSize;
      geom.Appearance()->drawEdges = true;
    }
    if(e->Attribute("silhouette")) {
      Real radius;
      Vector3 rgb;
      Real a=1.0;
      stringstream ss(e->Attribute("silhouette"));
      ss>>radius;
      geom.Appearance()->silhouetteRadius = radius;
      if(ss >> rgb) {
        if(ss >> a) { }
        else a=1.0;
        geom.Appearance()->silhouetteColor.set(rgb.x,rgb.y,rgb.z,a);
      }
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
          LOG4CXX_WARN(GET_LOGGER(XmlParser),"Unsupported value for texture_projection: "<<e->Attribute("texture_projection"));
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
    terrain.geometry.Appearance()->faceColor.set(0.8f,0.6f,0.2f);
    Texturizer tex;
    //checker by default
    tex.texture = "checker";
    tex.texCoordAutoScale = false;
    tex.Set(terrain.geometry);
    return Get(terrain.geometry);
  }
  bool Get(Robot& robot) {
    const char* link = e->Attribute("link");
    if(link == NULL) {
      //apply to all geometries
      for(size_t j=0;j<robot.links.size();j++) {
        if(!Get(robot.geomManagers[j]))
          return false;
      }
      return true;
    }
    int linkindex = robot.LinkIndex(link);
    if(linkindex < 0) {
      stringstream ss;
      if(ss>>linkindex) {
        if(linkindex < 0 || linkindex >= (int)robot.links.size()) {
          LOG4CXX_ERROR(GET_LOGGER(XmlParser),"XmlWorld: Warning, invalid robot link specified "<<link);
          return false;
        }
      }
      else {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"XmlWorld: Warning, invalid robot link specified "<<link);
        return false;
      }
    }
    return Get(robot.geomManagers[linkindex]);
  }

  TiXmlElement* e;
  string path;
};

void WriteAppearance(ManagedGeometry& geom,FILE* out,int indent=0,const char* link=NULL)
{
  GLDraw::GeometryAppearance* app=geom.Appearance().get();
  for(int i=0;i<indent;i++)
    fprintf(out," ");
  fprintf(out,"<display");
  if(link) 
    fprintf(out," link=\"%s\"",link);
  float* rgba = app->faceColor;
  if(rgba[3] != 0.0 && app->drawFaces)
    fprintf(out," faceColor=\"%f %f %f %f\"",rgba[0],rgba[1],rgba[2],rgba[3]);
  rgba = app->edgeColor;
  if(rgba[3] != 0.0 && app->drawEdges)
    fprintf(out," edgeColor=\"%f %f %f %f\" edgeSize=\"%f\"",rgba[0],rgba[1],rgba[2],rgba[3],app->edgeSize);
  rgba = app->vertexColor;
  if(rgba[3] != 0.0 && app->drawVertices)
    fprintf(out," vertexColor=\"%f %f %f %f\" vertexSize=\"%f\"",rgba[0],rgba[1],rgba[2],rgba[3],app->vertexSize);
  rgba = app->silhouetteColor;
  if(app->drawFaces && app->silhouetteRadius > 0 && rgba[3] != 0.0)
    fprintf(out," silhouette=\"%f %f %f %f %f\"",app->silhouetteRadius,rgba[0],rgba[1],rgba[2],rgba[3]);
  //TODO: any other display stuff?
  fprintf(out,"/>\n");
}


XmlWorld::XmlWorld()
  :elem(NULL)
{}

bool XmlWorld::Load(const string& fn)
{
  string localfile = MakeURLLocal(fn);
  if(localfile.empty()) return false;
  if(!doc.LoadFile(localfile.c_str())) return false;
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
      LOG4CXX_ERROR(GET_LOGGER(XmlParser),"XmlWorld: Unable to load robot "<<sname);
      delete r;
      return false;
    }
    int i = world.AddRobot(sname,r);
    //parse appearances
    TiXmlElement* d = e->FirstChildElement(display);
    while(d) {
      if(!XmlAppearance(d,path).Get(*world.robots[i])) {
        d = d->NextSiblingElement();
        continue;
      }
      d = d->NextSiblingElement();
    }
    e->FirstChildElement(appearance);
    while(d) {
      if(!XmlAppearance(d,path).Get(*world.robots[i])) {
        d = d->NextSiblingElement();
        continue;
      }
      d = d->NextSiblingElement();
    }
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
      LOG4CXX_ERROR(GET_LOGGER(XmlParser),"XmlWorld: Unable to load rigid object "<<sname);
      delete o;
      return false;
    }
    int i = world.AddRigidObject(sname,o);
    TiXmlElement* d = e->FirstChildElement(display);
    if(!d) d = e->FirstChildElement(appearance);
    if(d) {
      if(!XmlAppearance(d,path).Get(world.rigidObjects[i]->geometry)) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"XmlWorld: Warning, unable to load geometry appearance "<<sname);
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
      LOG4CXX_WARN(GET_LOGGER(XmlParser),"XmlWorld: Unable to load terrain "<<sname);
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
        LOG4CXX_WARN(GET_LOGGER(XmlParser),"XmlWorld: Warning, unable to load terrain appearance "<<sname);
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

bool IsAbsolutePath(const std::string& path)
{
  #ifdef _WIN32
    return path.length() >= 2 && path[1] == ':';
  #else
    return path.length() >= 1 && path[0] == '/';
  #endif //_WIN32
}

string GetRelativeFilename(const std::string& filename,const std::string& path)
{
  if(IsAbsolutePath(filename) && !IsAbsolutePath(path)) {
    string cwd = FileUtils::GetWorkingDirectory();
    return GetRelativeFilename(filename,cwd+"/"+path);
  }
  if(!IsAbsolutePath(filename) && IsAbsolutePath(path)) {
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
  if(start==0 && IsAbsolutePath(filename))
    return filename;
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
  LOG4CXX_INFO(GET_LOGGER(XmlParser),"World::Save(): Saving world item files to "<<itempath);
  if(!FileUtils::IsDirectory(itempath.c_str())) {
    if(!FileUtils::MakeDirectoryRecursive(itempath.c_str())) {
      LOG4CXX_ERROR(GET_LOGGER(XmlParser),"World::Save(): could not make directory "<<itempath<<" for world items");
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
      snprintf(buf,32,"_%d",names[world.robots[i]->name]);
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
      snprintf(buf,32,"_%d",names[world.rigidObjects[i]->name]);
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
      snprintf(buf,32,"_%d",names[world.terrains[i]->name]);
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
      if(world.robots[i]->geomManagers[j].IsOriginal()) {
        //modify geomFiles[j] to point to path of geomfile *relative* to where the robot will be saved
        const string& geomfile = world.robots[i]->geomManagers[j].CachedFilename();
        string relfile = GetRelativeFilename(geomfile,itempath);
        LOG4CXX_INFO(GET_LOGGER(XmlParser),"  Saving reference to original geometry for link "<<world.robots[i]->linkNames[j]<<" to "<<relfile);
        world.robots[i]->geomFiles[j] = relfile;
      }
      else {
        //save the geometry to the item path too
        string geomdir = robotFileNames[i];
        StripExtension(geomdir);
        if(!FileUtils::IsDirectory(((relpath)+geomdir).c_str()))
          if(!FileUtils::MakeDirectory((relpath+geomdir).c_str())) {
            LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Unable to make directory "<<relpath+geomdir);
          }
        world.robots[i]->geomFiles[j] = geomdir + "/" + FileUtils::SafeFileName(world.robots[i]->linkNames[j]) + DefaultFileExtension(*world.robots[i]->geomManagers[j]);
        LOG4CXX_INFO(GET_LOGGER(XmlParser),"  Saving modified geometry for link "<<world.robots[i]->linkNames[j]<<" to "<<relpath + world.robots[i]->geomFiles[j]);
        world.robots[i]->geomManagers[j]->Save((relpath + world.robots[i]->geomFiles[j]).c_str());
      }
    }
  }
  for(size_t i=0;i<world.rigidObjects.size();i++) {
    if(world.rigidObjects[i]->geometry.Empty()) continue;
    if(world.rigidObjects[i]->geometry.IsOriginal()) {
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
        if(!FileUtils::MakeDirectory((relpath+geomdir).c_str())) {
          LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Unable to make directory "<<relpath+geomdir);
        }
      world.rigidObjects[i]->geomFile = geomdir + "/" + FileUtils::SafeFileName(world.rigidObjects[i]->name) + DefaultFileExtension(*world.rigidObjects[i]->geometry);
      LOG4CXX_INFO(GET_LOGGER(XmlParser),"  Saving modified geometry for rigid object "<<world.rigidObjects[i]->name<<" to "<<relpath + world.rigidObjects[i]->geomFile);
      world.rigidObjects[i]->geometry->Save((relpath + world.rigidObjects[i]->geomFile).c_str());
    }
  }
  for(size_t i=0;i<world.terrains.size();i++) {
    if(world.terrains[i]->geometry.Empty()) continue;
    if(world.terrains[i]->geometry.IsOriginal()) {
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
        if(!FileUtils::MakeDirectory((relpath+geomdir).c_str())) {
          LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Unable to make directory "<<relpath+geomdir);
        }
      world.terrains[i]->geomFile = geomdir + "/" + FileUtils::SafeFileName(world.terrains[i]->name) + DefaultFileExtension(*world.terrains[i]->geometry);
      LOG4CXX_INFO(GET_LOGGER(XmlParser),"  Saving modified geometry for terrain "<<world.terrains[i]->name<<" to "<<relpath + world.terrains[i]->geomFile);
      world.terrains[i]->geometry->Save((relpath + world.terrains[i]->geomFile).c_str());
    }
  }

  for(size_t i=0;i<world.robots.size();i++) {
    LOG4CXX_INFO(GET_LOGGER(XmlParser),"  Saving robot to "<<itempath+robotFileNames[i]);
    if(!world.robots[i]->Save((itempath + robotFileNames[i]).c_str())) {
      LOG4CXX_WARN(GET_LOGGER(XmlParser),"  Robot saving failed.");
      return false;
    }
  }
  for(size_t i=0;i<world.rigidObjects.size();i++) {
    LOG4CXX_INFO(GET_LOGGER(XmlParser),"  Saving rigid object to "<<itempath+objectFileNames[i]);
    if(!world.rigidObjects[i]->Save((itempath + objectFileNames[i]).c_str())) {
      LOG4CXX_WARN(GET_LOGGER(XmlParser),"  Rigid object saving failed.");
      return false;
    }
  }
  for(size_t i=0;i<world.terrains.size();i++) {
    LOG4CXX_INFO(GET_LOGGER(XmlParser),"  Saving terrain to "<<itempath+terrainFileNames[i]);
    if(!world.terrains[i]->Save((itempath + terrainFileNames[i]).c_str())) {
      LOG4CXX_WARN(GET_LOGGER(XmlParser),"  Terrain saving failed.");
      return false;
    }
  }
  FILE* out = fopen(fn.c_str(),"w");
  if(!out) return false;
  fprintf(out,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<world>\n");
  fprintf(out,"  <display background=\"%f %f %f %f\" />\n",world.background.rgba[0],world.background.rgba[1],world.background.rgba[2],world.background.rgba[3]);
  for(size_t i=0;i<world.robots.size();i++) {
    fprintf(out,"  <robot name=\"%s\" file=\"%s\" >\n",world.robots[i]->name.c_str(),(relpath+robotFileNames[i]).c_str());
    for(size_t j=0;j<world.robots[i]->links.size();j++) {
      if(!world.robots[i]->IsGeometryEmpty(j))
        WriteAppearance(world.robots[i]->geomManagers[j],out,4,world.robots[i]->linkNames[i].c_str());
    }
    fprintf(out,"  </robot>\n");
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
    LOG4CXX_WARN(GET_LOGGER(XmlParser),"World::Save(): warning: geometry files may not be saved properly");
  return true; 
}