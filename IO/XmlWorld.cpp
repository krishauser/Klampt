#include "XmlWorld.h"
#include <utils/stringutils.h>

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
      printf("%d!=%d\n",q.n,robot.q.n);
      fprintf(stderr,"XmlRobot: element's configuration doesnt match size with the robot\n");
      return false;
    }
    robot.UpdateConfig(q);
  }
  return true;
}


XmlRigidObject::XmlRigidObject(TiXmlElement* _element,string _path)
  :e(_element),path(_path)
{}

bool XmlRigidObject::GetObject(RigidObject& obj)
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
    const char* fn = geom->Attribute("mesh");
    if(fn) {
      string sfn = path + string(fn);
      if(!obj.geometry.Load(sfn.c_str())) {
	fprintf(stderr,"XmlRigidObject: error loading meshfile %s, trying absolute path\n",sfn.c_str());
	if(!obj.geometry.Load(fn)) {
	  fprintf(stderr,"XmlRigidObject: error loading meshfile %s\n",sfn.c_str());
	  return false;
	}
	else
	  fprintf(stderr,"XmlRigidObject: absolute path succeeded\n");
      }
    }
    Matrix4 xform;
    xform.setIdentity();
    Real temp;
    Vector3 temp3;
    if(geom->QueryValueAttribute("scale",&temp3) == TIXML_SUCCESS) {
      temp3.get(xform(0,0),xform(1,1),xform(2,2));
    }
    else if(geom->QueryValueAttribute("scale",&temp) == TIXML_SUCCESS) {
      xform(0,0)=xform(1,1)=xform(2,2)=temp;
    }
    //TODO rotations too
    if(geom->QueryValueAttribute("translate",&temp3) == TIXML_SUCCESS) {
      xform(0,3)=temp3.x;
      xform(1,3)=temp3.y;
      xform(2,3)=temp3.z;
    }
    if(geom->QueryValueAttribute("margin",&temp) == TIXML_SUCCESS) {
      obj.geometry.margin = temp;
    }
    obj.geometry.Transform(xform);
    obj.geometry.InitCollisions();
  }
  if(obj.geometry.Empty()) {
    fprintf(stderr,"XmlRigidObject: element does not contain geometry attribute\n");
    return false;
  }
  Vector3 position;
  if(e->QueryValueAttribute("position",&position)==TIXML_SUCCESS) {
    obj.T.t = position;
  }
  Real val;
  if(e->QueryValueAttribute("rotateX",&val)==TIXML_SUCCESS) {
    Matrix3 R;
    R.setRotateX(val);
    obj.T.R = R*obj.T.R;
  }
  if(e->QueryValueAttribute("rotateY",&val)==TIXML_SUCCESS) {
    Matrix3 R;
    R.setRotateY(val);
    obj.T.R = R*obj.T.R;
  }
  if(e->QueryValueAttribute("rotateZ",&val)==TIXML_SUCCESS) {
    Matrix3 R;
    R.setRotateY(val);
    obj.T.R = R*obj.T.R;
  }
  //TODO: more attributes
  if(e->Attribute("rotationMoment")) {
    MomentRotation pos;
    stringstream ss(e->Attribute("rotationMoment"));
    ss >> pos;
    Matrix3 R;
    pos.getMatrix(R);
    obj.T.R = R*obj.T.R;
  }
  /*
  if(e->Attribute("rotationAngleAxis")) {
    AngleAxisRotation pos;
    stringstream ss(e->Attribute("rotationAngleAxis"));
    ss >> pos;
    obj.T.t = pos;
  }
  */

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
    
    if(phys->QueryValueAttribute("kRestitution",&val)==TIXML_SUCCESS)
      obj.kRestitution=val;
    if(phys->QueryValueAttribute("kFriction",&val)==TIXML_SUCCESS)
      obj.kFriction = val;
    if(phys->QueryValueAttribute("kStiffness",&val)==TIXML_SUCCESS)
      obj.kStiffness = val;
    if(phys->QueryValueAttribute("kDamping",&val)==TIXML_SUCCESS)
      obj.kDamping = val;
  }

  obj.geometry.SetTransform(obj.T);
  return true;
}

XmlTerrain::XmlTerrain(TiXmlElement* _element,string _path)
  :e(_element),path(_path)
{}

bool XmlTerrain::GetTerrain(Environment& env)
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

  Matrix3 R;
  R.setIdentity();
  Vector3 s;
  Vector3 pos(Zero);
  bool transform=false;
  if(e->QueryValueAttribute("scale",&s)==TIXML_SUCCESS) 
    transform=true;
  else s.set(1,1,1);

  if(e->QueryValueAttribute("translation",&pos)==TIXML_SUCCESS) 
    transform=true;
  else if(e->QueryValueAttribute("position",&pos)==TIXML_SUCCESS) 
    transform=true;
  else pos.setZero();

  if(e->Attribute("rotateRPY")) {
    stringstream ss(e->Attribute("rotateRPY"));
    EulerAngleRotation ea;
    ss>>ea;
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
    R.setRotateX(val);
    transform = true;
  }
  if(e->Attribute("rotateY")) {
    Real val;
    stringstream ss(e->Attribute("rotateY"));
    ss >> val;
    R.setRotateY(val);
    transform = true;
  }
  if(e->Attribute("rotateZ")) {
    Real val;
    stringstream ss(e->Attribute("rotateZ"));
    ss >> val;
    R.setRotateZ(val);
    transform = true;
  }
  if(transform) {
    Matrix4 xform;
    xform.setIdentity();
    for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
	xform(i,j)=R(i,j)*s[j];
    pos.get(xform(0,3),xform(1,3),xform(2,3));
    env.geometry.Transform(xform);
    env.geometry.InitCollisions();
  }
  Real margin;
  if(e->QueryValueAttribute("margin",&margin) == TIXML_SUCCESS) {
    env.geometry.margin = margin;
  }
  return true;
}

class XmlViewTerrain
{
 public:
  XmlViewTerrain(TiXmlElement* element) : e(element) {}
  bool GetView(ViewEnvironment& view)
  {
    if(e->Attribute("color")) {
      Vector3 rgb;
      stringstream ss(e->Attribute("color"));
      ss >> rgb;
      Real a=1.0;
      if(ss >> a) { }
      else a=1.0;
      view.texture = ViewEnvironment::NoTexture;
      view.appearance.faceColor.set(rgb.x,rgb.y,rgb.z,a);
    }
    if(e->Attribute("texture")) {
      if(0==strcmp(e->Attribute("texture"),"checker")) {
	view.texture = ViewEnvironment::CheckerTexture;
	view.texCoords = ViewEnvironment::XYTexCoords;
      }
      else if(0==strcmp(e->Attribute("texture"),"noise")) {
	view.texture = ViewEnvironment::NoiseTexture;
	view.texCoords = ViewEnvironment::XYTexCoords;
      }
      else if(0==strcmp(e->Attribute("texture"),"gradient")) {
	view.texture = ViewEnvironment::GradientTexture;
	view.texCoords = ViewEnvironment::ZTexCoord;
      }
      else if(0==strcmp(e->Attribute("texture"),"colorgradient")) {
	view.texture = ViewEnvironment::ColorGradientTexture;
	view.texCoords = ViewEnvironment::ZTexCoord;
      }
    }
    return true;
  }

  TiXmlElement* e;
};




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
  string goal="goal";
  TiXmlElement* e;
  //parse display
  e = GetElement(display);
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
    if(!XmlRigidObject(e,path).GetObject(*o)) {
      printf("XmlWorld: Unable to load rigid object %s\n",sname.c_str());
      delete o;
      return false;
    }
    int i = world.AddRigidObject(sname,o);
    e = e->NextSiblingElement(object);
  }
  //parse objects
  e = GetElement(terrain);
  while(e) {
    const char* name = e->Attribute("name");
    string sname = "Terrain";
    if(name) sname=name;
    Environment* t = new Environment;
    if(!XmlTerrain(e,path).GetTerrain(*t)) {
      printf("XmlWorld: Unable to load terrain %s\n",sname.c_str());
      delete t;
      return false;
    }
    int i = world.AddTerrain(sname,t);
    TiXmlElement* d = e->FirstChildElement(display);
    if(d) {
      if(!XmlViewTerrain(d).GetView(world.terrains[i].view)) {
	printf("XmlWorld: Warning, unable to load terrain view %s\n",sname.c_str());
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
