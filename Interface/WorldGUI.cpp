#include "WorldGUI.h"
#include "IO/XmlWorld.h"
#include <utils/stringutils.h>
#include <GLdraw/GL.h>
#include <GLdraw/drawextra.h>
#include <fstream>
using namespace GLDraw;

WorldGUIBackend::WorldGUIBackend(RobotWorld* _world)
  :world(_world)
{
}

bool WorldGUIBackend::OnCommand(const string& cmd,const string& args)
{
  if(cmd=="load_file") {
    LoadFile(args.c_str());
    return true;
  }
  return GLNavigationBackend::OnCommand(cmd,args);
  return false;
}

bool WorldGUIBackend::LoadFile(const char* fn)
{
    const char* ext=FileExtension(fn);
    if(0==strcmp(ext,"xml")) {
      XmlWorld xmlWorld;
      if(!xmlWorld.Load(fn)) {
	printf("Error loading world file %s\n",fn);
	return false;
      }
      if(!xmlWorld.GetWorld(*world)) {
	printf("Error loading world from %s\n",fn);
	return false;
      }
    }
    else {
      if(world->LoadElement(fn) < 0) {
	return false;
      }
    }
    return true;
}


bool WorldGUIBackend::LoadCommandLine(int argc,const char** argv)
{
  XmlWorld xmlWorld;
  vector<string> configs;

  world->lights.resize(1);
  world->lights[0].setColor(GLColor(1,1,1));
  world->lights[0].setDirectionalLight(Vector3(0.2,-0.4,1));
  world->lights[0].setColor(GLColor(1,1,1));

  for(int i=1;i<argc;i++) {
    if(argv[i][0] == '-') {
      if(0==strcmp(argv[i],"-config")) {
	configs.push_back(argv[i+1]);
	i++;
      }
      else {
	printf("Unknown option %s",argv[i]);
	return false;
      }
    }
    else {
      const char* ext=FileExtension(argv[i]);
      if(0==strcmp(ext,"xml")) {
	if(!xmlWorld.Load(argv[i])) {
	  printf("Error loading world file %s\n",argv[i]);
	  return false;
	}
	if(!xmlWorld.GetWorld(*world)) {
	  printf("Error loading world from %s\n",argv[i]);
	  return false;
	}
      }
      else {
	if(world->LoadElement(argv[i]) < 0) {
	  return false;
	}
      }
    }
  }

  if(configs.size() > world->robots.size()) {
    printf("Warning, too many configs specified\n");
  }
  for(size_t i=0;i<configs.size();i++) {
    if(i >= world->robots.size()) break;
    ifstream in(configs[i].c_str(),ios::in);
    if(!in) printf("Could not open config file %s\n",configs[i].c_str());
    Vector temp;
    in >> temp;
    if(!in) printf("Error reading config file %s\n",configs[i].c_str());
    if(temp.n != (int)world->robots[i].robot->links.size()) {
      printf("Incorrect number of DOFs in config %d\n",i);
      continue;
    }
    world->robots[i].robot->UpdateConfig(temp);
  }
  return true;
}

void WorldGUIBackend::RenderWorld()
{
  glDisable(GL_LIGHTING);
  drawCoords(0.1);
  glEnable(GL_LIGHTING);
  world->DrawGL();
}

void WorldGUIBackend::Start()
{
  //TODO: read in the camera from the world?
  camera.dist = 6;
  viewport.n = 0.1;
  viewport.f = 100;
  viewport.setLensAngle(DtoR(30.0));

  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glClearColor(world->background.rgba[0],world->background.rgba[1],world->background.rgba[2],world->background.rgba[3]);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  return GLNavigationBackend::Start();
}

RobotInfo* WorldGUIBackend::ClickRobot(const Ray3D& r,int& body,Vector3& localpt) const
{
  return world->ClickRobot(r,body,localpt);
}

RigidObjectInfo* WorldGUIBackend::ClickObject(const Ray3D& r,Vector3& localpt) const
{
  return world->ClickObject(r,localpt);
}
