#include "WorldGUI.h"
#include "IO/ROS.h"
#include <string.h>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <fstream>
using namespace GLDraw;

WorldGUIBackend::WorldGUIBackend(RobotWorld* _world)
  :world(_world)
{
}

WorldGUIBackend::~WorldGUIBackend()
{
  if(ROSNumSubscribedTopics() > 0 || ROSNumPublishedTopics() > 0) ROSShutdown();
}

bool WorldGUIBackend::OnIdle()
{
  bool res = GLNavigationBackend::OnIdle();
  if(ROSNumSubscribedTopics() > 0) {
    if(ROSSubscribeUpdate()) {
      //need to refresh appearances
      for(size_t i=0;i<world->robots.size();i++)
	for(size_t j=0;j<world->robots[i]->links.size();j++)
	  world->robots[i]->geomManagers[j].DynamicGeometryUpdate();
      for(size_t i=0;i<world->rigidObjects.size();i++)
	world->rigidObjects[i]->geometry.DynamicGeometryUpdate();
      for(size_t i=0;i<world->terrains.size();i++)
	world->terrains[i]->geometry.DynamicGeometryUpdate();
      SendRefresh();
    }
    else
      SendPauseIdle(0.05);
  }
  else
    SendPauseIdle();
  return res;
}

bool WorldGUIBackend::OnCommand(const string& cmd,const string& args)
{
  if(cmd=="load_file") {
    LoadFile(args.c_str());
    return true;
  }
  return GLNavigationBackend::OnCommand(cmd,args);
}

bool WorldGUIBackend::OnMouseWheel(int dwheel)
{
  camera.dist *= (1 + 0.01*Real(dwheel)/20);
  show_view_target=1; 
  t_hide_view_target=timer.LastElapsedTime()+0.5; 
  SendPauseIdle(0);
  SendRefresh();
  return true;
}

bool WorldGUIBackend::LoadFile(const char* fn)
{
    const char* ext=FileExtension(fn);
    if(0==strcmp(ext,"xml")) {
      if(!world->LoadXML(fn)) {
	printf("Error loading world file %s\n",fn);
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
	if(!world->LoadXML(argv[i])) {
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
    if(temp.n != (int)world->robots[i]->links.size()) {
      printf("Incorrect number of DOFs in config %d\n",i);
      continue;
    }
    world->robots[i]->UpdateConfig(temp);
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

Robot* WorldGUIBackend::ClickRobot(const Ray3D& r,int& body,Vector3& localpt) const
{
  return world->ClickRobot(r,body,localpt);
}

RigidObject* WorldGUIBackend::ClickObject(const Ray3D& r,Vector3& localpt) const
{
  return world->ClickObject(r,localpt);
}
