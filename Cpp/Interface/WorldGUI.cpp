#include "WorldGUI.h"
#include "IO/ROS.h"
#include <string.h>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <fstream>
using namespace GLDraw;
using namespace Klampt;

WorldGUIBackend::WorldGUIBackend(WorldModel* _world)
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
  else if(cmd=="reload_file") {
    ReloadFile(args.c_str());
    return true;
  }
  else if(cmd=="save_world") {
    SaveWorld(args.c_str());
    return true;
  }
  return GLNavigationBackend::OnCommand(cmd,args);
}

bool WorldGUIBackend::OnMouseWheel(int dwheel)
{
  camera.dist *= (1 + 0.01*Real(dwheel)/20);
  show_view_target=1; 
  t_hide_view_target=float(timer.LastElapsedTime()+0.5); 
  SendPauseIdle(0);
  SendRefresh();
  return true;
}

bool WorldGUIBackend::LoadFile(const char* fn)
{
    const char* ext=FileExtension(fn);
    if(!ext) {
      printf("Error loading file %s, no extension provided\n",fn);
      return false;
    }
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

bool WorldGUIBackend::ReloadFile(const char* fn)
{
  const char* ext=FileExtension(fn);
  if(!ext) {
    printf("WorldGUIBackend:: Error loading file %s, no extension provided\n",fn);
    return false;
  }
  printf("Reloading %s\n",fn);
    if(0==strcmp(ext,"xml")) {
      if(!world->LoadXML(fn)) {
  printf("WorldGUIBackend::ReloadFile: Error loading world file %s\n",fn);
  return false;
      }
    }
    else {
      int id = world->LoadElement(fn);
      if(id < 0) {
        printf("WorldGUIBackend::ReloadFile: Error loading file %s\n",fn);
        return false;
      }
      string name = world->GetName(id);
      bool found = false;
      if(world->IsRobot(id) >= 0) {
        int index = world->IsRobot(id);
        //check for previous robots of the same filename
        for(size_t i=0;i<world->robots.size();i++) {
          if(name == world->robots[i]->name) {
            shared_ptr<RobotModel> r = world->robots[index];
            shared_ptr<RobotModel> s = world->robots[i];
            //copy configuration, if possible
            map<string,int> jqmap;
            for(size_t j=0;j<r->links.size();j++)
              jqmap[r->linkNames[j]] = (int)j;
            for(size_t j=0;j<s->links.size();j++) {
              string n = s->linkNames[j];
              if(jqmap.count(n) > 0) {
                r->q[jqmap[n]] = s->q[j];
                r->dq[jqmap[n]] = s->dq[j];
              }
            }
            r->UpdateFrames();
            r->UpdateGeometry();
            world->robots[i] = r;
            world->robots.erase(world->robots.begin()+index);
            world->robotViews[i] = ViewRobot(r.get());
            found = true;
            break;
          }
        }
      }
      else if(world->IsRigidObject(id) >= 0) {
        int index = world->IsRigidObject(id);
        //check for previous rigid objects of the same filename
        for(size_t i=0;i<world->rigidObjects.size();i++) {
          if(name == world->rigidObjects[i]->name) {
            world->rigidObjects[index]->T = world->rigidObjects[i]->T;
            world->rigidObjects[i] = world->rigidObjects[index];
            world->rigidObjects.erase(world->rigidObjects.begin()+index);
            found = true;
            break;
          }
        }
      }
      else  {
        int index = world->IsTerrain(id);
        Assert(index >= 0);
        //check for previous rigid objects of the same filename
        for(size_t i=0;i<world->terrains.size();i++) {
          if(name == world->terrains[i]->name) {
            world->terrains[i] = world->terrains[index];
            world->terrains.erase(world->terrains.begin()+index);
            found = true;
            break;
          }
        }
      }
      if(!found) {
        printf("WorldGUIBackend::ReloadFile: unable to find a previous item named %s\n",name.c_str());
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
      if(!ext) {
        printf("Error loading file %s, no extension provided\n",argv[i]);
        return false;
      }
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

bool WorldGUIBackend::SaveWorld(const char* fn,const char* elementPath)
{
  if(!world->SaveXML(fn,elementPath)) {
printf("Error saving world file %s\n",fn);
return false;
  }
  return true;
}

void WorldGUIBackend::RenderWorld()
{
  glDisable(GL_LIGHTING);
  drawCoords(0.1f);
  glEnable(GL_LIGHTING);
  world->DrawGL();
}

void WorldGUIBackend::Start()
{
  //TODO: read in the camera from the world?
  camera.dist = 6;
  viewport.n = 0.1f;
  viewport.f = 100;
  viewport.setLensAngle(DtoR(60.0f));
  //viewport.setLensAngle(DtoR(90.0f));

  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glClearColor(world->background.rgba[0],world->background.rgba[1],world->background.rgba[2],world->background.rgba[3]);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  return GLNavigationBackend::Start();
}

RobotModel* WorldGUIBackend::ClickRobot(const Ray3D& r,int& body,Vector3& localpt) const
{
  return world->RayCastRobot(r,body,localpt);
}

RigidObjectModel* WorldGUIBackend::ClickObject(const Ray3D& r,Vector3& localpt) const
{
  return world->RayCastObject(r,localpt);
}
