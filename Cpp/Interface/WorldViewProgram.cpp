#include "WorldViewProgram.h"
#include <KrisLibrary/Logger.h>
#include <Klampt/IO/XmlWorld.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/Widget.h>
#include <KrisLibrary/utils/stringutils.h>
#include <fstream>
using namespace Math3D;
using namespace GLDraw;
namespace Klampt {

WorldViewWidget::WorldViewWidget(WorldModel* _world)
  :world(_world), clickedRobot(NULL), clickedObject(NULL)
{}

bool WorldViewWidget::Hover(int x, int y, Camera::Viewport& viewport, double& distance)
{
  Ray3D r;
  viewport.getClickSource(float(x), float(y), r.source);
  viewport.getClickVector(float(x), float(y), r.direction);
  clickedRobot = world->RayCastRobot(r, body, localpt);
  if (clickedRobot) {
    Vector3 worldpt = clickedRobot->links[body].T_World*localpt;
    distance = r.direction.dot(worldpt - r.source);
  }
  Vector3 localpt2;
  clickedObject = world->RayCastObject(r, localpt2);
  if (clickedObject) {
    Vector3 worldpt2 = clickedObject->T*localpt2;
    Real distance2 = r.direction.dot(worldpt2 - r.source);
    if (clickedRobot && distance < distance2) {
      //robot is closest
      clickedObject = NULL;
    }
    else {
      clickedRobot = NULL;
      localpt = localpt2;
      distance = distance2;
    }
  }
  return clickedRobot || clickedObject;
}
 
 bool WorldViewWidget::BeginDrag(int x, int y, Camera::Viewport& viewport, double& distance)
 {
  bool res = Hover(x, y, viewport, distance);
  return false;
}
 
 void WorldViewWidget::DrawGL(Camera::Viewport& viewport)
 {
  glEnable(GL_LIGHTING);
  world->DrawGL();
}



bool LoadWorldCommandLine(WorldModel& world,int argc, const char** argv)
{
  XmlWorld xmlWorld;
  vector<string> configs;
  world.lights.resize(1);
  world.lights[0].setColor(GLColor(1, 1, 1));
  world.lights[0].setDirectionalLight(Vector3(0.2, -0.4, 1));

  for (int i = 1; i<argc; i++) {
    if (argv[i][0] == '-') {
      if (0 == strcmp(argv[i], "-config")) {
        configs.push_back(argv[i + 1]);
        i++;
      }
      else {
        printf("Unknown option %s", argv[i]);
        return false;
      }
    }
    else {
      const char* ext = FileExtension(argv[i]);
      if (0 == strcmp(ext, "xml")) {
        if (!xmlWorld.Load(argv[i])) {
          printf("Error loading world file %s\n", argv[i]);
          return false;
        }
        if (!xmlWorld.GetWorld(world)) {
          printf("Error loading world from %s\n", argv[i]);
          return false;
        }
      }
      else {
        if (world.LoadElement(argv[i]) < 0) {
          return false;
        }
      }
    }
  }

  if (configs.size() > world.robots.size()) {
    printf("Warning, too many configs specified\n");
  }
  for (size_t i = 0; i<configs.size(); i++) {
    if (i >= world.robots.size()) break;
    ifstream in(configs[i].c_str(), ios::in);
    if (!in) printf("Could not open config file %s\n", configs[i].c_str());
    Vector temp;
    in >> temp;
    if (!in) printf("Error reading config file %s\n", configs[i].c_str());
    if (temp.n != (int)world.robots[i]->links.size()) {
      printf("Incorrect number of DOFs in config %d\n", i);
      continue;
    }
    world.robots[i]->UpdateConfig(temp);
  }

  return true;
}


#if HAVE_GLUI || HAVE_GLUT


WorldViewProgram::WorldViewProgram(WorldModel* _world)
  :world(_world)
{
}

bool WorldViewProgram::Initialize()
{
  //TODO: read in the camera from the world?
  camera.dist = 6;
  viewport.n = 0.1;
  viewport.f = 100;
  viewport.setLensAngle(DtoR(60.0));

  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glClearColor(world->background.rgba[0],world->background.rgba[1],world->background.rgba[2],world->background.rgba[3]);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  return BASE_PROGRAM::Initialize();
}

void WorldViewProgram::SetWorldLights()
{
  world->SetGLLights();
}
void WorldViewProgram::RefreshIdle() { SleepIdleCallback(0); }
void WorldViewProgram::RenderWorld()
{
  glDisable(GL_LIGHTING);
  drawCoords(0.1);
  glEnable(GL_LIGHTING);
  world->DrawGL();
}

void WorldViewProgram::DoFreeDrag(int dx,int dy,int button)
{
  if(button == GLUT_LEFT_BUTTON)  DragRotate(dx,dy);
}

void WorldViewProgram::DoCtrlDrag(int dx,int dy,int button)
{
  if(button == GLUT_LEFT_BUTTON)  DragPan(dx,dy);
}

void WorldViewProgram::DoAltDrag(int dx,int dy,int button)
{
  if(button == GLUT_LEFT_BUTTON)  DragZoom(dx,dy);
}

void WorldViewProgram::DoShiftDrag(int dx,int dy,int button)
{
  if(button == GLUT_LEFT_BUTTON) { camera.dist *= (1 + 0.01*Real(dy)); }
}


bool WorldViewProgram::LoadCommandLine(int argc,const char** argv)
{
  return LoadWorldCommandLine(*world, argc, argv);
}


void WorldViewProgram::ClickRay(int x,int y,Ray3D& r) const
{
  viewport.getClickSource(x,viewport.y+viewport.h-y,r.source);
  viewport.getClickVector(x,viewport.y+viewport.h-y,r.direction);
  //cout<<"Ray "<<r.source<<" -> "<<r.direction<<endl;
}


RobotModel* WorldViewProgram::ClickRobot(const Ray3D& r,int& body,Vector3& localpt) const
{
  return world->RayCastRobot(r,body,localpt);
}

RigidObjectModel* WorldViewProgram::ClickObject(const Ray3D& r,Vector3& localpt) const
{
  return world->RayCastObject(r,localpt);
}

#endif // HAVE_GLUI

} //namespace Klampt

