#ifndef WORLD_VIEW_PROGRAM_H
#define WORLD_VIEW_PROGRAM_H

#include <Klampt/Modeling/World.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/Widget.h>
#include <KrisLibrary/math3d/Ray3D.h>

#if HAVE_GLUI

  #include <KrisLibrary/GLdraw/GLUINavigationProgram.h>
  #include <KrisLibrary/GLdraw/GLScreenshotProgram.h>
  #if defined (__APPLE__) || defined (MACOSX)
  #include <glui.h>
  #else
  #include <GL/glui.h>
  #endif //__APPLE__ || MACOSX

  #define BASE_PROGRAM GLUINavigationProgram

#else

  #if HAVE_GLUT

  #include <KrisLibrary/GLdraw/GLUTNavigationProgram.h>
  #include <KrisLibrary/GLdraw/GLScreenshotProgram.h>
  #define BASE_PROGRAM GLUTNavigationProgram

  #if defined (__APPLE__) || defined (MACOSX)
  #include <GLUT/glut.h>
  #else
  #include <GL/glut.h>
  #endif // Apple

  #if FREEGLUT
  #include <GL/freeglut_ext.h>
  #endif //FREEGLUT


  #endif //HAVE_GLUT

#endif //HAVE_GLUI

namespace Klampt {
  using namespace Math3D;
  using namespace GLDraw;

bool LoadWorldCommandLine(WorldModel& world,int argc, const char** argv);


#if HAVE_GLUI || HAVE_GLUT

/** @brief A very simple interface that directly uses GLUT or GLUI to pop up a window
 * where you can interact with a world.
 * 
 * Just subclass and override Handle_X functions of GLUTNavigationProgram/GLUINavigationProgram,
 * and call Run() on the object at the end of your main() function.
 */
class WorldViewProgram : public GLScreenshotProgram<BASE_PROGRAM>
{
public:
  WorldViewProgram(WorldModel* world);
  virtual ~WorldViewProgram() {}

  bool LoadCommandLine(int argc,const char** argv);
  virtual bool Initialize();
  virtual void SetWorldLights();
  void ClickRay(int x,int y,Ray3D& r) const;
  RobotModel* ClickRobot(const Ray3D& r,int& body,Vector3& localpt) const;
  RigidObjectModel* ClickObject(const Ray3D& r,Vector3& localpt) const;
  virtual void RefreshIdle();
  virtual void RenderWorld();
  virtual void DoFreeDrag(int dx,int dy,int button);
  virtual void DoCtrlDrag(int dx,int dy,int button);
  virtual void DoAltDrag(int dx,int dy,int button);
  virtual void DoShiftDrag(int dx,int dy,int button);

  WorldModel* world;
};

#endif // HAVE_GLUI || HAVE_GLUT


/** @brief A widget interface that can be overloaded to interact with a world. 
 */
class WorldViewWidget : public Widget
{
public:
  WorldViewWidget(WorldModel* world);
  virtual bool Hover(int x, int y, Camera::Viewport& viewport, double& distance);
  //these currently don't do anything, but can be overloaded to do something with the current robot
  virtual bool BeginDrag(int x, int y, Camera::Viewport& viewport, double& distance);
  virtual void Drag(int dx, int dy, Camera::Viewport& viewport) {}
  virtual void EndDrag() {}

  virtual void DrawGL(Camera::Viewport& viewport);

  WorldModel* world;
  //click information
  RobotModel* clickedRobot;
  RigidObjectModel* clickedObject;
  int body;
  Vector3 localpt;
};

} // namespace Klampt

#endif
