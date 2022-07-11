#include "NavigationGUI.h"
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/GLView.h>
#include <KrisLibrary/GLdraw/GLError.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <iostream>
#include <string>
#include <fstream>
using namespace GLDraw;
using namespace Klampt;

#if HAVE_GLUT

#if defined (__APPLE__) || defined (MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif //Apple

#else

/* Mouse buttons. */
#define GLUT_LEFT_BUTTON		0
#define GLUT_MIDDLE_BUTTON		1
#define GLUT_RIGHT_BUTTON		2

/* glutGetModifiers return mask. */
#define GLUT_ACTIVE_SHIFT               1
#define GLUT_ACTIVE_CTRL                2
#define GLUT_ACTIVE_ALT                 4

#endif //HAVE_GLUT

#define SHOW_VIEW_TARGET(secs) {\
  show_view_target=1; \
  t_hide_view_target=float(timer.LastElapsedTime()+secs); \
  SendPauseIdle(0);				   \
}

MouseDragBackend::MouseDragBackend()
  :oldmousex(0),oldmousey(0),
   clickButton(-1),clickModifiers(0)
{}

bool MouseDragBackend::OnKeyDown(const string& key)
{
  if(key=="shift") {
    clickModifiers |= GLUT_ACTIVE_SHIFT;
    return true;
  }
  else if(key=="control") {
    clickModifiers |= GLUT_ACTIVE_CTRL;
    return true;
  }
  else if(key=="alt") {
    clickModifiers |= GLUT_ACTIVE_ALT;
    return true;
  }
  return GenericBackendBase::OnKeyDown(key);
}

bool MouseDragBackend::OnKeyUp(const string& key)
{
  if(key=="shift") {
    clickModifiers &= ~GLUT_ACTIVE_SHIFT;
    return true;
  }
  else if(key=="control") {
    clickModifiers &= ~GLUT_ACTIVE_CTRL;
    return true;
  }
  else if(key=="alt") {
    clickModifiers &= ~GLUT_ACTIVE_ALT;
    return true;
  }
  return GenericBackendBase::OnKeyUp(key);
}


bool MouseDragBackend::OnMouseClick(int button,int state,int x,int y)
{
  oldmousex=x;
  oldmousey=y;
  clickButton = button;
  if(state == 0) {
    EndDrag(x,y,clickButton,clickModifiers);
    clickButton=-1;
  }
  else {
    BeginDrag(x,y,clickButton,clickModifiers);
  }
  return true;
}

bool MouseDragBackend::OnMouseMove(int x,int y)
{
  if(clickButton >= 0) {
    int dx=x-oldmousex,dy=y-oldmousey;
    DoDrag(dx,dy,clickButton,clickModifiers);
  }
  else 
    DoPassiveMouseMove(x,y);
  oldmousex=x;
  oldmousey=y;
  return true;
}

void MouseDragBackend::DoDrag(int dx,int dy,int button,int modifiers)
{
  if(button != -1) {
    int ctrl_pressed = modifiers&GLUT_ACTIVE_CTRL;
    int alt_pressed = modifiers&GLUT_ACTIVE_ALT;
    int shift_pressed = modifiers&GLUT_ACTIVE_SHIFT;

    if(ctrl_pressed) DoCtrlDrag(dx,dy,button);
    else if(alt_pressed) DoAltDrag(dx,dy,button);
    else if(shift_pressed) DoShiftDrag(dx,dy,button);
    else DoFreeDrag(dx,dy,clickButton);
  }
}




GLNavigationBackend::GLNavigationBackend()
  :stereo_mode(false),stereo_offset(.02f),
   show_view_target(0),t_hide_view_target(0),
   frames_per_second(0),show_frames_per_second(false),
   frames_rendered(0),
   mode_2d(false)
{
  //const int control=-'a'+1; // only works reliably for lowercase letters
  camera.tgt.setZero();
  camera.rot.setZero();
  camera.dist=100;
  camera.ori=Camera::Camera::XZnY;
}

void GLNavigationBackend::Start()
{
  //glClearColor(0,0,0,0);
  glClearDepth(1);
  timer.Reset();
}

bool GLNavigationBackend::OnGLViewport(int x,int y,int w,int h)
{
  viewport.x=x;
  viewport.y=y;
  viewport.w=w;
  viewport.h=h;
  return true;
}


bool GLNavigationBackend::OnGLRender()
{
  DEBUG_GL_ERRORS()
  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  //SetWorldLights();

  camera.toCamera(viewport);

  DEBUG_GL_ERRORS()
  GLView view;
  view.setViewport(viewport);
  view.setCurrentGL();
  SetWorldLights();

  if(stereo_mode){
    glColorMask(GL_FALSE,GL_TRUE,GL_TRUE,GL_TRUE); // draw only to green, blue and alpha
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glTranslatef(-stereo_offset*float(camera.dist),0,0);
    RenderWorld();
    glPopMatrix();
    glClear(GL_DEPTH_BUFFER_BIT); // leave the blue image but clear Z (NOTE: may need to clear alpha as well for transparency effects!)
    glColorMask(GL_TRUE,GL_FALSE,GL_FALSE,GL_TRUE); // draw only to red and alpha
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glTranslatef(stereo_offset*float(camera.dist),0,0);
    RenderWorld();
    glPopMatrix();
    glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);}
  else {
    RenderWorld();
    if(show_view_target) DisplayCameraTarget();
  }

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0,viewport.w,viewport.h,0,-100,100);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  RenderScreen();

  ++frames_rendered;
  SendPauseIdle(0);

  DEBUG_GL_ERRORS()
  return true;
}


void GLNavigationBackend::ClickRay(int x,int y,Math3D::Vector3& src,Math3D::Vector3& dir) const
{
  viewport.getClickSource(x,viewport.y+viewport.h-y,src);
  viewport.getClickVector(x,viewport.y+viewport.h-y,dir);
  //cout<<"Ray "<<r.source<<" -> "<<r.direction<<endl;
}


bool GLNavigationBackend::OnMouseWheel(int dwheel)
{
  viewport.scale *= (1+float(dwheel)/20*0.01f);
  SHOW_VIEW_TARGET(0.5);
  SendRefresh();
  return true;
}


bool GLNavigationBackend::OnCommand(const string& cmd,const string& args)
{
  if(cmd == "save_view") {
    printf("Saving viewport to %s\n",args.c_str());
    ofstream out(args.c_str(),ios::out);
    WriteDisplaySettings(out);
    return true;
  }
  else if(cmd == "load_view") {
    printf("Loading viewport from %s...\n",args.c_str());
    ifstream in(args.c_str(),ios::in);
    if(!in) {
      printf("Unable to open %s\n",args.c_str());
    }
    else {
      ReadDisplaySettings(in);
      SendRefresh();
    }
    return true;
  }
  return BaseT::OnCommand(cmd,args);
}

void GLNavigationBackend::BeginDrag(int x,int y,int button,int modifiers)
{
  SHOW_VIEW_TARGET(1);
}

void GLNavigationBackend::DoFreeDrag(int dx,int dy,int button)
{
  switch(button){
  case GLUT_LEFT_BUTTON:
    if (mode_2d) DragPan(dx,dy);
    else  DragRotate(dx,dy);
    break;
  case GLUT_RIGHT_BUTTON:
    DragZoom(dx,dy);
    break;
  case GLUT_MIDDLE_BUTTON:
    DragPan(dx,dy);
    break;
  }
}

void GLNavigationBackend::DoCtrlDrag(int dx,int dy,int button)
{
  switch(button) {
  case GLUT_LEFT_BUTTON:
    DragPan(dx,dy);
    break;
  case GLUT_RIGHT_BUTTON:
    DragTruck(dx,dy);
    break;
  default:
    return;
  }
}

void GLNavigationBackend::DoAltDrag(int dx,int dy,int button)
{}

void GLNavigationBackend::DoShiftDrag(int dx,int dy,int button)
{}


void GLNavigationBackend::DragPan(int dx,int dy)
{
  Vector3 v;
  viewport.getMovementVectorAtDistance(-dx,dy,camera.dist,v);
  camera.tgt+=v;
  //viewport.scroll(dx,dy);
  SHOW_VIEW_TARGET(0.5);
  SendRefresh();
}

void GLNavigationBackend::DragRotate(int dx,int dy)
{
  camera.rot.y-=DtoR((Real)dx);
  camera.rot.x-=DtoR((Real)dy);
  SHOW_VIEW_TARGET(0.5);
  SendRefresh();
}

void GLNavigationBackend::DragZoom(int dx,int dy)
{
  viewport.scale *= (1+float(dy)*0.01f);
  SHOW_VIEW_TARGET(0.5);
  SendRefresh();
}

void GLNavigationBackend::DragTruck(int dx,int dy)
{
  Vector3 v(viewport.zDir());
  camera.tgt.madd(v,Real(dy)/viewport.scale/**camera.dist*/);
  SHOW_VIEW_TARGET(0.5);
  SendRefresh();
}


bool GLNavigationBackend::OnIdle()
{
  double old_time = timer.LastElapsedTime();
  double new_time = timer.ElapsedTime();
  double delta_time=new_time-old_time;
  if(delta_time>0)
    frames_per_second = float(1.0/delta_time);
  else
    frames_per_second = 0.0f;

  if(show_view_target && t_hide_view_target <= new_time) {
    show_view_target = 0;
    SendRefresh();
  }
  if(t_hide_view_target <= new_time) 
    SendPauseIdle();
  else 
    SendPauseIdle(t_hide_view_target - new_time);
  return true;
}

void GLNavigationBackend::DisplayCameraTarget()
{
  glMatrixMode (GL_MODELVIEW);
  glPushMatrix();
  glTranslatef(camera.tgt.x,camera.tgt.y,camera.tgt.z);
  glDisable(GL_LIGHTING);
  GLfloat clear_color[4];
  glGetFloatv(GL_COLOR_CLEAR_VALUE,clear_color);
  glColor3f(1-clear_color[0],1-clear_color[1],1-clear_color[2]);
  double logcd=log(camera.dist)/log(20.);
  float smallsize=float(pow(20.0,floor(logcd-.5)));
  drawWireBox(smallsize,smallsize,smallsize);
  drawWireBox(20*smallsize,20*smallsize,20*smallsize);
  glEnable(GL_LIGHTING); 
  glPopMatrix();
}

void GLNavigationBackend::Set2DMode(bool mode)
{
  mode_2d=mode;
}

void GLNavigationBackend::CenterCameraOn(const AABB3D& aabb)
{
  aabb.getMidpoint(camera.tgt);
}

void GLNavigationBackend::WriteDisplaySettings(ostream& out) const
{
  out<<viewport<<endl;
  out<<"ORBITDIST "<<camera.dist<<endl;
}

void GLNavigationBackend::ReadDisplaySettings(istream& in)
{
  //viewport should not change
  int x=viewport.x,y=viewport.y,w=viewport.w,h=viewport.h;
  in>>viewport;
  string str;
  in>>str;
  if(str != "ORBITDIST") { in.setstate(ios::badbit); return; }
  in>>camera.dist;

  camera.fromCamera(viewport,camera.dist);

  //restore viewport
  viewport.x=x;
  viewport.y=y;
  viewport.w=w;
  viewport.h=h;

  //request resize -- may or may not succeed!
  SendResize(w,h);
}
