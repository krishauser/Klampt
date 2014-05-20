#include "qklamptdisplay.h"
#include <QCoreApplication>

/** @brief QKlamptDisplay.
 */

QKlamptDisplay::QKlamptDisplay(QWidget *parent) :
  QGLWidget(parent),GLScreenshotPlugin(),gui(NULL)
{
    setMouseTracking(true);
}

void QKlamptDisplay::initializeGL(){
    glClearColor(.4,.4,1,1);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    //glEnable(GL_MULTISAMPLE);
    static GLfloat lightPosition[4] = { 0.5, 5.0, 7.0, 1.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}

void QKlamptDisplay::paintGL(){
  glClear(0);
  if(gui != NULL)
    gui->SendGLRender();
}


void QKlamptDisplay::resizeGL(int w, int h){
  if(gui == NULL) return;
  gui->SendGLViewport(0,0,w,h);
}

void QKlamptDisplay::SetVideoEncoding(const std::string& args)
{
  GLScreenshotPlugin::video_encoding_command = args;
}

void QKlamptDisplay::SetVideoOutputFile(const std::string& fn)
{
  GLScreenshotPlugin::moviefile = fn;
}




void QKlamptDisplay::keyPressEvent(QKeyEvent *e){
  if(gui == NULL) return;
  int key = e->key();
  if(key==Qt::Key_Shift || key==Qt::Key_Control){
    return;}
  if(!(e->modifiers() & Qt::ShiftModifier))key=tolower(key);
  gui->SendKeyDown(string(1,key));
}

void QKlamptDisplay::keyReleaseEvent(QKeyEvent *e){
  if(gui == NULL) return;
  int key = e->key();
  if(!(e->modifiers() & Qt::ShiftModifier))key=tolower(key);
  gui->SendKeyUp(string(1,key));
}

void QKlamptDisplay::mouseMoveEvent(QMouseEvent *e){
  if(gui == NULL) return;
  gui->SendMouseMove(e->x(),e->y());
}

void QKlamptDisplay::mousePressEvent(QMouseEvent *e){
  int button=e->button();
  if(button==1) button=0;
  if(e->modifiers()&Qt::ControlModifier)
    gui->SendKeyDown("control");
  else
    gui->SendKeyUp("control");
  if(e->modifiers()&Qt::ShiftModifier)
    gui->SendKeyDown("shift");
  else
      gui->SendKeyUp("shift");
  if(e->modifiers()&Qt::AltModifier)
    gui->SendKeyDown("alt");
  else
    gui->SendKeyUp("alt");
  gui->SendMouseClick(button,1,e->x(),e->y());
}

void QKlamptDisplay::mouseReleaseEvent(QMouseEvent *e){
  if(gui == NULL) return;
  int button=e->button();
  if(button==1) button=0;
  gui->SendMouseClick(button,0,e->x(),e->y());
}

void QKlamptDisplay::wheelEvent(QWheelEvent *e){
  if(gui == NULL) return;
  gui->SendMouseWheel(e->delta());
}

void QKlamptDisplay::enterEvent(QEvent *){
    this->focusWidget();
}
