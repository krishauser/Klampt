#include "qklamptdisplay.h"
#include <QCoreApplication>

/** @brief QKlamptDisplay.
 */

QKlamptDisplay::QKlamptDisplay(QWidget *parent) :
  QGLWidget(parent),GLScreenshotPlugin(),gui(NULL),painted(true)
{
    setMouseTracking(true);
}

void QKlamptDisplay::SetGUI(GenericGUIBase* _gui)
{
  gui = _gui;
}

void QKlamptDisplay::initializeGL(){
    glClearColor(.4,.4,1,1);
#ifdef GL_MULTISAMPLE
    glEnable(GL_MULTISAMPLE);
#elif defined(GL_MULTISAMPLE_ARB)
	glEnable(GL_MULTISAMPLE_ARB);
#endif //GL_MULTISAMPLE
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    static GLfloat lightPosition[4] = { 0.5, 5.0, 7.0, 1.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}

void QKlamptDisplay::paintGL(){
  if(gui != NULL) 
    gui->SendGLRender();
  else
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
  painted = true;
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


string QtKeyToString(int key)
{
  switch(key) {
  case Qt::Key_Shift:
    return "shift";
  case Qt::Key_Control:
    return "control";
  case Qt::Key_Alt:
    return "alt";
  case Qt::Key_Escape:
    return "escape";
  case Qt::Key_Delete:
    return "delete";
  case Qt::Key_Backspace:
    return "backspace";
  case Qt::Key_Backtab:
    return "backtab";
  case Qt::Key_Tab:
    return "tab";
  case Qt::Key_Return:
    return "return";
  case Qt::Key_Enter:
    return "enter";
  case Qt::Key_Home:
    return "home";
  case Qt::Key_End:
    return "end";
  case Qt::Key_Left:
    return "left";
  case Qt::Key_Right:
    return "right";
  case Qt::Key_Up:
    return "up";
  case Qt::Key_Down:
    return "down";
  case Qt::Key_PageUp:
    return "pageup";
  case Qt::Key_PageDown:
    return "pagedown";
  case Qt::Key_Meta:
    return "meta";
  }
  if(key < 256)
    return string(1,char(key));
  return "";
}



void QKlamptDisplay::keyPressEvent(QKeyEvent *e){
  if(gui == NULL) return;
  if(e->text().isEmpty()) {
    int key = e->key();
    string str = QtKeyToString(key);
    if(!str.empty())
      gui->SendKeyDown(str);
  }
  else {
    gui->SendKeyDown(e->text().toStdString());
  }
}

void QKlamptDisplay::keyReleaseEvent(QKeyEvent *e){
  if(gui == NULL) return;
  int key = e->key();
  string str = QtKeyToString(key);
  if(!str.empty())
    gui->SendKeyUp(str);
}

void QKlamptDisplay::mouseMoveEvent(QMouseEvent *e){
  if(gui == NULL) return;
#if QT_VERSION >= 0x050000
  int scale=devicePixelRatio();
#else
  int scale=1;
#endif
  gui->SendMouseMove(e->x()*scale,e->y()*scale);
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
#if QT_VERSION >= 0x050000
  int scale=devicePixelRatio();
#else
  int scale=1;
#endif
  gui->SendMouseClick(button,1,e->x()*scale,e->y()*scale);
}

void QKlamptDisplay::mouseReleaseEvent(QMouseEvent *e){
  if(gui == NULL) return;
  int button=e->button();
  if(button==1) button=0;
#if QT_VERSION >= 0x050000
  int scale=devicePixelRatio();
#else
  int scale=1;
#endif
  gui->SendMouseClick(button,0,e->x()*scale,e->y()*scale);
}

void QKlamptDisplay::wheelEvent(QWheelEvent *e){
  if(gui == NULL) return;
  gui->SendMouseWheel(e->delta());
}

void QKlamptDisplay::enterEvent(QEvent *){
    this->focusWidget();
}
