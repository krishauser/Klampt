#include "qsimtestbackend.h"
#include "GLdraw/GLView.h"

/** @brief QSimTestBackend.
 *
 * Messages are defined as follows.
 * 
 * command:
 * - record(status): turns the recording on or off
 * - set_record_command(command): sets the system call to use while recording
 * - record_file(filename): sets the record file location
 */

QSimTestBackend::QSimTestBackend(QWidget *parent) :
    QGLWidget(parent),SimTestBackend(new RobotWorld()),GLScreenshotPlugin()
{
    setMouseTracking(true);
    MapButtonToggle("simulate",&simulate);
}

void QSimTestBackend::initializeGL(){
    glClearColor(.4,.4,1,1);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_MULTISAMPLE);
    static GLfloat lightPosition[4] = { 0.5, 5.0, 7.0, 1.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}

bool QSimTestBackend::OnCommand(const string &cmd, const string &args){
  //todo make recordgui class
    stringstream ss(args);
    if(cmd=="record"){
        int status;
        ss >> status;
        if(status){
            StartMovie();
        }
        else{
            StopMovie();
        }
        return true;
    }
    else if(cmd=="set_record_command"){
        GLScreenshotPlugin::video_encoding_command = args;
        return true;
    }
    else if(cmd=="record_file"){
        moviefile=args;
    }
    else return BaseT::OnCommand(cmd,args);
}

//widget specific function
void QSimTestBackend::paintGL(){
    OnGLRender();
}

bool QSimTestBackend::OnIdle(){
    MovieUpdate(sim.time);
    BaseT::OnIdle();
    return true;
}

bool QSimTestBackend::OnMouseMove(int mx, int my){
    BaseT::OnMouseMove(mx,my);
}

bool QSimTestBackend::OnMouseWheel(int dwheel){
    viewport.scale *=(1+dwheel/1000.0);
//    DragZoom(0,dwheel);
    //ToggleSensorPlot(0,1);
    SendRefresh();
    return true;
}


void QSimTestBackend::resizeGL(int w, int h){
    //emit ResizeFrame(e);
    viewport.w=w;
    viewport.h=h;
    glViewport(0,0,(GLsizei)w,(GLsizei)h);
}

bool QSimTestBackend::OnButtonToggle(const string &button, int checked){
    BaseT::OnButtonToggle(button,checked);
}

//GUI functionality: Possible new file

void QSimTestBackend::keyPressEvent(QKeyEvent *e){
    emit KeyPress(e);
}

void QSimTestBackend::keyReleaseEvent(QKeyEvent *e){
    emit KeyRelease(e);
}

void QSimTestBackend::mouseMoveEvent(QMouseEvent *e){
    emit MouseMove(e);
}

void QSimTestBackend::mousePressEvent(QMouseEvent *e){
    emit MousePress(e);
}

void QSimTestBackend::mouseReleaseEvent(QMouseEvent *e){
    emit MouseRelease(e);
}

void QSimTestBackend::wheelEvent(QWheelEvent *e){
    emit MouseWheel(e);
}

void QSimTestBackend::enterEvent(QEvent *){
    this->focusWidget();
}
