#include "qrobottestbackend.h"
#include "GLdraw/GLView.h"

QRobotTestBackend::QRobotTestBackend(QWidget *parent) :
      QGLWidget(parent),RobotTestBackend(new RobotWorld())//,GLScreenshotPlugin()
{
    setMouseTracking(true);
}

void QRobotTestBackend::initializeGL(){
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

//widget specific function
void QRobotTestBackend::paintGL(){
    OnGLRender();
}

bool QRobotTestBackend::OnIdle(){
    //MovieUpdate(sim.time);
    BaseT::OnIdle();
    return true;
}

bool QRobotTestBackend::OnMouseMove(int mx, int my){
    BaseT::OnMouseMove(mx,my);
}

bool QRobotTestBackend::OnMouseWheel(int dwheel){
    viewport.scale *=(1+dwheel/1000.0);
//    DragZoom(0,dwheel);
    //ToggleSensorPlot(0,1);
    SendRefresh();
    return true;
}

bool QRobotTestBackend::OnCommand(const string &cmd, const string &args){
    if(cmd=="set_q"){
        Config q;
        stringstream ss(args);
        while(ss>>q);
        cout<<q.n<<robotWidgets[0].Pose().n<<"\n";
        robotWidgets[0].SetPose(q);
        SendCommand("update_config","");
    }
    return BaseT::OnCommand(cmd,args);
}

void QRobotTestBackend::resizeGL(int w, int h){
    //emit ResizeFrame(e);
    viewport.w=w;
    viewport.h=h;
    glViewport(0,0,(GLsizei)w,(GLsizei)h);
}

bool QRobotTestBackend::OnButtonToggle(const string &button, int checked){
    BaseT::OnButtonToggle(button,checked);
}

//GUI functionality: Possible new file

void QRobotTestBackend::keyPressEvent(QKeyEvent *e){
    emit KeyPress(e);
}

void QRobotTestBackend::keyReleaseEvent(QKeyEvent *e){
    emit KeyRelease(e);
}

void QRobotTestBackend::mouseMoveEvent(QMouseEvent *e){
    emit MouseMove(e);
}

void QRobotTestBackend::mousePressEvent(QMouseEvent *e){
    emit MousePress(e);
}

void QRobotTestBackend::mouseReleaseEvent(QMouseEvent *e){
    emit MouseRelease(e);
}

void QRobotTestBackend::wheelEvent(QWheelEvent *e){
    emit MouseWheel(e);
}

void QRobotTestBackend::enterEvent(QEvent *){
    this->focusWidget();
}
