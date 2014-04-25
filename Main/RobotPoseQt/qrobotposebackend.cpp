#include "qrobotposebackend.h"
#include "GLdraw/GLView.h"

QRobotPoseBackend::QRobotPoseBackend(QWidget *parent) :
    QGLWidget(parent),RobotPoseBackend(new RobotWorld(),new ResourceLibrary())//,GLScreenshotPlugin()
{
    setMouseTracking(true);
}

void QRobotPoseBackend::initializeGL(){
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
void QRobotPoseBackend::paintGL(){
    OnGLRender();
}

bool QRobotPoseBackend::OnIdle(){
    //MovieUpdate(sim.time);
    BaseT::OnIdle();
    return true;
}

bool QRobotPoseBackend::OnMouseMove(int mx, int my){
    BaseT::OnMouseMove(mx,my);
}

bool QRobotPoseBackend::OnMouseWheel(int dwheel){
    viewport.scale *=(1+dwheel/1000.0);
//    DragZoom(0,dwheel);
    //ToggleSensorPlot(0,1);
    SendRefresh();
    return true;
}

bool QRobotPoseBackend::OnCommand(const string &cmd, const string &args){
    if(cmd=="set_q"){
        Config q;
        stringstream ss(args);
        while(ss>>q);
        cout<<q.n<<robotWidgets[0].Pose().n<<"\n";
        robotWidgets[0].SetPose(q);
        SendCommand("update_config","");
        return 1;
    }
    return BaseT::OnCommand(cmd,args);
}

void QRobotPoseBackend::resizeGL(int w, int h){
    //emit ResizeFrame(e);
    viewport.w=w;
    viewport.h=h;
    glViewport(0,0,(GLsizei)w,(GLsizei)h);
}

void QRobotPoseBackend::RenderCurrentResource(){
    if(manager->open == NULL) return;
    viewResource.DrawGL(manager->open->resource);
}

void QRobotPoseBackend::RenderWorld(){
    RobotPoseBackend::RenderWorld();
    RenderCurrentResource();
}


//GUI functionality: Possible new file

void QRobotPoseBackend::keyPressEvent(QKeyEvent *e){
    emit KeyPress(e);
}

void QRobotPoseBackend::keyReleaseEvent(QKeyEvent *e){
    emit KeyRelease(e);
}

void QRobotPoseBackend::mouseMoveEvent(QMouseEvent *e){
    emit MouseMove(e);
}

void QRobotPoseBackend::mousePressEvent(QMouseEvent *e){
    emit MousePress(e);
}

void QRobotPoseBackend::mouseReleaseEvent(QMouseEvent *e){
    emit MouseRelease(e);
}

void QRobotPoseBackend::wheelEvent(QWheelEvent *e){
    emit MouseWheel(e);
}

void QRobotPoseBackend::enterEvent(QEvent *){
    this->focusWidget();
}
