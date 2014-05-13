#ifndef QROBOTTESTBACKEND
#define QROBOTTESTBACKEND

#include <QGLWidget>
#include <QKeyEvent>

#include <Interface/RobotPoseGUI.h>
#include <resourcemanager.h>
#include <View/ViewResource.h>
#include <KlamptQt/GLScreenshotPlugin.h>
//#include <GLdraw/GLScreenshotProgram.h>

class QRobotPoseBackend : public QGLWidget, public RobotPoseBackend, public GLScreenshotPlugin
{
    Q_OBJECT
public:
    typedef RobotPoseBackend BaseT;
    explicit QRobotPoseBackend(QWidget *parent = 0);
    ViewResource viewResource;

    void initializeGL();
    void paintGL();
    bool OnIdle();
    void keyPressEvent(QKeyEvent *e);
    bool OnMouseMove(int mx, int my);
    bool OnMouseWheel(int dwheel);
    void wheelEvent(QWheelEvent *e);
    void resizeGL(int w, int h);
//    bool OnButtonToggle(const string &button, int checked);
    void keyReleaseEvent(QKeyEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void mousePressEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void enterEvent(QEvent *);
    //    bool OnCommand(const string &cmd, const string &args);
    bool OnCommand(const string &cmd, const string &args);
    void RenderCurrentResource();
    void RenderWorld();
    void SetPathTime(double time);
signals:
    void ResizeFrame(QResizeEvent *e);
    void MouseMove(QMouseEvent *e);
    void MouseWheel(QWheelEvent *e);
    void KeyPress(QKeyEvent *e);
    void KeyRelease(QKeyEvent *e);
    void MousePress(QMouseEvent *e);
    void MouseRelease(QMouseEvent *e);
public slots:
    
};

#endif // QROBOTTESTBACKEND_H
