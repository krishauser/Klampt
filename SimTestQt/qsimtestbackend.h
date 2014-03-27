#ifndef QSIMTESTBACKEND_H
#define QSIMTESTBACKEND_H

#include <QGLWidget>
#include <QKeyEvent>

#include <Interface/SimTestGUI.h>
//#include <GLdraw/GLScreenshotProgram.h>

#include <GLScreenshotPlugin.h>

class QSimTestBackend : public QGLWidget, public SimTestBackend, public GLScreenshotPlugin
{
    Q_OBJECT
public:
    typedef SimTestBackend BaseT;
    explicit QSimTestBackend(QWidget *parent = 0);
    
    void initializeGL();
    void paintGL();
    bool OnIdle();
    void keyPressEvent(QKeyEvent *e);
    bool OnMouseMove(int mx, int my);
    bool OnMouseWheel(int dwheel);
    void wheelEvent(QWheelEvent *e);
    void resizeGL(int w, int h);
    bool OnButtonToggle(const string &button, int checked);
    void keyReleaseEvent(QKeyEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void mousePressEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void enterEvent(QEvent *);
    bool OnCommand(const string &cmd, const string &args);
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

#endif // QSIMTESTBACKEND_H
