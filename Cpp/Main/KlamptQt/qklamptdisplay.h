#ifndef QKLAMPTDISPLAY_H
#define QKLAMPTDISPLAY_H

#include <QGLWidget>
#include <QKeyEvent>
#include "GLScreenshotPlugin.h"
#include "Interface/GenericGUI.h"
using namespace Klampt;

class QKlamptDisplay : public QGLWidget, public GLScreenshotPlugin
{
    Q_OBJECT
public:
    explicit QKlamptDisplay(QWidget *parent = 0);
    virtual ~QKlamptDisplay() {}

    GenericGUIBase* gui;
    bool painted;

    void SetGUI(GenericGUIBase* gui);
    void SetVideoOutputFile(const std::string& fn);
    void SetVideoEncoding(const std::string& args);
    virtual void initializeGL();
    virtual void paintGL();
    virtual void resizeGL(int w, int h);
public slots:
    void keyPressEvent(QKeyEvent *e);
    void wheelEvent(QWheelEvent *e);
    void keyReleaseEvent(QKeyEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void mousePressEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void enterEvent(QEvent *);    
};

#endif // QKLAMPTDISPLAY_H
