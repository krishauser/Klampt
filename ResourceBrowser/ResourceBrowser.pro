#-------------------------------------------------
#
# Project created by QtCreator 2014-04-03T15:50:09
#
#-------------------------------------------------

QT       += core gui opengl

TARGET = ResourceBrowser
TEMPLATE = app


SOURCES += main.cpp\
    resourcedockwidget.cpp \
    multipathdock.cpp \
    linearpathdock.cpp \
    stancedock.cpp \
    holddock.cpp \
    qresourcetreeitem.cpp \
    mainwindow.cpp

HEADERS  += \
    resourcedockwidget.h \
    multipathdock.h \
    linearpathdock.h \
    stancedock.h \
    holddock.h \
    qresourcetreeitem.h \
    mainwindow.h

FORMS    += \
    resourcedockwidget.ui \
    mainwindow.ui

#RobotTest ../RobotTestQt/
SOURCES +=  ../RobotTestQt/mainwindow.cpp \
    ../RobotTestQt/qrobottestbackend.cpp \
   ../RobotTestQt/qtguibase.cpp \
    ../RobotTestQt/qrobottestguibase.cpp \
    ../RobotTestQt/collisionoutput.cpp

HEADERS  += mainwindow.h \
    ../RobotTestQt/qrobottestbackend.h \
    ../RobotTestQt/qtguibase.h \
    ../RobotTestQt/qrobottestguibase.h \
    ../RobotTestQt/collisionoutput.h

FORMS    += ../RobotTestQt/mainwindow.ui \
    ../RobotTestQt/collisionoutput.ui
INCLUDEPATH += ../RobotTestQt

LIBS += -L../lib -lKlampt  -L/usr/lib -L../Library/glui-2.36/src/lib -L/usr/lib/glut -L/usr/X11R6/lib -L/usr/X11R6/lib/modules/extensions -L/src -L../Library/KrisLibrary/lib -L../Library/ode-0.11.1/ode/src/.libs -L../Library/tinyxml
LIBS += -lKrisLibrary -lglut -lGL -lm -lGLU -lXm -lXi -lXext -lXmu -lX11 -lode -ltinyxml -lglpk
LIBS += -L../Library/glui-2.36/src/include -lglui -lGLU -lassimp
INCLUDEPATH +=/usr/include ../ ../Library/KrisLibrary ../Library/glui-2.36/src/include ../Library/ode0.11.1/ode/src
DEFINES += dDOUBLE

#! include( ../common.pri ) {
#    error( Couldn't find the common.pri file! )
#}

RESOURCES += \
    ../SimTestQt/icons.qrc
