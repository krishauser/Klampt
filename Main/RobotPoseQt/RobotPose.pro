#-------------------------------------------------
#
# Project created by QtCreator 2014-02-04T20:00:59
#
#-------------------------------------------------

QT       += core gui opengl

TARGET = RobotPose

TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    qrobottestbackend.cpp \
    ../KlamptQt/qtguibase.cpp \
    qrobottestguibase.cpp \
    collisionoutput.cpp \
    resourceframe.cpp \
    qresourcetreeitem.cpp \
    resourcemanager.cpp \ #\
    playresourceframe.cpp
   # ../../Interface/RobotTestGUI.cpp

HEADERS  += mainwindow.h \
    qrobottestbackend.h \
    ../KlamptQt/qtguibase.h \
    qrobottestguibase.h \
    collisionoutput.h \
    resourceframe.h \
    qresourcetreeitem.h \
    resourcemanager.h \ #\
    playresourceframe.h
    #../../Interface/RobotTestGUI.h

FORMS    += mainwindow.ui \
    collisionoutput.ui \
    resourceframe.ui \
    playresourceframe.ui

include(../KlamptQt/common.pri)

LIBS += -lKlampt -lKrisLibrary -lglut -lGL -lm -lGLU -lXm -lXi -lXext -lXmu -lX11 -lode -ltinyxml -lglpk -lassimp -lglui

DEFINES += dDOUBLE

QMAKE_CFLAGE_DEGUG += -g

RESOURCES += \
    ../KlamptQt/icons.qrc
