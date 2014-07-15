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
    ../KlamptQt/qtguibase.cpp \
    collisionoutput.cpp \
    resourceframe.cpp \
    qresourcetreeitem.cpp \
    resourcemanager.cpp \ 
    playresourceframe.cpp \
    qrobotposebackend.cpp \
    qrobotposegui.cpp

HEADERS  += mainwindow.h \
    ../KlamptQt/qtguibase.h \
    collisionoutput.h \
    resourceframe.h \
    qresourcetreeitem.h \
    resourcemanager.h \ 
    playresourceframe.h \
    qrobotposebackend.h \
    qrobotposegui.h \
    ../KlamptQt/GLScreenshotPlugin.h

FORMS    += mainwindow.ui \
    collisionoutput.ui \
    resourceframe.ui \
    playresourceframe.ui

include(../KlamptQt/common.pri)

DEFINES += dDOUBLE

QMAKE_CFLAGE_DEGUG += -g

RESOURCES += \
    ../KlamptQt/icons.qrc
