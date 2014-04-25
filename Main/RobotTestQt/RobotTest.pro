#-------------------------------------------------
#
# Project created by QtCreator 2014-02-04T20:00:59
#
#-------------------------------------------------

QT       += core gui opengl

TARGET = RobotTest

TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    qrobottestbackend.cpp \
    ../KlamptQt/qtguibase.cpp \
    collisionoutput.cpp \
    qrobottestgui.cpp

HEADERS  += mainwindow.h \
    qrobottestbackend.h \
    ../KlamptQt/qtguibase.h \
    collisionoutput.h \
    qrobottestgui.h

FORMS    += mainwindow.ui \
    collisionoutput.ui

include(../KlamptQt/common.pri)

QMAKE_CFLAGE_DEGUG += -g

RESOURCES += \
    ../KlamptQt/icons.qrc
