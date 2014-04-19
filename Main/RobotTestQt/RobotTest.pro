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
    qtguibase.cpp \
    qrobottestguibase.cpp \
    collisionoutput.cpp

HEADERS  += mainwindow.h \
    qrobottestbackend.h \
    qtguibase.h \
    qrobottestguibase.h \
    collisionoutput.h

FORMS    += mainwindow.ui \
    collisionoutput.ui

include(../KlamptQt/common.pri)

LIBS += -lKlampt -lKrisLibrary -lglut -lGL -lm -lGLU -lXm -lXi -lXext -lXmu -lX11 -lode -ltinyxml -lglpk -lassimp -lglui

DEFINES += dDOUBLE

QMAKE_CFLAGE_DEGUG += -g

RESOURCES += \
    ../KlamptQt/icons.qrc
