#-------------------------------------------------
#
# Project created by QtCreator 2014-02-04T20:00:59
#
#-------------------------------------------------

QT       += core gui opengl

TARGET = SimTest
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    qsimtestbackend.cpp \
    logoptions.cpp \
    driveredit.cpp \
    controllersettings.cpp \
    controllercommanddialog.cpp \
    ../KlamptQt/qtguibase.cpp \
    qsimtestgui.cpp

HEADERS  += mainwindow.h \
    qsimtestbackend.h \
    logoptions.h \
    driveredit.h \
    controllersettings.h \
    GLScreenshotPlugin.h \
    controllercommanddialog.h \
    ../KlamptQt/qtguibase.h \
    qsimtestgui.h

FORMS    += mainwindow.ui \
    logoptions.ui \
    driveredit.ui \
    controllersettings.ui \
    controllercommanddialog.ui

include(../KlamptQt/common.pri)


LIBS += -lKlampt -lKrisLibrary -lglut -lGL -lm -lGLU -lXm -lXi -lXext -lXmu -lX11 -lode -ltinyxml -lglpk
LIBS += -L../Library/glui-2.36/src/include -lglui -lGLU -lassimp
INCLUDEPATH +=/usr/include ../ ../Library/KrisLibrary ../Library/glui-2.36/src/include ../Library/ode0.11.1/ode/src
DEFINES += dDOUBLE

QMAKE_CFLAGE_DEGUG += -g

RESOURCES += \
    icons.qrc
