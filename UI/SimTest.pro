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
    qtguibase.cpp \
    logoptions.cpp \
    driveredit.cpp \
    controllersettings.cpp

HEADERS  += mainwindow.h \
    qsimtestbackend.h \
    qtguibase.h \
    logoptions.h \
    driveredit.h \
    controllersettings.h \
    GLScreenshotPlugin.h

FORMS    += mainwindow.ui \
    logoptions.ui \
    driveredit.ui \
    controllersettings.ui

LIBS += -L../lib -lKlampt  -L/usr/lib -L../Library/glui-2.36/src/lib -L/usr/lib/glut -L/usr/X11R6/lib -L/usr/X11R6/lib/modules/extensions -L/src -L../Library/KrisLibrary/lib -L../Library/ode-0.11.1/ode/src/.libs -L../Library/tinyxml
LIBS += -lKrisLibrary -lglut -lGL -lm -lGLU -lXm -lXi -lXext -lXmu -lX11 -lode -ltinyxml -lglpk
LIBS += -L../Library/glui-2.36/src/include -lglui -lGLU -lassimp
INCLUDEPATH +=/usr/include ../ ../Library/KrisLibrary ../Library/glui-2.36/src/include ../Library/ode0.11.1/ode/src
DEFINES += dDOUBLE

QMAKE_CFLAGE_DEGUG += -g

RESOURCES += \
    icons.qrc
