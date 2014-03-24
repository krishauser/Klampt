#-------------------------------------------------
#
# Project created by QtCreator 2014-03-13T00:38:28
#
#-------------------------------------------------

QT       += core gui

TARGET = URDFtoRob
TEMPLATE = app


SOURCES += main.cpp\
    dialog.cpp \
    ../Main/urdftorob.cpp


HEADERS  += \
    dialog.h \
    ../Main/urdftorob.h

FORMS    += \
    dialog.ui

INCLUDEPATH += ../ ../Library/KrisLibrary
LIBS += -L../lib -L../Library/KrisLibrary/lib -lKrisLibrary -lKlampt


LIBS += -L../lib -lKlampt  -L/usr/lib -L../Library/glui-2.36/src/lib -L/usr/lib/glut -L/usr/X11R6/lib -L/usr/X11R6/lib/modules/extensions -L/src -L../Library/KrisLibrary/lib -L../Library/ode-0.11.1/ode/src/.libs -L../Library/tinyxml
LIBS += -lKrisLibrary -lglut -lGL -lm -lGLU -lXm -lXi -lXext -lXmu -lX11 -lode -ltinyxml -lglpk -lassimp

DEFINES += HAVE_QT
