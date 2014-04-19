#-------------------------------------------------
#
# Project created by QtCreator 2014-03-13T00:21:44
#
#-------------------------------------------------

QT       += core gui

TARGET = MotorCalibrate
TEMPLATE = app


SOURCES += main.cpp\
    dialog.cpp \
    ../motorcalibrate.cpp \
    showtext.cpp

HEADERS  += \
    dialog.h \
    ../motorcalibrate.h \
    showtext.h

FORMS    += \
    dialog.ui \
    showtext.ui

include(../KlamptQt/common.pri)

LIBS += -lKlampt -lKrisLibrary -lglut -lGL -lm -lGLU -lXm -lXi -lXext -lXmu -lX11 -lode -ltinyxml -lglpk -lassimp -lglui

DEFINES += HAVE_QT
