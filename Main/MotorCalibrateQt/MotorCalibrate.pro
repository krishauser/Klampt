#-------------------------------------------------
#
# Project created by QtCreator 2014-03-13T00:21:44
#
#-------------------------------------------------

QT       += core gui opengl

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

DEFINES += HAVE_QT
