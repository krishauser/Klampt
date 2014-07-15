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
    ../urdftorob.cpp


HEADERS  += \
    dialog.h \
    ../urdftorob.h

FORMS    += \
    dialog.ui

include(../KlamptQt/common.pri)
LIBS += -lKlampt -lKrisLibrary -lglut -lGL -lm -lGLU -lXm -lXi -lXext -lXmu -lX11 -lode -ltinyxml -lglpk -lassimp -lglui

DEFINES += HAVE_QT
