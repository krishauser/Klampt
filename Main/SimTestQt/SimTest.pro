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
    qsimtestgui.cpp \
    connectserial.cpp

HEADERS  += mainwindow.h \
    qsimtestbackend.h \
    logoptions.h \
    driveredit.h \
    controllersettings.h \
    GLScreenshotPlugin.h \
    controllercommanddialog.h \
    ../KlamptQt/qtguibase.h \
    qsimtestgui.h \
    connectserial.h \
    ../KlamptQt/GLScreenshotPlugin.h

FORMS    += mainwindow.ui \
    logoptions.ui \
    driveredit.ui \
    controllersettings.ui \
    controllercommanddialog.ui \
    connectserial.ui

include(../KlamptQt/common.pri)

QMAKE_CFLAGE_DEGUG += -g

RESOURCES += \
    icons.qrc
