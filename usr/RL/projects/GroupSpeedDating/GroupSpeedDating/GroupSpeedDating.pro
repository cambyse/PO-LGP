#-------------------------------------------------
#
# Project created by QtCreator 2015-02-23T14:31:52
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GroupSpeedDating
TEMPLATE = app


SOURCES += main.cpp\
        GroupSpeedDating.cpp

HEADERS  += GroupSpeedDating.h

FORMS    += GroupSpeedDating.ui

unix:!macx: LIBS += -L$$PWD/../../../../../../../../MLR/build/debug/util/ -lutil

INCLUDEPATH += $$PWD/../../../../../../../../MLR/src
DEPENDPATH += $$PWD/../../../../../../../../MLR/src

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../../../../../MLR/build/debug/util/libutil.a

QMAKE_CXXFLAGS += -std=c++0x
