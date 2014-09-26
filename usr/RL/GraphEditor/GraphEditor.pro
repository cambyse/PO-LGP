#-------------------------------------------------
#
# Project created by QtCreator 2014-09-24T14:38:44
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GraphEditor
TEMPLATE = app


SOURCES += main.cpp\
        Editor.cpp \
        Parser.cpp \
    QtUtil.cpp

HEADERS  += Editor.h \
        Parser.h \
    QtUtil.h

FORMS    += Editor.ui

QMAKE_CXXFLAGS += -std=c++0x
