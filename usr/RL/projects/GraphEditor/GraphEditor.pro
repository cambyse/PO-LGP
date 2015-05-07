#-------------------------------------------------
#
# Project created by QtCreator 2014-09-24T14:38:44
#
#-------------------------------------------------

QT       += core gui svg

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GraphEditor
TEMPLATE = app


SOURCES += main.cpp\
        Editor.cpp \
    Parser.cpp \
    util.cpp

HEADERS  += Editor.h \
    Parser.h \
    util.h

FORMS    += Editor.ui

QMAKE_CXXFLAGS += -std=c++0x
