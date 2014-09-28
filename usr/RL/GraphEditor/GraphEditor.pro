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
    ParserOld.cpp \
    Parser.cpp \
    util.cpp

HEADERS  += Editor.h \
    ParserOld.h \
    Parser.h \
    util.h

FORMS    += Editor.ui

QMAKE_CXXFLAGS += -std=c++0x
