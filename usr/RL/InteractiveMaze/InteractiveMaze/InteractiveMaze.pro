#-------------------------------------------------
#
# Project created by QtCreator 2014-06-23T05:44:57
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = InteractiveMaze
TEMPLATE = app


SOURCES += main.cpp\
        InteractiveMaze.cpp \
    InputFilter.cpp

HEADERS  += InteractiveMaze.h \
    InputFilter.h

FORMS    += InteractiveMaze.ui

QMAKE_CXXFLAGS += -std=c++0x

LIBS += -larmadillo
