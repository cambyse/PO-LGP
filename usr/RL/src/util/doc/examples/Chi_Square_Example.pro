CONFIG += debug
TARGET = Chi_Square_Example
QT += core \
    gui \
    svg
HEADERS += \
    ../util/ChiSquareTest.h
SOURCES += \
    Chi_Square_Example.cpp \
    ../util/ChiSquareTest.cpp
INCLUDEPATH +=
LIBS +=
FORMS +=
RESOURCES +=
QMAKE_CXXFLAGS += -std=c++0x

