CONFIG += debug
TARGET = Commander_Example
QT += core
HEADERS += \
    ../util/Commander.h \
    ../util/QtUtil.h \
    ../util/debug.h \
    ../util/ColorOutput.h \
    ../util/function_signature.h
SOURCES += \
    Commander_Example.cpp \
    ../util/Commander.cpp \
    ../util/QtUtil.cpp \
    ../util/ColorOutput.cpp
LIBS +=
FORMS +=
RESOURCES +=
QMAKE_CXXFLAGS += -std=c++0x
