CONFIG += debug
TARGET = Instance_Example
QT += core \
    gui \
    svg
HEADERS += \
    ../Representation/Action.h \
    ../Representation/State.h \
    ../Representation/Reward.h \
    ../Representation/Instance.h \
    ../util.h
SOURCES += \
    Instance_Example.cpp \
    ../Representation/Action.cpp \
    ../Representation/State.cpp \
    ../Representation/Reward.cpp \
    ../Representation/Instance.cpp \
    ../util.cpp
LIBS +=
FORMS +=
RESOURCES +=
QMAKE_CXXFLAGS += -std=c++0x
