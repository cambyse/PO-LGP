TEMPLATE = app
TARGET = LangeNacht
QT += core \
    gui
HEADERS += WorldModel/StateActionValueStateModel.h \
    WorldModel/AbstractState.h \
    WorldModel/StateActionValueFunction.h \
    WorldModel/MyDisplay.h \
    WorldModel/debug_exclude.h \
    WorldModel/GridworldModel.h \
    WorldModel/GraphModel.h \
    WorldModel/NodeStateModel.h \
    WorldModel/GridworldActionModel.h \
    WorldModel/GridworldStateModel.h \
    WorldModel/debug.h \
    WorldModel/TransitionGraph.h \
    langenacht.h
SOURCES += WorldModel/GridworldStateModel.cpp \
    WorldModel/GridworldModel.cpp \
    main.cpp \
    langenacht.cpp
FORMS += langenacht.ui
RESOURCES += 
