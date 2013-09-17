BASE=../../../../share

DEPEND = Core Gui
QT = 1

CXX = clang++

PREOBJS = ./recorder_moc.cpp ./ueyecamera_moc.cpp

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp')
OBJS := $(filter-out $(PREOBJS),$(OBJS))
OBJS := $(OBJS:%.cpp=%.o)

CXXFLAGS += -DMT_IMPLEMENT_TEMPLATES -D__STDC_CONSTANT_MACROS
LPATHS += /usr/local/lib
LIBS += -lueye_api -lavformat -lavcodec -lavutil -lswscale -lx264 -lz

include $(BASE)/make-generic

