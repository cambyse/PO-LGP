BASE=../../../../share

DEPEND = Core Gui
QT = 1

CXX = clang++

PREOBJS = ./recorder_moc.cpp ./ueyecamera_moc.cpp ./recworker_moc.cpp

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp')
OBJS := $(filter-out $(PREOBJS),$(OBJS))
OBJS := $(OBJS:%.cpp=%.o)

CXXFLAGS += -DMLR_IMPLEMENT_TEMPLATES -D__STDC_CONSTANT_MACROS #-march=native
LPATHS += /usr/local/lib
LIBS += -lueye_api -lavformat -lavcodec -lavutil -lswscale -lx264 -lz

include $(BASE)/build/generic.mk

# to make api-example
# gcc api-example.c -o api-example -lavcodec -lavutil -lx264 -lz
# -L/usr/local/lib -D__STDC_CONSTANT_MACROS
