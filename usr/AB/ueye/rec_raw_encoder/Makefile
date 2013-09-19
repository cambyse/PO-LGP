BASE=../../../../share

DEPEND = Core # remove
QT = 1

CXX = clang++

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp')
OBJS := $(OBJS:%.cpp=%.o)

CXXFLAGS += -DMT_IMPLEMENT_TEMPLATES -D__STDC_CONSTANT_MACROS
LPATHS += /usr/local/lib
LIBS += -lavformat -lavcodec -lavutil -lswscale -lx264 -lz

include $(BASE)/make-generic

