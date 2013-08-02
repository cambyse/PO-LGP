BASE=../../../../share

DEPEND = Core Gui
QT = 1

CXX = clang++

PREOBJS = ./recorder_moc.cpp

#OBJS = main.o recorder.o ueyecamera.o camerathread.o VideoWriter_x264.o
OBJS  = $(shell find . -maxdepth 1 -name '*.cpp')
OBJS := $(filter-out ./mainwindow.cpp,$(OBJS))
OBJS := $(filter-out ./recorder.cpp,$(OBJS))
#OBJS := $(filter-out ./recorder_moc.cpp,$(OBJS))
#OBJS := $(filter-out ./recorder.cpp,$(OBJS))
OBJS := $(OBJS:%.cpp=%.o)

CXXFLAGS += -DMT_IMPLEMENT_TEMPLATES -D__STDC_CONSTANT_MACROS
LPATHS += /usr/local/lib
LIBS += -lueye_api -lavformat -lavcodec -lavutil -lswscale -lx264 -lz

include $(BASE)/make-generic

# to make api-example
# gcc api-example.c -o api-example -lavcodec -lavutil -lx264 -lz
# -L/usr/local/lib -D__STDC_CONSTANT_MACROS
