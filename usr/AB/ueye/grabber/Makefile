BASE=../../../../share

DEPEND = Core Gui
QT = 1

OBJS = main.o VideoWriter_x264.o

CXXFLAGS += -DMT_IMPLEMENT_TEMPLATES -D__STDC_CONSTANT_MACROS
LIBS += -lueye_api -lavcodec -lavformat -lavutil -lswscale -lx264

include $(BASE)/make-generic
