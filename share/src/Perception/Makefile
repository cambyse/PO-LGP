BASE   = ../..
NAME   = lib$(shell basename `pwd`)
OUTPUT := $(BASE)/lib/$(NAME).so

LAPACK = 1
OPENCV = 1
#PCL = 1
DEPEND = Core Gui

CXXFLAGS += -D__STDC_CONSTANT_MACROS
#LPATHS += /usr/local/lib
LIBS += -lavformat -lavcodec -lavutil -lswscale -lx264 -lz

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/make-generic
