BASE   = ../..
NAME   = lib$(shell basename `pwd`)
OUTPUT := $(BASE)/lib/$(NAME).so

GTK = 1
GTKGL = 1
QHULL = 1
PLY = 1
DEPENDS = Array Kvg

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
