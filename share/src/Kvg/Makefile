BASE   = ../..
NAME   = lib$(shell basename `pwd`)
OUTPUT := $(BASE)/lib/$(NAME).so

GTK = 1
GRAPHVIZ = 1

DEPEND = Array

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
