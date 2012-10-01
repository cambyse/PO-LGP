BASE   = ../..
NAME   = librelational
OUTPUT = $(BASE)/lib/$(NAME).so

MAKEMODE = mlrlib

BUILDS += buildComponent/MT
LIBS += -lMT

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
