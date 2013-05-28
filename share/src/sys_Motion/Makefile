BASE   = ../..
NAME   = lib$(shell basename $(PWD))
OUTPUT := $(BASE)/lib/$(NAME).so

DEPEND = Array

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
