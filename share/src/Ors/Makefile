BASE   = ../..
NAME   = lib$(shell basename `pwd`)
OUTPUT := $(BASE)/lib/$(NAME).so

SWIFT = 1
GL = 1
PHYSX = 1
DEPENDS = Array Gui Kvg

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
