BASE = ../../../

DEPEND = Ors Gui Core Perception

CXX = clang++

OBJS = $(shell find . -maxdepth 1 -name '*.cpp')
OBJS := $(OBJS:%.cpp=%.o)

CLEAN = $(shell find . -maxdepth 1 -name 'z.*')

include $(BASE)/make-generic
