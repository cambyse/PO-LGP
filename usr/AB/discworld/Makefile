BASE = ../../../share/

DEPEND = Geo Kin Gui Core Perception
PHYSX = 1

CXX = clang++

OBJS = $(shell find . -maxdepth 1 -name '*.cpp')
OBJS := $(OBJS:%.cpp=%.o)

CLEAN = $(shell find . -maxdepth 1 -name 'z.*')

include $(BASE)/build/generic.mk
