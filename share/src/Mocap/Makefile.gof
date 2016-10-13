BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

GL = 0
PHYSX = 0
ODE = 0
DEPEND = Core Perception extern_json
CXXFLAGS += -Wno-unknown-pragmas

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
