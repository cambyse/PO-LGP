BASE = ../..

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

PLIB = 1
ROS = 1
PCL = 1
GL = 1
DEPEND = Core RosCom Control Gui Hardware_gamepad Kin KOMO Geo Roopi Perception PCL Algo Drake

DRAKE := /opt/drake
CPATH := $(CPATH):$(DRAKE)/include:$(DRAKE)/include/drake/lcmtypes:$(DRAKE)/include/lcmtypes:$(DRAKE)/include/bullet:$(DRAKE)/include/eigen3:$(DRAKE)/include/nlopt
LPATHS += $(DRAKE)/lib

LIBS += -ldrake

include $(BASE)/build/generic.mk
