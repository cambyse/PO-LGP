NAME   = lewiner
BASE   = ../..

OUTPUT = lib$(NAME).so

MAKEMODE = externlib

CXXFLAGS += -fPIC

LEWINER=0
PCL=0

#CXXFLAGS += -DMT_IMPLEMENT_TEMPLATES #causes all *.cpp to implement the used templates
OBJS  = $(shell find . -maxdepth 1 -name '*.c*')
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
