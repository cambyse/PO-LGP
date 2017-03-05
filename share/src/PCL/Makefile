BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

CXXFLAGS += -O1 #unbelievable!!! without this there is a seg fault in boost
PCL = 1
OPENCV = 1
DEPEND = Core Geo Gui

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
