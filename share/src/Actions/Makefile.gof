BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so swig_wrap.cxx _swig.so

ROS = 1

DEPEND = RosCom Motion Ors Core Control FOL Hardware Perception Hardware_gamepad Gui Media

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
