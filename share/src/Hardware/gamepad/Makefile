BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT = libHardware_$(NAME).so

PLIB = 1
DEPEND = Core

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
