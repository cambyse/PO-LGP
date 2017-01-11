BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT = libHardware_$(NAME).so

DEPEND = Core Kin Gui Mocap

G4 = 1

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
