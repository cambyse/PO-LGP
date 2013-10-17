BASE   = ../../..
NAME   = libHardware_$(shell basename `pwd`)
OUTPUT := $(BASE)/lib/$(NAME).so

LIBS += -lueye_api
DEPEND = Core

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/make-generic
