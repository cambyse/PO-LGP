BASE   = ../../..
NAME   = libhardware_$(shell basename `pwd`)
OUTPUT := $(BASE)/lib/$(NAME).so

LIBS += -lfreenect
DEPEND = Core

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/make-generic
