BASE   = ..
NAME   = $(shell basename $(BASE))
OUTPUT = $(BASE)/lib/$(NAME).so

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
