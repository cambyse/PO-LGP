BASE   = ../..
NAME   = lib$(shell basename $(PWD))
OUTPUT := $(BASE)/lib/$(NAME).so

DEPEND = Array

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/make-generic
