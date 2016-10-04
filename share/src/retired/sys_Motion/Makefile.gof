BASE   = ../..
NAME   = $(shell basename $(PWD))
OUTPUT = lib$(NAME).so

DEPEND = Array

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
