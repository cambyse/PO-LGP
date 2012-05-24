BASE   = ../..
NAME   = libDZ
OUTPUT = $(BASE)/lib/$(NAME).so

MAKEMODE = mlrlib

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
