BASE   = ../..
NAME   = SD
OUTPUT = lib$(NAME).so

MAKEMODE = mlrlib

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
