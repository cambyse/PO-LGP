BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

DEPEND = Core Geo Kin Optim Algo

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
