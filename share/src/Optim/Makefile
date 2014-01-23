BASE   = ../..
NAME   = lib$(shell basename `pwd`)
OUTPUT := $(BASE)/lib/$(NAME).so

DEPEND = Core

SRCS = $(shell find . -maxdepth 1 -name '*.cpp') cmaes.c
OBJS = $(SRCS:%.cpp=%.o) cmaes.o

include $(BASE)/make-generic
