BASE   = ../..
NAME   = librelational
OUTPUT = $(BASE)/lib/$(NAME).so

MAKEMODE = mlrlib

BUILDS += buildComponent/MT
LIBS += -lMT

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/make-generic
