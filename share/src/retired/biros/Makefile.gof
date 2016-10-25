BASE   = ../..
NAME   = biros
OUTPUT = lib$(NAME).so

MAKEMODE = mlrlib

LIBS += -lviews

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
