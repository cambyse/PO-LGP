BASE   = ../..
NAME   = libviews
OUTPUT = $(BASE)/lib/$(NAME).so

MAKEMODE = mlrlib

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/make-generic
