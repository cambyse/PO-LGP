BASE   = ../..
NAME   = libbiros
OUTPUT = $(BASE)/lib/$(NAME).so

MAKEMODE = mlrlib

LIBS += -lviews

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
