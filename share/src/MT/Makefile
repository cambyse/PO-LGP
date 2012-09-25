BASE   = ../..
NAME   = libMT
OUTPUT = $(BASE)/lib/$(NAME).so

MAKEMODE = mlrlib

BUILDS += buildComponent/NJ buildComponent/biros
LIBS += -lNJ -lbiros

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
