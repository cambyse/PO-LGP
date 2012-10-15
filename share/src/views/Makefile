BASE   = ../..
NAME   = libviews
OUTPUT = $(BASE)/lib/$(NAME).so

MAKEMODE = mlrlib

#BUILDS += buildComponent/biros
#LIBS += -lbiros

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
