BASE   = ../..
NAME   = libperception
OUTPUT = $(BASE)/lib/$(NAME).so

MAKEMODE = mlrlib

#NOBODY MAKES CHANGES HERE ANYMORE OTHER THAN MARC :-)
#if in doubt, you actually want to change make-config!
#otherwise ask Marc

BUILDS += buildComponent/NJ buildComponent/system
LIBS += -lNJ -lsystem

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/make-generic
