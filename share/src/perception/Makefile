BASE   = ../..
NAME   = libperception
OUTPUT = $(BASE)/lib/$(NAME).so

MAKEMODE = mlrlib

#NOBODY MAKES CHANGES HERE ANYMORE OTHER THAN MARC :-)
#if in doubt, you actually want to change make-config!
#otherwise ask Marc

BUILDS += buildComponent/NJ buildComponent/DZ buildComponent/SD buildComponent/biros
LIBS += -lNJ -lDZ -lSD -lbiros

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
