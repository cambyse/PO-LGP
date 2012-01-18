BASE   = ../..
NAME   = libmt
OUTPUT = $(BASE)/lib/$(NAME).so

MAKEMODE = mlrlib

#NOBODY MAKES CHANGES HERE ANYMORE OTHER THAN MARC :-)
#if in doubt, you actually want to change make-config!
#otherwise ask Marc

NILS = 1
NIKOLAY = 1
DMITRY = 1
STANIMIR = 1

OBJS  = $(shell find . -maxdepth 1 \( -name '*.cpp' -and -not -name '*_t.cpp' -and -not -name '*_instantiate.cpp' \) )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
