BASE   = ../..
NAME   = librelational
OUTPUT = $(BASE)/lib/$(NAME).so

MAKEMODE = mlrlib

BUILDS += buildComponent/NJ buildComponent/DZ buildComponent/SD buildComponent/biros
LIBS += -lMT

OBJS  = $(shell find . -maxdepth 1 \( -name '*.cpp' -and -not -name '*_t.cpp' -and -not -name '*_instantiate.cpp' \) )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
