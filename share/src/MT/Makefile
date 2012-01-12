BASE   = ../..
NAME   = libmt
OUTPUT = $(BASE)/lib/$(NAME).so

MAKEMODE = mlrlib

NILS = 1
NIKOLAY = 1
CUDA = 1

OBJS  = $(shell find . -maxdepth 1 \( -name '*.cpp' -and -not -name '*_t.cpp' -and -not -name '*_instantiate.cpp' \) )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
