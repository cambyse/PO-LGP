BASE   = ../..
NAME   = libdz
OUTPUT = $(BASE)/lib/$(NAME).so

MAKEMODE = mlrlib

OBJS  = $(shell find . -maxdepth 1 \( -name '*.cpp' -and -not -name '*_t.cpp' -and -not -name '*_instantiate.cpp' \) )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
