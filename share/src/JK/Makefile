BASE   = ../..
NAME   = libjk
OUTPUT = $(BASE)/lib/$(NAME).so

MAKEMODE = mlrlib

OBJS  = $(shell find . -maxdepth 2 \( -name '*.cpp' -and -not -name '*_t.cpp' -and -not -name '*_instantiate.cpp' \) )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
