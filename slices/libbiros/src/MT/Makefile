BASE   = $(shell realpath ../..)
NAME   = libmt
OUTPUT = $(BASE)/lib/$(NAME).so

CXXFLAGS += -DMT_ORS_ONLY_BASICS

MAKEMODE = mlrlib

OBJS  = $(shell find . -maxdepth 1 \( -name '*.cpp' -and -not -name '*_t.cpp' -and -not -name '*_instantiate.cpp' \) )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
