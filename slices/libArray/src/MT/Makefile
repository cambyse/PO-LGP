BASE   = ../..
NAME   = $(shell basename $(BASE))
OUTPUT = $(BASE)/lib/$(NAME).so
CXXFLAGS += -fPIC
LIBS   += -lrt

#OPTIM  = fast_debug
OPTIM  = debug

LAPACK = 1

CXXFLAGS += -DMT_IMPLEMENT_TEMPLATES #causes all *.cpp to implement the used templates

ifndef OBJS
OBJS  = $(shell find . \( -name '*.cpp' -and -not -name '*_t.cpp' -and -not -name '*_instantiate.cpp' \) )
OBJS := $(OBJS:%.cpp=%.o)
endif

include $(BASE)/make-generic
