NAME   = lewiner
BASE   = ../..

OUTPUT = lib$(NAME).so

MAKEMODE = externlib

CXXFLAGS += -fPIC

#CXXFLAGS += -DMT_IMPLEMENT_TEMPLATES #causes all *.cpp to implement the used templates
OBJS  = $(shell find . \( -name '*.cpp' -and -not -name '*_t.cpp' -and -not -name '*_instantiate.cpp' \) )
OBJS := $(OBJS:%.cpp=%.o)
OBJS_C  = $(shell find . -name '*.c')
OBJS += $(OBJS_C:%.c=%.o)

include $(BASE)/make-generic
