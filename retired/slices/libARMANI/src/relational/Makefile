BASE   = ../..
NAME   = $(shell basename $(BASE))
OUTPUT = ../$(NAME).so
CXXFLAGS += -fPIC
LIBS   += -lrt

#OPTIM  = fast_debug
OPTIM  = debug

LAPACK = 0

CXXFLAGS += -DMLR_IMPLEMENT_TEMPLATES #causes all *.cpp to implement the used templates

OBJS  = $(shell find . -maxdepth 1 \( -name '*.cpp' -and -not -name '*_t.cpp' -and -not -name '*_instantiate.cpp' \) )
OBJS += $(shell find ../MT/ -maxdepth 1 \( -name '*.cpp' -and -not -name '*_t.cpp' -and -not -name '*_instantiate.cpp' \) )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/build/generic.mk
