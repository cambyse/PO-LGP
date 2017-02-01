BASE   = ../..
NAME   = $(shell basename $(BASE))
OUTPUT = ../$(NAME).so
CXXFLAGS += -fPIC
LIBS   += -lrt

#OPTIM  = fast_debug
OPTIM  = debug

CXXFLAGS += -DMLR_ORS_ONLY_BASICS -DMLR_GLOBALMEM -DMLR_NO_THREADS

FREEGLUT = 1
ANN = 1
LAPACK = 1
#NO_BLAS = 1

CXXFLAGS += -DMLR_IMPLEMENT_TEMPLATES #causes all *.cpp to implement the used templates

OBJS  = $(shell find . -maxdepth 1 \( -name '*.cpp' -and -not -name '*_t.cpp' -and -not -name '*_instantiate.cpp' \) )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/build/generic.mk
