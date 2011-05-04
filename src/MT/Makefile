NAME   = mt
BASE   = ../..
OUTPUT = $(BASE)/lib/lib$(NAME).so
CXXFLAGS += -fPIC
LIBS   += -lrt

#OPTIM  = fast_debug
OPTIM  = debug

##AICO:
#OBJS = 	aico.o soc.o soc_system_ors.o soc_method_AICO.o\
#	util.o array.o opengl.o plot.o \
#	ors.o ors_taskVariables.o ors_swift.o ors_opengl.o

ANN = 1
#TONY = 1
OPENCV = 1
SCHUNK = 1
#ODE = 1
QHULL = 1
FREEGLUT = 1
SWIFT = 1
LAPACK = 1
PLIB = 1
BUMBLE = 1
#URGLASER = 1
CUDA = 1
#REVEL = 1
LEWINER = 1
NIKOLAY = 1
TOBIAS = 1
NILS = 1

CXXFLAGS += -DMT_IMPLEMENT_TEMPLATES #causes all *.cpp to implement the used templates

ifndef OBJS
OBJS  = $(shell find . \( -name '*.cpp' -and -not -name '*_t.cpp' -and -not -name '*_instantiate.cpp' \) )
OBJS := $(OBJS:%.cpp=%.o)
endif

include $(BASE)/make-generic
