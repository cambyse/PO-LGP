BASE   = ../..
NAME   = libmt
OUTPUT = $(BASE)/lib/$(NAME).so
CXXFLAGS += -fPIC
LIBS   += -lrt

#OPTIM  = fast_debug
OPTIM  = debug

FREEGLUT = 1
#OPENCV = 1
ANN = 1
QHULL = 1
SWIFT = 1
LAPACK = 1
ODE = 1

TONY = 1
OPENCV_UBUNTU = 1
SCHUNK = 1
PLIB = 1
BUMBLE = 1
#URGLASER = 1
CUDA = 1
LEWINER = 1
NIKOLAY = 1
NILS = 1
#STANIMIR = 1

#GL2PS = 1

CXXFLAGS += -DMT_IMPLEMENT_TEMPLATES #causes all *.cpp to implement the used templates
CXXFLAGS += -DMT_Lewiner
OBJS  = $(shell find . -maxdepth 1 \( -name '*.cpp' -and -not -name '*_t.cpp' -and -not -name '*_instantiate.cpp' \) )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
