BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

GL = 1
FREEGLUT = 1
LAPACK = 1
PTHREAD = 1
QHULL = 1
GJK = 1
LEWINER = 1
PLY = 1
GL = 1
ANN = 1

CXXFLAGS += -DMLR_CORE_PATH=\"`pwd`\" -DMLR_extern_SWIFT
CPATHS += ../Core ../Kin ../Algo ../Geo ../GeoOptim ../Plot ../KOMO ../Gui ../Algo ../Optim 

DEPEND = extern_SWIFT extern_GJK extern_ply extern_Lewiner

OPTIM = debug
CXXFLAGS += -fprofile-arcs -ftest-coverage -O0
LIBS += -lgcov --coverage

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

#default:
#	gcc -E $(CXXFLAGS) all.cpp > z.cpp

include $(BASE)/build/generic.mk

