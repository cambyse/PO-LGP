BASE = $(shell pwd)/../..

OPTIM = debug

CXXFLAGS += -DMT_Linux -DMT_IMPLEMENT_TEMPLATES -DMT_FREEGLUT
LDFLAGS += -lglut

default: libNILS x.exe

libNILS::
	make -C $(BASE)/src/NP_2Drec

OBJS = main.o 
#   libMT.o
#   $(BASE)/src/common.o \
#   $(BASE)/src/image.o \
#   $(BASE)/src/vision_low/projective_transformation.o \
#   $(BASE)/src/vision_low/correspondance.o \
#   $(BASE)/src/vision_low/tracking.o

OBJ := $(OBJS)

FREEGLUT = 1
OPENCV = 1
ANN = 1
NILS = 1
SURF = 1
LAPACK = 1
QHULL = 1
#ESS = 1

include $(BASE)/make-generic
