BASE = $(shell pwd)/../..

OBJS = main.o \
	$(BASE)/include/Lewiner/MarchingCubes.o \
	$(BASE)/include/Lewiner/ply.o

default: x.exe

CXXFLAGS += -DMLR_NO_GLOBALMEM
##OPTIM = fast

FREEGLUT = 1
SWIFT = 1
LAPACK = 1
SCHUNK = 1
PLIB = 1  #for the Gamepad
VISION = 1
OPENCV = 1
NILS = 1
#BUMBLE = 1
#CUDA = 1

CXXFLAGS += -DMLR_Lewiner

CLEAN = *_moc.cpp *_ui.h MT.log

include $(BASE)/build/generic.mk

