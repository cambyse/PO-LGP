#BASE = /home/mtoussai/code
BASE = ../../../share

#CXXFLAGS += -DMLR_NO_THREADS

CPATH := $(CPATH):$(shell pwd)/../../../share/src
LPATH := $(LPATH):$(shell pwd)/../../../share/src/NP

# OBJS = gui_moc.o main.o #libMT.o
OBJS = 	main.o \
#	$(BASE)/src/MT/vision_cuda.o


default: x.exe

gui_moc.o: gui_moc.cpp gui_ui.h

FREEGLUT = 1
SWIFT = 1
LAPACK = 1
SCHUNK = 1
PLIB = 1  #for the Gamepad
#BUMBLE = 1
#URGLASER = 1
#QTGLUT = 1
#CUDA = 1
OPENCV = 1
#REVEL = 1

CLEAN = *_moc.cpp *_ui.h MT.log

include $(BASE)/build/generic.mk

