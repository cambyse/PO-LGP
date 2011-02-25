BASE = $(shell pwd)/../..
#LIBS = ../../lib/

CXXFLAGS += -DMT_Linux -DMT_IMPLEMENT_TEMPLATES -DMT_NOCHECK -I$(BASE)/include/NP

LIBVOC2009 = 1
OPENCV = 1
ANN = 1
FREEGLUT = 1
OBJS = \
   libMT.o \
   main.o

include $(BASE)/make-generic
