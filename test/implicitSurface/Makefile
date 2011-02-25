BASE = $(shell pwd)/../..

OBJS = \
  main.o \
  $(BASE)/include/Lewiner/MarchingCubes.o \
  $(BASE)/include/Lewiner/ply.o

LAPACK = 1
FREEGLUT = 1
CXXFLAGS += -DMT_Lewiner

include $(BASE)/make-generic

