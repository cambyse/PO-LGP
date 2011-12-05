BASE = ../../../share

OBJS = main.o $(BASE)/src/MT/vision_cuda.o

LIBS = -lmt
LPATH := $(LPATH):../../src/MT

default: x.exe

gui_moc.o: gui_moc.cpp gui_ui.h

CUDA = 1

CLEAN = *_moc.cpp *_ui.h MT.log

include $(BASE)/make-generic

