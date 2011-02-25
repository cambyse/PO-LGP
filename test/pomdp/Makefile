BASE = $(shell pwd)/../..

OBJS = main.o

LAPACK = 1
TONY = 1
QT = 1   ## comment this line if you don't want to compile the GUI

ifdef QT
OBJS    += gui_moc.o
PREOBJS += gui_ui.h gui_moc.cpp
endif

CLEAN = *_moc.cpp *_ui.h MT.log

include $(BASE)/make-generic

