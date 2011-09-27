#BASE = /home/mtoussai/code
BASE = $(shell pwd)/../../../share

# OBJS = gui_moc.o main.o #libMT.o
OBJS = main.o 
#	libMT.o

default: x.exe

# default: libNILS x.exe

# libNILS::
# 	make -C $(BASE)/src/NP_2Drec

#OPTIM=fast

gui_moc.o: gui_moc.cpp gui_ui.h

#CXXFLAGS += -DMT_QTGLUT

FREEGLUT = 1
SWIFT = 1
LAPACK = 1
#QT = 1
SCHUNK = 1
PLIB = 1  #for the Joystick
#URGLASER = 1
#QTGLUT = 1
#NILS = 1
OPENCV = 1
#REVEL = 1

CLEAN = *_moc.cpp *_ui.h MT.log

include $(BASE)/make-generic

