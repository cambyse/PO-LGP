BASE = ../../../share

GTK = 1

OBJS = main.o graphvizGtk.o

#OBJS = gvplugin_gtk.o
#OUTPUT = libgvplugin_gtk.so

CPATH	:= $(HOME)/tmp/gvd/include:$(CPATH)
LPATH	:= $(HOME)/tmp/gvd/lib:$(LPATH)

CXXFLAGS += -DMT_IMPLEMENT_TEMPLATES

LIBS += -lgraph -lgvc

include $(BASE)/make-generic
