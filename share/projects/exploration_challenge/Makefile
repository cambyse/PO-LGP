BASE = ../..

OBJS = main.o  MLRFactory.o  OpSpaceController.o  pr2ControlSet.o  pr2System.o  qItselfController.o  util.o

EIGEN = 1
HYBRID_AUTOMATON=1
DEPEND = Core RosCom Actions Geo

LIBS += -lhybrid_automaton
CPATH := $(CPATH):$(HOME)/BLA/rswin/hybrid_automaton/include
LPATH := $(LPATH):$(HOME)/BLA/rswin/hybrid_automaton/build

include $(BASE)/build/generic.mk
