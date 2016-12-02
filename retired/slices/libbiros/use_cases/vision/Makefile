BASE = $(shell realpath ../..)

OBJS = main.o processes.o extern/MT/libcolorseg.o

OPENCV = 1
FELZ = 1

CXXFLAGS += -DMLR_IMPLEMENT_TEMPLATES -Iextern -Iextern/libcolorseg/include
LPATH := $(LPATH):extern/libcolorseg/lib

include $(BASE)/build/generic.mk
