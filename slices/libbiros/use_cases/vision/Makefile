BASE = $(shell realpath ../..)

OBJS = main.o processes.o extern/MT/libcolorseg.o

OPENCV_UBUNTU = 1
FELZ = 1

CXXFLAGS += -DMT_IMPLEMENT_TEMPLATES -Iextern -Iextern/libcolorseg/include
LPATH := $(LPATH):extern/libcolorseg/lib

include $(BASE)/make-generic
