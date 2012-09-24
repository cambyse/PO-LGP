# Symbol grounding Makefile
# Johannes, 09 Dec 2011
#

BASE = ../../../share

#LIBS += -lvision
CUDA = 0
OBJS  = $(shell find . -maxdepth 1 \( -name '*.cpp' -and -not -name '*_t.cpp' -and -not -name '*_instantiate.cpp' \) )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
