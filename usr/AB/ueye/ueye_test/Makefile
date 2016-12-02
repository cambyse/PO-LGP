BASE=../../../../share

CXX = clang++

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp')
OBJS := $(OBJS:%.cpp=%.o)

CXXFLAGS += -DMLR_IMPLEMENT_TEMPLATES -D__STDC_CONSTANT_MACROS #-march=native
LPATHS += /usr/local/lib
LIBS += -lueye_api

include $(BASE)/build/generic.mk
