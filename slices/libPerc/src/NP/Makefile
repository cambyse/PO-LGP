BASE   = ../..
NAME   = libnp
OUTPUT = $(BASE)/lib/$(NAME).so
CXXFLAGS += -fPIC

#OPTIM  = fast_debug
OPTIM  = debug

CXXFLAGS += -O3 -Wall -DNP_DC1394 
CXXFLAGS += -DMT_Linux -DMT_IMPLEMENT_TEMPLATES -DMT_BUMBLE -DMT_NO_GLOBALMEM
CXXFLAGS += -DNP_IMPLEMENT_TEMPLATES

ifndef OBJS
OBJS  = $(shell find . \( -name '*.cpp' -and -not -name '*_t.cpp' -and -not -name '*_instantiate.cpp' \) )
OBJS := $(OBJS:%.cpp=%.o)
endif

include $(BASE)/make-generic
