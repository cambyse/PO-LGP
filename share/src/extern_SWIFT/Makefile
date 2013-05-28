BASE   = ../..
NAME   = lib$(shell basename `pwd`)
OUTPUT := $(BASE)/lib/$(NAME).a

CXXFLAGS += -O3 -fPIC -I../include -DNO_CYCLE_DETECTION -DSWIFT_DECOMP -Wall -Wno-deprecated

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
