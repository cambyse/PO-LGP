BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT := $(BASE)/lib/libextern_$(NAME).a

CXXFLAGS += -O3 -fPIC -I../ply -w

DEPEND = extern_ply

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS := $(OBJS:%.cpp=%.o)

include $(BASE)/make-generic
