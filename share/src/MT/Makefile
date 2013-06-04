BASE   = ../..
NAME   = lib$(shell basename `pwd`)
OUTPUT = $(BASE)/lib/$(NAME).so

GTK = 1
GTKGL = 1
QHULL = 1
GRAPHVIZ = 1
DEPEND = Core Ors Gui Optim extern_ply

OBJS  = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS := $(OBJS:%.cpp=%.o)

#OPTIM=fast

include $(BASE)/make-generic
