BASE   = ../..
NAME   = lib$(shell basename `pwd`)
OUTPUT = $(BASE)/lib/$(NAME).so

GTK = 1
GTKGL = 1
QHULL = 1
GRAPHVIZ = 1
DEPEND = Core extern_ply extern_GJK

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/make-generic
