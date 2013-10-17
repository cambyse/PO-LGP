BASE   = ../..
NAME   = lib$(shell basename `pwd`)
OUTPUT = $(BASE)/lib/$(NAME).so

GTK = 0
GTKGL = 0
FREEGLUT = 1
QHULL = 1
GRAPHVIZ = 0
DEPEND = Core # extern_ply extern_GJK

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/make-generic
