BASE   = ../..
NAME   = lib$(shell basename `pwd`)
OUTPUT := $(BASE)/lib/$(NAME).a

CXXFLAGS += -DSWIFT_DECOMP -DDECOMP_GRAPHICS -O3 -fPIC -Wno-deprecated -Wno-write-strings -IRAPID -I../include -I../../include -I/usr/include/tcl8.5

LIBS = -lqhull -lm -ltcl -ltk -lglut -lGLU -lXmu -lX11 -lGL

OBJS  = $(shell find . -maxdepth 2 -name '*.cpp' )
OBJS := $(OBJS:%.cpp=%.o)

target: $(OUTPUT) ./decomposer.exe
./decomposer.exe: main.o
./decomposer.exe: OBJS+=main.o

DEPEND = extern_SWIFT

include $(BASE)/make-generic