BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT := $(BASE)/lib/libextern_$(NAME).a

CXXFLAGS += -DSWIFT_DECOMP -DDECOMP_GRAPHICS -O3 -fPIC -w -IRAPID -I/usr/include/tcl8.5

LIBS = -lqhull -lm -ltcl -ltk -lglut -lGLU -lXmu -lX11 -lGL

OBJS  = $(shell find . -maxdepth 2 -name '*.cpp' )
OBJS := $(OBJS:%.cpp=%.o)

target: $(OUTPUT) ./decomposer.exe
./decomposer.exe: main.o
./decomposer.exe: OBJS+=main.o

DEPEND = extern_SWIFT

include $(BASE)/make-generic