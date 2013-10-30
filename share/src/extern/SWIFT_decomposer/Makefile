BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT := $(BASE)/lib/libextern_$(NAME).a

MLR_NO_CXX11 = 1

CXXFLAGS += -DSWIFT_DECOMP -DDECOMP_GRAPHICS -O3 -fPIC -w -IRAPID -I/usr/include/tcl8.5

LIBS += -lqhull -lm -ltcl -ltk -lglut -lGLU -lXmu -lX11 -lGL

SRCS = $(shell find . -maxdepth 2 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

target: $(OUTPUT) ./decomposer.exe
./decomposer.exe: main.o
./decomposer.exe: OBJS+=main.o

DEPEND = extern_SWIFT

include $(BASE)/make-generic
