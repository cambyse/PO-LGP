BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT = libHardware_$(NAME).so

QT = 1
CXXFLAGS += -D__STDC_CONSTANT_MACROS
LPATHS += /usr/local/lib
LIBS += -lavformat -lavcodec -lavutil -lswscale -lx264 -lz

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
