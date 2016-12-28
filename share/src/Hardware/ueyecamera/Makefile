BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT = libHardware_$(NAME).so

ifeq ($(wildcard /usr/lib/libueye_api.so), )
$(error "libueye_api not installed. Skipping.")
endif

CXXFLAGS += -DUEYE_INSTALLED
LIBS += -lueye_api

DEPEND = Core

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
