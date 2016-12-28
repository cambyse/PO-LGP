BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

LAPACK = 1
#OPTIM=fast_debug
CXXFLAGS += -DMLR_CORE_PATH=\"`pwd`\"

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
