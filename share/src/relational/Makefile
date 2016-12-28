BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

-include $(BASE)/build/config.mk
ifeq ($(PURE_RELATIONAL),1)
DEPEND = Core
else
DEPEND = Core Ors Gui Motion
endif

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
