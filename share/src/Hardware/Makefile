BASE   = ../..
NAME   = lib$(shell basename `pwd`)
OUTPUT := $(BASE)/lib/$(NAME).so

FREENECT = 1
PLIB = 1
DEPEND = Core

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/make-generic

# BASE   = ../..
# NAME   = libhardware
# OUTPUT = $(BASE)/lib/$(NAME).so

# MAKEMODE = mlrlib

# #NOBODY MAKES CHANGES HERE ANYMORE OTHER THAN MARC :-)
# #if in doubt, you actually want to change make-config!
# #otherwise ask Marc

# buildAndLink_MT = 1
# buildAndLink_NIKOLAY = 1

# SRCS = $(shell find . -maxdepth 1 \( -name '*.cpp' -and -not -name '*_t.cpp' -and -not -name '*_instantiate.cpp' \) )
# OBJS = $(SRCS:%.cpp=%.o)

# include $(BASE)/make-generic
