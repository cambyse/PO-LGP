BASE   = ../..
NAME   = libsd
OUTPUT = $(BASE)/lib/$(NAME).so

CXXFLAGS += -fPIC -DMT_IMPLEMENT_TEMPLATES #causes all *.cpp to implement the used templates
#CXXFLAGS += -fPIC 
OBJS  = $(shell find . \( -name '*.cpp' \) )
OBJS := $(OBJS:%.cpp=%.o)
OBJS_C  = $(shell find . -name '*.c')
OBJS += $(OBJS_C:%.c=%.o)

include $(BASE)/make-generic
