BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT := $(BASE)/lib/libextern_$(NAME).a

CFLAGS += -O3 -fPIC

OBJS  = $(shell find . -maxdepth 1 -name '*.c' )
OBJS := $(OBJS:%.c=%.o)

include $(BASE)/make-generic
