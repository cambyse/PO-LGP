BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT := $(BASE)/lib/libextern_$(NAME).a

CFLAGS += -O3 -fPIC

SRCS = $(shell find . -maxdepth 1 -name '*.c' )
OBJS = $(SRCS:%.c=%.o)

include $(BASE)/make-generic
