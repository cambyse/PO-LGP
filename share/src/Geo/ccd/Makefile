BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT = libextern_$(NAME).a

CFLAGS += -std=gnu99 -O3 -fPIC -w -I.. 

SRCS = $(shell find . -maxdepth 1 -name '*.c' )
OBJS = $(SRCS:%.c=%.o)

include $(BASE)/build/generic.mk
