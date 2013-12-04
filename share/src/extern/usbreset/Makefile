BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT := $(BASE)/bin/$(NAME).exe

SRCS = $(shell find . -maxdepth 1 -name '*.c' )
OBJS = $(SRCS:%.c=%.o)

include $(BASE)/make-generic
