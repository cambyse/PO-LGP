BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT := $(BASE)/lib/libextern_$(NAME).a

CFLAGS += -O3 -fPIC -w

OBJS = ply.o 

include $(BASE)/make-generic
