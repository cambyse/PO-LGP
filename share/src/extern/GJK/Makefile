BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT := $(BASE)/lib/libextern_$(NAME).a

CFLAGS += -fPIC -w -g
#CFLAGS += 0O3 -fPIC -w

OBJS = gjk.o

include $(BASE)/make-generic
