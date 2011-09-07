######################################################################
#
#  File: Makefile
#  Author: A. R. Cassandra
#  July, 1998
#
#  *****
#  Copyright 1994-1997, Brown University
#  Copyright 1998, 1999 Anthony R. Cassandra
#
#                           All Rights Reserved
#                           
#  Permission to use, copy, modify, and distribute this software and its
#  documentation for any purpose other than its incorporation into a
#  commercial product is hereby granted without fee, provided that the
#  above copyright notice appear in all copies and that both that
#  copyright notice and this permission notice appear in supporting
#  documentation.
#  
#  ANTHONY CASSANDRA DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
#  INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR ANY
#  PARTICULAR PURPOSE.  IN NO EVENT SHALL ANTHONY CASSANDRA BE LIABLE FOR
#  ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#  *****
#
#####################################################################
# System specific settings - set these to suit your system's program
# name and flags.
#


# The compiler
CC = g++ -fPIC

# For scanning
LEX = flex

# For parsing
YACC = bison -v #-d

# Removing files
RM = /bin/rm -f

# Archiver name and flags
AR = ar -crvs

######################################################################
# OS specific flags

# Flags specific to the machine architecture.  The CPLEX tech-support
# people tell me that I should use SYSSOLARIS on the Suns.  I am not
# sure it matters.

SYS_FLAGS = 
SYS_LIBS = -lm 

######################################################################
# Compiler - compile time flags

DEBUG_FLAGS = -g
#PROFILE_FLAGS = -pg
#OPTIMIZE_FLAGS = -O3

CFLAGS = $(DEBUG_FLAGS) \
	$(PROFILE_FLAGS) \
	$(OPTIMIZE_FLAGS) \
	$(SYS_FLAGS)


######################################################################
# Compiler load-time flags

LIB_PATH = 

CLDLIBS  = $(SYS_LIBS)

############################################################
LIB_NAME = mdp
LIB_OBJS =  \
	parser.tab.o \
	lex.yy.o \
	imm-reward.o \
	parse_err.o \
	parse_hash.o \
	sparse-matrix.o \
	mdp.o


lib: lib_shared

lib_shared: $(LIB_OBJS)
	gcc -shared -Wl,-soname,libmdp.so -o libmdp.so $(LIB_OBJS) 

libmdp.a: $(LIB_OBJS)
	$(AR) lib$(LIB_NAME).a $(LIB_OBJS)

######################################################################
# Specifies the rules

.SUFFIXES: .o .cpp .y .l .h .c

%.o:	%.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $<

%.o:	%.cpp
	$(CC) $(CFLAGS) $(INCLUDES) -c $<

# There are always some exceptions to the rules ;-)

lex.yy.c:	scanner.l parser.tab.cpp parse_err.h \
		parse_hash.h parse_constant.h
		$(LEX) scanner.l
		cp lex.yy.c1 lex.yy.c


parser.tab.cpp:	parser.ypp  parse_err.h mdp.h \
		parse_hash.h parse_constant.h sparse-matrix.h \
		imm-reward.h
		$(YACC) parser.ypp

######################################################################
#  Specifies the dependencies

# Name of the dependency file
DEPEND = Makefile.depend

dep $(DEPEND):
	$(CC) -E -M $(INCLUDES) *.[c] > $(DEPEND)

# See if a Dependency file exists and if so use it
ifeq ($(DEPEND),$(wildcard $(DEPEND)))
include $(DEPEND)
endif

######################################################################
# Do this to clean up before a fresh make

clean:
	$(RM) *.o parser.tab.cpp 
#libmdp.a libmdp*
#lex.yy.c

######################################################################
