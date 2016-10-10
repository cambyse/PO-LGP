BASE = ../../../../share
OUTPUT = Editor_ui.h Editor_moc.cpp x.exe

QT = 1
#DEPEND = Core MCTS Gui

CPATHS += ../../src/util

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
