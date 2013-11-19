BASE = ../../..
NAME = lib$(shell basename `pwd`)
DEPEND = Motion Core Ors Gui Optim

GL = 1
OBJS = main.o mobject.o pfc.o ../splines/spline.o
LIBS += -lGL
all: $(OBJS)


OUTPUTLIB = $(BASE)/lib/$(NAME).so
SRCSLIB = mobject.cpp pfc.cpp ../splines/spline.cpp 
OBJSLIB = $(SRCSLIB:%.cpp=%.o)
lib: $(OUTPUTLIB) $(SRCSLIB) $(OBJSLIB)


include $(BASE)/make-generic
