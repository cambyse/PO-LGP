BASE = ../../..

PLIB = 1
DEPEND = Algo Core Gui Kin Motion Optim RosCom Hardware_joystick

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

LPATHS += /opt/ros/groovy/lib


include $(BASE)/build/generic.mk
