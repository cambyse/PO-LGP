BASE = ../..

PLIB = 1
ROS = 1
PCL = 1

LIBS += -lopencv_imgproc -lopencv_core
DEPEND = Core RosCom Control Gui Hardware_gamepad Kin Motion Geo Roopi Perception PCL

OBJS = main.o HRI_task.o HRI_state.o BridgeTask.o StackTask.o SortTask.o BalanceTask.o
include $(BASE)/build/generic.mk
