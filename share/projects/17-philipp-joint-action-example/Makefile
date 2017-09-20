BASE = ../..

PLIB = 1
ROS = 1
PCL = 1

LIBS += -lopencv_imgproc -lopencv_core
DEPEND = Core RosCom Control Gui Hardware_gamepad Kin Motion Geo Roopi Perception PCL

OBJS = main.o OrderingTask.o ../17-philipp-HRI-task/HRI_task.o ../17-philipp-HRI-task/HRI_state.o
#OBJS = main.o HRI_task.o HRI_state.o OrderingTask.o
include $(BASE)/build/generic.mk
