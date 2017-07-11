BASE = ../../..

PLIB = 1

OBJS = main.o pd_executor_module.o
DEPEND = Core Mocap System Hardware_gamepad Hardware_G4 KOMO Kin Perception

# comment/uncomment to disable/enable ROS
ROS = 1
# CXXFLAGS = -DWITH_ROS
DEPEND += RosCom Actions

include $(BASE)/build/generic.mk
