BASE = ../..


OBJS = main.o
PLIB = 1
ROS = 1
DEPEND = Geo Optim Core Hardware_gamepad KOMO Kin Gui RosCom

CPATHS += $(BASE)/../../../rosbuild_ws/sandbox/articulation/articulation_models/include/
CPATHS += $(BASE)/../../../rosbuild_ws/sandbox/articulation/articulation_msgs/msg_gen/cpp/include/
CPATHS += $(BASE)/../../../rosbuild_ws/sandbox/articulation/articulation_msgs/srv_gen/cpp/include/
CPATHS += $(BASE)/../../../rosbuild_ws/sandbox/articulation/articulation_structure/src/

LPATHS += $(BASE)/../../../rosbuild_ws/sandbox/articulation/articulation_models/lib/
LIBS += -larticulation_models


include $(BASE)/build/generic.mk
