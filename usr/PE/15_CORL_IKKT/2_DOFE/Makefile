BASE = ../../../../share


OBJS = main.o src/articulation_interface.o ../../src/traj_factory.o ../../src/task_grasp.o ../../src/task_door.o ../../src/task_button.o ../../src/plotUtil.o
PLIB = 1
ROS = 1
DEPEND = Kin Geo Optim Core Hardware_gamepad KOMO Gui pr2

CPATHS += $(BASE)/../../../rosbuild_ws/sandbox/articulation/articulation_models/include/
CPATHS += $(BASE)/../../../rosbuild_ws/sandbox/articulation/articulation_structure/src/
CPATHS += $(BASE)/../../../rosbuild_ws/sandbox/articulation/articulation_msgs/msg_gen/cpp/include/
CPATHS += $(BASE)/../../../rosbuild_ws/sandbox/articulation/articulation_msgs/srv_gen/cpp/include/

CPATHS += $(BASE)/../../../rosbuild_ws/sandbox/articulation/articulation_structure/src/


LPATHS += $(BASE)/../../../rosbuild_ws/sandbox/articulation/articulation_models/lib/

LIBS += -larticulation_models

include $(BASE)/build/generic.mk
