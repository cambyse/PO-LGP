BASE = ../../../../share

#OBJS = main.o src/mb_strategy.o ../../src/traj_factory.o ../../src/task_grasp.o ../../src/task_door.o ../../src/task_button.o ../../src/plotUtil.o

OBJS = main.o src/conbopt.o ../../src/traj_factory.o ../../src/task_grasp.o ../../src/task_door.o ../../src/task_button.o ../../src/plotUtil.o
DEPEND = Core Kin Gui Algo KOMO Optim pr2 Geo
PTHREAD = 0
PHYSX = 0

CPATHS += /usr/local/MATLAB/R2016a/extern/include
LPATHS += /usr/local/MATLAB/R2016a/bin/glnxa64
LIBS += -lmat -leng -lmx -lut -lmwi18n -licuuc -licuio -licuuc -licudata  -licui18n -lz -lmwcpp11compat -lmwresource_core  -lmwfl -lboost_signals -lboost_system -lboost_date_time -lboost_thread -licudata -licuuc -licui18n -licuio -lboost_log -lboost_log_setup -lboost_regex -lboost_chrono -lboost_filesystem


include $(BASE)/build/generic.mk
