BASE = ../../../../../share

OBJS = main.o ../src/mf_strategy.o ../../src/traj_factory.o  ../src/task_grasp.o ../src/task_door.o ../../src/plotUtil.o ../../src/phase_optimization.o
DEPEND = Core Kin Gui Algo KOMO Optim pr2 Hardware_gamepad Actions Geo
PTHREAD = 0
PHYSX = 0

CPATHS += /usr/local/MATLAB/R2015a/extern/include
LPATHS += /usr/local/MATLAB/R2015a/bin/glnxa64
LIBS += -lmat -leng -lmx -lut -lmwcpp11compat -lmwresource_core -lmwi18n -lmwfl -lmwMATLAB_res -lboost_signals -lboost_system -lboost_date_time -lboost_thread -licudata -licuuc -licui18n -licuio -lboost_log -lboost_log_setup -lboost_regex -lboost_chrono -lboost_filesystem

include $(BASE)/build/generic.mk
