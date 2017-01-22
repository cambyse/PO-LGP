BASE = ../../../../share
OBJS = main.o InnerCostFunction.o OuterCostFunction.o Demonstration.o TrajectoryFactory.o Simulator.o

# InnerCostFunction.o : InnerCostFunction.cpp InnerCostFunction.h
#	g++ -DMLR_Core -DMLR_Motion -DMLR_Gui -DMLR_Optim -DMLR_Algo -DMLR_Kin -g -Wall -fPIC -std=c++0x -DMLR_PTHREAD -c InnerCostFunction.cpp

DEPEND = Core Motion Gui Optim Algo Kin
PTHREAD = 1
include $(BASE)/build/generic.mk
