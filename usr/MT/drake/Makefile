BASE = ../../../share

DEPEND = Core Algo Kin KOMO Drake

MLR_NO_CXX11 = 1
CXXFLAGS += -DBT_USE_DOUBLE_PRECISION -DHAVE_SPDLOG -DSPDLOG_FMT_EXTERNAL -std=gnu++14

DRAKE := /opt/drake
#DRAKE := $(HOME)/git/drake-distro/drake
CPATH := $(CPATH):$(DRAKE)/include:$(DRAKE)/include/drake/lcmtypes:$(DRAKE)/include/lcmtypes:$(DRAKE)/include/bullet:$(DRAKE)/include/eigen3:$(DRAKE)/include/nlopt
LPATHS += $(DRAKE)/lib

LIBS += -ldrake -lBulletCollision -lLinearMath -lfcl -lccd -loctomap -llcm -lglib-2.0 -lpthread -lnlopt -lprotobuf -lsdformat -lignition_math -ltinyxml -lfmt -lyaml_cpp -lz

include $(BASE)/build/generic.mk
